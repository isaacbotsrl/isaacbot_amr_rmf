#!/usr/bin/env python3
# Copyright 2026 IsaacBot
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Who is on the fleet's network, for the robots page's Network section.

The broker is the one place every isaacbot connects to, so its log says where each one is: the
address it connects from (its VPN address on the server), its MQTT client id, and since when it is
connected or when it was last seen. On the server deployment Mosquitto also logs to
<broker-auth>/broker.log (mosquitto.server.conf); mosquitto-run.sh rotates it to broker.log.1.

Everything here but probe() is pure, so it is unit-tested on its own.
"""

import os
import re
import socket
import threading
from dataclasses import asdict, dataclass, replace
from typing import Dict, Iterable, List, Optional

# Mosquitto 2 log lines (default epoch timestamps).
CONNECTED = re.compile(r'(\d+): New client connected from (\S+):(\d+) as (\S+) \(([^)]*)\)')
LOGIN = re.compile(r"u'([^']*)'")
DISCONNECTED = re.compile(
    r'(\d+): Client (\S+) (?:\[[^\]]*\] )?'
    r'(?:disconnected|closed its connection|has exceeded timeout)')
BROKER_START = re.compile(r'(\d+): mosquitto version \S+ (?:starting|terminating)')

# The network inside every isaacbot (the same on each today), from
# isaacbot_amr_networking/rutx11/README.md ("The router"), shown on each robot's page. Operators
# use it on site, with a laptop on the isaacbot's Wi-Fi or its wired LAN.
ISAACBOT_LAN = [
    {'device': 'Router (Teltonika RUTX11)', 'network': 'lan + Wi-Fi AP', 'ip': '192.168.1.1',
     'note': 'web UI https://192.168.1.1, ssh root@192.168.1.1; also .1 on 192.168.77.0/24'},
    {'device': 'ROS controller', 'network': 'lan (wired)', 'ip': '192.168.1.201',
     'note': 'also .102 (same MAC, the lidar sends to it); orchestrator sshd on port 24'},
    {'device': 'Network page (uplink manager)', 'network': 'lan (wired)',
     'ip': '192.168.1.201:5080', 'note': "switches the router's Wi-Fi uplink safely"},
    {'device': 'RS16 lidar', 'network': 'lan (wired)', 'ip': '192.168.1.200',
     'note': "never in the router's ARP table; that is normal"},
    {'device': 'BMS', 'network': 'Wi-Fi AP ISAACBOT-01-2.4GHz', 'ip': '192.168.77.201',
     'note': ''},
    {'device': 'Front LED', 'network': 'Wi-Fi AP ISAACBOT-01-2.4GHz', 'ip': '192.168.77.202',
     'note': ''},
    {'device': 'Laptops', 'network': 'lan / Wi-Fi AP', 'ip': 'DHCP',
     'note': 'lan .110–.249, AP .100–.149 (never hand out 192.168.1.102)'},
    {'device': 'GPS NMEA', 'network': 'router → controller', 'ip': '192.168.1.201:10110',
     'note': 'UDP'},
]


@dataclass(frozen=True)
class Session:
    client_id: str
    login: str             # '' when the client gave no user name
    ip: str
    port: int
    online: bool
    since: float           # when it connected
    last_seen: float       # when it disconnected (since while online)


class BrokerSessions:
    """The broker's client sessions, fed one log line at a time (latest per client id)."""

    def __init__(self):
        self.sessions: Dict[str, Session] = {}

    def feed(self, line: str) -> None:
        m = CONNECTED.match(line)
        if m:
            t, ip, port, client_id, flags = m.groups()
            login = LOGIN.search(flags)
            self.sessions[client_id] = Session(
                client_id=client_id, login=login.group(1) if login else '',
                ip=ip.strip('[]'), port=int(port), online=True, since=float(t),
                last_seen=float(t))
            return
        m = DISCONNECTED.match(line)
        if m:
            t, client_id = m.groups()
            old = self.sessions.get(client_id)
            if old and old.online:
                self.sessions[client_id] = replace(old, online=False, last_seen=float(t))
            return
        m = BROKER_START.match(line)
        if m:
            t = float(m.group(1))
            for client_id, old in self.sessions.items():
                if old.online:
                    self.sessions[client_id] = replace(old, online=False, last_seen=t)

    def feed_all(self, lines: Iterable[str]) -> 'BrokerSessions':
        for line in lines:
            self.feed(line)
        return self

    def for_login(self, login: str) -> Optional[Session]:
        """The login's current session: an online one, else the one seen last."""
        mine = [s for s in self.sessions.values() if s.login == login]
        return max(mine, key=lambda s: (s.online, s.last_seen), default=None)

    def others(self, logins: Iterable[str]) -> List[Session]:
        """Sessions of every other login (the fleet adapter, an old shared login, ...)."""
        skip = set(logins)
        return sorted((s for s in self.sessions.values() if s.login not in skip),
                      key=lambda s: (not s.online, s.login, s.client_id))


class BrokerLog:
    """
    The broker's log file and its rotated predecessor, read incrementally.

    Each read() continues where the last one stopped. When the file was rotated or truncated
    (new inode, or shorter than what was read), it starts over from <path>.1 and <path>.
    """

    def __init__(self, path: str):
        self.path = path
        self._inode = None
        self._offset = 0
        self._partial = ''
        self._lock = threading.Lock()   # the page may poll from several requests at once
        self.sessions = BrokerSessions()

    def available(self) -> bool:
        return os.path.isfile(self.path)

    def read(self) -> BrokerSessions:
        with self._lock:
            return self._read()

    def _read(self) -> BrokerSessions:
        try:
            st = os.stat(self.path)
        except OSError:
            return self.sessions
        if st.st_ino != self._inode or st.st_size < self._offset:
            self.sessions = BrokerSessions()
            self._partial = ''
            self._offset = 0
            self._inode = st.st_ino
            self._feed_file(self.path + '.1')
        self._offset = self._feed_file(self.path, self._offset)
        return self.sessions

    def _feed_file(self, path: str, offset: int = 0) -> int:
        try:
            with open(path, 'rb') as f:
                f.seek(offset)
                data = f.read()
        except OSError:
            return offset
        lines = data.decode(errors='replace').split('\n')
        if path == self.path:
            lines[0] = self._partial + lines[0]
            self._partial = lines.pop()       # a line Mosquitto is still writing
        self.sessions.feed_all(lines)
        return offset + len(data)


def session_dict(session: Optional[Session]) -> Optional[dict]:
    return asdict(session) if session else None


def probe(host: str, port: int, timeout: float = 0.3) -> bool:
    """Does something accept TCP connections on host:port."""
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False
