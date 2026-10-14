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
The fleet's isaacbots: one list (<sites>/robots.json) that RMF and the broker both follow.

Each robot has an RMF name, a VDA 5050 identity (manufacturer, serialNumber: its topics are
uagv/v2/<manufacturer>/<serial>/...), a broker login that may only use those topics, and a
charger waypoint (empty: the site's charger).

- RMF: docker/run_rmf.sh merges the list into the active site's fleet config before launching
  (`robots.py merge`), and restarts RMF when the list changes.
- Broker (server deployment, mosquitto.server.conf): the site manager writes the Mosquitto
  password file and ACL into the broker-auth volume; docker/mosquitto-run.sh makes Mosquitto
  reload them (SIGHUP) without dropping clients.

    robots.py merge <fleet config> <robots.json> <out>   the fleet config RMF runs

Everything here is pure (no HTTP, no RMF) so it is unit-tested on its own.
"""

import base64
import copy
import hashlib
import json
import os
import re
import secrets
import sys
from dataclasses import asdict, dataclass
from typing import Iterable, List, Optional, Sequence

NAME = re.compile(r'[A-Za-z][A-Za-z0-9_]{0,63}')
# VDA 5050 2.0 serialNumber / manufacturer characters (topic levels: no / + # $).
VDA_ID = re.compile(r'[A-Za-z0-9_.:-]{1,64}')
LOGIN = re.compile(r'[A-Za-z0-9_.-]{1,64}')
DEFAULT_MANUFACTURER = 'IsaacBot'
MOSQUITTO_UID = 1883          # the eclipse-mosquitto image's user; it must own its auth files
PBKDF2_ITERATIONS = 101       # mosquitto_passwd's default for $7$ (PBKDF2-SHA512)


class RobotError(ValueError):
    """A bad robot request: the message goes back to the page as is."""


class RobotConflict(RobotError):
    """The robot (or its serial / login) already exists."""


class RobotNotFound(RobotError):
    """No robot of that name."""


@dataclass(frozen=True)
class Robot:
    name: str                      # RMF robot name (fleet config key)
    serial_number: str             # VDA 5050 serialNumber
    manufacturer: str = DEFAULT_MANUFACTURER
    login: str = ''                # broker login; '' = name
    charger: str = ''              # waypoint; '' = the site's charger

    @property
    def broker_login(self) -> str:
        return self.login or self.name

    @property
    def topic_prefix(self) -> str:
        return f'uagv/v2/{self.manufacturer}/{self.serial_number}'


def validate(robot: Robot, reserved_logins: Sequence[str] = ()) -> Robot:
    if not NAME.fullmatch(robot.name or ''):
        raise RobotError(f'{robot.name!r} is not a robot name (a letter, then A-Z a-z 0-9 _)')
    for label, value in (('serial number', robot.serial_number),
                         ('manufacturer', robot.manufacturer)):
        if not VDA_ID.fullmatch(value or ''):
            raise RobotError(f'{value!r} is not a VDA 5050 {label} (A-Z a-z 0-9 _ . : -)')
    if not LOGIN.fullmatch(robot.broker_login):
        raise RobotError(f'{robot.broker_login!r} is not a broker login (A-Z a-z 0-9 _ . -)')
    if robot.broker_login in reserved_logins:
        raise RobotError(f'the broker login {robot.broker_login!r} is reserved')
    if robot.charger and not re.fullmatch(r'[^\s]{1,128}', robot.charger):
        raise RobotError(f'{robot.charger!r} is not a waypoint name')
    return robot


def check_unique(robots: Sequence[Robot], new: Robot) -> None:
    for r in robots:
        if r.name == new.name:
            raise RobotConflict(f'robot {new.name!r} already exists')
        if (r.manufacturer, r.serial_number) == (new.manufacturer, new.serial_number):
            raise RobotConflict(f'{r.name!r} already uses {new.topic_prefix}')
        if r.broker_login == new.broker_login:
            raise RobotConflict(f'{r.name!r} already uses the broker login {new.broker_login!r}')


# --- robots.json -------------------------------------------------------------------------------

def load(path: str) -> Optional[List[Robot]]:
    """The robots in path, or None when the file does not exist (not yet seeded)."""
    try:
        with open(path) as f:
            data = json.load(f)
    except FileNotFoundError:
        return None
    return [Robot(**{k: str(v) for k, v in r.items() if k in Robot.__dataclass_fields__})
            for r in data.get('robots', [])]


def save(path: str, robots: Sequence[Robot]) -> None:
    _write_atomic(path, json.dumps({'robots': [asdict(r) for r in robots]}, indent=1) + '\n')


def from_fleet_config(config: dict) -> List[Robot]:
    """The robots a fleet config names (to seed robots.json from the fleet template)."""
    vda = config.get('vda5050') or {}
    vda_robots = vda.get('robots') or {}
    manufacturer = str(vda.get('manufacturer') or DEFAULT_MANUFACTURER)
    robots = []
    for name in (config.get('rmf_fleet') or {}).get('robots') or {}:
        entry = vda_robots.get(name) or {}
        robots.append(Robot(
            name=name,
            serial_number=str(entry.get('serial_number') or name),
            manufacturer=str(entry.get('manufacturer') or manufacturer)))
    return robots


# --- fleet config ------------------------------------------------------------------------------

def site_charger(config: dict) -> str:
    """The site's charger: the one its fleet config gives its first robot."""
    for entry in ((config.get('rmf_fleet') or {}).get('robots') or {}).values():
        if isinstance(entry, dict) and entry.get('charger'):
            return str(entry['charger'])
    return ''


def apply_to_fleet(config: dict, robots: Iterable[Robot],
                   waypoints: Optional[Sequence[str]] = None) -> dict:
    """
    The fleet config with exactly these robots.

    Each keeps the site's robot settings (the first robot's entry is the template), gets its own
    charger when it names one the site has (waypoints; None skips the check), else the site's,
    and its VDA identity under vda5050.robots.
    """
    config = copy.deepcopy(config)
    fleet = config.setdefault('rmf_fleet', {})
    vda = config.setdefault('vda5050', {})
    old = fleet.get('robots') or {}
    template = copy.deepcopy(next(iter(old.values()), None) or {})
    default_charger = site_charger(config)
    fleet['robots'], vda['robots'] = {}, {}
    for robot in robots:
        entry = copy.deepcopy(old.get(robot.name) or template)
        charger = robot.charger
        if charger and waypoints is not None and charger not in waypoints:
            charger = ''
        entry['charger'] = charger or default_charger
        fleet['robots'][robot.name] = entry
        vda['robots'][robot.name] = {'serial_number': robot.serial_number,
                                     'manufacturer': robot.manufacturer}
    return config


def site_waypoints(building_path: str) -> Optional[List[str]]:
    """Named vertices of a building.yaml (all levels), or None when it cannot be read."""
    import yaml
    try:
        with open(building_path) as f:
            building = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError):
        return None
    names = []
    for level in (building.get('levels') or {}).values():
        for v in (level or {}).get('vertices') or []:
            if len(v) > 3 and v[3]:
                names.append(str(v[3]))
    return names


def merge(config_path: str, robots_path: str, out_path: str, building_path: str = '') -> str:
    """The fleet config RMF runs: config_path with robots.json's robots, written to out_path."""
    import yaml
    with open(config_path) as f:
        config = yaml.safe_load(f)
    robots = load(robots_path)
    notes = []
    if robots is not None:
        waypoints = site_waypoints(building_path) if building_path else None
        for r in robots:
            if r.charger and waypoints is not None and r.charger not in waypoints:
                notes.append(f'{r.name}: charger {r.charger!r} is not in this site; '
                             'using the site charger')
        config = apply_to_fleet(config, robots, waypoints)
    os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)
    _write_atomic(out_path, f'# Generated by run_rmf.sh from {config_path} and {robots_path}.\n'
                  + yaml.safe_dump(config, sort_keys=False, default_flow_style=None))
    return '\n'.join(notes)


# --- broker logins -----------------------------------------------------------------------------

def make_password() -> str:
    return secrets.token_hex(16)


def hash_password(password: str, salt: Optional[bytes] = None,
                  iterations: int = PBKDF2_ITERATIONS) -> str:
    """A Mosquitto 2 password file hash: $7$<iterations>$<b64 salt>$<b64 PBKDF2-SHA512>."""
    salt = salt if salt is not None else secrets.token_bytes(12)
    digest = hashlib.pbkdf2_hmac('sha512', password.encode(), salt, iterations)
    return (f'$7${iterations}${base64.b64encode(salt).decode()}'
            f'${base64.b64encode(digest).decode()}')


def check_password(password: str, hashed: str) -> bool:
    try:
        _, kind, iterations, salt, digest = hashed.split('$')
    except ValueError:
        return False
    if kind != '7':
        return False
    return hash_password(password, base64.b64decode(salt), int(iterations)) == hashed


def passwd_set(text: str, login: str, password: Optional[str]) -> str:
    """The password file with login set to password (None: removed); other lines kept."""
    lines = [line for line in text.splitlines()
             if line.strip() and line.split(':', 1)[0] != login]
    if password is not None:
        lines.append(f'{login}:{hash_password(password)}')
    return '\n'.join(lines) + '\n'


def passwd_logins(text: str) -> List[str]:
    return [line.split(':', 1)[0] for line in text.splitlines() if ':' in line]


def render_acl(robots: Iterable[Robot], master_login: str = 'rmf') -> str:
    lines = [
        '# Generated by the RMF site manager from robots.json; edits here are overwritten.',
        '# VDA 5050 topics: uagv/v2/<manufacturer>/<serial>/<topic>.',
        '',
        '# The RMF fleet adapter (master control) talks to every isaacbot.',
        f'user {master_login}',
        'topic readwrite uagv/#',
    ]
    for robot in robots:
        lines += ['', f'# {robot.name}: only its own topics.',
                  f'user {robot.broker_login}', f'topic readwrite {robot.topic_prefix}/#']
    return '\n'.join(lines) + '\n'


class BrokerAuth:
    """The Mosquitto password file and ACL in a directory (the broker-auth volume)."""

    def __init__(self, directory: str, master_login: str = 'rmf', owner_uid: int = MOSQUITTO_UID):
        self.directory = directory
        self.master_login = master_login
        self.owner_uid = owner_uid
        self.passwd_path = os.path.join(directory, 'passwd')
        self.acl_path = os.path.join(directory, 'acl')

    def ready(self) -> bool:
        return os.path.isfile(self.passwd_path)

    def _read_passwd(self) -> str:
        try:
            with open(self.passwd_path) as f:
                return f.read()
        except FileNotFoundError:
            return ''

    def logins(self) -> List[str]:
        return passwd_logins(self._read_passwd())

    def set_password(self, login: str, password: Optional[str]) -> None:
        if login == self.master_login:
            raise RobotError(f'{login!r} is the fleet adapter login; it is not managed here')
        self._write(self.passwd_path, passwd_set(self._read_passwd(), login, password))

    def write_acl(self, robots: Iterable[Robot]) -> None:
        self._write(self.acl_path, render_acl(robots, self.master_login))

    def _write(self, path: str, text: str) -> None:
        _write_atomic(path, text, mode=0o600, uid=self.owner_uid)


def _write_atomic(path: str, text: str, mode: int = 0o644, uid: Optional[int] = None) -> None:
    tmp = f'{path}.tmp'
    with open(tmp, 'w') as f:
        f.write(text)
    os.chmod(tmp, mode)
    if uid is not None and os.geteuid() == 0:
        os.chown(tmp, uid, uid)
    os.replace(tmp, path)


def main(argv: Sequence[str]) -> int:
    if len(argv) >= 4 and argv[0] == 'merge':
        notes = merge(argv[1], argv[2], argv[3], argv[4] if len(argv) > 4 else '')
        if notes:
            print(notes, file=sys.stderr)
        return 0
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
