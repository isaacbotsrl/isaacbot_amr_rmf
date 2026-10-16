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
RMF site manager: a small web page for the fleet's robots and the RMF sites made from their maps.

    http://localhost:8020              the robots
    http://localhost:8020/robots/<n>   one robot: its ssh access, its saved maps (import)
    http://localhost:8020/sites        the RMF sites (one runs, for the whole fleet): activate

A robot page lists the maps isaacbot_amr_indoor_nav_manager saved on that isaacbot (over ssh, at the
robot's own address; the page's one key is installed on each robot with isaacbot-authorize).
Import turns one into a site in the rmf-sites volume: floor plan, building, nav graph, fleet
config and a preview with the lanes. RMF runs one site for the whole fleet. Activate writes the site's name to <sites>/active; docker/run_rmf.sh, which runs RMF,
restarts RMF on it. Built-in sites (isaacbot_amr_world, and CLI imports in isaacbot_amr_rmf_maps) can be
activated too.

Robots: the fleet's isaacbots (<sites>/robots.json, see robots.py). Adding one gives it a broker
login limited to its own VDA 5050 topics (server deployment: BROKER_AUTH_DIR) and an RMF fleet
entry; run_rmf.sh restarts RMF on the new list. The new password is shown once, with the balena
variables to paste.

Environment:
    SITES_DIR       the rmf-sites volume (default /sites)
    BUILTIN_MAPS    isaacbot_amr_rmf_maps' installed maps directory
    FLEET_TEMPLATE  fleet config the imported sites copy (fleet_isaacbot_amr_world.yaml)
    ISAACBOT_AMR_SSH       user@host of the isaacbot's orchestrator sshd (default root@192.168.1.201):
                           the ssh address of robots seeded from the fleet template
    ISAACBOT_AMR_SSH_PORT  default 24
    AUTHORIZE_CMD   what runs a command in this container, for the isaacbot-authorize command the
                    page shows (default: docker compose -f docker/docker-compose.yml exec site-manager)
    ISAACBOT_AMR_BROKER    host[:port] written into imported sites (default the ISAACBOT_AMR_SSH host, 1883)
    MAPS_DIR        read every robot's maps from this directory instead of ssh (the sim's --maps-dir)
    BROKER_AUTH_DIR Mosquitto password file + ACL to manage (the broker-auth volume); unset: the
                    broker is anonymous (local compose) and robots get no login
    ROBOT_BROKER    host[:port] the isaacbots reach the broker on, for the balena variables
                    (default ISAACBOT_AMR_BROKER)
    FLEET_API       the fleet adapter's control API, for live robot state (default http://rmf:8030)
    BROKER_LOG      the broker's log file, for where each robot connects from (default
                    $BROKER_AUTH_DIR/broker.log; none without BROKER_AUTH_DIR)
    SERVER_HOSTNAME, SERVER_VPN_IP, SERVER_LAN_IP   the host's name and addresses, for the
                    robots page's Network section (docker/rmf-server.sh passes them)
    SITE_MANAGER_PASSWORD  when set, every request needs HTTP Basic auth with this password
    PORT            default 8020
"""

import base64
from dataclasses import asdict, replace
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from typing import Callable, Optional, Sequence
from urllib.error import URLError
from urllib.parse import unquote
from urllib.request import urlopen

HERE = os.path.dirname(os.path.abspath(__file__))
for candidate in (os.path.join(HERE, '..', 'isaacbot_amr_rmf_maps', 'scripts'),
                  '/opt/isaacbot_amr_rmf_maps_scripts'):
    if os.path.isdir(candidate):
        sys.path.insert(0, candidate)
sys.path.insert(0, HERE)

from isaacbot_amr_map_import import import_site, MapImportError  # noqa: E402
import network  # noqa: E402
import robots as fleet_robots  # noqa: E402
from robots import Robot, RobotConflict, RobotError, RobotNotFound  # noqa: E402
from site_io import DirMapSource, render_preview, SshMapSource  # noqa: E402
from site_io import write_site  # noqa: E402
import yaml  # noqa: E402

NAME = re.compile(r'[A-Za-z0-9_-]{1,64}')
NAV_GRAPH_CMD = ('ros2', 'run', 'rmf_building_map_tools', 'building_map_generator', 'nav')
DEFAULT_AUTHORIZE_CMD = 'docker compose -f docker/docker-compose.yml exec site-manager'
# The stack's services: (name, host and port in the compose network, port published on the
# host (None: the robots' broker port), has a web page). The site manager itself answers this request, so it is always up.
SERVICES = (
    ('RMF dashboard', 'dashboard', 3000, 3000, True),
    ('RMF API server', 'api-server', 8000, 8010, True),
    ('Site manager', '', 8020, 8020, True),
    ('MQTT broker', 'mosquitto', 1883, None, False),
    ('Fleet control API', 'rmf', 8030, 8030, False),
    ('RMF schedule websocket', 'rmf', 8006, 8006, False),
)


class RequestError(ValueError):
    """Bad request: the message goes back to the page as is."""


def _valid_name(name: str) -> str:
    if not isinstance(name, str) or not NAME.fullmatch(name):
        raise RequestError(f'{name!r} is not a map name ([A-Za-z0-9_-], up to 64 characters)')
    return name


class SiteManager:

    def __init__(self, sites_dir: str, builtin_maps: str, template_path: str,
                 source_for: Callable[[Robot], object],
                 broker: tuple, nav_graph_cmd: Sequence[str] = NAV_GRAPH_CMD,
                 clock: Callable[[], float] = time.time,
                 broker_auth: Optional[fleet_robots.BrokerAuth] = None,
                 robot_broker: Optional[tuple] = None,
                 fleet_state: Optional[Callable[[], dict]] = None,
                 default_ssh: str = '', authorize_cmd: str = DEFAULT_AUTHORIZE_CMD,
                 broker_log: Optional[network.BrokerLog] = None, host_info: Optional[dict] = None,
                 services: Sequence[tuple] = SERVICES,
                 probe: Callable[[str, int], bool] = network.probe):
        """
        source_for(robot): where that robot's saved maps are read from (an SshMapSource, a
        DirMapSource for the simulation), or None when it has none. default_ssh: the ssh address
        robots seeded from the fleet template get. authorize_cmd: what runs isaacbot-authorize
        in the site-manager container, for the command the page shows. broker_log: the broker's
        log (None: no connection data). host_info: hostname, vpn_ip, lan_ip of the host. services:
        see SERVICES; probe(host, port) checks one.
        """
        self.broker_log = broker_log
        self.host_info = host_info or {}
        self.services = list(services)
        self.probe = probe
        self._services = (-1e9, [])            # (checked, services)
        self.sites_dir = sites_dir
        self.builtin_maps = builtin_maps
        self.template_path = template_path
        self.source_for = source_for
        self.default_ssh = default_ssh
        self.authorize_cmd = authorize_cmd
        self.broker = broker
        self.nav_graph_cmd = list(nav_graph_cmd)
        self.clock = clock
        self.broker_auth = broker_auth
        self.robot_broker = robot_broker or broker
        self.fleet_state = fleet_state or (lambda: {})
        self.robots_path = os.path.join(sites_dir, 'robots.json')
        self._lock = threading.Lock()          # one import/activate/delete at a time
        self._access = {}                      # robot -> (ssh, state, detail, checked)
        os.makedirs(sites_dir, exist_ok=True)
        self._seed_robots()

    # --- sites -----------------------------------------------------------------------------

    def _site_dir(self, name: str, builtin: bool) -> str:
        return os.path.join(self.builtin_maps if builtin else self.sites_dir, name)

    def _site_info(self, name: str, builtin: bool) -> Optional[dict]:
        directory = self._site_dir(name, builtin)
        building = os.path.join(directory, f'{name}.building.yaml')
        if not os.path.isfile(building):
            return None
        info = {'name': name, 'builtin': builtin, 'waypoints': [], 'lanes': 0, 'charger': None,
                'modified': os.path.getmtime(building),
                'nav_graph': os.path.isfile(os.path.join(directory, 'nav_graphs', '0.yaml'))}
        try:
            with open(building) as f:
                level = (yaml.safe_load(f).get('levels') or {}).get(name) or {}
            for v in level.get('vertices') or []:
                if len(v) > 3 and v[3]:
                    info['waypoints'].append(str(v[3]))
                    if len(v) > 4 and (v[4] or {}).get('is_charger', [0, False])[1]:
                        info['charger'] = str(v[3])
            info['lanes'] = len(level.get('lanes') or [])
        except (OSError, yaml.YAMLError, AttributeError):
            pass
        meta_path = os.path.join(directory, 'import.json')
        if not builtin and os.path.isfile(meta_path):
            with open(meta_path) as f:
                info['import'] = json.load(f)
        return info

    def sites(self) -> list:
        found = {}
        if os.path.isdir(self.builtin_maps):
            for name in sorted(os.listdir(self.builtin_maps)):
                info = self._site_info(name, builtin=True)
                if info:
                    found[name] = info
        for name in sorted(os.listdir(self.sites_dir)):
            info = self._site_info(name, builtin=False)
            if info:
                info['overrides_builtin'] = name in found
                found[name] = info       # an imported site wins, as in run_rmf.sh
        return sorted(found.values(), key=lambda s: (s['builtin'], s['name']))

    def active(self) -> str:
        try:
            with open(os.path.join(self.sites_dir, 'active')) as f:
                return f.read().strip()
        except OSError:
            return ''

    def rmf_status(self) -> dict:
        try:
            with open(os.path.join(self.sites_dir, 'rmf_status.json')) as f:
                return json.load(f)
        except (OSError, ValueError):
            return {}

    def access(self, robot: Robot, max_age: float = 15.0) -> dict:
        """Can the page read this robot's maps (cached max_age s per robot and address)."""
        source = self.source_for(robot)
        if source is None:
            return {'state': 'no_ssh', 'detail': '', 'source': ''}
        ssh, state, detail, checked = self._access.get(robot.name, ('', 'unknown', '', -1e9))
        if ssh != robot.ssh or self.clock() - checked > max_age:
            state, detail = source.check()
            self._access[robot.name] = (robot.ssh, state, detail, self.clock())
        return {'state': state, 'detail': detail, 'source': source.describe()}

    def status(self) -> dict:
        return {'active': self.active(), 'rmf': self.rmf_status(), 'sites': self.sites(),
                'broker': f'{self.broker[0]}:{self.broker[1]}'}

    def _source(self, robot: Robot):
        source = self.source_for(robot)
        if source is None:
            raise RequestError(f'{robot.name} has no ssh address: set one to read its maps')
        return source

    def robot_maps(self, name: str) -> list:
        return self._source(self._robot(name)).list_maps()

    # --- actions ---------------------------------------------------------------------------

    def import_map(self, robot_name: str, request: dict) -> dict:
        source = self._source(self._robot(robot_name))
        name = _valid_name(request.get('name'))
        if name == 'isaacbot_amr_world':
            raise RequestError('isaacbot_amr_world is the Gazebo site; save the map under another name')
        charger = str(request.get('charger') or 'dock')
        try:
            clearance = float(request.get('clearance', 0.8))
            max_lane = float(request.get('max_lane', 15.0))
            neighbours = int(request.get('neighbours', 3))
        except (TypeError, ValueError):
            raise RequestError('clearance, max_lane and neighbours must be numbers')
        if not (clearance > 0 and max_lane > 0 and neighbours >= 0):
            raise RequestError('clearance and max_lane must be > 0, neighbours >= 0')

        with self._lock, tempfile.TemporaryDirectory() as tmp:
            map_dir = source.fetch(name, tmp)
            site = import_site(name, map_dir, charger, clearance, max_lane, neighbours)
            # Build the new site next to the old one, swap only once the nav graph exists: a
            # failed import leaves the previous version (maybe the one RMF runs) intact.
            staging = os.path.join(self.sites_dir, f'.{name}.new')
            shutil.rmtree(staging, ignore_errors=True)
            try:
                charger_name = self._build(site, staging)
            except BaseException:  # noqa: B902 cleanup, then re-raised
                shutil.rmtree(staging, ignore_errors=True)
                raise
            summary = {
                'name': name, 'charger': charger_name, 'imported': self.clock(),
                'robot': robot_name, 'source': source.describe(),
                'size_px': [site.width, site.height], 'resolution': site.resolution,
                'translation': list(site.translation),
                'waypoints': [w.name for w in site.waypoints],
                'lanes': [list(lane) for lane in site.lanes], 'warnings': site.warnings,
                'options': {'clearance': clearance, 'max_lane': max_lane,
                            'neighbours': neighbours}}
            with open(os.path.join(staging, 'import.json'), 'w') as f:
                json.dump(summary, f, indent=1)
            final = os.path.join(self.sites_dir, name)
            shutil.rmtree(final, ignore_errors=True)
            os.rename(staging, final)
        summary['active'] = self.active() == name
        return summary

    def _build(self, site, directory: str) -> str:
        with open(self.template_path) as f:
            template = yaml.safe_load(f)
        charger = write_site(site, directory, os.path.join(directory, f'fleet_{site.name}.yaml'),
                             template, self.broker[0], self.broker[1],
                             origin='the RMF site manager')
        self._nav_graph(os.path.join(directory, f'{site.name}.building.yaml'),
                        os.path.join(directory, 'nav_graphs'))
        render_preview(site, os.path.join(directory, 'preview.png'))
        return charger

    def _nav_graph(self, building: str, out_dir: str) -> None:
        result = subprocess.run(self.nav_graph_cmd + [building, out_dir],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=120)
        if result.returncode != 0 or not os.path.isfile(os.path.join(out_dir, '0.yaml')):
            output = result.stdout.decode(errors='replace').strip().splitlines()[-5:]
            raise MapImportError('building_map_generator could not make the nav graph: '
                                 + ' | '.join(output))

    def activate(self, request: dict) -> dict:
        name = _valid_name(request.get('name'))
        if not any(s['name'] == name for s in self.sites()):
            raise RequestError(f'no site {name!r}; import it first')
        with self._lock:
            path = os.path.join(self.sites_dir, 'active')
            with open(path + '.tmp', 'w') as f:
                f.write(name + '\n')
            os.replace(path + '.tmp', path)
        return {'active': name}

    def delete(self, name: str) -> dict:
        name = _valid_name(name)
        directory = self._site_dir(name, builtin=False)
        if not os.path.isfile(os.path.join(directory, f'{name}.building.yaml')):
            raise RequestError(f'{name!r} is not an imported site (built-in sites stay)')
        if self.active() == name or self.rmf_status().get('site') == name:
            raise RequestError(f'RMF runs {name!r}; activate another site first')
        with self._lock:
            shutil.rmtree(directory)
        return {'deleted': name}

    def image(self, name: str, which: str) -> Optional[str]:
        """preview.png of an imported site, or a site's plain floor plan."""
        name = _valid_name(name)
        for builtin in (False, True):
            directory = self._site_dir(name, builtin)
            path = os.path.join(directory, 'preview.png' if which == 'preview'
                                else f'{name}.png')
            if os.path.isfile(path):
                return path
        return None

    # --- robots ----------------------------------------------------------------------------

    def _seed_robots(self) -> None:
        """robots.json starts as the fleet template's robots (Ardennes keeps its login)."""
        if os.path.isfile(self.robots_path):
            return
        try:
            with open(self.template_path) as f:
                template = yaml.safe_load(f) or {}
        except OSError:
            return
        fleet_robots.save(self.robots_path,
                          fleet_robots.from_fleet_config(template, ssh=self.default_ssh))

    def _robot_list(self) -> list:
        return fleet_robots.load(self.robots_path) or []

    def _active_site(self) -> Optional[dict]:
        """The site RMF runs (or will): as run_rmf.sh picks it, isaacbot_amr_world by default."""
        name = (self.rmf_status().get('site') or self.active()
                or os.environ.get('RMF_SITE') or 'isaacbot_amr_world')
        return next((s for s in self.sites() if s['name'] == name), None)

    def robots(self) -> dict:
        try:
            live = {r.get('name'): r for r in (self.fleet_state() or {}).get('robots') or []}
        except Exception:  # noqa: B902 RMF restarting or down: no live state, still the list
            live = {}
        site = self._active_site() or {}
        robots = []
        robot_list = self._robot_list()
        for robot in robot_list:
            entry = asdict(robot)
            entry.update(broker_login=robot.broker_login, topic_prefix=robot.topic_prefix,
                         live=live.get(robot.name),
                         ssh_shared_with=fleet_robots.ssh_shared_with(robot_list, robot))
            robots.append(entry)
        return {'robots': robots, 'broker_auth': self.broker_auth is not None,
                'broker': f'{self.robot_broker[0]}:{self.robot_broker[1]}',
                'site': site.get('name'), 'waypoints': site.get('waypoints') or [],
                'site_charger': site.get('charger')}

    def _robot(self, name: str) -> Robot:
        return self._find(self._robot_list(), name)

    def robot(self, name: str) -> dict:
        """One robot's page: identity, live state, settings, and whether its maps are readable."""
        robot = self._robot(name)
        fleet = self.robots()
        entry = next(r for r in fleet['robots'] if r['name'] == name)
        target = robot.ssh_target()
        entry.update(
            access=self.access(robot), public_key=public_key(),
            authorize=(f'{self.authorize_cmd} isaacbot-authorize {target[0]} {target[1]}'
                       if target else ''),
            broker_auth=fleet['broker_auth'], broker=fleet['broker'], site=fleet['site'],
            waypoints=fleet['waypoints'], site_charger=fleet['site_charger'],
            session=network.session_dict(self._session(robot.broker_login, fleet['broker_auth'])),
            broker_log=bool(self.broker_log and self.broker_log.available()),
            onboard_lan=network.ISAACBOT_LAN)
        return entry

    # --- network ---------------------------------------------------------------------------

    def service_status(self, max_age: float = 10.0) -> list:
        """The stack's services, each probed at most every max_age s."""
        checked, services = self._services
        if self.clock() - checked > max_age:
            services = []
            vpn_ip = self.host_info.get('vpn_ip')
            if vpn_ip:
                services.append({'name': 'Isaacbot website', 'port': 80, 'web': True,
                                 'up': self.probe(vpn_ip, 80)})
            for name, host, port, public, web in self.services:
                services.append({'name': name, 'port': public or self.robot_broker[1], 'web': web,
                                 'up': True if not host else self.probe(host, port)})
            self._services = (self.clock(), services)
        return services

    def _session(self, login: str, broker_auth: bool) -> Optional[network.Session]:
        """The login's broker session, from the broker log (None without logins or a log)."""
        if not broker_auth or not self.broker_log:
            return None
        return self.broker_log.read().for_login(login)

    def network(self) -> dict:
        """Everyone on the fleet's network: the server, its services, each robot, other clients."""
        fleet = self.robots()
        sessions = self.broker_log.read() if self.broker_log else network.BrokerSessions()
        robots = []
        for entry in fleet['robots']:
            session = sessions.for_login(entry['broker_login']) if fleet['broker_auth'] else None
            robots.append({k: entry[k] for k in (
                'name', 'manufacturer', 'serial_number', 'broker_login', 'topic_prefix', 'ssh',
                'live')} | {'session': network.session_dict(session)})
        others = sessions.others(r['broker_login'] for r in fleet['robots'])
        return {
            'server': dict(self.host_info, services=self.service_status()),
            'broker': {'address': fleet['broker'], 'auth': fleet['broker_auth'],
                       'log': bool(self.broker_log and self.broker_log.available())},
            'robots': robots,
            'others': [network.session_dict(s) for s in others],
        }

    def update_robot(self, name: str, request: dict) -> dict:
        """Change a robot's charger and/or ssh address (its identity and login stay)."""
        changes = {k: request[k].strip() for k in ('charger', 'ssh')
                   if isinstance(request.get(k), str)}
        if not changes:
            raise RequestError('nothing to change: send charger and/or ssh')
        waypoints = (self._active_site() or {}).get('waypoints')
        if changes.get('charger') and waypoints and changes['charger'] not in waypoints:
            raise RobotError(f'{changes["charger"]!r} is not a waypoint of the active site')
        with self._lock:
            robots = self._robot_list()
            robot = self._find(robots, name)
            updated = fleet_robots.validate(replace(robot, **changes))
            if 'ssh' in changes:
                fleet_robots.check_ssh_unique(robots, updated)
            fleet_robots.save(self.robots_path,
                              [updated if r.name == name else r for r in robots])
        self._access.pop(name, None)
        return asdict(updated)

    def add_robot(self, request: dict) -> dict:
        def field(key: str, default: str = '') -> str:
            value = request.get(key, default)
            return value.strip() if isinstance(value, str) else default

        robot = Robot(name=field('name'), serial_number=field('serial_number'),
                      manufacturer=field('manufacturer') or fleet_robots.DEFAULT_MANUFACTURER,
                      login=field('login'), charger=field('charger'), ssh=field('ssh'))
        reserved = (self.broker_auth.master_login,) if self.broker_auth else ()
        fleet_robots.validate(robot, reserved)
        waypoints = (self._active_site() or {}).get('waypoints')
        if robot.charger and waypoints and robot.charger not in waypoints:
            raise RobotError(f'{robot.charger!r} is not a waypoint of the active site')
        with self._lock:
            robots = self._robot_list()
            fleet_robots.check_unique(robots, robot)
            if self.broker_auth and robot.broker_login in self.broker_auth.logins():
                raise RobotConflict(f'the broker already has a login {robot.broker_login!r}')
            robots.append(robot)
            password = self._set_login(robot, robots)
            fleet_robots.save(self.robots_path, robots)
        return self._credentials(robot, password)

    def rotate_password(self, name: str) -> dict:
        if self.broker_auth is None:
            raise RequestError('this broker has no logins (local compose): nothing to rotate')
        with self._lock:
            robots = self._robot_list()
            robot = self._find(robots, name)
            password = self._set_login(robot, robots)
        return self._credentials(robot, password)

    def delete_robot(self, name: str) -> dict:
        with self._lock:
            robots = self._robot_list()
            robot = self._find(robots, name)
            if len(robots) == 1:
                raise RequestError('the fleet needs at least one robot; add the new one first')
            robots = [r for r in robots if r.name != name]
            fleet_robots.save(self.robots_path, robots)
            if self.broker_auth:
                self.broker_auth.write_acl(robots)
                self.broker_auth.set_password(robot.broker_login, None)
        return {'deleted': name}

    @staticmethod
    def _find(robots: list, name: str) -> Robot:
        robot = next((r for r in robots if r.name == name), None)
        if robot is None:
            raise RobotNotFound(f'no robot {name!r}')
        return robot

    def _set_login(self, robot: Robot, robots: list) -> Optional[str]:
        """A new password for the robot's login (and the ACL for robots); None without auth."""
        if self.broker_auth is None:
            return None
        password = fleet_robots.make_password()
        self.broker_auth.set_password(robot.broker_login, password)
        self.broker_auth.write_acl(robots)
        return password

    def _credentials(self, robot: Robot, password: Optional[str]) -> dict:
        """What the isaacbot's balena variables must say (the password is not stored)."""
        host, port = self.robot_broker
        variables = {
            'ISAACBOT_AMR_VDA5050_ENABLE': 'true',
            'ISAACBOT_AMR_VDA5050_LOCAL_BROKER': 'false',
            'ISAACBOT_AMR_VDA5050_BROKER_HOST': str(host),
            'ISAACBOT_AMR_VDA5050_BROKER_PORT': str(port),
            'ISAACBOT_AMR_VDA5050_MANUFACTURER': robot.manufacturer,
            'ISAACBOT_AMR_VDA5050_SERIAL_NUMBER': robot.serial_number,
        }
        if password is not None:
            variables.update({
                'ISAACBOT_AMR_VDA5050_BROKER_USER': robot.broker_login,
                'ISAACBOT_AMR_VDA5050_BROKER_PASSWORD': password,
                'ISAACBOT_AMR_VDA5050_BROKER_TLS': 'false',
            })
        return {'robot': asdict(replace(robot, login=robot.broker_login)),
                'password': password, 'balena': variables}


def fleet_api_state(url: str, timeout: float = 1.0) -> Callable[[], dict]:
    """GET <url>/api/robots on the fleet adapter's control API ({} when it does not answer)."""
    def fetch() -> dict:
        try:
            with urlopen(f'{url.rstrip("/")}/api/robots', timeout=timeout) as response:
                return json.load(response)
        except (URLError, OSError, ValueError):
            return {}
    return fetch


def public_key() -> str:
    path = os.path.expanduser('~/.ssh/id_ed25519.pub')
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return ''


def ensure_key() -> None:
    """The site manager's own ssh key, made once (it lives in the site-manager-ssh volume)."""
    key = os.path.expanduser('~/.ssh/id_ed25519')
    if not os.path.isfile(key):
        os.makedirs(os.path.dirname(key), mode=0o700, exist_ok=True)
        subprocess.run(['ssh-keygen', '-q', '-t', 'ed25519', '-N', '', '-C',
                        'isaacbot-rmf-site-manager', '-f', key], check=True)


# --- HTTP ---------------------------------------------------------------------------------------

# The only static files served, besides the two pages: no path from a request reaches the disk.
STATIC = {'/isaacbot-mark-black.svg': 'image/svg+xml', '/isaacbot-mark-white.svg': 'image/svg+xml',
          '/common.js': 'text/javascript; charset=utf-8', '/common.css': 'text/css; charset=utf-8',
          '/isaacbot-lockup-black-bold.svg': 'image/svg+xml',
          '/isaacbot-lockup-white.svg': 'image/svg+xml'}
ROBOT_PATH = re.compile(r'/api/robots/([^/]+)')


def make_handler(manager: SiteManager, static_dir: str, password: str = ''):
    """password: when set, every request needs HTTP Basic auth with it (any user name)."""

    class Handler(BaseHTTPRequestHandler):
        server_version = 'IsaacbotRmfSites/1'

        def log_message(self, fmt, *args):
            if (not self.path.startswith(('/api/status', '/api/robots', '/api/network'))
                    or 'GET' not in fmt % args):
                # (the pages poll those every 3 s)
                sys.stderr.write('site-manager: ' + fmt % args + '\n')

        def _authorized(self) -> bool:
            if not password:
                return True
            header = self.headers.get('Authorization', '')
            if header.startswith('Basic '):
                try:
                    _, _, given = base64.b64decode(header[6:]).decode().partition(':')
                except (ValueError, UnicodeDecodeError):
                    given = ''
                if hmac.compare_digest(given.encode(), password.encode()):
                    return True
            self.send_response(401)
            self.send_header('WWW-Authenticate', 'Basic realm="RMF sites", charset="UTF-8"')
            self.send_header('Content-Length', '0')
            self.end_headers()
            return False

        def _send(self, code: int, body: bytes, content_type: str):
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code: int, data) -> None:
            self._send(code, json.dumps(data).encode(), 'application/json')

        def _call(self, action):
            try:
                self._json(200, action())
            except RobotNotFound as e:
                self._json(404, {'error': str(e)})
            except RobotConflict as e:
                self._json(409, {'error': str(e)})
            except (RequestError, MapImportError, RobotError) as e:
                self._json(400, {'error': str(e)})
            except Exception as e:  # noqa: B902 the page shows it; the server keeps running
                self._json(500, {'error': f'{type(e).__name__}: {e}'})

        def _body(self) -> dict:
            length = int(self.headers.get('Content-Length') or 0)
            try:
                data = json.loads(self.rfile.read(length) or b'{}')
            except ValueError:
                raise RequestError('the request body is not JSON')
            if not isinstance(data, dict):
                raise RequestError('the request body must be a JSON object')
            return data

        def _page(self, name: str):
            with open(os.path.join(static_dir, name), 'rb') as f:
                self._send(200, f.read(), 'text/html; charset=utf-8')

        def do_GET(self):  # noqa: N802
            if not self._authorized():
                return
            path = unquote(self.path.split('?')[0])
            if path in ('/', '/index.html'):
                return self._page('index.html')
            if path in ('/sites', '/sites/'):
                return self._page('sites.html')
            if re.fullmatch(r'/robots/[^/]+', path):
                return self._page('robot.html')       # the page reads the name from its URL
            if path in STATIC:
                with open(os.path.join(static_dir, path[1:]), 'rb') as f:
                    return self._send(200, f.read(), STATIC[path])
            if path == '/api/status':
                return self._call(manager.status)
            if path == '/api/robots':
                return self._call(manager.robots)
            if path == '/api/network':
                return self._call(manager.network)
            match = re.fullmatch(r'/api/robots/([^/]+)(/maps)?', path)
            if match:
                if match.group(2):
                    return self._call(lambda: manager.robot_maps(match.group(1)))
                return self._call(lambda: manager.robot(match.group(1)))
            match = re.fullmatch(r'/sites/([^/]+)/(preview|floorplan)\.png', path)
            if match:
                try:
                    image = manager.image(match.group(1), match.group(2))
                except RequestError:
                    image = None
                if image:
                    with open(image, 'rb') as f:
                        return self._send(200, f.read(), 'image/png')
            self._json(404, {'error': 'not found'})

        def do_POST(self):  # noqa: N802
            if not self._authorized():
                return
            if self.path == '/api/activate':
                return self._call(lambda: manager.activate(self._body()))
            if self.path == '/api/robots':
                return self._call(lambda: manager.add_robot(self._body()))
            match = re.fullmatch(r'/api/robots/([^/]+)/(password|import)', unquote(self.path))
            if match and match.group(2) == 'password':
                return self._call(lambda: manager.rotate_password(match.group(1)))
            if match:
                return self._call(lambda: manager.import_map(match.group(1), self._body()))
            self._json(404, {'error': 'not found'})

        def do_PATCH(self):  # noqa: N802
            if not self._authorized():
                return
            match = ROBOT_PATH.fullmatch(unquote(self.path))
            if match:
                return self._call(lambda: manager.update_robot(match.group(1), self._body()))
            self._json(404, {'error': 'not found'})

        def do_DELETE(self):  # noqa: N802
            if not self._authorized():
                return
            match = re.fullmatch(r'/api/sites/([^/]+)', unquote(self.path))
            if match:
                return self._call(lambda: manager.delete(match.group(1)))
            match = re.fullmatch(r'/api/robots/([^/]+)', unquote(self.path))
            if match:
                return self._call(lambda: manager.delete_robot(match.group(1)))
            self._json(404, {'error': 'not found'})

    return Handler


def main():
    ssh_target = os.environ.get('ISAACBOT_AMR_SSH', 'root@192.168.1.201')
    ssh_port = os.environ.get('ISAACBOT_AMR_SSH_PORT', '24')
    host = ssh_target.split('@')[-1]
    broker = os.environ.get('ISAACBOT_AMR_BROKER', f'{host}:1883')
    broker_host, _, broker_port = broker.partition(':')
    if os.environ.get('MAPS_DIR'):
        # The simulation: every robot's maps are this directory.
        maps_dir = DirMapSource(os.environ['MAPS_DIR'])
        source_for = lambda robot: maps_dir  # noqa: E731
        describe = maps_dir.describe()
    else:
        ensure_key()

        def source_for(robot):
            target = robot.ssh_target()
            return SshMapSource(*target) if target else None
        describe = "each robot's ssh address"
    robot_host, _, robot_port = os.environ.get('ROBOT_BROKER', broker).partition(':')
    auth_dir = os.environ.get('BROKER_AUTH_DIR', '')
    broker_log = os.environ.get('BROKER_LOG') or (
        os.path.join(auth_dir, 'broker.log') if auth_dir else '')
    host_info = {'hostname': os.environ.get('SERVER_HOSTNAME', ''),
                 'vpn_ip': os.environ.get('SERVER_VPN_IP', ''),
                 'lan_ip': os.environ.get('SERVER_LAN_IP', '')}
    manager = SiteManager(
        sites_dir=os.environ.get('SITES_DIR', '/sites'),
        builtin_maps=os.environ.get(
            'BUILTIN_MAPS', '/rmf_ws/install/isaacbot_amr_rmf_maps/share/isaacbot_amr_rmf_maps/maps'),
        template_path=os.environ.get(
            'FLEET_TEMPLATE',
            '/rmf_ws/install/isaacbot_amr_rmf_bringup/share/isaacbot_amr_rmf_bringup/config/'
            'fleet_isaacbot_amr_world.yaml'),
        source_for=source_for, broker=(broker_host, int(broker_port or 1883)),
        default_ssh=f'{ssh_target}:{ssh_port}',
        authorize_cmd=os.environ.get('AUTHORIZE_CMD', DEFAULT_AUTHORIZE_CMD),
        broker_auth=fleet_robots.BrokerAuth(auth_dir) if auth_dir else None,
        robot_broker=(robot_host, int(robot_port or 1883)),
        fleet_state=fleet_api_state(os.environ.get('FLEET_API', 'http://rmf:8030')),
        broker_log=network.BrokerLog(broker_log) if broker_log else None, host_info=host_info)
    port = int(os.environ.get('PORT', '8020'))
    password = os.environ.get('SITE_MANAGER_PASSWORD', '')
    server = ThreadingHTTPServer(('0.0.0.0', port),
                                 make_handler(manager, os.path.join(HERE, 'static'), password))
    print(f'RMF site manager on :{port}; maps from {describe}, sites in '
          f'{manager.sites_dir}; broker logins '
          f'{"managed in " + auth_dir if auth_dir else "off (anonymous broker)"}; '
          f'{"password protected" if password else "no password"}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
