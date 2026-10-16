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

import base64
from http.server import ThreadingHTTPServer
import json
import os
import sys
import threading
import urllib.error
import urllib.request

import pytest

HERE = os.path.dirname(__file__)
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'site_manager'))
sys.path.insert(0, os.path.join(REPO, 'isaacbot_amr_rmf_maps', 'test'))

import server  # noqa: E402
from site_io import DirMapSource  # noqa: E402
from test_isaacbot_amr_map_import import write_map  # noqa: E402
import yaml  # noqa: E402

# Stands in for building_map_generator nav <building> <out_dir>: writes out_dir/0.yaml.
FAKE_NAV_GRAPH = ['python3', '-c',
                  'import os, sys; os.makedirs(sys.argv[2], exist_ok=True); '
                  'open(os.path.join(sys.argv[2], "0.yaml"), "w").write("levels: {}\\n")']
FAILING_NAV_GRAPH = ['python3', '-c', 'import sys; print("no lanes"); sys.exit(1)']


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


@pytest.fixture
def manager(tmp_path):
    maps = tmp_path / 'maps'
    (maps / 'lab').mkdir(parents=True)
    write_map(maps / 'lab')
    source = DirMapSource(str(maps))
    manager = server.SiteManager(
        sites_dir=str(tmp_path / 'sites'),
        builtin_maps=os.path.join(REPO, 'isaacbot_amr_rmf_maps', 'maps'),
        template_path=os.path.join(REPO, 'isaacbot_amr_rmf_bringup', 'config',
                                   'fleet_isaacbot_amr_world.yaml'),
        # A robot's maps come from its ssh address; here every address reads the test maps.
        source_for=lambda robot: source if robot.ssh else None,
        broker=('192.168.1.201', 1883), nav_graph_cmd=FAKE_NAV_GRAPH, clock=Clock(),
        default_ssh='root@192.168.1.201:24')
    manager.test_source = source
    return manager


def test_builtin_sites_are_listed(manager):
    sites = {s['name']: s for s in manager.sites()}
    world = sites['isaacbot_amr_world']
    assert world['builtin'] and world['charger'] == 'ardennes_charger'
    assert len(world['waypoints']) == 9 and world['lanes'] == 11


def test_import_writes_a_complete_site(manager):
    summary = manager.import_map('ardennes', {'name': 'lab', 'charger': 'dock', 'clearance': 0.3})
    assert summary['charger'] == 'dock' and len(summary['waypoints']) == 5
    site_dir = os.path.join(manager.sites_dir, 'lab')
    assert sorted(os.listdir(site_dir)) == [
        'fleet_lab.yaml', 'import.json', 'lab.building.yaml', 'lab.png', 'nav_graphs',
        'preview.png']
    with open(os.path.join(site_dir, 'fleet_lab.yaml')) as f:
        config = yaml.safe_load(f)
    assert config['vda5050']['broker'] == {'host': '192.168.1.201', 'port': 1883,
                                           'username': '', 'password': ''}
    lab = next(s for s in manager.sites() if s['name'] == 'lab')
    assert not lab['builtin'] and lab['charger'] == 'dock' and lab['import']['options'][
        'clearance'] == 0.3


def test_import_errors_are_request_errors(manager):
    with pytest.raises(server.MapImportError, match="no place named 'charger'"):
        manager.import_map('ardennes', {'name': 'lab', 'charger': 'charger', 'clearance': 0.3})
    with pytest.raises(server.MapImportError, match="no map named 'nope'"):
        manager.import_map('ardennes', {'name': 'nope'})
    with pytest.raises(server.RequestError, match='not a map name'):
        manager.import_map('ardennes', {'name': '../etc'})
    with pytest.raises(server.RequestError, match='Gazebo site'):
        manager.import_map('ardennes', {'name': 'isaacbot_amr_world'})
    with pytest.raises(server.RequestError, match='must be > 0'):
        manager.import_map('ardennes', {'name': 'lab', 'charger': 'dock', 'clearance': 0})


def test_failed_reimport_keeps_the_previous_site(manager):
    manager.import_map('ardennes', {'name': 'lab', 'charger': 'dock', 'clearance': 0.3})
    manager.nav_graph_cmd = FAILING_NAV_GRAPH
    with pytest.raises(server.MapImportError, match='no lanes'):
        manager.import_map('ardennes', {'name': 'lab', 'charger': 'dock', 'clearance': 0.3})
    site_dir = os.path.join(manager.sites_dir, 'lab')
    assert os.path.isfile(os.path.join(site_dir, 'nav_graphs', '0.yaml'))
    assert not any(n.startswith('.') for n in os.listdir(manager.sites_dir) if n != 'lab')


def test_activate_and_delete(manager):
    manager.import_map('ardennes', {'name': 'lab', 'charger': 'dock', 'clearance': 0.3})
    assert manager.activate({'name': 'lab'}) == {'active': 'lab'}
    with open(os.path.join(manager.sites_dir, 'active')) as f:
        assert f.read() == 'lab\n'
    with pytest.raises(server.RequestError, match='RMF runs'):
        manager.delete('lab')
    manager.activate({'name': 'isaacbot_amr_world'})
    with pytest.raises(server.RequestError, match='not an imported site'):
        manager.delete('isaacbot_amr_world')
    assert manager.delete('lab') == {'deleted': 'lab'}
    with pytest.raises(server.RequestError, match='import it first'):
        manager.activate({'name': 'lab'})


def test_robot_access_is_checked_at_most_every_15_s(manager):
    calls = []
    manager.test_source.check = lambda: calls.append(1) or ('ok', '')
    robot = manager._robot('ardennes')
    manager.access(robot)
    manager.access(robot)
    manager.clock.now += 16
    assert manager.access(robot)['state'] == 'ok'
    assert len(calls) == 2
    # A new address is checked at once.
    manager.update_robot('ardennes', {'ssh': 'root@10.0.0.9'})
    manager.access(manager._robot('ardennes'))
    assert len(calls) == 3


@pytest.fixture
def http(manager):
    httpd = ThreadingHTTPServer(('127.0.0.1', 0), server.make_handler(
        manager, os.path.join(REPO, 'site_manager', 'static')))
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    yield f'http://127.0.0.1:{httpd.server_address[1]}'
    httpd.shutdown()


def request(url, method='GET', body=None):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data, method=method,
                                 headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req) as res:
            return res.status, res.headers.get('Content-Type'), res.read()
    except urllib.error.HTTPError as e:
        return e.code, e.headers.get('Content-Type'), e.read()


def test_http_end_to_end(http):
    code, kind, body = request(f'{http}/')
    assert code == 200 and kind.startswith('text/html') and b'Isaacbot Fleets' in body

    code, kind, body = request(f'{http}/robots/ardennes')
    assert code == 200 and kind.startswith('text/html') and b'Maps on this robot' in body
    assert b'id="sites"' not in body                 # the sites have their own page
    code, kind, body = request(f'{http}/sites')
    assert code == 200 and kind.startswith('text/html') and b'id="sites"' in body
    assert b'<h1>RMF sites</h1>' in body
    code, kind, body = request(f'{http}/common.js')
    assert code == 200 and kind.startswith('text/javascript') and b'function api' in body
    assert request(f'{http}/common.css')[0] == 200
    for logo in ('isaacbot-lockup-black-bold.svg', 'isaacbot-lockup-white.svg'):
        code, kind, body = request(f'{http}/{logo}')
        assert code == 200 and kind == 'image/svg+xml' and body.startswith(b'<svg')

    code, _, body = request(f'{http}/api/robots/ardennes')
    robot = json.loads(body)
    assert code == 200 and robot['access']['state'] == 'ok'
    assert robot['authorize'] == ('docker compose -f docker/docker-compose.yml exec site-manager '
                                  'isaacbot-authorize root@192.168.1.201 24')

    code, _, body = request(f'{http}/api/robots/ardennes/maps')
    assert code == 200 and json.loads(body)[0]['name'] == 'lab'

    code, _, body = request(f'{http}/api/robots/ardennes/import', 'POST',
                            {'name': 'lab', 'clearance': 0.3})
    assert code == 200 and json.loads(body)['charger'] == 'dock'
    assert json.loads(body)['robot'] == 'ardennes'

    code, kind, body = request(f'{http}/sites/lab/preview.png')
    assert code == 200 and kind == 'image/png' and body[:4] == b'\x89PNG'
    code, _, _ = request(f'{http}/sites/isaacbot_amr_world/floorplan.png')
    assert code == 200

    code, _, body = request(f'{http}/api/robots/ardennes/import', 'POST',
                            {'name': 'lab', 'charger': 'x'})
    assert code == 400 and "no place named 'x'" in json.loads(body)['error']

    assert request(f'{http}/api/activate', 'POST', {'name': 'lab'})[0] == 200
    status = json.loads(request(f'{http}/api/status')[2])
    assert status['active'] == 'lab' and 'access' not in status
    assert {s['name'] for s in status['sites']} >= {'lab', 'isaacbot_amr_world'}

    assert request(f'{http}/api/sites/lab', 'DELETE')[0] == 400      # active
    code, kind, body = request(f'{http}/isaacbot-mark-white.svg')
    assert code == 200 and kind == 'image/svg+xml' and body.startswith(b'<svg')
    assert request(f'{http}/server.py')[0] == 404
    assert request(f'{http}/sites/..%2F..%2Fetc/preview.png')[0] == 404
    assert request(f'{http}/api/nothing')[0] == 404
    # The single-robot routes are gone: maps and imports are per robot.
    assert request(f'{http}/api/isaacbot/maps')[0] == 404
    assert request(f'{http}/api/import', 'POST', {'name': 'lab'})[0] == 404
    assert request(f'{http}/robots/ardennes/../../server.py')[0] == 404


# --- robots ---------------------------------------------------------------------------------------

LIVE = {'robots': [{'name': 'ardennes', 'connection': 'ONLINE', 'operating_mode': 'AUTOMATIC',
                    'in_fleet': True, 'blocked_reason': None, 'localized': True,
                    'battery_soc': 0.8}]}


@pytest.fixture
def robot_manager(manager, tmp_path):
    auth_dir = tmp_path / 'broker'
    auth_dir.mkdir()
    (auth_dir / 'passwd').write_text('rmf:$7$101$aaa$bbb\nardennes:$7$101$ccc$ddd\n')
    manager.broker_auth = server.fleet_robots.BrokerAuth(str(auth_dir), owner_uid=os.getuid())
    manager.robot_broker = ('10.8.0.1', 1883)
    manager.fleet_state = lambda: LIVE
    return manager


def test_robots_are_seeded_from_the_fleet_template(robot_manager):
    data = robot_manager.robots()
    assert [r['name'] for r in data['robots']] == ['ardennes']
    ardennes = data['robots'][0]
    assert ardennes['serial_number'] == 'ardennis-3420987-k' and ardennes['broker_login'] == 'ardennes'
    assert ardennes['live']['battery_soc'] == 0.8
    assert data['broker_auth'] and data['broker'] == '10.8.0.1:1883'
    assert data['site'] == 'isaacbot_amr_world' and 'ardennes_charger' in data['waypoints']


def test_add_rotate_and_delete_a_robot(robot_manager):
    auth = robot_manager.broker_auth
    added = robot_manager.add_robot({'name': 'ardennes_e', 'serial_number': 'ardennis-3420987-e'})
    password = added['password']
    assert len(password) == 32 and added['robot']['login'] == 'ardennes_e'
    variables = added['balena']
    assert variables['ISAACBOT_AMR_VDA5050_BROKER_HOST'] == '10.8.0.1'
    assert variables['ISAACBOT_AMR_VDA5050_BROKER_USER'] == 'ardennes_e'
    assert variables['ISAACBOT_AMR_VDA5050_BROKER_PASSWORD'] == password
    assert variables['ISAACBOT_AMR_VDA5050_SERIAL_NUMBER'] == 'ardennis-3420987-e'
    assert variables['ISAACBOT_AMR_VDA5050_MANUFACTURER'] == 'IsaacBot'

    line = next(li for li in open(auth.passwd_path).read().splitlines() if li.startswith('ardennes_e:'))
    assert server.fleet_robots.check_password(password, line.split(':', 1)[1])
    assert auth.logins() == ['rmf', 'ardennes', 'ardennes_e']      # existing logins untouched
    acl = open(auth.acl_path).read()
    assert 'user ardennes_e\ntopic readwrite uagv/v2/IsaacBot/ardennis-3420987-e/#' in acl
    assert 'user ardennes\ntopic readwrite uagv/v2/IsaacBot/ardennis-3420987-k/#' in acl
    assert [r.name for r in server.fleet_robots.load(robot_manager.robots_path)] == [
        'ardennes', 'ardennes_e']

    rotated = robot_manager.rotate_password('ardennes_e')['password']
    assert rotated != password and auth.logins() == ['rmf', 'ardennes', 'ardennes_e']

    assert robot_manager.delete_robot('ardennes_e') == {'deleted': 'ardennes_e'}
    assert auth.logins() == ['rmf', 'ardennes'] and 'ardennes_e' not in open(auth.acl_path).read()
    with pytest.raises(server.RequestError, match='at least one robot'):
        robot_manager.delete_robot('ardennes')


def test_add_robot_errors(robot_manager):
    with pytest.raises(server.RobotConflict, match='already uses uagv'):
        robot_manager.add_robot({'name': 'twin', 'serial_number': 'ardennis-3420987-k'})
    # A login the broker has but robots.json does not (made by hand): not taken over.
    with open(robot_manager.broker_auth.passwd_path, 'a') as f:
        f.write('ghost:$7$101$a$b\n')
    with pytest.raises(server.RobotConflict, match='broker already has a login'):
        robot_manager.add_robot({'name': 'ghost', 'serial_number': 'g'})
    with pytest.raises(server.RobotError, match='reserved'):
        robot_manager.add_robot({'name': 'rmf', 'serial_number': 's'})
    with pytest.raises(server.RobotError, match='not a waypoint of the active site'):
        robot_manager.add_robot({'name': 'b', 'serial_number': 'b', 'charger': 'nowhere'})
    with pytest.raises(server.RobotNotFound):
        robot_manager.rotate_password('nobody')


def test_robots_without_broker_logins(manager):
    """Local compose: anonymous broker, so a robot gets an identity but no login."""
    added = manager.add_robot({'name': 'ardennes_e', 'serial_number': 'ardennis-3420987-e',
                               'charger': 'ardennes_charger'})
    assert added['password'] is None
    assert 'ISAACBOT_AMR_VDA5050_BROKER_USER' not in added['balena']
    assert manager.robots()['robots'][1]['charger'] == 'ardennes_charger'
    with pytest.raises(server.RequestError, match='no logins'):
        manager.rotate_password('ardennes_e')


def test_robots_list_survives_a_dead_fleet_api(robot_manager):
    def down():
        raise OSError('connection refused')
    robot_manager.fleet_state = down
    assert robot_manager.robots()['robots'][0]['live'] is None


def test_network(robot_manager, tmp_path):
    log = tmp_path / 'broker.log'
    log.write_text(
        "100: New client connected from 10.8.0.5:59127 as auto-1 (p4, c1, k60, u'ardennes').\n"
        '101: New client connected from 172.18.0.2:5000 as rmf-fleet-ardennes '
        "(p4, c1, k30, u'rmf').\n")
    probed = []
    robot_manager.broker_log = server.network.BrokerLog(str(log))
    robot_manager.host_info = {'hostname': 'isaacbot-server-001', 'vpn_ip': '10.8.0.1',
                               'lan_ip': '192.168.100.12'}
    robot_manager.probe = lambda host, port: probed.append((host, port)) or host != 'rmf'
    net = robot_manager.network()
    assert net['server']['hostname'] == 'isaacbot-server-001'
    services = {s['name']: s for s in net['server']['services']}
    assert services['Isaacbot website']['up'] and services['Site manager']['up']
    assert not services['Fleet control API']['up'] and services['RMF API server']['port'] == 8010
    assert ('10.8.0.1', 80) in probed and ('', 8020) not in probed
    ardennes = net['robots'][0]
    assert ardennes['session']['ip'] == '10.8.0.5' and ardennes['session']['online']
    assert ardennes['live']['connection'] == 'ONLINE'
    assert [o['login'] for o in net['others']] == ['rmf']
    assert net['broker'] == {'address': '10.8.0.1:1883', 'auth': True, 'log': True}
    assert 'onboard_lan' not in net
    page = robot_manager.robot('ardennes')
    assert page['session']['ip'] == '10.8.0.5' and page['broker_log']
    assert page['onboard_lan'][0]['ip'] == '192.168.1.1'
    # Probes are cached for 10 s.
    n = len(probed)
    robot_manager.network()
    assert len(probed) == n


def test_network_without_broker_log(manager):
    manager.probe = lambda host, port: False
    net = manager.network()
    assert net['robots'][0]['session'] is None and net['others'] == []
    assert net['broker']['log'] is False and net['server']['services']


def test_http_robots(robot_manager):
    httpd = ThreadingHTTPServer(('127.0.0.1', 0), server.make_handler(
        robot_manager, os.path.join(REPO, 'site_manager', 'static')))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    url = f'http://127.0.0.1:{httpd.server_address[1]}'
    try:
        code, _, body = request(f'{url}/api/robots')
        assert code == 200 and json.loads(body)['robots'][0]['name'] == 'ardennes'
        robot_manager.probe = lambda host, port: False
        code, _, body = request(f'{url}/api/network')
        assert code == 200 and json.loads(body)['robots'][0]['name'] == 'ardennes'
        body = {'name': 'ardennes_e', 'serial_number': 'ardennis-3420987-e'}
        code, _, data = request(f'{url}/api/robots', 'POST', body)
        assert code == 200 and json.loads(data)['password']
        assert request(f'{url}/api/robots', 'POST', body)[0] == 409
        assert request(f'{url}/api/robots', 'POST', {'name': '9'})[0] == 400
        assert request(f'{url}/api/robots/ardennes_e/password', 'POST', {})[0] == 200
        assert request(f'{url}/api/robots/nobody/password', 'POST', {})[0] == 404
        assert request(f'{url}/api/robots/ardennes_e', 'DELETE')[0] == 200
        assert request(f'{url}/api/robots/ardennes_e', 'DELETE')[0] == 404
    finally:
        httpd.shutdown()


def test_http_basic_auth(manager):
    httpd = ThreadingHTTPServer(('127.0.0.1', 0), server.make_handler(
        manager, os.path.join(REPO, 'site_manager', 'static'), password='s3cret'))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    url = f'http://127.0.0.1:{httpd.server_address[1]}'

    def get(path, auth=None):
        req = urllib.request.Request(url + path)
        if auth:
            req.add_header('Authorization', 'Basic ' + base64.b64encode(auth.encode()).decode())
        try:
            with urllib.request.urlopen(req) as res:
                return res.status, res.headers
        except urllib.error.HTTPError as e:
            return e.code, e.headers
    try:
        code, headers = get('/')
        assert code == 401 and headers['WWW-Authenticate'].startswith('Basic')
        assert get('/api/robots', 'admin:wrong')[0] == 401
        assert get('/api/robots', 'admin:s3cret')[0] == 200
        assert get('/', 'anyone:s3cret')[0] == 200
        req = urllib.request.Request(url + '/api/robots', data=b'{}', method='POST')
        with pytest.raises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(req)
        assert e.value.code == 401
    finally:
        httpd.shutdown()


# --- one robot's page ------------------------------------------------------------------------------

def rmf_fingerprint(path):
    """What run_rmf.sh restarts RMF on: robots.json without its "ssh" lines."""
    with open(path) as f:
        return [line for line in f if '"ssh"' not in line]


def test_robot_page_data(manager):
    robot = manager.robot('ardennes')
    assert robot['ssh'] == 'root@192.168.1.201:24' and robot['access']['state'] == 'ok'
    assert robot['site'] == 'isaacbot_amr_world' and 'ardennes_charger' in robot['waypoints']
    assert robot['authorize'].endswith('isaacbot-authorize root@192.168.1.201 24')
    with pytest.raises(server.RobotNotFound):
        manager.robot('nobody')


def test_update_ssh_does_not_restart_rmf_but_charger_does(manager):
    before = rmf_fingerprint(manager.robots_path)
    assert manager.update_robot('ardennes', {'ssh': 'root@10.0.0.9:2222'})['ssh'] == \
        'root@10.0.0.9:2222'
    assert manager._robot('ardennes').ssh_target() == ('root@10.0.0.9', 2222)
    assert rmf_fingerprint(manager.robots_path) == before
    manager.update_robot('ardennes', {'charger': 'south'})
    assert rmf_fingerprint(manager.robots_path) != before
    with pytest.raises(server.RobotError, match='not an ssh address'):
        manager.update_robot('ardennes', {'ssh': 'no-user-here'})
    with pytest.raises(server.RobotError, match='not a waypoint'):
        manager.update_robot('ardennes', {'charger': 'nowhere'})
    with pytest.raises(server.RequestError, match='nothing to change'):
        manager.update_robot('ardennes', {'serial_number': 'x'})


def test_ssh_address_must_be_unique(manager):
    """Every isaacbot's LAN is the same: one ssh address reaches one isaacbot only."""
    with pytest.raises(server.RobotConflict, match="'ardennes' already uses the ssh address"):
        manager.add_robot({'name': 'ardennes_e', 'serial_number': 'e',
                           'ssh': 'admin@192.168.1.201:24'})
    manager.add_robot({'name': 'ardennes_e', 'serial_number': 'e', 'ssh': 'root@10.8.0.6:24'})
    with pytest.raises(server.RobotConflict, match='192.168.1.201:24'):
        manager.update_robot('ardennes_e', {'ssh': 'root@192.168.1.201'})   # port 24 default
    manager.update_robot('ardennes_e', {'ssh': 'root@192.168.1.201:2222'})  # another sshd
    manager.update_robot('ardennes_e', {'ssh': ''})
    manager.update_robot('ardennes', {'ssh': 'root@10.8.0.5:24'})           # its own again
    # A list saved before the check: both pages say so.
    robots = server.fleet_robots
    robots.save(manager.robots_path, [replace_ssh(r, 'root@192.168.1.201:24')
                                      for r in manager._robot_list()])
    assert manager.robot('ardennes')['ssh_shared_with'] == ['ardennes_e']
    assert manager.robot('ardennes_e')['ssh_shared_with'] == ['ardennes']


def replace_ssh(robot, ssh):
    return server.replace(robot, ssh=ssh)


def test_robot_without_ssh_has_no_maps(manager):
    manager.add_robot({'name': 'sim', 'serial_number': 'sim-1'})
    assert manager.robot('sim')['access']['state'] == 'no_ssh'
    assert manager.robot('sim')['authorize'] == ''
    with pytest.raises(server.RequestError, match='no ssh address'):
        manager.robot_maps('sim')
    with pytest.raises(server.RequestError, match='no ssh address'):
        manager.import_map('sim', {'name': 'lab', 'charger': 'dock'})


def test_http_patch_robot(http):
    code, _, body = request(f'{http}/api/robots/ardennes', 'PATCH', {'ssh': 'root@10.0.0.9'})
    assert code == 200 and json.loads(body)['ssh'] == 'root@10.0.0.9'
    assert request(f'{http}/api/robots/ardennes', 'PATCH', {'ssh': 'bad'})[0] == 400
    assert request(f'{http}/api/robots/nobody', 'PATCH', {'ssh': 'root@h'})[0] == 404
