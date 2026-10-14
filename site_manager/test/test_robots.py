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
import hashlib
import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(__file__)
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'site_manager'))

import robots  # noqa: E402
from robots import Robot  # noqa: E402
import yaml  # noqa: E402

TEMPLATE = os.path.join(REPO, 'isaacbot_amr_rmf_bringup', 'config', 'fleet_isaacbot_amr_world.yaml')
WORLD = os.path.join(REPO, 'isaacbot_amr_rmf_maps', 'maps', 'isaacbot_amr_world',
                     'isaacbot_amr_world.building.yaml')
K = Robot('ardennes', 'ardennis-3420987-k')
E = Robot('ardennes_e', 'ardennis-3420987-e', charger='parking')


def template() -> dict:
    with open(TEMPLATE) as f:
        return yaml.safe_load(f)


@pytest.mark.parametrize('robot, message', [
    (Robot('1st', 'x'), 'not a robot name'),
    (Robot('a-b', 'x'), 'not a robot name'),
    (Robot('ok', 'has space'), 'serial number'),
    (Robot('ok', 'a/b'), 'serial number'),
    (Robot('ok', 'sn', manufacturer='Isaac#Bot'), 'manufacturer'),
    (Robot('ok', 'sn', login='a b'), 'broker login'),
    (Robot('ok', 'sn', login='rmf'), 'reserved'),
])
def test_validate_rejects(robot, message):
    with pytest.raises(robots.RobotError, match=message):
        robots.validate(robot, reserved_logins=('rmf',))


def test_validate_accepts_vda_serials():
    for serial in ('ardennis-3420987-e', 'ARDENNIS-280287D', 'a.b:c_1'):
        assert robots.validate(Robot('ardennes_e', serial)).serial_number == serial
    assert K.broker_login == 'ardennes' and Robot('x', 's', login='y').broker_login == 'y'
    assert E.topic_prefix == 'uagv/v2/IsaacBot/ardennis-3420987-e'


def test_check_unique():
    for clash in (Robot('ardennes', 'other'), Robot('b', 'ardennis-3420987-k'),
                  Robot('b', 'other', login='ardennes')):
        with pytest.raises(robots.RobotConflict):
            robots.check_unique([K], clash)
    robots.check_unique([K], E)
    # Same serial, other manufacturer: other topics, no clash.
    robots.check_unique([K], Robot('b', 'ardennis-3420987-k', manufacturer='Other'))


def test_robots_json_round_trip_and_seed(tmp_path):
    path = str(tmp_path / 'robots.json')
    assert robots.load(path) is None
    seeded = robots.from_fleet_config(template())
    assert seeded == [Robot('ardennes', 'ardennis-3420987-k', 'IsaacBot')]
    robots.save(path, seeded + [E])
    assert robots.load(path) == seeded + [E]


def test_apply_to_fleet_sets_exactly_the_robots():
    config = robots.apply_to_fleet(template(), [K, E], waypoints=['ardennes_charger', 'parking'])
    fleet, vda = config['rmf_fleet']['robots'], config['vda5050']['robots']
    assert list(fleet) == ['ardennes', 'ardennes_e'] and list(vda) == list(fleet)
    assert fleet['ardennes']['charger'] == 'ardennes_charger'      # site charger
    assert fleet['ardennes_e']['charger'] == 'parking'
    assert vda['ardennes_e'] == {'serial_number': 'ardennis-3420987-e', 'manufacturer': 'IsaacBot'}
    # A charger this site does not have falls back to the site's.
    config = robots.apply_to_fleet(template(), [E], waypoints=['ardennes_charger'])
    assert config['rmf_fleet']['robots'] == {'ardennes_e': {'charger': 'ardennes_charger'}}
    # The rest of the fleet config is untouched.
    assert config['rmf_fleet']['limits'] == template()['rmf_fleet']['limits']


def test_hash_password_is_mosquitto_7_format():
    salt = bytes(range(12))
    hashed = robots.hash_password('secret', salt)
    _, kind, iterations, b64salt, b64digest = hashed.split('$')
    assert (kind, iterations) == ('7', '101') and base64.b64decode(b64salt) == salt
    assert base64.b64decode(b64digest) == hashlib.pbkdf2_hmac('sha512', b'secret', salt, 101)
    assert robots.check_password('secret', robots.hash_password('secret'))
    assert not robots.check_password('wrong', hashed)
    assert robots.hash_password('secret') != robots.hash_password('secret')    # random salt


def test_passwd_set_keeps_other_logins():
    text = 'rmf:$7$101$aaa$bbb\nardennes:$7$101$ccc$ddd\n'
    text = robots.passwd_set(text, 'ardennes_e', 'pw')
    assert robots.passwd_logins(text) == ['rmf', 'ardennes', 'ardennes_e']
    assert text.startswith('rmf:$7$101$aaa$bbb\nardennes:$7$101$ccc$ddd\n')
    text = robots.passwd_set(text, 'ardennes', 'new')                 # rotate
    assert robots.passwd_logins(text) == ['rmf', 'ardennes_e', 'ardennes']
    line = text.splitlines()[-1]
    assert robots.check_password('new', line.split(':', 1)[1])
    text = robots.passwd_set(text, 'ardennes_e', None)               # remove
    assert robots.passwd_logins(text) == ['rmf', 'ardennes']


def test_render_acl():
    acl = robots.render_acl([K, E])
    assert 'user rmf\ntopic readwrite uagv/#\n' in acl
    assert 'user ardennes\ntopic readwrite uagv/v2/IsaacBot/ardennis-3420987-k/#\n' in acl
    assert 'user ardennes_e\ntopic readwrite uagv/v2/IsaacBot/ardennis-3420987-e/#\n' in acl


def test_broker_auth_files(tmp_path):
    auth = robots.BrokerAuth(str(tmp_path), owner_uid=os.getuid())
    (tmp_path / 'passwd').write_text('rmf:$7$101$aaa$bbb\n')
    assert auth.ready() and auth.logins() == ['rmf']
    auth.set_password('ardennes_e', 'pw')
    auth.write_acl([K, E])
    assert auth.logins() == ['rmf', 'ardennes_e']
    assert oct(os.stat(auth.passwd_path).st_mode & 0o777) == '0o600'
    assert 'uagv/v2/IsaacBot/ardennis-3420987-e/#' in (tmp_path / 'acl').read_text()
    with pytest.raises(robots.RobotError, match='fleet adapter login'):
        auth.set_password('rmf', 'x')


def test_merge_cli(tmp_path):
    robots_json = str(tmp_path / 'robots.json')
    robots.save(robots_json, [K, Robot('ardennes_e', 'ardennis-3420987-e', charger='nowhere')])
    out = str(tmp_path / 'effective' / 'fleet.yaml')
    result = subprocess.run(
        [sys.executable, os.path.join(REPO, 'site_manager', 'robots.py'), 'merge', TEMPLATE,
         robots_json, out, WORLD], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert "charger 'nowhere' is not in this site" in result.stderr
    with open(out) as f:
        config = yaml.safe_load(f)
    assert set(config['rmf_fleet']['robots']) == {'ardennes', 'ardennes_e'}
    assert config['rmf_fleet']['robots']['ardennes_e']['charger'] == 'ardennes_charger'
    assert config['vda5050']['robots']['ardennes_e']['serial_number'] == 'ardennis-3420987-e'


def test_site_waypoints():
    names = robots.site_waypoints(WORLD)
    assert 'ardennes_charger' in names and len(names) == 9
    assert robots.site_waypoints('/nonexistent.yaml') is None


def test_ssh_address():
    assert Robot('a', 's', ssh='root@192.168.1.201').ssh_target() == ('root@192.168.1.201', 24)
    assert Robot('a', 's', ssh='root@10.0.0.2:2222').ssh_target() == ('root@10.0.0.2', 2222)
    assert Robot('a', 's').ssh_target() is None
    for bad in ('192.168.1.201', 'root@', 'root@host:', 'root@host:99999', 'a b@host'):
        with pytest.raises(robots.RobotError, match='ssh address'):
            robots.validate(Robot('a', 's', ssh=bad))


def test_robots_json_one_field_per_line_and_old_files_load(tmp_path):
    """run_rmf.sh drops the "ssh" lines to fingerprint robots.json: one field per line."""
    path = str(tmp_path / 'robots.json')
    robots.save(path, [Robot('a', 's', ssh='root@h:24')])
    ssh_lines = [line for line in open(path) if '"ssh"' in line]
    assert ssh_lines == ['   "ssh": "root@h:24"\n']
    (tmp_path / 'old.json').write_text('{"robots": [{"name": "a", "serial_number": "s"}]}')
    assert robots.load(str(tmp_path / 'old.json')) == [Robot('a', 's')]


def test_merge_ignores_ssh():
    config = robots.apply_to_fleet(template(), [Robot('ardennes', 'k', ssh='root@h')])
    assert 'ssh' not in yaml.safe_dump(config)
