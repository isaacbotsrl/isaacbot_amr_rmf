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

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from network import BrokerLog, BrokerSessions  # noqa: E402

# Lines as the server's Mosquitto 2 writes them.
ROBOT_IN = ('1792146382: New client connected from 10.8.0.5:59127 as auto-AE0B5D08 '
            "(p4, c1, k60, u'ardennes_001').")
RMF_IN = ('1792147288: New client connected from 172.18.0.2:50407 as '
          "rmf-isaacbot_amr_fleet-ardennes_002 (p4, c1, k30, u'rmf').")


def test_connected_robot():
    s = BrokerSessions().feed_all([
        '1792146382: New connection from 10.8.0.5:59127 on port 1883.', ROBOT_IN])
    robot = s.for_login('ardennes_001')
    assert (robot.ip, robot.port, robot.client_id) == ('10.8.0.5', 59127, 'auto-AE0B5D08')
    assert robot.online and robot.since == robot.last_seen == 1792146382
    assert s.for_login('ardennes_002') is None


def test_each_disconnect_form():
    for line in ('1792146400: Client auto-AE0B5D08 [10.8.0.5:59127] disconnected: '
                 'connection closed by client.',
                 '1792146400: Client auto-AE0B5D08 disconnected.',
                 '1792146400: Client auto-AE0B5D08 closed its connection.',
                 '1792146400: Client auto-AE0B5D08 has exceeded timeout, disconnecting.'):
        robot = BrokerSessions().feed_all([ROBOT_IN, line]).for_login('ardennes_001')
        assert not robot.online and robot.last_seen == 1792146400 and robot.ip == '10.8.0.5'


def test_reconnect_from_a_new_address_wins():
    s = BrokerSessions().feed_all([
        ROBOT_IN, '1792146400: Client auto-AE0B5D08 disconnected.',
        '1792146500: New client connected from 10.8.0.9:4000 as auto-FFFF (p4, c1, k60, '
        "u'ardennes_001')."])
    robot = s.for_login('ardennes_001')
    assert robot.online and robot.ip == '10.8.0.9' and robot.client_id == 'auto-FFFF'


def test_online_session_beats_a_newer_offline_one():
    s = BrokerSessions().feed_all([
        ROBOT_IN,
        '1792146500: New client connected from 10.8.0.9:4000 as other (p4, c1, k60, '
        "u'ardennes_001').",
        '1792146600: Client other disconnected.'])
    assert s.for_login('ardennes_001').client_id == 'auto-AE0B5D08'


def test_broker_restart_marks_everyone_offline():
    s = BrokerSessions().feed_all([
        ROBOT_IN, RMF_IN, '1792147300: mosquitto version 2.0.18 starting'])
    assert not s.for_login('ardennes_001').online and not s.for_login('rmf').online
    assert s.for_login('rmf').last_seen == 1792147300


def test_ipv6_anonymous_and_unknown_lines():
    s = BrokerSessions().feed_all([
        '1792146382: New client connected from [fd00::5]:1234 as anon (p2, c1, k60).',
        '1792146383: Saving in-memory database to /mosquitto/data//mosquitto.db.',
        'garbage', ''])
    anon = s.for_login('')
    assert anon.ip == 'fd00::5' and anon.port == 1234


def test_others_are_the_non_robot_logins():
    s = BrokerSessions().feed_all([ROBOT_IN, RMF_IN])
    assert [o.login for o in s.others(['ardennes_001'])] == ['rmf']


def test_log_is_read_incrementally_and_across_rotation(tmp_path):
    path = tmp_path / 'broker.log'
    path.write_text(ROBOT_IN + '\n' + RMF_IN[:30])      # Mosquitto is mid-line
    log = BrokerLog(str(path))
    assert log.available()
    assert log.read().for_login('ardennes_001').online
    assert log.read().for_login('rmf') is None
    with open(path, 'a') as f:
        f.write(RMF_IN[30:] + '\n')
    assert log.read().for_login('rmf').online

    # Rotated: the old file is broker.log.1, the new one starts empty.
    os.rename(path, str(path) + '.1')
    path.write_text('1792146400: Client auto-AE0B5D08 disconnected.\n')
    sessions = log.read()
    assert not sessions.for_login('ardennes_001').online and sessions.for_login('rmf').online

    # Truncated in place: start over.
    path.write_text('')
    os.remove(str(path) + '.1')
    assert log.read().for_login('ardennes_001') is None


def test_missing_log(tmp_path):
    log = BrokerLog(str(tmp_path / 'nope.log'))
    assert not log.available() and log.read().for_login('ardennes_001') is None
