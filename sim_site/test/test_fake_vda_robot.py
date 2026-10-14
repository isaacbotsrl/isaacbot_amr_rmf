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
The fake robot against the fleet adapter's own VDA 5050 code: orders and instant actions are
built with its vda_messages, the robot's state is read back with its parse_state.
"""

import os
import sys

HERE = os.path.dirname(__file__)
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'sim_site'))
sys.path.insert(0, os.path.join(REPO, 'isaacbot_amr_rmf_fleet_adapter'))

from fake_vda_robot import FakeRobot  # noqa: E402
from isaacbot_amr_rmf_fleet_adapter.domain.model import Pose2D  # noqa: E402
from isaacbot_amr_rmf_fleet_adapter.domain.robot_status import parse_state  # noqa: E402
from isaacbot_amr_rmf_fleet_adapter.domain.vda_messages import (  # noqa: E402
    action, cancel_order_action, instant_actions_body, NavigationOrder, set_drive_mode_action)


def order(order_id='o1', goal=(2.0, 0.0)):
    return NavigationOrder(order_id, Pose2D(0.0, 0.0, 0.0), Pose2D(goal[0], goal[1], 1.0), 'L1')


def run(robot, seconds, dt=0.1):
    for _ in range(int(seconds / dt)):
        robot.step(dt)


def test_drives_an_rmf_order_to_its_goal():
    robot, nav = FakeRobot(speed=1.0), order()
    robot.on_order(nav.body())
    status = parse_state(robot.state())
    assert status.order_id == 'o1' and status.has_active_order
    assert status.last_node_id == nav.start_node_id and status.pose == Pose2D(0.0, 0.0, 0.0)
    run(robot, 1.0)
    status = parse_state(robot.state())
    assert status.driving and 0.8 < status.pose.x < 1.2 and status.has_active_order
    run(robot, 1.5)
    status = parse_state(robot.state())
    assert not status.has_active_order and not status.driving
    assert status.last_node_id == nav.goal_node_id
    assert (status.pose.x, status.pose.y, status.pose.theta) == (2.0, 0.0, 1.0)


def test_rejects_a_new_order_while_one_runs_and_cancels():
    robot = FakeRobot(speed=1.0)
    robot.on_order(order('o1').body())
    robot.on_order(order('o2').body())
    status = parse_state(robot.state())
    assert status.order_id == 'o1'
    assert status.errors_for_order('o2') and not status.errors_for_order('o1')

    cancel = cancel_order_action()
    robot.on_instant_actions(instant_actions_body([cancel]))
    status = parse_state(robot.state())
    assert not status.has_active_order and status.action_state(cancel['actionId']).finished
    robot.on_order(order('o2').body())
    status = parse_state(robot.state())
    assert status.order_id == 'o2' and status.has_active_order and not status.errors


def test_pause_stops_and_resumes():
    robot = FakeRobot(speed=1.0)
    robot.on_order(order().body())
    robot.on_instant_actions(instant_actions_body([action('startPause')]))
    run(robot, 1.0)
    status = parse_state(robot.state())
    assert status.paused and not status.driving and status.pose.x == 0.0
    robot.on_instant_actions(instant_actions_body([action('stopPause')]))
    run(robot, 3.0)
    assert parse_state(robot.state()).last_node_id == 'o1-goal'


def test_drive_mode_takes_the_robot_out_of_the_fleet():
    robot = FakeRobot()
    manual = set_drive_mode_action('MANUAL')
    robot.on_instant_actions(instant_actions_body([manual]))
    status = parse_state(robot.state())
    assert status.operating_mode == 'MANUAL' and status.blocked_reason()
    assert status.action_state(manual['actionId']).status == 'FINISHED'
    robot.on_order(order('o3').body())
    assert parse_state(robot.state()).errors_for_order('o3')
    robot.on_instant_actions(instant_actions_body([set_drive_mode_action('AUTOMATIC')]))
    assert parse_state(robot.state()).blocked_reason() is None


def test_order_update_is_stitched():
    robot = FakeRobot(speed=1.0)
    first = order('o1', goal=(1.0, 0.0)).body()
    robot.on_order(first)
    goal = first['nodes'][1]
    update = {'orderId': 'o1', 'orderUpdateId': 1,
              'nodes': [goal, {'nodeId': 'n3', 'sequenceId': 4, 'released': True,
                               'nodePosition': {'x': 1.0, 'y': 1.0, 'mapId': 'L1'}}],
              'edges': [{'edgeId': 'e3', 'sequenceId': 3, 'released': True,
                         'startNodeId': goal['nodeId'], 'endNodeId': 'n3'}]}
    robot.on_order(update)
    assert [n['nodeId'] for n in robot.state()['nodeStates']] == ['o1-goal', 'n3']
    run(robot, 3.0)
    status = parse_state(robot.state())
    assert status.last_node_id == 'n3' and not status.has_active_order


def test_factsheet_request_and_unknown_actions():
    robot = FakeRobot()
    assert robot.on_instant_actions(instant_actions_body([action('factsheetRequest')])) == {
        'state', 'factsheet'}
    unknown = action('pick')
    robot.on_instant_actions(instant_actions_body([unknown]))
    assert parse_state(robot.state()).action_state(unknown['actionId']).status == 'FAILED'
