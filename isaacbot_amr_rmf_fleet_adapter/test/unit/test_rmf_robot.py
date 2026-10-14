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

"""RmfRobot against a stand-in for RMF's EasyFullControl binding (not installed on the host)."""

import sys
import types

from isaacbot_amr_rmf_fleet_adapter.application.robot_session import RmfRobotState

# rmf_robot imports rmf_adapter.easy_full_control: a stand-in when RMF is not installed (the
# host), and the test uses the stand-in either way (the image build has the real binding).
easy = types.ModuleType('rmf_adapter.easy_full_control')
easy.RobotState = lambda map_name, position, battery: (map_name, position, battery)
easy.RobotCallbacks = lambda *args: args
try:
    import rmf_adapter.easy_full_control  # noqa: F401
except ImportError:
    sys.modules['rmf_adapter'] = types.ModuleType('rmf_adapter')
    sys.modules['rmf_adapter.easy_full_control'] = easy

from isaacbot_amr_rmf_fleet_adapter.infrastructure import rmf_robot  # noqa: E402
from isaacbot_amr_rmf_fleet_adapter.infrastructure.rmf_robot import RmfRobot  # noqa: E402


class Commission:
    accept_dispatched_tasks = True
    accept_direct_tasks = True
    perform_idle_behavior = True


class More:
    def __init__(self):
        self.commission_ready = False        # RMF finishes add_robot asynchronously
        self.applied = None
        self.status = 'unset'

    def commission(self):
        return Commission() if self.commission_ready else None

    def set_commission(self, commission):
        self.applied = commission

    def override_status(self, status):
        self.status = status


class Handle:
    def __init__(self):
        self._more = More()
        self.updates = 0

    def more(self):
        return self._more

    def update(self, state, activity):
        self.updates += 1


class Fleet:
    def __init__(self):
        self.handle = Handle()

    def add_robot(self, name, state, configuration, callbacks):
        return self.handle


class Session:
    execution = None

    def tick(self, now):
        pass

    def rmf_state(self):
        return RmfRobotState('L1', (1.0, 2.0, 0.0), 0.8)


class Log:
    def __init__(self):
        self.lines = []

    def info(self, message):
        self.lines.append(message)

    debug = warning = error = info


def test_commission_is_applied_once_rmf_has_registered_the_robot(monkeypatch):
    monkeypatch.setattr(rmf_robot, 'rmf_easy', easy)
    fleet, log = Fleet(), Log()
    robot = RmfRobot('ardennes_001', fleet, configuration=None, log=log)
    robot.attach(Session())
    robot.set_in_fleet(False)                # MANUAL: RMF must not give it tasks

    robot.update()                           # registers; the commission does not exist yet
    more = fleet.handle.more()
    assert more.applied is None
    assert more.status is None               # status still applied (no exception)
    assert any('registered with RMF' in line for line in log.lines)

    more.commission_ready = True
    robot.update()                           # retried on the next update
    assert more.applied is not None
    assert not more.applied.accept_dispatched_tasks and not more.applied.accept_direct_tasks
    assert not more.applied.perform_idle_behavior

    more.applied = None
    robot.update()                           # applied once, not on every update
    assert more.applied is None and fleet.handle.updates == 2
