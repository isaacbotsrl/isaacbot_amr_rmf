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
A fake isaacbot that speaks VDA 5050 2.0 over MQTT, for testing the server side with more robots.

It answers like Ardennes's connector (isaacbot_amr_vda5050) as far as the RMF fleet adapter can
tell: connection (retained, CONNECTIONBROKEN last will), factsheet, state every 1 s and on events,
visualization every 0.5 s. It drives an order's released nodes in a straight line at SPEED and
stitches order updates; it runs the instant actions the adapter sends (cancelOrder, startPause,
stopPause, setDriveMode) plus stateRequest / factsheetRequest. No ROS, no Nav 2, no map.

Environment:
    BROKER_HOST, BROKER_PORT        default 10.8.0.1, 1883
    BROKER_USER, BROKER_PASSWORD    the robot's login (the sites page shows it when adding it)
    MANUFACTURER, SERIAL_NUMBER     its VDA 5050 identity (default IsaacBot, ardennis-3420987-e)
    INTERFACE_NAME                  default uagv; topics are <interface>/v2/<manufacturer>/<serial>/
    START_X, START_Y, START_THETA   start pose in the robot's map frame (m, rad): put it on a
                                    nav graph waypoint, usually its charger
    MAP_ID                          reported mapId (default '': the fleet adapter's map_name)
    SPEED                           m/s (default 0.5)
    BATTERY                         state of charge % (default 80)
"""

from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import math
import os
import signal
import sys
import threading
import time
from typing import Dict, List, Set

VERSION = '2.0.0'
ARRIVED = 0.05            # m: a node counts as reached
FLEET_MODES = ('AUTOMATIC', 'SEMIAUTOMATIC')


@dataclass
class ActionState:
    action_type: str
    status: str
    result: str = ''


@dataclass
class FakeRobot:
    """The robot's state and behaviour; no MQTT (unit-tested on its own)."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    speed: float = 0.5
    battery: float = 80.0
    map_id: str = ''
    operating_mode: str = 'AUTOMATIC'
    paused: bool = False
    driving: bool = False
    order_id: str = ''
    order_update_id: int = 0
    last_node_id: str = ''
    last_node_sequence_id: int = 0
    nodes: List[dict] = field(default_factory=list)       # still to reach, in order
    edges: List[dict] = field(default_factory=list)       # still to traverse
    errors: List[dict] = field(default_factory=list)
    actions: Dict[str, ActionState] = field(default_factory=dict)

    # --- orders ---------------------------------------------------------------------------

    def on_order(self, order: dict) -> None:
        order_id = str(order.get('orderId', ''))
        update_id = int(order.get('orderUpdateId', 0))
        nodes = sorted((n for n in order.get('nodes') or [] if n.get('released', True)),
                       key=lambda n: n.get('sequenceId', 0))
        edges = sorted((e for e in order.get('edges') or [] if e.get('released', True)),
                       key=lambda e: e.get('sequenceId', 0))
        if not nodes:
            return self._reject(order_id, 'validationError', 'the order has no released node')
        if self.operating_mode not in FLEET_MODES:
            return self._reject(order_id, 'orderError',
                                f'operatingMode {self.operating_mode}: not taking orders')
        if order_id == self.order_id:
            if update_id <= self.order_update_id:
                return                                    # duplicate or outdated: ignore
            # Stitch: the update starts at our last base node; keep what follows it.
            if nodes[0].get('nodeId') != self.last_node_id and self.nodes:
                stitch = next((i for i, n in enumerate(self.nodes)
                               if n.get('nodeId') == nodes[0].get('nodeId')), None)
                if stitch is None:
                    return self._reject(order_id, 'orderUpdateError',
                                        'the update does not start on the current base')
                # Keep the way up to and including the stitch node (edge i leads to node i).
                self.nodes = self.nodes[:stitch + 1]
                self.edges = self.edges[:stitch + 1]
            self.nodes += nodes[1:]
            self.edges += edges
            self.order_update_id = update_id
            return
        if self.nodes:
            return self._reject(order_id, 'orderError',
                                f'order {self.order_id!r} is still active; cancel it first')
        self.errors = []
        self.order_id, self.order_update_id = order_id, update_id
        # The first node is where the robot stands: reached at once.
        self._reach(nodes[0])
        self.nodes, self.edges = nodes[1:], edges

    def _reject(self, order_id: str, error_type: str, description: str) -> None:
        self.errors = [{'errorType': error_type, 'errorLevel': 'WARNING',
                        'errorDescription': description,
                        'errorReferences': [{'referenceKey': 'orderId',
                                             'referenceValue': order_id}]}]

    def _reach(self, node: dict) -> None:
        self.last_node_id = str(node.get('nodeId', ''))
        self.last_node_sequence_id = int(node.get('sequenceId', 0))
        position = node.get('nodePosition') or {}
        if position.get('theta') is not None and 'x' in position:
            self.theta = float(position['theta'])

    # --- instant actions ------------------------------------------------------------------

    def on_instant_actions(self, message: dict) -> Set[str]:
        """Runs the actions; returns the topics to publish right away (state, factsheet)."""
        publish = {'state'}
        for action in message.get('actions') or []:
            action_id = str(action.get('actionId', ''))
            kind = str(action.get('actionType', ''))
            params = {p.get('key'): p.get('value') for p in action.get('actionParameters') or []}
            status, result = 'FINISHED', ''
            if kind == 'cancelOrder':
                if not self.nodes:
                    status, result = 'FAILED', 'no order to cancel'
                self.nodes, self.edges, self.driving = [], [], False
            elif kind == 'startPause':
                self.paused, self.driving, result = True, False, 'Paused.'
            elif kind == 'stopPause':
                self.paused, result = False, 'Resumed.'
            elif kind == 'setDriveMode':
                mode = str(params.get('mode', ''))
                if mode in ('MANUAL', 'AUTOMATIC'):
                    self.operating_mode, result = mode, mode
                    if mode == 'MANUAL':
                        self.nodes, self.edges, self.driving = [], [], False
                else:
                    status, result = 'FAILED', f'unknown mode {mode!r}'
            elif kind == 'stateRequest':
                pass
            elif kind == 'factsheetRequest':
                publish.add('factsheet')
            else:
                status, result = 'FAILED', f'{kind} is not supported by the fake robot'
            self.actions[action_id] = ActionState(kind, status, result)
        return publish

    # --- motion ---------------------------------------------------------------------------

    def step(self, dt: float) -> bool:
        """Advance dt seconds; True when a node was reached (an event: publish state)."""
        moving = (bool(self.nodes) and not self.paused
                  and self.operating_mode in FLEET_MODES)
        self.driving = moving
        if not moving:
            return False
        target = self.nodes[0].get('nodePosition') or {}
        tx, ty = float(target.get('x', self.x)), float(target.get('y', self.y))
        dx, dy = tx - self.x, ty - self.y
        distance = math.hypot(dx, dy)
        travel = self.speed * dt
        if distance > ARRIVED:
            self.theta = math.atan2(dy, dx)
        if distance <= max(travel, ARRIVED):
            self.x, self.y = tx, ty
            self._reach(self.nodes.pop(0))
            if self.edges:
                self.edges.pop(0)
            self.driving = bool(self.nodes)
            return True
        self.x += dx / distance * travel
        self.y += dy / distance * travel
        return False

    # --- messages (bodies without the header) ---------------------------------------------

    def position(self) -> dict:
        return {'x': self.x, 'y': self.y, 'theta': self.theta, 'mapId': self.map_id,
                'positionInitialized': True}

    def state(self) -> dict:
        return {
            'orderId': self.order_id,
            'orderUpdateId': self.order_update_id,
            'lastNodeId': self.last_node_id,
            'lastNodeSequenceId': self.last_node_sequence_id,
            'nodeStates': [{'nodeId': n.get('nodeId'), 'sequenceId': n.get('sequenceId'),
                            'released': True} for n in self.nodes],
            'edgeStates': [{'edgeId': e.get('edgeId'), 'sequenceId': e.get('sequenceId'),
                            'released': True} for e in self.edges],
            'agvPosition': self.position(),
            'velocity': {'vx': self.speed if self.driving else 0.0, 'vy': 0.0, 'omega': 0.0},
            'driving': self.driving,
            'paused': self.paused,
            'newBaseRequest': False,
            'operatingMode': self.operating_mode,
            'batteryState': {'batteryCharge': self.battery, 'batteryVoltage': 24.0,
                             'charging': False},
            'actionStates': [{'actionId': i, 'actionType': a.action_type,
                              'actionStatus': a.status, 'resultDescription': a.result}
                             for i, a in self.actions.items()],
            'errors': self.errors,
            'information': [{'infoType': 'simulation', 'infoLevel': 'INFO',
                             'infoDescription': 'fake_vda_robot (no real isaacbot)'}],
            'safetyState': {'eStop': 'NONE', 'fieldViolation': False},
        }

    def visualization(self) -> dict:
        return {'agvPosition': self.position(),
                'velocity': {'vx': self.speed if self.driving else 0.0, 'vy': 0.0, 'omega': 0.0}}


def factsheet(speed: float) -> dict:
    return {
        'typeSpecification': {'seriesName': 'Ardennes (simulated)', 'agvKinematic': 'DIFF',
                              'agvClass': 'CARRIER', 'maxLoadMass': 0.0,
                              'localizationTypes': ['NATURAL'],
                              'navigationTypes': ['AUTONOMOUS']},
        'physicalParameters': {'speedMin': 0.0, 'speedMax': speed, 'accelerationMax': 1.4,
                               'decelerationMax': 1.4, 'length': 0.913, 'width': 0.803,
                               'heightMin': 0.0, 'heightMax': 0.0},
        'protocolFeatures': {'agvActions': [
            {'actionType': t, 'actionScopes': ['INSTANT']}
            for t in ('cancelOrder', 'startPause', 'stopPause', 'setDriveMode', 'stateRequest',
                      'factsheetRequest')]},
    }


# --- MQTT ----------------------------------------------------------------------------------------

class MqttRobot:

    def __init__(self, robot: FakeRobot, env: dict):
        from paho.mqtt import client as mqtt
        self.mqtt = mqtt
        self.robot = robot
        self.lock = threading.Lock()
        self.manufacturer = env.get('MANUFACTURER', 'IsaacBot')
        self.serial = env.get('SERIAL_NUMBER', 'ardennis-3420987-e')
        self.prefix = f'{env.get("INTERFACE_NAME", "uagv")}/v2/{self.manufacturer}/{self.serial}'
        self.host = env.get('BROKER_HOST', '10.8.0.1')
        self.port = int(env.get('BROKER_PORT', '1883'))
        self.headers: Dict[str, int] = {}
        self.pending: Set[str] = set()
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                                  client_id=f'fake-{self.serial}')
        if env.get('BROKER_USER'):
            self.client.username_pw_set(env['BROKER_USER'], env.get('BROKER_PASSWORD') or None)
        self.client.will_set(self.topic('connection'),
                             json.dumps(self.message('connection',
                                                     {'connectionState': 'CONNECTIONBROKEN'})),
                             qos=1, retain=True)
        self.client.on_connect = self._connected
        self.client.on_message = self._message
        self.client.reconnect_delay_set(1, 10)

    def topic(self, name: str) -> str:
        return f'{self.prefix}/{name}'

    def message(self, topic: str, body: dict) -> dict:
        self.headers[topic] = self.headers.get(topic, 0) + 1
        now = datetime.now(timezone.utc).isoformat(timespec='milliseconds').replace('+00:00', 'Z')
        return {'headerId': self.headers[topic], 'timestamp': now, 'version': VERSION,
                'manufacturer': self.manufacturer, 'serialNumber': self.serial, **body}

    def publish(self, topic: str, body: dict, qos: int = 0, retain: bool = False) -> None:
        self.client.publish(self.topic(topic), json.dumps(self.message(topic, body)),
                            qos=qos, retain=retain)

    def _connected(self, client, userdata, flags, reason_code, properties=None):
        if reason_code.is_failure:
            print(f'fake_vda_robot: broker {self.host}:{self.port} refused: {reason_code}',
                  flush=True)
            return
        print(f'fake_vda_robot: connected to {self.host}:{self.port} as {self.prefix}', flush=True)
        client.subscribe([(self.topic('order'), 0), (self.topic('instantActions'), 0)])
        self.publish('connection', {'connectionState': 'ONLINE'}, qos=1, retain=True)
        self.publish('factsheet', factsheet(self.robot.speed), retain=True)
        with self.lock:
            self.pending.add('state')

    def _message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload)
        except ValueError:
            return
        name = msg.topic.rsplit('/', 1)[-1]
        with self.lock:
            if name == 'order':
                print(f'fake_vda_robot: order {payload.get("orderId")} '
                      f'update {payload.get("orderUpdateId")}', flush=True)
                self.robot.on_order(payload)
                self.pending.add('state')
            elif name == 'instantActions':
                kinds = [a.get('actionType') for a in payload.get('actions') or []]
                print(f'fake_vda_robot: instant actions {kinds}', flush=True)
                self.pending |= self.robot.on_instant_actions(payload)

    def run(self, stop: threading.Event) -> None:
        self.client.connect_async(self.host, self.port, keepalive=30)
        self.client.loop_start()
        period, last_state, last_vis = 0.1, 0.0, 0.0
        while not stop.wait(period):
            now = time.monotonic()
            with self.lock:
                if self.robot.step(period):
                    self.pending.add('state')
                if now - last_state >= 1.0:
                    self.pending.add('state')
                pending, self.pending = self.pending, set()
                state = self.robot.state() if 'state' in pending else None
                vis = self.robot.visualization() if now - last_vis >= 0.5 else None
            if state is not None:
                self.publish('state', state)
                last_state = now
            if vis is not None:
                self.publish('visualization', vis)
                last_vis = now
            if 'factsheet' in pending:
                self.publish('factsheet', factsheet(self.robot.speed), retain=True)
        self.publish('connection', {'connectionState': 'OFFLINE'}, qos=1, retain=True)
        time.sleep(0.5)
        self.client.loop_stop()
        self.client.disconnect()


def main() -> int:
    env = os.environ
    robot = FakeRobot(x=float(env.get('START_X', 0.0)), y=float(env.get('START_Y', 0.0)),
                      theta=float(env.get('START_THETA', 0.0)),
                      speed=float(env.get('SPEED', 0.5)), battery=float(env.get('BATTERY', 80)),
                      map_id=env.get('MAP_ID', ''))
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    MqttRobot(robot, dict(env)).run(stop)
    return 0


if __name__ == '__main__':
    sys.exit(main())
