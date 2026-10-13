#!/usr/bin/env python3
# Copyright 2026 Mechatronics Academy
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
Import a map saved on the isaacbot as an Open-RMF site.

    import_isaacbot_amr_map.py lab --from-isaacbot root@192.168.1.201     # over ssh (one password prompt)
    import_isaacbot_amr_map.py lab --dir ~/isaacbot_amr_maps/lab               # a local copy

The map is one saved with the drive UI in indoor mode (isaacbot_amr_indoor_nav_manager:
/maps/<name>/map.yaml, map.pgm, places.yaml). Every place becomes an RMF waypoint; the place
named --charger (default "dock") is where the isaacbot parks and charges. Writes:

    isaacbot_amr_rmf_maps/maps/<name>/<name>.png, <name>.building.yaml   floor plan, waypoints, lanes
    isaacbot_amr_rmf_bringup/config/fleet_<name>.yaml                    transform, map name, broker

The RMF level is named <name>, like the isaacbot's map, because the isaacbot reports that name as its
VDA 5050 mapId. Then rebuild and start RMF for the site:

    RMF_SITE=<name> RMF_BROKER_HOST=192.168.1.201 \\
        docker compose -f docker/docker-compose.yml up -d --build
"""

import argparse
import os
import re
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'isaacbot_amr_rmf_maps', 'scripts'))

from isaacbot_amr_map_import import import_site, MapImportError  # noqa: E402
from site_io import SshMapSource, write_site  # noqa: E402
import yaml  # noqa: E402

TEMPLATE = os.path.join(REPO, 'isaacbot_amr_rmf_bringup', 'config', 'fleet_isaacbot_amr_world.yaml')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('name', help='map name on the isaacbot (becomes the RMF site and level)')
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--from-isaacbot', metavar='USER@HOST', help='copy the map from the isaacbot')
    source.add_argument('--dir', help='local directory with map.yaml, map.pgm, places.yaml')
    parser.add_argument('--ssh-port', type=int, default=24,
                        help="the orchestrator container's sshd (default 24)")
    parser.add_argument('--charger', default='dock', help='place where the isaacbot parks')
    parser.add_argument('--clearance', type=float, default=0.8,
                        help='m kept from obstacles along lanes (footprint radius 0.61 + margin)')
    parser.add_argument('--max-lane', type=float, default=15.0, help='longest lane, m')
    parser.add_argument('--neighbours', type=int, default=3,
                        help='shortest lanes kept per place, on top of a spanning tree')
    parser.add_argument('--broker-host', default='192.168.1.201',
                        help="MQTT broker of the isaacbot's VDA 5050 connector")
    parser.add_argument('--broker-port', type=int, default=1883)
    args = parser.parse_args()

    if not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', args.name):
        sys.exit(f'{args.name!r} is not a isaacbot map name ([A-Za-z0-9_-], up to 64)')
    if args.name == 'isaacbot_amr_world':
        sys.exit('isaacbot_amr_world is the Gazebo site; pick another name')

    with tempfile.TemporaryDirectory() as tmp:
        try:
            if args.from_isaacbot:
                # batch=False: ssh may ask for the password (once, one connection).
                source = SshMapSource(args.from_isaacbot, args.ssh_port, batch=False)
                print(f'Copying /maps/{args.name} from {source.describe()}...', flush=True)
                map_dir = source.fetch(args.name, tmp)
            else:
                map_dir = os.path.expanduser(args.dir)
            site = import_site(args.name, map_dir, args.charger, args.clearance,
                               args.max_lane, args.neighbours)
        except MapImportError as e:
            sys.exit(f'Import failed: {e}')

    out = os.path.join(REPO, 'isaacbot_amr_rmf_maps', 'maps', args.name)
    config_path = os.path.join(REPO, 'isaacbot_amr_rmf_bringup', 'config', f'fleet_{args.name}.yaml')
    with open(TEMPLATE) as f:
        template = yaml.safe_load(f)
    charger = write_site(site, out, config_path, template, args.broker_host, args.broker_port,
                         origin='scripts/import_isaacbot_amr_map.py')

    tx, ty = site.translation
    print(f'Site {args.name}: {site.width}x{site.height} px at {site.resolution} m/px, '
          f'{len(site.waypoints)} waypoints, {len(site.lanes)} lanes, charger {charger!r}')
    print(f'  robot = RMF + ({tx:.3f}, {ty:.3f})')
    for a, b in site.lanes:
        print(f'  lane {a} <-> {b}')
    for w in site.warnings:
        print(f'  WARNING: {w}')
    print(f'Wrote {os.path.relpath(out, REPO)}/ and {os.path.relpath(config_path, REPO)}')
    print(f'Next (not committed: git ignores imported maps): RMF_SITE={args.name} '
          'docker compose -f docker/docker-compose.yml up -d --build')


if __name__ == '__main__':
    main()
