#!/usr/bin/env bash
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
#
# The simulated isaacbot's side of the Open-RMF demo, in one terminal. Start Gazebo first
# (isaacbot_amr_ros/isaacbot_amr_gazebo/scripts/isaacbot_amr_sim.sh), then:
#
#   start_sim_isaacbot.sh [--broker-port 1884] [--localization slam|indoor|...] [--maps-dir DIR]
#
# --localization indoor runs isaacbot_amr_indoor_nav_manager like the real isaacbot (save a map and places,
# then scripts/import_isaacbot_amr_map.py NAME --dir DIR/NAME); --maps-dir is where it keeps them
# (default ~/isaacbot_amr_maps; the isaacbot uses /maps).
#
# It starts, in the isaacbot_amr_sim.sh environment (rmw_zenoh, local router, namespace `isaacbot`):
#   - isaacbot_amr_navigation bringup (Nav 2, use_sim_time, SLAM by default)
#   - isaacbot_amr_drive_mode and isaacbot_amr_mission_manager
#   - the isaacbot's VDA 5050 connector, pointed at the isaacbot_amr_rmf compose broker (localhost:1884)
# then switches the drive mode to AUTOMATIC (the mission manager refuses orders otherwise).
# Ctrl-C stops everything. Logs: /tmp/isaacbot_amr_rmf_sim/*.log.
set -eo pipefail

BROKER_PORT=1884
LOCALIZATION=slam
MAPS_DIR="$HOME/isaacbot_amr_maps"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --broker-port) BROKER_PORT="$2"; shift 2 ;;
        --localization) LOCALIZATION="$2"; shift 2 ;;
        --maps-dir) MAPS_DIR="$2"; shift 2 ;;
        -h | --help) sed -n '5,20p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done
case "$LOCALIZATION" in
    odom) MAP_FRAME=odom ;;
    *) MAP_FRAME=map ;;
esac

HERE=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
ISAACBOT_AMR_SIM="$HERE/../../isaacbot_amr_ros/isaacbot_amr_gazebo/scripts/isaacbot_amr_sim.sh"
if [[ ! -f "$ISAACBOT_AMR_SIM" ]]; then
    echo "isaacbot_amr_sim.sh not found at $ISAACBOT_AMR_SIM" >&2
    exit 1
fi
# Sourcing only prepares the shell: RMW, namespace, workspace overlay.
# shellcheck disable=SC1090
source "$ISAACBOT_AMR_SIM"
NS="${ISAACBOT_AMR_SYSTEM_NAMESPACE:-isaacbot}"

if ! ss -Hltn 'sport = :7447' 2>/dev/null | grep -q LISTEN; then
    echo "No zenoh router on :7447: start the simulation (isaacbot_amr_sim.sh) first." >&2
    exit 1
fi
if ! ss -Hltn "sport = :$BROKER_PORT" 2>/dev/null | grep -q LISTEN; then
    echo "Warning: nothing listens on :$BROKER_PORT yet (docker compose up in isaacbot_amr_rmf/docker);" \
         "the connector keeps retrying." >&2
fi

LOG_DIR=/tmp/isaacbot_amr_rmf_sim
mkdir -p "$LOG_DIR"

# Each launch is a job with its own process group, so it can be stopped with SIGINT as a whole
# (ros2 launch on SIGTERM orphans its nodes under rmw_zenoh).
set -m
PGIDS=()
start() {
    local name="$1"
    shift
    echo "starting $name (log: $LOG_DIR/$name.log)"
    "$@" > "$LOG_DIR/$name.log" 2>&1 &
    PGIDS+=("$!")
}

stop_all() {
    trap - INT TERM EXIT
    echo "stopping..."
    for pgid in "${PGIDS[@]}"; do kill -INT -- "-$pgid" 2> /dev/null || true; done
    for _ in $(seq 20); do
        local alive=0
        for pgid in "${PGIDS[@]}"; do kill -0 -- "-$pgid" 2> /dev/null && alive=1; done
        [[ $alive -eq 0 ]] && return
        sleep 0.5
    done
    # Nav 2's component container ignores SIGINT.
    for pgid in "${PGIDS[@]}"; do kill -KILL -- "-$pgid" 2> /dev/null || true; done
}
trap stop_all INT TERM EXIT

# Everything but Nav 2 runs as a zenoh CLIENT of the local router. A peer that exits (a
# restarted connector, a CLI call) stalls the other peers' links: slam_toolbox then logs
# "Unable to push non droppable network message", drops off, and the map frame disappears
# (seen 2026-09-29 when restarting the connector). Clients leave the peer mesh alone.
ZENOH_CLIENT='mode="client";connect/endpoints=["tcp/127.0.0.1:7447"];listen/endpoints=[]'

mkdir -p "$MAPS_DIR"
start navigation ros2 launch isaacbot_amr_navigation bringup.launch.py \
    namespace:="$NS" use_sim_time:=True localization_source:="$LOCALIZATION" maps_dir:="$MAPS_DIR"
start drive_mode env ZENOH_CONFIG_OVERRIDE="$ZENOH_CLIENT" \
    ros2 launch isaacbot_amr_drive_mode isaacbot_amr_drive_mode.launch.py namespace:="$NS" use_sim_time:=True
start mission_manager env ZENOH_CONFIG_OVERRIDE="$ZENOH_CLIENT" \
    ros2 launch isaacbot_amr_mission_manager isaacbot_amr_mission_manager.launch.py \
    namespace:="$NS" use_sim_time:=True localization_source:="$LOCALIZATION"
start vda5050 env ZENOH_CONFIG_OVERRIDE="$ZENOH_CLIENT" \
    ros2 launch isaacbot_amr_vda5050_bringup vda5050.launch.py \
    namespace:="$NS" use_sim_time:=true map_frame:="$MAP_FRAME" \
    broker_host:=127.0.0.1 broker_port:="$BROKER_PORT"

export ZENOH_CONFIG_OVERRIDE="$ZENOH_CLIENT"
echo "switching /$NS to AUTOMATIC (waits for the mission manager)..."
automatic=false
for _ in $(seq 60); do
    if timeout -k 2 10 ros2 service call "/$NS/set_drive_mode" isaacbot_amr_msgs/srv/SetDriveMode \
        '{mode: 3}' 2> /dev/null | grep -q 'success=True'; then
        automatic=true
        break
    fi
    sleep 2
done
if $automatic; then
    echo "drive mode AUTOMATIC. The isaacbot is ready for RMF."
else
    echo "Could not switch to AUTOMATIC after 2 min; see $LOG_DIR/drive_mode.log." >&2
fi
unset ZENOH_CONFIG_OVERRIDE

echo "running; Ctrl-C to stop."
wait
