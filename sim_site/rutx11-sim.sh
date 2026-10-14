#!/bin/sh
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
#
# A stand-in for a second site's RUTX11: the WireGuard peer (10.8.0.6) that a robot reaches the
# server through. The robot container shares this container's network (network_mode), so its
# MQTT connection to 10.8.0.1 leaves through this tunnel, as a real robot's does.
#
# /secrets/wg0.conf is the client config wg-add-client.sh made on the server (split tunnel).
set -e
CONF=/etc/wireguard/wg0.conf
SERVER="${WG_CHECK_HOST:-10.8.0.1}"

[ -s /secrets/wg0.conf ] || { echo "rutx11-sim: no /secrets/wg0.conf (see README.md)" >&2; exit 1; }
mkdir -p /etc/wireguard
# No resolvconf in the container: the tunnel carries only 10.8.0.0/24, so DNS stays local.
grep -viE '^\s*DNS\s*=' /secrets/wg0.conf > "$CONF"
chmod 600 "$CONF"

down() { wg-quick down wg0 2>/dev/null || true; exit 0; }
trap down TERM INT

wg-quick up wg0
echo "rutx11-sim: tunnel up as $(ip -4 -br addr show wg0 | awk '{print $3}')"

while true; do
    if ping -c 1 -W 3 "$SERVER" > /dev/null 2>&1; then
        state="server $SERVER reachable"
    else
        state="server $SERVER NOT reachable"
    fi
    handshake="$(wg show wg0 latest-handshakes | awk '{print $2}')"
    echo "rutx11-sim: $state; last handshake $(( $(date +%s) - ${handshake:-0} )) s ago"
    sleep 60 &
    wait $!
done
