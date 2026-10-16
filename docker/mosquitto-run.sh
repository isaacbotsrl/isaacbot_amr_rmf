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
# Mosquitto for the server deployment (docker-compose.server.yml), with logins the RMF site
# manager can change at runtime.
#
# The password file and ACL live in the broker-auth volume (/mosquitto/auth), which the site
# manager writes when robots are added or removed. This script starts Mosquitto, then sends it
# SIGHUP whenever either file changes: Mosquitto reloads both without dropping any client.
#
# First start: the volume is seeded from /mosquitto/seed (server/mosquitto.passwd from
# rmf-server.sh init, and mosquitto.acl), so the existing logins keep working.
#
# Mosquitto also logs to $AUTH/broker.log for the site manager (who is connected, from where).
# Past LOG_MAX bytes it becomes broker.log.1 and Mosquitto reopens a new one on SIGHUP.
AUTH=/mosquitto/auth
SEED=/mosquitto/seed
LOG="$AUTH/broker.log"
LOG_MAX=524288

for f in passwd acl; do
    if [ ! -s "$AUTH/$f" ] && [ -s "$SEED/$f" ]; then
        cp "$SEED/$f" "$AUTH/$f"
        echo "mosquitto-run: seeded $AUTH/$f from $SEED/$f"
    fi
    if [ -f "$AUTH/$f" ]; then
        chown 1883:1883 "$AUTH/$f"
        chmod 0600 "$AUTH/$f"
    fi
done

new_log() {
    touch "$LOG"
    chown 1883:1883 "$LOG"
}
new_log

stamp() { cat "$AUTH/passwd" "$AUTH/acl" 2>/dev/null | md5sum; }

mosquitto -c /mosquitto/config/mosquitto.conf &
PID=$!
trap 'kill -TERM "$PID" 2>/dev/null; wait "$PID"; exit 0' TERM INT

LAST="$(stamp)"
while kill -0 "$PID" 2>/dev/null; do
    sleep 2
    if [ "$(wc -c < "$LOG" 2>/dev/null || echo 0)" -gt "$LOG_MAX" ]; then
        mv "$LOG" "$LOG.1"
        new_log
        kill -HUP "$PID"
    fi
    NOW="$(stamp)"
    if [ "$NOW" != "$LAST" ]; then
        LAST="$NOW"
        for f in passwd acl; do
            [ -f "$AUTH/$f" ] && chown 1883:1883 "$AUTH/$f" && chmod 0600 "$AUTH/$f"
        done
        echo "mosquitto-run: logins or ACL changed; reloading (SIGHUP)"
        kill -HUP "$PID"
    fi
done
wait "$PID"
