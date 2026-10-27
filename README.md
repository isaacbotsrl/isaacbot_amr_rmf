<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="icons/isaacbot-lockup-white.svg">
    <img src="icons/isaacbot-lockup-black-bold.svg" alt="IsaacBot" width="240">
  </picture>
</p>

# isaacbot_amr_rmf — Open-RMF fleet management for Ardennes over MQTT

Open-RMF plans, schedules and dispatches tasks for Ardennes. The only link between the two is
**VDA 5050 2.0 over MQTT**: RMF's fleet adapter acts as VDA 5050 master control, and the isaacbot
keeps its existing connector (`isaacbot_amr_vda5050`) unchanged. RMF runs off-robot in a **ROS 2 Jazzy**
container, because RMF's fleet adapter, dispatcher and websocket have no Lyrical release. The two
ROS distros never share a graph.

```
 Browser ── rmf-web dashboard :3000 ── api-server :8010
                                          │ ROS 2 Jazzy, rmw_cyclonedds, compose network only
 ┌─ docker compose (docker/) ─────────────┴────────────────────────────────────────┐
 │ rmf_traffic_schedule · blockade · rmf_task_dispatcher · building_map_server      │
 │ schedule visualizer (:8006) · isaacbot_amr_rmf_fleet_adapter (EasyFullControl + paho)   │
 │ mosquitto (host :1884)                                                           │
 └──────────────────────────────┬──────────────────────────────────────────────────┘
      MQTT uagv/v2/IsaacBot/ardennis-3420987-k/{order, instantActions ↓ · state, connection, visualization ↑}
 Host (Lyrical, rmw_zenoh)      ▼
   isaacbot_amr_vda5050 connector → isaacbot_amr_mission_manager → Nav 2 → Gazebo (isaacbot_amr_gazebo)
```

## Packages

| Package | What |
|---------|------|
| `isaacbot_amr_rmf_fleet_adapter` | EasyFullControl fleet adapter. `domain/` holds the VDA 5050 messages, the state parsing and the command tracker. `application/` holds `RobotSession`. `infrastructure/` holds the paho link and the RMF binding. `presentation/` is the entry point. Only `infrastructure/` and `presentation/` import RMF, ROS or paho, and `scripts/check_domain_purity.sh` enforces it. |
| `isaacbot_amr_rmf_maps` | One RMF site per directory in `maps/<site>/`: a floor plan and a building whose level is named `<site>`. `isaacbot_amr_world` is the Gazebo world (9 waypoints such as `ardennes_charger` and `north`, 11 two-way lanes, generated from `isaacbot_amr_world.sdf`). Real sites are imported from the isaacbot's saved maps. Nav graphs are generated at build time. `scripts/building_writer.py` and `scripts/isaacbot_amr_map_import.py` hold the shared code. |
| `isaacbot_amr_rmf_bringup` | `isaacbot_amr_rmf.launch.xml` (RMF core and the adapter; `site:=<site>`) and `config/fleet_<site>.yaml` (RMF fleet config plus the adapter's `vda5050` section). |
| `dashboard/` | Our rmf-web dashboard app: IsaacBot branding, the **ISAACBOT TASK** dialog (`tasks/`) and the **Isaacbot** window (`isaacbot/`: drive mode, in or out of the fleet). See [Dashboard](#dashboard). |
| `docker/` | `Dockerfile.rmf` (Jazzy, RMF debs, builds and tests the packages), `docker-compose.yml` (mosquitto, rmf, rmf-web api-server and dashboard), broker and api-server configs. `Dockerfile.dashboard` builds `dashboard/` inside a pinned rmf-web checkout. |
| `scripts/start_sim_isaacbot.sh` | Starts the simulated isaacbot's side in one terminal: Nav 2, drive mode, mission manager and the VDA 5050 connector on `:1884`. It then sets the drive mode to AUTOMATIC. Localization is automatic, as on the real isaacbot. |
| `site_manager/` | The RMF robots and sites page (<http://localhost:8020>, compose service `site-manager`). The main page lists the fleet's robots. A robot's page lists the maps saved on that isaacbot and imports them as sites into the `rmf-sites` volume; `/sites` lists the sites, which the whole fleet shares, and activates one. `docker/run_rmf.sh` runs RMF for the active site and restarts it on a switch. |
| `scripts/import_isaacbot_amr_map.py` | The same import from the command line, into `isaacbot_amr_rmf_maps/maps/` (git-ignored, built into the local image). |

## How a command reaches the isaacbot

RMF's EasyFullControl hands the adapter one destination at a time. Each destination becomes a
two-node VDA order, from the isaacbot's current pose to the destination, in the format
`fake_master.py` verified. The node ids carry the order id (`<order>-start`, `<order>-goal`).

- **Arrival:** the order counts as arrived when the isaacbot reports it with no nodes or edges left
  and `lastNodeId == <order>-goal`. The adapter then calls `execution.finished()`.
- **Failure:** the command fails in any of these cases, and the adapter asks RMF to
  `replan()`:
  - an error references the order or one of its nodes, for example `orderUpdateError`, or the
    `FATAL` `noRouteError` that follows a refused or failed mission;
  - the order ends away from the goal, for example because it was cancelled on the web UI or
    the isaacbot left AUTOMATIC;
  - the isaacbot switches to another order;
  - the order is not accepted within 10 s.
- **Replacing a running order:** the connector rejects a new `orderId` while an order still has
  nodes or edges. So when an order is running, the adapter sends `cancelOrder`, waits until the
  isaacbot is idle (up to 20 s), and then sends the new order from where the isaacbot stopped.
- **Isaacbot not localized:** a command that arrives while the isaacbot reports no position
  (`positionInitialized: false`) or no state waits up to 10 s for one, then fails. It does not
  fail at once, because RMF answers a failure with an immediate replan and the two would spin.
- **Stall watchdog:** an active order that makes no progress (0.3 m or 0.5 rad) for 60 s
  (`vda5050.stall_timeout`) fails, and RMF replans. Nav 2 never gives up on its own when the
  skid steer stalls near a goal.
- **Isaacbot not available:** while the isaacbot reports `operatingMode` `MANUAL`/`SERVICE` (drive mode
  not Automatic) or a `motionLocked` error (e-stop, lock), the adapter sends no order. RMF's
  command waits, with no timeout, and goes out once the isaacbot is back in Automatic and unlocked.
  The log says `holding the command` once. (`missionRefused` doesn't hold: it is why the *last*
  order was refused and stays until the next one is accepted.)
- **Replan backoff:** a failed command asks RMF to replan after 1 s, then 2, 4 … up to 60 s for
  failures in a row; an arrival resets it. On 2026-10-01 the real isaacbot refused every order at
  once, and without the backoff RMF and the isaacbot exchanged an order and a refusal every second
  over 4G.
- **`stop()`:** sends `cancelOrder`, and cancels a running perform-action.
- **In or out of the fleet:** the adapter decommissions the isaacbot in RMF while it is offline or
  an operator has it (`operatingMode` `MANUAL`/`SERVICE`: drive mode Manual or Assisted). It
  recommissions it on its next `AUTOMATIC` state. Decommissioned, RMF gives it no new tasks and a
  task sent anyway fails at dispatch ("No fleet adapters offered a bid"). Queued tasks are kept.
  Because this is derived from the isaacbot's state, it holds whoever switched the mode and survives
  RMF restarts. A decommission from rmf-web's robot dialog lasts until the next mode change. While
  the connection is down, RMF also shows the robot `offline`. RMF never removes a robot
  otherwise; it stays registered for the adapter's lifetime.
- **Drive mode from RMF:** the adapter's **isaacbot control API**
  (`presentation/control_api.py`, port `vda5050.control_api_port`, 8030) serves the dashboard's
  Isaacbot window. `POST /api/robots/<name>/drive_mode {"mode": "MANUAL"|"AUTOMATIC"}` sends the
  isaacbot's custom VDA 5050 instant action `setDriveMode` (isaacbot_amr_vda5050). The request answers once
  the isaacbot reports the action `FINISHED` or `FAILED` in `actionStates`, or after 10 s.
  `GET /api/robots` returns each isaacbot's connection, mode, fleet membership and last request.
- **Idle isaacbot:** `finishing_request: "nothing"` and `responsive_wait: false`, so the isaacbot stays
  where its last task ended and starting RMF never moves it by itself. "park" would send it to its
  charger after every task and on startup. A responsive wait would drive it to the nearest
  waypoint.
- **Perform-actions:** a compose task's `perform_action` runs in the adapter
  (`application/actions.py`), by category. The only one so far is `wait`
  (`{"duration_sec": N}`, up to 3600 s): the isaacbot stays where it is and the adapter finishes
  the action when the time is up. A category the fleet config lists under `actions` but the
  adapter has no handler for is logged at startup. If such an action is dispatched anyway, the
  adapter skips it with an error.
- **Reported state:** position comes from `state` (1 Hz) and `visualization` (2 Hz). The map is
  `mapId` (the isaacbot's indoor map name), or `vda5050.map_name` while that is empty. The battery comes from `batteryState`; when the isaacbot
  reports 0 % and 0 V (no reading, as in Gazebo), the adapter reports `unknown_battery_soc`
  (1.0) instead. Nothing is reported while `connection` is not `ONLINE`.

**Coordinates:** each site's fleet config has `transforms.<site>` from RMF coordinates to the
isaacbot's map frame. EasyFullControl applies it to destinations and its inverse to reported
positions.
- **Gazebo:** RMF (x, y) = world (x + 12.5, y − 12.5). The simulated isaacbot's SLAM map starts at
  its spawn pose, world (0, −2), so the translation is `[-12.5, 14.5]`.
- **Imported sites:** the floor plan is the isaacbot's own map image, so the translation is exact:
  `(origin_x, origin_y + height · resolution)` from `map.yaml`. Nothing is surveyed.

## Running the simulation

Terminal 1, Gazebo:

```bash
~/ros2_ws/src/isaacbot_amr_ros/isaacbot_amr_gazebo/scripts/isaacbot_amr_sim.sh
```

Terminal 2, RMF (Docker Desktop):

```bash
cd ~/ros2_ws/src/isaacbot_amr_rmf
docker compose -f docker/docker-compose.yml up --build
```

Terminal 3, the isaacbot stack:

```bash
~/ros2_ws/src/isaacbot_amr_rmf/scripts/start_sim_isaacbot.sh
```

Then:

- **Dashboard:** open <http://localhost:3000>. `ardennes` appears at `ardennes_charger` on the `isaacbot_amr_world` map.
  **ISAACBOT TASK** → *Patrol with pause* → stops `north` and `east`, pause 10 s, 2 rounds. The `rmf`
  log then shows `perform-action 'wait'` after each arrival.
- **Patrol task from the CLI:**

  ```bash
  docker compose -f docker/docker-compose.yml exec rmf \
      /entrypoint.sh ros2 run rmf_demos_tasks dispatch_patrol -p north east -n 2
  ```

  The dashboard's task panel can create the same task.
- **Watching the VDA traffic:** `mosquitto_sub -p 1884 -t 'uagv/v2/#' -v`
- **The whole real-isaacbot flow in the sim:** run `start_sim_isaacbot.sh`,
  then map and save the map and its places with the indoor manager's services.
  - Start the stack with `-f docker/docker-compose.sim-maps.yml` added, so the sites page reads
    `~/isaacbot_amr_maps` instead of the isaacbot's `/maps`.
  - On <http://localhost:8020>, import and activate the map, as on the real isaacbot.

**Zenoh:** `start_sim_isaacbot.sh` runs everything except Nav 2 as zenoh clients. A peer that
exits, such as a restarted connector, stalls `slam_toolbox`'s links, and the map frame
disappears.

**Ports:** the broker is published on **1884**, because the host may run its own mosquitto on
1883. WSL 2 distros share the Docker Desktop VM's network, so the host-side connector reaches it
on `127.0.0.1:1884`.

The api-server is published on **8010** (`RMF_API_PORT`), not upstream's 8000. A Windows service
(`Manager.exe`) holds `0.0.0.0:8000` on this box. Docker Desktop then silently publishes nothing,
and the dashboard's LOGIN goes to that service and hangs. The dashboard is built with the same
port. To pick another port, run
`RMF_API_PORT=<port> docker compose -f docker/docker-compose.yml up -d --build`.

## Real isaacbot

RMF runs on the laptop. It talks to the isaacbot's own broker, `192.168.1.201:1883`, which the
isaacbot's `ardennes-vda5050` container runs. The isaacbot localizes indoors on a map saved with the
drive UI, and that map becomes the RMF site.

1. **Deploy.** Push the VDA 5050 and drive-mode work (isaacbot_amr_vda5050 `master`, isaacbot_amr_orchestrator,
   isaacbot_amr_ros, isaacbot_amr_drive_interface, rover_docker), then `balena push`.
2. **balenaCloud variables, on all services:**
   - `ISAACBOT_AMR_ORCH_MISSION_MANAGER=true`
   - `ISAACBOT_AMR_VDA5050_ENABLE=true`
   - `ISAACBOT_AMR_VDA5050_LOCAL_BROKER=true` is already the default.
3. **Map the area** in the drive UI.
   - Start mapping, drive the area, and save the map (e.g. `lab`). Then load it.
   - Save places. `dock` is where the isaacbot parks; it becomes RMF's charger and parking spot.
   - Add 4–8 destinations at corridor junctions and ends, about 1 m from obstacles.
   - RMF drives only between places, along straight lanes the importer finds free in the map.
4. **Check the isaacbot on its own** from the laptop, on the isaacbot LAN (`.184` wired, or the
   ISAACBOT-A1 AP):
   ```bash
   mosquitto_sub -h 192.168.1.201 -t 'uagv/v2/#' -v   # connection ONLINE, state mapId "lab"
   ros2 run isaacbot_amr_vda5050_bringup fake_master.py --host 192.168.1.201 order <x>,<y>   # a short hop
   ```
5. **Start the stack and let it read the isaacbot's maps.**
   ```bash
   docker compose -f docker/docker-compose.yml up -d --build
   docker compose -f docker/docker-compose.yml exec site-manager isaacbot-authorize   # once; password root
   ```
   `isaacbot-authorize` puts the sites page's own ssh key on the isaacbot, which asks for its
   password once. A new isaacbot release forgets the key; the robot's page then says so, and you
   run the command again. Each robot's page shows the command for that robot (its ssh address
   as arguments).
6. **Import and activate the map** on <http://localhost:8020>: open the robot (`ardennes`).
   - **Maps on this robot** lists the saved maps with their places. Pick the charger place
     (default `dock`) and press **Import**.
   - The site appears on the **RMF sites** page (`/sites`) with a preview: lanes in blue, places in orange, the
     charger in green. Warnings name places too close to obstacles.
   - If the places can't all be joined by clear straight lanes, the import fails and names the
     groups. Add a place where the corridors meet, then import again.
   - Press **Activate**: RMF restarts on the site within a few seconds, with no rebuild.
     Importing the running site again restarts RMF on the new version.
7. **On the dashboard** (<http://localhost:3000>, reload it after a site switch), `ardennes`
   appears at `dock` on the `lab` map. Start with a patrol between two places.

Sites imported on the page live in the `rmf-sites` Docker volume. The CLI,
`scripts/import_isaacbot_amr_map.py lab --from-isaacbot root@<isaacbot-lan-ip>`, does the same import into
`isaacbot_amr_rmf_maps/maps/lab/` and `config/fleet_lab.yaml`. It uses ssh port 24 and asks for the
password once. Those files are built into the image made on this machine, but git ignores them:
maps imported from the isaacbot are not committed; only the Gazebo site `isaacbot_amr_world` is. A page
import with the same name overrides a built-in site.

**First runs, safety:**
- Keep the RC transmitter / E-stop at hand and use a clear area.
- RMF only drives in AUTOMATIC. Moving the web joystick takes over to ASSISTED and cancels the
  order, and RMF then replans.
- **Known isaacbot issues:**
  - Goal overshoot and slow in-place turns were DCC1000-era (host wheel PIDs); re-check them
    with the DCC1020 drive. The 60 s stall watchdog makes RMF replan if a turn stalls.
  - Nav 2 bringup aborts if the RS16 lidar is silent at boot, so check that the isaacbot is
    localized (`positionInitialized` true) before dispatching.
- `isaacbot_amr_fleet` limits (0.8 m/s) only shape RMF's schedule. Nav 2's own limits drive the isaacbot.

## Dashboard

`dashboard/` is our app on rmf-web's dashboard framework (`rmf-dashboard-framework`), the same
way rmf-web's own demo dashboard is built. `docker/Dockerfile.dashboard` downloads rmf-web at a
pinned commit (`RMF_WEB_REF`, the `jazzy` branch the jazzy-nightly images come from) and copies
`dashboard/` in as `examples/isaacbot`. It runs the task model tests and a type check, then builds.
The API and trajectory URLs are compiled in from the `API_URL` / `TRAJECTORY_URL` build args.

What it changes from the stock dashboard:
- **Branding:** the official IsaacBot white lockup in the app bar
  (`public/resources/isaacbot-lockup-white.svg`, from `icons/`), the tab title, the favicon (the
  black or white IsaacBot mark, following the browser's colour scheme) and the orange theme (`theme.ts`).
- **Task list:** NEW TASK keeps only Patrol and Custom Compose. The fleet refuses Delivery
  and Clean.
- **Isaacbot window** (Map and Robots tabs, also addable on Custom): connection, drive mode, in or out
  of the fleet, RMF status and battery, with **Manual** and **Automatic** buttons. Manual takes the
  isaacbot out of the fleet. Automatic asks first whether the area is clear, because the isaacbot
  rejoins the fleet and resumes its queued tasks. It polls the fleet adapter's isaacbot control API
  every 2 s; the URL is compiled in from the `ISAACBOT_AMR_API_URL` build arg. The buttons need the isaacbot's
  connector with `setDriveMode` (isaacbot_amr_vda5050 `feature/remote-drive-mode` or later). An older
  connector fails the action, and the window shows that.
- **ISAACBOT TASK:** a button in the app bar opens our own dialog with the isaacbot's task types. It
  dispatches to "any robot" or to a chosen one.
  - *Patrol with pause* visits stops in order and waits at each (0 = drive through), for a
    number of rounds.
  - It is sent as a `compose` task with one phase per stop: `go_to_place`, then a
    `perform_action` `wait`. Jazzy compose tasks have no wait event of their own.

**Adding a isaacbot task:**
1. Add `dashboard/tasks/<name>/` with a `IsaacbotTaskType`: a pure model (`model.ts` with
   `makeDefault`, `validate`, `toRequest`, plus a `model.test.ts`) and a form (`form.tsx`).
2. Add one line to `tasks/registry.ts`.
3. If the robot has to *do* something other than drive (switch an aux output, dock …):
   - add a handler to `isaacbot_amr_rmf_fleet_adapter/application/actions.py`, with its description
     in `domain/actions.py`;
   - add its category to `actions` in `isaacbot_amr_rmf_bringup/config/fleet_isaacbot_amr_world.yaml`.

**Sites imported before an `actions` change** keep their old fleet config, which
`site_io.fleet_config` copied from that template. Re-import them on the sites page, or RMF
refuses tasks that use the new action ("Fleet not configured to perform this action").

## Server deployment (WireGuard)

For permanent use, RMF and the fleet's MQTT broker run on a server, reached at
`<server-vpn-ip>` over WireGuard. The isaacbot's connector dials out to that broker, so RMF keeps working
while the laptop is off. Every port is published on the WireGuard address only.

| Port on <server-vpn-ip> | Service |
|---|---|
| 1883 | Mosquitto with logins: `rmf` (this stack, all of `uagv/#`) and one login per isaacbot, limited to its own `uagv/v2/<manufacturer>/<serial>/#` (Ardennes: `ardennes`, `uagv/v2/IsaacBot/ardennis-3420987-k/#`) |
| 3000 | Dashboard |
| 8010 | api-server |
| 8006 | Trajectory websocket (dashboard map) |
| 8030 | Isaacbot control API (fleet adapter): the dashboard's Isaacbot window |
| 8020 | Sites page |

**On the server**, once:

1. Install Docker Engine with the compose plugin, and start it after `wg-quick@wg0`. Ports bound
   to <server-vpn-ip> fail if `wg0` isn't up yet.
2. Check out this repo and run:
   ```bash
   docker/rmf-server.sh init            # docker/server/.env, broker logins, sites page password (git-ignored)
   docker/rmf-server.sh up -d --build
   docker/rmf-server.sh exec site-manager isaacbot-authorize <user@host> <port>   # per robot, once per release
   ```
   Each robot's page on the sites page shows that command with its own ssh address.

`docker/rmf-server.sh` wraps `docker compose` with `docker-compose.server.yml` and
`server/.env`. That overlay does three things:
- moves every port to `RMF_BIND_IP`;
- switches Mosquitto to `mosquitto.server.conf` + `mosquitto.acl`;
- points RMF at that broker for every site, logging in with `RMF_BROKER_USERNAME` /
  `RMF_BROKER_PASSWORD`.

### Robots

The sites page (`:8020`) lists the fleet's isaacbots. It keeps one list, `robots.json` in the
rmf-sites volume, and both RMF and the broker follow it.

Clicking a robot opens its page (`/robots/<name>`), which has:
- its identity and live state, and its charger and SSH address, both editable;
- **SSH access**: whether the page can read the robot's maps, with the `isaacbot-authorize`
  command for this robot;
- **Maps on this robot**, with Import.

The RMF sites have their own page, `/sites` (**RMF sites** in the header), with Activate and
Delete. They are shared by all robots, because RMF runs one site for the fleet.

Changing a robot's charger restarts RMF. Changing its SSH address does not: `run_rmf.sh`
ignores the `ssh` lines of `robots.json`. A robot with no SSH address, such as a simulated one,
has no maps of its own and uses the site imported from another robot.

On the main page:

- **Add** creates the robot's broker login, which may only use its own topics, and its ACL entry,
  and adds it to RMF's fleet config with its charger. It shows the balena variables, password
  included, **once**.
- **New password** rotates the robot's login. The robot is offline until its balena variable has
  the new password.
- **Remove** drops the robot's login, its ACL entry and its RMF entry. The last robot cannot be
  removed.

Behind it:
- The logins and ACL live in the `broker-auth` volume. On first start they are seeded from
  `server/mosquitto.passwd` and `mosquitto.acl`, so `rmf` and `ardennes` keep their passwords.
- `docker/mosquitto-run.sh` reloads Mosquitto (SIGHUP) when the page changes them, without
  dropping clients.
- `docker/run_rmf.sh` merges `robots.json` into the active site's fleet config, and restarts RMF
  when the list changes (about 10 s, all robots briefly offline in RMF).

The page creates broker credentials, so it asks for a password when `SITE_MANAGER_PASSWORD` is
set. `rmf-server.sh init` generates one, and `rmf-server.sh site-password` prints it, adding
one to an older `server/.env` first (then `rmf-server.sh up -d site-manager`).

To try a second robot before one exists, use `sim_site/`. It is a WireGuard peer at <sim-site-vpn-ip>
with a fake VDA 5050 robot behind it; see [sim_site/README.md](sim_site/README.md).

A second site's router needs its own WireGuard peer. If the server must reach that site's LAN
(the sites page reading maps over SSH), the LAN also needs its own subnet, because the server
routes 192.168.1.0/24 to Ardennes's RUTX11.

**On the isaacbot**, set these balenaCloud variables on `ardennes-vda5050`. For a robot added on
the sites page, the page shows them. Ardennes's initial password comes from
`docker/rmf-server.sh isaacbot-password` until the page rotates it.

| Variable | Value |
|---|---|
| `ISAACBOT_AMR_VDA5050_ENABLE` | `true` |
| `ISAACBOT_AMR_VDA5050_LOCAL_BROKER` | `false` |
| `ISAACBOT_AMR_VDA5050_BROKER_HOST` | `<server-vpn-ip>` |
| `ISAACBOT_AMR_VDA5050_BROKER_PORT` | `1883` |
| `ISAACBOT_AMR_VDA5050_BROKER_USER` | `ardennes` |
| `ISAACBOT_AMR_VDA5050_BROKER_PASSWORD` | the password from `isaacbot-password` |
| `ISAACBOT_AMR_VDA5050_BROKER_TLS` | `false`; WireGuard already encrypts the link |

The isaacbot reaches <server-vpn-ip> through the RUTX11's tunnel. The sites page reaches the isaacbot's ssh
(`<isaacbot-lan-ip>:24`) the other way, which needs a RUTX11 firewall rule for `<server-vpn-ip>`.
Over 4G, VDA `state` (1 Hz) and `visualization` (2 Hz) are the steady load.

## Tests

```bash
colcon build --packages-select isaacbot_amr_rmf_fleet_adapter --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select isaacbot_amr_rmf_fleet_adapter && colcon test-result --verbose
(cd isaacbot_amr_rmf_maps && python3 -m pytest test)  # map importer, map sources, preview (colcon
                                               # needs RMF's building map tools for this one)
python3 -m pytest site_manager/test            # the sites page's server, API and robots
python3 -m pytest sim_site/test                # the fake VDA 5050 robot
```

These unit tests run on the host without RMF. The image build runs them again under Jazzy. The
dashboard's task model tests (`dashboard/tasks/**/*.test.ts`, vitest) and its type check run in
`docker compose -f docker/docker-compose.yml build dashboard`.

## Regenerating the Gazebo map

Edit `WAYPOINTS` / `LANES` in `isaacbot_amr_rmf_maps/scripts/generate_isaacbot_amr_world.py`, then run it. It
reads `isaacbot_amr_world.sdf` and rewrites the PNG and the building file.

## Known limitations

- **Stops at every waypoint.** Each RMF waypoint is its own order, so the isaacbot stops at every
  graph waypoint. Keep the lane graph sparse. The fix would be order stitching (same `orderId`,
  `orderUpdateId + 1`).
- **Final heading.** The isaacbot arrives facing along the path (the connector's
  `orientation_mode: path`), not at the yaw RMF asked for.
- **Missing features:** no docking, doors or lifts, and no battery model in simulation. The
  only perform-action is `wait`.
- **Battery and mechanics are estimates.** They are ASSUMPTION values in the fleet configs.
  They only shape RMF's planning; the real isaacbot reports its own battery.
- **Straight lanes between places.** Imported sites get only these, so a place hidden behind a
  corner needs an intermediate place.

## License

Apache-2.0 (`LICENSE`). `dashboard/main.tsx` is derived from rmf-web's demo dashboard
([open-rmf/rmf-web](https://github.com/open-rmf/rmf-web), Apache-2.0); its header names the
upstream file and commit.
