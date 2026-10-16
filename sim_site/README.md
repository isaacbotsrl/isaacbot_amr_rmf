# Simulated second site (<sim-site-vpn-ip>)

Tests the server side with a second isaacbot before a real one exists. It runs on this laptop
with Docker:

```
fake_vda_robot ──MQTT──► rutx11-sim (WireGuard peer <sim-site-vpn-ip>) ══tunnel══► server <server-vpn-ip>:1883 ──► RMF
```

- **`rutx11-sim`** stands in for the site's RUTX11. It is a WireGuard client of the server with
  its own VPN address, so the robot reaches the broker the way a real one does: through its
  router's tunnel, not through the laptop's own VPN address (<laptop-vpn-ip>).
- **`robot`** runs `fake_vda_robot.py`, which shares `rutx11-sim`'s network. It speaks VDA 5050
  2.0 the way Ardennes's connector does, as far as the RMF fleet adapter can tell:
  - **Publishes:** connection, factsheet, state at 1 Hz, visualization at 2 Hz.
  - **Orders:** drives each order's nodes in straight lines.
  - **Instant actions:** handles cancelOrder, startPause/stopPause and setDriveMode.
  - **Leaves out:** no ROS, no Nav 2, no map, no LAN behind the router.

It covers the VPN peer, the robot's own broker login and ACL, and RMF running two robots. It
does not cover navigation, map import, or the site manager reading maps from the robot over SSH.

## Steps

1. **Add the robot on the sites page.** Open `http://<server-vpn-ip>:8020` (password:
   `docker/rmf-server.sh site-password` on the server), go to **Robots**, and add:
   - RMF name `ardennes_e`
   - serial `ardennis-3420987-e`
   - manufacturer `IsaacBot`
   - charger `south` (on the built-in `isaacbot_amr_world` site)

   Copy the password the page shows once. RMF restarts with the new robot.

2. **Make the site's WireGuard peer.** This is done on the server and needs sudo:
   ```bash
   ssh -t <server-user>@<server-vpn-ip> 'cd wireguard && sudo ./wg-add-client.sh rutx-11-sim split'
   ```
   It takes the lowest free address (the next one after the existing peers) and prints the client
   config. Save that config as `sim_site/secrets/wg0.conf`.

3. **Give the robot its login:**
   ```bash
   cp sim_site/robot.env.example sim_site/secrets/robot.env   # then set BROKER_PASSWORD
   ```
   The start pose in the example is the `south` waypoint in the robot's map frame. Use another
   waypoint for other sites.

4. **Run it:**
   ```bash
   docker compose -f sim_site/docker-compose.yml up -d --build
   docker compose -f sim_site/docker-compose.yml logs -f
   ```
   Check the logs for both of these:
   - `rutx11-sim: tunnel up as <sim-site-vpn-ip>/32` and `server <server-vpn-ip> reachable`
   - `fake_vda_robot: connected to <server-vpn-ip>:1883 as uagv/v2/IsaacBot/ardennis-3420987-e`

5. **Check RMF.**
   - On the sites page, `ardennes_e` shows `AUTOMATIC · 80 %`.
   - The dashboard (`http://<server-vpn-ip>:3000`, Robots) lists both robots.
   - A go-to-place task for `ardennes_e` makes the fake robot drive, as shown by its logs and its
     position on the map.

`secrets/` is git-ignored: it holds the WireGuard private key and the broker password.

## Removing it

```bash
docker compose -f sim_site/docker-compose.yml down
ssh -t <server-user>@<server-vpn-ip> 'cd wireguard && sudo ./wg-remove-client.sh rutx-11-sim'
```

Then remove `ardennes_e` on the sites page: its login stops working and RMF restarts without it.

## Tests

```bash
python3 -m pytest sim_site/test    # the fake robot against the fleet adapter's VDA 5050 code
```
