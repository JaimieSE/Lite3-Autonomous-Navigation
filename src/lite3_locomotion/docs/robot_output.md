> Historical standalone-stack workflow. Its motion entry points are retired from
> the current installation. Use the [current architecture](autonomy_stack.md)
> and [change audit](simplification_audit.md); these earlier steps are not the
> current startup procedure.

# Supervised robot-output commissioning

The new `motion_sender` implements actual UDP body-velocity transport following
the pinned vendor source, NOT verified deployed firmware. It retains the vendor
gait controller. No posture, joint, gait or Navigation/Manual-mode commands are
implemented. No physical motion has been performed during development.

## Connection and limits

```text
LiDAR/IMU -> existing Fast-LIO + SLAM -> /map, /scan, /odom, TF
  -> frontier coordinator + Nav2 -> guarded adapter (explicit robot output)
  -> /lite3_robot_link/cmd_vel -> motion_sender (explicit enable) -> motion host
```

The sender is a separate process, fixed to `/lite3_robot_link/motion_sender`.
It requires an explicit IPv4 target and UDP port; no robot address is guessed.
Non-loopback output additionally requires `--allow-robot-output`. Startup opens
a socket but sends **nothing until enabled and a valid command arrives**. It
creates no telemetry receiver or sensor driver.

Limits: x +/-0.03 m/s, ROS yaw +/-0.10 rad/s. These are commissioning placeholders,
not measured robot capabilities. Lateral, vertical, roll, pitch, NaN and infinity
commands latch stop. Wire layout is little-endian `<iiid`: x code 320, zero lateral
325, negative ROS yaw 321, each with size 8 and type 1. Firmware ABI must be checked.

A monotonic watchdog worker separate from the ROS executor sends at 20 Hz. It
latches disabled at command age >=0.25 s, graph-check age >=0.5 s, unexpected
command endpoints, socket errors or session deadline (default 30 s, maximum 60 s).
First input is required within 10 s of enabling. Repeated enable while enabled
does not extend the session. Adapter disarm following an observed armed heartbeat
also disables the sender. After transmitting, stop attempts zero triplets for one
second then goes silent. The adapter requires a fresh enabled-sender heartbeat
before it will arm in robot mode. Each enable creates a new session token; a
fresh explicit adapter arm must acknowledge that token before any commands are
accepted. Old acknowledgments and disarmed adapter zeros cannot authorize a new
session. Only one guarded-adapter publisher and the
sender's own subscription are permitted on the command topic. ROS names are
wiring checks, not authentication; receipt-time expiry cannot detect replay.

## Remaining physical prerequisites

Verify the motion endpoint, firmware ABI, velocity units/signs and zero semantics.
Stand the robot using the manufacturer's documented procedure and establish its
compatible Navigation mode. Arming services do not verify posture or mode. The
earlier [interface audit](vendor_interface_audit.md) records that the lab's
Navigation mode disables handheld walking control; verify independent emergency
stop and Manual handback instead of assuming remote walking control remains.

The watchdog can handle loss of Nav2/adapter commands or a stalled ROS executor
**only while the sender process and host remain functioning**. SIGKILL, a frozen
process, power loss, network failure or firmware rejection can prevent stopping.
An independent, verified motion-host command timeout/stop mechanism is required
before autonomous walking. This change does NOT install one on the motion host.
Successful UDP sending is not motion-host acknowledgment or proof of stopping.

The 0.4 m goal clearance, 0.36 m footprint and obstacle guard remain unchanged.
The earlier 0.2 m preview is NOT a physical operating setting. No eligible goals
means stop; there is no blind forward/spin fallback. Flat-floor exploration only;
no stairs, drop-off or terrain detector is included.

## Build and isolated verification

```bash
cd /home/user/lite3_ws/src/lite3_locomotion
source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 colcon --log-base "$PWD/.colcon/log" build \
  --base-paths "$PWD" --packages-select lite3_locomotion \
  --build-base "$PWD/.colcon/build" --install-base "$PWD/.colcon/install"
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider test
PYTHONDONTWRITEBYTECODE=1 python3 tools/run_motion_sender_integration.py
```

The integration runner uses a random domain 40-89 and loopback-only DDS and UDP.
It never contacts a robot and does not certify physical operation.

Verified locally on 2026-11-12: isolated package build passed; ordinary suite
268 passed with 22 opt-in cases skipped; sender integration 5 passed. The latter
includes actual Nav2 -> robot-mode adapter -> localhost UDP, startup silence,
speed/packet limits, command timeout, invalid input, extra-publisher rejection,
software stop, re-arming handshake and bounded shutdown zeros. No robot packets,
physical movement or mode changes were sent during these tests.

## Explicit commissioning sequence (not run during development)

Keep domain-8 sensor bridging, Fast-LIO and SLAM running. Do not launch the full
vendor transfer stack or duplicate drivers. Stop only the old autonomy launch
before selecting robot output. First establish a usable map, scan, TF and goals
at the original clearance with the robot standing. A supervised small-motion test
and verified zero/timeout stopping must precede autonomous walking.

In every new terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/user/lite3_ws/src/lite3_locomotion/.colcon/install/local_setup.bash
export ROS_DOMAIN_ID=8
```

After verifying the prerequisites above, start the sender in its own terminal.
Replace both placeholders with verified deployment values:

```bash
ros2 run lite3_locomotion motion_sender \
  --target-ip VERIFIED_MOTION_IPV4 --target-port VERIFIED_UDP_PORT \
  --allow-robot-output --session-seconds 30
```

In another terminal start Nav2 and its gate, still disarmed:

```bash
ros2 launch lite3_locomotion autonomy.launch.py \
  output_mode:=robot radius:=3 session_seconds:=30
```

Wait for active Nav2 lifecycle nodes. Adapter health reports sender disarmed
until it is enabled. Enabling alone sends no packets, but the subsequent start
can produce real movement. Run together from a sourced terminal only once all
physical prerequisites hold:

```bash
ros2 service call /lite3_robot_link/enable std_srvs/srv/Trigger '{}'
ros2 run lite3_locomotion autonomy_control start
```

Start within 10 seconds of enabling. Both requests must succeed. Monitor
`/lite3_robot_link/status`, `/lite3_locomotion/guard_status` and
`/lite3_locomotion/exploration_status`. No frontiers still stops the mission.

For a software stop, request both gates (run both even if one fails):

```bash
ros2 run lite3_locomotion autonomy_control stop
ros2 service call /lite3_robot_link/stop std_srvs/srv/Trigger '{}'
```

These acknowledge local handling only, NOT physical stopping or Manual-mode
handback. Use the verified independent manufacturer emergency stop when needed.
Ctrl+C, stopping DDS and killing the sender are not independent emergency stops.
