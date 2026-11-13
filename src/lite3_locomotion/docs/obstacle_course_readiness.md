# Obstacle-course readiness: perception, gaps and roadmap (2026-11-13)

The goal is for the Lite3 to enter an obstacle course and complete it on its own.
The course has terrain, walls, curtains and stairs. Everything below comes from
read-only inspection of the live robot (domains 8 and 64), plus short,
**perception-only** runs of the university bringup and the new terrain monitor.
No motion was commanded during this assessment.

## Sensors that are accessible now

| Sensor / stream | Status (measured) | Use |
| --- | --- | --- |
| Livox Mid-360 `/livox/points`, `/livox/lidar` | Live, 10 Hz, about 16k points per frame. Mounted 0.187 m forward and 0.125 m up from `base_link`, pitched 15° down. Elevation −6° to +53° | The main sensor for walls, obstacles and terrain |
| Mid-360 IMU `/livox/imu` | Live, 200 Hz | Fast-LIO input |
| Robot IMU, joints, leg odometry | Live, 200 Hz | Attitude, posture, floor height from leg geometry |
| Ultrasound `/us_publisher/ultrasound_distance` | Live, 200 Hz (read 0.80 m) | One range value; direction not yet verified |
| Orbbec camera `/image_raw`, `/camera_info` | **Offline**: `device_online=False`, no frames | Would cover the LiDAR's near-field blind zone |
| Handheld remote `/handle_state` | Live | Operator-takeover detection |
| Static TF `base_link → livox_frame` and `→ camera_link` | Available (bridged) | Sensor extrinsics |

The university perception (`lite3_bringup.launch.py`: Fast-LIO, `odom_relay`,
`pointcloud_to_laserscan`) was run briefly. It worked on live data:

- `/odom` and the `odom → base_link` TF at 10 Hz
- `/scan` at 10 Hz
- registered clouds

It publishes no commands. It is **not** running by default in the group domain,
and neither are Nav2 or SLAM.

## Can the robot sense its surroundings?

**Yes for walls and large obstacles, but not yet for terrain.**

- **Walls and tall obstacles:** Fast-LIO and `/scan` give localization plus a 2D
  obstacle band that Nav2 can use.
- **Steps, stairs, curbs, low obstacles and drop-offs: invisible to the university
  stack.** Its `/scan` keeps only points 0.1–0.5 m above `base_link`. That is
  about 0.43–0.83 m above the floor when standing, and the config itself marks
  this as a "TODO: tune".
- **A near-field blind zone exists in every scan.** The 15° tilt and −7° lower
  field of view mean the floor closer than about 0.7 m (lying) or 1.1 m
  (standing) ahead is never seen, and almost nothing behind is seen.
- **Curtains look like walls.** The LiDAR returns them as solid surfaces, so Nav2
  will route around them or refuse to pass.

## Developed now: `terrain_monitor` (perception only)

Code: `lite3_locomotion/terrain.py` (pure numpy) and `terrain_monitor.py` (ROS).
Run it with `ros2 run lite3_locomotion terrain_monitor`.

It builds a gravity-aligned 6 × 6 m elevation grid at 0.05 m resolution from
the existing cloud. The IMU removes tilt. The floor reference comes from the
validated leg geometry and is refined by LiDAR floor points when the two agree.
It accumulates the last 4 s of scans whenever the university `odom → base_link`
TF exists.

Each cell is labelled as one of:

| Label | Meaning |
| --- | --- |
| `free` | Floor |
| `step` | A rise of 0.04–0.10 m |
| `obstacle` | A rise above 0.10 m, up to the 0.75 m body clearance |
| `drop` | More than 0.10 m below the floor |
| `overhead` | Points only above the body clearance |
| `unknown` | Not observed |

Outputs:

- `/lite3_terrain/grid`: an `OccupancyGrid`
- `/lite3_terrain/hazards`: obstacle and drop cells as a `PointCloud2` in
  `base_link`, ready to add as a Nav2 costmap observation source
- `/lite3_terrain/status`: JSON with label counts and a forward-corridor summary
  (nearest obstacle, step and drop, clear distance, near-field unknown fraction)

There are no command, service or mode interfaces; a test enforces this.
Unobserved space is never reported as free.

Validation:

- **Synthetic scenes (15 tests):** wall, low step, stair riser, drop-off,
  curtain, overhead structure, blind zone, extrinsic and tilt correctness,
  self-filter, and accumulation.
- **Live, robot lying:**
  - processing takes 6 ms per frame (28 ms with accumulation)
  - the LiDAR floor estimate (−0.056 m) agrees with the leg estimate (−0.081 m)
  - an obstacle was detected 2.18 m ahead
  - single-scan unknown cells: 86%, falling to 49% with 4 s of accumulation
  - near-field unknown fraction fell from 0.59 to 0.34

**Not validated:** the step and drop limits (0.10 m) are placeholders until the
Lite3's real climbing capability is measured. No walking data has been used yet.

## What is needed, in order

1. **Locomotion envelope (physical, with `lite3_movement_tests`).**
   - Forward walking has produced 50% of the commanded displacement at 0.03 m/s.
     Run speed sweeps (0.03–0.10 m/s), turn in place, and walk backward.
   - Add a "silence stop" test: stop publishing and verify rest. Twist_mux stops
     publishing after its 0.5 s timeout, and the vendor sender has no watchdog, so
     the firmware's behavior on silence decides whether a crashed planner leaves
     the robot walking.
   - Then set the Nav2 velocity limits to the validated envelope. The university
     MPPI currently allows vx 0.5, vy 0.5 and wz 1.9, none of which is validated.
2. **Feed terrain into Nav2 without editing university files.**
   `lite3_navigation.launch.py` accepts `params_file`. Add a project-owned
   overlay that does three things:
   - adds `/lite3_terrain/hazards` as an observation source
   - lowers the `/scan` band so it starts just above the floor
   - caps velocities
   Validate it with the coordinator's dispatch disabled first.
3. **Mapping for an unknown course.** Run the university launch in SLAM mode
   (`slam:=True`) so `/map` exists for the frontier coordinator.
4. **Camera.** Bring the Orbbec online (university-owned driver) for near-field
   depth at 0.3–1 m ahead, where the LiDAR is blind. This matters for stairs and
   drop-off edges.
5. **Stairs.** We need the manufacturer's stair-gait or gait-switch commands and
   their verified acknowledgment. The vendor `/simple_cmd` codes known here cover
   only stand/sit and Manual/Navigation mode. We also need a stair-approach
   behavior: align, switch gait, climb, verify by IMU pitch and odometry, then
   switch back. Without this, stairs must be treated as obstacles.
6. **Curtains.** We need a course rule (must the robot push through?) and a
   policy. One option is a known curtain region, or a sensed "soft"
   classification, followed by a slow, bounded push-through with tilt and
   progress monitoring. Today a curtain is a wall.
7. **Supervised integration.**
   - Nav2 plans while dispatch is disabled.
   - Then single short goals at the validated speed.
   - Then frontier or course missions, with geofence, tilt and abort checks.
8. **Independent stop.** Before unattended runs, confirm the e-stop and the
   firmware command-loss behavior (item 1).

## Information needed from the operator / course

- Stair rise and run, number of steps, handrails, and whether stairs are mandatory.
- Whether curtains must be pushed through, and their positions if known.
- The terrain types (gravel, ramps with slope, foam, grass?).
- Whether the start and finish positions are known, or the robot must explore.
- Time limits, and whether a human may intervene.
- Access to the manufacturer gait and stair documentation for this firmware.
