# Frontier exploration preview in RViz

The preview reads `/map` and `map -> base_link` TF and suggests the next
exploration location. It publishes **MarkerArray visualization only** on
`/lite3_locomotion/exploration_markers`. It has no Nav2 action client, goal-pose
publisher, velocity publisher, SDK, mode-switch interface, or robot transport.
It cannot cause the robot to explore physically.

## View the running preview

Use the RViz instance that already sees the robot's map in ROS domain **8**.
The preview can run on the development computer while RViz runs on another VM
in the same ROS domain; RViz does not need a copy of this Python package.

1. In RViz, set **Global Options -> Fixed Frame** to `map`.
2. Click **Add -> By topic**.
3. Select `/lite3_locomotion/exploration_markers`, with display type **MarkerArray**.
4. Enable the display and its namespaces. Allow a few seconds for the update.

| Marker | Meaning |
| --- | --- |
| Yellow spheres | Candidate free-side exploration approach points |
| Green arrow | Highest-ranked suggested point and heading toward its frontier |
| Blue circle | Current local search radius around the robot |
| White text | `PREVIEW ONLY`, status, candidate count, and visited-cell count |

Do not use the Nav2 Goal tool to display these suggestions. They are drawn as
markers specifically so displaying them does not send a navigation goal.
Markers update every two seconds and expire after five seconds without refresh.

## Run from this workspace if the preview is stopped

The prepared development workspace is on `192.168.10.78`, not the other VM's
home directory. Run the following only there, from this package directory,
after its isolated build. Use one preview instance at a time:

```bash
source .colcon/install/local_setup.bash
PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=8 \
  ROS_LOG_DIR="$PWD/.colcon/exploration/ros_logs" \
  .colcon/install/lite3_locomotion/lib/lite3_locomotion/exploration_preview \
  --radius 5 --clearance 0.30
```

Stop a foreground instance with Ctrl+C. This only stops visualization; it does
not perform a robot stop or mode change. The agent-started preview is managed
by the agent's process session; ask it to stop that instance before starting
your own. No launch files or robot service configurations are changed.

## Selection logic and limits

The pure [frontier selector](../lite3_locomotion/frontiers.py) flood-fills known
free cells from the robot location using four-neighbor connectivity. Values
0–20 are treated as free; unknown -1 and uncertain/occupied 21–100 are not
traversed. Known obstacles are inflated conservatively using square clearance.
Frontiers are reachable free cells adjacent to unknown cells inside the map;
the map's outer edge alone is not counted. Connected frontier groups smaller
than three cells are discarded. A candidate is placed back from the frontier
where its clearance square contains only known free cells. Groups are ranked
by frontier extent divided by one plus grid path distance.

The default radius is five metres, adjustable up to ten. It moves with the robot
and is **not a fixed mission geofence**. The default clearance is 0.30 m, a
preview assumption rather than a calibrated robot footprint. Limits are one
million map cells, 200,000 local-table cells, 30,000 BFS cells, 200 clusters, and
50 published candidates. Hitting a computation budget clears goal suggestions.

The selector supports a planar rotated map origin. The ROS adapter requires
the map frame `map` and robot frame `base_link`, normalized planar map-origin
quaternions, a map received within 30 seconds, and a latest robot transform no
more than two seconds old (up to 0.5 seconds future clock tolerance). Map receipt
time is not proof of sensor-data freshness. This preview uses wall/ROS system
time and does not implement simulated-clock configuration.

Grid connectivity and clearance do not certify an executable path. The BFS may
pass beside unknown space even though selected goals have known-space clearance.
There is no 3D terrain analysis, slope/step/drop-off detection, Nav2 feasibility
check, goal-progress tracking, failed-goal blacklist, or complete exploration
mission loop yet. The preview can repeat a suggestion while the robot remains
stationary. No unexplored area is declared safe by this tool.

## Troubleshooting status

| Status | Meaning |
| --- | --- |
| `waiting_for_map` | No compatible `/map` sample received yet |
| `invalid_map` | Frame, geometry, occupancy data, or cell budget rejected |
| `map_receipt_stale` | No accepted map sample in the last 30 seconds |
| `waiting_for_map_to_base_link_tf` | Required transform is unavailable |
| `robot_transform_stale_or_invalid` | Pose is stale, nonfinite, or clocks disagree |
| `robot_not_in_known_free_cell` | Robot pose lies outside the map or on a nonfree cell |
| `robot_clearance_insufficient` | Configured known-obstacle clearance fails at the robot |
| `no_frontiers_in_preview_area` | No suitable candidate within this radius; not proof that the world is fully explored |
| `*_limit_exceeded` | Work budget reached; no partial goal is published |

Do not restart robot services to clear these messages. Check the existing map,
TF, domain, clocks, and RViz display configuration first.

## Verification

Offline tests cover disconnected regions, obstacle walls, unknown/uncertain
cells, goal clearance, rotated origins, invalid maps, search budgets, radius,
and argument/remapping rejection. A live **visualization-only** check in domain
8 discovered the map and fresh robot TF and produced four candidates after
6,553 visited cells. Graph inspection confirmed one MarkerArray output plus
standard parameter events, with no action clients, command publishers, or
service interfaces. This confirms preview operation, not physical autonomy.
