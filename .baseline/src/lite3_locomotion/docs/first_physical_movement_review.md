# Lite3 Venture: first physical movement review and proposed test

Report prepared: 2026-11-12, 12:14 UTC; live checks earlier in this review. Environment: `group8`,
onboard LXC, ROS 2 Humble, workspace `/home/user/lite3_ws`, live ROS domain 8.

**Decision: NOT READY FOR PHYSICAL EXECUTION.** This is a findings report and
conditional test plan, not authorization or an executable movement procedure.
No movement/zero-velocity commands were published to physical topics, no robot
control modes were changed, no motion bridge was launched, and no services were
restarted. Runtime code and existing configuration were not modified. The
attached autonomy-stack snapshot was not edited.

## Evidence and scope

Read-only checks included live node/topic/service discovery, endpoint ownership,
guard/mission status, a five-second odometry observation, process/socket/route
inspection, package source review, historical integration results, and public
manufacturer documentation. The diagnostic observer subscribed only to odometry;
it did not create motion publishers or call control services.

ROS discovery is limited to endpoints visible from this container in domain 8.
It cannot exclude controllers in other domains, other containers/hosts, direct
UDP senders, the manufacturer's app, or embedded behavior. Discovery and node
names are not authentication. Observations must be repeated before any test.

## 1. Active velocity subscriber: exact live result

| Topic | Message type | Publisher | Subscriber | Interpretation |
| --- | --- | --- | --- | --- |
| `/lite3_autonomy/nav_cmd_vel` | `geometry_msgs/msg/Twist` | `/lite3_autonomy/controller_server` | `/lite3_guarded_adapter` | Nav2-to-local-guard input, not a demonstrated motor connection |
| `/lite3_locomotion/guarded_test_cmd_vel` | `geometry_msgs/msg/Twist` | `/lite3_guarded_adapter` | None observed | Dry-run output; no discovered forwarding consumer |
| `/handle_state` | `geometry_msgs/msg/Twist` | `/lite3_domain_bridge_8` | None observed | Incoming controller telemetry; NOT established as a motion input |
| `/cmd_vel` | No discovered topic | None discovered | None discovered | No basis to assume a motor path |
| `/lite3_robot_link/cmd_vel` | No discovered topic | None discovered | None discovered | Proposed sender connection is not running |

**No active motor-facing velocity subscriber was identified.** The identified
velocity-command subscriber is the local guard, which is running dry-run.
An independent rclpy graph inspection confirmed the three visible Twist topics
above; no TwistStamped velocity topic was discovered.

The live guard reported `armed=false`, `reason=operator_stop`,
`health=healthy`, `output_mode=dry-run`. The coordinator reported `running=false`,
`reason=no_eligible_frontiers_in_session_area`, `goals_sent=0`, `nav2_active=true`.
"Healthy" here means the implemented dry-run checks passed, not that physical
transport, firmware mode, emergency stopping or exclusive authority is ready.

## 2. Trace to an existing Lite3 motion bridge

The live path currently ends here:

```text
controller_server -> /lite3_autonomy/nav_cmd_vel -> lite3_guarded_adapter
  -> /lite3_locomotion/guarded_test_cmd_vel -> no discovered subscriber
```

`/lite3_domain_bridge_8` publishes robot telemetry and static TF. Its only
reported subscriber is `/parameter_events`; it reports no command services.
It is not demonstrated to forward outbound movement. Its upstream host, source
domain, executable/configuration and upstream motion sender remain unidentified.

No local `motion_sender`, `jetson2motion`, or `domain_bridge` process matching
those executable names was found; `transfer` was not found in the sourced base
ROS package index. No matching listener on UDP 43893/43897/43899 was visible in
this network namespace. These checks do not establish absence on the LXC host
or robot motion host.

Source evidence, not deployment evidence:

- The pinned [vendor sender](../vendor/Lite3_ROS/src/transfer/src/Jetson2Motion.cpp)
  subscribes to relative `cmd_vel` and `cmd_vel_corrected` (Twist, best-effort,
  depth 1), plus `simple_cmd` and `complex_cmd`. Namespace/remapping determines
  their actual deployed names. Both velocity inputs and both command channels
  must be included when checking competing sources.
- Its callback sends UDP codes 320/325/321 for forward/lateral/negative ROS yaw,
  with double payloads. Its IP/port parameters default to `192.168.1.120:43893`.
  A source default is not this robot's deployed configuration.
- The new [local sender](../legacy/lite3_legacy/motion_sender.py) expects
  `/lite3_robot_link/cmd_vel`, requires an explicit target and opt-in for
  non-loopback output, but is not active on the live graph.

The [manufacturer communication specification](https://developer.deeprobotics.cn/docs/lite3/guide/communication/architecture/)
documents a ROS-to-UDP message transformer and target port 43893. The
[network instructions](https://developer.deeprobotics.cn/docs/lite3/sdk/motion-sdk/preparation/getting-started/)
list standard motion-host addresses `192.168.1.120` (wired) and `192.168.2.1`
(Wi-Fi subnet 2). Neither address is established as this unit's active endpoint.

This container has `192.168.1.78/24` and `192.168.10.78/24`. The former has a
direct route to the wired candidate; the user-reported app/remote Wi-Fi address
`192.168.2.235` routes through `192.168.10.1`. Neither routing nor an SSH banner
would establish UDP protocol identity, mode or command ownership. Do not change
remote Wi-Fi settings or probe candidate addresses with movement commands.

**Blocker B1:** obtain the deployed motion bridge's resolved command endpoints,
target, firmware/protocol identity and upstream domain/host. Possible evidence:
authorized read-only host configuration or narrowly filtered passive capture.
Prior packet-capture attempts lacked permission and passwordless sudo; do not
bypass this through privileged containers. Capturing the wrong namespace may
also miss the relevant traffic.

## 3. Command authority

Locally, Nav2 is the only discovered publisher on the guard input, and the
guard is disarmed. This does NOT prove the dog is under no other control.
`/handle_state` is telemetry, not a command-authority acknowledgment. No reliable
active-owner or Navigation-mode feedback was found in the exposed interfaces.

**Blocker B2:** identify the session's sole authorized motion operator and all
upstream command sources, including app joysticks/voice/following functions,
other groups, ROS domains, and direct SDK/UDP clients. Obtain an operator/host
owner agreement that they will not send motion or change modes during the test.
Do not disable someone else's processes, rely on node-name checks as exclusion,
or assert ownership from a quiet command topic.

## 4. Autonomous-control mode and authorization

The [manufacturer advanced interface](https://developer.deeprobotics.cn/docs/lite3/interfaces/control/advanced/)
distinguishes Navigation mode (perception-host velocity) from Manual mode
(remote-controller velocity), and documents Navigation `0x21010C03` and Manual
`0x21010C02`, type 0. It also describes a connection heartbeat at least 2 Hz.
That heartbeat is not by itself proof of command-loss stopping.

The package's arm/enable services authorize LOCAL software gates only. They do
not establish manufacturer mode, standing posture, controller ownership, or a
physical acknowledgment. They do not send the manufacturer mode commands.

**Blocker B3:** determine how this Venture's actual firmware authorizes high-level
velocity control, how that mode is observed/confirmed, which app or host owns it,
and how Manual handback is confirmed. Do not assume a LiDAR/Pro app menu applies
to Venture, or that its handheld walking joystick remains effective in Navigation
mode. No mode command should be attempted before explicit reviewed approval.

## 5. Limits and frequency: distinguish interfaces

The [manufacturer control interface](https://developer.deeprobotics.cn/docs/lite3/interfaces/control/state/)
documents these high-level velocity-interface ranges: x +/-1.0 m/s, y +/-0.5 m/s,
yaw +/-1.5 rad/s, using double payloads in Navigation mode. These are interface
ranges, NOT safe first-test limits or a verified operating envelope for this unit.

That same reference specifies >=20 Hz and a 250 ms loss timeout for the distinct
integer joystick/AXIS interface. It does not explicitly attach that guarantee to
the double-valued Navigation velocity commands. A 50 Hz robot-state upload rate
is telemetry frequency, not a velocity-command requirement.

| Layer | Source-reviewed configuration | Status |
| --- | --- | --- |
| Nav2 controller | 10 Hz; nominal desired x 0.10 m/s | Local planning settings |
| Guard adapter | 20 Hz timer; x +/-0.10, y zero, yaw +/-0.20; input expiry 0.40 s | Local software policy |
| New UDP sender | Approximately 20 Hz triplets (each axis 20 Hz); x +/-0.03, y rejected if nonzero, yaw +/-0.10; expiry 0.25 s | Local commissioning placeholders |
| Pinned vendor sender | Callback-driven; no sender-side rate limiter or expiry | Source review, not proven deployed |

**Blocker B4:** confirm firmware support, units/signs, minimum useful command,
velocity-command refresh/heartbeat requirements and loss behavior for the exact
Navigation interface. Do not substitute joystick timeout or joint-SDK watchdog
claims. Do not automatically increase a test speed if the robot does not move.

## 6. Emergency stop and lost communication

Two manufacturer-authored manual revisions illustrate why the actual equipment
must be matched:

- [Venture/Pro/LiDAR V1.0.3-0 (2023), manufacturer manual hosted by distributor](https://static.rapidonline.com/pdf/35-9001m_v1.pdf)
  describes app STOP as soft emergency protection that lowers the robot.
- [Venture V2.0.0-0 (2026), manufacturer manual reproduced online](https://device.report/m/e936d1b1f7d28a82be179d66824c7a95ed4e869d0ec9443ac871f447aace27b0)
  describes a separate Emergency STOP Controller with a push-to-stop,
  twist-release button. It also references app soft STOP.

Neither revision establishes which hardware and firmware this dog has. The user
has identified an app STOP; a separate physical emergency controller has NOT
been established. App STOP may be independent of the LXC process, but its path
and operation when Wi-Fi/host connectivity is lost are not verified. Do not
classify it as a hardware safety-rated or independent link-loss stop without
evidence. STOP may lower/shut down the dog rather than leave it standing.

The pinned vendor sender has no independent command-expiry timer or shutdown
zero. The new sender's worker detects lost input/graph heartbeat and requests a
one-second zero burst, but cannot survive its own process/host loss or deliver
packets over a failed network. No motor-controller acknowledgment is checked.

**Blocker B5:** match the installed emergency-stop equipment/manual, establish
its override of Navigation commands and failure dependencies, and document the
motion-host loss policy. If a prescribed stationary/secured emergency-stop check
is needed, it is a separately approved physical action: lowering also counts as
movement. Do not first learn the emergency procedure while the robot is walking.
An unverified app button alone is insufficient for this supervised test's
independent emergency path. Do not approach moving legs or remove the battery
from an uncontrolled moving robot as an improvised stop.

## 7. Stop confirmation: actual telemetry observation

A roughly five-second passive observation produced:

| Metric | `/odom` | `/leg_odom2` |
| --- | --- | --- |
| Owner | `/odom_relay` | `/lite3_domain_bridge_8` |
| Source meaning | Fast-LIO pose; twist computed by finite differences | Bridged robot telemetry; upstream not traced |
| Samples / receipt rate | 37 / approximately 9.97 Hz after initial discovery | 935 / approximately 186.51 Hz |
| Frame / child frame | `odom` / `base_link` | Both empty |
| Last header age against ROS time | Approximately 0.029 s | Approximately 1,794,480,421 s: incompatible time base |
| Maximum receipt gap | 0.180 s | 0.327 s |
| Maximum absolute vx | 0.0683 m/s | 0.0641 m/s |
| Maximum absolute vy | 0.0763 m/s | 0.0258 m/s |
| Maximum absolute yaw rate | 0.0712 rad/s | 0.00565 rad/s |
| Diagonal of observed x/y range | 0.026 m | 0.000 m |

These numbers are NOT a declaration that the robot moved or stopped. There was
no synchronized physical observation, and peaks may include estimator noise.
The two streams are not yet a validated independent agreement. Blank frames,
incompatible timestamps, surprisingly high bridged rate, and zero pose span
alongside nonzero velocity require provenance/clock/scale checks. Do not fix
freshness merely by replacing message timestamps with the current time.

The source of `/odom` is [the existing odometry relay](../../../../ros2_ws/src/lite3_launcher/scripts/odom_relay.py).
Use continuous odom rather than potentially jumping map localization to measure
short travel. Fresh receipt alone cannot rule out replay or stuck telemetry.

**Proposed stop acceptance, pending calibration:** collect at least 10 seconds
of operator-confirmed stationary data, validate frames, timestamps, actual sample
progression and units, and pre-register filtered speed/pose criteria. Candidate
criteria are planar filtered speed <0.01 m/s and absolute filtered yaw rate
<0.02 rad/s for 2 continuous seconds, with advancing, <=0.25 s-old samples and
<=0.25 s receipt gaps. A 0.5 s filter window is a candidate, not a certified
detector. These criteria must first pass a stationary baseline and be consistent
with pose stability and direct observation; do not mark stale/stuck zero data as
stopped or loosen thresholds merely to pass.

The operator must independently report no body translation/rotation or continued
walking. Ordinary balance motion must not be confused with commanded travel.
Record stop-command time and actual settling time. App/service acknowledgments,
zero output, disarmed flags, and successful UDP sends are not a verified stop.

**Blocker B6:** validated stop observation and an agreed settling/escalation
deadline are absent. The proposed 0.015 m travel is smaller than the sampled
0.026 m pose range, so odometry alone cannot validate that tiny motion reliably;
use a suitable independent video/floor reference and operator observation.

## 8. Package suitability review

Useful building blocks exist: finite-command validation, speed clamping,
disarmed startup, freshness/scan/TF checks, graph wiring checks, two-gate session
tokens, watchdog worker, and bounded zero attempts. Historical results record
268 ordinary tests and five sender/end-to-end localhost cases, plus six Nav2
dry-run integration cases. Existing result files confirm the integration passes;
these are previous localhost/synthetic tests, not new physical verification.
No integration runner or robot-output executable was launched for this review.

Blocking gaps for the requested first movement:

1. [bounded_test.py](../legacy/lite3_legacy/bounded_test.py) implements a 0.5 s,
   0.03 m/s pulse only as OFFLINE intents. Its duration cap is 1 s; the earlier
   proposed two-second test is outside that sequencer's existing bounds.
2. [autonomy_control.py](../lite3_locomotion/autonomy_control.py) starts frontier
   exploration, not a forward-only timed pulse. Do not use it as a first-pulse
   command. Nav2 may turn or send a different sequence, or find no goals.
3. [motion_sender.py](../legacy/lite3_legacy/motion_sender.py) bounds a session
   (default 30 s), not a pulse measured from first nonzero output. It permits
   reverse/yaw within limits and has no measured stop detector or mode feedback.
   `physical_stop_verified` is explicitly always false.
4. [guarded_adapter.py](../legacy/lite3_legacy/guarded_adapter.py) permits only the
   expected Nav2 publisher in robot mode. An ad-hoc test publisher will fail the
   check. Do not spoof `controller_server` or remap around the guard to make a
   pulse work. A separately reviewed commissioning path is needed.
5. Offline handback emits zero/Manual intents; live Manual-mode transmission and
   confirmation are not implemented. Software stop responses acknowledge local
   handling, not a physical result.
6. Neither graph names nor session tokens establish exclusive physical control
   against other hosts/direct UDP. No independent host-failure stop is added.
7. [deployment_candidate.json](../review/deployment_candidate.json) is historical,
   review-only material; its `live_transport_implemented=false` predates the new
   sender and is stale. Its unverified deployment/stop findings remain unresolved.

**Blocker B7:** a live, explicitly approved one-shot commissioning runner with
forward-only output, hard pulse deadline, latched completion, stop monitoring
and reviewed integration into the guards does not yet exist. This report does
not implement or activate one. Earlier statements that the movement software
was complete applied to localhost transport, not physical commissioning.

## Proposed first-movement plan — conditional, NOT APPROVED

### A. Close blockers before requesting execution approval

Resolve B1–B7 with recorded evidence. Establish the exact Venture firmware and
protocol, live outbound path, sole authority, standing/mode confirmation,
independent emergency method and validated stop monitor. Implement/review the
separate commissioning runner and test its whole path against localhost,
including deadlines, loss, late callbacks, stops and no automatic retry.

Before approval, review one concrete manifest: host/domain, exact resolved topic
and message, subscriber owner/executable, IP/port/protocol, authority owner,
mode procedure, pulse parameters, deadline enforcement, zero burst, measured
stop criteria, escalation time and named safety observer/emergency control.
Leave unknown fields explicitly unresolved. Do not approve only a guessed IP.

No fresh physical preparation is assumed from earlier conversational answers;
the operator must reconfirm readiness immediately before an approved trial.

### B. Proposed parameters for that review

| Item | Proposal |
| --- | --- |
| Motion | Forward only; all other linear/angular components zero |
| Forward request | +0.03 m/s, subject to confirmed units/support |
| Nonzero duration | **0.5 s**, not 2 s; no automatic repeat |
| Output schedule | Candidate 20 Hz per axis, subject to exact velocity-interface requirement |
| Nominal travel | 0.015 m; NOT guaranteed travel or stopping-distance bound |
| Producer-gap abort | >=0.20 s; monotonic clock; reject regressing clocks |
| Local sender expiry | Existing 0.25 s; not a substitute for the motion-host watchdog |
| Completion | Latch nonzero output prohibited; explicit review required for another trial |
| Zero phase | Candidate at least 1 s at the agreed command rate, with all three axes zero |
| Normal stop settling deadline | Candidate 0.5 s from first stop request; must be agreed against actual system behavior before approval |
| Verification dwell | Candidate 2 s of valid filtered telemetry plus operator confirmation |

An independent deadline enforcement path must stop nonzero output even if the
pulse producer continues sending. The current 30 s sender session is not that
boundary. ROS/Python timing is best effort, not a hard-real-time safety limit.

### C. Supervised sequence, only after explicit execution approval

1. Confirm a flat, non-slip, unobstructed test area with no people/animals in the
   travel/leg sweep or emergency-lowering area. Assign separate command operator
   and safety observer. Keep operator access to the verified emergency control
   outside the robot's motion envelope. Match the actual manual's clearance.
2. Reserve command authority. Under approved coordination, prevent frontier
   exploration and all competing inputs; do not stop shared services without
   their owner's permission. Recheck the live graph and physical mode/owner.
3. Establish standing and the required manufacturer mode through the specifically
   approved procedure. Confirm the independent emergency path is available;
   mode requests or a working app connection are not mode/stop acknowledgments.
4. Observe the stationary baseline, freshness and observer readiness. Only after
   approval, test zero-command acceptance and the guard/stop acknowledgment path.
   Zero publication still uses the physical command interface and is prohibited
   during the present review phase. Do not confuse the zero-only check with proof
   of stopping a moving robot.
5. Arm once and issue the reviewed +0.03 m/s request for at most 0.5 s. Abort for
   unexpected direction/yaw, estimator loss, obstacle, loss of authority/mode,
   command timing fault, or operator concern. The operator may stop earlier.
6. At the deadline/abort, latch all nonzero requests off; immediately request zero
   on forward, lateral and yaw channels; maintain the bounded zero phase through
   the reviewed command path. Request both local gates stopped if both are used,
   without waiting serially through multi-second discovery timeouts. Cancel the
   pulse; do not start/restart exploration or re-arm automatically.
7. Verify filtered odometry and direct observation together. If unexpected motion
   continues, telemetry is lost, or the agreed settling deadline is exceeded,
   the safety observer activates the verified independent emergency stop without
   waiting for a ROS service, packet acknowledgment or full verification dwell.
   Do not deliberately cut network communication on the first moving trial.
8. After confirmed physical rest, perform only the approved Manual-mode handback
   procedure and independently confirm it. If handback is unconfirmed, remain
   latched off under the safety observer's control; no new motion trial.
9. Record output timing, measured displacement/direction, settling time, telemetry
   validity, operator observation and any faults. No movement at this request is
   an inconclusive interface/minimum-speed result, not permission to increase
   speed, repeat commands, change modes or try other destination addresses.

### D. Pass, fail and approval boundary

**Pass:** only the reviewed source commanded the correct direction within the
reviewed nonzero window, explicit zero was requested, fresh validated telemetry
met the pre-agreed stop criterion and dwell, the observer confirmed rest, and
control handback was confirmed. Report measured results, not nominal distance.

**Fail/abort:** unknown authority/mode, telemetry disagreement, missed deadline,
continued/unexpected motion, emergency activation or unconfirmed handback. Latch
off and investigate; do not repeat automatically. Apparent success on localhost
does not close a physical failure.

**Current status remains NO-GO.** No execution approval is requested until the
safety-critical unknowns have concrete answers. Review of this report alone does
not authorize motion, mode changes, physical-topic publication or service restarts.
