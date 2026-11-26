# Supervised stand and one-second walking trial — 2026-11-13

## Outcome

The operator confirmed that the robot stood after one posture request. A single
forward-velocity pulse then produced **marching in place**, according to the
operator. Forward travel was not demonstrated. After the pulse's trailing zeros
and a further zero-only burst, the operator confirmed **not moving**.

This was a low-speed motion-interface commissioning test, not a Nav2 goal or
autonomous exploration test. No successful autonomous navigation is claimed.
No further nonzero trial, speed increase, gait change or automatic retry occurred.

## Authorization and observed interfaces

The operator supplied system IP `192.168.10.64`, confirmed exclusive control,
a clear area and a verified emergency stop in hand, and reported Auto mode
enabled. The operator initially reported not standing, then explicitly requested
stand, walk and stop. After the posture request, the operator confirmed standing
before the forward pulse began.

Read-only ROS inspection in the documented system domain 64 found one
`/motion_sender` subscriber on `/cmd_vel`, `/cmd_vel_corrected`, `/simple_cmd`
and `/complex_cmd`. No velocity publishers were found before the trial.
Its live parameters reported target `192.168.1.120:43893`. SSH authentication
was unavailable, so no system source, services or configuration were changed.

## Commands and evidence

1. One reliable, volatile `/simple_cmd` request was published directly in system
   domain 64: `cmd_code=553714178` (`0x21010202`), `size=0`, `type=0`.
   This is the previously documented posture toggle; it was not repeated.
   The local observer received the same message. The operator then confirmed
   standing. An earlier sender initialization error occurred before publication.
2. The retained [commissioning pulse](../legacy/lite3_legacy/commissioning_pulse.py)
   was invoked from source with explicit physical-test options. It used the
   existing group-domain-8 `/cmd_vel` bridge and observed system domain 64.
   An initial import-path error occurred before any command; after fixing the
   launch environment, exactly one forward phase executed.
3. Requested forward velocity: `+0.03 m/s`; all other axes zero. Requested forward
   duration: one second, with 0.5 s of zero warmup and two seconds of trailing
   zero requests. The test had no LiDAR collision protection; the nearby operator
   provided the confirmed clear-area and emergency-stop supervision.
4. Runner result: `duration_complete`, 20 forward messages and 51 zero messages.
   First local forward to first trailing zero: `1.000169116 s`.
   Last local forward to first trailing zero: `0.041092358 s`.
   The system-domain observer saw 20 forward and 51 zero messages, with
   `0.041019027 s` from last observed forward to first trailing zero.
   These are ROS receipt/publication timings, not actuator timestamps or
   guaranteed physical-motion duration.
5. The operator reported marching in place. Because physical rest had not yet
   been confirmed, another two-second **zero-only** burst was sent directly to
   system-domain-64 `/cmd_vel`: 40 all-zero messages, one matched subscriber.
   No posture or mode change was sent during stopping.
6. The operator then confirmed the robot was not moving. No emergency-stop
   activation was reported. Physical stop is confirmed by that observation,
   not by ROS command counts. Displacement and stopping time were not measured.

Before the physical sequence, all 15 commissioning-pulse tests passed in an
isolated loopback environment. The isolated receiver measured about
`1.000164321 s` from first forward to first zero. That validation did not
establish physical watchdog behavior or firmware acceptance.

## Interpretation and next step

The observed response supports that the existing command path can trigger a
locomotion response. It does not establish correct velocity scaling, minimum
walking speed, gait state, forward displacement or reliable autonomous stopping
under failures. The cause of stepping without translation remains unknown.
Do not infer that more speed, another mode command or another posture toggle
is the solution.

Next inspect the actual deployed bridge/firmware control state and recorded
odometry/IMU alongside operator observations, then define a separate reviewed
trial if needed. Independently verify watchdog and emergency-stop behavior
before Nav2 goals or frontier exploration are physically enabled. The current
project launch remains preview-only and university packages remain unchanged.
