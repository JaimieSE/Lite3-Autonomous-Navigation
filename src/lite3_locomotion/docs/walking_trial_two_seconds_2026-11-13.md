# Supervised two-second trial — 2026-11-13

## Scope and outcome evidence

The operator requested stand, two seconds of forward walking, stop and sit using
the existing university movement path. The operator reported initially sitting,
then confirmed steady standing after the stand request and authorized proceeding.

One forward command phase completed, followed by trailing zeros. The operator
responded "stop in place"; this was treated as a stop request and followed by
an additional zero-only burst. The operator then explicitly requested "sit now",
and one sit toggle was sent. No clear forward-displacement report was obtained.
Physical sitting was not confirmed before the operator requested that future
tests avoid repeated human verification prompts. Do not mark the final posture
or forward walking verified from silence.

This was direct motion-interface commissioning through the university bridge,
not a Nav2 navigation or frontier-exploration trial. No custom UDP sender or
additional motion bridge was started, and no shared service was modified.

## Preparation and commands

- Baseline preserved on branch `preserve/pre-two-second-trial-2026-11-13` before
  adapting the excluded commissioning script. Its default remains one second,
  with explicit duration limited to at most two seconds; speed cap remains
  0.03 m/s. Active navigation package and default preview launch are unchanged.
- All 17 focused pulse tests passed, including one- and two-second isolated ROS
  loopback timing. Two-second receiver forward-to-zero span: `2.000115575 s`.
- Read-only discovery confirmed system-domain-64 `/motion_sender` inputs and
  live target parameters `192.168.1.120:43893`. Source group-domain-8 `/cmd_vel`
  had no publisher before the trial; the persistent domain-64 bridge publisher
  is not itself evidence of ongoing nonzero commands.
- One volatile/reliable stand request was sent directly to domain-64
  `/simple_cmd`: `cmd_code=0x21010202`, `size=0`, `type=0`. The source checks
  rejected conflicting command publishers or observed nonzero velocities.
  Local ROS receipt was observed, and the operator confirmed standing.
- The excluded single-publisher pulse used group-domain-8 `/cmd_vel` through
  the existing university bridge. Requested forward speed `+0.03 m/s`, all other
  axes zero, requested forward duration two seconds, zero warmup 0.5 seconds
  and trailing zero phase two seconds. No LiDAR collision gate was active.
- Pulse result: `duration_complete`; **40 forward messages and 51 zero messages**,
  also observed in domain 64. Local first-forward to first trailing zero:
  `2.000183513 s`. Last-forward to first zero: local `0.024569481 s`, remote
  observer `0.024496935 s`. These are ROS timings, not physical-motion duration
  or a hard real-time guarantee.
- Following the operator's stop response, another **40 all-zero messages** were
  sent directly in domain 64 over two seconds, with one matched subscriber.
  No additional forward command or automatic retry occurred.
- On the explicit "sit now" instruction, one volatile/reliable posture toggle
  `0x21010202` was sent via the existing system-domain-64 `/simple_cmd` input.
  ROS receipt was observed. It was not repeated. No final physical posture
  acknowledgment was available through the inspected interfaces.

## Feedback assessment for future autonomous sequencing

After the sit request, the operator asked to avoid repeated manual verification
and said they would report unsuccessful motion. A read-only five-second sample
of existing system-domain-64 feedback obtained 1,005 joint messages, 1,000 leg
odometry messages and 1,000 IMU messages. No commands were sent by that check.

The joint messages contained 12 finite positions but no velocity or effort
arrays. Leg odometry had empty parent/child frame names and all-zero pose/twist
covariance arrays. Its maximum reported planar speed was about 0.0161 in the
nominal ROS velocity units; this is not interpreted as actual movement or proof
of rest. No explicit posture, gait, mode or control-state topic was discovered.
The pinned receiver source similarly publishes joint positions and body-motion
estimates without an explicit posture acknowledgment.

These signals are candidates for calibrated state estimation. They are not
already validated stand/sit/physical-stop acknowledgments. Autonomous sequencing
requires verified posture criteria, motion/stationarity thresholds, freshness
checks, transition deadlines and stop/abort behavior. Until those are established,
silence from the operator is not evidence of success, and a timed toggle sequence
must not be presented as feedback-controlled autonomy. No further movement test
was started as part of this assessment.
