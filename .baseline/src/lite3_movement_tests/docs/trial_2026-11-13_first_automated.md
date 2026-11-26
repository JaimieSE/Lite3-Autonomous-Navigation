# First automated physical session: 2026-11-13, about 12:29 UTC

These results come from **physical robot telemetry**. The run used sequence
`lite3_basic_bounded_walk` (sha256 `25a5546f…`) under authorization
`lite3_basic_bounded_walk-20261113T1428`. The operator confirmed every safety item
once, before the run. There were no prompts during the run. The full data is in
`~/lite3_ws/log/movement_tests/20261113-122939-physical/` (gitignored).

**Verdict: `FAILED_SAFE`.** The forward step missed its acceptance threshold by
about 1%. The robot finished lying and at rest, as the telemetry confirms.

| Step | Result | Telemetry evidence |
| --- | --- | --- |
| Readiness, telemetry contract, rest | PASSED | Clean command path; sensor checkers true |
| Stand (one toggle) | PASSED in 4.45 s | Leg extensions 0.3265 / 0.3274 / 0.3269 / 0.3283 m; tilt below 0.006 rad; settled |
| Verify standing | PASSED | Held for 2 s at rest |
| Forward, 0.03 m/s for 2 s | **FAILED** | Forward **0.0298 m** against a minimum of 0.03 m; lateral 0.003 m; yaw change 0.006 rad |
| Safe stop | Rest verified | 0 zero-send errors |
| Lie down (finalize) | PASSED in 4.6 s | Extensions 0.081 m; at rest |

## Findings

- **The standing band is now physically validated.** The measured stance was about
  0.327 m on all four legs, well clear of the 0.22 m threshold. The lying band
  measured 0.081 m. Both transitions were detected from telemetry alone.
- **Leg odometry recorded forward translation this time.** The heading was about
  1.64 rad (forward is +y in the odometry frame). The odometry y-coordinate rose
  from −1.0207 to a peak of −0.9846, about +0.036 m, then settled at −0.9912
  (net +0.0295 m). Joint activity reached 0.51 rad, with diagonal pairs (LF/RB
  and RF/LB) alternating: a trot gait. Peak odometry speed was 0.089 m/s and peak
  yaw rate 0.21 rad/s, both oscillating with the gait.
- **Delivery:** all 40 nonzero and 50 zero commands were observed in system domain 64.
- **Interpretation:** the robot achieved about 50% of the commanded displacement,
  and odometry shows about 0.007 m of rollback while stopping. Leg odometry is the
  vendor's estimator, not ground truth. Translation was not measured externally
  (for example with tape or video).
- The acceptance range was not changed after the run. Any change to the criteria
  or speed requires a new sequence hash and a new authorization.
