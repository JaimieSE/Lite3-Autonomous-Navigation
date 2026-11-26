# Lite3 lab cheat sheet

Everything below runs on the **group8 container**. Details are in
`operator/README.md`. The first block is non-actuating (dry). Keep the e-stop
within reach anyway. Physical tests are in the last block and in
`operator/PHYSICAL_TESTS.md`.

```bash
ssh user@192.168.10.78
cd ~/lite3_ws

operator/build.sh                 # only after code changes
operator/preflight.sh             # READY / DEGRADED / NOT READY, with a fix per item
operator/start_navigation.sh      # background start; waits for "NAVIGATION READY (dry, non-actuating)"
operator/status.sh                # health check  (--watch 10 to repeat)
operator/run_test4.sh             # 5-min Test 4; follow the on-screen phases; Ctrl+C aborts
operator/abort_test4.sh           # abort Test 4 from another terminal
operator/run_test4.sh --phases P3            # one phase (P2/P4/P5 alone add a 20 s BASE capture first)
operator/run_test4.sh --manual --phases P2,P3   # Enter starts each phase
operator/run_test4.sh --list-phases          # what each phase asks you to do
operator/run_test4.sh --stop-distance 0.40   # dry test with a larger proximity stop (>= 0.30 m)
operator/stop_navigation.sh       # ALWAYS when done; also cleans up after crashes
```

**Test 4 setup.** Clear 0.6 m all round the robot and 2.5 m ahead. Measure
distances from the robot's nose.

| Phase | Time | Action |
| --- | --- | --- |
| P1 | 0–60 s | Nothing |
| P2 | 60–120 s | Box at 1.2 m |
| P3 | 120–180 s | Box at 0.25 m (expect STOP) |
| P4 | 180–240 s | 5–8 cm object at 1.0 m |
| P5 | 240–300 s | Walk across at 1–1.5 m, then clear |

**Live line:** `robot cmd_vel msgs` must stay at 0.

**Results:**

```bash
cat "$(ls -td ~/lite3_ws/log/operator/run-*/test4/* | head -1)/summary.md"
```

**LiDAR slow, with failed reassemblies in the preflight.** Run this, which needs
your sudo password and lasts until the container restarts:

```bash
sudo sysctl -w net.ipv4.ipfrag_high_thresh=67108864 net.ipv4.ipfrag_low_thresh=50331648 net.ipv4.ipfrag_time=3
```

To revert:

```bash
sudo sysctl -w net.ipv4.ipfrag_low_thresh=3145728 net.ipv4.ipfrag_high_thresh=4194304 net.ipv4.ipfrag_time=30
```

**Something wrong?**

```bash
operator/stop_navigation.sh && operator/preflight.sh
```

Then read the `->` hints after each failed check.

**Physical tests (the robot MOVES).** Remote / e-stop in your hand for all of
these.

```bash
operator/run_physical.sh --list-phases                    # M0-M9: setup + PASS criteria
operator/run_physical.sh --sim --phases M0,M1,M2           # rehearsal, no robot
operator/run_physical.sh --phases M0                       # dry readiness: lists blockers, publishes nothing
operator/stop_navigation.sh && operator/start_navigation.sh --physical   # type ARM; dead-man LOCKED
operator/run_physical.sh --physical --phases M0,M1,M2      # type each test ID to start it
operator/run_physical.sh --phases DET --object-distance 0.20   # dry, no motion: object seen steadily?
operator/run_physical.sh --physical --phases M3            # box 0.20 m from the nose: must not move
operator/start_navigation.sh --physical --clearance 0.40   # collision stop boundary for 0.40 m clearance
operator/run_physical.sh --phases M4                       # dry: M4 READY? (nothing published)
operator/run_physical.sh --phases M4 --shadow              # dry: slide a chair toward the robot
operator/run_physical.sh --physical --phases M4            # chair ANYWHERE ahead: walk until it stops the robot
operator/run_physical.sh --physical --phases M4 --profile operator/profiles/m4_first_session.yaml
operator/run_physical.sh --physical --phases M7            # Nav2 goes around a box
operator/walk.sh --distance 3.0 --physical                 # walk 3 m, going around obstacles
operator/run_physical.sh --physical --phases M3 --stop-distance 0.40
operator/run_physical.sh --physical --phases M5 --goal-distance 1.5 --max-speed 0.08
operator/abort_physical.sh                                 # commanded stop from another terminal (NOT an e-stop)
operator/roam.sh start --physical                          # M10: roam on its own until stopped (type ROAM)
operator/roam.sh pause | resume | stop | status            # M10 control, from any terminal
operator/roam.sh speed slow|normal|fast                    # M10 speed mode (<= --speed-limit)
operator/start_navigation.sh --physical --direct-telemetry # if preflight fails on robot telemetry (bridge relay slow)
operator/run_physical.sh --physical --phases SIT && operator/stop_navigation.sh
```

M4 needs navigation started with `--clearance` ≥ 0.19 (boundary ≥ 0.35 m, clear of the 0.30 m
backup lock); it says so and refuses otherwise. Outcomes and options: PHYSICAL_TESTS.md §3.1.

Order is enforced per navigation run: M0 → M1 → M2; M0 → M3; M2 + M3 → M4;
M2 → M5 → M6, M7, M8 → M9.
