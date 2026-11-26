# M3 abort "a forward command passed the proximity lock": analysis and fix (2026-11-16)

**Report:** `log/operator/run-20261116-120716/physical/20261116-120920-physical/120920-M3/`.
The verdict was INTERVENTION_REQUIRED, and the safe stop afterwards verified rest.

## What the record shows

**Before the move:**
- The robot was already standing; `stand` was skipped.
- `nav_readiness` set the stop distance to 0.605 m from the centre and saw
  `proximity_stop_now: true`.
- The terrain monitor was running at **8.0 Hz, exactly the readiness minimum**
  (normally about 10 Hz).

**The abort:** `blocked_move` started at 1.796 s and lasted 3.067 s. That breaks
down as about 1.0 s of commanding, then the abort, then the 2 s closing zero
burst. So, about 1 s after the first forward command, a non-zero command was
observed at `motion_sender`, and the test stopped at once.

**After the abort:**
- Zeros were commanded, and rest was verified.
- The terrain monitor log shows the stop distance restored to 0.60 m at
  1794830970.03, consistent with that timeline.

**Not recorded:** an aborted step discarded its measurements and trace. The
precondition's stop reason and distance, the stop flag over time, and the leaked
command values were therefore all lost.

## What can be concluded

- **twist_mux was not at fault.** It forwards a priority-20 test command only
  when no lock is engaged. Its lock logic was verified on the real binary,
  including the case where the lock input goes silent. The dead-man was held
  during the move by design, and Nav2 had no goal.
- **So the terrain monitor reported "no stop" while the test was commanding.**
  That required no hazard for more than its 1 s release hold.
- **The M3 precondition was too weak to catch this.** It only checked that the
  stop flag was true at one instant. Two causes fit the evidence, and the lost
  data cannot distinguish them:
  - (a) The stop was active for another reason, such as a person within 0.45 m
    after placing the box (`hazard within 0.45 m`), and it cleared once they
    stepped back.
  - (b) Detection of the box dropped out for more than 1 s on a marginal (8 Hz)
    LiDAR link.
- **The test itself turned the perception release into motion.** It kept
  publishing forward commands without re-checking the lock.

## Fixes (no limit relaxed)

1. **Aborts keep their evidence.** The runner now saves, for any aborted or
   errored step:
   - the step's recorded preconditions;
   - every robot-side command;
   - Nav2 commands;
   - every terrain status sample (stop, reason, distance, problems);
   - the actual lock input `/lite3_terrain/stop`;
   - footprint clearance;
   - the last dead-man feed;
   - a full trace.

   `/lite3_terrain/stop` is now recorded continuously. Terrain processing time is
   recorded per scan.
2. **Detection check before M3 (and M4), with no commands sent.** For 3 s, the
   object must be:
   - seen ahead in ≥ 95% of scans;
   - with no status gap > 0.3 s;
   - with a distance spread ≤ 0.05 m;
   - inside the expected distance band.

   For M3, the stop must also be held **for the object ahead** (reason
   "… m ahead", not "within 0.45 m") in ≥ 95% of scans, and the lock input must
   never be false. Cause (a) and a flickering detection (b) now fail here, before
   anything moves.
3. **The lock is re-confirmed before every M3 command.** Both the status and the
   lock input must be fresh (≤ 0.3 s), true, and caused by the object ahead.
   Otherwise the test **withholds the command** and aborts.

   One limit remains: the runner and twist_mux receive the lock input in
   parallel. A release that arrives between the runner's check and its next send
   can still let at most one 50 ms command through. Previously, commands flowed
   until the leak was detected.
4. **New test DET** (no motion, also usable in dry mode). It is the same
   detection check, reporting rate, gaps, distance statistics and reasons for any
   object you place. Use it to check a specific object, such as the chair, before
   M3/M4.

## Validation (no actuation)

**Simulation:**
- A new fault, `detection_dropout`, makes the object invisible for 1.5 s once
  commanding starts. M3 now aborts with "lock released during the test, command
  withheld", and **no** command reaches the simulated `motion_sender`.
- A person-only stop (an obstacle 0.40 m to the side) fails the detection check;
  nothing is commanded.
- The abort report contains the terrain samples, lock input, preconditions and
  trace.

**Live, read-only:** DET ran against the robot (nothing published):
- an object 1.17 m ahead, seen in 100% of 49 scans;
- 9.8 Hz;
- max gap 0.26 s.

**Test suites:** 112 movement-test and 191 locomotion tests pass.

## Next physical step

Run on the current physical run (M0 and M2 have passed):

1. `operator/run_physical.sh --phases DET --object-distance 0.20` with the box in
   place (dry; nothing moves).
2. `operator/run_physical.sh --physical --phases M3`.

If M3 fails its detection check, the report shows which of (a) or (b) applied.
