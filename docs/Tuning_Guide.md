# Comprehensive Tuning Guide — Pololu 3Pi+ 32U4

## Overview

This guide accompanies the tuning sketch (`3Pi_CodeStub.ino` on the `tuning` branch). It covers every tunable parameter related to movement, organised by layer, and provides step-by-step procedures for calibrating the robot to the accuracy required for the 4-minute foraging challenge.

**Tuning order**: 1 → 3 → 0 → 4 → 5
Each layer depends on the accuracy of the layers tuned before it. Do not skip ahead.

**Equipment needed**: ruler (300 mm+), protractor or tape marker on floor, Serial Monitor (115200 baud), Serial Plotter (Arduino IDE), stopwatch (optional).

---

## How to Use the Tuning Sketch

1. Open `3Pi_CodeStub.ino` (tuning branch).
2. Set `#define TUNE_LAYER` at the top of the file to the layer you want to calibrate (1, 3, 0, 4, or 5).
3. Compile and upload.
4. Open Serial Monitor at **115200 baud**.
5. Press **Button A** on the robot to start.
6. Follow the on-screen instructions and the procedures below.
7. After adjusting values, recompile and repeat until targets are met.

---

## Complete Parameter Table

### Layer 1 — Motor Actuation (Motors.h)

| Parameter | Current Value | Target Accuracy | Location | Description |
|-----------|--------------|-----------------|----------|-------------|
| `L_DEADBAND_FWD` | 15 | ±1 PWM | Motors.h:66 | Minimum PWM to start left wheel forward |
| `L_DEADBAND_REV` | 14 | ±1 PWM | Motors.h:67 | Minimum PWM to start left wheel reverse |
| `R_DEADBAND_FWD` | 13 | ±1 PWM | Motors.h:68 | Minimum PWM to start right wheel forward |
| `R_DEADBAND_REV` | 12 | ±1 PWM | Motors.h:69 | Minimum PWM to start right wheel reverse |
| `MAX_PWM` | 180.0 | N/A (safety limit) | Motors.h:59 | Maximum PWM sent to motors (prevents damage) |
| `FWD` / `REV` | LOW / HIGH | Exact | Motors.h:51-52 | Direction pin logic levels |

### Layer 3 — PID Speed Control (3Pi_CodeStub.ino + PID.h)

| Parameter | Current Value | Target Accuracy | Location | Description |
|-----------|--------------|-----------------|----------|-------------|
| `TUNE_KP` | 40.0 | ±20% | .ino:64 | Proportional gain — drives response speed |
| `TUNE_KI` | 0.8 | ±50% | .ino:65 | Integral gain — eliminates steady-state error |
| `TUNE_KD` | 0.0 | ±100% | .ino:66 | Derivative gain — dampens oscillation |
| `i_limit` | 250.0 | Order of magnitude | PID.h:60 | Anti-windup clamp on integral accumulator |
| `PID_INTERVAL` | 5 ms | Fixed | .ino:54 | How often PID update runs |
| `SPEED_INTERVAL` | 5 ms | Fixed | .ino:45 | How often speed is measured |

### Layer 0 — Kinematics (Kinematics.h)

| Parameter | Current Value | Target Accuracy | Location | Description |
|-----------|--------------|-----------------|----------|-------------|
| `wheel_radius` | 16.4 mm | ±0.1 mm | Kinematics.h:29 | Wheel radius — scales linear distance |
| `wheel_sep` | 42.7 mm | ±0.2 mm | Kinematics.h:30 | Half wheel separation — scales rotation |
| `count_per_rev` | 358.3 | Fixed (from docs) | Kinematics.h:28 | Encoder counts per wheel revolution |

### Layer 4 — Navigation Primitives (3Pi_CodeStub.ino)

| Parameter | Current Value | Target Accuracy | Location | Description |
|-----------|--------------|-----------------|----------|-------------|
| `TURN_THRESHOLD` | 0.01 rad | ±0.005 rad | .ino:77 | Heading error below which turn is "done" |
| `TURN_GAIN` | 5.0 | ±2.0 | .ino:78 | Proportional gain: heading error → turn speed |
| `MAX_TURN_SPEED` | 0.5 c/ms | ±0.1 | .ino:79 | Maximum rotation speed during turns |
| `MIN_TURN_SPEED` | 0.15 c/ms | ±0.05 | .ino:80 | Minimum rotation speed (must exceed dead-band) |
| `TRAVEL_THRESHOLD` | 10.0 mm | ±5 mm | .ino:82 | Distance below which travel is "arrived" |
| `TRAVEL_SPEED` | 0.5 c/ms | ±0.1 | .ino:83 | Base forward speed during travel |
| `HEADING_CORR_GAIN` | 0.1 | ±0.05 | .ino:84 | Heading correction during straight travel |
| `TRAVEL_DECEL_DIST` | 50.0 mm | ±20 mm | .ino:85 | Distance from target at which deceleration begins |
| `MIN_TRAVEL_SPEED` | 0.15 c/ms | ±0.05 | .ino:86 | Floor speed during deceleration |
| `SETTLE_TIME_MS` | 50 ms | ±25 ms | .ino:88 | Pause between turn and travel phases |
| `SETTLE_SPEED_THRESH` | 0.02 c/ms | ±0.01 | .ino:89 | Speed below which robot is considered stopped |

### Layer 5 — Composite Navigation (3Pi_CodeStub.ino)

Layer 5 has no independent tunable parameters. It validates the combined behaviour of Layers 0-4 using the waypoint coordinates defined at the top of the file. Adjust those if the physical map layout differs from expected.

| Parameter | Current Value | Target Accuracy | Location | Description |
|-----------|--------------|-----------------|----------|-------------|
| `l5_x[]`, `l5_y[]` | See .ino:191 | ±10 mm | .ino:191-192 | Waypoint coordinates from map measurements |
| `L5_ENDURANCE_MS` | 240000 (4 min) | Exact | .ino:192 | Endurance test duration |

---

## Layer 1: Motor Dead-Band Measurement

### What You Are Tuning
The **dead-band** is the minimum PWM value that produces actual wheel rotation. Below this value, the motor receives current but cannot overcome static friction. Dead-band compensation shifts any non-zero PID output up by this offset, linearising the speed-vs-PWM relationship so the PID controller works predictably at low speeds.

### Interactions
- Dead-band values feed into `Motors.h` `setPWM()`, which is called by every higher layer.
- If dead-band is set too high, the robot will jump to unnecessarily high speeds on small PID outputs.
- If dead-band is set too low, the PID will command speeds that produce no movement, causing integral windup and eventual overshoot.

### Procedure

1. Set `TUNE_LAYER 1`, compile, upload.
2. Open Serial Monitor (115200 baud).
3. Press Button A.
4. The robot ramps PWM from 0 to 25 in steps of 1, holding each step for 500 ms.
5. For each step, the Serial Monitor shows:
   ```
   L_FWD | PWM: 14 | delta:0
   L_FWD | PWM: 15 | delta:3  <<< MOVING
   ```
6. **Record the first PWM where `delta` becomes non-zero** — this is the dead-band.
7. The test runs 4 phases automatically: Left Forward, Left Reverse, Right Forward, Right Reverse.

### Recording Results

| Phase | Dead-band PWM |
|-------|--------------|
| Left Forward | ___ |
| Left Reverse | ___ |
| Right Forward | ___ |
| Right Reverse | ___ |

### Applying Results

Open `Motors.h` and update the four defines:
```c
#define L_DEADBAND_FWD ___   // Left motor forward dead-band
#define L_DEADBAND_REV ___   // Left motor reverse dead-band
#define R_DEADBAND_FWD ___   // Right motor forward dead-band
#define R_DEADBAND_REV ___   // Right motor reverse dead-band
```

### Target
Each value should be accurate to **±1 PWM step**. A single run is usually sufficient. If the robot's surface (carpet vs. hard floor) or battery charge changes significantly, re-measure.

### Troubleshooting
- **No movement up to PWM 25**: Check motor connections, battery charge, direction pin wiring.
- **Movement at PWM 0**: Encoder noise or vibration. Ensure robot is on a flat surface with wheels free to turn.
- **Inconsistent results**: Battery voltage affects dead-band. Measure with a freshly charged battery for consistency.

---

## Layer 3: PID Speed Controller Tuning

### What You Are Tuning
The PID controllers translate a demanded speed (counts/ms) into a PWM value that actually achieves that speed. Good PID tuning means:
- **Fast response**: speed reaches demand quickly after a step change
- **No steady-state error**: speed matches demand exactly at steady state
- **No oscillation**: speed doesn't bounce around the target

### Interactions
- **Kp** (proportional): Higher = faster response, but too high causes oscillation. Since speed is measured in counts/ms (small numbers like 0.5), Kp needs to be large (typically 20-80) to produce useful PWM values.
- **Ki** (integral): Eliminates offset between demand and actual speed. Too high causes slow oscillation. The `i_limit` anti-windup clamp prevents the integral from growing unboundedly during state transitions.
- **Kd** (derivative): Dampens overshoot. Usually not needed for this system. The EMA filter in PID.h (alpha=0.2) already smooths encoder noise.
- Dead-band compensation (Layer 1) must be correct first — if not, the PID has a "dead zone" near zero that causes windup.

### Procedure

1. Set `TUNE_LAYER 3`, compile, upload.
2. Open **Serial Plotter** (Tools → Serial Plotter, 115200 baud).
3. Press Button A.
4. Three phases run automatically:
   - **Phase 0**: demand = 0.50 c/ms for 5 s (medium speed)
   - **Phase 1**: demand = 0.15 c/ms for 5 s (low speed, hardest to control)
   - **Phase 2**: demand = -0.30 c/ms for 3 s (reverse)
5. The Serial Plotter shows three traces: `demand`, `left`, `right`.
6. Press Button A to repeat with different gains.

### What to Look For

| Symptom | Diagnosis | Fix |
|---------|-----------|-----|
| Speed never reaches demand | Kp too low | Increase Kp by 50% |
| Speed overshoots then oscillates | Kp too high | Decrease Kp by 30% |
| Speed reaches demand but offset remains | Ki too low | Increase Ki by 50% |
| Slow, large-amplitude oscillation | Ki too high | Decrease Ki by 50% |
| Noisy/jittery speed trace | Kd too high or encoder noise | Reduce Kd, or reduce derivative filter alpha |
| Left and right tracks don't match | Normal — motors differ slightly | PID compensates automatically; ensure both are acceptably close |

### Tuning Methodology (Ziegler-Nichols lite)

1. Start with `TUNE_KP = 10.0`, `TUNE_KI = 0.0`, `TUNE_KD = 0.0`.
2. Double Kp until the speed response oscillates continuously (Ku = "ultimate gain").
3. Set Kp = 0.6 × Ku.
4. Run test. If steady-state error exists, add Ki = 0.5 and increase until error disappears.
5. If overshoot is excessive, add Kd = 0.1 and increase until damped.
6. **Critical check**: test at demand = 0.15 (Phase 1). Low-speed control is the hardest. If it oscillates at low speed but is fine at medium speed, reduce Kp slightly and increase Ki.

### Targets
- Rise time to 90% of demand: < 200 ms
- Steady-state error: < 5% of demand
- No sustained oscillation at any test speed
- Both wheels within 10% of each other at steady state

### Applying Results

Edit the top of `3Pi_CodeStub.ino`:
```c
#define TUNE_KP ___
#define TUNE_KI ___
#define TUNE_KD ___
```

These values will also need to be transferred to the main codebase when switching back from the tuning branch.

---

## Layer 0: Kinematics Calibration

### What You Are Tuning
Two physical dimensions that convert encoder counts into real-world distances and angles:
- **`wheel_radius`**: Scales linear distance. If too large, the robot thinks it's traveled further than it has.
- **`wheel_sep`**: Scales rotation. If too large, the robot thinks it's rotated further than it has (under-rotates in practice).

### Interactions
- `wheel_radius` affects **both** linear and rotational measurements (since rotation is computed from the difference in wheel distances).
- **Always calibrate `wheel_radius` first**, then `wheel_sep`.
- These values depend on tire inflation, surface friction, and wear. Recalibrate if the robot moves to a different surface.
- `count_per_rev` (358.3) is a hardware constant — do not change it.

### Sub-Test A: Linear Distance (wheel_radius)

**What it does**: Robot drives straight for 1000 encoder counts (~288 mm), then stops and reports its calculated distance.

**Procedure**:
1. Set `TUNE_LAYER 0`, compile, upload.
2. Place robot at the 0 mark of a ruler, front of robot aligned with the ruler.
3. Press Button A to start.
4. Robot drives forward and stops.
5. Read `pose.x` from Serial Monitor.
6. Measure actual distance traveled with the ruler.
7. Calculate new wheel_radius:

```
new_wheel_radius = current_wheel_radius × (actual_distance_mm / pose.x)
```

**Example**: If `wheel_radius = 16.4`, `pose.x = 295.2`, and ruler shows 288 mm:
```
new_wheel_radius = 16.4 × (288 / 295.2) = 15.99 mm
```

8. Update `wheel_radius` in `Kinematics.h`, recompile, repeat.

**Target**: `pose.x` matches ruler measurement within **±2 mm** over 288 mm (~0.7% error).

### Sub-Test B: Rotation (wheel_sep)

**What it does**: Robot performs 4 complete counter-clockwise rotations (8π radians) using the `setRotation()` function.

**Procedure**:
1. Place a piece of tape on the floor aligned with the robot's front.
2. Press Button A.
3. Robot rotates 4 full turns and stops.
4. Check whether the robot's front is aligned with the tape.

**Interpreting results**:
- **Under-rotated** (didn't reach the tape): `wheel_sep` is too **small** → increase it.
- **Over-rotated** (passed the tape): `wheel_sep` is too **large** → decrease it.

**Correction formula**:
```
actual_turns = 4.0 + (overshoot_degrees / 360.0)
new_wheel_sep = current_wheel_sep × (4.0 / actual_turns)
```

If the robot is 15° past the tape:
```
actual_turns = 4.0 + (15 / 360) = 4.0417
new_wheel_sep = 42.7 × (4.0 / 4.0417) = 42.26 mm
```

If the robot is 20° short of the tape:
```
actual_turns = 4.0 - (20 / 360) = 3.9444
new_wheel_sep = 42.7 × (4.0 / 3.9444) = 43.30 mm
```

8. Update `wheel_sep` in `Kinematics.h`, recompile, repeat.

**Target**: After 4 full turns, robot faces within **±5°** of the starting orientation.

**Tip**: Using 4 turns instead of 1 amplifies rotational error by 4×, making small misalignments easier to see and correct. A 1° per-turn error becomes a visible 4° offset.

### Sub-Test C: Square Validation (combined)

**What it does**: Robot drives a 200 mm square (4 corners) using `travelToXY()` and returns to the start. Tests both linear and rotational accuracy together.

**Procedure**:
1. Press Button A.
2. Robot drives: (0,0) → (200,0) → (200,-200) → (0,-200) → (0,0).
3. Check final position error in Serial Monitor.

**Target**: Final position error < **20 mm**, heading error < **5°**.

**If the square test fails**:
- Large X or Y offset → re-run Sub-Test A (wheel_radius).
- Large heading error → re-run Sub-Test B (wheel_sep).
- Systematic drift in one direction → may indicate asymmetric tire wear or surface friction. Consider adding a small bias to `wheel_sep`.

---

## Layer 4: Navigation Primitive Tuning

### What You Are Tuning
The parameters that control how the robot executes turns and straight-line travel. These build on PID speed control (Layer 3) and kinematics (Layer 0).

### Parameter Interactions

```
Turn Controller:
  heading_error → × TURN_GAIN → speed demand
                                    ↓
                    clamped to [MIN_TURN_SPEED, MAX_TURN_SPEED]
                                    ↓
                    compared to TURN_THRESHOLD → stop

Travel Controller:
  distance_to_target → base speed (TRAVEL_SPEED)
                           ↓
           if dist < TRAVEL_DECEL_DIST → linear ramp down
                           ↓
           floored at MIN_TRAVEL_SPEED
                           ↓
           heading correction: ± HEADING_CORR_GAIN × heading_error
```

### Key Relationships
- `TURN_GAIN` and `MIN_TURN_SPEED` interact: if TURN_GAIN is high, the proportional region is narrow and MIN_TURN_SPEED dominates. If TURN_GAIN is low, turns are slow but smooth.
- `TRAVEL_DECEL_DIST` and `TRAVEL_THRESHOLD` interact: deceleration must bring speed low enough that the robot stops within `TRAVEL_THRESHOLD` of the target.
- `HEADING_CORR_GAIN` trades path straightness against stability: too high causes weaving, too low allows drift.
- `SETTLE_TIME_MS` prevents PID integral carryover between turn and travel phases.

### Sub-Test A: Turn Accuracy

**What it does**: Robot performs 4 absolute turns: 90°, 180°, -90°, 0°. Reports the error for each.

**Procedure**:
1. Set `TUNE_LAYER 4`, compile, upload.
2. Press Button A.
3. Watch the robot turn to each target angle.
4. Serial Monitor shows target, actual, and error for each turn.

**Target**: Each turn error < **2°** (0.035 rad).

**Adjustments**:
| Symptom | Fix |
|---------|-----|
| Overshoots then settles | Decrease `TURN_GAIN` or increase `TURN_THRESHOLD` |
| Undershoots (doesn't quite reach) | Decrease `TURN_THRESHOLD` |
| Oscillates around target | Decrease `TURN_GAIN`, increase `SETTLE_TIME_MS` |
| Very slow turns | Increase `TURN_GAIN` or `MIN_TURN_SPEED` |
| Doesn't turn at all | `MIN_TURN_SPEED` below motor dead-band — increase it |

### Sub-Test B: Travel Accuracy

**What it does**: Robot travels to (300, 0) then back to (0, 0). Reports position error at each destination.

**Procedure**:
1. Press Button A.
2. Watch the robot drive out and back.
3. Serial Monitor shows pose and error at each waypoint.

**Target**: Arrival error < **15 mm** at each destination.

**Adjustments**:
| Symptom | Fix |
|---------|-----|
| Overshoots target | Increase `TRAVEL_DECEL_DIST` or decrease `TRAVEL_SPEED` |
| Stops short of target | Decrease `TRAVEL_THRESHOLD` or `TRAVEL_DECEL_DIST` |
| Weaves during travel | Decrease `HEADING_CORR_GAIN` |
| Drifts off course | Increase `HEADING_CORR_GAIN` |
| Jerky deceleration | Increase `TRAVEL_DECEL_DIST`, decrease `TRAVEL_SPEED` |

### Sub-Test C: Square Validation

Same 200 mm square as Layer 0 Sub-Test C, but now testing the full navigation pipeline (Layer 4 turn+travel primitives on top of Layer 0 kinematics).

**Target**: Return error < **20 mm**, heading error < **5°**.

If this fails but Layer 0's square test passed, the issue is in Layer 4 parameters. If both fail, go back to Layer 0.

---

## Layer 5: Composite Navigation & Endurance

### What You Are Tuning
Layer 5 has no unique tunable parameters — it validates the integrated system. If Layer 5 tests fail, the fix is always in a lower layer.

### Sub-Test A: Waypoint Circuit

**What it does**: Robot navigates to all 6 puck waypoints in order, then returns home (0, 0). Reports position error at each waypoint and total circuit time.

**Procedure**:
1. Set `TUNE_LAYER 5`, compile, upload.
2. Place robot at the start position on the map.
3. Press Button A.
4. Watch robot visit all waypoints and return.
5. Serial Monitor shows arrival error at each WP and final home error.

**Targets**:
- Home return error: < **30 mm**
- Circuit time: < **60 s** (robot isn't sluggish)
- Individual waypoint error: < **20 mm** (check which waypoints have large error)

**Diagnosis**:
- Consistently large errors at far waypoints → Layer 0 kinematics drift (re-calibrate wheel_radius/wheel_sep).
- Large errors only on turns → Layer 4 turn parameters.
- Robot very slow → increase `TRAVEL_SPEED` or `MAX_TURN_SPEED` in Layer 4.

### Sub-Test B: 4-Minute Endurance

**What it does**: Robot continuously navigates the waypoint circuit for 4 minutes (matching the assessment duration), returning home between laps. After 4 minutes, it makes one final trip home. Reports pose every 30 seconds.

**Procedure**:
1. Press Button A.
2. Let the robot run for the full 4 minutes.
3. Monitor Serial output for pose drift over time.
4. At the end, check home return error.

**Targets**:
- Home return error after 4 minutes: < **30 mm**
- Pose should not drift monotonically in one direction (this indicates a kinematics bias)

**Diagnosis**:
| Pattern | Cause | Fix |
|---------|-------|-----|
| X drifts positively over time | `wheel_radius` too large | Decrease slightly |
| X drifts negatively over time | `wheel_radius` too small | Increase slightly |
| Heading drifts clockwise | `wheel_sep` too small or right wheel slightly larger | Increase `wheel_sep` |
| Heading drifts counter-clockwise | `wheel_sep` too large or left wheel slightly larger | Decrease `wheel_sep` |
| Error grows then shrinks (random walk) | Normal odometry noise | Acceptable if final error < 30 mm |
| Robot leaves map boundary | Travel speed too high or heading correction too weak | Reduce `TRAVEL_SPEED`, increase `HEADING_CORR_GAIN` |

---

## Quick Reference: Tuning Checklist

Use this checklist during the tuning session:

### Layer 1 (5 minutes)
- [ ] Dead-band measured for all 4 motor/direction combinations
- [ ] Values updated in `Motors.h`
- [ ] Re-uploaded and verified (no movement at PWM = dead-band - 1)

### Layer 3 (15-30 minutes)
- [ ] Kp tuned: fast response without oscillation
- [ ] Ki tuned: no steady-state error at medium speed (0.5 c/ms)
- [ ] Low-speed test (0.15 c/ms) tracks without oscillation
- [ ] Reverse test (-0.3 c/ms) works correctly
- [ ] Values recorded: Kp=___, Ki=___, Kd=___

### Layer 0 (15-20 minutes)
- [ ] Sub-Test A: pose.x within ±2 mm of ruler
- [ ] wheel_radius updated in `Kinematics.h`: ___
- [ ] Sub-Test B: 4 turns end within ±5° of start
- [ ] wheel_sep updated in `Kinematics.h`: ___
- [ ] Sub-Test C: square return error < 20 mm, heading < 5°

### Layer 4 (10-15 minutes)
- [ ] Turn errors all < 2°
- [ ] Travel arrival errors < 15 mm
- [ ] Square return error < 20 mm
- [ ] Any parameters changed: ___

### Layer 5 (10-15 minutes)
- [ ] Waypoint circuit home error < 30 mm
- [ ] 4-minute endurance home error < 30 mm
- [ ] No monotonic drift pattern observed

---

## Transferring Values to Main Codebase

After tuning is complete, transfer the calibrated values back to the main branch:

1. **Motors.h**: Copy `L_DEADBAND_FWD`, `L_DEADBAND_REV`, `R_DEADBAND_FWD`, `R_DEADBAND_REV`.
2. **Kinematics.h**: Copy `wheel_radius`, `wheel_sep`.
3. **PID gains**: Copy `TUNE_KP`, `TUNE_KI`, `TUNE_KD` to wherever PID is initialised in the main sketch.
4. **Navigation parameters**: Copy any changed values for `TURN_GAIN`, `TRAVEL_SPEED`, etc.

---

## Notes

- **Battery voltage**: Motor dead-band and PID response vary with battery charge. Tune with a freshly charged battery and note that performance may degrade as the battery drains during a 4-minute run.
- **Surface matters**: Wheel slip differs between carpet, laminate, and the printed map. Always do final calibration on the **actual competition surface**.
- **Temperature**: Motor resistance changes with temperature. If the robot has been running continuously, let it cool for 2-3 minutes before re-measuring dead-band.
- **Encoder mapping reminder**: `count_e0` = RIGHT wheel, `count_e1` = LEFT wheel. This is verified in the tuning sketch.
