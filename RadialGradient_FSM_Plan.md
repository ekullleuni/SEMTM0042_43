# Radial-Gradient Centre-Finder FSM — Implementation Plan

> **Target model: Claude Opus 4.7 with no prior context.**
> This document is self-contained. All paths are relative to the repo root
> `SEMTM0042_43/`.

---

## 1. Goal & High-Level Behaviour

The 3Pi+ robot is placed at an unknown pose on a printed radial grayscale
gradient (pure black at the centre, fading to white at the perimeter).
When the operator presses **Button A**, the robot must:

1. Reset its pose to `(0, 0, 0)` and start a run timer.
2. **Orient** itself along a radius pointing towards the centre, using a
   *null-seeking* algorithm on the two edge sensors (see §4).
3. **Travel** forwards along that radius, continuously monitoring for
   the loss of alignment (edge sensor difference growing) or arrival
   (centre sensor dark enough).
4. If alignment is lost mid-travel, re-enter the orientation routine
   at the current pose.
5. When the centre is detected, stop, and show `(x, y)` relative to the
   start pose along with elapsed time on the OLED for long enough for a
   human to record them.
6. Wait for another Button A press to restart.

All readings used by the algorithm must be **normalised via a per-sensor
transfer function** so the five sensors are directly comparable.

---

## 2. Baseline Code & What to Reuse

### Source directory
`final code/3Pi_CodeStub/` — this is the Part 1 coursework solution for
this author's specific robot. It contains the tuned hardware drivers,
kinematics, PID gains, motor dead-bands, and an onboard gradient-strip
calibration routine.

### Files to reuse **unchanged** (copy into the new sketch folder)
| File | What it gives you |
|---|---|
| `Motors.h` | Motor PWM + dead-band compensation (L: +13/+11, R: +9/+10 — **measured for this robot, do not edit**) |
| `Encoders.h` | Interrupt-driven quadrature counting (`count_e0` = RIGHT, `count_e1` = LEFT) |
| `PID.h` | Generic PID class |
| `Kinematics.h` | Dead-reckoning (`wheel_radius = 16.5 mm`, `wheel_sep = 43.01 mm`, `count_per_rev = 358.3`) |
| `SpeedMeasurement.h` | `countsToMM()`, etc. |
| `LineSensors.h` | 5-channel ADC reads (`line_sensors.readings[0..4]` = DN1..DN5) |
| `GradientCalibration.h` | **The transfer function** — see §3 |
| `oled.h` | Pololu OLED wrapper |

### What to keep from the existing `3Pi_CodeStub.ino`
- Hardware instances and constants: `motors`, `line_sensors`, `pose`,
  `left_pid`, `right_pid`, `display(1, 30, 0, 17, 13)`, `BUZZER_PIN=6`,
  `BUTTON_A_PIN=14`.
- **PID gains**: `Kp=48.0, Ki=0.5, Kd=0.0` — this robot's tuned values.
- Right-motor bias correction: `setDemandSpeeds()` multiplies the right
  demand by `0.88` — **keep this, it's specific to this robot's drift**.
- Non-blocking loop skeleton: `measureSpeeds()`, `updatePID()`, `updatePose()`
  called every iteration.
- `setTurn()/checkTurn()` — absolute-heading turn (rotates to a target
  `pose.theta`).
- `setRotation()/checkRotation()` — relative rotation (rotate by a delta
  in radians, accumulated from encoder-integrated pose).
- `stopWithPID()`, `setDemandSpeeds(l,r)`, `setDemandSpeed(s)` helpers.
- `buttonAPressed()` non-blocking edge-detecting Button A check.
- `calibrateGradientSensors(25)` call in `setup()` (the onboard drive-over-
  strip calibration). **Keep this as-is**.
- `waitForButtonA()` blocking wait used once in `setup()` after calibration.
- Serial init at 9600 baud (matches the Part 1 log style).

### What to **remove / replace** from the existing .ino
- The scan-then-pick FSM: `GSTATE_SCAN_ROTATE`, `GSTATE_TURN_TO_BEST`,
  `GSTATE_DRIVE_GRADIENT`, `GSTATE_AT_CENTER`, and their associated
  variables (`scan_best_score`, `scan_best_theta`, `scan_lightness[]`,
  `SCAN_SAMPLE_INTERVAL_MS`, `SYMMETRY_WEIGHT`, `GRADIENT_STEER_GAIN`,
  `GRADIENT_MAX_STEER`, `center_confirm`, `checkCenterReached()`,
  `computeScanScore()`, and the edge-differential steering inside
  `GSTATE_DRIVE_GRADIENT`).
- The Phase-1-reset logic inside `GSTATE_WAIT_START` that picks a CCW/CW
  rotation based on left/right edge reads — the new orientation routine
  replaces this.
- The final-results block built into `GSTATE_DRIVE_GRADIENT`.
- `updateGradientDisplay()` — rewrite to match the new states.

The new FSM replaces all of the above.

---

## 3. Transfer Function — How Readings Are Normalised

`GradientCalibration.h` is already wired to provide:

```c
float gradientToPercent(int sensor_idx, float raw_reading);
// Returns 0.0 % (black) → 100.0 % (white) for sensor_idx ∈ 0..4
// Dispatches to piecewise-linear (default) or monotone-cubic interpolation
```

The robot must run `calibrateGradientSensors(25)` once in `setup()` over
a printed gradient strip (black square first). This populates
`grad_cal_raw[division][sensor]` and sets `grad_cal_complete = true`.

**All sensor values used by the FSM must go through this function.**
Wherever you need a sensor reading, do:

```c
line_sensors.readSensorsADC();
float L[5];
for (int i = 0; i < NUM_SENSORS; i++) {
  L[i] = gradientToPercent(i, line_sensors.readings[i]);
}
// L[0] = DN1 (left edge)
// L[1] = DN2
// L[2] = DN3 (centre)
// L[3] = DN4
// L[4] = DN5 (right edge)
// 0 = black, 100 = white.  For "darkness", compute (100 - L[i]).
```

Do **not** compare raw ADC values between sensors anywhere in the FSM.

**Optional**: toggle `use_cubic_interp = true;` if piecewise-linear causes
visible steering chatter. Leave it `false` by default.

---

## 4. Algorithm — Orientation + Travel

### 4.1 Orientation routine (null-seeking)

The core idea: along any radius of a radial gradient, the left and right
edge sensors see **identical** lightness. Off-axis, the sensor facing the
centre sees a darker value than the one facing the perimeter. So we
rotate until `L[0] ≈ L[4]` — then the robot is aligned along a radius
(either towards or away from the centre).

```
Given:  L[5] = transfer-function-normalised readings, 0 = black, 100 = white.
        edge_diff = L[0] - L[4]                       # signed, percent
        edge_err  = fabs(edge_diff)

Rotate slowly in-place:
    If edge_diff > 0 → left is LIGHTER → centre lies to the RIGHT → rotate CW
    If edge_diff < 0 → right is LIGHTER → centre lies to the LEFT → rotate CCW
    The sign determines rotation direction; the magnitude (optionally)
    scales the rotation speed for smoother null-seeking.

Declare alignment when:
    edge_err < EDGE_EQUAL_TOLERANCE   for NULL_DWELL_MS continuously.

Once aligned at heading θ_A:
    Record  centre_A = L[2]
    Command a relative 180° rotation (setRotation(PI)).
    After rotation completes, stop briefly and record centre_B = L[2].
    If centre_B is darker (lower %) than centre_A:
        Robot is now pointing at the true centre — proceed.
    Else:
        Command another 180° rotation to return to θ_A — proceed.

This 180° check resolves the axis ambiguity (the null point is reached
both pointing toward AND away from the centre).
```

### 4.2 Travel routine

Drive forward at a constant modest speed with equal left/right demands
(no proprioceptive steering — we rely on the orientation routine to
keep us pointed at the centre). While driving, monitor:

```
centre_pct = L[2]
edge_err   = fabs(L[0] - L[4])

Arrival:   If (100 - centre_pct) > CENTRE_DARKNESS_THRESHOLD
           AND that holds for CENTRE_CONFIRM_COUNT consecutive readings
           → centre found.  Stop.  Display.  Transition to AT_CENTRE.

Lost alignment:
           If edge_err > EDGE_LOST_THRESHOLD
           AND that holds for LOST_CONFIRM_COUNT consecutive readings
           → stop, return to ORIENT_SEEK_NULL from the current pose.
           Because the gradient is radial, this re-orients toward the centre.
```

`LOST_CONFIRM_COUNT` prevents transient noise from re-triggering orient.

### 4.3 Timeout

Keep a `GRADIENT_TIMEOUT_MS` safety net (e.g. 60 s). If exceeded, stop,
beep, and jump to `STATE_AT_CENTRE` with a "timeout" flag in the serial
log but still show the current coordinates on the OLED.

---

## 5. FSM Design

### 5.1 State enumeration

```c
enum GradientState {
  STATE_WAIT_PLACEMENT,     // After gradient-strip calibration; await first A press
  STATE_WAIT_START,         // Between runs; A starts next run and resets pose+timer
  STATE_ORIENT_SEEK_NULL,   // Rotating; looking for edge sensors to equalise
  STATE_ORIENT_MEASURE_A,   // Null found once; record centre reading; start 180° flip
  STATE_ORIENT_FLIP_180,    // Non-blocking rotation by PI radians
  STATE_ORIENT_MEASURE_B,   // After flip; record centre B and choose darker
  STATE_ORIENT_RESOLVE,     // If chosen heading was A, rotate 180° back; else proceed
  STATE_TRAVEL,             // Drive forward; monitor for arrival or lost alignment
  STATE_AT_CENTRE,          // Centre found (or timeout); display results, hold for OLED dwell
  STATE_DONE                // Idle; Button A restarts
};
```

### 5.2 State transition diagram

```
                          Button A
   STATE_WAIT_PLACEMENT ───────────▶ STATE_WAIT_START
                                          │ Button A (pose := 0, timer := 0)
                                          ▼
                                  STATE_ORIENT_SEEK_NULL  ◀─────────────┐
                                          │ edge_err < TOL               │
                                          │ for NULL_DWELL_MS            │
                                          ▼                              │
                                  STATE_ORIENT_MEASURE_A                 │
                                          │ record centre_A              │
                                          │ setRotation(PI)              │
                                          ▼                              │
                                  STATE_ORIENT_FLIP_180                  │
                                          │ checkRotation() == true      │
                                          ▼                              │
                                  STATE_ORIENT_MEASURE_B                 │
                                          │ record centre_B; pick darker │
                                          ▼                              │
                                  STATE_ORIENT_RESOLVE                   │
                                          │ (maybe setRotation(PI) back) │
                                          │ wait for rotation to finish  │
                                          ▼                              │
                                  STATE_TRAVEL ─── edge_err > LOST ──────┘
                                          │
                                          │ centre found  |  timeout
                                          ▼
                                  STATE_AT_CENTRE
                                          │ after RESULT_DISPLAY_MS
                                          ▼
                                  STATE_DONE
                                          │ Button A
                                          ▼
                                  STATE_WAIT_START   (loop)
```

### 5.3 Per-state logic (pseudocode)

All states run inside the existing non-blocking `loop()` after
`measureSpeeds() / updatePID() / updatePose()` and a call to
`buttonAPressed()`.

```text
STATE_WAIT_PLACEMENT:
  OLED: "Move to" / "map. A?"
  On A pressed:
    pose.initialise(0, 0, 0)
    pose_update_ts = millis()
    tone(BUZZER_PIN, 880, 100)
    state = STATE_WAIT_START

STATE_WAIT_START:
  OLED: "Ready" / "Press A"
  On A pressed:
    pose.initialise(0, 0, 0)
    run_start_ms = millis()
    null_dwell_start_ts = 0
    lost_confirm = 0
    arrival_confirm = 0
    state = STATE_ORIENT_SEEK_NULL
    Serial: "--- RUN STARTED ---"

STATE_ORIENT_SEEK_NULL:
  Read sensors, compute L[5], edge_diff, edge_err.
  Set demand speeds for slow in-place rotation:
      turn_speed_raw = ORIENT_TURN_GAIN * edge_diff
      clip |turn_speed_raw| to [ORIENT_TURN_SPEED_MIN, ORIENT_TURN_SPEED_MAX]
      positive edge_diff → rotate CW    → setDemandSpeeds( +turn, -turn )
      negative edge_diff → rotate CCW   → setDemandSpeeds( -turn, +turn )
      (Sign here follows the same convention as setTurn/checkTurn, which
       uses setDemandSpeeds(-turn_speed, turn_speed) where a positive
       turn_speed drives CCW.)
  If edge_err < EDGE_EQUAL_TOLERANCE:
      if null_dwell_start_ts == 0: null_dwell_start_ts = now
      if now - null_dwell_start_ts >= NULL_DWELL_MS:
          stopWithPID()
          theta_A     = pose.theta
          centre_A    = L[2]
          setRotation(PI)              # begin 180° flip
          state = STATE_ORIENT_FLIP_180
  else:
      null_dwell_start_ts = 0

STATE_ORIENT_FLIP_180:
  # We are still rotating; the previous state captured centre_A BEFORE
  # commanding the flip, so the flip takes the robot away from θ_A.
  # Wait for checkRotation() (non-blocking) to report complete.
  if checkRotation():
      stopWithPID()
      # Let the robot physically settle for a moment before reading ADC.
      settle_start_ts = millis()
      state = STATE_ORIENT_MEASURE_B

STATE_ORIENT_MEASURE_B:
  # Small settle delay (non-blocking) before reading
  if millis() - settle_start_ts < ORIENT_SETTLE_MS: break
  Read sensors, L[5]
  centre_B = L[2]
  theta_B  = pose.theta
  if centre_B < centre_A:     # lower % = darker = closer to centre
      chosen_theta = theta_B
      # Already facing the darker direction — no further turn needed.
      state = STATE_TRAVEL
      travel_start_ts = millis()
  else:
      chosen_theta = theta_A
      setRotation(PI)          # flip back to the original heading
      state = STATE_ORIENT_RESOLVE

STATE_ORIENT_RESOLVE:
  if checkRotation():
      stopWithPID()
      state = STATE_TRAVEL
      travel_start_ts = millis()
      arrival_confirm = 0
      lost_confirm = 0

STATE_TRAVEL:
  Read sensors, L[5], edge_err = fabs(L[0] - L[4]), darkness_centre = 100 - L[2]
  setDemandSpeed(TRAVEL_SPEED)    # equal L/R, straight forward

  if darkness_centre > CENTRE_DARKNESS_THRESHOLD:
      arrival_confirm++
  else:
      arrival_confirm = 0

  if arrival_confirm >= CENTRE_CONFIRM_COUNT:
      stopWithPID()
      captured_x = pose.x; captured_y = pose.y
      captured_time_ms = millis() - run_start_ms
      reason = "found"
      display_hold_start_ts = millis()
      beep victory chord
      reportSerialResults(reason)
      state = STATE_AT_CENTRE

  else if edge_err > EDGE_LOST_THRESHOLD:
      lost_confirm++
  else:
      lost_confirm = 0

  if lost_confirm >= LOST_CONFIRM_COUNT:
      stopWithPID()
      null_dwell_start_ts = 0
      state = STATE_ORIENT_SEEK_NULL

  if millis() - run_start_ms >= GRADIENT_TIMEOUT_MS:
      stopWithPID()
      captured_x = pose.x; captured_y = pose.y
      captured_time_ms = millis() - run_start_ms
      reason = "timeout"
      display_hold_start_ts = millis()
      reportSerialResults(reason)
      state = STATE_AT_CENTRE

STATE_AT_CENTRE:
  # OLED shows captured_x, captured_y, captured_time — see §6.
  # Stay here until RESULT_DISPLAY_MS has elapsed, then go to DONE so the
  # display continues to show the results (updateGradientDisplay skips
  # redraw in DONE too, or we leave the final screen in place).
  if millis() - display_hold_start_ts >= RESULT_DISPLAY_MS:
      state = STATE_DONE
  # Button A is ignored during the hold — operator should not be able to
  # skip past results prematurely.

STATE_DONE:
  # Keep the results visible. Only redraw OLED if it is flickering — ideally
  # updateGradientDisplay() returns early in this state.
  if buttonAPressed():
      state = STATE_WAIT_PLACEMENT   # same prompt as after calibration
      tone(BUZZER_PIN, 440, 100)
```

### 5.4 Why this ordering

- The **180° flip happens after the first null is found** (not during the
  initial rotation). This makes the rotation direction for the flip
  deterministic (always CCW via `setRotation(PI)`), and avoids having to
  reason about hysteresis when the edge sensors equalise twice during a
  single 360° sweep.
- Centre readings A and B are captured **after** the robot is stationary
  and has had `ORIENT_SETTLE_MS` to settle (the ADC + LED reflectance
  change a little while the robot is still momentum-rotating).
- `STATE_ORIENT_RESOLVE` is a separate state because the "turn back"
  itself is non-blocking and may take hundreds of ms.
- Re-entering `STATE_ORIENT_SEEK_NULL` from `STATE_TRAVEL` resets
  `null_dwell_start_ts` to force a fresh dwell measurement.

---

## 6. OLED Output

Pololu SH1106, two-line layout via `display.gotoXY(col, row)`. Columns
are characters (~21 wide); rows are text rows (0..7 addressable but
only a few fit with the library's default font).

### Per-state OLED content

| State | Row 0 | Row 1 |
|---|---|---|
| `WAIT_PLACEMENT` | `Place on` | `map. A?` |
| `WAIT_START` | `Ready` | `Press A` |
| `ORIENT_SEEK_NULL` | `Orient` | `dE:<edge_err,1dp>%` |
| `ORIENT_FLIP_180` | `Flip180` | (blank) |
| `ORIENT_MEASURE_B` | `Check` | `A:<a,0dp> B:<b,0dp>` |
| `ORIENT_RESOLVE` | `Back180` | (blank) |
| `TRAVEL` | `Going` | `C:<centre,1dp>%` |
| `AT_CENTRE` | `X:<x,0dp> Y:<y,0dp>` | `T:<secs,2dp>s` |
| `DONE` | `X:<x,0dp> Y:<y,0dp>` | `T:<secs,2dp>s` |

The **AT_CENTRE / DONE screens persist together** for `RESULT_DISPLAY_MS`
+ whatever the human takes to press A; this is the window for manual
recording. `updateGradientDisplay()` must therefore **not** overwrite
the screen while in `AT_CENTRE` or `DONE`.

Recommended defaults:
- `DISPLAY_UPDATE_INTERVAL_MS = 250` (active states only)
- `RESULT_DISPLAY_MS         = 20000` (20 s hold before accepting restart)

If the coordinates don't fit on one row, use row 0 for X, row 1 for Y,
and row 2 for T (the SH1106 has plenty of vertical space). See the
current `updateGradientDisplay()` for the `display.gotoXY(0, 2)` pattern.

---

## 7. Parameters — Complete Tuning Table

All parameters go near the top of the new `.ino`, grouped with
`// HUMAN TUNE` comments where values are first-pass estimates.

| Constant | Suggested default | Purpose |
|---|---|---|
| `ORIENT_TURN_SPEED_MIN` | `0.10` counts/ms | Minimum rotation speed during null-seeking (below this, dead-band eats it) |
| `ORIENT_TURN_SPEED_MAX` | `0.35` counts/ms | Max rotation speed during null-seeking |
| `ORIENT_TURN_GAIN` | `0.010` (counts/ms per %) | Proportional gain: `|edge_diff|` → rotation speed |
| `EDGE_EQUAL_TOLERANCE` | `3.0` % | Max \|L[0] − L[4]\| to declare the edges "equal" |
| `NULL_DWELL_MS` | `120` ms | Must stay within tolerance this long before measuring A |
| `ORIENT_SETTLE_MS` | `150` ms | Pause after stopping rotation before reading ADC |
| `TRAVEL_SPEED` | `0.25` counts/ms | Forward drive speed during `STATE_TRAVEL` |
| `EDGE_LOST_THRESHOLD` | `12.0` % | Triggers re-orientation if exceeded |
| `LOST_CONFIRM_COUNT` | `4` | Consecutive travel-loop readings above threshold required before giving up alignment |
| `CENTRE_DARKNESS_THRESHOLD` | `80.0` % | `(100 − L[2])` above this = at centre (80 means L[2] < 20%) |
| `CENTRE_CONFIRM_COUNT` | `3` | Consecutive readings required to confirm arrival |
| `GRADIENT_TIMEOUT_MS` | `60000` | Overall safety timeout |
| `RESULT_DISPLAY_MS` | `20000` | Hold the results screen for at least this long |
| `TRAVEL_LOOP_READ_INTERVAL_MS` | `20` | Sensor-read throttle inside `STATE_TRAVEL` (avoid spamming ADC) |

**Human tuning guidance** (include as comments in the `.ino`):

1. Dry-run on the gradient map with the robot held still, Serial open:
   verify that `L[0]` and `L[4]` are within a few % of each other when
   the robot is placed pointing exactly at the centre, and become
   significantly different (> 20 %) when rotated ±45° off-axis. If not,
   the transfer function or sensor physical alignment needs work before
   tuning continues.
2. Set `EDGE_EQUAL_TOLERANCE` to ≈ 2–3× the observed edge noise at a
   static pose on-axis. If the robot can never trigger null, raise it;
   if it triggers at random orientations, lower it.
3. Tune `ORIENT_TURN_GAIN` so the robot overshoots by < 5° when starting
   from a large edge error. Halve it if it hunts; double it if it's
   glacial.
4. Set `CENTRE_DARKNESS_THRESHOLD` from the black patch reading you get
   at the true centre — pick ~80 % of that darkness, not the full value
   (the robot won't sit perfectly on the bullseye).
5. `EDGE_LOST_THRESHOLD` must be comfortably above
   `EDGE_EQUAL_TOLERANCE` to avoid chatter (≥ 3–4× is sane).
6. Keep `TRAVEL_SPEED` conservative — the algorithm depends on being
   able to *react* when the edges drift, and at high speed the robot
   leaves the tolerance window before re-orienting can catch up.

---

## 8. File Layout

Create a new Arduino sketch folder so that the Part 1 code remains
intact:

```
final code/
├── 3Pi_CodeStub/                  ← Part 1 (unchanged)
│   └── …
└── GradientCentreFinder/          ← NEW (this plan)
    ├── GradientCentreFinder.ino   ← NEW — main FSM
    ├── Motors.h                   ← COPY of Part 1
    ├── Encoders.h                 ← COPY
    ├── PID.h                      ← COPY
    ├── Kinematics.h               ← COPY
    ├── SpeedMeasurement.h         ← COPY
    ├── LineSensors.h              ← COPY
    ├── GradientCalibration.h      ← COPY
    ├── oled.h                     ← COPY
    └── lcd.h                      ← COPY (not required but sketch expects it if referenced)
```

Arduino IDE requires all .h files to live next to the `.ino` — copy,
don't symlink. Do not edit any of the copied .h files; all tuned
hardware constants are in `Motors.h`, `Kinematics.h`, and
`GradientCalibration.h` and must stay as the author has them.

---

## 9. Bring-Up Order & Implementation Steps

Implement in this order to de-risk:

1. **Scaffold** the new sketch folder. Copy all .h files; copy the .ino
   and prune to only: `setup()` with gradient calibration + OLED prompt
   + `waitForButtonA()`, and a minimal `loop()` with just the
   continuous updates and `STATE_WAIT_PLACEMENT`/`STATE_WAIT_START`/
   `STATE_DONE` producing OLED and serial messages. Verify it compiles
   and runs: the robot should calibrate, beep, prompt, and echo button
   presses.

2. **Add `STATE_ORIENT_SEEK_NULL`**. Rotate in-place driven by
   `edge_diff`. Log `edge_diff` at 10 Hz. Confirm the robot finds
   **some** stable rotation direction and stops when the edges
   equalise. Tune `EDGE_EQUAL_TOLERANCE`, `ORIENT_TURN_GAIN`, speeds.
   (At this point the robot stops at a null and stays there — do not
   yet flip.)

3. **Add flip-180° logic**: `STATE_ORIENT_MEASURE_A` →
   `STATE_ORIENT_FLIP_180` → `STATE_ORIENT_MEASURE_B` →
   `STATE_ORIENT_RESOLVE`. Log `centre_A`, `centre_B`, `chosen_theta`.
   Place the robot at several known poses (pointing at centre,
   pointing away) and verify the chosen heading is correct.

4. **Add `STATE_TRAVEL`** with arrival detection only (no loss-of-
   alignment logic yet). Tune `TRAVEL_SPEED`, `CENTRE_DARKNESS_THRESHOLD`,
   `CENTRE_CONFIRM_COUNT` by driving straight across the centre a few
   times. Confirm arrival is reliable.

5. **Add loss-of-alignment → re-orient** edge in `STATE_TRAVEL`. Test
   by starting the robot deliberately off-axis so it must re-orient
   mid-travel. Tune `EDGE_LOST_THRESHOLD`, `LOST_CONFIRM_COUNT`.

6. **Finalise OLED + serial results reporting** for
   `STATE_AT_CENTRE` / `STATE_DONE`. Confirm that Button A correctly
   restarts runs, and that the results screen stays up for at least
   `RESULT_DISPLAY_MS`.

7. **Run the full acceptance suite** (§10).

---

## 10. Acceptance Criteria

| # | Criterion |
|---|---|
| 1 | `GradientCentreFinder.ino` compiles with `arduino-cli compile --fqbn pololu-a-star:avr:a-star32U4 final\ code/GradientCentreFinder/GradientCentreFinder.ino` without errors or warnings |
| 2 | On boot, the robot performs the on-board gradient-strip calibration, then displays a placement prompt |
| 3 | After a Button A press on the radial gradient, the robot resets its pose and starts the orientation routine |
| 4 | The orientation routine always terminates (within 1 full rotation + 1 flip) and leaves the robot stationary facing the darker of the two null-point measurements |
| 5 | In the travel phase the robot drives forward until either arrival or loss-of-alignment is detected; re-orientation is triggered at least once in a test where the robot is deliberately perturbed mid-travel |
| 6 | When arrival is detected, the OLED displays `X`, `Y` (in mm) and `T` (in seconds, 2 d.p.) continuously for at least 20 s |
| 7 | Serial output shows a complete timestamped trace: run start, orient measurements A/B, chosen heading, any re-orient events, arrival or timeout, and final pose |
| 8 | A second Button A press after the results hold restarts the run cycle from placement-prompt |
| 9 | Timeout at 60 s triggers a stop and a results screen flagged as a timeout in serial (but still shows the last pose on OLED) |
| 10 | Tuned hardware values (`Kp=48, Ki=0.5`, `wheel_radius=16.5 mm`, `wheel_sep=43.01 mm`, `0.88` right-bias factor, dead-bands in `Motors.h`) are unchanged from the Part 1 source |

---

## 11. Things NOT To Do

- **Do not modify** any of the copied .h files. All author-specific
  tuning lives there and must be preserved verbatim.
- **Do not add** the Part 1 edge-differential steering inside
  `STATE_TRAVEL` — this algorithm intentionally relies on
  orient-then-drive, not continuous steering. The edge sensors drive
  *re-orientation events*, not steering inputs.
- **Do not** use raw ADC values anywhere in the FSM — always pipe them
  through `gradientToPercent()`.
- **Do not** introduce a `delay()` anywhere inside `loop()`. Settle
  windows (`ORIENT_SETTLE_MS`) are implemented with `millis()` timers.
- **Do not** rename or restructure the existing state-update helpers
  (`measureSpeeds()`, `updatePID()`, `updatePose()`, `setTurn()`,
  `checkTurn()`, `setRotation()`, `checkRotation()`, `stopWithPID()`,
  `setDemandSpeed()/setDemandSpeeds()`) — they are used elsewhere in
  the coursework and must remain drop-in compatible.
- **Do not** skip the on-board `calibrateGradientSensors(25)` step in
  `setup()`. Without it, `gradientToPercent()` returns `-1.0` and the
  entire FSM is meaningless. (The alternative offline calibration
  workflow in `3Pi_CodeStub/robotics-c1-codebase/` is **not** used here —
  that project is a separate characterisation exercise.)
- **Do not** attempt to tune PID gains — they are already tuned for
  this specific robot.

---

## 12. Quick Reference — Build & Flash

```bash
# From the repo root
arduino-cli compile \
  --fqbn pololu-a-star:avr:a-star32U4 \
  "final code/GradientCentreFinder/GradientCentreFinder.ino"

arduino-cli upload \
  -p /dev/tty.usbmodem* \
  --fqbn pololu-a-star:avr:a-star32U4 \
  "final code/GradientCentreFinder/GradientCentreFinder.ino"
```

Serial monitor: 9600 baud (matches the Part 1 convention).
