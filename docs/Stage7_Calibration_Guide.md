# Stage 7: Kinematics Calibration & Testing Guide

## Overview

Stage 7 implements dead-reckoning navigation using wheel encoders. The robot tracks its position (x, y, θ) over time, enabling precise rotation and navigation to target coordinates.

**⚠️ CRITICAL**: Calibration is MANDATORY for accurate navigation. Uncalibrated kinematics will accumulate error rapidly!

---

## Prerequisites

- ✅ Stage 4 complete (PID speed control working)
- ✅ Encoders functional (verified in Stage 3)
- ✅ Motors calibrated (Stage 2)
- ✅ Coursework map with 10mm grid and radial calibration figure

---

## Part 1: Translation Calibration (20-30 minutes)

### Goal
Adjust `wheel_radius` parameter so robot accurately measures forward/backward distance.

### Procedure

**Step 1: Test Current Calibration**
```c
void loop() {
  use_pid_control = false;  // Disable PID for manual testing

  // Print current position
  Serial.print("x=");
  Serial.print(pose.x, 2);
  Serial.print(", y=");
  Serial.print(pose.y, 2);
  Serial.print(", theta=");
  Serial.println(pose.theta, 4);

  measureSpeeds();
  updatePose();  // Update kinematics

  delay(100);
}
```

**Step 2: Physical Test**
1. Upload code, open Serial Monitor
2. Place robot on map grid with wheel axles aligned at **0mm mark**
3. **With motors OFF**, gently push robot forward to **100mm mark**
4. Keep robot as straight as possible
5. Read `pose.x` value from Serial Monitor

**Step 3: Adjust wheel_radius**

Open `Kinematics.h` (line 29):

| Observed pose.x | Action |
|-----------------|--------|
| **pose.x > 100** (e.g., 105mm) | **Decrease** `wheel_radius`<br>Try: 15.0 → 14.3 |
| **pose.x < 100** (e.g., 95mm) | **Increase** `wheel_radius`<br>Try: 15.0 → 15.8 |
| **pose.x ≈ 100** (±1mm) | ✅ Calibrated! |

**Step 4: Iterate**
1. Change `wheel_radius` in Kinematics.h
2. Re-upload code
3. Power off robot (to reset pose to 0,0,0)
4. Repeat physical test
5. Continue until pose.x ≈ 100mm (±1mm tolerance)

**Expected final value**: 14.5 - 16.0mm (varies by robot)

---

## Part 2: Rotation Calibration (20-30 minutes)

### Goal
Adjust `wheel_sep` parameter so robot accurately measures rotation angle.

### Procedure

**Step 1: Position Robot**
1. Place robot at center of radial calibration figure on map
2. Align wheel axles with horizontal line (0° line)
3. Align gap between bumper whiskers with vertical line

**Step 2: Test Current Rotation Calibration**

Keep same test code from Part 1, but now watch `pose.theta`:

```c
void loop() {
  Serial.print("theta=");
  Serial.println(pose.theta, 4);  // Print theta with 4 decimal places

  measureSpeeds();
  updatePose();
  delay(100);
}
```

**Step 3: Physical Rotation Test**
1. Upload code, open Serial Monitor
2. With robot centered on radial figure
3. **With motors OFF**, rotate robot **90° clockwise** (to π/2 radians = 1.5708 rad)
4. Use radial figure lines to ensure exactly 90°
5. Read `pose.theta` value

**Step 4: Adjust wheel_sep**

Open `Kinematics.h` (line 30):

| Observed pose.theta | Action |
|---------------------|--------|
| **theta > 1.571** (e.g., 1.65 rad) | **Decrease** `wheel_sep`<br>Try: 45.0 → 43.0 |
| **theta < 1.571** (e.g., 1.48 rad) | **Increase** `wheel_sep`<br>Try: 45.0 → 47.0 |
| **theta ≈ 1.571** (±0.02 rad) | ✅ Calibrated! |

**Step 5: Iterate**
1. Change `wheel_sep` in Kinematics.h
2. Re-upload code
3. Power off robot (to reset pose)
4. Repeat rotation test
5. Continue until pose.theta ≈ 1.5708 rad (±0.02 tolerance)

**Expected final value**: 43.0 - 47.0mm (varies by robot)

**Note**: 1.5708 radians = π/2 = 90°

---

## Part 3: Testing Navigation Functions (30-45 minutes)

### Test 1: Simple Turn Test

```c
void loop() {
  static bool test_started = false;

  if (!test_started) {
    Serial.println("Starting turn test: rotating to 90 degrees");
    setTurn(PI/2);  // Turn to 90° (1.5708 radians)
    test_started = true;
  }

  if (checkTurn()) {
    Serial.println("Turn complete!");
    Serial.print("Final angle: ");
    Serial.println(pose.theta, 4);
    // Stop testing (do nothing further)
  }

  measureSpeeds();
  updatePID();
  updatePose();
}
```

**Expected**: Robot rotates on spot to face 90°, stops within ~±3°

**If problems**:
- Robot overshoots: Reduce `TURN_GAIN` (line ~176)
- Robot too slow: Increase `TURN_GAIN`
- Robot oscillates: Reduce `TURN_GAIN` significantly

### Test 2: Travel Forward Test

```c
void loop() {
  static bool test_started = false;
  use_pid_control = true;  // Enable PID for smooth travel

  if (!test_started) {
    Serial.println("Starting travel test: moving to (200, 0)");
    setTravel(200, 0);  // Travel 200mm forward
    test_started = true;
  }

  if (checkTravel()) {
    Serial.println("Travel complete!");
    Serial.print("Final position: x=");
    Serial.print(pose.x, 2);
    Serial.print(", y=");
    Serial.println(pose.y, 2);
  }

  measureSpeeds();
  updatePID();
  updatePose();
}
```

**Expected**: Robot travels forward ~200mm, stops

**If problems**:
- Veers left/right: Adjust `HEADING_CORRECTION_GAIN` (line ~183)
- Too slow: Increase `TRAVEL_SPEED` (line ~182)
- Overshoots: Reduce `TRAVEL_THRESHOLD` or reduce speed

### Test 3: Combined Navigation (travelToXY)

```c
void loop() {
  static bool test_started = false;
  use_pid_control = true;

  if (!test_started) {
    Serial.println("Navigating to (150, 150)");
    travelToXY(150, 150);  // Move to diagonal position
    test_started = true;
  }

  if (updateTravelToXY()) {
    Serial.println("Navigation complete!");
    Serial.print("Position: (");
    Serial.print(pose.x, 1);
    Serial.print(", ");
    Serial.print(pose.y, 1);
    Serial.println(")");
  }

  measureSpeeds();
  updatePID();
  updatePose();
}
```

**Expected**:
1. Robot rotates to face (150, 150)
2. Travels forward to target
3. Stops when within 10mm

### Test 4: Multiple Waypoints

```c
// Define waypoints
float waypoints_x[] = {100, 100, 0, 0};
float waypoints_y[] = {0, 100, 100, 0};
int num_waypoints = 4;
int current_waypoint = 0;

void loop() {
  use_pid_control = true;

  static bool navigating = false;

  if (!navigating && current_waypoint < num_waypoints) {
    Serial.print("Going to waypoint ");
    Serial.println(current_waypoint);
    travelToXY(waypoints_x[current_waypoint], waypoints_y[current_waypoint]);
    navigating = true;
  }

  if (navigating && updateTravelToXY()) {
    Serial.println("Waypoint reached!");
    current_waypoint++;
    navigating = false;
  }

  measureSpeeds();
  updatePID();
  updatePose();
}
```

**Expected**: Robot visits all 4 waypoints in sequence (square pattern)

---

## Part 4: Tuning Parameters

### Turn Behavior

**TURN_GAIN** (default: 50.0)
- Controls how aggressively robot rotates
- **Too high**: Oscillation, overshooting
- **Too low**: Slow, may not overcome friction
- **Typical range**: 30.0 - 100.0

**TURN_THRESHOLD** (default: 0.05 rad ≈ 2.9°)
- How close to target angle before stopping
- **Too small**: May never reach (stuck below dead-band)
- **Too large**: Inaccurate positioning
- **Typical range**: 0.03 - 0.10 radians

### Travel Behavior

**TRAVEL_SPEED** (default: 0.3 counts/ms)
- Base forward speed during travel
- Should match PID-tuned speeds from Stage 4
- **Typical range**: 0.2 - 0.8 counts/ms

**HEADING_CORRECTION_GAIN** (default: 0.5)
- How strongly to correct heading while traveling
- **Too high**: Zigzag motion, oscillation
- **Too low**: Drifts off course
- **Typical range**: 0.3 - 1.0

**TRAVEL_THRESHOLD** (default: 10.0mm)
- How close to target before stopping
- **Too small**: May never reach exactly
- **Too large**: Inaccurate positioning
- **Typical range**: 5.0 - 20.0mm

---

## Common Problems & Solutions

### Problem: Robot reports position but doesn't match reality

**Causes:**
1. `wheel_radius` not calibrated → Recalibrate translation
2. `wheel_sep` not calibrated → Recalibrate rotation
3. Wheel slip (sudden movements) → Use smoother acceleration
4. Not calling `updatePose()` regularly → Check it's in loop()

### Problem: Robot turns wrong direction

**Causes:**
1. Encoder mapping swapped → Already fixed in Stage 4
2. Sign error in turn speed → Check code uses `+turn_speed, -turn_speed`

### Problem: Robot drifts during straight travel

**Causes:**
1. Motor bias → PID should compensate (verify Stage 4 works)
2. `HEADING_CORRECTION_GAIN` too low → Increase gain
3. Surface not level → Test on flat surface

### Problem: Position drifts over time

**Expected behavior!** Dead-reckoning accumulates error:
- Wheel slip
- Encoder quantization
- Model simplifications
- Floor irregularities

**Mitigation:**
- Good calibration reduces drift
- Use boundary detection (Stage 5) to reset position
- Keep movements short and precise
- For Assessment 1: Accept some drift, use robust behaviors

---

## Acceptance Criteria

✅ **Translation calibrated**: pose.x reads 100mm when pushed 100mm (±1mm)
✅ **Rotation calibrated**: pose.theta reads 1.571 rad when rotated 90° (±0.02 rad)
✅ **Turn function works**: Robot can turn to any angle from any starting angle
✅ **Travel function works**: Robot can travel to nearby x,y coordinates
✅ **travelToXY works**: Robot rotates then travels to target
✅ **Waypoint navigation**: Robot can visit multiple waypoints in sequence

---

## Next Steps

After Stage 7 calibration:
1. **Test navigation accuracy** over extended time (2-4 minutes)
2. **Measure drift** - how far off after multiple movements?
3. **Integrate with Stage 5** - use line detection to avoid boundary
4. **Stage 6**: Add timer and state machine for autonomous operation

---

## Quick Reference: Key Functions

| Function | Purpose | Usage |
|----------|---------|-------|
| `updatePose()` | Update x,y,θ from encoders | Call every loop iteration |
| `setTurn(angle)` | Start turn to angle | Call once to initiate |
| `checkTurn()` | Update turn, check complete | Call repeatedly in loop |
| `setTravel(x,y)` | Start travel to x,y | Call once to initiate |
| `checkTravel()` | Update travel, check complete | Call repeatedly in loop |
| `travelToXY(x,y)` | Navigate to x,y (rotate + travel) | Call once to initiate |
| `updateTravelToXY()` | Update navigation state | Call repeatedly in loop |
| `angleDiff(target, current)` | Calculate turn direction | Used internally |
| `angleToPoint(x,y)` | Angle from robot to point | Used internally |
| `distanceToPoint(x,y)` | Distance from robot to point | Used internally |

---

## Notes

- **Non-blocking architecture**: All functions return quickly, must be called repeatedly
- **PID integration**: Navigation uses PID speed control when `use_pid_control = true`
- **Global frame**: Defined at power-on by `pose.initialise(0,0,0)` in setup()
- **Units**: Distances in mm, angles in radians
- **Error accumulation**: Expected! Calibration minimizes but doesn't eliminate drift

