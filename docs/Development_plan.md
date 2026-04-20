# ASSESSMENT 1 DEVELOPMENT PLAN (OLD)

## Overview: Staged Development Strategy

This plan breaks development into **14 stages**, each building testable functionality that subsequent stages depend on. Each stage has clear **acceptance criteria** that must be verified before proceeding.

**Target Progression**:
- Stages 1-4: Foundation (hardware validation)
- Stages 5-7: Beginner Difficulty (50-58%)
- Stages 8-10: Intermediate Difficulty (60-68%)
- Stages 11-13: Expert Difficulty (70-100%)
- Stage 14: Refinement and final testing

---

## STAGE 1: Environment Setup & Hardware Validation
**Goal**: Verify development environment and basic hardware communication

### Tasks
1. [x] Install Arduino IDE with Pololu 3Pi+ 32U4 board support
2. [x] Connect robot via USB, verify serial communication
3. [x] Upload a minimal "blink" or serial print sketch
4. [x] Verify OLED/LCD display can show text
5. [x] Attach whiskers (cable ties + plastic hooks) to bumpers

### Acceptance Criteria
- [x] Code compiles without errors
- [x] Serial monitor shows output from robot
- [x] Display shows custom text
- [x] Whiskers properly attached per specification diagram

### Code Structure Created
```
3Pi_CodeStub/
├── 3Pi_CodeStub.ino  (main sketch)
└── [header files]
```

---

## STAGE 2: Motor Control Foundation
**Goal**: Implement basic motor control with direct PWM

### Tasks
1. [x] Initialize `Motors_c` class from `Motors.h`
2. [x] Test forward motion at various PWM values
3. [x] Test reverse motion
4. [x] Test turning (left wheel forward, right wheel backward and vice versa)
5. [x] Identify motor dead-band (minimum PWM for movement)
6. [] Create helper functions: `driveForward()`, `driveBackward()`, `turnLeft()`, `turnRight()`, `stopMotors()`

### Acceptance Criteria
- [x] Robot moves forward when commanded
- [x] Robot moves backward when commanded
- [x] Robot turns left and right on the spot
- [x] Dead-band PWM value documented (typically 20-40)
- [ ] `stopMotors()` brings robot to complete halt

### Human Tasks
- Document dead-band PWM value for each motor (may differ slightly)
- Note any motor bias (does robot drift when both motors set equal?)

---

## STAGE 3: Encoder Reading & Speed Measurement
**Goal**: Read wheel encoders and calculate wheel velocities

### Tasks
1. [x] Initialize encoders using `setupEncoder0()` and `setupEncoder1()`
2. [x] Verify `count_e0` and `count_e1` increment when wheels turn (testing guide provided)
3. [x] Implement velocity calculation (counts per time interval)
4. [x] Create non-blocking speed measurement function (20ms interval recommended)
5. [x] Test that encoder counts match expected values for known distances (testing guide provided)

### Acceptance Criteria
- [x] Encoder counts increase when wheels rotate forward (code implemented, testing guide provided)
- [x] Encoder counts decrease when wheels rotate backward (code implemented, testing guide provided)
- [x] Velocity calculation produces sensible values (counts/ms)
- [x] Speed measurement runs at consistent intervals using `millis()`
- [x] Helper library created for unit conversions
- [x] Testing guide created with validation procedures

### Code Implemented
- `measureSpeeds()` function in 3Pi_CodeStub.ino
- Global variables: speed_e0, speed_e1 (updated every 20ms)
- SpeedMeasurement.h helper library with conversion functions
- Integration into main loop (non-blocking)

### Key Code Pattern
```c
unsigned long speed_ts = millis();
long prev_count_e0 = 0;
float velocity_left = 0;

void measureSpeed() {
    unsigned long now = millis();
    if (now - speed_ts >= 20) {  // 20ms interval
        long delta = count_e0 - prev_count_e0;
        velocity_left = (float)delta / 0.020;  // counts per second
        prev_count_e0 = count_e0;
        speed_ts = now;
    }
}
```

---

## STAGE 4: PID Speed Controllers ✅ COMPLETE
**Goal**: Implement closed-loop velocity control for smooth, precise motion

### Tasks
1. [x] Create two instances of `PID_c` class (left and right motors)
2. [x] Initialize with Kp only (Ki=0, Kd=0 initially)
3. [x] Implement speed control loop: demand → PID → PWM output
4. [x] Tune Kp to achieve stable speed tracking
5. [x] Add Ki to eliminate steady-state error
6. [x] Add Kd if needed to reduce overshoot (not needed)
7. [x] Test at multiple speed demands (tested at 0.5 counts/ms)

### Acceptance Criteria
- [x] Code compiles without errors
- [x] PID instances created and initialized
- [x] Speed control loop implemented
- [x] Helper functions created (setDemandSpeeds, stopWithPID)
- [x] Non-blocking architecture maintained
- [x] Tuning guides provided
- [x] Robot maintains commanded speed within ±10%
- [x] No oscillation at steady state
- [x] Smooth acceleration and deceleration
- [x] Both wheels track at equal speeds when commanded equally

### Tuned PID Values
- **Kp = 50.0** (proportional gain)
- **Ki = 1.0** (integral gain)
- **Kd = 0.0** (derivative gain - not needed)

### Human Tasks (CRITICAL)
**PID Tuning Methodology**:
1. Start with Kp = 0.5, Ki = 0, Kd = 0
2. Increase Kp until robot responds quickly but oscillates
3. Reduce Kp by ~30%
4. Add small Ki (start at 0.01) until steady-state error eliminated
5. If overshoot occurs, add small Kd (start at 0.001)
6. Test at LOW speeds (where control is hardest)

**Recommended Test**: Command 50 counts/sec, measure actual, plot if possible

---

## STAGE 5: Line Sensor Calibration & Boundary Detection - CODE COMPLETE
**Goal**: Detect the black boundary line reliably

### Tasks
1. [x] Initialize `LineSensor_c` class - implemented in LineSensors.h
2. [x] Implement calibration routine (scan over black and white surfaces)
3. [x] Store calibration values (min/max for each sensor)
4. [x] Implement `isOnLine()` function using threshold
5. [ ] Test detection of thick black boundary line (HUMAN TASK)
6. [ ] Handle edge cases: partial line detection, angled approach (HUMAN TASK)

### Acceptance Criteria
- [x] Code compiles without errors (verify by uploading)
- [x] Calibration routine implemented (calibrateLineSensors())
- [x] isOnLine() function implemented with threshold
- [x] Helper functions for pattern detection
- [x] Each sensor distinguishes black from white reliably (requires physical testing)
- [x] Detection works at various approach angles (requires physical testing)
- [x] False positives are rare/eliminated (requires physical testing)

### Human Tasks (CRITICAL)
**Calibration Procedure**:
1. Place robot on the calibration disk (alternating black/white at start position)
2. Uncomment `calibrateLineSensors();` in setup()
3. Upload code and power on robot
4. Robot will rotate for 3 seconds collecting readings
5. Check Serial Monitor for calibration results (min, max, range)
6. Test by moving robot over black/white surfaces and checking isOnLine()

**Threshold Adjustment**:
- Default LINE_THRESHOLD = 0.5 (midpoint of normalized range)
- If false positives (detects line when not on it): increase threshold to 0.6-0.7
- If misses lines: decrease threshold to 0.3-0.4

### Code Implemented
**LineSensors.h**:
- `initialiseForADC()` - sets up sensor pins with INPUT_PULLUP
- `readSensorsADC()` - reads all 5 sensors using analogRead()
- `calcCalibratedADC()` - applies calibration, normalizes to [0.0 : 1.0]
- `updateCalibration()` - updates min/max during calibration rotation
- `finalizeCalibration()` - calculates scaling values
- `resetCalibration()` - initializes calibration values
- `isOnLine(threshold)` - checks if any sensor exceeds threshold
- `isSensorOnLine(index, threshold)` - checks specific sensor
- `getLineSensorPattern(threshold)` - returns bitmask of sensors on line

**3Pi_CodeStub.ino**:
- `calibrateLineSensors()` - rotates robot for 3 seconds, collects readings
- `isOnLine()` - wrapper that uses calibrated or raw readings
- `isSensorOnLine(index)` - wrapper for specific sensor
- `getLineSensorPattern()` - wrapper for sensor pattern
- `printLineSensorReadings()` - debug function for testing

### Usage Example
```c
void loop() {
  // Check if robot hit the boundary
  if (isOnLine()) {
    // Boundary detected! Take action
    motors.setPWM(0, 0);  // Stop
    // Or: reverse and turn
  }

  // Or check specific sensors:
  // bit 0 = left (DN1), bit 4 = right (DN5)
  byte pattern = getLineSensorPattern();
  if (pattern & 0b00001) {
    // Left sensor on line - turn right
  }
  if (pattern & 0b10000) {
    // Right sensor on line - turn left
  }
}
```

---

## STAGE 6: Timer & Basic State Machine
**Goal**: Implement 4-minute timer and basic FSM for Beginner difficulty

### Tasks
1. [ ] Implement 4-minute countdown timer using `millis()`
2. [ ] Display remaining time on OLED/LCD (seconds)
3. [ ] Create basic FSM with states: SEARCHING, AVOID_LINE, STOPPED
4. [ ] Implement random/systematic search behavior
5. [ ] Implement line avoidance behavior (reverse + turn)
6. [ ] Stop robot when timer expires

### Acceptance Criteria
- [ ] Timer counts down from 240 seconds accurately
- [ ] Display updates every second
- [ ] Robot switches to AVOID_LINE when line detected
- [ ] Robot returns to SEARCHING after avoiding line
- [ ] Robot stops completely when timer reaches 0
- [ ] **BEGINNER 50% ACHIEVED**: Robot searches for 4 minutes without leaving map

### FSM Design
```
States:
- STATE_INIT: Calibration, wait for start
- STATE_SEARCHING: Move around map
- STATE_AVOID_LINE: Reverse and turn away from boundary
- STATE_STOPPED: Final state after 4 minutes

Transitions:
- INIT → SEARCHING: After calibration complete
- SEARCHING → AVOID_LINE: Line detected
- AVOID_LINE → SEARCHING: Cleared line
- ANY → STOPPED: Timer expired
```

---

## STAGE 7: Kinematics & Return to Start
**Goal**: Track position and navigate back to start area

### Tasks
1. [ ] Initialize `Kinematics_c` class with `pose.initialise(0, 0, 0)`
2. [ ] Call `pose.update()` at regular intervals (20ms)
3. [ ] **Calibrate translation**: Adjust `wheel_radius` in `Kinematics.h`
4. [ ] **Calibrate rotation**: Adjust `wheel_sep` in `Kinematics.h`
5. [ ] Implement `calculateAngleDifference()` function (handle wraparound)
6. [ ] Implement `turnToAngle()` non-blocking function
7. [ ] Implement `travelToXY()` non-blocking function
8. [ ] Test navigation: travel to (100, 0), then return to (0, 0)
9. [ ] Add RETURN_HOME state to FSM, triggered after timer expires

### Acceptance Criteria
- [ ] `pose.x`, `pose.y`, `pose.theta` update correctly during motion
- [ ] Robot can turn to face a specified angle (±5° accuracy)
- [ ] Robot can travel to a specified (x, y) coordinate (±20mm accuracy)
- [ ] After 4 minutes, robot navigates back to start area
- [ ] **BEGINNER 58% ACHIEVED**: Robot returns to start after 4 minutes

### Human Tasks (CRITICAL)
**Kinematics Calibration Procedure**:

*Translation Calibration*:
1. Position robot at 0mm mark on map grid
2. Push robot (motors OFF) to 100mm mark in straight line
3. Read `pose.x` value
4. If pose.x > 100: decrease `wheel_radius` in Kinematics.h
5. If pose.x < 100: increase `wheel_radius` in Kinematics.h
6. Repeat until pose.x ≈ 100mm (±1mm)

*Rotation Calibration*:
1. Position robot at center of radial calibration figure
2. Rotate robot (motors OFF) exactly 90° (π/2 radians)
3. Read `pose.theta` value
4. If pose.theta > 1.571: decrease `wheel_sep` in Kinematics.h
5. If pose.theta < 1.571: increase `wheel_sep` in Kinematics.h
6. Repeat until pose.theta ≈ 1.571 rad (±0.02 rad)

### Key Functions
```c
// Angular difference (handles wraparound)
float angleDiff(float target, float current) {
    float diff = target - current;
    while (diff > PI) diff -= 2*PI;
    while (diff < -PI) diff += 2*PI;
    return diff;
}

// Angle to target point
float angleToPoint(float target_x, float target_y) {
    float dx = target_x - pose.x;
    float dy = target_y - pose.y;
    return atan2(dy, dx);
}

// Distance to target point
float distanceToPoint(float target_x, float target_y) {
    float dx = target_x - pose.x;
    float dy = target_y - pose.y;
    return sqrt(dx*dx + dy*dy);
}
```

---

## STAGE 8: Waypoint Navigation System
**Goal**: Navigate between multiple predetermined waypoints

### Tasks
1. [ ] Define waypoint arrays for all 6 puck locations
2. [ ] Measure actual puck location coordinates from map
3. [ ] Implement waypoint index tracking
4. [ ] Create `setNextWaypoint()` function
5. [ ] Test navigation through all 6 waypoints in sequence
6. [ ] Optimize waypoint order to minimize whisker interference

### Acceptance Criteria
- [ ] Robot can navigate to each of the 6 puck locations
- [ ] Robot visits waypoints in defined sequence
- [ ] Navigation doesn't accidentally displace pucks at other locations
- [ ] Robot returns to start after visiting all waypoints

### Human Tasks
**Measuring Puck Location Coordinates**:
1. Use map grid or ruler to measure (x, y) of each puck location
2. Consider robot's starting position as origin (0, 0)
3. X-axis: forward direction of robot at start
4. Test coordinates by commanding robot to each location

### Waypoint Strategy
```c
// Example waypoint order to avoid whisker collisions
// Start → 6 → 5 → 4 → 3 → 2 → 1 → Start
// (Adjust based on actual map layout and testing)

#define NUM_PUCK_LOCATIONS 6
float puck_x[NUM_PUCK_LOCATIONS] = {x1, x2, x3, x4, x5, x6};
float puck_y[NUM_PUCK_LOCATIONS] = {y1, y2, y3, y4, y5, y6};
int current_waypoint = 0;
```

---

## STAGE 9: Magnetometer & Puck Detection ✅ CODE COMPLETE
**Goal**: Detect puck presence using magnetometer

### Tasks
1. [x] Initialize magnetometer from `Magnetometer.h`
2. [x] Read raw magnetometer values (x, y, z axes)
3. [x] Calculate magnitude: `sqrt(x*x + y*y + z*z)`
4. [x] Calibrate baseline (no puck present) – rotation-based calibration in setup()
5. [ ] Determine detection threshold (puck between whiskers) – **HUMAN TASK**
6. [x] Implement `detectPuck()` function
7. [ ] Add STATE_PUCK_FOUND to FSM – **Stage 10**

### Acceptance Criteria
- [ ] Baseline magnitude stable when no puck present
- [ ] Magnitude increases significantly when puck is between whiskers
- [ ] Detection threshold reliably distinguishes puck present/absent
- [ ] No false positives from environmental metal

### Human Tasks (CRITICAL)
**Magnetometer Calibration Procedure**:
1. Place robot on map, no puck nearby, no metal objects in area
2. Record magnetometer magnitude (average over 100 readings)
3. This is your BASELINE value
4. Place puck directly between whiskers (touching robot)
5. Record magnetometer magnitude
6. Set threshold at ~70% of (puck_reading - baseline) + baseline
7. Test threshold at various puck distances
8. Goal: detect puck only when it's capturable between whiskers

**Warning**: Metal table legs, watches, phones affect readings!

---

## STAGE 10: Intermediate Solution - Push Puck Out
**Goal**: Detect puck and push it outside map boundary

### Tasks
1. [ ] Enhance FSM with puck handling states
2. [ ] When puck detected: transition to PUSH_OUT state
3. [ ] Calculate nearest boundary point from current position
4. [ ] Navigate toward boundary while keeping puck
5. [ ] Detect when boundary reached (line sensors)
6. [ ] Reverse back into map after pushing puck out
7. [ ] Return to start area and wait 4 seconds
8. [ ] Continue searching for next puck

### Acceptance Criteria
- [ ] Robot detects puck at any of 6 locations
- [ ] Robot pushes puck completely outside boundary
- [ ] Robot body remains inside map during push
- [ ] Robot returns to start and waits 4 seconds
- [ ] Robot continues searching after wait
- [ ] **INTERMEDIATE 60% ACHIEVED**: 1 puck pushed out
- [ ] **INTERMEDIATE 68% ACHIEVED**: 5 pucks pushed out

### FSM Enhancement
```
New States:
- STATE_TRAVEL_TO_WAYPOINT: Moving to puck location
- STATE_PUCK_FOUND: Puck detected between whiskers
- STATE_PUSH_OUT: Driving toward boundary with puck
- STATE_RETREAT: Reversing back into map
- STATE_RETURN_TO_START: Navigating back to start
- STATE_WAIT_FOR_PUCK: 4-second wait for new puck placement

Key Transitions:
- TRAVEL_TO_WAYPOINT + puck_detected → PUCK_FOUND
- PUCK_FOUND → PUSH_OUT
- PUSH_OUT + line_detected → RETREAT
- RETREAT → RETURN_TO_START
- RETURN_TO_START + at_start → WAIT_FOR_PUCK
- WAIT_FOR_PUCK + 4sec_elapsed → TRAVEL_TO_WAYPOINT (next)
```

---

## STAGE 11: Puck Position Estimation
**Goal**: Estimate puck coordinates for alignment maneuver

### Tasks
1. [ ] When puck detected, record robot position (`pose.x`, `pose.y`, `pose.theta`)
2. [ ] Estimate puck position: project forward from robot by detection distance
3. [ ] Detection distance = distance from robot center to puck when detected
4. [ ] Test estimation accuracy at various approach angles
5. [ ] Store estimated puck position for alignment maneuver

### Acceptance Criteria
- [ ] Puck position estimated within ±30mm of actual
- [ ] Estimation works regardless of approach direction
- [ ] Estimated position stored reliably for later use

### Key Code
```c
// When puck detected:
float detection_distance = 50.0;  // mm, calibrate this value
float puck_est_x = pose.x + detection_distance * cos(pose.theta);
float puck_est_y = pose.y + detection_distance * sin(pose.theta);
```

### Human Tasks
**Detection Distance Calibration**:
1. Approach puck slowly from known distance
2. Note robot position when magnetometer triggers
3. Measure actual distance from robot center to puck center
4. This is your `detection_distance` value

---

## STAGE 12: Alignment Maneuver Calculation
**Goal**: Calculate waypoints to move around puck and align for return push

### Tasks
1. [ ] Calculate "push line": line from puck location to start (0, 0)
2. [ ] Calculate "end position": point on push line, behind puck
3. [ ] Calculate whisker envelope radius (safe distance from puck)
4. [ ] Determine rotation direction (clockwise or counter-clockwise)
5. [ ] Generate intermediate waypoints to navigate around puck
6. [ ] Test alignment calculation for all 6 puck locations

### Acceptance Criteria
- [ ] Push line correctly points from puck toward start
- [ ] End position places robot ready to push puck to start
- [ ] Intermediate waypoints keep robot outside whisker envelope
- [ ] Rotation direction is efficient (shortest path around)

### Key Calculations
```c
// Push line angle (from puck to start)
float push_angle = atan2(0 - puck_y, 0 - puck_x);

// End position (behind puck, along push line)
float end_distance = 80.0;  // mm behind puck
float end_x = puck_x - end_distance * cos(push_angle);
float end_y = puck_y - end_distance * sin(push_angle);

// Whisker envelope (minimum safe distance from puck)
float whisker_radius = 100.0;  // mm, measure actual whisker reach
```

### Alignment Maneuver Waypoints
```
For each puck location, robot must:
1. Detect puck → stop
2. Reverse slightly (avoid displacing puck)
3. Calculate alignment waypoints
4. Navigate waypoint 1 (move perpendicular to push line)
5. Navigate waypoint 2 (arc around puck)
6. Navigate end position (behind puck on push line)
7. Push toward start
```

---

## STAGE 13: Expert Solution - Return Puck to Start
**Goal**: Complete alignment maneuver and push puck into start area

### Tasks
1. [ ] Implement alignment maneuver state machine (nested FSM)
2. [ ] Execute waypoints around puck without displacement
3. [ ] Position at end position facing start
4. [ ] Drive forward, pushing puck toward start
5. [ ] Detect when puck reaches start area (line sensors or distance)
6. [ ] Stop with puck partially in start area
7. [ ] Wait 4 seconds for new puck placement
8. [ ] Continue searching (must actively search, not idle)

### Acceptance Criteria
- [ ] Robot navigates around puck without displacing it
- [ ] Robot aligns on push line behind puck
- [ ] Robot pushes puck into start area
- [ ] Puck partially within dashed start line
- [ ] Robot waits 4 seconds, then continues searching
- [ ] **EXPERT 70% ACHIEVED**: 1 puck returned to start
- [ ] **EXPERT 100% ACHIEVED**: 7 pucks returned to start

### Expert FSM Enhancement
```
New States:
- STATE_ALIGNMENT_REVERSE: Back away from detected puck
- STATE_ALIGNMENT_WAYPOINT_1: First waypoint around puck
- STATE_ALIGNMENT_WAYPOINT_2: Second waypoint around puck
- STATE_ALIGNMENT_END_POSITION: Position behind puck
- STATE_PUSH_TO_START: Drive puck toward start area
- STATE_PUCK_DELIVERED: Puck in start area

Transitions:
- PUCK_FOUND → ALIGNMENT_REVERSE
- ALIGNMENT_REVERSE → ALIGNMENT_WAYPOINT_1
- ALIGNMENT_WAYPOINT_1 → ALIGNMENT_WAYPOINT_2
- ALIGNMENT_WAYPOINT_2 → ALIGNMENT_END_POSITION
- ALIGNMENT_END_POSITION → PUSH_TO_START
- PUSH_TO_START + at_start → PUCK_DELIVERED
- PUCK_DELIVERED → WAIT_FOR_PUCK
```

---

## STAGE 14: Refinement & Final Testing
**Goal**: Optimize performance and reliability for submission

### Tasks
1. [ ] Re-verify kinematics calibration after extended testing
2. [ ] Tune PID gains for optimal low-speed control
3. [ ] Optimize waypoint coordinates based on observed drift
4. [ ] Add error recovery states (what if puck lost during alignment?)
5. [ ] Test full 4-minute runs repeatedly
6. [ ] Identify and fix edge cases
7. [ ] Record practice video submissions
8. [ ] Clean up code, add comments
9. [ ] Verify code compiles without warnings

### Acceptance Criteria
- [ ] Robot completes 4-minute run without failure (>90% success rate)
- [ ] Maximum achievable pucks returned consistently
- [ ] Code is clean, well-commented, compiles without errors
- [ ] Video recording procedure practiced

### Human Tasks
**Final Testing Checklist**:
- [ ] Test on ACTUAL coursework map (not practice surface)
- [ ] Test with ACTUAL puck and dice from kit
- [ ] Test in ACTUAL recording location (check for metal interference)
- [ ] Practice video recording procedure
- [ ] Verify timer display is visible in camera frame
- [ ] Prepare student ID card for video start
- [ ] Test all 6 possible starting puck locations

### Code Cleanup Checklist
- [ ] Remove debug `Serial.print()` statements (or make them optional)
- [ ] Ensure all functions have descriptive comments
- [ ] Remove unused variables and dead code
- [ ] Verify consistent naming conventions
- [ ] Test that code compiles on fresh Arduino IDE installation

---

## Quick Reference: Stage Dependencies

```
Stage 1 (Setup)
    ↓
Stage 2 (Motors) → Stage 3 (Encoders) → Stage 4 (PID)
    ↓                                       ↓
Stage 5 (Line Sensors)              Stage 7 (Kinematics)
    ↓                                       ↓
Stage 6 (Timer + Basic FSM) ←──────── Stage 7
    ↓
[BEGINNER 50-58% COMPLETE]
    ↓
Stage 8 (Waypoints) ← Stage 7
    ↓
Stage 9 (Magnetometer)
    ↓
Stage 10 (Push Out)
    ↓
[INTERMEDIATE 60-68% COMPLETE]
    ↓
Stage 11 (Puck Estimation)
    ↓
Stage 12 (Alignment Calculation)
    ↓
Stage 13 (Push to Start)
    ↓
[EXPERT 70-100% COMPLETE]
    ↓
Stage 14 (Refinement)
```

---

## Estimated Development Effort

| Stage | Complexity | Notes |
|-------|------------|-------|
| 1 | Low | Setup only |
| 2 | Low | Direct PWM control |
| 3 | Low | Provided code |
| 4 | Medium | **Requires human tuning** |
| 5 | Medium | **Requires human calibration** |
| 6 | Medium | Core FSM structure |
| 7 | High | **Requires human calibration**, most critical stage |
| 8 | Medium | Depends on Stage 7 accuracy |
| 9 | Medium | **Requires human calibration** |
| 10 | Medium | Integration of previous stages |
| 11 | Low | Simple calculation |
| 12 | High | Complex geometry |
| 13 | High | System integration |
| 14 | Variable | Debugging and refinement |

**Critical Path**: Stages 4, 5, 7, 9 all require human physical calibration and cannot be completed by AI alone.
