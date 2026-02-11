# PROJECT LOG

## Development History
*(Record all development work, bugs found, solutions implemented)*

**Session 1 - Stage 1 Complete (28 Jan 2026)**
- Arduino IDE installed with Pololu 3Pi+ 32U4 board support
- Robot connected via USB and verified serial communication working
- Minimal test sketch compiled and uploaded successfully
- OLED/LCD display verified to show custom text output
- Whiskers (cable ties + plastic hooks) attached to front bumpers for puck manipulation
- All Stage 1 acceptance criteria passed
- Ready to proceed with Stage 2: Motor Control Foundation

**Session 2 - Stage 2 Code Complete (28 Jan 2026)**
- **Completed Motors.h implementation** (Steps 2-4):
  - Fixed pin definitions: L_PWM=10, L_DIR=16, R_PWM=9, R_DIR=15
  - Implemented `initialise()` function: configures pins, sets safe initial state (motors OFF)
  - Implemented `setPWM()` function with full safety features:
    - Handles positive/negative PWM (forward/backward)
    - Converts to absolute values for analogWrite()
    - Constrains to MAX_PWM (180) to prevent motor damage
    - Sets direction pins based on power sign
  - Added FWD/REV direction constants for code readability
- **Created non-blocking test code** in 3Pi_CodeStub.ino:
  - State machine cycles through motor test patterns every 2 seconds
  - No delay() calls - fully non-blocking architecture
  - Test states disabled by default for safety
- **Code compiles without errors** ✅
- **Completed Step 5: Physical Testing & Calibration** ✅
  - Dead-band identification: PWM +15, -14 (left motor), PWM +13, -12 (right motor)
  - Direction validated: Positive = forward, negative = backward
  - Motor bias: ~13° left drift at PWM=50 over 280mm
  - All calibration data documented in Motors.h
- **STAGE 2 COMPLETE** - All acceptance criteria met ✅

**Session 3 - Stage 3 COMPLETE (28 Jan 2026)**
- **Implemented Speed Measurement System**:
  - Non-blocking `measureSpeeds()` function (20ms intervals)
  - Calculates wheel velocities from encoder counts
  - Speed units: encoder counts per millisecond
  - Tracks both wheels independently (speed_e0, speed_e1)
  - Fully integrated with existing code structure
- **Created SpeedMeasurement.h helper library**:
  - Unit conversion functions (counts → mm, counts/ms → mm/s)
  - Encoder utility functions (reset, get distance traveled)
  - Debug functions for testing and calibration
  - Well-documented with usage examples
- **Updated main sketch (3Pi_CodeStub.ino)**:
  - Speed measurement initialized in setup()
  - measureSpeeds() called in loop()
  - Ready for PID integration (Stage 4)
- **Created testing guide**: Stage3_SpeedTest_Instructions.md
  - 4 comprehensive tests with acceptance criteria
  - Troubleshooting guide
  - Expected values and calibration procedures
- **Code compiles without errors** ✅
- **STAGE 3 COMPLETE** - Ready for physical validation ✅
- **Next**: Stage 4 PID Speed Controllers (requires physical tuning)

**Session 4 - Stage 4 COMPLETE (28 Jan 2026)**
- **Implemented PID Speed Controllers**:
  - Two PID instances (left_pid, right_pid) for independent motor control
  - PID control variables: demand_speed_left, demand_speed_right
  - Enable/disable flag: use_pid_control (starts disabled for safety)
  - PID update function: updatePID() (runs every 50ms)
- **Created helper functions**:
  - setDemandSpeeds(left, right) - Set target speeds
  - setDemandSpeed(speed) - Set same speed for both wheels
  - stopWithPID() - Smooth stop
- **Integration into main loop**:
  - PID initialized in setup() with starting gains (Kp=1.0, Ki=0.0, Kd=0.0)
  - updatePID() called in loop() after measureSpeeds()
  - Non-blocking architecture maintained
  - Automatic PID reset when disabled (prevents integral windup)
- **Created comprehensive tuning guides**:
  - Stage4_PID_TuningGuide.md - Full 7-step tuning methodology
  - Stage4_QuickStart.md - 5-minute quick test
  - Includes troubleshooting, common problems, expected values
- **Code compiles without errors** ✅
- **STAGE 4 CODE COMPLETE** - Ready for physical tuning ✅
- **Next**: Physical PID tuning (HUMAN TASK, 30-90 min)

**Session 5 - Stage 4 FULLY COMPLETE (29 Jan 2026)**
- **Fixed critical encoder mapping bug**:
  - Discovered encoder-to-motor assignment was swapped
  - From Encoders.h: count_e0 = RIGHT wheel, count_e1 = LEFT wheel
  - PID was using e0 for left motor and e1 for right motor (backwards!)
  - Fixed by swapping encoder readings in updatePID()
- **Fixed initial Kp gain issue**:
  - Original Kp = 1.0 was far too small (produced PWM ~0.3, below dead-band)
  - Changed to Kp = 100.0 initially, then tuned down to Kp = 50.0
  - Speed units (counts/ms) are small, so Kp must be large
- **Completed PID tuning**:
  - Final values: Kp = 50.0, Ki = 1.0, Kd = 0.0
  - Both wheels track 0.5 counts/ms demand within ±10%
  - No oscillation, stable control
  - Motor bias automatically compensated by PID
- **STAGE 4 FULLY COMPLETE** ✅

**Session 6 - Stage 5 CODE COMPLETE (29 Jan 2026)**
- **Completed LineSensors.h implementation**:
  - Fixed typo: constructor name changed from `LineSensor_c` to `LineSensors_c`
  - Implemented `initialiseForADC()`: configures pins with INPUT_PULLUP
  - Implemented `readSensorsADC()`: reads all 5 sensors via analogRead()
  - Implemented `calcCalibratedADC()`: normalizes readings to [0.0 : 1.0]
  - Added calibration functions: resetCalibration(), updateCalibration(), finalizeCalibration()
  - Added detection functions: isOnLine(), isSensorOnLine(), getLineSensorPattern()
- **Added calibration routine to main sketch**:
  - `calibrateLineSensors()` function rotates robot for 3 seconds
  - Collects min/max values for each sensor over black/white surfaces
  - Reports calibration results via Serial
- **Added helper functions to main sketch**:
  - `isOnLine()` - wrapper with fallback for uncalibrated sensors
  - `isSensorOnLine(index)` - check specific sensor
  - `getLineSensorPattern()` - bitmask of sensors on line
  - `printLineSensorReadings()` - debug output
- **Added configuration variables**:
  - `LINE_THRESHOLD = 0.5` (adjustable)
  - `line_sensors_calibrated` flag
- **STAGE 5 CODE COMPLETE** - Requires physical testing ✅
- **Stage 5 tested and working** - Line detection functional ✅

**Session 7 - Stage 7 CODE COMPLETE (29 Jan 2026)**
- **Skipped Stage 6** (Timer & FSM) - will integrate later
- **Jumped directly to Stage 7** (Kinematics & Navigation):
  - Kinematics.h already provided with complete odometry implementation
  - Added pose update timing (20ms intervals, matches speed measurement)
  - Implemented navigation variables and state tracking
- **Implemented core navigation functions**:
  - `angleDiff()` - calculates smallest angle between two orientations (handles 0°/360° wraparound)
  - `angleToPoint()` - calculates angle from robot to target x,y
  - `distanceToPoint()` - calculates distance from robot to target x,y
- **Implemented turn behavior** (non-blocking):
  - `setTurn(angle)` - initialize turn to specific orientation
  - `checkTurn()` - update turn, proportional control, returns completion status
  - Uses TURN_GAIN for responsiveness, TURN_THRESHOLD for stopping
- **Implemented travel behavior** (non-blocking):
  - `setTravel(x,y)` - initialize travel to coordinates
  - `checkTravel()` - update travel with heading correction
  - Uses TRAVEL_SPEED and HEADING_CORRECTION_GAIN for smooth motion
- **Implemented combined navigation**:
  - `travelToXY(x,y)` - high-level function: rotate then travel
  - `updateTravelToXY()` - state machine manages IDLE → ROTATING → MOVING → COMPLETE
  - Enables robust waypoint navigation
- **Added updatePose() to main loop**:
  - Called every 20ms after measureSpeeds() and updatePID()
  - Tracks robot position (x, y, θ) via dead-reckoning
- **Created comprehensive calibration guide**:
  - Stage7_Calibration_Guide.md with step-by-step procedures
  - Translation calibration (adjust wheel_radius)
  - Rotation calibration (adjust wheel_sep)
  - Testing procedures for all navigation functions
  - Parameter tuning guidelines
- **STAGE 7 CODE COMPLETE** - Requires physical calibration ✅

**Session 8 - Stage 6 FULLY COMPLETE (3 Feb 2026)**
- **Kinematics calibrated** by user (physical task completed)
- **Implemented Button A start control**:
  - Blocking wait in setup() after all hardware init, before calibration
  - BUTTON_A_PIN = 14 (active-low with INPUT_PULLUP)
  - Debouncing implemented; beep on start for audio confirmation
- **Implemented full Stage 6 FSM with 5 states**:
  - `STATE_SEARCHING`: Drive forward, check line sensors each iteration
  - `STATE_REVERSING`: Back up for 400ms after boundary hit
  - `STATE_TURNING`: Turn random amount (300-1000ms) in random direction
  - `STATE_RETURN_HOME`: Timer expired → uses travelToXY(0,0) to navigate back
  - `STATE_STOPPED`: Arrived at start, motors held off
- **4-minute countdown timer**:
  - Started via millis() AFTER calibration so calibration time excluded
  - Serial prints remaining seconds + current state every 1 s
  - On expiry: transitions to STATE_RETURN_HOME regardless of current state
- **Return-to-start navigation**:
  - Uses existing Stage 7 travelToXY(0,0) / updateTravelToXY()
  - Rotate-then-travel to origin; stops on arrival
- **LINE_THRESHOLD raised to 0.7** (was 0.5) to fix false positives
- **Cleaned up dead code**: removed unused motor-test vars, robot_started flag, duplicate button prompt
- **STAGE 6 FULLY COMPLETE** ✅ – targets Beginner 50-58%

**Session 9 - Stage 8 CODE COMPLETE (3 Feb 2026)**
- **Extracted puck waypoints from Map.svg**:
  - Chained nested SVG group `translate()` transforms to get absolute positions
  - Verified with Python script: all 6 pucks inside map boundary, units = mm
  - Origin = start-area centre; robot assumed to face +X at launch
  - Waypoint coordinates (robot frame, mm):
    - WP1: (60.6, 262.9), WP2: (260.7, 211.4), WP3: (429.0, 267.7)
    - WP4: (408.8, 113.9), WP5: (354.3, -10.9), WP6: (156.7, 83.9)
- **Replaced wandering FSM with waypoint navigation FSM**:
  - `STATE_SEARCHING` removed; replaced by `STATE_TRAVEL_TO_WAYPOINT` and `STATE_AT_WAYPOINT`
  - Robot visits waypoints in order, cycling back to WP1 after WP6
  - `waypoint_order[]` array allows reordering the circuit without moving coordinates
  - 1-second pause at each waypoint (WAYPOINT_PAUSE_MS) reserved for Stage 9 puck detection
- **Safety: boundary detection during travel**:
  - `isOnLine()` checked every loop iteration while in STATE_TRAVEL_TO_WAYPOINT
  - On line hit: stopWithPID → STATE_REVERSING → STATE_TURNING → resume travel to the SAME waypoint
  - Prevents robot leaving the map even if odometry drifts
- **Fixed odometry corruption bug**:
  - `pose.initialise(0,0,0)` was called BEFORE `calibrateLineSensors()` which spins robot 3 s
  - Encoder deltas from that spin would corrupt first pose.update() in loop()
  - Added second `pose.initialise(0,0,0)` AFTER calibration to reset cleanly
- **Removed unused FORWARD_SPEED define** (was 0.4; no longer needed)
- **STAGE 8 CODE COMPLETE** ✅ – requires physical testing of waypoint accuracy

---

## Current Project State
*(Update with current architecture, known bugs, working features)*


**Status**: ✅ Stage 8 CODE COMPLETE – waypoint circuit + boundary safety + return to start

**Working Features**:
- ✅ Arduino IDE with correct board configured
- ✅ Serial communication functional
- ✅ Display output working
- ✅ Physical whiskers attached
- ✅ **Motors.h class fully implemented & calibrated**:
  - Correct pin assignments for 3Pi+ hardware
  - Safe initialization (motors OFF at startup)
  - Signed PWM control (-255 to +255)
  - Automatic direction handling (FWD=LOW, REV=HIGH)
  - Power limiting (MAX_PWM = 180)
  - **Dead-band measured**: L: ±14-15 PWM, R: ±12-13 PWM
  - **Motor bias documented**: ~13° left drift at PWM=50
- ✅ **Non-blocking test framework** in main sketch
- ✅ **All motor primitives tested**: forward, backward, turn left, turn right
- ✅ **Speed measurement system (Stage 3)**:
  - Non-blocking speed calculation (20ms intervals)
  - Measures both wheels independently
  - Speed in counts/ms (convertible to mm/s)
  - Global variables: speed_e0, speed_e1, count_e0, count_e1
  - Helper library with conversion and utility functions
- ✅ **PID speed controllers (Stage 4) - FULLY TUNED**:
  - Two independent PID controllers (left_pid, right_pid)
  - Closed-loop speed control (command speed, not PWM)
  - Enable/disable flag for testing
  - Helper functions: setDemandSpeeds(), stopWithPID()
  - **Tuned gains: Kp=50.0, Ki=1.0, Kd=0.0**
  - Both wheels track demand speed within ±10%
  - Motor bias automatically compensated
  - **IMPORTANT**: Encoder mapping corrected (e0=RIGHT, e1=LEFT)
- ✅ Code compiles and runs without errors
- ✅ **Line sensor system (Stage 5) - TESTED & WORKING**:
  - LineSensors.h fully implemented
  - ADC-based sensor reading with INPUT_PULLUP
  - Calibration routine (rotate over black/white)
  - Normalized readings [0.0 : 1.0] after calibration
  - isOnLine() function for boundary detection
  - Pattern detection for differential response
  - **Calibrated and verified on robot** ✅
- ✅ **Kinematics & Navigation (Stage 7) - CALIBRATED & WORKING**:
  - Kinematics.h implements dead-reckoning odometry
  - updatePose() called every 20ms for position tracking
  - Navigation helper functions (angleDiff, angleToPoint, distanceToPoint)
  - Turn behavior: setTurn(), checkTurn() - proportional control
  - Travel behavior: setTravel(), checkTravel() - with heading correction
  - Combined navigation: travelToXY(), updateTravelToXY() - state machine
  - **Calibrated**: wheel_radius and wheel_sep adjusted by user
- ✅ **Stage 6 FSM & Timer - FULLY COMPLETE**:
  - Button A start (blocking wait in setup after hw init)
  - 4-minute millis() timer with Serial countdown
  - On expiry: STATE_RETURN_HOME via travelToXY(0,0)
  - STATE_STOPPED on arrival
  - LINE_THRESHOLD = 0.7 (tuned to reduce false positives)
- ✅ **Stage 8 Waypoint Navigation - CODE COMPLETE**:
  - 6 puck waypoints extracted from Map.svg and verified
  - FSM: TRAVEL_TO_WAYPOINT → AT_WAYPOINT → (next wp) → repeat
  - Boundary safety: line detection aborts travel, reverse+turn, then resumes
  - After full circuit (6 waypoints) wraps back to WP1
  - 1 s pause at each waypoint (placeholder for Stage 9 puck detection)
  - Pose re-initialised AFTER calibration spin (odometry corruption fix)

**Known Bugs**:
- None currently - motor bias is now compensated by PID

**Current Architecture**:
- **Motors_c class**: Provides low-level motor control via PWM
  - `initialise()`: Sets up pins, safe defaults
  - `setPWM(left, right)`: Commands motor power with sign-based direction
- **Encoder system** (Encoders.h): Interrupt-driven encoder counting
  - Global variables: count_e0, count_e1 (auto-updated)
  - setupEncoder0/1() called in setup()
- **Speed measurement** (3Pi_CodeStub.ino):
  - `measureSpeeds()`: Non-blocking, runs every 20ms
  - Calculates speed_e0, speed_e1 from encoder deltas
  - Fully integrated into main loop
- **SpeedMeasurement.h**: Helper library
  - Unit conversions (counts ↔ mm, counts/ms ↔ mm/s)
  - Utility functions (reset encoders, get distance)
  - Debug functions for testing
- **PID Controllers** (PID.h):
  - Two instances: left_pid, right_pid
  - update(demand, measurement) returns PWM feedback
  - Gains: Kp, Ki, Kd (tunable)
  - Automatic integral reset when disabled
- **Main loop**: Non-blocking multi-task architecture
  - No delay() calls
  - Motor control (Stage 2)
  - Speed measurement (Stage 3 - 20ms intervals)
  - PID control (Stage 4 - 50ms intervals)
  - All systems integrated and running concurrently

**Completed Tasks**:
- ✅ Stage 1: Environment setup & hardware validation
- ✅ Stage 2: Motor control implementation & physical testing
  - Motors.h fully implemented with calibration data
  - Dead-band, direction, bias all measured and documented
- ✅ Stage 3: Encoder reading & speed measurement
  - Non-blocking speed measurement system
  - Helper library for unit conversions
  - Testing guide created
- ✅ Stage 4: PID speed controller implementation - FULLY COMPLETE
  - Two PID instances integrated
  - Helper functions for speed commands
  - Encoder mapping bug fixed (e0=RIGHT, e1=LEFT)
  - PID gains tuned: Kp=50.0, Ki=1.0, Kd=0.0
  - Both wheels track demand speed within ±10%
- ✅ Stage 5: Line sensor implementation - TESTED & WORKING
  - LineSensors.h fully implemented
  - Calibration routine in main sketch
  - isOnLine() and helper functions
  - Calibrated and tested on robot ✅
- ✅ Stage 7: Kinematics & Navigation - CALIBRATED & COMPLETE
  - Navigation functions implemented (turn, travel, travelToXY)
  - updatePose() integrated into main loop
  - **Calibrated by user** (wheel_radius, wheel_sep)
- ✅ Stage 6: Wandering Behavior / Timer - COMPLETE
  - Button A start control implemented
  - 4-min timer with return-to-start
  - **BEGINNER 50% ACHIEVED**: Robot searches for 4 min without leaving map
- ✅ Stage 8: Waypoint Navigation - CODE COMPLETE
  - 6 puck waypoints from Map.svg; cycled in order
  - Boundary-safe travel with reverse+turn recovery
  - Odometry corruption bug fixed (pose re-init after calibration)

---

## Next Steps
*(Update after each development session)*

1. ✅ Complete Stage 1: Environment Setup & Hardware Validation
2. ✅ Complete Stage 2: Motor Control Foundation
3. ✅ Complete Stage 3: Encoder Reading & Speed Measurement
4. ✅ Complete Stage 4: PID Speed Controllers - FULLY TUNED
5. ✅ Stage 5: Line Sensor - TESTED & WORKING
6. ✅ Stage 7: Kinematics & Navigation - CALIBRATED & COMPLETE
7. ✅ Stage 6: FSM & Timer - FULLY COMPLETE
   - 5-state FSM, 4-min timer, return-to-start via odometry
   - **BEGINNER 50-58% TARGET MET**
8. ✅ Stage 8: Waypoint Navigation System - CODE COMPLETE
   - 6 waypoints extracted and verified from Map.svg
   - Waypoint circuit with boundary-safe travel implemented
   - **Requires physical test**: confirm odometry accuracy to waypoints
9. **CURRENT**: Stage 9: Magnetometer & Puck Detection
   - Magnetometer.h needs implementing
   - Physical calibration of baseline / puck threshold required
10. **THEN**: Stage 10: Intermediate Solution (push puck out)
11. **THEN**: Stages 11-13: Expert Solution (return puck to start)