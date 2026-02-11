# SEMTM0042: Robotics Science & Systems - Comprehensive Project Analysis

## Executive Summary for LLM Assistants

This document provides a complete technical briefing for AI assistants working on this robotics coursework project. The project involves programming a **Pololu 3Pi+ mobile robot** to complete autonomous foraging tasks within a 4-minute time window.

---

## Assessment Structure Overview

### Assessment 1 (30% of unit grade) - Individual Work
- **Deadline**: Thursday 19th February 1pm (UoB Week 17)
- **Task**: Program robot to autonomously complete the "Foraging Challenge"
- **Submissions Required**:
  1. Online self-assessment form with URL to video demonstration
  2. Working source code uploaded to Blackboard (must compile without errors)
- **Plagiarism**: Code will be algorithmically checked against all other submissions

### Assessment 2 (70% of unit grade) - Team Work
- **Deadline**: Thursday 23rd April 1pm (UoB Week 23)
- **Task**: 6-page scientific report on experiment conducted with 3Pi+ robot
- Not the focus of this project currently

---

## Hardware Platform: Pololu 3Pi+ Mobile Robot

### Key Components
| Component | Function |
|-----------|----------|
| ATmega32U4 Microcontroller | Arduino-compatible, USB programmable |
| Dual Micro Metal Gearmotors | 8V operation, differential drive |
| Dual Quadrature Encoders | 12 counts per revolution of motor shaft |
| IR Line Sensors (5x) | Detect black/white surfaces (DN1-DN5 pins) |
| Three-axis Gyro/Accelerometer/Compass | LSM6DS33 IMU + LIS3MDL magnetometer |
| OLED/LCD Display | Status output |
| Bump Sensors | Front collision detection |
| User LEDs/Buzzer/Buttons | Feedback and control |

### Critical Dimensions (for Kinematics)
- **Wheel Radius**: ~16mm (requires calibration)
- **Wheel Separation**: ~85mm (requires calibration)
- **Encoder Resolution**: 12 CPR (motor) × gear ratio = counts per wheel revolution

### Physical Modifications Required
- Cable ties and plastic hooks must be attached to bumpers to create "whiskers" for puck manipulation
- Clear plastic puck with embedded magnet is detected via magnetometer

---

## Assessment 1: Foraging Challenge - Detailed Requirements

### Coursework Map
- Size: A2 paper (or two A3 sheets taped together)
- Features:
  - **Thick black boundary line** - outer map edge
  - **Start area** - marked by dashed line in corner (with compass rose graphic)
  - **6 puck locations** - numbered circles where puck can be placed
  - **10mm grid** - for calibration purposes

### Marking Tiers

#### Fail (0-49%)
- Partial functionality only (e.g., motors work OR sensors work, but not integrated)
- Code doesn't compile or doesn't match video demonstration

#### Beginner Difficulty (50-58%)
| Mark | Requirement |
|------|-------------|
| 50% | Robot actively searches for 4 minutes, stays within boundary, interacts with black line, stops after 4 minutes |
| 58% | As above PLUS returns to start area after 4 minutes (robot body partially within start area) |

**Key Skills**: Motor control, line sensor reading, time management, basic boundary avoidance

#### Intermediate Difficulty (60-68%)
| Mark | Requirement |
|------|-------------|
| 58 + (n×2) | n = number of pucks pushed OUT of map (min 1, max 5 = 68%) |

**Process**:
1. Dice roll determines puck location (1-6)
2. Robot searches and visits puck locations
3. Detects puck using magnetometer
4. Pushes puck OUTSIDE map boundary
5. Returns to start area, waits 4 seconds for new puck placement
6. Repeat until 4 minutes elapsed
7. Robot must NOT leave map area during operation

**Key Skills**: Waypoint navigation, magnetometer sensing, state machine, puck manipulation

#### Expert Difficulty (70-100%)
| Mark | Requirement |
|------|-------------|
| 65 + (n×5) | n = number of pucks returned TO START (min 1, max 7 = 100%) |

**Process**:
1. Same detection as Intermediate
2. Robot must maneuver AROUND puck to position itself
3. Push puck INTO start area (some part of puck within dashed line)
4. Wait 4 seconds for new puck placement
5. Continue searching (cannot idle after first puck)
6. May leave map boundary ONLY to position for return push

**Key Skills**: Path planning, alignment maneuvers, precise motion control, error recovery

### Video Requirements
1. Student ID card visible at start
2. Single unedited video
3. Robot cannot be touched/reset after power-on
4. Display must show countdown timer (code provided)
5. Dice roll must be visible for puck placement
6. 4-minute limit strictly enforced
7. Robot must come to complete stop at end

---

## Technical Implementation Guide (from Labsheets)

### Labsheet 0: Getting Started
- Arduino IDE setup
- Board selection: "Pololu 3Pi+ 32U4"
- Serial communication for debugging
- **Non-blocking code patterns** (critical for this project)
  ```c
  // Use millis() instead of delay()
  unsigned long ts = millis();
  if (millis() - ts >= INTERVAL) {
      // Execute periodic task
      ts = millis();
  }
  ```

### Labsheet 1: Motors
- **Motor Class** (`Motors.h`): Direct PWM control
  ```c
  Motors_c motors;
  motors.initialise();
  motors.setPWM(left_pwm, right_pwm);  // -255 to +255
  ```
- **Encoder Class** (`Encoders.h`): Interrupt-driven counting
  ```c
  setupEncoder0();  // Left wheel
  setupEncoder1();  // Right wheel
  count_e0, count_e1  // Global variables
  ```
- **PID Speed Control** (`PID.h`): Closed-loop velocity control
  - Requires tuning Kp, Ki, Kd gains
  - Essential for precise motion control

### Labsheet 2: Sensors
- **Line Sensors** (`LineSensors.h`):
  - 5 IR sensors (DN1-DN5)
  - Require calibration (store min/max for black/white)
  - Used for boundary detection
  ```c
  LineSensor_c line_sensors;
  line_sensors.calibrate();  // Run-once at startup
  line_sensors.getReadings();
  ```

- **Magnetometer** (`Magnetometer.h`):
  - Detects puck magnet
  - Requires threshold calibration
  - Sensitive to environmental metal objects
  ```c
  // Check if magnitude exceeds threshold
  if (mag_reading > PUCK_THRESHOLD) {
      // Puck detected
  }
  ```

### Labsheet 3: Motion Control (Kinematics)
- **Coordinate Frames**:
  - Global frame (XI, YI): Fixed to where robot starts
  - Local frame (XR, YR): Fixed to robot body

- **Kinematics Class** (`Kinematics.h`):
  ```c
  Kinematics_c pose;
  pose.initialise(0, 0, 0);  // x, y, theta
  pose.update();  // Call regularly (~20ms intervals)
  // Access: pose.x, pose.y, pose.theta
  ```

- **Calibration Required**:
  1. Translation calibration: Adjust `wheel_radius` parameter
  2. Rotation calibration: Adjust `wheel_sep` parameter

- **Critical Functions to Implement**:
  - `setTurn(target_angle)` / `checkTurn()` - Rotate to specific angle
  - `setTravel(x, y)` / `checkTravel()` - Move to coordinates
  - `travelToXY()` - Combined rotate-then-travel behavior
  - Angular difference calculation (handle 0°/360° wraparound using atan2)

### Labsheet 4: System Integration
- **Finite State Machine (FSM)** architecture recommended
  ```c
  #define STATE_SEARCHING   0
  #define STATE_AVOID_LINE  1
  #define STATE_FOUND_PUCK  2
  #define STATE_RETURN_HOME 3
  int state;
  ```

- **Waypoint System**:
  ```c
  #define NUM_WAYPOINTS 6
  float x[NUM_WAYPOINTS] = {...};
  float y[NUM_WAYPOINTS] = {...};
  int waypoint_index;
  ```

- **Expert Difficulty: Alignment Maneuver**
  - Calculate "push line" from puck to start area
  - Navigate around puck without displacing it
  - Position robot on opposite side of push line
  - Maintain whisker envelope distance

---

## Provided Code Stub Files

| File | Purpose |
|------|---------|
| `3Pi_CodeStub.ino` | Main sketch template |
| `Encoders.h` | Interrupt-driven encoder counting |
| `Kinematics.h` | Odometry/dead-reckoning calculations |
| `LineSensors.h` | IR line sensor interface |
| `Magnetometer.h` | Puck detection interface |
| `Motors.h` | PWM motor control |
| `PID.h` | Generic PID controller class |
| `lcd.h` / `oled.h` | Display interfaces |

---

## Restrictions and Rules

### Permitted
- Arduino C programming
- Standard C libraries (e.g., math.h)
- Libraries specified in labsheets
- Generative AI for Python plotting code ONLY

### Forbidden
- External software libraries (e.g., Pololu3piPlus32U4.h)
- Adding extra electronics/sensors
- Video editing or speed manipulation
- Public code repositories (GitHub etc.)
- Any form of code sharing

---

## Common Sources of Error

1. **Kinematics drift**: Accumulates over 4 minutes (12,000 updates at 20ms)
2. **Wheel slip**: Sudden movements cause encoder miscounts
3. **Sensor calibration**: Line sensors and magnetometer need proper thresholds
4. **Blocking code**: Using `delay()` prevents kinematics updates
5. **Angular wraparound**: 0°/360° boundary causes navigation bugs
6. **Metal interference**: Environmental metal affects magnetometer
7. **PID tuning**: Poor gains cause oscillation or sluggish response

---

## Development Strategy Recommendations

1. **Incremental development**: Build and test each subsystem separately
2. **Non-blocking architecture**: All behaviors must yield control regularly
3. **Controlled experiments**: Initialize robot at known positions for testing
4. **State machine debugging**: Add a DEBUG state to print sensor values
5. **Calibration first**: Tune kinematics parameters before complex behaviors
6. **Video practice**: Record test runs before final submission

---

## Key Formulas

### Distance to Target
```c
float dx = target_x - pose.x;
float dy = target_y - pose.y;
float distance = sqrt(dx*dx + dy*dy);
```

### Angle to Target
```c
float angle_to_target = atan2(dy, dx);
```

### Angular Difference (shortest path)
```c
float diff = target_angle - current_angle;
// Normalize to [-π, π]
while (diff > PI) diff -= 2*PI;
while (diff < -PI) diff += 2*PI;
```

---

Robotics Laboratory Coursework.

For this project you are tasked with planning and implementing Arduino code for a 3PI+ robot.

PROJECT FOLDER TOPOLOGY:

  - 3PI_CodeStub/3PI_CodeStub Directory: Contains code stubs to be completed
  - Images Directory: Contains various images deemed helpful to understand aspects of the project, however these are not central to the project.
  - Labsheets Directory **IMPORTANT** : Contains labsheets which contain instruction on how to complete the project. The most important labsheets are stored within the Core subdirectory.
  - Map Directory: Contains PDF files of the map used for the 3PI+ robot to navigate.
  - SEMTM0042_Specification.pdf File: Contains the specification the project will be marked on. This project is using the specification for Assessment 1.



---