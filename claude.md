# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

University of Bristol SEMTM0042/43 Robotics Science & Systems coursework. Program a **Pololu 3Pi+ mobile robot** to autonomously complete a "Foraging Challenge" - search a map for 4 minutes, detect magnetic pucks, and push them out of (Intermediate) or back to (Expert) a start area.

**Assessment 1 Deadline**: Thursday 19th February 1pm (30% of unit grade)

## Build Commands

```bash
# Compile Arduino sketch (requires arduino-cli installed)
arduino-cli compile --fqbn pololu-a-star:avr:a-star32U4 3Pi_CodeStub/3Pi_CodeStub/3Pi_CodeStub.ino

# Upload to robot (replace /dev/tty.usbmodem* with actual port)
arduino-cli upload -p /dev/tty.usbmodem* --fqbn pololu-a-star:avr:a-star32U4 3Pi_CodeStub/3Pi_CodeStub/3Pi_CodeStub.ino
```

Board: "Pololu 3Pi+ 32U4" (Arduino-compatible ATmega32U4)

## Code Architecture

### Main Code Location
`3Pi_CodeStub/3Pi_CodeStub/` - All robot code lives here

### Key Files and Their Purposes

| File | Purpose | Status |
|------|---------|--------|
| `3Pi_CodeStub.ino` | Main sketch with `setup()` and `loop()` | Active development |
| `Motors.h` | PWM motor control (pins L:10/16, R:9/15) | ✅ Complete & calibrated |
| `Encoders.h` | Interrupt-driven wheel counting | ✅ Complete |
| `PID.h` | Generic PID controller class | ✅ Complete (Kp=50, Ki=1, Kd=0) |
| `LineSensors.h` | 5x IR boundary detection | ✅ Complete & calibrated |
| `Kinematics.h` | Dead-reckoning odometry | Code complete, needs calibration |
| `SpeedMeasurement.h` | Encoder unit conversions | ✅ Complete |
| `Magnetometer.h` | Puck detection | Not yet implemented |

### Critical Architecture Patterns

**Non-blocking event loop**: All code must use `millis()` timing, never `delay()`. The main loop runs at ~1-2ms intervals.

```c
// Correct pattern
unsigned long ts = millis();
if (millis() - ts >= INTERVAL) {
    // Execute task
    ts = millis();
}
```

**Naming conventions**: Classes use `_c` suffix (e.g., `Motors_c`, `Kinematics_c`)

**Encoder mapping**: `count_e0` = RIGHT wheel, `count_e1` = LEFT wheel (this was a source of bugs)

### Calibrated Hardware Values

```c
// Motor dead-band (minimum PWM for movement)
// Left: +15/-14, Right: +13/-12

// PID speed controllers
// Kp=50.0, Ki=1.0, Kd=0.0

// Kinematics (require physical calibration)
// wheel_radius: ~16.4mm
// wheel_sep: ~41.23mm (from center to wheel)
// encoder counts per revolution: 358.3
```

## Development Progress

**Completed**: Stages 1-5, 7 (code only)
- Motor control, encoders, PID speed control, line sensors, kinematics framework

**Current blocker**: Kinematics calibration (human task - adjust wheel_radius and wheel_sep based on physical measurements)

**Remaining**: Stages 6, 8-14
- Timer & FSM, waypoint navigation, magnetometer, puck manipulation, expert alignment maneuver

## Project Structure

```
3Pi_CodeStub/3Pi_CodeStub/  - Arduino code (main development folder)
docs/                        - Development guides and logs
  ├── Executive_Summary.md   - Full technical briefing
  ├── Development_plan.md    - 14-stage implementation plan
  ├── Project_log.md         - Session history and current state
  └── Stage*_*.md            - Stage-specific guides
Map/                         - Coursework map PDFs
Images/                      - Reference diagrams
```

## Restrictions

**Forbidden**:
- External libraries (e.g., Pololu3piPlus32U4.h) - must use provided stubs
- Adding extra electronics
- Public code repositories
- Generative AI for Arduino code (only allowed for Python plotting)

**Required**:
- Non-blocking architecture throughout
- Code must compile without errors
- Robot must not leave map boundary (except for Expert difficulty positioning)

## Human-Required Tasks

Several calibration tasks require physical robot access:
1. **Kinematics calibration** - adjust wheel_radius and wheel_sep by measuring actual vs. reported distances
2. **Magnetometer threshold** - measure baseline and puck-present readings
3. **PID retuning** - if behavior changes after hardware adjustments
4. **Line sensor threshold** - adjust LINE_THRESHOLD if false positives occur

See `docs/Stage7_Calibration_Guide.md` for detailed procedures.

## Marking Tiers

- **Beginner (50-58%)**: Search 4 min, stay in bounds, return to start
- **Intermediate (60-68%)**: Push 1-5 pucks OUT of map (+2% each)
- **Expert (70-100%)**: Return 1-7 pucks TO start area (+5% each)


## Notes on behaviour
 - You should be mindful of the lack of physical feedback you have compared to a human developer when developing code for the robot. Therefore you should write code in a clear, human understandable manor, ensuring all code has concise, clear comments describing their function. You should also understand which tasks (such as tuning) are more suited for human implementation. When this is the case, you should ensure you provide detailed and insightful methodology for the human to follow.
 - You should follow the methodology detailed within the lab scripts contained within the Labsheets directory.
 - You should keep an up to date log of the project within the file docs/Project_log.md containing:
    1. a history of All previous development, bugs, solutions and anything else future claude sessions would benefit knowing.
    2. current state of the project, detailing the current architecture, bugs and anything else you believe new claude sessions would benefit from knowing.
 -  You should keep an up to date step by step plan for future development to be able to meet the goals listed within Assessment 1 of the SEMTM0042_Specification.pdf file located within the docs/Development_plan.md file.