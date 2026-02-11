# Stage 4: PID Speed Controller Tuning Guide

## Overview

PID (Proportional-Integral-Derivative) controllers automatically adjust motor PWM to maintain constant wheel speeds. This guide will walk you through tuning the PID gains step-by-step.

**Why PID is needed:**
- Motor bias (from Stage 2): Robot drifts ~13° left at PWM=50
- Variable loads: Speed changes on inclines, different surfaces
- Dead-band: Motors don't respond below certain PWM values
- Motor differences: Left and right motors respond differently

**What PID will fix:**
✅ Maintains constant speed regardless of load
✅ Eliminates motor bias (robot drives straight)
✅ Overcomes dead-band automatically
✅ Smooth acceleration and deceleration

---

## What Was Implemented

### PID Controllers
```c
PID_c left_pid;   // Controls left motor
PID_c right_pid;  // Controls right motor
```

### Control Variables
```c
float demand_speed_left;   // Target speed for left wheel (counts/ms)
float demand_speed_right;  // Target speed for right wheel (counts/ms)
bool use_pid_control;      // Enable/disable PID
```

### Helper Functions
```c
setDemandSpeeds(left, right);  // Set target speeds
setDemandSpeed(speed);         // Set same speed for both wheels
stopWithPID();                 // Stop smoothly
```

### How It Works
```
Every 50ms:
1. Measure current wheel speeds (speed_e0, speed_e1)
2. Calculate error: error = demand - measured
3. PID calculates PWM: pwm = Kp×error + Ki×∫error + Kd×(Δerror/Δt)
4. Apply PWM to motors
5. Repeat
```

---

## PID Tuning Process (HUMAN TASK)

### Prerequisites
- ⚠️ Battery power ON (blue LED)
- ⚠️ Robot lifted OFF surface (or upside-down) for initial tests
- ⚠️ Serial Monitor or Serial Plotter open (9600 baud)
- ⚠️ Time required: 30-90 minutes

---

## Step 1: Enable PID Control (5 min)

### Modify Your Code

**In your loop() function, BEFORE the motor test switch:**

```c
void loop() {

  // ============================================
  // ENABLE PID FOR TESTING
  // ============================================
  use_pid_control = true;  // Enable PID control

  // Set a test demand speed
  setDemandSpeed(0.3);  // Target: 0.3 counts/ms for both wheels

  // Continue with rest of loop...
  measureSpeeds();
  updatePID();

  // Comment out or disable motor test states
  // (PID will control motors instead)
}
```

**Enable Debug Output:**

In `updatePID()` function, uncomment the Serial.print lines:
```c
Serial.print("DemL:");
Serial.print(demand_speed_left, 4);
Serial.print(",MeasL:");
Serial.print(speed_e0, 4);
// ... etc
```

**Upload and verify:**
- Serial output shows demand and measured speeds
- Motors should start attempting to reach 0.3 counts/ms

---

## Step 2: Tune Kp (P-Controller) (15-20 min)

**Goal:** Find Kp that makes robot respond quickly without oscillating

**Current values:** Kp=1.0, Ki=0.0, Kd=0.0

### Testing Procedure

**2a. Test Initial Kp=1.0**
1. Upload code with Kp=1.0
2. Open Serial Plotter (Tools > Serial Plotter)
3. Observe measured speed (MeasL, MeasR) vs demand (DemL, DemR)

**Expected behaviors:**

**If Kp is TOO SMALL:**
- Speed rises very slowly
- Never reaches demand
- Large steady-state error (gap between demand and measured)
- **Action:** INCREASE Kp (try 2.0, then 3.0, etc.)

**If Kp is TOO LARGE:**
- Speed overshoots demand
- Oscillates (bounces up and down)
- Unstable behavior
- **Action:** DECREASE Kp (try 0.5, then 0.3, etc.)

**If Kp is GOOD:**
- Speed rises quickly (within 1-2 seconds)
- Reaches close to demand (within ~10%)
- Minimal oscillation
- Stable behavior
- Some steady-state error is OK (Ki will fix this later)

### Finding the Right Kp

**Methodology:**
1. Start at Kp=1.0
2. If too small: DOUBLE it (1.0 → 2.0 → 4.0 → 8.0)
3. If too large: HALVE it (1.0 → 0.5 → 0.25 → 0.125)
4. Once you bracket the right value (one too small, one too large):
   - Use intermediate values (e.g., if 2.0 is too small, 4.0 is too large, try 3.0)
5. Stop when you find a Kp that:
   - Responds quickly (< 2 seconds)
   - Minimal overshoot (< 20%)
   - Stable (doesn't oscillate continuously)

**Typical values:** Kp between 0.5 and 5.0 (varies per robot)

**Record your value:**
```
Best Kp for left motor:  _______
Best Kp for right motor: _______
```

---

## Step 3: Add Ki (PI-Controller) (15-20 min)

**Goal:** Eliminate steady-state error (make measured = demand exactly)

**Current values:** Kp=[your value from Step 2], Ki=0.0, Kd=0.0

### Why Ki is Needed

With P-only control, you probably observe:
- Speed gets close to demand, but not exact
- Always a small gap (steady-state error)
- Error is ~0.05 to 0.1 counts/ms

Ki **integrates** error over time → builds up → fixes this gap!

### Testing Procedure

**3a. Add Small Ki**
1. In setup(), modify PID initialization:
   ```c
   float initial_Kp = YOUR_VALUE_FROM_STEP_2;
   float initial_Ki = 0.01;  // Start very small
   float initial_Kd = 0.0;
   ```
2. Upload and observe

**Expected behaviors:**

**If Ki is TOO SMALL:**
- Steady-state error still exists (but smaller)
- Slow to eliminate error (takes >10 seconds)
- **Action:** INCREASE Ki (try 0.02, 0.05, 0.1)

**If Ki is TOO LARGE:**
- Speed overshoots demand
- Oscillates wildly
- System becomes unstable
- "Integral windup" - keeps increasing even after overshoot
- **Action:** DECREASE Ki (try 0.005, 0.001)

**If Ki is GOOD:**
- Steady-state error eliminated (measured = demand)
- Smooth approach (no overshoot)
- Stable at demand speed
- Response time reasonable (2-5 seconds total)

### Finding the Right Ki

**Methodology:**
1. Start at Ki=0.01
2. If steady-state error remains: DOUBLE Ki (0.01 → 0.02 → 0.04)
3. If oscillates: HALVE Ki (0.01 → 0.005 → 0.0025)
4. Stop when measured speed matches demand within ±0.01 counts/ms

**Typical values:** Ki between 0.001 and 0.1 (much smaller than Kp)

**Record your value:**
```
Best Ki for left motor:  _______
Best Ki for right motor: _______
```

---

## Step 4: Test on Surface (10-15 min)

**⚠️ IMPORTANT:** All previous tuning was OFF-SURFACE (unloaded motors)

**Now test ON-SURFACE (loaded motors):**

1. Place robot on flat surface
2. Set demand speed: `setDemandSpeed(0.3);`
3. Observe:
   - Does robot move at consistent speed?
   - Does robot drive straight?
   - Any oscillation?

**Common issues:**

**Robot still drifts (not straight):**
- Tune left and right PIDs separately
- Different Kp/Ki for each motor
- Left and right motors have different characteristics

**Speed oscillates:**
- Reduce Ki slightly (surface friction causes more integral buildup)
- May need to reduce Kp slightly

**Robot doesn't move:**
- Demand speed too low (below dead-band)
- Try 0.5 or 0.8 counts/ms

**Robot drives straight!**
- ✅ PID is working correctly!
- ✅ Motor bias from Stage 2 is now compensated

---

## Step 5: Kd Tuning (OPTIONAL) (10-15 min)

**Goal:** Reduce overshoot and improve response time

**When to use Kd:**
- If you have significant overshoot (>20%)
- If response is sluggish
- If you want faster settling time

**When NOT to use Kd:**
- If system is already stable and fast
- If you observe noise amplification (jittery behavior)
- Kd is often not needed for wheel speed control

### Testing Procedure

**5a. Add Small Kd**
1. In setup(), modify PID initialization:
   ```c
   float initial_Kp = YOUR_VALUE;
   float initial_Ki = YOUR_VALUE;
   float initial_Kd = 0.001;  // Start very small
   ```
2. Upload and observe

**Expected behaviors:**

**If Kd helps:**
- Reduced overshoot
- Faster settling time
- Smoother response

**If Kd causes problems:**
- Jittery, noisy behavior
- Amplifies measurement noise
- System becomes unstable
- **Action:** Remove Kd (set to 0.0)

**Typical values:** Kd between 0.0 and 0.01 (often 0.0 is best)

---

## Step 6: Test Different Speeds (10 min)

**Goal:** Verify PID works across range of speeds

**Test at:**
- Low speed: 0.2 counts/ms
- Medium speed: 0.5 counts/ms
- High speed: 1.0 counts/ms

**For each speed:**
1. Set demand: `setDemandSpeed(speed);`
2. Observe response time and stability
3. Check if robot drives straight

**If PID doesn't work well at different speeds:**
- May need "gain scheduling" (different gains at different speeds)
- Advanced topic - stick with one set of gains for now
- Optimize for your most-used speed range

---

## Step 7: Final Testing (10 min)

**Test complete robot behaviors:**

**Test 1: Start/Stop**
```c
setDemandSpeed(0.5);  // Start moving
delay(3000);
stopWithPID();        // Stop smoothly
```

**Test 2: Speed Changes**
```c
setDemandSpeed(0.3);
delay(2000);
setDemandSpeed(0.8);  // Speed up
delay(2000);
setDemandSpeed(0.3);  // Slow down
```

**Test 3: Turning**
```c
setDemandSpeeds(0.5, 0.3);  // Turn right (left faster)
delay(2000);
setDemandSpeeds(0.3, 0.5);  // Turn left
```

**Test 4: Straight Line Travel**
```c
setDemandSpeed(0.5);
// Let robot drive 1 meter
// Measure deviation from straight line
```

**✅ Success criteria:**
- Robot responds smoothly to speed commands
- Robot drives straight when equal demands
- Robot stops smoothly
- No oscillation or instability

---

## Tuning Summary Table

| Gain | Effect | Typical Range | Tuning Method |
|------|--------|---------------|---------------|
| **Kp** | Response speed | 0.5 - 5.0 | Double/halve until fast without oscillation |
| **Ki** | Eliminates steady-state error | 0.001 - 0.1 | Add small amount, increase until error = 0 |
| **Kd** | Reduces overshoot | 0.0 - 0.01 | Often not needed, try last |

---

## Common Problems & Solutions

### Problem: Speed Never Reaches Demand

**Possible causes:**
- Kp too small
- Demand speed impossible (too high or below dead-band)
- Motors not receiving power (battery OFF?)

**Solutions:**
- Increase Kp
- Try different demand speed
- Check battery power (blue LED ON?)

---

### Problem: Oscillation (Speed Bounces Up/Down)

**Possible causes:**
- Kp too large
- Ki too large
- Kd causing noise amplification

**Solutions:**
- Reduce Kp by 30%
- Reduce Ki by 50%
- Set Kd = 0.0

---

### Problem: Slow Response (Takes >5 seconds)

**Possible causes:**
- Kp too small
- Ki too small
- PID update interval too slow

**Solutions:**
- Increase Kp
- Slightly increase Ki
- Check PID_UPDATE_INTERVAL = 50ms (don't go below 20ms)

---

### Problem: Robot Still Doesn't Drive Straight

**Possible causes:**
- Left and right motors need different gains
- Calibration issue
- Mechanical issue (bent wheel, damaged gearbox)

**Solutions:**
- Tune left_pid and right_pid separately with different gains:
  ```c
  left_pid.initialise(Kp_left, Ki_left, Kd_left);
  right_pid.initialise(Kp_right, Ki_right, Kd_right);
  ```
- Check encoder counts are correct
- Inspect robot for physical damage

---

### Problem: "Integral Windup"

**Symptoms:**
- Speed overshoots demand significantly
- Takes long time to settle
- Oscillates for a long time after demand change

**Cause:** Integral term accumulates too much error

**Solutions:**
- Reduce Ki
- Add integral clamping (advanced - see PID.h)
- Call `left_pid.reset()` when changing demands significantly

---

## Recording Your Final Values

**After tuning, record your optimal gains:**

```c
// FINAL TUNED VALUES - [DATE]
// Tuned on [SURFACE TYPE] at room temperature

// Left motor PID gains
float Kp_left = ______;
float Ki_left = ______;
float Kd_left = ______;

// Right motor PID gains
float Kp_right = ______;
float Ki_right = ______;
float Kd_right = ______;

// Update these in setup():
left_pid.initialise(Kp_left, Ki_left, Kd_left);
right_pid.initialise(Kp_right, Ki_right, Kd_right);
```

**Add these to your code permanently!**

---

## Using PID in Your Robot

### Example: Drive Forward for 1 Meter

```c
#include "SpeedMeasurement.h"  // For distance functions

void driveForwardOneMeter() {
    // Reset distance tracking
    resetEncoderCounts();

    // Enable PID
    use_pid_control = true;

    // Set target speed
    setDemandSpeed(0.5);  // 0.5 counts/ms

    // Drive until 1000mm traveled
    while (getAverageDistanceMM() < 1000.0) {
        measureSpeeds();
        updatePID();
        // Loop continues
    }

    // Stop smoothly
    stopWithPID();

    // Wait for stop
    delay(500);
}
```

### Example: Turn 90 Degrees

```c
void turnRight90Degrees() {
    use_pid_control = true;

    // Turn by driving one wheel faster
    setDemandSpeeds(0.5, -0.5);  // Left forward, right backward

    // Calculate rotation based on encoder difference
    // This is approximate - better to use pose.theta from kinematics
    resetEncoderCounts();

    // ~358 counts per wheel rev, ~85mm wheel separation
    // 90° turn ≈ (π/2) × 85mm / 2 = ~67mm per wheel
    // ~67mm ≈ 240 encoder counts

    while (abs(count_e0) < 240) {
        measureSpeeds();
        updatePID();
    }

    stopWithPID();
}
```

---

## Stage 4 Complete Checklist

Before proceeding to Stage 5:

- [ ] PID code compiles without errors
- [ ] Kp tuned (robot responds quickly, minimal oscillation)
- [ ] Ki tuned (steady-state error eliminated)
- [ ] Kd tuned (optional - if needed)
- [ ] Tested on surface (robot drives straight)
- [ ] Tested at multiple speeds (0.2, 0.5, 1.0 counts/ms)
- [ ] Final gains recorded in code
- [ ] Motor bias from Stage 2 is now compensated
- [ ] Robot can drive straight reliably

---

## What's Next: Stage 5 Preview

**Stage 5: Line Sensor Calibration & Boundary Detection**

With PID working, you can now:
- Drive at precise speeds
- Navigate in straight lines
- Make controlled turns

Next, you'll add:
- Line sensor reading
- Boundary detection
- Line-following or boundary-avoidance behaviors

This builds toward Assessment 1 requirements!

---

## Quick Reference: PID Tuning Cheat Sheet

```
1. START: Kp=1.0, Ki=0.0, Kd=0.0
2. TUNE Kp:
   - Too slow? → INCREASE Kp
   - Oscillates? → DECREASE Kp
   - Goal: Fast response, stable
3. ADD Ki:
   - Start Ki=0.01
   - Steady-state error remains? → INCREASE Ki
   - Oscillates? → DECREASE Ki
   - Goal: Measured = Demand
4. (Optional) ADD Kd:
   - Start Kd=0.001
   - Overshoot? → INCREASE Kd
   - Jittery? → DECREASE or REMOVE Kd
5. TEST on surface, test different speeds
6. RECORD final values in code
```

Good luck with tuning! 🎯
