# ✅ Stage 4: PID Speed Controllers - CODE COMPLETE

## What Was Implemented

### 1. PID Controller Instances

**File: 3Pi_CodeStub/3Pi_CodeStub/3Pi_CodeStub.ino**

```c
// Two independent PID controllers
PID_c left_pid;   // Controls left motor speed
PID_c right_pid;  // Controls right motor speed
```

**Initialized in setup():**
```c
// Starting gains (MUST be tuned physically)
left_pid.initialise(Kp=1.0, Ki=0.0, Kd=0.0);
right_pid.initialise(Kp=1.0, Ki=0.0, Kd=0.0);
```

---

### 2. PID Control System

**Control Variables:**
```c
float demand_speed_left;    // Target speed for left wheel (counts/ms)
float demand_speed_right;   // Target speed for right wheel (counts/ms)
bool use_pid_control;       // Enable/disable PID
float left_pwm_output;      // PWM calculated by left PID
float right_pwm_output;     // PWM calculated by right PID
```

**Update Timing:**
```c
#define PID_UPDATE_INTERVAL 50  // PID runs every 50ms
```

**Why 50ms?**
- Faster than 20ms speed measurement → fresh data
- Slow enough for PID to respond smoothly
- Prevents divide-by-zero errors in PID.update()
- Good balance for wheel speed control

---

### 3. Core PID Function

**updatePID() - Called from loop()**

```c
void updatePID() {
    // Runs every 50ms

    if (use_pid_control) {
        // CLOSED-LOOP CONTROL
        // Calculate PWM based on speed error
        left_pwm_output = left_pid.update(demand_speed_left, speed_e0);
        right_pwm_output = right_pid.update(demand_speed_right, speed_e1);

        // Apply to motors
        motors.setPWM(left_pwm_output, right_pwm_output);
    } else {
        // PID DISABLED
        // Reset PIDs to prevent integral windup
        left_pid.reset();
        right_pid.reset();
        // Direct motor control used instead
    }
}
```

**How PID Works:**
1. Measure current speed (from Stage 3)
2. Calculate error: `error = demand - measured`
3. PID calculates correction:
   - P-term: `Kp × error` (immediate response)
   - I-term: `Ki × ∫error` (eliminates steady-state error)
   - D-term: `Kd × Δerror/Δt` (reduces overshoot)
4. Output PWM = P + I + D
5. Apply PWM to motors
6. Repeat every 50ms

---

### 4. Helper Functions

**Set Target Speeds:**
```c
setDemandSpeeds(0.5, 0.5);    // Both wheels 0.5 counts/ms
setDemandSpeed(0.3);          // Both wheels same speed (straight line)
stopWithPID();                // Smooth stop (demand = 0)
```

**Example Usage:**
```c
void loop() {
    use_pid_control = true;      // Enable PID
    setDemandSpeed(0.5);         // Drive forward at 0.5 counts/ms

    measureSpeeds();             // Update speed measurements
    updatePID();                 // PID calculates and applies PWM
}
```

---

## Integration with Existing Systems

### System Flow (Every Loop Cycle)

```
1. measureSpeeds() [Every 20ms]
   ↓
   Updates: speed_e0, speed_e1

2. updatePID() [Every 50ms]
   ↓
   IF use_pid_control == true:
       Calculate: left_pwm = PID(demand, speed_e0)
       Calculate: right_pwm = PID(demand, speed_e1)
       Apply: motors.setPWM(left_pwm, right_pwm)
   ELSE:
       Reset PIDs
       Do nothing (direct control elsewhere)

3. Other systems continue
   ↓
   Loop repeats
```

**Non-blocking Architecture Maintained:**
- No delay() calls added
- All timing uses millis()
- Compatible with future additions (kinematics, line sensors, etc.)

---

## Files Created/Modified

### Modified:
- ✅ `3Pi_CodeStub/3Pi_CodeStub/3Pi_CodeStub.ino`
  - Added PID instances (left_pid, right_pid)
  - Added PID control variables
  - Added updatePID() function
  - Added helper functions (setDemandSpeeds, etc.)
  - Integrated into setup() and loop()

### Created:
- ✅ `Stage4_PID_TuningGuide.md` - Comprehensive 7-step tuning methodology
- ✅ `Stage4_QuickStart.md` - 5-minute quick test guide
- ✅ `Stage4_COMPLETE_Summary.md` - This file!

### Updated:
- ✅ `claude.md` - Project log with Stage 4 completion

---

## What You Can Do Now

### Test PID Immediately (5 min)

```c
void loop() {
    // Enable PID
    use_pid_control = true;
    setDemandSpeed(0.3);  // Target: 0.3 counts/ms

    // Update systems
    measureSpeeds();
    updatePID();

    // Rest of code...
}
```

**Observe:**
- Motors automatically adjust PWM to reach 0.3 counts/ms
- Speed stays constant even with load changes
- Both wheels run at same speed (fixes motor bias!)

---

### Benefits of PID Control

**Before PID (Open-loop):**
```c
motors.setPWM(30, 30);  // Both motors same PWM
// Result: Robot drifts left (~13° over 280mm)
// Speed varies with load, surface, battery
```

**After PID (Closed-loop):**
```c
setDemandSpeed(0.3);    // Both motors same SPEED
// Result: Robot drives straight!
// Speed constant regardless of load
// Motor bias automatically compensated
```

**What PID Fixes:**
- ✅ Motor bias (no more drift)
- ✅ Dead-band (PID compensates automatically)
- ✅ Load variations (speed stays constant)
- ✅ Motor differences (each controlled independently)
- ✅ Battery discharge (compensates for voltage drop)

---

## Stage 4 Acceptance Criteria

### Code Implementation ✅
- [x] Two PID instances created (left_pid, right_pid)
- [x] Initialized with starting gains (Kp=1.0, Ki=0.0, Kd=0.0)
- [x] updatePID() function implemented
- [x] Helper functions created
- [x] Integrated into main loop
- [x] Non-blocking architecture maintained
- [x] Code compiles without errors
- [x] Tuning guides created

### Physical Tuning ⏳ (HUMAN TASK)
- [ ] Kp tuned for stable speed tracking
- [ ] Ki tuned to eliminate steady-state error
- [ ] Kd tuned if needed (optional)
- [ ] Robot maintains speed within ±10%
- [ ] No oscillation at steady state
- [ ] Robot drives straight with equal demands
- [ ] Tested at multiple speeds (0.2, 0.5, 1.0 counts/ms)
- [ ] Final gains recorded in code

---

## What's Next: Physical Tuning

### Quick Test (5 minutes)
→ See `Stage4_QuickStart.md`

**Steps:**
1. Enable PID: `use_pid_control = true`
2. Set demand: `setDemandSpeed(0.3)`
3. Upload and open Serial Plotter
4. Watch PID in action!

**Expected result:**
- Measured speed rises toward demand
- Motors run at steady speed
- PWM output adjusts automatically

---

### Full Tuning (30-90 minutes)
→ See `Stage4_PID_TuningGuide.md`

**7-Step Process:**
1. Enable PID control
2. Tune Kp (P-controller)
3. Add Ki (PI-controller)
4. Test on surface
5. (Optional) Add Kd
6. Test different speeds
7. Final testing

**Result:**
- Robot drives straight
- Constant speed maintained
- Motor bias eliminated
- Ready for navigation tasks!

---

## Common Questions

### "Do I need to tune PID now?"

**No, you can skip to Stage 5 if you want.**

PID tuning is optional for now. You can:
- **Option A:** Tune PID now (30-90 min) → Robot drives straight
- **Option B:** Skip tuning, continue to Stage 5 → Come back to tune later
- **Option C:** Use open-loop control (Stage 2) for simple tasks

**Recommendation:** Quick test now (5 min), full tuning later when needed.

---

### "What if I can't get PID to work?"

**Fallback options:**
1. Use open-loop control (Stage 2)
   - `use_pid_control = false`
   - Direct PWM commands work fine
   - Just need to compensate for drift manually

2. Try different starting gains
   - Some robots need Kp=0.5, others need Kp=2.0
   - Depends on motor characteristics
   - See tuning guide for methodology

3. Tune left and right PIDs separately
   - Different gains for each motor
   - Handles motor differences better

---

### "When do I NEED PID?"

**PID becomes essential for:**
- **Straight line travel** (Assessment 1 - Beginner 58%)
- **Waypoint navigation** (Assessment 1 - Intermediate/Expert)
- **Precise motion control** (Assessment 1 - Expert 70-100%)

**Can work without PID:**
- Basic movement
- Simple exploration
- Testing sensors
- Early development

**Bottom line:** PID is HIGHLY RECOMMENDED for Assessment 1, but not strictly required for early testing.

---

## Debugging Tips

### "PID doesn't seem to do anything"

**Check:**
- `use_pid_control = true` set?
- `demand_speed` > 0.0?
- `measureSpeeds()` called before `updatePID()`?
- Battery power ON (blue LED)?

---

### "Speed oscillates wildly"

**Likely cause:** Kp too large

**Solution:**
```c
// Reduce Kp in setup()
left_pid.initialise(0.5, 0.0, 0.0);  // Was 1.0, now 0.5
```

---

### "Speed never reaches demand"

**Likely cause:** Kp too small OR demand impossible

**Solutions:**
```c
// Increase Kp
left_pid.initialise(2.0, 0.0, 0.0);  // Was 1.0, now 2.0

// OR reduce demand
setDemandSpeed(0.3);  // Instead of 2.0
```

---

### "Robot still drifts (doesn't go straight)"

**Likely cause:** Motors need different gains

**Solution:**
```c
// Tune separately in setup()
left_pid.initialise(Kp_left, Ki_left, Kd_left);
right_pid.initialise(Kp_right, Ki_right, Kd_right);

// Example:
left_pid.initialise(1.0, 0.01, 0.0);   // Left motor gains
right_pid.initialise(1.2, 0.015, 0.0); // Right motor different
```

---

## Summary

✅ **Stage 4 CODE is COMPLETE**

**You now have:**
- Two independent PID speed controllers
- Automatic speed regulation
- Helper functions for easy control
- Comprehensive tuning guides
- Non-blocking architecture

**You can now:**
- Command speeds instead of PWM
- Maintain constant velocity
- Drive in straight lines (after tuning)
- Build navigation behaviors (Assessment 1!)

**Next steps:**
1. **Quick test** (5 min) - See if PID works
2. **Full tuning** (30-90 min) - Optimize gains
3. **Continue to Stage 5** - Line sensors & boundary detection

---

## Ready to Test?

→ **Quick Test**: Open `Stage4_QuickStart.md`

→ **Full Tuning**: Open `Stage4_PID_TuningGuide.md`

→ **Skip for now**: Continue to Stage 5

**Your choice! All paths are valid.** 🎯
