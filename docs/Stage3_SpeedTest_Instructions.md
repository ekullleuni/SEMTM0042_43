# Stage 3: Encoder & Speed Measurement Testing Guide

## What Was Implemented

✅ **Speed Measurement System:**
- Non-blocking speed measurement function (`measureSpeeds()`)
- Runs every 20ms using `millis()` timing
- Calculates speed for both wheels (left: speed_e0, right: speed_e1)
- Speed units: **encoder counts per millisecond**
- Global variables available: `speed_e0`, `speed_e1`, `count_e0`, `count_e1`

✅ **Integration:**
- Initialized in `setup()` function
- Called automatically in `loop()` every cycle
- Fully non-blocking (no delay() calls)
- Compatible with existing motor control code

---

## Testing Procedure

### Test 1: Verify Encoder Counts (5 min)

**Goal:** Confirm encoders are counting correctly

**Method:**
1. Upload the current code to your robot
2. Open Serial Monitor (9600 baud)
3. You should see: "Stage 3: Speed measurement initialized"

**Manual Test (Motors OFF):**
1. Lift robot or turn upside-down
2. Add this to your `loop()` function temporarily:
   ```c
   // Add at end of loop(), just before closing brace
   static unsigned long debug_ts = 0;
   if (millis() - debug_ts >= 500) {  // Every 500ms
       debug_ts = millis();
       Serial.print("E0:");
       Serial.print(count_e0);
       Serial.print(" E1:");
       Serial.println(count_e1);
   }
   ```
3. Upload and open Serial Monitor
4. Rotate left wheel FORWARD by hand (slowly)
   - count_e0 should INCREASE
5. Rotate left wheel BACKWARD by hand
   - count_e0 should DECREASE
6. Repeat for right wheel with count_e1

✅ **Success criteria:**
- count_e0 increases when left wheel rotates forward
- count_e0 decreases when left wheel rotates backward
- count_e1 increases when right wheel rotates forward
- count_e1 decreases when right wheel rotates backward

⚠️ **IMPORTANT:** Rotate wheels slowly and gently - fast rotation can damage gearbox!

---

### Test 2: Verify Speed Measurement (10 min)

**Goal:** Confirm speed calculation is working

**Method:**
1. Uncomment the debug Serial.print lines in `measureSpeeds()` function:
   ```c
   // In measureSpeeds() function, uncomment these lines:
   Serial.print("L:");
   Serial.print(speed_e0, 4);
   Serial.print(",R:");
   Serial.println(speed_e1, 4);
   ```

2. Upload and open Serial Monitor (or Serial Plotter for graphs)

**Manual Test (Motors OFF):**
1. Rotate left wheel slowly by hand
   - You should see L: value changing (positive or negative)
   - When you stop, L: should return to ~0.0
2. Rotate right wheel slowly by hand
   - You should see R: value changing
   - When you stop, R: should return to ~0.0

**Motor Test (Motors ON):**
1. Change the motor test switch statement to use `test_state` instead of `3`:
   ```c
   switch(test_state) {  // Change from switch(3) to switch(test_state)
   ```
2. Uncomment the motor commands in states 1-3
3. Open Serial Plotter (Tools > Serial Plotter)
4. ⚠️ Lift robot OFF surface
5. Upload and observe:
   - State 0: Both speeds should be ~0.0
   - State 1: L: should be positive, R: ~0.0
   - State 2: L: ~0.0, R: should be positive
   - State 3: Both L: and R: should be positive

✅ **Success criteria:**
- Speed measurements respond to wheel rotation
- Speed returns to ~0.0 when wheels stop
- Positive rotation → positive speed
- Negative rotation → negative speed
- Speed measurements update every 20ms

---

### Test 3: Speed Calibration (15 min)

**Goal:** Understand relationship between PWM and speed

**Procedure:**
1. Modify state 3 to test at PWM=30:
   ```c
   case 3:
       motors.setPWM(30, 30);
       Serial.println("State 3: Both motors PWM=30");
       break;
   ```

2. Open Serial Plotter
3. ⚠️ Lift robot OFF surface (or turn upside-down)
4. Upload and observe speed readings

**Record these values:**
- Average speed_e0 at PWM=30: ______ counts/ms
- Average speed_e1 at PWM=30: ______ counts/ms

**Repeat at different PWM values:**
- PWM=50: speed_e0=______, speed_e1=______
- PWM=80: speed_e0=______, speed_e1=______

**On Surface Test (Optional):**
- Place robot ON flat surface
- Test PWM=30 (both motors)
- Note: Speeds will be DIFFERENT than off-surface due to friction/load
- This is NORMAL and demonstrates why PID is needed!

✅ **Success criteria:**
- Higher PWM → higher speed (roughly linear)
- Speed values are consistent (±10% variation)
- Left and right speeds are similar but not identical (motor bias from Stage 2)

---

### Test 4: Encoder Count Verification (10 min)

**Goal:** Verify encoder counts match physical rotation

**Known Values:**
- Encoder resolution: **358.3 counts per wheel revolution**
  - (12 counts per motor shaft revolution × 29.86 gear ratio)
- Wheel circumference: ~100mm (π × 32mm diameter)

**Test:**
1. Mark a point on the wheel with tape
2. Place robot ON surface (motors OFF)
3. Note starting count_e0: ______
4. Roll robot forward, rotating left wheel EXACTLY 1 full revolution
5. Note ending count_e0: ______
6. Calculate difference: ______

**Expected result:** ~358 counts (±10 counts acceptable)

If far off (>50 counts error):
- Check if wheel slipped during test
- Ensure encoders initialized correctly (setupEncoder0/1 called)
- Verify you rotated exactly 1 revolution

✅ **Success criteria:**
- 1 wheel revolution ≈ 358 encoder counts

---

## Common Issues & Solutions

| Problem | Likely Cause | Solution |
|---------|--------------|----------|
| Encoder counts don't change | Encoders not initialized | Check setupEncoder0/1 called in setup() |
| Counts change erratically | Electrical noise | Normal at very low speeds, smooth at higher speeds |
| Speed always 0.0 | Motors not running | Check motor power (blue LED on?), check setPWM() |
| Speed oscillates wildly | Measurement interval too short | 20ms is good, don't go below 10ms |
| Left/right speeds very different | Motor bias (normal) | Will be fixed in Stage 4 PID |
| Negative speed when going forward | Encoder wiring inverted | Either reverse motor direction OR negate encoder count |

---

## Expected Speed Values

**Typical speeds (measured off-surface, motors unloaded):**
- PWM=30: ~0.3 to 0.5 counts/ms
- PWM=50: ~0.5 to 0.8 counts/ms
- PWM=80: ~0.8 to 1.2 counts/ms

**On-surface speeds will be LOWER** due to friction and load.

**Speed units conversion:**
- counts/ms × 1000 = counts/second
- counts/second ÷ 358.3 = revolutions/second
- revolutions/second × 100mm = mm/second (linear speed)

Example: 0.5 counts/ms = 500 counts/sec = 1.4 rev/sec = 140 mm/sec

---

## Stage 3 Acceptance Criteria Checklist

Before proceeding to Stage 4:

- [ ] Encoder counts increase when wheels rotate forward
- [ ] Encoder counts decrease when wheels rotate backward
- [ ] Speed measurement produces sensible values (0.0 when stopped)
- [ ] Speed measurement runs at consistent 20ms intervals
- [ ] Speed responds correctly to motor commands
- [ ] ~358 counts per wheel revolution verified
- [ ] No delay() calls in speed measurement code
- [ ] Code compiles without errors
- [ ] Both wheels measured independently

---

## What's Next: Stage 4 Preview

Once Stage 3 is validated, Stage 4 will implement **PID Speed Controllers**.

PID will:
- Command **speed** (not PWM) to motors
- Automatically adjust PWM to maintain constant speed
- Compensate for motor bias (fix the 13° drift from Stage 2)
- Enable smooth, reliable motion control

**Stage 4 will require physical tuning** (finding Kp, Ki, Kd values).

---

## Debugging Tips

**Serial Plotter is your friend!**
- Tools > Serial Plotter
- Shows real-time graphs of speed_e0 and speed_e1
- Much easier to see patterns than Serial Monitor

**Useful debug code:**
```c
// Add to loop() for detailed debugging
Serial.print("E0:");
Serial.print(count_e0);
Serial.print(" E1:");
Serial.print(count_e1);
Serial.print(" | L:");
Serial.print(speed_e0, 4);
Serial.print(" R:");
Serial.println(speed_e1, 4);
```

**Remember:**
- Always comment out debug prints before final submission
- Excessive Serial.print() can slow down your loop()
- Use conditional prints (every 100ms, not every loop)
