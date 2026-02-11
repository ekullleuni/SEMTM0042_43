# Stage 2 Physical Testing Guide (Step 5)

## What Was Completed (Steps 1-4)

✅ **Motors.h Implementation:**
- Pin definitions: L_PWM=10, L_DIR=16, R_PWM=9, R_DIR=15
- `initialise()`: Configures pins, motors start OFF
- `setPWM(left, right)`: Handles signed PWM (-255 to +255)
  - Positive = forward, Negative = backward
  - Auto-constrains to MAX_PWM (180)
  - Converts to absolute values for analogWrite()

✅ **Non-blocking Test Code:**
- 3Pi_CodeStub.ino has safe test framework
- Motors disabled by default (uncomment to activate)
- State machine cycles every 2 seconds

✅ **Code compiles without errors**

---

## What You Need to Do (Step 5 - HUMAN TASK)

### Prerequisites
- ⚠️ **SAFETY FIRST**: Lift robot OFF surface or turn upside-down
- ⚠️ Do NOT run motors at high speed for >10 seconds
- ⚠️ Start with LOW PWM values (20-40)
- Battery power must be ON (blue LED next to Power button)

### Test Procedure

#### Test 1: Code Verification (5 min)
**Goal:** Verify code uploads and runs without errors

1. Connect robot via USB
2. Open `3Pi_CodeStub/3Pi_CodeStub/3Pi_CodeStub.ino`
3. Upload to robot
4. Open Serial Monitor (9600 baud)
5. You should see: "State 0: Motors OFF" repeating

✅ **Success criteria:** No compilation errors, serial output visible

---

#### Test 2: Dead-Band Identification (15 min)
**Goal:** Find minimum PWM where wheels start rotating

**For LEFT motor:**

1. Modify loop() - uncomment line in state 1:
   ```c
   case 1:
       motors.setPWM(20, 0);  // Start at PWM=20
       Serial.println("State 1: Left motor forward (PWM=20)");
       break;
   ```

2. Upload and observe left wheel (should be lifted off surface)
3. Does wheel rotate?
   - **NO**: Increase PWM to 25, re-upload, test again
   - **YES**: This is your dead-band! Record it.
4. Repeat for backward: `motors.setPWM(-20, 0);`
5. Record both values:
   ```
   Left motor forward dead-band: _____ PWM
   Left motor backward dead-band: _____ PWM
   ```

**Repeat for RIGHT motor:**
- Change to: `motors.setPWM(0, 20);` in state 2
- Repeat process above

**Expected values:** Typically 20-40 PWM

**Document findings:**
Add to Motors.h at top:
```c
/*
 * DEAD-BAND CALIBRATION (measured [DATE])
 * Left forward:  XX PWM
 * Left backward: XX PWM
 * Right forward: XX PWM
 * Right backward: XX PWM
 */
```

---

#### Test 3: Direction Validation (10 min)
**Goal:** Verify positive PWM = forward motion

1. Set test to: `motors.setPWM(30, 0);`
2. Upload and observe left wheel
3. **Does wheel rotate in FORWARD direction?**
   - Imagine robot on ground - would this move robot forward?
   - **YES**: Direction is correct ✅
   - **NO**: Direction is inverted ❌

4. If NO, swap FWD and REV in Motors.h:
   ```c
   #define FWD HIGH   // Changed from LOW
   #define REV LOW    // Changed from HIGH
   ```

5. Test negative PWM: `motors.setPWM(-30, 0);`
   - Should rotate BACKWARD now

6. Repeat for right motor: `motors.setPWM(0, 30);`

✅ **Success criteria:**
- Positive PWM → forward rotation
- Negative PWM → backward rotation
- Both motors behave identically (forward is forward for both)

---

#### Test 4: Motor Bias Check (10 min)
**Goal:** Measure difference between left/right motors

1. Place robot ON FLAT SURFACE (table, not carpet)
2. Modify code to run both motors:
   ```c
   case 3:
       motors.setPWM(30, 30);
       Serial.println("State 3: Both forward (PWM=30)");
       break;
   ```
3. Upload and let robot drive for 3 seconds
4. Press Power button to stop robot

**Observe:**
- Does robot drive perfectly straight? (Unlikely!)
- Which direction does it curve? (Left or Right?)
- Approximate angle of deviation: _____ degrees

**Document:**
```c
/*
 * MOTOR BIAS (measured [DATE] on [SURFACE])
 * At PWM=30, robot drifts: [LEFT/RIGHT/MINIMAL]
 * Approximate deviation: XX degrees over 1 meter
 */
```

⚠️ **This is normal!** Motors are never perfectly matched. PID control in Stage 4 will fix this.

---

#### Test 5: Movement Primitives (10 min)
**Goal:** Verify all basic movements work

Test each for 2 seconds, then stop:

```c
// Forward
motors.setPWM(30, 30);

// Backward
motors.setPWM(-30, -30);

// Turn left (on spot)
motors.setPWM(-30, 30);

// Turn right (on spot)
motors.setPWM(30, -30);

// Arc left (gentle curve)
motors.setPWM(20, 30);
```

✅ **Success criteria:** All movements execute as expected

---

### Troubleshooting

| Problem | Likely Cause | Solution |
|---------|--------------|----------|
| Wheels don't move at all | Wrong pin assignments | Re-check documentation, verify L_DIR=16, R_PWM=9, R_DIR=15 |
| Only one motor works | Code only implemented for left | Check setPWM() has right motor code too |
| Motors run opposite directions | FWD/REV inverted | Swap #define values in Motors.h |
| Robot won't stop | Power not turned off | Press Power button on robot |
| Code won't upload | USB not connected / wrong board | Check Tools > Board > "Pololu 3Pi+ 32U4" |
| Erratic behavior | MAX_PWM too high | Try reducing to 100 for testing |

---

### After Testing Complete

Once all tests pass:

1. ✅ Update claude.md with your calibration values
2. ✅ Comment out test code in loop() (return motors to OFF state)
3. ✅ Commit changes: `git add . && git commit -m "Stage 2 complete - motors calibrated"`
4. ✅ Report completion to AI assistant
5. ✅ Ready for Stage 3: Encoder Reading & Speed Measurement

---

### Quick Checklist

- [ ] Code uploads without errors
- [ ] Serial monitor shows output
- [ ] Dead-band values measured (left & right, fwd & back)
- [ ] Direction validated (positive = forward)
- [ ] Motor bias documented
- [ ] All movement primitives work
- [ ] Findings documented in Motors.h
- [ ] Test code disabled (motors OFF)
- [ ] Ready for Stage 3

**Estimated completion time: 45-60 minutes**

---

### Safety Reminders

- ⚠️ Never let motors run at high speed for >10 seconds
- ⚠️ Lift robot during testing to prevent falls
- ⚠️ If you smell burning, STOP immediately
- ⚠️ Don't force wheels by hand (damages gearbox)
- ⚠️ Turn battery power OFF when not testing
