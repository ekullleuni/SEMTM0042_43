# Stage 4: PID Quick Start Guide

## 5-Minute PID Test

Want to see PID in action right away? Follow these steps:

### Step 1: Enable PID (2 min)

Open `3Pi_CodeStub.ino` and add this code at the START of `loop()`:

```c
void loop() {

  // ============================================
  // PID QUICK TEST
  // ============================================
  use_pid_control = true;        // Enable PID
  setDemandSpeed(0.3);           // Target: 0.3 counts/ms

  // ============================================
  // Continue with normal loop
  // ============================================
  measureSpeeds();
  updatePID();

  // Rest of your code...
}
```

### Step 2: Enable Debug Output (1 min)

In the `updatePID()` function, find this section and UNCOMMENT it:

```c
// Optional: Debug output (uncomment for tuning)
Serial.print("DemL:");
Serial.print(demand_speed_left, 4);
Serial.print(",MeasL:");
Serial.print(speed_e0, 4);
Serial.print(",PWML:");
Serial.print(left_pwm_output, 1);
Serial.print(" | DemR:");
Serial.print(demand_speed_right, 4);
Serial.print(",MeasR:");
Serial.print(speed_e1, 4);
Serial.print(",PWMR:");
Serial.println(right_pwm_output, 1);
```

### Step 3: Upload and Test (2 min)

1. ⚠️ **Lift robot OFF surface** (or turn upside-down)
2. Upload code to robot
3. Open **Serial Plotter** (Tools > Serial Plotter)
4. Watch the graph!

**What you should see:**
- DemL and DemR (demand) as flat lines at 0.3
- MeasL and MeasR (measured speed) rising toward 0.3
- PWML and PWMR (PID output) adjusting to reach target

**If it works:**
- Measured speed approaches demand (within ~0.1 counts/ms)
- Motors run at steady speed
- Minimal oscillation

**If it doesn't work:**
- See full tuning guide: `Stage4_PID_TuningGuide.md`

---

## What the Numbers Mean

**Serial Plotter shows 6 lines:**

1. **DemL** (blue) - Target speed for left wheel
2. **MeasL** (red) - Actual measured left wheel speed
3. **PWML** (green) - PWM value PID is sending to left motor
4. **DemR** (purple) - Target speed for right wheel
5. **MeasR** (orange) - Actual measured right wheel speed
6. **PWMR** (yellow) - PWM value PID is sending to right motor

**Good PID behavior:**
- Measured lines (red, orange) follow demand lines (blue, purple)
- PWM values (green, yellow) are steady when speed is constant
- No wild oscillations

---

## Quick Tuning Adjustments

**If measured speed never reaches demand:**
```c
// In setup(), INCREASE Kp:
float initial_Kp = 2.0;  // Was 1.0, now 2.0
```

**If measured speed oscillates (bounces up and down):**
```c
// In setup(), DECREASE Kp:
float initial_Kp = 0.5;  // Was 1.0, now 0.5
```

**If there's always a small error (~0.05 counts/ms):**
```c
// In setup(), ADD Ki:
float initial_Ki = 0.01;  // Was 0.0, now 0.01
```

---

## Testing on Surface (IMPORTANT!)

After PID works off-surface, test ON surface:

1. Place robot on flat ground
2. Same code (demand = 0.3)
3. Watch if robot drives straight

**Expected result:**
- Robot drives in straight line (PID fixes motor bias!)
- No drift left or right
- Steady speed

**If robot still drifts:**
- Need different gains for left vs right motors
- See full tuning guide for separate tuning

---

## Next Steps

**For full PID tuning:**
→ See `Stage4_PID_TuningGuide.md`

**Includes:**
- Step-by-step Kp tuning (15-20 min)
- Step-by-step Ki tuning (15-20 min)
- Testing procedures
- Troubleshooting guide
- Common problems and solutions

**Total tuning time:** 30-90 minutes (depends on your robot)

---

## Disabling PID

To go back to direct PWM control:

```c
void loop() {
  use_pid_control = false;  // Disable PID

  // Now use direct motor commands
  motors.setPWM(30, 30);

  measureSpeeds();
  updatePID();  // Safe to call - does nothing when disabled
}
```

When `use_pid_control = false`:
- PID controllers are reset automatically
- Direct motor control works normally
- No interference between PID and manual control

---

## Common First-Time Issues

**"Motors don't run at all"**
- Check battery power (blue LED ON?)
- Check use_pid_control = true
- Check demand speed > 0.0

**"Speed goes to demand immediately (too fast)"**
- This is likely measurement lag, not actual motor speed
- Lift robot higher (ensure wheels spin freely)
- Normal - PID is working!

**"Speed never changes from 0.0"**
- Check measureSpeeds() is called before updatePID()
- Check encoders initialized (setupEncoder0/1 in setup)
- Rotate wheels by hand - do count_e0, count_e1 change?

**"Serial Plotter shows nothing"**
- Check debug Serial.print lines are uncommented
- Check Serial Monitor is CLOSED (can't have both open)
- Tools > Serial Plotter

---

## Success Checklist

Quick test is successful when:

- [ ] Code compiles and uploads without errors
- [ ] Serial Plotter shows 6 lines of data
- [ ] Measured speed (MeasL, MeasR) rises toward demand
- [ ] Motors spin at steady speed
- [ ] PWM output (PWML, PWMR) is steady when speed constant
- [ ] Robot ready for full tuning

If all checked → Proceed to full tuning guide!

If problems → Check troubleshooting section above

---

**Ready to tune properly?**

→ Open `Stage4_PID_TuningGuide.md`

**Want to understand PID better?**

→ Review labsheet: `Labsheets/Core/L1_Motors_SEMTM0042_43.ipynb`

**Happy tuning! 🎯**
