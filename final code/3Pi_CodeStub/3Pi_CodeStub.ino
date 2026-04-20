/******************************************
  Gradient Centre-Finding Test

  The robot drives across a printed grey-gradient calibration strip
  to build a per-sensor transfer function, then navigates a radial
  gradient map to find and stop at its darkest point (the centre).

  Flow:
    1. Power on  -> gradient calibration (blocking)
    2. Button A  -> placement acknowledged, wait for second press
    3. Button A  -> scan rotation, find best heading
    4. Drive toward centre using edge-differential steering
    5. Stop at centre, display results on OLED
    6. Button A  -> loop back to step 2

 SEMTM0042/43: University of Bristol.
*******************************************/

// ============================================
// INCLUDES
// ============================================
#include "Motors.h"
#include "PID.h"
#include "LineSensors.h"
#include "Kinematics.h"
#include "Encoders.h"
#include "GradientCalibration.h"

// OLED display (blue, single row of pins)
#include "oled.h"
OLED_c display(1, 30, 0, 17, 13);

// ============================================
// PIN DEFINITIONS
// ============================================
#define BUZZER_PIN   6
#define BUTTON_A_PIN 14

// ============================================
// HARDWARE INSTANCES
// ============================================
Motors_c      motors;
LineSensors_c line_sensors;
Kinematics_c  pose;
PID_c         left_pid;
PID_c         right_pid;

// ============================================
// SPEED MEASUREMENT (Stage 3)
// ============================================
unsigned long speed_measurement_ts = 0;
#define SPEED_MEASUREMENT_INTERVAL 20   // ms

long  prev_count_e0 = 0;
long  prev_count_e1 = 0;
float speed_e0 = 0.0;   // RIGHT wheel (encoder 0)
float speed_e1 = 0.0;   // LEFT  wheel (encoder 1)

// ============================================
// PID CONTROL (Stage 4)
// ============================================
unsigned long pid_update_ts = 0;
#define PID_UPDATE_INTERVAL 20          // ms

float demand_speed_left  = 0.0;
float demand_speed_right = 0.0;
bool  use_pid_control    = false;
float left_pwm_output    = 0.0;
float right_pwm_output   = 0.0;

// ============================================
// KINEMATICS & TURN/ROTATION (Stage 7)
// ============================================
unsigned long pose_update_ts = 0;
#define POSE_UPDATE_INTERVAL 10         // ms

// --- Absolute turn (setTurn / checkTurn) ---
float target_angle = 0.0;
bool  turning      = false;
#define TURN_THRESHOLD  0.01
#define TURN_GAIN       5.0
#define MAX_TURN_SPEED  0.5
#define MIN_TURN_SPEED  0.15

// --- Relative rotation (setRotation / checkRotation) ---
float rotate_target      = 0.0;
float rotate_accumulated = 0.0;
float rotate_prev_theta  = 0.0;
bool  rotating           = false;

// ============================================
// GRADIENT TEST FSM
// ============================================
enum GradientState {
  GSTATE_WAIT_START,       // Waiting for Button A to begin a run
  GSTATE_SCAN_ROTATE,      // Rotating 360 deg, sampling sensors
  GSTATE_TURN_TO_BEST,     // Turning to face the best heading found
  GSTATE_DRIVE_GRADIENT,   // Driving toward centre with reactive steering
  GSTATE_AT_CENTER,        // Centre reached — displaying results
  GSTATE_DONE              // Idle — press Button A to restart
};
GradientState grad_state = GSTATE_WAIT_START;

// --- Run timer ---
unsigned long run_start_ms = 0;

// --- Scan (Phase 3) ---
float scan_best_score = -999.0;
float scan_best_theta = 0.0;
float scan_lightness[NUM_SENSORS];           // latest converted readings
unsigned long scan_sample_ts = 0;
#define SCAN_SAMPLE_INTERVAL_MS 50           // sample every 50 ms during rotation
#define SYMMETRY_WEIGHT 0.5                  // penalty per % asymmetry — HUMAN TUNE

// --- Gradient following (Phase 4) ---
#define GRADIENT_FORWARD_SPEED 0.3           // base forward speed (counts/ms) — HUMAN TUNE
#define GRADIENT_STEER_GAIN   0.005          // steer per % lightness diff     — HUMAN TUNE
#define GRADIENT_MAX_STEER    0.15           // max differential               — HUMAN TUNE

// --- Centre detection (Phase 5) ---
#define CENTER_DARKNESS_THRESHOLD 15.0       // all sensors below this % = centre — HUMAN TUNE
#define CENTER_CONFIRM_COUNT      3          // consecutive readings to confirm
int center_confirm = 0;

// --- Safety timeout ---
#define GRADIENT_TIMEOUT_MS 30000            // 30 s max drive time

// --- Display update ---
unsigned long display_update_ts = 0;
#define DISPLAY_UPDATE_INTERVAL 500          // update OLED every 500 ms

// --- Button debounce ---
bool  button_last_state    = HIGH;           // last raw read (HIGH = released)
unsigned long button_debounce_ts = 0;
#define BUTTON_DEBOUNCE_MS 200

// --- Serial debug throttle ---
unsigned long debug_print_ts = 0;
#define DEBUG_PRINT_INTERVAL 100             // print steering info every 100 ms


// ************************************************************
//                         setup()
// ************************************************************
void setup() {

  // --- Hardware init ---
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_A_PIN, INPUT_PULLUP);
  motors.initialise();
  line_sensors.initialiseForADC();
  setupEncoder0();
  setupEncoder1();
  pose.initialise(0, 0, 0);

  Serial.begin(9600);
  delay(2000);
  Serial.println(" *** GRADIENT TEST *** ");

  // --- PID init ---
  float Kp = 48.0, Ki = 0.5, Kd = 0.0;
  left_pid.initialise(Kp, Ki, Kd);
  right_pid.initialise(Kp, Ki, Kd);
  use_pid_control = false;        // PID off until after calibration
  pid_update_ts = millis();

  // --- Speed measurement init ---
  prev_count_e0 = count_e0;
  prev_count_e1 = count_e1;
  speed_measurement_ts = millis();

  // --- OLED init ---
  display.reset();
  display.clear();
  display.gotoXY(0, 0);
  display.print("GradCal");

  // ============================================
  // PHASE 1: Gradient calibration (blocking)
  // ============================================
  // Robot must be positioned over the BLACK square of the
  // calibration strip before powering on.
  calibrateGradientSensors(25);

  // Stop motors after calibration travel
  motors.setPWM(0, 0);
  tone(BUZZER_PIN, 1000, 200);
  delay(300);

  // Show placement prompt on OLED
  display.clear();
  display.gotoXY(0, 0);
  display.print("Move me");
  display.gotoXY(0, 1);
  display.print("Press A");

  Serial.println();
  Serial.println("Calibration complete.");
  Serial.println("Move robot to gradient map, then press Button A.");

  // --- Wait for Button A (blocking) ---
  waitForButtonA();

  // --- Prepare for first run ---
  pose.initialise(0, 0, 0);
  pose_update_ts = millis();
  use_pid_control = true;
  grad_state = GSTATE_WAIT_START;

  Serial.println("Ready. Press Button A to start a run.");
}


// ************************************************************
//  BLOCKING BUTTON WAIT  (used in setup only)
// ************************************************************
void waitForButtonA() {
  // Wait for press (active LOW)
  while (digitalRead(BUTTON_A_PIN) != LOW) {}
  delay(50);
  // Wait for release
  while (digitalRead(BUTTON_A_PIN) == LOW) {}
  delay(50);
  tone(BUZZER_PIN, 440, 100);
}


// ************************************************************
//  NON-BLOCKING BUTTON CHECK  (used in loop)
// ************************************************************
/*
 * Returns true once per button press (rising-edge detection
 * with debounce).  Safe to call every loop iteration.
 */
bool buttonAPressed() {
  bool raw = digitalRead(BUTTON_A_PIN);   // LOW = pressed
  unsigned long now = millis();

  // Debounce: ignore transitions within BUTTON_DEBOUNCE_MS
  if (now - button_debounce_ts < BUTTON_DEBOUNCE_MS) {
    button_last_state = raw;
    return false;
  }

  // Detect falling edge: was HIGH (released), now LOW (pressed)
  if (button_last_state == HIGH && raw == LOW) {
    button_debounce_ts = now;
    button_last_state = raw;
    return true;
  }

  button_last_state = raw;
  return false;
}


// ************************************************************
//  GRADIENT CALIBRATION HELPERS  (blocking — called from setup)
// ************************************************************

/*
 * Drive forward dist_mm using encoder counts.
 * BLOCKING — only for calibration in setup().
 */
void driveForwardBlocking(float dist_mm) {
  long target_counts = (long)(dist_mm * GRAD_COUNTS_PER_MM);
  long start_e0 = count_e0;
  long start_e1 = count_e1;

  motors.setPWM(40, 40);

  while (true) {
    long avg = ((count_e0 - start_e0) + (count_e1 - start_e1)) / 2;
    if (avg >= target_counts) break;
  }

  motors.setPWM(0, 0);
  delay(400);
}


/*
 * Step across a gradient calibration strip, recording averaged
 * line-sensor readings at each shade.  See GradientCalibration.h
 * for storage arrays and transfer function.
 */
void calibrateGradientSensors(int num_divisions) {

  if (num_divisions > GRAD_CAL_MAX_DIVISIONS) num_divisions = GRAD_CAL_MAX_DIVISIONS;
  if (num_divisions < 2) return;

  Serial.println();
  Serial.println("==========================================");
  Serial.println("  GRADIENT LINE-SENSOR CALIBRATION");
  Serial.println("==========================================");
  Serial.print("  Shades     : "); Serial.println(num_divisions);
  Serial.print("  Step size  : "); Serial.print(GRAD_CAL_STEP_MM, 0); Serial.println(" mm");
  Serial.print("  Samples/shade : "); Serial.println(GRAD_CAL_SAMPLES);
  Serial.println();
  Serial.println("  Place robot over BLACK square.");
  Serial.println("  Starting in 3 seconds...");
  delay(3000);

  tone(BUZZER_PIN, 880, 150);
  delay(250);

  for (int div = 0; div < num_divisions; div++) {

    // Lightness % for this division (0=black, 100=white)
    // (Computed on-the-fly via grad_cal_pct(div) in GradientCalibration.h)
    float div_pct = (float)div * 100.0f / (float)(num_divisions - 1);

    // Accumulate multiple ADC readings and average
    float sums[NUM_SENSORS];
    for (int s = 0; s < NUM_SENSORS; s++) sums[s] = 0.0f;

    for (int sample = 0; sample < GRAD_CAL_SAMPLES; sample++) {
      line_sensors.readSensorsADC();
      for (int s = 0; s < NUM_SENSORS; s++) {
        sums[s] += line_sensors.readings[s];
      }
      delay(5);
    }

    for (int s = 0; s < NUM_SENSORS; s++) {
      grad_cal_raw[div][s] = sums[s] / (float)GRAD_CAL_SAMPLES;
    }

    // Print progress
    Serial.print("DIV ");
    if (div < 10) Serial.print(" ");
    Serial.print(div);
    Serial.print("  (");
    if (div_pct < 10) Serial.print("  ");
    else if (div_pct < 100) Serial.print(" ");
    Serial.print(div_pct, 1);
    Serial.print("%)  S0:");
    Serial.print(grad_cal_raw[div][0], 0);
    Serial.print("  S1:");
    Serial.print(grad_cal_raw[div][1], 0);
    Serial.print("  S2:");
    Serial.print(grad_cal_raw[div][2], 0);
    Serial.print("  S3:");
    Serial.print(grad_cal_raw[div][3], 0);
    Serial.print("  S4:");
    Serial.println(grad_cal_raw[div][4], 0);

    // Advance to next shade (skip after last)
    if (div < num_divisions - 1) {
      driveForwardBlocking(GRAD_CAL_STEP_MM);
    }
  }

  grad_cal_num_divisions = num_divisions;
  grad_cal_complete = true;

  tone(BUZZER_PIN, 880,  100); delay(150);
  tone(BUZZER_PIN, 1100, 200); delay(250);

  // CSV output for Python/Excel analysis
  Serial.println();
  Serial.println("--- CSV (copy into Python/Excel) ---");
  Serial.println("Division,Percent,S0,S1,S2,S3,S4");
  for (int div = 0; div < num_divisions; div++) {
    Serial.print(div);
    Serial.print(",");
    Serial.print(grad_cal_pct(div), 2);
    for (int s = 0; s < NUM_SENSORS; s++) {
      Serial.print(",");
      Serial.print(grad_cal_raw[div][s], 1);
    }
    Serial.println();
  }
  Serial.println("--- END CSV ---");

  // Also print cubic vs linear comparison for first sensor
  Serial.println();
  Serial.println("--- INTERPOLATION COMPARISON (Sensor 2 / centre) ---");
  Serial.println("RawADC,Linear%,Cubic%");
  float r_min = grad_cal_raw[0][2];
  float r_max = grad_cal_raw[num_divisions - 1][2];
  for (int step = 0; step <= 50; step++) {
    float raw = r_min + (r_max - r_min) * step / 50.0f;
    use_cubic_interp = false;
    float lin = gradientToPercent(2, raw);
    use_cubic_interp = true;
    float cub = gradientToPercent(2, raw);
    Serial.print(raw, 1);
    Serial.print(",");
    Serial.print(lin, 2);
    Serial.print(",");
    Serial.println(cub, 2);
  }
  use_cubic_interp = false;  // default back to linear
  Serial.println("--- END COMPARISON ---");
  Serial.println();
  Serial.println("  Calibration COMPLETE.");
  Serial.println("==========================================");
}


// ************************************************************
//  SYSTEM UPDATE FUNCTIONS  (called every loop iteration)
// ************************************************************

// Measure wheel speeds from encoder deltas (non-blocking, 20 ms)
void measureSpeeds() {
  unsigned long now = millis();
  unsigned long dt  = now - speed_measurement_ts;

  if (dt >= SPEED_MEASUREMENT_INTERVAL) {
    long local_e0, local_e1;
    noInterrupts();
    local_e0 = count_e0;
    local_e1 = count_e1;
    interrupts();

    speed_e0 = (float)(local_e0 - prev_count_e0) / (float)dt;
    speed_e1 = (float)(local_e1 - prev_count_e1) / (float)dt;

    prev_count_e0 = local_e0;
    prev_count_e1 = local_e1;
    speed_measurement_ts = now;
  }
}

// PID motor control (non-blocking, 20 ms)
void updatePID() {
  unsigned long now = millis();
  if (now - pid_update_ts < PID_UPDATE_INTERVAL) return;

  if (use_pid_control) {
    if (demand_speed_left == 0.0) {
      left_pwm_output = 0.0;
      left_pid.reset();
    } else {
      left_pwm_output = left_pid.update(demand_speed_left, speed_e1);
    }
    if (demand_speed_right == 0.0) {
      right_pwm_output = 0.0;
      right_pid.reset();
    } else {
      right_pwm_output = right_pid.update(demand_speed_right, speed_e0);
    }
    motors.setPWM(left_pwm_output, right_pwm_output);
  } else {
    left_pid.reset();
    right_pid.reset();
  }

  pid_update_ts = now;
}

// Update dead-reckoning pose (non-blocking, 10 ms)
void updatePose() {
  unsigned long now = millis();
  if (now - pose_update_ts >= POSE_UPDATE_INTERVAL) {
    pose.update();
    pose_update_ts = now;
  }
}


// ************************************************************
//  MOTOR DEMAND HELPERS
// ************************************************************

void setDemandSpeeds(float left_speed, float right_speed) {
  demand_speed_left  = left_speed;
  demand_speed_right = 0.88 * right_speed;   // hardware bias correction
}

void setDemandSpeed(float speed) {
  setDemandSpeeds(speed, speed);
}

void stopWithPID() {
  setDemandSpeeds(0.0, 0.0);
  motors.setPWM(0, 0);
  left_pid.reset();
  right_pid.reset();
}


// ************************************************************
//  NAVIGATION HELPERS  (shared with existing codebase)
// ************************************************************

// Smallest angular difference, normalised to [-PI, PI]
float angleDiff(float target, float current) {
  float diff = target - current;
  return atan2(sin(diff), cos(diff));
}

// Angle from current pose to a point
float angleToPoint(float tx, float ty) {
  return atan2(ty - pose.y, tx - pose.x);
}

// Euclidean distance from current pose to a point
float distanceToPoint(float tx, float ty) {
  float dx = tx - pose.x;
  float dy = ty - pose.y;
  return sqrt(dx * dx + dy * dy);
}


// ============================================
// ABSOLUTE TURN  (setTurn / checkTurn)
// ============================================

void setTurn(float angle) {
  target_angle = angle;
  turning = true;
}

bool checkTurn() {
  if (!turning) return true;

  float angle_error = angleDiff(target_angle, pose.theta);

  if (fabs(angle_error) < TURN_THRESHOLD) {
    stopWithPID();
    turning = false;
    return true;
  }

  float turn_speed = angle_error * TURN_GAIN;
  if (turn_speed >  MAX_TURN_SPEED) turn_speed =  MAX_TURN_SPEED;
  if (turn_speed < -MAX_TURN_SPEED) turn_speed = -MAX_TURN_SPEED;
  if (fabs(turn_speed) < MIN_TURN_SPEED) {
    turn_speed = (angle_error > 0) ? MIN_TURN_SPEED : -MIN_TURN_SPEED;
  }

  setDemandSpeeds(-turn_speed, turn_speed);
  return false;
}


// ============================================
// RELATIVE ROTATION  (setRotation / checkRotation)
// ============================================

void setRotation(float radians) {
  rotate_target      = radians;
  rotate_accumulated = 0.0;
  rotate_prev_theta  = pose.theta;
  rotating           = true;
}

bool checkRotation() {
  if (!rotating) return true;

  float delta = angleDiff(pose.theta, rotate_prev_theta);
  rotate_accumulated += delta;
  rotate_prev_theta   = pose.theta;

  float remaining = rotate_target - rotate_accumulated;

  if (fabs(remaining) < TURN_THRESHOLD) {
    stopWithPID();
    rotating = false;
    return true;
  }

  float turn_speed = remaining * TURN_GAIN;
  if (turn_speed >  MAX_TURN_SPEED) turn_speed =  MAX_TURN_SPEED;
  if (turn_speed < -MAX_TURN_SPEED) turn_speed = -MAX_TURN_SPEED;
  if (fabs(turn_speed) < MIN_TURN_SPEED) {
    turn_speed = (remaining > 0) ? MIN_TURN_SPEED : -MIN_TURN_SPEED;
  }

  setDemandSpeeds(-turn_speed, turn_speed);
  return false;
}


// ************************************************************
//  GRADIENT FSM HELPER FUNCTIONS
// ************************************************************

/*
 * Read all 5 sensors, convert to lightness % via the transfer
 * function, and return a heading-quality score.
 *
 * Score = (100 - centre_lightness) - SYMMETRY_WEIGHT * asymmetry
 *
 * Higher score = darker centre + more symmetric sensor readings
 * = robot is better aligned along a radius toward the centre.
 */
float computeScanScore() {
  line_sensors.readSensorsADC();

  for (int i = 0; i < NUM_SENSORS; i++) {
    scan_lightness[i] = gradientToPercent(i, line_sensors.readings[i]);
  }

  float centre = scan_lightness[2];   // DN3 = centre sensor
  float asym_outer = fabs(scan_lightness[0] - scan_lightness[4]);  // |DN1 - DN5|
  float asym_inner = fabs(scan_lightness[1] - scan_lightness[3]);  // |DN2 - DN4|
  float asymmetry  = asym_outer + asym_inner;

  return (100.0f - centre) - SYMMETRY_WEIGHT * asymmetry;
}


/*
 * Check whether the robot has reached the centre of the gradient.
 *
 * Reads all 5 sensors, converts to lightness %.  If ALL sensors
 * are below CENTER_DARKNESS_THRESHOLD for CENTER_CONFIRM_COUNT
 * consecutive calls, returns true.
 */
bool checkCenterReached() {
  line_sensors.readSensorsADC();

  bool all_dark = true;
  for (int i = 0; i < NUM_SENSORS; i++) {
    float pct = gradientToPercent(i, line_sensors.readings[i]);
    if (pct > CENTER_DARKNESS_THRESHOLD) {
      all_dark = false;
      break;
    }
  }

  if (all_dark) {
    center_confirm++;
  } else {
    center_confirm = 0;
  }

  return (center_confirm >= CENTER_CONFIRM_COUNT);
}


/*
 * Print final results to Serial and OLED.
 */
void reportResults() {
  float elapsed_s  = (float)(millis() - run_start_ms) / 1000.0f;
  float dist       = sqrt(pose.x * pose.x + pose.y * pose.y);
  float heading_deg = pose.theta * 180.0f / PI;

  // --- Serial ---
  Serial.println();
  Serial.println("=== GRADIENT CENTRE FOUND ===");
  Serial.print("  Position : X=");
  Serial.print(pose.x, 1);
  Serial.print(" mm, Y=");
  Serial.print(pose.y, 1);
  Serial.println(" mm");
  Serial.print("  Distance : ");
  Serial.print(dist, 1);
  Serial.println(" mm from start");
  Serial.print("  Heading  : ");
  Serial.print(heading_deg, 1);
  Serial.println(" deg");
  Serial.print("  Time     : ");
  Serial.print(elapsed_s, 2);
  Serial.println(" s");
  Serial.print("  Scan score: ");
  Serial.print(scan_best_score, 1);
  Serial.print(" at ");
  Serial.print(scan_best_theta * 180.0f / PI, 1);
  Serial.println(" deg");
  Serial.println("=============================");

  // --- OLED ---
  display.clear();
  display.gotoXY(0, 0);
  display.print("X:");
  display.print((int)pose.x);
  display.print(" Y:");
  display.print((int)pose.y);

  display.gotoXY(0, 1);
  display.print("T:");
  display.print(elapsed_s, 1);
  display.print("s");
}


/*
 * Non-blocking OLED update.  Called every loop iteration;
 * only redraws when DISPLAY_UPDATE_INTERVAL has elapsed.
 * Content depends on the current FSM state.
 */
void updateGradientDisplay() {
  unsigned long now = millis();
  if (now - display_update_ts < DISPLAY_UPDATE_INTERVAL) return;
  display_update_ts = now;

  // Don't overwrite the results screen
  if (grad_state == GSTATE_AT_CENTER) return;

  display.clear();
  display.gotoXY(0, 0);

  switch (grad_state) {

    case GSTATE_WAIT_START:
      display.print("Press A");
      display.gotoXY(0, 1);
      display.print("to start");
      break;

    case GSTATE_SCAN_ROTATE:
      display.print("Scanning");
      display.gotoXY(0, 1);
      display.print("S:");
      display.print(scan_best_score, 1);
      break;

    case GSTATE_TURN_TO_BEST:
      display.print("Aligning");
      display.gotoXY(0, 1);
      display.print((int)(scan_best_theta * 180.0f / PI));
      display.print(" deg");
      break;

    case GSTATE_DRIVE_GRADIENT: {
      display.print("Driving");
      display.gotoXY(0, 1);
      // Show centre sensor lightness
      line_sensors.readSensorsADC();
      float c = gradientToPercent(2, line_sensors.readings[2]);
      display.print("C:");
      display.print(c, 1);
      display.print("%");
      break;
    }

    case GSTATE_DONE:
      display.print("Done!");
      display.gotoXY(0, 1);
      display.print("Press A");
      break;

    default:
      break;
  }
}


// ************************************************************
//                          loop()
// ************************************************************
void loop() {

  // --- Continuous system updates (every iteration) ---
  measureSpeeds();
  updatePID();
  updatePose();

  // --- Gradient FSM ---
  switch (grad_state) {

    // ========================================================
    // WAIT_START: robot is on the gradient map, awaiting go
    // ========================================================
    case GSTATE_WAIT_START:
      if (buttonAPressed()) {
        // Reset pose — robot is at the starting point
        pose.initialise(0, 0, 0);
        pose_update_ts = millis();
        run_start_ms   = millis();

        // Take one reading to decide scan rotation direction.
        // Rotate toward the side with the darker (lower lightness) edge sensor.
        line_sensors.readSensorsADC();
        float left_pct  = gradientToPercent(0, line_sensors.readings[0]);
        float right_pct = gradientToPercent(4, line_sensors.readings[4]);

        // Darker side (lower %) = toward centre.
        // CCW (+) if left darker, CW (-) if right darker.
        float scan_dir = (left_pct < right_pct) ? 1.0f : -1.0f;
        setRotation(scan_dir * 2.0f * PI);

        scan_best_score = -999.0f;
        scan_best_theta = pose.theta;
        scan_sample_ts  = millis();

        Serial.println();
        Serial.println("--- RUN STARTED ---");
        Serial.print("  Scan direction: ");
        Serial.println((scan_dir > 0) ? "CCW" : "CW");

        grad_state = GSTATE_SCAN_ROTATE;
      }
      break;

    // ========================================================
    // SCAN_ROTATE: full 360 deg rotation, sampling sensors
    // ========================================================
    case GSTATE_SCAN_ROTATE:
      // Sample at regular intervals during rotation
      if (millis() - scan_sample_ts >= SCAN_SAMPLE_INTERVAL_MS) {
        float score = computeScanScore();

        if (score > scan_best_score) {
          scan_best_score = score;
          scan_best_theta = pose.theta;
        }

        scan_sample_ts = millis();
      }

      // Check if full rotation is complete
      if (checkRotation()) {
        Serial.print("  Scan complete. Best heading: ");
        Serial.print(scan_best_theta * 180.0f / PI, 1);
        Serial.print(" deg, score: ");
        Serial.println(scan_best_score, 1);

        // Turn to face the best heading found
        setTurn(scan_best_theta);
        grad_state = GSTATE_TURN_TO_BEST;
      }
      break;

    // ========================================================
    // TURN_TO_BEST: align to the heading found during scan
    // ========================================================
    case GSTATE_TURN_TO_BEST:
      if (checkTurn()) {
        center_confirm = 0;
        debug_print_ts = millis();

        Serial.println("  Aligned. Driving toward centre...");
        grad_state = GSTATE_DRIVE_GRADIENT;
      }
      break;

    // ========================================================
    // DRIVE_GRADIENT: reactive edge-differential steering
    // ========================================================
    case GSTATE_DRIVE_GRADIENT: {

      // --- Read sensors and convert to lightness % ---
      line_sensors.readSensorsADC();
      float L[NUM_SENSORS];
      for (int i = 0; i < NUM_SENSORS; i++) {
        L[i] = gradientToPercent(i, line_sensors.readings[i]);
      }

      // --- Edge-differential steering (Method A) ---
      // Left avg = (DN1 + DN2) / 2,  Right avg = (DN4 + DN5) / 2
      float left_avg  = (L[0] + L[1]) * 0.5f;
      float right_avg = (L[3] + L[4]) * 0.5f;
      float diff = left_avg - right_avg;
      // diff > 0 → left is LIGHTER → centre is to the RIGHT → steer right
      // Steer right = increase left speed, decrease right speed

      float steer = diff * GRADIENT_STEER_GAIN;
      if (steer >  GRADIENT_MAX_STEER) steer =  GRADIENT_MAX_STEER;
      if (steer < -GRADIENT_MAX_STEER) steer = -GRADIENT_MAX_STEER;

      float left_speed  = GRADIENT_FORWARD_SPEED + steer;
      float right_speed = GRADIENT_FORWARD_SPEED - steer;
      setDemandSpeeds(left_speed, right_speed);

      // --- Serial debug output (~10 Hz) ---
      if (millis() - debug_print_ts >= DEBUG_PRINT_INTERVAL) {
        Serial.print("L_avg:");
        Serial.print(left_avg, 1);
        Serial.print(" R_avg:");
        Serial.print(right_avg, 1);
        Serial.print(" diff:");
        Serial.print(diff, 1);
        Serial.print(" steer:");
        Serial.print(steer, 4);
        Serial.print(" C:");
        Serial.println(L[2], 1);
        debug_print_ts = millis();
      }

      // --- Centre detection ---
      if (checkCenterReached()) {
        stopWithPID();
        reportResults();
        tone(BUZZER_PIN, 1100, 150); delay(200);
        tone(BUZZER_PIN, 1320, 150); delay(200);
        tone(BUZZER_PIN, 1540, 300);
        grad_state = GSTATE_AT_CENTER;
        break;
      }

      // --- Safety timeout ---
      if (millis() - run_start_ms >= GRADIENT_TIMEOUT_MS) {
        stopWithPID();
        Serial.println("*** TIMEOUT — centre not found ***");
        reportResults();
        grad_state = GSTATE_AT_CENTER;
      }

      break;
    }

    // ========================================================
    // AT_CENTER: results displayed, transition to DONE
    // ========================================================
    case GSTATE_AT_CENTER:
      // One-shot state: immediately transition to DONE
      grad_state = GSTATE_DONE;
      break;

    // ========================================================
    // DONE: idle, waiting for Button A to restart
    // ========================================================
    case GSTATE_DONE:
      if (buttonAPressed()) {
        Serial.println();
        Serial.println("--- RESTARTING ---");
        Serial.println("Move robot to new position, then press Button A.");

        // Show placement prompt
        display.clear();
        display.gotoXY(0, 0);
        display.print("Move me");
        display.gotoXY(0, 1);
        display.print("Press A");

        grad_state = GSTATE_WAIT_START;
      }
      break;

  } // end switch

  // --- OLED display update (non-blocking, 500 ms) ---
  updateGradientDisplay();
}
