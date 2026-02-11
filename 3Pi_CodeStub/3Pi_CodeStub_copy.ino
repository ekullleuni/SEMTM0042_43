/******************************************

	______      _           _   _              
	| ___ \    | |         | | (_)             
	| |_/ /___ | |__   ___ | |_ _  ___ ___     
	|    // _ \| '_ \ / _ \| __| |/ __/ __|    
	| |\ \ (_) | |_) | (_) | |_| | (__\__ \    
	\_| \_\___/|_.__/ \___/ \__|_|\___|___/    
											   
											   
	 _____      _                              
	/  ___|    (_)                       ___   
	\ `--.  ___ _  ___ _ __   ___ ___   ( _ )  
	 `--. \/ __| |/ _ \ '_ \ / __/ _ \  / _ \/\
	/\__/ / (__| |  __/ | | | (_|  __/ | (_>  <
	\____/ \___|_|\___|_| |_|\___\___|  \___/\/
											   
											   
	 _____           _                         
	/  ___|         | |                        
	\ `--. _   _ ___| |_ ___ _ __ ___  ___     
	 `--. \ | | / __| __/ _ \ '_ ` _ \/ __|    
	/\__/ / |_| \__ \ ||  __/ | | | | \__ \    
	\____/ \__, |___/\__\___|_| |_| |_|___/    
			__/ |                              
		   |___/                               


 SEMTM0042/43: University of Bristol.
 https://github.com/paulodowd/SEMTM0042_43
*******************************************/


// These #include commands essentially "copy and paste" 
// the above .h files (tabs above) into your code here.

// Labsheet 1: Build a class to operate your motors.
#include "Motors.h"         

// Labsheet 1: Tune PID to control wheel speed.
#include "PID.h"            

// Labsheet 2: Build a class to read the line sensors.
#include "LineSensors.h"    

// Labsheet 2: Operate the magnetometer to detect the 
//             puck.  You will need to follow the steps
//             to install the software library.
//#include "Magnetometer.h" // Labsheet 2 

// Labsheet 3: Calibrate and use kinematics to allow 
//             your robot to turn and travel between
//             locations.
#include "Kinematics.h"  

// Encoders.h does not need modifying.
#include "Encoders.h"     // For encoder counts


// If you want to use one of the Display modules for
// your robot, please read Supplementary Labsheet 4.
// You can only use one of the following.  Uncomment
// the correct one for your display module.  If you
// have not followed Supplementary Labsheet 4, these 
// will cause a compile error in your code.  

// Uncomment the next two lines if you will use
// the GREEN display with 2 rows of pins.
//#include "lcd.h"
//LCD_c display(0, 1, 14, 17, 13, 30);

// Uncomment the next two lines if you will use
// the BLUE display with 1 row of pins.
//#include "oled.h"
//OLED_c display(1, 30, 0, 17, 13);


// Used for Labsheet 0. Defines which pin the buzzer
// is attached to. #define works like a
// find-and-replace through your code.
// See Labsheet 0.
#define BUZZER_PIN 6

// Button A pin on Pololu 3Pi+ 32U4
// Button is active-low (pressed = LOW)
#define BUTTON_A_PIN 14

// See Labsheet 1.
Motors_c motors;    

// See Labsheet 2.
LineSensors_c line_sensors; 

// See Labsheet 2.
//Magnetometer_c magnetometer;

// See Labsheet 3.
Kinematics_c pose;

// ============================================
// STAGE 4: PID Speed Controllers
// ============================================
// Two PID controllers - one for each motor
// These will maintain constant wheel speeds by
// automatically adjusting PWM output
PID_c left_pid;   // PID controller for left motor
PID_c right_pid;  // PID controller for right motor

// ============================================
// STAGE 3: ENCODER READING & SPEED MEASUREMENT VARIABLES
// ============================================

// Speed measurement timing
unsigned long speed_measurement_ts = 0;
#define SPEED_MEASUREMENT_INTERVAL 20  // Measure speed every 20ms

// Previous encoder counts (for calculating change over time)
long prev_count_e0 = 0;
long prev_count_e1 = 0;

// Calculated wheel speeds (in encoder counts per millisecond)
// NOTE: From Encoders.h, e0 = RIGHT wheel, e1 = LEFT wheel
float speed_e0 = 0.0;  // RIGHT wheel speed (encoder 0)
float speed_e1 = 0.0;  // LEFT wheel speed (encoder 1)

// ============================================
// STAGE 4: PID CONTROL VARIABLES
// ============================================

// PID update timing
unsigned long pid_update_ts = 0;
#define PID_UPDATE_INTERVAL 10  // Update PID every 50ms

// Speed demands (in counts per millisecond)
float demand_speed_left = 0.0;
float demand_speed_right = 0.0;

// PID enable flag
bool use_pid_control = false;  // Start with PID disabled for safety

// PID output values (PWM signals calculated by PID)
float left_pwm_output = 0.0;
float right_pwm_output = 0.0;

// ============================================
// STAGE 5: LINE SENSOR VARIABLES
// ============================================

// Threshold for detecting black line (0.0 = white, 1.0 = black)
// After calibration, readings above this threshold indicate black line
// Start with 0.5 (midpoint), adjust based on testing
#define LINE_THRESHOLD 0.7

// Flag to indicate if line sensors have been calibrated
bool line_sensors_calibrated = false;

// ============================================
// STAGE 7: KINEMATICS & NAVIGATION VARIABLES
// ============================================

// Pose update timing (should match speed measurement interval)
unsigned long pose_update_ts = 0;
#define POSE_UPDATE_INTERVAL 20  // Update pose every 20ms

// Turn behavior variables
float target_angle = 0.0;         // Target orientation (radians)
bool turning = false;              // Is robot currently turning?
#define TURN_THRESHOLD 0.05        // Stop turning when within ±0.05 radians (~2.9°)
#define TURN_GAIN 5.0             // Proportional gain for turning (adjust based on testing)

// Travel behavior variables
float target_x = 0.0;              // Target X coordinate (mm)
float target_y = 0.0;              // Target Y coordinate (mm)
bool traveling = false;            // Is robot currently traveling?
#define TRAVEL_THRESHOLD 10.0      // Stop when within 10mm of target
#define TRAVEL_SPEED 0.3           // Base forward speed (counts/ms)
#define HEADING_CORRECTION_GAIN 0.5  // Gain for heading correction during travel

// Combined travelToXY behavior
enum TravelState {
  TRAVEL_IDLE,       // Not traveling
  TRAVEL_ROTATING,   // Initial rotation to face target
  TRAVEL_MOVING,     // Moving toward target
  TRAVEL_COMPLETE    // Reached target
};
TravelState travel_state = TRAVEL_IDLE;

// ============================================
// STAGE 6: FSM STATES & TIMER
// ============================================

// Robot behaviour states
enum RobotState {
  STATE_TRAVEL_TO_WAYPOINT,  // Navigating to the current puck waypoint
  STATE_AT_WAYPOINT,         // Arrived – brief pause (Stage 9 adds puck check here)
  STATE_REVERSING,           // Safety: backing up after hitting boundary during travel
  STATE_TURNING,             // Safety: random turn after reverse, then resumes travel
  STATE_RETURN_HOME,         // Timer expired: navigating back to start (0,0)
  STATE_STOPPED              // Arrived home, run complete
};
RobotState robot_state = STATE_TRAVEL_TO_WAYPOINT;

// 4-minute countdown timer.
// Started in setup() AFTER line-sensor calibration so the 3 s
// calibration spin does not eat into the 4-minute run.
#define TIMER_DURATION_MS 240000UL   // 4 minutes in ms
unsigned long timer_start_ms = 0;    // Set in setup()

// Serial countdown print (updates once per second)
unsigned long countdown_print_ts = 0;

// Shared timestamp for timed manoeuvres (reverse / turn)
unsigned long maneuver_ts = 0;

// --- Reversing parameters ---
#define REVERSE_DURATION 400    // How long to reverse (ms)
#define REVERSE_SPEED -0.3      // Reverse speed (counts/ms, negative)

// --- Turning parameters ---
#define MIN_TURN_DURATION 300   // Shortest possible turn (ms)
#define MAX_TURN_DURATION 1000  // Longest  possible turn (ms)
#define TURN_SPEED 0.4          // Wheel speed during turn (counts/ms)
unsigned long turn_duration = 0;  // Chosen randomly each turn
int turn_direction = 1;           // +1 = left, -1 = right

// ============================================
// STAGE 8: WAYPOINT NAVIGATION
// ============================================
// Number of puck locations extracted from Map.svg
#define NUM_WAYPOINTS 6

// Puck locations in the robot coordinate frame (mm).
// Origin = start-area centre; robot faces +X at launch.
// Coordinates extracted from Map.svg by chaining nested
// group translate() transforms.
float waypoint_x[NUM_WAYPOINTS] = {  60.6, 260.7, 429.0, 408.8, 354.3, 156.7 };
float waypoint_y[NUM_WAYPOINTS] = { 262.9, 211.4, 267.7, 113.9, -10.9,  83.9 };

// Visit order: indices into waypoint arrays.
// Sweep order roughly minimises total travel distance.
int waypoint_order[NUM_WAYPOINTS] = { 0, 1, 2, 3, 4, 5 };

// Index into waypoint_order for the waypoint currently being visited
int current_waypoint_idx = 0;

// How long (ms) to pause at each waypoint before moving on.
// Leaves time for future puck-detection (Stage 9).
#define WAYPOINT_PAUSE_MS 1000

// The setup() function runs only once when the
// robot is powered up (either by plugging in
// the USB cable, or activating the motor power.
// Use this function to do "once only" setup 
// and configuration of your robot.
void setup() {
  // Setup up the buzzer as an output
  pinMode( BUZZER_PIN, OUTPUT );

  // Setup button A as input with internal pull-up resistor
  // Button reads LOW when pressed, HIGH when released
  pinMode( BUTTON_A_PIN, INPUT_PULLUP );

  // Setup motors.  This is calling a function
  // from within the Motors_c class. You can 
  // review this inside Motors.h
  // Complete by working through Labsheet 1.
  motors.initialise();

  // Setup the line sensors.  This is calling a 
  // function from within the LineSensors_c 
  // class. You can review this inside 
  // LineSensors.h.  
  // Complete by working through Labsheet 2.
  line_sensors.initialiseForADC();

  // These two functions are in encoders.h and
  // they activate the encoder sensors to 
  // measure the wheel rotation.  You do not 
  // need to change or update these.
  // These are used in Labsheet 4.
  // Encoder counts are counted automatically.
  setupEncoder0();
  setupEncoder1();

  // Setup the pose (kinematics). This is calling 
  // a function within the Kinematics_c class.
  // You can review this within Kinematics.h
  // This is used in Labsheet 4.
  pose.initialise(0, 0, 0);

  // Activates the Serial port, and the delay
  // is used to wait for the connection to be
  // established.
  Serial.begin(9600);
  delay(2000);
  Serial.println(" *** READY *** ");

  // If you are using an OLED or LCD display,
  // the following will set up a count down
  // timer to be displayed.
  // display.setMaxMinutes(4);
  // display.startStopwatch();

  // ============================================
  // WAIT FOR BUTTON A PRESS TO START
  // ============================================
  // All hardware is now initialised. Wait here
  // so you can position the robot on the
  // calibration disk before anything moves.
  Serial.println();
  Serial.println("=================================");
  Serial.println("  Press BUTTON A to start robot");
  Serial.println("=================================");

  while (digitalRead(BUTTON_A_PIN) != LOW) {
    // Wait for button press
  }
  // Debounce: wait for button release
  delay(50);
  while (digitalRead(BUTTON_A_PIN) == LOW) {}
  delay(50);

  tone(BUZZER_PIN, 440, 100);  // Short beep to confirm start
  Serial.println("*** ROBOT STARTED ***");

  // ============================================
  // STAGE 3: Initialize speed measurement
  // ============================================
  // Capture initial encoder counts
  prev_count_e0 = count_e0;
  prev_count_e1 = count_e1;

  // Initialize speeds to zero (robot starts stationary)
  speed_e0 = 0.0;
  speed_e1 = 0.0;

  // Capture starting timestamp
  speed_measurement_ts = millis();

  Serial.println("Stage 3: Speed measurement initialized");

  // ============================================
  // STAGE 4: Initialize PID Controllers
  // ============================================

  // Starting gains for PID tuning
  // BEGIN WITH P-ONLY CONTROL: Kp = 100, Ki = 0.0, Kd = 0.0
  // These are initial guesses and MUST be tuned physically
  // NOTE: Kp must be large because speed is in counts/ms (small values)
  //       but PWM needs to be 20-255 (large values)
  float initial_Kp = 30.0;  // Start with proportional control only
  float initial_Ki = 0.5;    // Integral term disabled initially
  float initial_Kd = 0.0;    // Derivative term disabled initially

  // Initialize left motor PID
  left_pid.initialise(initial_Kp, initial_Ki, initial_Kd);

  // Initialize right motor PID
  right_pid.initialise(initial_Kp, initial_Ki, initial_Kd);

  // Initialize demand speeds to zero (stopped)
  demand_speed_left = 0.0;
  demand_speed_right = 0.0;

  // PID starts disabled for safety
  // Enable by setting use_pid_control = true in your code
  use_pid_control = false;

  // Capture starting timestamp for PID updates
  pid_update_ts = millis();

  Serial.println("Stage 4: PID controllers initialized");
  Serial.print("  Kp=");
  Serial.print(initial_Kp);
  Serial.print(", Ki=");
  Serial.print(initial_Ki);
  Serial.print(", Kd=");
  Serial.println(initial_Kd);
  Serial.println("  NOTE: Kp is large because speed units (counts/ms) are small");
  Serial.println("  PID control: DISABLED (use_pid_control=false)");
  Serial.println("  To enable: set use_pid_control=true in code");

  // ============================================
  // STAGE 5: Line Sensor Calibration
  // ============================================
  // The robot will rotate on the spot over black and white surfaces
  // to learn the min/max sensor values for calibration
  //
  // IMPORTANT: Place robot on the calibration disk (alternating
  // black/white pattern at start position) before powering on!
  //
  // Uncomment the following line to enable calibration at startup:
  //calibrateLineSensors();

  Serial.println("Stage 5: Line sensors initialized");

  // Re-initialise pose AFTER the calibration spin so the 3 s
  // of rotation does not corrupt the starting odometry.
  pose.initialise(0, 0, 0);

  // ============================================
  // STAGE 7: Initialize Kinematics
  // ============================================
  // Capture starting timestamp for pose updates
  pose_update_ts = millis();

  Serial.println("Stage 7: Kinematics initialized");
  Serial.println("  Call pose.update() every 20ms for odometry");
  Serial.println("  Navigation functions: setTurn(), checkTurn(), travelToXY()");
  Serial.println("  IMPORTANT: Calibrate wheel_radius and wheel_sep in Kinematics.h");

  // ============================================
  // STAGE 6: Start FSM & Timer
  // ============================================
  // Seed RNG (used for random turn durations / directions)
  randomSeed(analogRead(A0) + millis());

  // Enable PID closed-loop speed control for all driving
  use_pid_control = true;

  // Start the 4-minute countdown.  Placed last so the
  // calibration spin (3 s) does not count against the run.
  timer_start_ms = millis();

  setTurn(PI / 2);  // Example: set a turn of 90 degrees (π/2 radians)

}

// ============================================
// STAGE 5: LINE SENSOR CALIBRATION FUNCTION
// ============================================
// This function rotates the robot on the spot while collecting
// sensor readings to determine min/max values for each sensor.
// Call this once at startup with robot on calibration disk.
void calibrateLineSensors() {

  Serial.println("Starting line sensor calibration...");
  Serial.println("Robot will rotate for 3 seconds.");

  // Reset calibration to prepare for new values
  line_sensors.resetCalibration();

  // Start rotating on the spot (left wheel backward, right wheel forward)
  // Use low speed for better calibration
  motors.setPWM(-40, 40);

  // Calibration duration in milliseconds
  unsigned long calibration_duration = 3000;
  unsigned long start_time = millis();

  // Collect readings while rotating
  while (millis() - start_time < calibration_duration) {
    // Update calibration with new readings
    line_sensors.updateCalibration();

    // Small delay between readings
    delay(10);
  }

  // Stop motors
  motors.setPWM(0, 0);

  // Calculate final calibration values (scaling = range)
  line_sensors.finalizeCalibration();

  // Mark as calibrated
  line_sensors_calibrated = true;

  // Report calibration results
  Serial.println("Calibration complete!");
  Serial.println("Sensor calibration values:");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("  Sensor ");
    Serial.print(i);
    Serial.print(": min=");
    Serial.print(line_sensors.minimum[i]);
    Serial.print(", max=");
    Serial.print(line_sensors.maximum[i]);
    Serial.print(", range=");
    Serial.println(line_sensors.scaling[i]);
  }

  // Short delay before continuing
  delay(500);
}


// Function to measure and calculate wheel speeds
// This is non-blocking and should be called regularly from loop()
void measureSpeeds() {

  // Check if it's time to measure speed (every 20ms)
  unsigned long current_time = millis();
  unsigned long elapsed_time = current_time - speed_measurement_ts;

  if (elapsed_time >= SPEED_MEASUREMENT_INTERVAL) {

    // Calculate change in encoder counts since last measurement
    long delta_e0 = count_e0 - prev_count_e0;
    long delta_e1 = count_e1 - prev_count_e1;

    // Save current counts for next measurement
    prev_count_e0 = count_e0;
    prev_count_e1 = count_e1;

    // Calculate speed as counts per millisecond
    // Typecast to float for precision
    // Speed = change in counts / change in time
    speed_e0 = (float)delta_e0 / (float)elapsed_time;
    speed_e1 = (float)delta_e1 / (float)elapsed_time;

    // Update timestamp for next measurement
    speed_measurement_ts = current_time;

    // Optional: Print speeds for debugging (comment out if not needed)
    // Serial.print("L:");
    // Serial.print(speed_e0, 4);
    // Serial.print(",R:");
    // Serial.println(speed_e1, 4);
  }
}

// ============================================
// STAGE 4: PID CONTROL FUNCTION
// ============================================

/*
 * Update PID controllers and apply motor commands
 *
 * This function should be called regularly from loop()
 * It runs every 50ms to give PID controllers time to respond
 *
 * When use_pid_control is true:
 *   - PID controllers calculate PWM based on speed error
 *   - Motors are controlled to achieve demand speeds
 *
 * When use_pid_control is false:
 *   - Direct PWM control (Stage 2 behavior)
 *   - PID controllers are reset to prevent integral windup
 */
void updatePID() {

  // Check if it's time to update PID (every 50ms)
  unsigned long current_time = millis();
  unsigned long elapsed_time = current_time - pid_update_ts;

  if (elapsed_time >= PID_UPDATE_INTERVAL) {

    if (use_pid_control) {
      // ============================================
      // PID CLOSED-LOOP CONTROL ENABLED
      // ============================================

      // Update left motor PID
      // Inputs: demand speed, measured speed
      // Output: PWM value to apply to motor
      // NOTE: Encoder mapping from Encoders.h:
      //   count_e0 / speed_e0 = RIGHT wheel
      //   count_e1 / speed_e1 = LEFT wheel
      left_pwm_output = left_pid.update(demand_speed_left, speed_e1);   // Use e1 for left motor

      // Update right motor PID
      right_pwm_output = right_pid.update(demand_speed_right, speed_e0); // Use e0 for right motor

      // Apply PID outputs to motors
      motors.setPWM(left_pwm_output, right_pwm_output);

      // Debug output for tuning
      // NOTE: speed_e1 = LEFT, speed_e0 = RIGHT (from Encoders.h)
      //Serial.print("DemL:");
      //Serial.print(demand_speed_left, 4);
      //Serial.print(",MeasL:");
      //Serial.print(speed_e1, 4);           // Corrected: e1 is left
      //Serial.print(",PWML:");
      //Serial.print(left_pwm_output, 1);
      //Serial.print(" | DemR:");
      //Serial.print(demand_speed_right, 4);
      //Serial.print(",MeasR:");
      //Serial.print(speed_e0, 4);           // Corrected: e0 is right
      //Serial.print(",PWMR:");
      //Serial.println(right_pwm_output, 1);

    } else {
      // ============================================
      // PID DISABLED - Direct PWM control
      // ============================================

      // Reset PID controllers to prevent integral windup
      // This ensures clean startup when PID is re-enabled
      left_pid.reset();
      right_pid.reset();

      // Direct PWM control is handled elsewhere in code
      // (e.g., motor test states or manual commands)
    }

    // Update timestamp for next PID cycle
    pid_update_ts = current_time;
  }
}

/*
 * Set target speeds for both motors
 *
 * @param left_speed: Target speed for left wheel (counts/ms)
 * @param right_speed: Target speed for right wheel (counts/ms)
 *
 * Note: This only has effect when use_pid_control = true
 */
void setDemandSpeeds(float left_speed, float right_speed) {
  demand_speed_left = left_speed;
  demand_speed_right = right_speed;
}

/*
 * Convenience function: Set same speed for both motors (straight line)
 *
 * @param speed: Target speed for both wheels (counts/ms)
 */
void setDemandSpeed(float speed) {
  setDemandSpeeds(speed, speed);
}

/*
 * Stop motors smoothly using PID
 * Sets demand speeds to zero
 */
void stopWithPID() {
  setDemandSpeeds(0.0, 0.0);
}

// ============================================
// STAGE 5: LINE SENSOR HELPER FUNCTIONS
// ============================================

/*
 * Check if any line sensor detects a black line
 * Uses calibrated readings if calibration was performed
 *
 * @return true if any sensor detects black line, false otherwise
 */
bool isOnLine() {
  if (line_sensors_calibrated) {
    // Use calibrated isOnLine with threshold
    return line_sensors.isOnLine(LINE_THRESHOLD);
  } else {
    // Without calibration, use raw readings with typical threshold
    // Raw ADC values: ~600-700 for white, ~900-1000 for black (typical)
    line_sensors.readSensorsADC();
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (line_sensors.readings[i] > 800) {  // Typical threshold for uncalibrated
        return true;
      }
    }
    return false;
  }
}

/*
 * Check if a specific sensor detects the line
 *
 * @param sensor_index: 0=DN1(left), 1=DN2, 2=DN3(center), 3=DN4, 4=DN5(right)
 * @return true if specified sensor is on line
 */
bool isSensorOnLine(int sensor_index) {
  if (line_sensors_calibrated) {
    return line_sensors.isSensorOnLine(sensor_index, LINE_THRESHOLD);
  } else {
    line_sensors.readSensorsADC();
    if (sensor_index >= 0 && sensor_index < NUM_SENSORS) {
      return line_sensors.readings[sensor_index] > 800;
    }
    return false;
  }
}

/*
 * Get pattern of which sensors are on the line
 * Useful for determining line position relative to robot
 *
 * @return bitmask: bit 0=DN1(left), bit 1=DN2, bit 2=DN3(center), etc.
 *         Example: 0b00100 (=4) means only center sensor on line
 *         Example: 0b00001 (=1) means only left sensor on line
 *         Example: 0b10000 (=16) means only right sensor on line
 */
byte getLineSensorPattern() {
  if (line_sensors_calibrated) {
    return line_sensors.getLineSensorPattern(LINE_THRESHOLD);
  } else {
    line_sensors.readSensorsADC();
    byte pattern = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (line_sensors.readings[i] > 800) {
        pattern |= (1 << i);
      }
    }
    return pattern;
  }
}

/*
 * Print line sensor readings for debugging
 * Prints calibrated values if calibrated, raw values otherwise
 */
void printLineSensorReadings() {
  if (line_sensors_calibrated) {
    line_sensors.calcCalibratedADC();
    Serial.print("Cal: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(line_sensors.calibrated[i], 2);
      if (i < NUM_SENSORS - 1) Serial.print(", ");
    }
  } else {
    line_sensors.readSensorsADC();
    Serial.print("Raw: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(line_sensors.readings[i], 0);
      if (i < NUM_SENSORS - 1) Serial.print(", ");
    }
  }
  Serial.println();
}

// ============================================
// STAGE 7: KINEMATICS UPDATE FUNCTION
// ============================================

/*
 * Update robot pose (x, y, theta) using encoder counts
 * MUST be called regularly at fixed interval (20ms recommended)
 * Call this from loop() after measureSpeeds()
 */
void updatePose() {
  unsigned long current_time = millis();
  unsigned long elapsed_time = current_time - pose_update_ts;

  if (elapsed_time >= POSE_UPDATE_INTERVAL) {
    pose.update();  // Update kinematics
    pose_update_ts = current_time;
  }
}

// ============================================
// STAGE 7: NAVIGATION HELPER FUNCTIONS
// ============================================

/*
 * Calculate the smallest angular difference between two angles
 * Handles wraparound at 0/2π boundary
 *
 * @param target_angle: Desired angle (radians)
 * @param current_angle: Current angle (radians)
 * @return: Smallest difference in radians, range [-π, π]
 *          Positive = turn left, Negative = turn right
 */
float angleDiff(float target_angle, float current_angle) {
  // Calculate raw difference
  float diff = target_angle - current_angle;

  // Normalize to [-π, π] using atan2 trick
  // This handles the 0°/360° wraparound elegantly
  diff = atan2(sin(diff), cos(diff));

  return diff;
}

/*
 * Calculate angle from current position to target (x,y)
 * Uses atan2 to handle all quadrants correctly
 *
 * @param target_x: Target X coordinate (mm)
 * @param target_y: Target Y coordinate (mm)
 * @return: Angle to target in radians
 */
float angleToPoint(float target_x, float target_y) {
  float dx = target_x - pose.x;
  float dy = target_y - pose.y;
  return atan2(dy, dx);
}

/*
 * Calculate distance from current position to target (x,y)
 *
 * @param target_x: Target X coordinate (mm)
 * @param target_y: Target Y coordinate (mm)
 * @return: Distance to target in mm
 */
float distanceToPoint(float target_x, float target_y) {
  float dx = target_x - pose.x;
  float dy = target_y - pose.y;
  return sqrt(dx*dx + dy*dy);
}

// ============================================
// STAGE 7: TURN BEHAVIOR
// ============================================

/*
 * Initialize a turn to a specific angle
 * Call this ONCE when you want to start a turn
 *
 * @param angle: Target orientation in radians
 */
void setTurn(float angle) {
  target_angle = angle;
  turning = true;
}

/*
 * Non-blocking turn update function
 * Call this repeatedly from loop() while turning
 *
 * @return: true if turn complete, false if still turning
 */
bool checkTurn() {
  if (!turning) return true;  // Not turning, return complete

  // Calculate angular difference (smallest angle)
  float angle_error = angleDiff(target_angle, pose.theta);

  // Check if turn is complete
  if (abs(angle_error) < TURN_THRESHOLD) {
    // Stop motors
    if (use_pid_control) {
      setDemandSpeeds(0.0, 0.0);
    } else {
      motors.setPWM(0, 0);
    }
    turning = false;
    return true;  // Turn complete
  }

  // Calculate turn speed proportional to error
  float turn_speed = angle_error * TURN_GAIN;

  // Apply turn speeds (opposite directions for rotation on spot)
  if (use_pid_control) {
    // Using PID speed control
    // Left wheel: positive speed turns left
    // Right wheel: negative speed turns right
    setDemandSpeeds(turn_speed, -turn_speed);
  } else {
    // Direct PWM control
    float turn_pwm = turn_speed;
    // Constrain to reasonable PWM range
    if (turn_pwm > 100) turn_pwm = 100;
    if (turn_pwm < -100) turn_pwm = -100;
    motors.setPWM(turn_pwm, -turn_pwm);
  }

  return false;  // Still turning
}

// ============================================
// STAGE 7: TRAVEL BEHAVIOR
// ============================================

/*
 * Initialize travel to a specific (x,y) coordinate
 * Robot will drive straight, making small heading corrections
 * Call this ONCE when you want to start traveling
 *
 * @param x: Target X coordinate (mm)
 * @param y: Target Y coordinate (mm)
 */
void setTravel(float x, float y) {
  target_x = x;
  target_y = y;
  traveling = true;
}

/*
 * Non-blocking travel update function
 * Call this repeatedly from loop() while traveling
 * Robot moves toward target while correcting heading
 *
 * @return: true if travel complete, false if still traveling
 */
bool checkTravel() {
  if (!traveling) return true;  // Not traveling, return complete

  // Calculate distance to target
  float distance = distanceToPoint(target_x, target_y);

  // Check if travel is complete
  if (distance < TRAVEL_THRESHOLD) {
    // Stop motors
    if (use_pid_control) {
      setDemandSpeeds(0.0, 0.0);
    } else {
      motors.setPWM(0, 0);
    }
    traveling = false;
    return true;  // Travel complete
  }

  // Calculate desired heading to target
  float desired_heading = angleToPoint(target_x, target_y);

  // Calculate heading error
  float heading_error = angleDiff(desired_heading, pose.theta);

  // Calculate base forward speed
  float forward_speed = TRAVEL_SPEED;

  // Calculate heading correction
  float heading_correction = heading_error * HEADING_CORRECTION_GAIN;

  // Apply differential speeds for heading correction
  float left_speed = forward_speed + heading_correction;
  float right_speed = forward_speed - heading_correction;

  if (use_pid_control) {
    setDemandSpeeds(left_speed, right_speed);
  } else {
    // Direct PWM control
    float left_pwm = left_speed * 100;  // Scale to PWM
    float right_pwm = right_speed * 100;
    // Constrain PWM
    if (left_pwm > 100) left_pwm = 100;
    if (left_pwm < -100) left_pwm = -100;
    if (right_pwm > 100) right_pwm = 100;
    if (right_pwm < -100) right_pwm = -100;
    motors.setPWM(left_pwm, right_pwm);
  }

  return false;  // Still traveling
}

// ============================================
// STAGE 7: COMBINED TRAVEL-TO-XY BEHAVIOR
// ============================================

/*
 * High-level navigation function
 * Rotates to face target, then travels to it
 * This is the recommended function for waypoint navigation
 *
 * Call this ONCE to start navigation:
 *   travelToXY(x, y);
 *
 * Then call updateTravelToXY() repeatedly from loop():
 *   updateTravelToXY();
 *
 * @param x: Target X coordinate (mm)
 * @param y: Target Y coordinate (mm)
 */
void travelToXY(float x, float y) {
  target_x = x;
  target_y = y;
  travel_state = TRAVEL_ROTATING;  // Start with rotation phase
}

/*
 * Non-blocking update for travelToXY behavior
 * Call this repeatedly from loop()
 *
 * @return: true if navigation complete, false otherwise
 */
bool updateTravelToXY() {
  switch (travel_state) {
    case TRAVEL_IDLE:
      // Not navigating
      return true;

    case TRAVEL_ROTATING:
      // Calculate angle to target
      float desired_angle = angleToPoint(target_x, target_y);

      // If not yet started turning, initialize turn
      if (!turning) {
        setTurn(desired_angle);
      }

      // Update turn
      if (checkTurn()) {
        // Turn complete, start traveling
        travel_state = TRAVEL_MOVING;
      }
      return false;

    case TRAVEL_MOVING:
      // If not yet started traveling, initialize travel
      if (!traveling) {
        setTravel(target_x, target_y);
      }

      // Update travel
      if (checkTravel()) {
        // Travel complete
        travel_state = TRAVEL_COMPLETE;
      }
      return false;

    case TRAVEL_COMPLETE:
      travel_state = TRAVEL_IDLE;
      return true;
  }

  return true;
}

// put your main code here, to run repeatedly:
void loop() {
  checkTurn();
}