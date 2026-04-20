/************************************
,        .       .           .      ,
|        |       |           |     '|
|    ,-: |-. ,-. |-. ,-. ,-. |-     |
|    | | | | `-. | | |-' |-' |      |
`--' `-` `-' `-' ' ' `-' `-' `-'    '
*************************************/

// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _MOTORS_H
#define _MOTORS_H

/*
 * MOTOR CALIBRATION DATA - Measured 28 Jan 2026
 *
 * Dead-band (minimum PWM for movement):
 *   Left motor forward:   +15 PWM
 *   Left motor backward:  -14 PWM
 *   Right motor forward:  +13 PWM
 *   Right motor backward: -12 PWM
 *
 * Direction: Verified
 *   Positive PWM = Forward motion
 *   Negative PWM = Backward motion
 *
 * Motor Bias (at PWM=50):
 *   Robot drifts ~13° left over 280mm travel
 *   Right motor slightly faster than left
 *   PID control (Stage 4) will compensate for this
 *
 * Note: Dead-band means PWM values below these thresholds
 * will not produce any wheel rotation due to motor friction.
 */

// Pin definitions.  By using #define we can
// switch the number here, and everywhere the
// text appears (i.e. L_PWM) it will be
// replaced.
// Pin assignments verified against Pololu 3Pi+ 32U4 documentation
// https://www.pololu.com/docs/0J83/5.9
#define L_PWM 10  // Left motor PWM (speed control)
#define L_DIR 16  // Left motor direction
#define R_PWM 9   // Right motor PWM (speed control)
#define R_DIR 15  // Right motor direction

// Direction constants for motor control
// NOTE: These may need adjustment during physical testing (Step 5)
// If positive PWM causes backward motion, swap FWD and REV values
#define FWD LOW   // Forward direction
#define REV HIGH  // Reverse direction

// It is a good idea to limit the maximum power
// sent to the motors. Using #define means we
// can set this value just once here, and it
// can be used in many places in the code below.
// This prevents motor damage and makes control more predictable
#define MAX_PWM 180.0

// Dead-band compensation: minimum PWM to produce movement.
// Any non-zero command below these values just heats the motor
// without turning the wheel.  Adding this offset linearises
// the PID output → actual speed relationship.
// Measured 28 Jan 2026; re-measure if motors/gears change.
#define L_DEADBAND_FWD 13   // Left motor forward dead-band
#define L_DEADBAND_REV 11   // Left motor reverse dead-band
#define R_DEADBAND_FWD 9   // Right motor forward dead-band
#define R_DEADBAND_REV 10   // Right motor reverse dead-band

// Class to operate the motors.
class Motors_c {

  public:

    // Constructor, must exist.
    Motors_c() {
      // Leave empty. Ensure initialise() is called
      // instead.
    }

    // Use this function to initialise the pins that
    // will control the motors, and decide what first
    // value they should have.
    void initialise() {

      // Configure all motor pins as outputs
      // PWM pins control motor speed (0-255)
      pinMode( L_PWM, OUTPUT );
      pinMode( R_PWM, OUTPUT );

      // Direction pins control motor direction (HIGH/LOW)
      pinMode( L_DIR, OUTPUT );
      pinMode( R_DIR, OUTPUT );

      // Set initial direction to forward
      // This establishes a known starting state
      digitalWrite( L_DIR, FWD );
      digitalWrite( R_DIR, FWD );

      // Start with motors OFF for safety
      // Robot should not move immediately on power-up
      analogWrite( L_PWM, 0 );
      analogWrite( R_PWM, 0 );

    } // End of initialise()


    // This function will be used to send a power value
    // to the motors.
    //
    // The power sent to the motors is created by the
    // analogWrite() function, which is producing PWM.
    // analogWrite() is intended to use a range between
    // [0:255].
    //
    // This function takes two input arguments: "left_pwr"
    // and "right_pwr", (pwr = power) and they are of the
    // type float. A float might be a value like 0.01, or
    // -150.6
    //
    // Positive values = forward motion
    // Negative values = backward motion
    void setPWM( float left_pwr, float right_pwr ) {

      // LEFT MOTOR DIRECTION
      // Set direction based on sign of power value
      if ( left_pwr < 0 ) {
        // Negative power = reverse direction
        digitalWrite( L_DIR, REV );
      } else {
        // Positive power = forward direction
        digitalWrite( L_DIR, FWD );
      }

      // RIGHT MOTOR DIRECTION
      // Set direction based on sign of power value
      if ( right_pwr < 0 ) {
        // Negative power = reverse direction
        digitalWrite( R_DIR, REV );
      } else {
        // Positive power = forward direction
        digitalWrite( R_DIR, FWD );
      }

      // POWER PROCESSING
      // Convert to absolute values (analogWrite needs positive only)
      // and apply dead-band compensation so small PID outputs
      // actually produce wheel rotation.
      float abs_left  = abs( left_pwr );
      float abs_right = abs( right_pwr );

      // Dead-band compensation: if the command is non-zero,
      // shift it up by the dead-band offset so the motor
      // always receives enough PWM to turn.
      // A zero command stays zero (motors off).
      if ( abs_left > 0 ) {
        // Pick dead-band based on direction already set above
        float db = ( left_pwr < 0 ) ? L_DEADBAND_REV : L_DEADBAND_FWD;
        abs_left = abs_left + db;
      }
      if ( abs_right > 0 ) {
        float db = ( right_pwr < 0 ) ? R_DEADBAND_REV : R_DEADBAND_FWD;
        abs_right = abs_right + db;
      }

      // Constrain to maximum safe PWM value
      if ( abs_left  > MAX_PWM ) abs_left  = MAX_PWM;
      if ( abs_right > MAX_PWM ) abs_right = MAX_PWM;

      // Write the processed power values to the motors
      // This generates PWM signals to control motor speed
      analogWrite( L_PWM, (int)abs_left );
      analogWrite( R_PWM, (int)abs_right );

      // Done!
      return;

    } // End of setPWM()


}; // End of Motors_c class definition.



#endif
