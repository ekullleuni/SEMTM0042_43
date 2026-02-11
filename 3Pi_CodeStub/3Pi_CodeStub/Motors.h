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

      // LEFT MOTOR POWER PROCESSING
      // Convert to absolute value (analogWrite needs positive only)
      left_pwr = abs( left_pwr );

      // Constrain to maximum safe PWM value
      if ( left_pwr > MAX_PWM ) {
        left_pwr = MAX_PWM;
      }

      // Ensure value is not negative after abs() operation
      // (redundant but defensive programming)
      if ( left_pwr < 0 ) {
        left_pwr = 0;
      }

      // RIGHT MOTOR POWER PROCESSING
      // Convert to absolute value (analogWrite needs positive only)
      right_pwr = abs( right_pwr );

      // Constrain to maximum safe PWM value
      if ( right_pwr > MAX_PWM ) {
        right_pwr = MAX_PWM;
      }

      // Ensure value is not negative after abs() operation
      // (redundant but defensive programming)
      if ( right_pwr < 0 ) {
        right_pwr = 0;
      }

      // Write the processed power values to the motors
      // This generates PWM signals to control motor speed
      analogWrite( L_PWM, left_pwr );
      analogWrite( R_PWM, right_pwr );

      // Done!
      return;

    } // End of setPWM()


}; // End of Motors_c class definition.



#endif
