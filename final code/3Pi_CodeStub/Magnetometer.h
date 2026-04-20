/***************************************
 ,        .       .           .     ,-.
 |        |       |           |        )
 |    ,-: |-. ,-. |-. ,-. ,-. |-      /
 |    | | | | `-. | | |-' |-' |      /
 `--' `-` `-' `-' ' ' `-' `-' `-'   '--'
****************************************/

// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _MAGNETOMETER_H
#define _MAGNETOMETER_H

#include <Wire.h>
#include <LIS3MDL.h>

#define MAG_AXIS 3

class Magnetometer_c {

  public:

    // LIS3MDL library instance for I2C communication
    LIS3MDL mag;

    // Raw readings from sensor (x, y, z)
    float readings[ MAG_AXIS ];

    // Calibration data: captured during rotation in setup()
    float minimum[ MAG_AXIS ];   // Min value seen per axis
    float maximum[ MAG_AXIS ];   // Max value seen per axis
    float offset[ MAG_AXIS ];    // Midpoint: (min + max) / 2
    float scaling[ MAG_AXIS ];   // Half-range: (max - min) / 2

    // Calibrated readings normalised to approx [-1.0 : +1.0]
    float calibrated[ MAG_AXIS ];

    // Whether calibration has been performed
    bool calibrated_flag;

    // Constructor - must NOT call Wire.begin() here
    // (would crash the microcontroller before setup runs)
    Magnetometer_c() {
      calibrated_flag = false;
    }

    // Call within setup() to initialise I2C and the sensor.
    // Returns false if sensor not found on bus.
    bool initialise() {
      Wire.begin();

      if ( !mag.init() ) {
        return false;
      }

      // CRITICAL: enableDefault() configures the sensor's
      // output data rate and full-scale range. Without this
      // call the readings will never update.
      mag.enableDefault();

      return true;
    } // End of initialise()

    // Fetch latest raw x, y, z values from the sensor
    // over I2C and store in readings[].
    // WARNING: Do not call more often than every 100 ms
    // or the sensor may stop responding until reset.
    void getReadings() {
      mag.read();
      readings[0] = mag.m.x;
      readings[1] = mag.m.y;
      readings[2] = mag.m.z;
    } // End of getReadings()

    // ================================================
    // CALIBRATION ROUTINES
    // Follow the same reset / update / finalize pattern
    // used by LineSensors_c.
    // ================================================

    // Call BEFORE starting calibration rotation.
    // Sets min to very high value, max to very low value
    // so they will be immediately replaced.
    void resetCalibration() {
      for ( int i = 0; i < MAG_AXIS; i++ ) {
        minimum[i] =  32767;   // Will be replaced by lower values
        maximum[i] = -32768;   // Will be replaced by higher values
        offset[i]  = 0;
        scaling[i] = 1;        // Prevent divide-by-zero
      }
    }

    // Call repeatedly WHILE the robot is rotating.
    // Each call reads the sensor and tracks per-axis min/max.
    void updateCalibration() {
      getReadings();

      for ( int i = 0; i < MAG_AXIS; i++ ) {
        if ( readings[i] < minimum[i] ) minimum[i] = readings[i];
        if ( readings[i] > maximum[i] ) maximum[i] = readings[i];
      }
    }

    // Call AFTER rotation is complete.
    // Computes offset (midpoint) and scaling (half-range)
    // used to normalise subsequent readings.
    void finalizeCalibration() {
      for ( int i = 0; i < MAG_AXIS; i++ ) {
        offset[i]  = ( minimum[i] + maximum[i] ) / 2.0;
        scaling[i] = ( maximum[i] - minimum[i] ) / 2.0;

        // Guard against zero range (axis saw no variation)
        if ( scaling[i] == 0 ) scaling[i] = 1.0;
      }
      calibrated_flag = true;
    }

    // ================================================
    // CALIBRATED READINGS & PUCK DETECTION
    // ================================================

    // Apply calibration to normalise each axis to [-1, +1].
    // Result stored in calibrated[].
    // Must call getReadings() first (or use the timed
    // wrapper in the .ino) to refresh raw data.
    void getCalibratedReadings() {
      for ( int i = 0; i < MAG_AXIS; i++ ) {
        calibrated[i] = ( readings[i] - offset[i] ) / scaling[i];
      }
    }

    // Compute the vector magnitude from calibrated readings.
    // Baseline (Earth field only) is ~1.0 after calibration.
    // A nearby magnetic puck raises this above 1.0.
    float getMagnitude() {
      getCalibratedReadings();
      float sum_sq = 0.0;
      for ( int i = 0; i < MAG_AXIS; i++ ) {
        sum_sq += calibrated[i] * calibrated[i];
      }
      return sqrt( sum_sq );
    }

    // Returns true when the calibrated magnitude exceeds
    // the given threshold, indicating a magnetic puck is
    // close enough to detect.
    //
    // Typical baseline: ~1.0
    // Typical detection range: puck within ~80 mm
    //
    // HUMAN CALIBRATION REQUIRED:
    //   1. Print getMagnitude() to Serial with no puck nearby
    //      to confirm baseline is ~1.0
    //   2. Place a puck at varying distances and note the
    //      magnitude readings
    //   3. Choose a threshold that reliably distinguishes
    //      "puck present" from "no puck"
    //   4. Update PUCK_MAGNITUDE_THRESHOLD in the .ino
    bool detectPuck( float threshold ) {
      float mag_val = getMagnitude();
      return ( mag_val > threshold );
    }

}; // End of Magnetometer_c class definition

#endif
