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
#ifndef _LINESENSORS_H
#define _LINESENSORS_H

// We will use all 5 line sensors (DN1 - 5)
// and so define a constant here, rather than
// type '5' in lots of places.
#define NUM_SENSORS 5

// Pin definitions
// This time, we will use an array to store the
// pin definitions.  This is a bit like a list.
// This way, we can either loop through the
// list automatically, or we can ask for a pin
// by indexing, e.g. sensor_pins[0] is A11,
// sensors_pins[1] is A0.
const int sensor_pins[ NUM_SENSORS ] = { A11, A0, A2, A3, A4 };

// This is the pin used to turn on the infra-
// red LEDs.
#define EMIT_PIN   11


// Class to operate the linesensors.
class LineSensors_c {
  
  public:

    // Store your readings into this array.
    // You can then access these readings elsewhere
    // by using the syntax line_sensors.readings[n];
    // Where n is a value [0:4]
    float readings[ NUM_SENSORS ];

    // Variables to store calibration constants. 
    // Make use of these as a part of the exercises 
    // in labsheet 2.
    float minimum[ NUM_SENSORS ];
    float maximum[ NUM_SENSORS ];
    float scaling[ NUM_SENSORS ];

    // Variable to store the calculated calibrated
    // (corrected) readings. Needs to be updated via
    // a function call, which is completed in 
    // labsheet 2.
    float calibrated[ NUM_SENSORS ];

    // Constructor, must exist.
    LineSensors_c() {
      // leave this empty
    }

    // Refer to Labsheet 2: Approach 1
    // Use this function to setup the pins required
    // to perform an read of the line sensors using
    // the ADC.
    void initialiseForADC() {

      // Ensure that the IR LEDs are on
      // for line sensing
      pinMode( EMIT_PIN, OUTPUT );
      digitalWrite( EMIT_PIN, HIGH );

      // Configure the line sensor pins DN1, DN2, DN3, DN4, DN5.
      // INPUT_PULLUP enables internal pull-up resistor needed for
      // potential divider circuit (see Labsheet 2 theory section)
      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
        pinMode( sensor_pins[sensor], INPUT_PULLUP );
      }

    } // End of initialiseForADC()



    // Refer to Labsheet 2: Approach 1
    // This function reads all line sensors using the ADC
    // Readings are stored in the readings[] array
    void readSensorsADC() {

      // First, initialise the pins.
      initialiseForADC();

      // Read each sensor using analogRead()
      // ADC returns 10-bit value (0-1023)
      // Higher values = more reflection = white surface
      // Lower values = less reflection = black surface
      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
        readings[sensor] = analogRead( sensor_pins[sensor] );
      }

    } // End of readSensorsADC()

    

    // Use this function to apply the calibration values
    // that were captured in your calibration routine.
    // Returns normalized readings in range [0.0 : 1.0]
    // where 0.0 = white surface, 1.0 = black surface
    void calcCalibratedADC() {

      // Get latest readings (raw values)
      readSensorsADC();

      // Apply calibration values, store in calibrated[]
      // Formula: calibrated = (reading - minimum) / range
      // This normalizes to [0.0 : 1.0]
      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
        // Avoid divide by zero
        if( scaling[sensor] != 0 ) {
          calibrated[sensor] = (readings[sensor] - minimum[sensor]) / scaling[sensor];
        } else {
          calibrated[sensor] = 0;
        }
      }

    } // End of calcCalibratedADC()


    // Calibration routine - run this once at startup
    // Robot should be rotating over black and white surfaces
    // This captures min/max values for each sensor
    // Call this WHILE the robot is rotating, in a loop
    void updateCalibration() {

      // Get latest readings
      readSensorsADC();

      // Update min/max for each sensor
      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
        // Check for new minimum
        if( readings[sensor] < minimum[sensor] ) {
          minimum[sensor] = readings[sensor];
        }
        // Check for new maximum
        if( readings[sensor] > maximum[sensor] ) {
          maximum[sensor] = readings[sensor];
        }
      }
    }

    // Call this AFTER calibration rotation is complete
    // Calculates the scaling (range) values
    void finalizeCalibration() {
      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
        scaling[sensor] = maximum[sensor] - minimum[sensor];
      }
    }

    // Initialize calibration values before starting calibration
    // Sets min to very high value, max to very low value
    void resetCalibration() {
      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
        minimum[sensor] = 9999;  // Will be replaced by lower values
        maximum[sensor] = 0;     // Will be replaced by higher values
        scaling[sensor] = 1;     // Prevent divide by zero
      }
    }

    // Check if ANY sensor is detecting a black line
    // Returns true if any calibrated reading exceeds threshold
    // threshold should be between 0.0 and 1.0 (typically 0.5)
    bool isOnLine( float threshold ) {
      // Get calibrated readings
      calcCalibratedADC();

      // Check each sensor
      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
        if( calibrated[sensor] > threshold ) {
          return true;  // Found a sensor on the line
        }
      }
      return false;  // No sensor on line
    }

    // Check if a specific sensor is on the line
    // sensor_index: 0=DN1(left), 1=DN2, 2=DN3(center), 3=DN4, 4=DN5(right)
    bool isSensorOnLine( int sensor_index, float threshold ) {
      if( sensor_index < 0 || sensor_index >= NUM_SENSORS ) {
        return false;  // Invalid sensor index
      }
      // Get calibrated readings
      calcCalibratedADC();
      return calibrated[sensor_index] > threshold;
    }

    // Get which sensors are detecting the line
    // Returns bitmask: bit 0 = DN1, bit 1 = DN2, etc.
    // Example: 0b00100 means only center sensor (DN3) on line
    byte getLineSensorPattern( float threshold ) {
      calcCalibratedADC();
      byte pattern = 0;
      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
        if( calibrated[sensor] > threshold ) {
          pattern |= (1 << sensor);
        }
      }
      return pattern;
    }





  
    // Part of the Advanced Exercises for Labsheet 2
    void initialiseForDigital() {
      
      // Ensure that the IR LEDs are on
      // for line sensing
      pinMode( EMIT_PIN, OUTPUT );
      digitalWrite( EMIT_PIN, HIGH );

    } // End of initialiseForDigital()

    // Part of the Advanced Exercises for Labsheet 2
    void readSensorsDigital() {
    //  ???
    } // End of readSensorsDigital()

}; // End of LineSensor_c class defintion



#endif
