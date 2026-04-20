/***************************************
 ,        .       .           .     ,--, 
 |        |       |           |       /  
 |    ,-: |-. ,-. |-. ,-. ,-. |-     `.  
 |    | | | | `-. | | |-' |-' |        ) 
 `--' `-` `-' `-' ' ' `-' `-' `-'   `-'  
***************************************/

// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include <math.h>

// These two commands mean that this header file
// will attempt to use some global variables of
// the same name found in another header file.
// From encoders.h
extern volatile long count_e0;
extern volatile long count_e1;

// Some global definitions concerning
// the robot dimensions.  You will need
// to calibrate these to get the best
// performance. (see Labsheet 4)
const float count_per_rev = 358.3;   // From documentation - correct.
const float wheel_radius  = 16.5;    // mm, could vary - calibrate.
const float wheel_sep     = 43.01;    // mm, from centre of robot to wheel centre 
                                     //     - could vary, calibrate

// Take the circumference of the wheel and divide by the 
// number of counts per revolution. This provides the mm
// travelled per encoder count.
const float mm_per_count  = ( 2.0 * wheel_radius * PI ) / count_per_rev;

// Class to track robot position.
class Kinematics_c {
  public:

    // Pose
    float x,y,theta;

    // To calculate the difference
    // in encoder counts for each
    // call to update()
    long last_e1;
    long last_e0;
  
    // Constructor, must exist.
    Kinematics_c() {

    } 

    // Used to setup kinematics, and to set a start position
    void initialise( float start_x, float start_y, float start_th ) {
      last_e0 = count_e0; // Initisalise last count to current count
      last_e1 = count_e1; // Initisalise last count to current count
      x = start_x;
      y = start_y;
      theta = start_th;
    }
    
    // Here I have opted to use encoder counts rather than 
    // wheel velocity.  Either way will work.  
    // With velocity, the difference in time between updates 
    // is required (distance = speed / time ).
    // If we use the velocity, it means we have to do
    // extra computation just to get back to distance, which
    // we had in the first place (as change of encoder counts)
    void update( ) {
      
        long delta_e1;  // change in counts
        long delta_e0;  // change in counts
        float mean_delta;
         
        float x_contribution;   // linear translation
        float th_contribution;  // rotation

        // Atomic read: disable interrupts while copying volatile
        // encoder counts to prevent ISR from corrupting a mid-read
        // on the 8-bit AVR (4-byte long is not atomic)
        long local_e1, local_e0;
        noInterrupts();
        local_e1 = count_e1;
        local_e0 = count_e0;
        interrupts();

        // How many counts since last update()?
        delta_e1 = local_e1 - last_e1;
        delta_e0 = local_e0 - last_e0;

        // Used last encoder values, so now update to
        // current for next iteration
        last_e1 = local_e1;
        last_e0 = local_e0;
        
        // Work out x contribution in local frame.
        mean_delta = (float)delta_e1;
        mean_delta += (float)delta_e0;
        mean_delta /= 2.0;

        x_contribution = mean_delta * mm_per_count;

        // Work out rotation in local frame
        th_contribution = (float)delta_e0;
        th_contribution -= (float)delta_e1;
        th_contribution *= mm_per_count;
        th_contribution /= (wheel_sep *2.0);


        // Mid-point Euler integration: use heading at the
        // midpoint of the timestep for doubled accuracy
        // during curved paths (essentially free vs forward Euler)
        float theta_mid = theta + (th_contribution / 2.0);
        x = x + x_contribution * cos( theta_mid );
        y = y + x_contribution * sin( theta_mid );
        theta = theta + th_contribution;

        // Normalize theta to [-PI, PI] to preserve float
        // precision over long runs
        while (theta > PI) theta -= 2.0 * PI;
        while (theta < -PI) theta += 2.0 * PI;

        // Done!
    } // End of update()

}; // End of Kinematics_c class defintion



#endif
