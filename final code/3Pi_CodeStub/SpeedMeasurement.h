/*
 * SpeedMeasurement.h
 *
 * Helper functions for encoder reading and speed measurement
 * Stage 3: Encoder Reading & Speed Measurement
 *
 * This header provides utility functions for working with
 * encoder counts and wheel speeds.
 */

#ifndef _SPEED_MEASUREMENT_H
#define _SPEED_MEASUREMENT_H

// Include encoders for access to count_e0 and count_e1
#include "Encoders.h"

// ============================================
// CONSTANTS
// ============================================

// Encoder specifications for 3Pi+ robot
#define COUNTS_PER_MOTOR_REV 12.0        // Encoder counts per motor shaft revolution
#define GEAR_RATIO 29.86                 // Gearbox ratio (motor to wheel)
#define COUNTS_PER_WHEEL_REV 358.3       // Total counts per wheel revolution (12 × 29.86)

// Wheel physical dimensions (approximate - measure for accuracy)
#define WHEEL_DIAMETER_MM 32.0           // Wheel diameter in millimeters
#define WHEEL_CIRCUMFERENCE_MM 100.53    // Wheel circumference (π × diameter)

// ============================================
// UNIT CONVERSION FUNCTIONS
// ============================================

/*
 * Convert encoder counts to wheel rotations
 *
 * @param counts: Encoder count value
 * @return: Number of wheel revolutions
 */
float countsToRevolutions(long counts) {
    return (float)counts / COUNTS_PER_WHEEL_REV;
}

/*
 * Convert encoder counts to linear distance traveled (mm)
 *
 * @param counts: Encoder count value
 * @return: Distance in millimeters
 */
float countsToMM(long counts) {
    float revolutions = countsToRevolutions(counts);
    return revolutions * WHEEL_CIRCUMFERENCE_MM;
}

/*
 * Convert speed in counts/ms to counts/second
 *
 * @param counts_per_ms: Speed in encoder counts per millisecond
 * @return: Speed in counts per second
 */
float speedToCountsPerSecond(float counts_per_ms) {
    return counts_per_ms * 1000.0;
}

/*
 * Convert speed in counts/ms to revolutions/second (angular velocity)
 *
 * @param counts_per_ms: Speed in encoder counts per millisecond
 * @return: Angular velocity in revolutions per second
 */
float speedToRevPerSecond(float counts_per_ms) {
    float counts_per_sec = speedToCountsPerSecond(counts_per_ms);
    return counts_per_sec / COUNTS_PER_WHEEL_REV;
}

/*
 * Convert speed in counts/ms to mm/second (linear velocity)
 *
 * @param counts_per_ms: Speed in encoder counts per millisecond
 * @return: Linear velocity in millimeters per second
 */
float speedToMMPerSecond(float counts_per_ms) {
    float rev_per_sec = speedToRevPerSecond(counts_per_ms);
    return rev_per_sec * WHEEL_CIRCUMFERENCE_MM;
}

/*
 * Convert desired mm/second to required counts/ms (for PID demand)
 *
 * @param mm_per_second: Desired linear velocity in mm/s
 * @return: Equivalent speed in counts/ms
 */
float mmPerSecondToSpeed(float mm_per_second) {
    float rev_per_sec = mm_per_second / WHEEL_CIRCUMFERENCE_MM;
    float counts_per_sec = rev_per_sec * COUNTS_PER_WHEEL_REV;
    return counts_per_sec / 1000.0;
}

// ============================================
// ENCODER UTILITY FUNCTIONS
// ============================================

/*
 * Reset encoder counts to zero
 * Useful when starting a new motion sequence
 */
void resetEncoderCounts() {
    count_e0 = 0;
    count_e1 = 0;
}

/*
 * Get total distance traveled by left wheel since last reset
 *
 * @return: Distance in millimeters
 */
float getLeftWheelDistanceMM() {
    return countsToMM(count_e0);
}

/*
 * Get total distance traveled by right wheel since last reset
 *
 * @return: Distance in millimeters
 */
float getRightWheelDistanceMM() {
    return countsToMM(count_e1);
}

/*
 * Get average distance traveled by both wheels
 * Useful for estimating forward travel distance
 *
 * @return: Average distance in millimeters
 */
float getAverageDistanceMM() {
    float left_mm = getLeftWheelDistanceMM();
    float right_mm = getRightWheelDistanceMM();
    return (left_mm + right_mm) / 2.0;
}

// ============================================
// DEBUG FUNCTIONS
// ============================================

/*
 * Print encoder counts and speeds to Serial
 * Useful for debugging and calibration
 *
 * @param speed_e0: Left wheel speed (counts/ms)
 * @param speed_e1: Right wheel speed (counts/ms)
 */
void printEncoderDebug(float speed_e0, float speed_e1) {
    Serial.print("E0:");
    Serial.print(count_e0);
    Serial.print(" E1:");
    Serial.print(count_e1);
    Serial.print(" | L:");
    Serial.print(speed_e0, 4);
    Serial.print(" R:");
    Serial.println(speed_e1, 4);
}

/*
 * Print speeds in human-readable units
 *
 * @param speed_e0: Left wheel speed (counts/ms)
 * @param speed_e1: Right wheel speed (counts/ms)
 */
void printSpeedsHumanReadable(float speed_e0, float speed_e1) {
    Serial.print("Left: ");
    Serial.print(speedToMMPerSecond(speed_e0), 1);
    Serial.print(" mm/s | Right: ");
    Serial.print(speedToMMPerSecond(speed_e1), 1);
    Serial.println(" mm/s");
}

#endif
