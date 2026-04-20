/*************************************************************
 *  GradientCalibration.h
 *
 *  Stores calibration data collected while the robot drives
 *  across a printed grey-gradient strip and provides a
 *  per-sensor transfer function that converts a raw ADC reading
 *  into a lightness percentage.
 *
 *  Two interpolation methods are available, selected at runtime
 *  via the use_cubic_interp flag:
 *
 *    Piecewise-linear (default)
 *      Simple and fast.  With 25 calibration points each segment
 *      spans ~4 % of the range, so interpolation error is small.
 *
 *    Monotone cubic  (Fritsch-Carlson)
 *      Smoother curves with continuous first derivatives.
 *      Tangent slopes are computed on-the-fly per call (no extra
 *      RAM) which costs a few extra microseconds per lookup.
 *      Guaranteed monotone — will never overshoot between
 *      calibration points.
 *
 *  Lightness convention:
 *    0 % = pure black (darkest calibration shade)
 *  100 % = pure white (lightest calibration shade)
 *
 *  Usage
 *  -----
 *  1. In setup(), call  calibrateGradientSensors(25)   (defined in .ino)
 *     with the robot positioned over the first (black) square.
 *  2. Optionally set  use_cubic_interp = true;  for smoother curves.
 *  3. During normal operation, call
 *       line_sensors.readSensorsADC();
 *       float pct = gradientToPercent(sensor_idx, line_sensors.readings[sensor_idx]);
 *     to convert a raw ADC value to a lightness percentage.
 *
 *  Depends on: LineSensors.h  (NUM_SENSORS, readings[])
 *************************************************************/

#ifndef _GRADIENT_CALIBRATION_H
#define _GRADIENT_CALIBRATION_H

#include "LineSensors.h"

// ============================================================
// CONFIGURATION  — adjust to match your calibration strip
// ============================================================

// Maximum number of shade divisions the arrays can hold.
#define GRAD_CAL_MAX_DIVISIONS  25

// Number of ADC samples averaged at each shade to suppress noise.
// 30 samples x 5 ms gap = ~150 ms dwell time per shade.
#define GRAD_CAL_SAMPLES  30

// Distance the robot advances between consecutive shades (mm).
#define GRAD_CAL_STEP_MM  40.0f

// Encoder-to-distance conversion factor (counts per mm).
// From SpeedMeasurement.h: 358.3 / 100.53
#define GRAD_COUNTS_PER_MM  3.565f

// ============================================================
// INTERPOLATION MODE
// ============================================================

// Set to true for monotone-cubic (Fritsch-Carlson) interpolation.
// Set to false (default) for piecewise-linear interpolation.
bool use_cubic_interp = false;

// ============================================================
// CALIBRATION DATA STORAGE
// ============================================================

// Averaged raw ADC reading per shade per sensor.
// grad_cal_raw[division][sensor_index]
//   division 0 = blackest, division N-1 = lightest
//   sensor_index 0..4 = DN1..DN5
float grad_cal_raw[GRAD_CAL_MAX_DIVISIONS][NUM_SENSORS];

// Number of divisions actually recorded.
int   grad_cal_num_divisions = 0;

// True once calibration is complete.
bool  grad_cal_complete = false;

// Lightness % for a division is computed on-the-fly to save 100 bytes of RAM:
//   percent(i) = i * 100.0 / (num_divisions - 1)
// This inline helper replaces the old grad_cal_percent[] array.
static inline float grad_cal_pct(int i) {
  return (float)i * 100.0f / (float)(grad_cal_num_divisions - 1);
}


// ============================================================
// INTERNAL: Piecewise-linear interpolation
// ============================================================

/*
 * Convert raw ADC → lightness % using straight-line segments
 * between adjacent calibration points.
 */
float gradientToPercentLinear(int sensor_idx, float raw_reading) {

  float r_first = grad_cal_raw[0][sensor_idx];
  float r_last  = grad_cal_raw[grad_cal_num_divisions - 1][sensor_idx];

  // Clamp to calibrated range
  if (raw_reading <= r_first) return grad_cal_pct(0);
  if (raw_reading >= r_last)  return grad_cal_pct(grad_cal_num_divisions - 1);

  // Find bracket and interpolate
  for (int i = 0; i < grad_cal_num_divisions - 1; i++) {
    float r_lo = grad_cal_raw[i][sensor_idx];
    float r_hi = grad_cal_raw[i + 1][sensor_idx];

    if (raw_reading >= r_lo && raw_reading <= r_hi) {
      float range = r_hi - r_lo;
      if (range < 0.5f) {
        return (grad_cal_pct(i) + grad_cal_pct(i + 1)) * 0.5f;
      }
      float t = (raw_reading - r_lo) / range;
      return grad_cal_pct(i) + t * (grad_cal_pct(i + 1) - grad_cal_pct(i));
    }
  }

  return grad_cal_pct(grad_cal_num_divisions - 1);
}


// ============================================================
// INTERNAL: Monotone cubic (Fritsch-Carlson) interpolation
// ============================================================

/*
 * Compute the Fritsch-Carlson tangent slope for calibration
 * point i and the given sensor.  Tangents are computed on-the-fly
 * to avoid storing a [MAX_DIVISIONS][NUM_SENSORS] slopes array
 * (which would consume 500 bytes of precious SRAM).
 *
 * The algorithm:
 *   1. Compute secants delta[k] = (y[k+1]-y[k]) / (x[k+1]-x[k])
 *      for k = i-1 and k = i  (where x = raw ADC, y = percent).
 *   2. Initial tangent m = (delta[i-1] + delta[i]) / 2
 *      At endpoints:  m[0] uses only delta[0],
 *                     m[N-1] uses only delta[N-2].
 *   3. Monotonicity: if either adjacent secant is zero, m = 0.
 *      Otherwise clamp so alpha^2 + beta^2 <= 9 (Fritsch-Carlson
 *      condition), where alpha = m/delta_left, beta = m/delta_right.
 *
 * @param i           Calibration point index [0 .. N-1]
 * @param sensor_idx  Sensor [0 .. NUM_SENSORS-1]
 * @return            Tangent slope dy/dx at point i
 */
// Helper: compute secant (slope) between calibration points a and a+1
// for a given sensor.  x-axis = raw ADC, y-axis = lightness %.
static float _gcSecant(int a, int sensor_idx) {
  float dx = grad_cal_raw[a + 1][sensor_idx] - grad_cal_raw[a][sensor_idx];
  float dy = grad_cal_pct(a + 1) - grad_cal_pct(a);
  if (fabs(dx) < 0.001f) return 0.0f;
  return dy / dx;
}

float computeFCTangent(int i, int sensor_idx) {
  int N = grad_cal_num_divisions;

  // Endpoint tangents: use one-sided secant
  if (i == 0)     return _gcSecant(0, sensor_idx);
  if (i == N - 1) return _gcSecant(N - 2, sensor_idx);

  float d_left  = _gcSecant(i - 1, sensor_idx);   // secant to the left
  float d_right = _gcSecant(i, sensor_idx);       // secant to the right

  // If either adjacent secant is zero or they differ in sign,
  // set tangent to zero to preserve monotonicity.
  if (d_left * d_right <= 0.0f) return 0.0f;

  // Initial tangent: average of adjacent secants
  float m = (d_left + d_right) * 0.5f;

  // Fritsch-Carlson monotonicity enforcement:
  // alpha = m / d_left,  beta = m / d_right
  // Require alpha^2 + beta^2 <= 9
  float alpha = m / d_left;
  float beta  = m / d_right;
  float mag2  = alpha * alpha + beta * beta;

  if (mag2 > 9.0f) {
    // Scale m down so the condition is satisfied
    float tau = 3.0f / sqrt(mag2);
    m = tau * m;
  }

  return m;
}


/*
 * Convert raw ADC → lightness % using monotone cubic Hermite
 * interpolation with Fritsch-Carlson tangents.
 *
 * Within each segment [i, i+1] the Hermite basis functions give:
 *   p(t) = h00(t)*y_i + h10(t)*dx*m_i + h01(t)*y_{i+1} + h11(t)*dx*m_{i+1}
 * where t = (x - x_i) / (x_{i+1} - x_i),  dx = x_{i+1} - x_i,
 *   h00 = 2t^3 - 3t^2 + 1,  h10 = t^3 - 2t^2 + t,
 *   h01 = -2t^3 + 3t^2,     h11 = t^3 - t^2
 */
float gradientToPercentCubic(int sensor_idx, float raw_reading) {

  float r_first = grad_cal_raw[0][sensor_idx];
  float r_last  = grad_cal_raw[grad_cal_num_divisions - 1][sensor_idx];

  // Clamp to calibrated range
  if (raw_reading <= r_first) return grad_cal_pct(0);
  if (raw_reading >= r_last)  return grad_cal_pct(grad_cal_num_divisions - 1);

  // Find the segment that brackets raw_reading
  for (int i = 0; i < grad_cal_num_divisions - 1; i++) {
    float x0 = grad_cal_raw[i][sensor_idx];
    float x1 = grad_cal_raw[i + 1][sensor_idx];

    if (raw_reading >= x0 && raw_reading <= x1) {
      float dx = x1 - x0;

      // Degenerate segment: fall back to midpoint
      if (dx < 0.5f) {
        return (grad_cal_pct(i) + grad_cal_pct(i + 1)) * 0.5f;
      }

      // Normalised position within segment [0, 1]
      float t = (raw_reading - x0) / dx;

      // y values (lightness %) at segment endpoints
      float y0 = grad_cal_pct(i);
      float y1 = grad_cal_pct(i + 1);

      // Tangent slopes at segment endpoints (computed on-the-fly)
      float m0 = computeFCTangent(i, sensor_idx);
      float m1 = computeFCTangent(i + 1, sensor_idx);

      // Hermite basis polynomials
      float t2 = t * t;
      float t3 = t2 * t;
      float h00 =  2.0f * t3 - 3.0f * t2 + 1.0f;
      float h10 =  t3 - 2.0f * t2 + t;
      float h01 = -2.0f * t3 + 3.0f * t2;
      float h11 =  t3 - t2;

      // Evaluate the cubic polynomial
      float result = h00 * y0 + h10 * dx * m0 + h01 * y1 + h11 * dx * m1;

      // Safety clamp: ensure result stays within [y0, y1] even with
      // floating-point rounding.  Monotone tangents guarantee this
      // in theory, but belt-and-suspenders on an embedded system.
      if (y0 < y1) {
        if (result < y0) result = y0;
        if (result > y1) result = y1;
      } else {
        if (result > y0) result = y0;
        if (result < y1) result = y1;
      }

      return result;
    }
  }

  return grad_cal_pct(grad_cal_num_divisions - 1);
}


// ============================================================
// PUBLIC API: Transfer function with runtime method selection
// ============================================================

/*
 * Convert a single raw ADC reading to a lightness percentage.
 *
 * Dispatches to piecewise-linear or monotone-cubic interpolation
 * depending on the use_cubic_interp flag.
 *
 * @param sensor_idx   Sensor index [0 .. NUM_SENSORS-1]
 * @param raw_reading  Raw analogRead() value [0 .. 1023]
 * @return             Lightness % [0.0=black .. 100.0=white]
 *                     Returns -1.0 if calibration has not been run.
 */
float gradientToPercent(int sensor_idx, float raw_reading) {

  if (!grad_cal_complete || grad_cal_num_divisions < 2) return -1.0f;

  if (use_cubic_interp) {
    return gradientToPercentCubic(sensor_idx, raw_reading);
  } else {
    return gradientToPercentLinear(sensor_idx, raw_reading);
  }
}


/*
 * Compute the mean lightness across all five sensors.
 *
 * @param raw_readings  Array of NUM_SENSORS raw ADC values
 * @return              Mean lightness % [0.0 .. 100.0]
 *                      Returns -1.0 if calibration has not been run.
 */
float getAverageLightness(float raw_readings[NUM_SENSORS]) {
  if (!grad_cal_complete) return -1.0f;
  float sum = 0.0f;
  for (int i = 0; i < NUM_SENSORS; i++) {
    sum += gradientToPercent(i, raw_readings[i]);
  }
  return sum / (float)NUM_SENSORS;
}


#endif  // _GRADIENT_CALIBRATION_H
