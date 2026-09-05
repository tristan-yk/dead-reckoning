/**
  ******************************************************************************
  * @file           : bmp390.h
  * @brief          : BMP390 barometric pressure sensor, compensated to pascals.
  *
  * The EKF's barometric measurement model is p = p0 * (1 - L*h/T0)^k, so it
  * needs real pressure in pascals rather than the raw ADC counts. That means
  * reading the factory calibration out of the sensor's NVM at startup and
  * running Bosch's compensation formula on every sample.
  ******************************************************************************
  */

#ifndef BMP390_H
#define BMP390_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/* Reads the calibration coefficients and starts continuous measurement at
 * 25 Hz, matching the rate the filter consumes barometer data. */
bool bmp390_init(void);

/* Returns true only when the sensor has produced a new sample since the last
 * call, in which case *pressure_pa and *temperature_c are updated. Returns
 * false, leaving both untouched, when no new conversion is ready - the caller
 * uses this directly as the measurement's status flag. */
bool bmp390_read(float *pressure_pa, float *temperature_c);

#ifdef __cplusplus
}
#endif

#endif /* BMP390_H */
