/**
  ******************************************************************************
  * @file           : bno055.h
  * @brief          : BNO055 in AMG mode - raw accelerometer, magnetometer and
  *                   gyroscope, with no on-chip sensor fusion.
  *
  * The EKF does its own attitude estimation, so running the BNO055 in a fusion
  * mode such as NDOF would feed the output of one filter into another. AMG mode
  * (OPR_MODE 0x07) leaves the three sensors raw and independent.
  *
  * Units are selected on the chip rather than scaled here where possible:
  * UNIT_SEL is configured so acceleration reads out in m/s^2 and angular rate
  * in rad/s, which is what the filter expects. Magnetometer output is always in
  * microtesla; the filter only uses the magnetic field's direction, so its
  * scale does not matter.
  ******************************************************************************
  */

#ifndef BNO055_H
#define BNO055_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef struct {
  float accel[3]; /* m/s^2   */
  float gyro[3];  /* rad/s   */
  float mag[3];   /* microtesla */

  /* True only when that sensor produced a sample this read differs from the
   * previous one. The three sensors run at their own output data rates behind
   * a single register block, so polling faster than a sensor's rate returns
   * the same sample again; these flags are what let the caller tell a fresh
   * measurement from a repeat. */
  bool accel_new;
  bool gyro_new;
  bool mag_new;
} bno055_sample_t;

/* Resets the device, configures the three sensors and enters AMG mode.
 * Returns false if the chip ID never appears or the mode does not stick. */
bool bno055_init(void);

/* Reads accelerometer, magnetometer and gyroscope in one 18-byte burst. The
 * three data blocks are contiguous on the device, so this costs one bus
 * transaction rather than three. Values are in the sensor's own axes. */
bool bno055_read(bno055_sample_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BNO055_H */
