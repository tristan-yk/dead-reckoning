/**
  ******************************************************************************
  * @file           : sensors.c
  * @brief          : Sensor sampling in the shape the EKF expects.
  ******************************************************************************
  */

#include "sensors.h"

#include "bmp390.h"
#include "bno055.h"

static unsigned long fault_count;

bool sensors_init(void)
{
  bool imu_ok = bno055_init();
  bool baro_ok = bmp390_init();
  return imu_ok && baro_ok;
}

void sensors_read(sensors_t *out)
{
  bno055_sample_t imu;

  if (bno055_read(&imu)) {
    for (int i = 0; i < 3; i++) {
      if (imu.accel_new) {
        out->accel.meas[i] = imu.accel[i];
      }
      if (imu.gyro_new) {
        out->gyro.meas[i] = imu.gyro[i];
      }
      if (imu.mag_new) {
        out->mag.meas[i] = imu.mag[i];
      }
    }
    out->accel.status = imu.accel_new;
    out->gyro.status = imu.gyro_new;
    out->mag.status = imu.mag_new;
  } else {
    /* A failed read is not a new measurement. Leaving the statuses false means
     * the filter simply skips its updates this tick rather than acting on a
     * stale or partial sample. */
    out->accel.status = false;
    out->gyro.status = false;
    out->mag.status = false;
    fault_count++;
  }

  float pressure_pa = 0.0f;
  float temperature_c = 0.0f;
  if (bmp390_read(&pressure_pa, &temperature_c)) {
    out->baro.meas = pressure_pa;
    out->baro_temperature_c = temperature_c;
    out->baro.status = true;
  } else {
    out->baro.status = false;
  }
}

unsigned long sensors_fault_count(void)
{
  return fault_count;
}
