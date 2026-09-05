/**
  ******************************************************************************
  * @file           : sensors.h
  * @brief          : Sensor sampling in the shape the EKF expects.
  *
  * Mirrors the sens_in structure that filter_entry takes: a measurement and a
  * status flag per sensor. The status is edge triggered, meaning it is true
  * only on the one call that carries a genuinely new sample from that sensor
  * and false again until the next one arrives. That is what the filter is
  * built around - filter_loop gates each measurement update on the
  * corresponding status, so a status left permanently true would feed repeated
  * samples in as though they were independent measurements.
  *
  * Measurements here are in the sensors' own axes. The rotation into body axes
  * happens in the filter layer, matching where filter_caller.m does it.
  ******************************************************************************
  */

#ifndef SENSORS_H
#define SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef struct {
  float meas[3];
  bool status;
} sensor3_t;

typedef struct {
  float meas;
  bool status;
} sensor1_t;

typedef struct {
  sensor3_t accel; /* m/s^2      */
  sensor3_t gyro;  /* rad/s      */
  sensor3_t mag;   /* microtesla */
  sensor1_t baro;  /* pascals    */
} sensors_t;

bool sensors_init(void);

/* Samples every device and fills in the measurements that are new. A sensor
 * with status false keeps its previous measurement value, so the struct always
 * holds the most recent reading from each. */
void sensors_read(sensors_t *out);

/* Count of failed bus transactions since boot, for health reporting. */
unsigned long sensors_fault_count(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSORS_H */
