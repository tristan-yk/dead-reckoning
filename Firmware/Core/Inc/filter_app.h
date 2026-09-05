/**
  ******************************************************************************
  * @file           : filter_app.h
  * @brief          : The firmware's equivalent of filter_caller.m.
  *
  * Code generation for this project stops at filter_entry. Everything
  * filter_caller.m does around that call - holding the persistent state,
  * rotating measurements from sensor axes into body axes, seeding the
  * initialisation lowpass, and deriving dt - is simulation-side MATLAB and has
  * to be reproduced here.
  *
  * Note that filter_entry's is_init argument reads backwards: it runs
  * filter_init when the flag is false and filter_loop when it is true. It means
  * "past initialisation", so CALIBRATING passes false and RUNNING passes true.
  ******************************************************************************
  */

#ifndef FILTER_APP_H
#define FILTER_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "sensors.h"

typedef struct {
  float q[4];    /* attitude quaternion, body to nav, scalar first */
  float bias[3]; /* gyro bias, rad/s */
  /* The vertical channel, all positive UP, matching the filter's own states, so
   * within this struct d(h)/dt is v_z and d(v_z)/dt is a_z. Nothing is negated
   * on the way out. */
  float a_z; /* vertical acceleration, m/s^2, positive up */
  float v_z; /* vertical velocity, m/s, positive up */
  float h;   /* altitude relative to the calibration point, m, positive up */

  /* Frobenius norm of P. filter_caller.m reports norm(P), the induced 2-norm,
   * which would mean an eigenvalue solve on every step; this is the cheap
   * upper bound and is only used as a convergence indicator. */
  float P_frobenius;

  float dt;             /* interval handed to the filter on the last step */
  uint32_t last_cycles; /* cost of the last filter_entry call, CPU cycles */
} filter_output_t;

/* Prepares the generated filter's persistent data and clears the state. */
void filter_app_init(void);

/* Returns to the same starting point filter_caller.m uses: identity attitude,
 * zero P, and an unseeded lowpass. Called when the application leaves RUNNING. */
void filter_app_reset(void);

/* Runs one filter step for the current tick. Does nothing until every sensor
 * has delivered at least one sample to seed the lowpass with. */
void filter_app_step(const sensors_t *sens, double time_s, bool ekf_active);

/* True once every sensor has produced a sample and the filter is stepping. */
bool filter_app_seeded(void);

const filter_output_t *filter_app_output(void);

/* Mean cost of filter_entry in CPU cycles, split by which measurement updates
 * ran on the step. filter_loop always runs the dynamics and, at 100 Hz, the
 * accelerometer update; the magnetometer and barometer only join on their own
 * slower cycles. Breaking the cost out this way shows which update is actually
 * consuming the tick budget. Index with FILTER_COST_*. */
#define FILTER_COST_BASE 0u /* dynamics + accelerometer only */
#define FILTER_COST_MAG  1u /* ... plus magnetometer */
#define FILTER_COST_BARO 2u /* ... plus barometer */
#define FILTER_COST_BOTH 3u /* ... plus both */
#define FILTER_COST_BUCKETS 4u

uint32_t filter_app_mean_cycles(unsigned bucket);
uint32_t filter_app_max_cycles(unsigned bucket);
void filter_app_clear_cost(void);

#ifdef __cplusplus
}
#endif

#endif /* FILTER_APP_H */
