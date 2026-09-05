/**
  ******************************************************************************
  * @file           : filter_app.c
  * @brief          : The firmware's equivalent of filter_caller.m.
  ******************************************************************************
  */

#include "filter_app.h"

#include <math.h>
#include <string.h>

#include "app_tick.h"
#include "filter_entry.h"
#include "filter_entry_types.h"

/* Sensor axes to body axes, matching R_bs in filter_caller.m:
 *
 *     x_body = -y_sensor
 *     y_body = -x_sensor
 *     z_body = -z_sensor
 *
 * The z term is confirmed by measurement: at rest the accelerometer reads
 * gravity on +z_sensor, and the filter's body frame is NED, where a level board
 * must read -g on z. The x and y terms come from the simulation and depend on
 * which edge of the shield is treated as forward, which only changes the
 * heading reference, not whether the filter converges. */
static const float R_bs[3][3] = {
  { 0.0f, -1.0f,  0.0f},
  {-1.0f,  0.0f,  0.0f},
  { 0.0f,  0.0f, -1.0f},
};

/* The generated code keeps its persistent data behind a caller-owned pointer
 * rather than in static storage of its own. */
static filter_entryPersistentData persistent_data;
static filter_entryStackData stack_data;

static float state[10];
static float P[100];
static struct0_T mem;

static double t_old;
static bool seeded_accel;
static bool seeded_gyro;
static bool seeded_mag;
static bool seeded_baro;
static bool seeded;

static filter_output_t output;

/* Accumulated cost per measurement-update combination, so the tick budget can
 * be attributed to a specific update rather than to filter_loop as a whole. */
static struct {
  uint64_t total_cycles;
  uint32_t count;
  uint32_t max_cycles;
} cost[FILTER_COST_BUCKETS];

static void rotate_to_body(const float in[3], float out[3])
{
  for (int i = 0; i < 3; i++) {
    out[i] = (R_bs[i][0] * in[0]) +
             (R_bs[i][1] * in[1]) +
             (R_bs[i][2] * in[2]);
  }
}

void filter_app_init(void)
{
  stack_data.pd = &persistent_data;
  filter_entry_initialize(&stack_data);
  filter_app_reset();
}

void filter_app_reset(void)
{
  /* filter_caller.m starts from the identity quaternion with the rest of the
   * state and all of P at zero; filter_init then overwrites P with params.P0. */
  memset(state, 0, sizeof(state));
  state[0] = 1.0f;
  memset(P, 0, sizeof(P));
  memset(&mem, 0, sizeof(mem));

  seeded_accel = false;
  seeded_gyro = false;
  seeded_mag = false;
  seeded_baro = false;
  seeded = false;

  memset(&output, 0, sizeof(output));
  output.q[0] = 1.0f;
}

bool filter_app_seeded(void)
{
  return seeded;
}

void filter_app_step(const sensors_t *sens, double time_s, bool ekf_active)
{
  struct2_T sens_in;

  rotate_to_body(sens->accel.meas, sens_in.accel.meas);
  rotate_to_body(sens->gyro.meas, sens_in.gyro.meas);
  rotate_to_body(sens->mag.meas, sens_in.mag.meas);
  sens_in.accel.status = sens->accel.status;
  sens_in.gyro.status = sens->gyro.status;
  sens_in.mag.status = sens->mag.status;

  /* The barometer is a scalar and has no orientation to correct. */
  sens_in.baro.meas = sens->baro.meas;
  sens_in.baro.status = sens->baro.status;

  if (!seeded) {
    /* filter_caller.m seeds mem.sens_filt from the first measurement of each
     * sensor, deliberately taking it whatever the status says. In simulation
     * every signal already holds a value on the first call, but on hardware a
     * sensor that has not reported yet holds nothing - and seeding the
     * barometer with zero would set p0 = 0 and destroy the pressure model. So
     * each channel is seeded as its own first real sample arrives, and the
     * filter does not start stepping until all four have. */
    if (sens_in.accel.status && !seeded_accel) {
      memcpy(mem.sens_filt.accel, sens_in.accel.meas, sizeof(mem.sens_filt.accel));
      seeded_accel = true;
    }
    if (sens_in.gyro.status && !seeded_gyro) {
      memcpy(mem.sens_filt.gyro, sens_in.gyro.meas, sizeof(mem.sens_filt.gyro));
      seeded_gyro = true;
    }
    if (sens_in.mag.status && !seeded_mag) {
      memcpy(mem.sens_filt.mag, sens_in.mag.meas, sizeof(mem.sens_filt.mag));
      seeded_mag = true;
    }
    if (sens_in.baro.status && !seeded_baro) {
      mem.sens_filt.baro = sens_in.baro.meas;
      seeded_baro = true;
    }

    if (!(seeded_accel && seeded_gyro && seeded_mag && seeded_baro)) {
      return;
    }

    seeded = true;
    t_old = time_s;
  }

  /* dt is the time since the last gyroscope sample, not since the last call:
   * filter_caller.m only advances t_old when gyro.status is true, because
   * ekf_dynamics is the only consumer of dt and it only runs on a new rate
   * measurement. */
  double dt_s = time_s - t_old;
  if (sens_in.gyro.status) {
    t_old = time_s;
  }

  uint32_t start = app_cycles();
  filter_entry(&stack_data, state, P, &mem, (float)dt_s, &sens_in, ekf_active);
  output.last_cycles = app_cycles() - start;

  if (ekf_active) {
    unsigned bucket = (sens_in.mag.status ? FILTER_COST_MAG : 0u) |
                      (sens_in.baro.status ? FILTER_COST_BARO : 0u);
    cost[bucket].total_cycles += output.last_cycles;
    cost[bucket].count++;
    if (output.last_cycles > cost[bucket].max_cycles) {
      cost[bucket].max_cycles = output.last_cycles;
    }
  }

  for (int i = 0; i < 4; i++) {
    output.q[i] = state[i];
  }
  for (int i = 0; i < 3; i++) {
    output.bias[i] = state[4 + i];
  }
  /* The whole vertical channel is positive up and needs no sign correction on
   * the way out. Getting here took two wrong turns worth recording: the states
   * were briefly reported inverted, and a sign change was proposed for
   * ekf_innov_accel on the strength of a synthetic altitude profile. The real
   * fault was in the plant's barometer model, and once that was corrected the
   * accelerometer's a_nav sign followed, leaving a_z, v_z and h all agreeing.
   *
   * The lesson for anything that looks like a sign problem here: check it
   * against the Simulink plant rather than against a profile invented for the
   * purpose, because an invented one carries its own assumptions about which
   * way altitude counts. */
  output.a_z = state[7];
  output.v_z = state[8];
  output.h = state[9];

  output.dt = (float)dt_s;

  float sum_sq = 0.0f;
  for (int i = 0; i < 100; i++) {
    sum_sq += P[i] * P[i];
  }
  output.P_frobenius = sqrtf(sum_sq);
}

const filter_output_t *filter_app_output(void)
{
  return &output;
}

uint32_t filter_app_mean_cycles(unsigned bucket)
{
  if (bucket >= FILTER_COST_BUCKETS || cost[bucket].count == 0u) {
    return 0u;
  }
  return (uint32_t)(cost[bucket].total_cycles / cost[bucket].count);
}

uint32_t filter_app_max_cycles(unsigned bucket)
{
  if (bucket >= FILTER_COST_BUCKETS) {
    return 0u;
  }
  return cost[bucket].max_cycles;
}

void filter_app_clear_cost(void)
{
  memset(cost, 0, sizeof(cost));
}
