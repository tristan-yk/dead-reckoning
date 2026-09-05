/**
  ******************************************************************************
  * @file           : app_state.h
  * @brief          : Top-level mode of the dead-reckoning application.
  *
  *   IDLE  --CAL-->  CALIBRATING  --START-->  RUNNING
  *     ^                  |                      |
  *     +------ STOP ------+--------- STOP -------+
  *                        ^                      |
  *                        +--------- CAL --------+
  *
  * CALIBRATING runs filter_init, which low-pass filters the sensors to level
  * the attitude, seed the gyro bias, and capture the reference pressure.
  * RUNNING runs filter_loop. Starting straight from IDLE is refused, so the
  * filter is never handed an un-levelled initial state.
  ******************************************************************************
  */

#ifndef APP_STATE_H
#define APP_STATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef enum {
  APP_IDLE = 0,
  APP_CALIBRATING,
  APP_RUNNING
} app_state_t;

void app_state_init(void);

/* Applies any pending button presses. Returns true if the state changed, so
 * the caller can react to the transition (reset the filter, redraw, log it). */
bool app_state_update(void);

app_state_t app_state_get(void);
const char *app_state_name(app_state_t state);

/* True while the filter should be running its measurement update, which is the
 * is_init argument to filter_entry. */
bool app_state_filter_active(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_STATE_H */
