/**
  ******************************************************************************
  * @file           : app_state.c
  * @brief          : Top-level mode of the dead-reckoning application.
  ******************************************************************************
  */

#include "app_state.h"

#include "buttons.h"

static app_state_t state;

void app_state_init(void)
{
  state = APP_IDLE;
}

bool app_state_update(void)
{
  app_state_t previous = state;

  bool start_stop = buttons_pressed(BTN_START_STOP);
  bool calibrate = buttons_pressed(BTN_CAL);

  switch (state) {
  case APP_IDLE:
    if (calibrate) {
      state = APP_CALIBRATING;
    }
    /* START from IDLE is deliberately ignored: calibration is always required
     * before tracking. */
    break;

  case APP_CALIBRATING:
    if (start_stop) {
      state = APP_RUNNING;
    } else if (calibrate) {
      /* Already calibrating; restart it so a fresh press re-levels from the
       * current attitude rather than doing nothing. */
      return true;
    }
    break;

  case APP_RUNNING:
    if (start_stop) {
      state = APP_IDLE;
    } else if (calibrate) {
      /* Re-level without a full stop. */
      state = APP_CALIBRATING;
    }
    break;

  default:
    state = APP_IDLE;
    break;
  }

  return state != previous;
}

app_state_t app_state_get(void)
{
  return state;
}

const char *app_state_name(app_state_t s)
{
  switch (s) {
  case APP_IDLE:        return "IDLE";
  case APP_CALIBRATING: return "CALIBRATING";
  case APP_RUNNING:     return "RUNNING";
  default:              return "?";
  }
}

bool app_state_filter_active(void)
{
  return state == APP_RUNNING;
}
