/**
  ******************************************************************************
  * @file           : buttons.h
  * @brief          : Debounced edge detection for the three shield buttons.
  *
  * SW1/SW2/SW3 on the shield reach the Nucleo on D2/D3/D4, which are PA10/PB3/
  * PB5. They are active low against 10k pull-ups, with the internal pull-ups
  * enabled as well.
  ******************************************************************************
  */

#ifndef BUTTONS_H
#define BUTTONS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef enum {
  BTN_START_STOP = 0, /* SW1, PA10 */
  BTN_CAL,            /* SW2, PB3  */
  BTN_LAP,            /* SW3, PB5  */
  BTN_COUNT
} button_id_t;

void buttons_init(void);

/* Samples all three buttons. Call once per application tick; the debounce
 * interval is defined in ticks. */
void buttons_poll(void);

/* True once per press, on the release-to-press edge. Consumes the event, so
 * each press is reported to exactly one caller. */
bool buttons_pressed(button_id_t id);

/* Current debounced level, for status output. True while held down. */
bool buttons_held(button_id_t id);

#ifdef __cplusplus
}
#endif

#endif /* BUTTONS_H */
