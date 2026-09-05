/**
  ******************************************************************************
  * @file           : buttons.c
  * @brief          : Debounced edge detection for the three shield buttons.
  ******************************************************************************
  */

#include "buttons.h"

#include "app_tick.h"
#include "main.h"

/* Consecutive agreeing samples before a level change is accepted. At 100 Hz
 * this is 30 ms, comfortably longer than the contact bounce on these switches
 * and still far below a deliberate press. */
#define DEBOUNCE_TICKS 3u

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
  uint8_t candidate;  /* level currently being confirmed */
  uint8_t stable;     /* level accepted after DEBOUNCE_TICKS */
  uint8_t agree;      /* samples so far agreeing with candidate */
  bool press_pending; /* an unconsumed press edge */
} button_t;

static button_t buttons[BTN_COUNT] = {
  [BTN_START_STOP] = {GPIOA, GPIO_PIN_10},
  [BTN_CAL]        = {GPIOB, GPIO_PIN_3},
  [BTN_LAP]        = {GPIOB, GPIO_PIN_5},
};

/* Buttons are active low: the pin reads high when released. */
static uint8_t read_level(const button_t *b)
{
  return (HAL_GPIO_ReadPin(b->port, b->pin) == GPIO_PIN_RESET) ? 1u : 0u;
}

void buttons_init(void)
{
  for (int i = 0; i < BTN_COUNT; i++) {
    button_t *b = &buttons[i];
    /* Seed from the current level so a button held at boot is not reported as
     * a fresh press. */
    b->stable = read_level(b);
    b->candidate = b->stable;
    b->agree = DEBOUNCE_TICKS;
    b->press_pending = false;
  }
}

void buttons_poll(void)
{
  for (int i = 0; i < BTN_COUNT; i++) {
    button_t *b = &buttons[i];
    uint8_t level = read_level(b);

    if (level != b->candidate) {
      b->candidate = level;
      b->agree = 1u;
      continue;
    }

    if (b->agree < DEBOUNCE_TICKS) {
      b->agree++;
      if (b->agree == DEBOUNCE_TICKS && b->candidate != b->stable) {
        b->stable = b->candidate;
        if (b->stable == 1u) {
          b->press_pending = true;
        }
      }
    }
  }
}

bool buttons_pressed(button_id_t id)
{
  if (id >= BTN_COUNT) {
    return false;
  }
  bool pressed = buttons[id].press_pending;
  buttons[id].press_pending = false;
  return pressed;
}

bool buttons_held(button_id_t id)
{
  if (id >= BTN_COUNT) {
    return false;
  }
  return buttons[id].stable == 1u;
}
