/**
  ******************************************************************************
  * @file           : app_tick.h
  * @brief          : Fixed-rate application tick and cycle-accurate timing.
  *
  * TIM2 drives the filter at a fixed rate so the sample interval does not
  * depend on how long the main loop happens to take. The DWT cycle counter is
  * used separately to measure the interval that actually elapsed, which is what
  * the filter is given as dt, and to profile how long individual steps take.
  ******************************************************************************
  */

#ifndef APP_TICK_H
#define APP_TICK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define APP_TICK_HZ 100u

/* Starts TIM2 and enables the DWT cycle counter. Call after the clock and HAL
 * are up. */
void app_tick_init(void);

/* Consumes one pending tick. Returns false when the application is idle and
 * waiting for the next one. */
bool app_tick_pending(void);

/* Ticks that elapsed while the main loop was still busy with a previous one.
 * Any non-zero value means the loop is not keeping up with APP_TICK_HZ. */
uint32_t app_tick_overruns(void);

/* Index of the tick most recently returned by app_tick_pending. Divided by
 * APP_TICK_HZ this is a monotonic time in seconds that skips forward over any
 * missed ticks, which is what the filter needs for dt. The DWT counter is not
 * usable for this because it wraps every 25 seconds. */
uint32_t app_tick_index(void);

/* Monotonic seconds since the tick started, derived from app_tick_index. */
double app_tick_time_s(void);

/* Free-running CPU cycle count. Wraps every ~25 s at 170 MHz; differences
 * between two samples stay correct across a wrap in unsigned arithmetic. */
uint32_t app_cycles(void);

uint32_t app_cycles_to_us(uint32_t cycles);
float app_cycles_to_s(uint32_t cycles);

#ifdef __cplusplus
}
#endif

#endif /* APP_TICK_H */
