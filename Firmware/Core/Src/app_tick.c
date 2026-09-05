/**
  ******************************************************************************
  * @file           : app_tick.c
  * @brief          : Fixed-rate application tick and cycle-accurate timing.
  ******************************************************************************
  */

#include "app_tick.h"

#include "main.h"

/* TIM2 is deliberately not in the .ioc. It is set up here instead of in a
 * CubeMX-generated MX_TIM2_Init so that regenerating the project cannot produce
 * a duplicate definition of either the init function or TIM2_IRQHandler.
 *
 * The cost of staying out of the .ioc is that a regeneration also rewrites
 * stm32g4xx_hal_conf.h and will comment HAL_TIM_MODULE_ENABLED back out. If
 * this file suddenly fails to compile after a CubeMX run, re-enable that define.
 * If TIM2 is ever added to the .ioc properly, delete the setup below and the
 * handler at the bottom and use the generated ones. */
static TIM_HandleTypeDef htim2;

static volatile uint32_t tick_count;
static uint32_t ticks_consumed;
static uint32_t overrun_count;

static void dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void app_tick_init(void)
{
  dwt_init();

  __HAL_RCC_TIM2_CLK_ENABLE();

  /* APB1 runs undivided, so TIM2 is clocked at the full SystemCoreClock.
   * Prescale to 1 MHz and count one tick period. */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = (SystemCoreClock / 1000000u) - 1u;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = (1000000u / APP_TICK_HZ) - 1u;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }

  /* Below the default HAL tick priority so SysTick timekeeping still wins. */
  HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
    Error_Handler();
  }
}

bool app_tick_pending(void)
{
  uint32_t now = tick_count;
  uint32_t elapsed = now - ticks_consumed;

  if (elapsed == 0u) {
    return false;
  }
  if (elapsed > 1u) {
    overrun_count += elapsed - 1u;
  }

  /* Skip whatever was missed rather than trying to catch up: the filter is
   * given the dt that actually elapsed, so a late tick stays consistent. */
  ticks_consumed = now;
  return true;
}

uint32_t app_tick_overruns(void)
{
  return overrun_count;
}

uint32_t app_tick_index(void)
{
  return ticks_consumed;
}

double app_tick_time_s(void)
{
  return (double)ticks_consumed / (double)APP_TICK_HZ;
}

uint32_t app_cycles(void)
{
  return DWT->CYCCNT;
}

uint32_t app_cycles_to_us(uint32_t cycles)
{
  return cycles / (SystemCoreClock / 1000000u);
}

float app_cycles_to_s(uint32_t cycles)
{
  return (float)cycles / (float)SystemCoreClock;
}

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
    tick_count++;
  }
}
