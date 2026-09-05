/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_state.h"
#include "app_tick.h"
#include "buttons.h"
#include "display.h"
#include "filter_app.h"
#include "i2c_bus.h"
#include "sensors.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

extern UART_HandleTypeDef huart2;

// ---------- printf retarget (USART2) ----------
// Blocking. At 921600 baud a 200-character line costs about 2.2 ms, which fits
// inside the tick, but it is still the largest avoidable cost in the loop and
// is the thing to move to DMA when the serial stream goes continuous.
int _write(int file, char *ptr, int len) {
  (void)file;
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, (uint16_t)len, HAL_MAX_DELAY);
  return len;
}

static float vec3_norm(const float v[3]) {
  return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

// Body-to-nav quaternion (Hamilton, scalar first) to NED roll/pitch/yaw in
// degrees. For reporting only; the filter works in quaternions throughout.
static void quat_to_euler_deg(const float q[4], float *roll, float *pitch, float *yaw) {
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  const float rad_to_deg = 57.29577951f;

  *roll = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y)) * rad_to_deg;

  // Clamped because rounding can push the argument just outside asin's domain
  // at the poles, where pitch is +/-90 degrees.
  float s = 2.0f*(w*y - z*x);
  if (s > 1.0f) s = 1.0f;
  if (s < -1.0f) s = -1.0f;
  *pitch = asinf(s) * rad_to_deg;

  *yaw = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z)) * rad_to_deg;
}

// ---------- status screen ----------
// Redrawing only writes the framebuffer; display_service() pushes changed pages
// to the panel one at a time from the main loop, so nothing here blocks.
//
// The font is uppercase-only, so all text is written that way.
// How long a LAP snapshot stays on screen before live updates resume.
#define LAP_FREEZE_MS 1000u

// lap_marker labels the frame as a held snapshot. Freezing only suspends
// redrawing - the filter keeps stepping every tick throughout, so the values
// shown are a sample of a running estimate, not a paused one.
static void render_screen(bool lap_marker) {
  char line[DISPLAY_COLS + 1];
  const app_state_t state = app_state_get();

  if (lap_marker) {
    snprintf(line, sizeof(line), "LAP  %s", app_state_name(state));
    display_line(0, line);
  } else {
    display_line(0, app_state_name(state));
  }

  if (state == APP_IDLE) {
    display_line(2, "PRESS CAL TO LEVEL");
    display_line(3, "");
    display_line(4, "");
    display_line(6, "");
    display_line(7, "");
    return;
  }

  if (!filter_app_seeded()) {
    display_line(2, "WAITING FOR SENSORS");
    return;
  }

  const filter_output_t *f = filter_app_output();
  float roll, pitch, yaw;
  quat_to_euler_deg(f->q, &roll, &pitch, &yaw);

  // Heading reads more naturally as 0..360 than as the filter's -180..180.
  float heading = yaw < 0.0f ? yaw + 360.0f : yaw;

  snprintf(line, sizeof(line), "ROLL   %+7.1f", (double)roll);
  display_line(2, line);
  snprintf(line, sizeof(line), "PITCH  %+7.1f", (double)pitch);
  display_line(3, line);
  snprintf(line, sizeof(line), "HDG     %6.1f", (double)heading);
  display_line(4, line);

  snprintf(line, sizeof(line), "ALT   %+7.2f M", (double)f->h);
  display_line(6, line);

  if (state == APP_CALIBRATING) {
    display_line(7, "LEVELLING");
  } else {
    // Both saturate: the exact value past the cap does not tell the user
    // anything the cap does not, and it keeps the row inside 21 characters.
    unsigned cpu_ms = (unsigned)(app_cycles_to_us(f->last_cycles) / 1000u);
    unsigned missed = (unsigned)app_tick_overruns();
    if (cpu_ms > 99u) cpu_ms = 99u;
    if (missed > 999u) missed = 999u;

    snprintf(line, sizeof(line), "CPU %2u MS  MISS %3u", cpu_ms, missed);
    display_line(7, line);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  display_init();

  buttons_init();
  app_state_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  bool sensors_ok = sensors_init();
  printf("sensors init ok=%u\r\n", (unsigned)sensors_ok);

  filter_app_init();
  printf("STATE %s\r\n", app_state_name(app_state_get()));
  render_screen(false);

  // Started last, so the overrun count reflects the running loop rather than
  // the ~800 ms of blocking delays in the BNO055 reset sequence above.
  app_tick_init();

  uint32_t last_tick_cycles = app_cycles();
  bool have_last_tick = false;

  // Tick interval statistics, reset at the end of each reporting window.
  uint32_t dt_min_us = 0xFFFFFFFFu;
  uint32_t dt_max_us = 0;
  uint32_t ticks_in_window = 0;

  uint32_t next_report_ms = HAL_GetTick() + 500; // 2 Hz

  sensors_t sens = {0};

  // Per-sensor counts of genuinely new samples in the reporting window. These
  // should converge on each sensor's configured output data rate.
  uint32_t n_accel = 0, n_gyro = 0, n_mag = 0, n_baro = 0;

  uint32_t ticks_since_render = 0;

  // Integrated gyro angle per axis, in degrees, reset by the LAP button. Turning
  // the board through a known angle and comparing settles whether the raw counts
  // are being scaled correctly, which no static reading can tell us.
  float gyro_angle_deg[3] = {0.0f, 0.0f, 0.0f};

  // Deadline until which the LAP snapshot is held on screen.
  uint32_t display_freeze_until_ms = 0;

  while (1)
  {
    if (!app_tick_pending()) {
      continue;
    }

    uint32_t now_cycles = app_cycles();
    if (have_last_tick) {
      uint32_t dt_us = app_cycles_to_us(now_cycles - last_tick_cycles);
      if (dt_us < dt_min_us) dt_min_us = dt_us;
      if (dt_us > dt_max_us) dt_max_us = dt_us;
      ticks_in_window++;
    }
    last_tick_cycles = now_cycles;
    have_last_tick = true;

    // Every device is polled each tick, but each one reports a status that is
    // true only when it actually produced a new sample. The filter consumes
    // those statuses directly.
    sensors_read(&sens);
    if (sens.gyro.status) {
      for (int i = 0; i < 3; i++) {
        gyro_angle_deg[i] += sens.gyro.meas[i] * (57.29578f / (float)APP_TICK_HZ);
      }
    }

    if (sens.accel.status) n_accel++;
    if (sens.gyro.status) n_gyro++;
    if (sens.mag.status) n_mag++;
    if (sens.baro.status) n_baro++;

    // IDLE leaves the filter untouched. CALIBRATING runs filter_init to level
    // the attitude and capture the reference pressure, RUNNING runs filter_loop.
    if (app_state_get() != APP_IDLE) {
      filter_app_step(&sens, app_tick_time_s(), app_state_filter_active());
    }

    buttons_poll();

    if (app_state_update()) {
      // Stopping discards the estimate, so the next calibration starts clean
      // rather than continuing from a state the user has abandoned.
      if (app_state_get() == APP_IDLE) {
        filter_app_reset();
      }
      if (app_state_get() == APP_RUNNING) {
        filter_app_clear_cost();
      }
      printf("STATE %s\r\n", app_state_name(app_state_get()));
      render_screen(false);
    }

    if (buttons_pressed(BTN_LAP)) {
      gyro_angle_deg[0] = gyro_angle_deg[1] = gyro_angle_deg[2] = 0.0f;
      printf("LAP t=%lums state=%s\r\n",
             (unsigned long)HAL_GetTick(), app_state_name(app_state_get()));

      // Capture the state as it stands and hold it on screen. The filter is
      // untouched by this: it keeps stepping every tick, and only the redraw
      // is suspended, so the snapshot ages rather than the estimate stalling.
      render_screen(true);
      display_freeze_until_ms = HAL_GetTick() + LAP_FREEZE_MS;
    }

    // Live redraw at 5 Hz, unless a LAP snapshot is being held. display_service
    // runs either way, so the held frame still finishes reaching the panel.
    if (++ticks_since_render >= APP_TICK_HZ / 5u) {
      ticks_since_render = 0;
      if ((int32_t)(HAL_GetTick() - display_freeze_until_ms) >= 0) {
        render_screen(false);
      }
    }
    display_service();

    if ((int32_t)(HAL_GetTick() - next_report_ms) >= 0) {
      // Magnitudes are the quickest check that the units are right: at rest
      // |accel| should be ~9.81 m/s^2 and |mag| should be tens of microtesla.
      float a_norm = vec3_norm(sens.accel.meas);
      float m_norm = vec3_norm(sens.mag.meas);

      // Sample counts are doubled into per-second rates: the window is 500 ms.
      printf("%-11s | tick(n=%lu dt=%lu..%lu us over=%lu)"
             " | a[%+6.2f %+6.2f %+6.2f]=%5.2f"
             " | g[%+6.3f %+6.3f %+6.3f]"
             " | m[%+7.1f %+7.1f %+7.1f]=%5.1f"
             " | baro=%9.1f Pa"
             " | gyro_int[%+7.1f %+7.1f %+7.1f] deg"
             " | rate(a=%lu g=%lu m=%lu b=%lu Hz) | faults=%lu | BTN(%u%u%u)\r\n",
             app_state_name(app_state_get()),
             (unsigned long)ticks_in_window,
             (unsigned long)(ticks_in_window ? dt_min_us : 0),
             (unsigned long)dt_max_us,
             (unsigned long)app_tick_overruns(),
             (double)sens.accel.meas[0], (double)sens.accel.meas[1],
             (double)sens.accel.meas[2], (double)a_norm,
             (double)sens.gyro.meas[0], (double)sens.gyro.meas[1],
             (double)sens.gyro.meas[2],
             (double)sens.mag.meas[0], (double)sens.mag.meas[1],
             (double)sens.mag.meas[2], (double)m_norm,
             (double)sens.baro.meas,
             (double)gyro_angle_deg[0], (double)gyro_angle_deg[1],
             (double)gyro_angle_deg[2],
             (unsigned long)(n_accel * 2), (unsigned long)(n_gyro * 2),
             (unsigned long)(n_mag * 2), (unsigned long)(n_baro * 2),
             sensors_fault_count(),
             (unsigned)buttons_held(BTN_START_STOP),
             (unsigned)buttons_held(BTN_CAL),
             (unsigned)buttons_held(BTN_LAP));

      if (app_state_get() != APP_IDLE && filter_app_seeded()) {
        const filter_output_t *f = filter_app_output();
        float roll, pitch, yaw;
        quat_to_euler_deg(f->q, &roll, &pitch, &yaw);

        printf("            > filter rpy[%+7.2f %+7.2f %+7.2f] deg"
               " | bias[%+7.4f %+7.4f %+7.4f] rad/s"
               " | a_z=%+6.2f v_z=%+6.2f h=%+7.2f"
               " | |P|=%9.3e | dt=%.4f\r\n",
               (double)roll, (double)pitch, (double)yaw,
               (double)f->bias[0], (double)f->bias[1], (double)f->bias[2],
               (double)f->a_z, (double)f->v_z, (double)f->h,
               (double)f->P_frobenius, (double)f->dt);

        // Mean and worst cost of filter_entry, split by which measurement
        // updates ran. base = dynamics + accel only; the others add the
        // magnetometer and barometer updates on their slower cycles.
        printf("            > cost us  base=%lu/%lu  +mag=%lu/%lu"
               "  +baro=%lu/%lu  +both=%lu/%lu  (mean/max)\r\n",
               (unsigned long)app_cycles_to_us(filter_app_mean_cycles(FILTER_COST_BASE)),
               (unsigned long)app_cycles_to_us(filter_app_max_cycles(FILTER_COST_BASE)),
               (unsigned long)app_cycles_to_us(filter_app_mean_cycles(FILTER_COST_MAG)),
               (unsigned long)app_cycles_to_us(filter_app_max_cycles(FILTER_COST_MAG)),
               (unsigned long)app_cycles_to_us(filter_app_mean_cycles(FILTER_COST_BARO)),
               (unsigned long)app_cycles_to_us(filter_app_max_cycles(FILTER_COST_BARO)),
               (unsigned long)app_cycles_to_us(filter_app_mean_cycles(FILTER_COST_BOTH)),
               (unsigned long)app_cycles_to_us(filter_app_max_cycles(FILTER_COST_BOTH)));
      }

      n_accel = n_gyro = n_mag = n_baro = 0;
      dt_min_us = 0xFFFFFFFFu;
      dt_max_us = 0;
      ticks_in_window = 0;
      next_report_ms += 500;
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x30422838;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
