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
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

// ----- I2C addresses (7-bit) -----
#define OLED_ADDR_7B   0x3C
#define BNO055_ADDR_7B 0x28
#define BMP390_ADDR_7B 0x77

// ----- SH1106 geometry -----
#define SH1106_WIDTH   128
#define SH1106_HEIGHT  64
#define SH1106_PAGES   (SH1106_HEIGHT/8)
#define SH1106_COL_OFF 2   // your panel needed offset=2 to fully clear

// ----- Buttons (adjust if CubeMX names differ) -----
#define BTN_STARTSTOP_GPIO_Port GPIOA
#define BTN_STARTSTOP_Pin       GPIO_PIN_10

#define BTN_CAL_GPIO_Port       GPIOB
#define BTN_CAL_Pin             GPIO_PIN_3

#define BTN_LAP_GPIO_Port       GPIOB
#define BTN_LAP_Pin             GPIO_PIN_5

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

// ---------- printf retarget (USART2) ----------
int _write(int file, char *ptr, int len) {
  (void)file;
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, (uint16_t)len, HAL_MAX_DELAY);
  return len;
}

// ---------- I2C helpers ----------
static HAL_StatusTypeDef i2c_reg_read(uint8_t addr7, uint8_t reg, uint8_t *buf, uint16_t len) {
  return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr7 << 1), reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

static HAL_StatusTypeDef i2c_reg_write(uint8_t addr7, uint8_t reg, uint8_t val) {
  return HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(addr7 << 1), reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

// ---------- SH1106: command/data ----------
static HAL_StatusTypeDef sh1106_cmd(uint8_t cmd) {
  uint8_t pkt[2] = {0x00, cmd}; // 0x00 = control byte for command
  return HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(OLED_ADDR_7B << 1), pkt, sizeof(pkt), 100);
}

static HAL_StatusTypeDef sh1106_data(const uint8_t *data, uint16_t len) {
  // 0x40 = control byte for data; send in chunks
  uint8_t buf[1 + 32];
  buf[0] = 0x40;

  while (len) {
    uint16_t chunk = (len > 32) ? 32 : len;
    memcpy(&buf[1], data, chunk);
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(OLED_ADDR_7B << 1), buf, 1 + chunk, 200);
    if (st != HAL_OK) return st;
    data += chunk;
    len  -= chunk;
  }
  return HAL_OK;
}

static void sh1106_init(void) {
  // Minimal SH1106 init (works for typical 128x64 SH1106 modules)
  HAL_Delay(50);

  sh1106_cmd(0xAE); // display off
  sh1106_cmd(0xD5); sh1106_cmd(0x80); // clock
  sh1106_cmd(0xA8); sh1106_cmd(0x3F); // multiplex 1/64
  sh1106_cmd(0xD3); sh1106_cmd(0x00); // display offset
  sh1106_cmd(0x40); // start line = 0
  sh1106_cmd(0xAD); sh1106_cmd(0x8B); // DC-DC on (common on SH1106 boards)
  sh1106_cmd(0xA1); // segment remap
  sh1106_cmd(0xC8); // COM scan direction
  sh1106_cmd(0xDA); sh1106_cmd(0x12); // COM pins
  sh1106_cmd(0x81); sh1106_cmd(0x7F); // contrast
  sh1106_cmd(0xD9); sh1106_cmd(0x22); // precharge
  sh1106_cmd(0xDB); sh1106_cmd(0x20); // VCOMH
  sh1106_cmd(0xA4); // resume RAM display
  sh1106_cmd(0xA6); // normal display
  sh1106_cmd(0xAF); // display on
}

static void sh1106_fill(uint8_t color /*0x00 black, 0xFF white*/) {
  uint8_t line[SH1106_WIDTH];
  memset(line, color, sizeof(line));

  for (uint8_t page = 0; page < SH1106_PAGES; page++) {
    sh1106_cmd((uint8_t)(0xB0 | page)); // set page address
    // set column address with offset
    uint8_t col = SH1106_COL_OFF;
    sh1106_cmd((uint8_t)(0x00 | (col & 0x0F)));       // lower nibble
    sh1106_cmd((uint8_t)(0x10 | ((col >> 4) & 0x0F))); // upper nibble
    sh1106_data(line, sizeof(line));
  }
}

// ---------- BNO055 Euler read (degrees) ----------
static bool bno055_read_euler_deg(float *heading, float *roll, float *pitch) {
  // Euler registers: H_LSB at 0x1A, 6 bytes total. Units: 1/16 degree.
  uint8_t buf[6];
  if (i2c_reg_read(BNO055_ADDR_7B, 0x1A, buf, 6) != HAL_OK) return false;

  int16_t h = (int16_t)((buf[1] << 8) | buf[0]);
  int16_t r = (int16_t)((buf[3] << 8) | buf[2]);
  int16_t p = (int16_t)((buf[5] << 8) | buf[4]);

  *heading = (float)h / 16.0f;
  *roll    = (float)r / 16.0f;
  *pitch   = (float)p / 16.0f;
  return true;
}

// ---------- BMP390 raw read (no compensation) ----------
static bool bmp390_read_raw(int32_t *temp_raw, int32_t *press_raw) {
  // Data regs typically start at 0x04: press_xlsb, press_lsb, press_msb, temp_xlsb, temp_lsb, temp_msb
  // (3 bytes each, 24-bit unsigned). This is RAW; compensation not applied.
  uint8_t d[6];
  if (i2c_reg_read(BMP390_ADDR_7B, 0x04, d, 6) != HAL_OK) return false;

  uint32_t p = ((uint32_t)d[2] << 16) | ((uint32_t)d[1] << 8) | d[0];
  uint32_t t = ((uint32_t)d[5] << 16) | ((uint32_t)d[4] << 8) | d[3];

  *press_raw = (int32_t)p;
  *temp_raw  = (int32_t)t;
  return true;
}

// ---------- Button edge detection ----------
typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
  uint8_t prev; // 0/1
  const char *name;
} button_t;

static uint8_t read_btn(GPIO_TypeDef *port, uint16_t pin) {
  return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1 : 0; // released=1, pressed=0
}

static bool button_pressed_edge(button_t *b) {
  uint8_t cur = read_btn(b->port, b->pin);
  bool pressed = (b->prev == 1 && cur == 0);
  b->prev = cur;
  return pressed;
}

static bool bno055_init_ndof(void)
{
  uint8_t id = 0;
  if (i2c_reg_read(BNO055_ADDR_7B, 0x00, &id, 1) != HAL_OK || id != 0xA0) return false;

  HAL_Delay(650);                 // BNO055 boot time per datasheet-ish practice

  i2c_reg_write(BNO055_ADDR_7B, 0x3F, 0x20);  // SYS_TRIGGER: reset
  HAL_Delay(650);

  // wait for chip ID again
  for (int i=0;i<50;i++){
    if (i2c_reg_read(BNO055_ADDR_7B, 0x00, &id, 1)==HAL_OK && id==0xA0) break;
    HAL_Delay(20);
  }

  i2c_reg_write(BNO055_ADDR_7B, 0x3D, 0x00);  // OPR_MODE = CONFIG
  HAL_Delay(30);

  i2c_reg_write(BNO055_ADDR_7B, 0x3E, 0x00);  // PWR_MODE = normal
  HAL_Delay(10);

  i2c_reg_write(BNO055_ADDR_7B, 0x07, 0x00);  // PAGE_ID = 0
  HAL_Delay(10);

  i2c_reg_write(BNO055_ADDR_7B, 0x3D, 0x0C);  // OPR_MODE = NDOF
  HAL_Delay(30);

  return true;
}

static bool bmp390_init_basic(void)
{
  uint8_t id=0;
  if (i2c_reg_read(BMP390_ADDR_7B, 0x00, &id, 1) != HAL_OK || id != 0x60) return false;

  // Soft reset
  i2c_reg_write(BMP390_ADDR_7B, 0x7E, 0xB6);
  HAL_Delay(10);

  // Enable pressure + temperature
  // PWR_CTRL (0x1B): bit0=press_en, bit1=temp_en, bit4=mode (normal) depending on BMP390 map
  // Common pattern: press_en=1, temp_en=1, mode=normal (0b11 in mode field)
  i2c_reg_write(BMP390_ADDR_7B, 0x1B, 0x33); // typical: press_en=1,temp_en=1, normal_mode=1 (verify if needed)
  HAL_Delay(5);

  // OSR (0x1C): set modest oversampling
  i2c_reg_write(BMP390_ADDR_7B, 0x1C, 0x02); // example
  // ODR (0x1D): output data rate
  i2c_reg_write(BMP390_ADDR_7B, 0x1D, 0x03); // example
  // CONFIG (0x1F): IIR etc (optional)
  i2c_reg_write(BMP390_ADDR_7B, 0x1F, 0x00);

  return true;
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
  // --- Optional: quick ID checks ---
  uint8_t id = 0;

  if (i2c_reg_read(BNO055_ADDR_7B, 0x00, &id, 1) == HAL_OK) {
    printf("BNO055 ID: 0x%02X\r\n", id);
  } else {
    printf("BNO055 ID read failed\r\n");
  }

  if (i2c_reg_read(BMP390_ADDR_7B, 0x00, &id, 1) == HAL_OK) {
    printf("BMP390 ID: 0x%02X\r\n", id);
  } else {
    printf("BMP390 ID read failed\r\n");
  }

  sh1106_init();
  sh1106_fill(0x00); // start black



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  button_t btn_ss  = {BTN_STARTSTOP_GPIO_Port, BTN_STARTSTOP_Pin, 1, "STARTSTOP"};
  button_t btn_cal = {BTN_CAL_GPIO_Port,      BTN_CAL_Pin,       1, "CAL"};
  button_t btn_lap = {BTN_LAP_GPIO_Port,      BTN_LAP_Pin,       1, "LAP"};

  // initialize prev states to current (avoid fake edge at boot)
  btn_ss.prev  = read_btn(btn_ss.port,  btn_ss.pin);
  btn_cal.prev = read_btn(btn_cal.port, btn_cal.pin);
  btn_lap.prev = read_btn(btn_lap.port, btn_lap.pin);

  uint32_t next_oled_toggle_ms = HAL_GetTick() + 1000; // toggle every 1000ms => 0.5Hz flash
  uint8_t  oled_white = 0;

  uint32_t next_print_ms = HAL_GetTick() + 200; // 5Hz

  printf("BNO055 init: %d\r\n", bno055_init_ndof());
  printf("BMP390 init: %d\r\n", bmp390_init_basic());



  while (1)
  {
    uint32_t now = HAL_GetTick();

    // ----- OLED flash at 0.5 Hz -----
    if ((int32_t)(now - next_oled_toggle_ms) >= 0) {
      oled_white ^= 1;
      sh1106_fill(oled_white ? 0xFF : 0x00);
      next_oled_toggle_ms += 1000;
    }

    // ----- 5 Hz status line -----
    if ((int32_t)(now - next_print_ms) >= 0) {
      float h=0, r=0, p=0;
      bool ok_bno = bno055_read_euler_deg(&h, &r, &p);

      int32_t traw=0, praw=0;
      bool ok_bmp = bmp390_read_raw(&traw, &praw);

      uint8_t ss  = read_btn(btn_ss.port,  btn_ss.pin);
      uint8_t cal = read_btn(btn_cal.port, btn_cal.pin);
      uint8_t lap = read_btn(btn_lap.port, btn_lap.pin);

      // One line, readable headings (not CSV)
      // Note: BMP390 values are RAW until you implement compensation.
      printf(
        "t=%lums | BNO055(h=%.2f r=%.2f p=%.2f ok=%u) | BMP390(rawT=%ld rawP=%ld ok=%u) | BTN(SS=%u CAL=%u LAP=%u) | OLED=%s\r\n",
        (unsigned long)now,
        (double)h, (double)r, (double)p, (unsigned)ok_bno,
        (long)traw, (long)praw, (unsigned)ok_bmp,
        (unsigned)ss, (unsigned)cal, (unsigned)lap,
        oled_white ? "WHITE" : "BLACK"
      );

      next_print_ms += 200;
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
  hi2c1.Init.Timing = 0x40B285C2;
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
  huart2.Init.BaudRate = 115200;
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
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
  /* User can add his own implementation to report the HAL error return state */
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
