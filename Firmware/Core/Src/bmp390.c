/**
  ******************************************************************************
  * @file           : bmp390.c
  * @brief          : BMP390 barometric pressure sensor, compensated to pascals.
  ******************************************************************************
  */

#include "bmp390.h"

#include <stdio.h>

#include "i2c_bus.h"
#include "main.h"

#define BMP390_ADDR 0x77

#define REG_CHIP_ID  0x00
#define REG_STATUS   0x03
#define REG_DATA     0x04 /* press[3] then temp[3], both little endian */
#define REG_PWR_CTRL 0x1B
#define REG_OSR      0x1C
#define REG_ODR      0x1D
#define REG_CONFIG   0x1F
#define REG_CALIB    0x31 /* 21 bytes of factory trim */
#define REG_CMD      0x7E

#define CHIP_ID 0x60

#define STATUS_DRDY_PRESS (1u << 5)

/* PWR_CTRL: pressure and temperature enabled, normal (continuous) mode. */
#define PWR_CTRL_VALUE 0x33
/* OSR: pressure oversampled x32, temperature x2. Oversampling happens inside
 * the sensor, before a sample is emitted, so unlike the IIR below it lowers the
 * noise without correlating consecutive samples - which is what the EKF needs.
 * Conversion time is 234 + (392 + 2^osr_p * 2020) + (163 + 2^osr_t * 2020) us,
 * here 69.5 ms, so it no longer fits a 40 ms period and the ODR drops to match.
 * Measured noise: 1.71 Pa at x8, 0.86 Pa expected here. Halving the rate while
 * halving sigma still roughly doubles the information reaching the filter. */
#define OSR_VALUE 0x0D
/* ODR: 12.5 Hz, the fastest rate the conversion above fits inside (80 ms). */
#define ODR_VALUE 0x04
/* CONFIG: IIR filter disabled. The EKF assumes measurement noise is white, and
 * an on-chip IIR would correlate consecutive samples and break that. */
#define CONFIG_VALUE 0x00

/* Factory trim, quantised per Bosch's floating-point compensation. */
static struct {
  double t1, t2, t3;
  double p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
} calib;

/* Temperature in Bosch's linearised form, carried from the temperature
 * compensation into the pressure compensation. */
static double t_lin;

static uint16_t u16(const uint8_t *p)
{
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int16_t s16(const uint8_t *p)
{
  return (int16_t)u16(p);
}

static bool read_calibration(void)
{
  uint8_t d[21];
  if (!i2c_read(BMP390_ADDR, REG_CALIB, d, sizeof(d))) {
    return false;
  }

  /* Each coefficient is scaled by the power of two Bosch specifies, so that
   * the compensation below is a plain polynomial evaluation. */
  calib.t1 = (double)u16(&d[0]) * 256.0;               /* 2^8    */
  calib.t2 = (double)u16(&d[2]) / 1073741824.0;        /* 2^-30  */
  calib.t3 = (double)(int8_t)d[4] / 281474976710656.0; /* 2^-48  */

  calib.p1 = (double)(s16(&d[5]) - 16384) / 1048576.0;    /* 2^-20 */
  calib.p2 = (double)(s16(&d[7]) - 16384) / 536870912.0;  /* 2^-29 */
  calib.p3 = (double)(int8_t)d[9] / 4294967296.0;         /* 2^-32 */
  calib.p4 = (double)(int8_t)d[10] / 137438953472.0;      /* 2^-37 */
  calib.p5 = (double)u16(&d[11]) * 8.0;                   /* 2^3   */
  calib.p6 = (double)u16(&d[13]) / 64.0;                  /* 2^-6  */
  calib.p7 = (double)(int8_t)d[15] / 256.0;               /* 2^-8  */
  calib.p8 = (double)(int8_t)d[16] / 32768.0;             /* 2^-15 */
  calib.p9 = (double)s16(&d[17]) / 281474976710656.0;     /* 2^-48 */
  calib.p10 = (double)(int8_t)d[19] / 281474976710656.0;  /* 2^-48 */
  calib.p11 = (double)(int8_t)d[20] / 36893488147419103232.0; /* 2^-65 */

  return true;
}

/* Bosch's compensation is evaluated in double precision on purpose. The raw
 * pressure is around 7e6 counts and the polynomial cubes it, which overflows
 * the 24-bit mantissa of a float long before the result is meaningful. */
static double compensate_temperature(uint32_t uncomp_temp)
{
  double partial1 = (double)uncomp_temp - calib.t1;
  double partial2 = partial1 * calib.t2;
  t_lin = partial2 + (partial1 * partial1) * calib.t3;
  return t_lin;
}

static double compensate_pressure(uint32_t uncomp_press)
{
  double partial1 = calib.p6 * t_lin;
  double partial2 = calib.p7 * (t_lin * t_lin);
  double partial3 = calib.p8 * (t_lin * t_lin * t_lin);
  double out1 = calib.p5 + partial1 + partial2 + partial3;

  partial1 = calib.p2 * t_lin;
  partial2 = calib.p3 * (t_lin * t_lin);
  partial3 = calib.p4 * (t_lin * t_lin * t_lin);
  double out2 = (double)uncomp_press * (calib.p1 + partial1 + partial2 + partial3);

  double press_sq = (double)uncomp_press * (double)uncomp_press;
  partial1 = calib.p9 + calib.p10 * t_lin;
  partial2 = press_sq * partial1;
  partial3 = partial2 + (press_sq * (double)uncomp_press) * calib.p11;

  return out1 + out2 + partial3;
}

bool bmp390_init(void)
{
  uint8_t id = 0;
  if (!i2c_read(BMP390_ADDR, REG_CHIP_ID, &id, 1) || id != CHIP_ID) {
    printf("BMP390: chip ID 0x%02X, expected 0x%02X\r\n", id, CHIP_ID);
    return false;
  }

  (void)i2c_write(BMP390_ADDR, REG_CMD, 0xB6); /* soft reset */
  HAL_Delay(10);

  if (!read_calibration()) {
    printf("BMP390: calibration read failed\r\n");
    return false;
  }

  (void)i2c_write(BMP390_ADDR, REG_OSR, OSR_VALUE);
  (void)i2c_write(BMP390_ADDR, REG_ODR, ODR_VALUE);
  (void)i2c_write(BMP390_ADDR, REG_CONFIG, CONFIG_VALUE);
  (void)i2c_write(BMP390_ADDR, REG_PWR_CTRL, PWR_CTRL_VALUE);
  HAL_Delay(10);

  printf("BMP390: ready\r\n");
  return true;
}

bool bmp390_read(float *pressure_pa, float *temperature_c)
{
  uint8_t status = 0;
  if (!i2c_read(BMP390_ADDR, REG_STATUS, &status, 1)) {
    return false;
  }
  if ((status & STATUS_DRDY_PRESS) == 0u) {
    return false; /* conversion still in progress; no new sample */
  }

  uint8_t d[6];
  if (!i2c_read(BMP390_ADDR, REG_DATA, d, sizeof(d))) {
    return false;
  }

  uint32_t raw_press = ((uint32_t)d[2] << 16) | ((uint32_t)d[1] << 8) | d[0];
  uint32_t raw_temp = ((uint32_t)d[5] << 16) | ((uint32_t)d[4] << 8) | d[3];

  /* Temperature has to be compensated first: it produces t_lin, which the
   * pressure compensation depends on. */
  double temp_c = compensate_temperature(raw_temp);
  double press_pa = compensate_pressure(raw_press);

  *temperature_c = (float)temp_c;
  *pressure_pa = (float)press_pa;
  return true;
}
