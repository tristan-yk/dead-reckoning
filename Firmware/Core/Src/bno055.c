/**
  ******************************************************************************
  * @file           : bno055.c
  * @brief          : BNO055 in AMG mode - raw accelerometer, magnetometer and
  *                   gyroscope.
  ******************************************************************************
  */

#include "bno055.h"

#include <stdio.h>

#include "i2c_bus.h"
#include "main.h"

#define BNO055_ADDR 0x28

/* Page 0 registers */
#define REG_CHIP_ID     0x00
#define REG_PAGE_ID     0x07
#define REG_ACC_DATA_X  0x08 /* accel, mag and gyro are contiguous from here */
#define REG_UNIT_SEL    0x3B
#define REG_SYS_ERR     0x3A
#define REG_OPR_MODE    0x3D
#define REG_PWR_MODE    0x3E
#define REG_SYS_TRIGGER 0x3F

/* Page 1 registers */
#define REG_ACC_CONFIG  0x08
#define REG_MAG_CONFIG  0x09
#define REG_GYR_CONFIG0 0x0A
#define REG_GYR_CONFIG1 0x0B

#define CHIP_ID       0xA0
#define OPR_MODE_CONFIG 0x00
#define OPR_MODE_AMG    0x07

/* UNIT_SEL: accelerometer in m/s^2 (bit 0 clear) and gyroscope in rad/s
 * (bit 1 set), so no unit conversion is needed beyond the fixed LSB scale. */
#define UNIT_SEL_VALUE 0x02

/* Fixed-point scales that follow from UNIT_SEL, per the datasheet:
 * 100 LSB per m/s^2, 900 LSB per rad/s, 16 LSB per microtesla. */
#define ACCEL_LSB_PER_UNIT 100.0f
#define GYRO_LSB_PER_UNIT  900.0f
#define MAG_LSB_PER_UNIT   16.0f

/* Accelerometer calibration, from a six-position measurement on this board:
 * each axis was held pointing up and then down, and bias and scale solved as
 * bias = (up + down)/2 and scale = (up - down)/2g. Applying these brought |a|
 * across all poses from a spread of 1.41 m/s^2 down to 0.034, centred on g.
 *
 * The bias matters more than the scale. filter_loop derives g_magnitude from
 * norm(mem.sens_filt.accel), so a uniform scale error largely cancels, but a
 * bias does not: the -0.706 on Y alone tilted the levelled horizon by 4.1 deg.
 *
 * These are specific to this unit. Re-run the six-position capture if the
 * sensor is replaced or remounted. */
static const float accel_bias[3] = {-0.0721f, -0.7060f, -0.1478f};
static const float accel_scale[3] = {0.98668f, 0.98615f, 0.98775f};

/* Magnetometer hard-iron offsets, taken from the same capture as the midpoint
 * of each axis's range. This removes the constant field contributed by the
 * board itself, which would otherwise bias heading. Correcting it tightened the
 * spread in |m| from 21.7 to 6.5 uT around a mean of 57.6 uT.
 *
 * Six orientations is a sparse sample for this, so the residual spread is
 * probably soft-iron distortion that a full ellipsoid fit would catch. */
static const float mag_hard_iron[3] = {7.45f, 0.95f, -8.15f};

/* Sensor configuration, chosen against the 100 Hz filter rate.
 *
 * ACC_CONFIG  0x0D: normal mode, 62.5 Hz bandwidth, +/-4 g. The bandwidth is
 *   just under the Nyquist limit of the 100 Hz sample rate.
 * MAG_CONFIG  0x0D: normal power, regular mode, 20 Hz output. The filter
 *   consumes magnetometer data at 20 Hz, so a faster rate would only repeat.
 * GYR_CONFIG0 0x18: +/-2000 deg/s with 47 Hz bandwidth. This was briefly set to
 *   +/-500 deg/s on the theory that a handheld device would never turn faster,
 *   which was wrong twice over. Measured hand motion reached 696 deg/s, and at
 *   500 deg/s the readings also disagreed with the datasheet's fixed scaling of
 *   16 LSB per deg/s - counts appeared that the range should not have allowed.
 *   At 2000 deg/s the question does not arise: fixed scaling puts full travel at
 *   32000 counts against an int16 limit of 32767, so the fixed and full-scale
 *   interpretations agree to within 2.3% and the conversion below is right
 *   either way.
 */
#define ACC_CONFIG_VALUE  0x0D
#define MAG_CONFIG_VALUE  0x0D
#define GYR_CONFIG0_VALUE 0x18

static bool wait_for_chip_id(void)
{
  for (int i = 0; i < 100; i++) {
    uint8_t id = 0;
    if (i2c_read(BNO055_ADDR, REG_CHIP_ID, &id, 1) && id == CHIP_ID) {
      return true;
    }
    HAL_Delay(10);
  }
  return false;
}

static int16_t le16(const uint8_t *p)
{
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

bool bno055_init(void)
{
  /* The chip is not addressable for the first several hundred milliseconds
   * after power-on, so poll rather than failing on the first attempt. */
  if (!wait_for_chip_id()) {
    printf("BNO055: no chip ID before reset\r\n");
    return false;
  }

  (void)i2c_write(BNO055_ADDR, REG_SYS_TRIGGER, 0x20); /* soft reset */
  HAL_Delay(650);

  if (!wait_for_chip_id()) {
    printf("BNO055: no chip ID after reset\r\n");
    return false;
  }

  /* Everything below has to happen in config mode. */
  (void)i2c_write(BNO055_ADDR, REG_OPR_MODE, OPR_MODE_CONFIG);
  HAL_Delay(30);
  (void)i2c_write(BNO055_ADDR, REG_PWR_MODE, 0x00); /* normal power */
  HAL_Delay(10);

  (void)i2c_write(BNO055_ADDR, REG_PAGE_ID, 0x01);
  HAL_Delay(2);
  (void)i2c_write(BNO055_ADDR, REG_ACC_CONFIG, ACC_CONFIG_VALUE);
  (void)i2c_write(BNO055_ADDR, REG_MAG_CONFIG, MAG_CONFIG_VALUE);
  (void)i2c_write(BNO055_ADDR, REG_GYR_CONFIG0, GYR_CONFIG0_VALUE);
  (void)i2c_write(BNO055_ADDR, REG_GYR_CONFIG1, 0x00); /* normal power */
  (void)i2c_write(BNO055_ADDR, REG_PAGE_ID, 0x00);
  HAL_Delay(2);

  (void)i2c_write(BNO055_ADDR, REG_UNIT_SEL, UNIT_SEL_VALUE);
  HAL_Delay(10);

  (void)i2c_write(BNO055_ADDR, REG_OPR_MODE, OPR_MODE_AMG);
  HAL_Delay(30);

  uint8_t opr = i2c_read_u8(BNO055_ADDR, REG_OPR_MODE);
  uint8_t unit = i2c_read_u8(BNO055_ADDR, REG_UNIT_SEL);
  uint8_t sys_err = i2c_read_u8(BNO055_ADDR, REG_SYS_ERR);
  printf("BNO055: OPR_MODE=0x%02X UNIT_SEL=0x%02X SYS_ERR=0x%02X\r\n",
         opr, unit, sys_err);

  /* Read the page 1 sensor configuration back. These registers are only
   * writable in config mode and are not obviously reported anywhere else, so
   * without this there is no way to tell a write that took from one that did
   * not - and the gyroscope range decides how the raw counts should be scaled. */
  (void)i2c_write(BNO055_ADDR, REG_PAGE_ID, 0x01);
  HAL_Delay(2);
  uint8_t acc_cfg = i2c_read_u8(BNO055_ADDR, REG_ACC_CONFIG);
  uint8_t mag_cfg = i2c_read_u8(BNO055_ADDR, REG_MAG_CONFIG);
  uint8_t gyr_cfg = i2c_read_u8(BNO055_ADDR, REG_GYR_CONFIG0);
  (void)i2c_write(BNO055_ADDR, REG_PAGE_ID, 0x00);
  HAL_Delay(2);

  static const char *gyro_range[8] = {"2000", "1000", "500", "250",
                                      "125",  "?",    "?",   "?"};
  static const char *accel_range[4] = {"2G", "4G", "8G", "16G"};

  printf("BNO055: ACC_CONFIG=0x%02X (wrote 0x%02X, range %s)"
         "  MAG_CONFIG=0x%02X (wrote 0x%02X)"
         "  GYR_CONFIG0=0x%02X (wrote 0x%02X, range %s dps)\r\n",
         acc_cfg, ACC_CONFIG_VALUE, accel_range[acc_cfg & 0x03],
         mag_cfg, MAG_CONFIG_VALUE,
         gyr_cfg, GYR_CONFIG0_VALUE, gyro_range[gyr_cfg & 0x07]);

  if (opr != OPR_MODE_AMG) {
    printf("BNO055: failed to enter AMG mode\r\n");
    return false;
  }
  return true;
}

/* The previous raw burst, used to tell a new sample from a repeated one. */
static uint8_t previous_raw[18];
static bool have_previous;

static bool block_changed(const uint8_t *now, int offset)
{
  for (int i = 0; i < 6; i++) {
    if (now[offset + i] != previous_raw[offset + i]) {
      return true;
    }
  }
  return false;
}

bool bno055_read(bno055_sample_t *out)
{
  uint8_t d[18];
  if (!i2c_read(BNO055_ADDR, REG_ACC_DATA_X, d, sizeof(d))) {
    return false;
  }

  for (int i = 0; i < 3; i++) {
    float accel_raw = (float)le16(&d[i * 2]) / ACCEL_LSB_PER_UNIT;
    out->accel[i] = (accel_raw - accel_bias[i]) / accel_scale[i];

    out->mag[i] = ((float)le16(&d[6 + i * 2]) / MAG_LSB_PER_UNIT) - mag_hard_iron[i];

    /* The gyro is left uncorrected on purpose: its bias is part of the filter
     * state, seeded by filter_init from the low-passed rate at standstill. */
    out->gyro[i] = (float)le16(&d[12 + i * 2]) / GYRO_LSB_PER_UNIT;
  }

  /* On the very first read every sensor counts as new: there is nothing to
   * compare against, and the caller needs a sample to seed its filters with. */
  out->accel_new = !have_previous || block_changed(d, 0);
  out->mag_new = !have_previous || block_changed(d, 6);
  out->gyro_new = !have_previous || block_changed(d, 12);

  for (int i = 0; i < 18; i++) {
    previous_raw[i] = d[i];
  }
  have_previous = true;
  return true;
}
