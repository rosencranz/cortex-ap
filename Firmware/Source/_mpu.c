/*
*/

/**
 * "read through write" paradigm.
 * not standard, but most of I2C devices use this paradigm.
 * You must write to device reading address, send restart to bus,
 * and then begin reading process.
 */

#include <stdlib.h>

#include "ch.h"
#include "hal.h"
#include "mpu.h"

/* buffers */
static uint8_t mpu_rx_data[ACCEL_RX_DEPTH];
static uint8_t mpu_tx_data[ACCEL_TX_DEPTH];

static i2cflags_t errors = 0;

static int16_t acceleration_x = 0;
static int16_t acceleration_y = 0;
static int16_t acceleration_z = 0;
static int16_t rotation_x = 0;
static int16_t rotation_y = 0;
static int16_t rotation_z = 0;


/**
 * Init function. Here we will also start personal serving thread.
 */
int init_mpu(void) {
  msg_t status = RDY_OK;
  systime_t tmo = MS2ST(4);

  /* configure clock */
  mpu_tx_data[0] = MPU6050_RA_PWR_MGMT_1;   /* register address */
  mpu_tx_data[1] = MPU6050_CLOCK_PLL_XGYRO;

  /* sending */
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, MPU_ADDR, mpu_tx_data, 2, mpu_rx_data, 0, tmo);
  i2cReleaseBus(&I2CD1);

  if (status != RDY_OK){
    errors = i2cGetErrors(&I2CD1);
  }

  /* configure accel & gyro full scale */
  mpu_tx_data[0] = MPU6050_RA_GYRO_CONFIG | AUTO_INCREMENT_BIT;   /* register address */
  mpu_tx_data[1] = MPU6050_GYRO_FS_250;
  mpu_tx_data[2] = MPU6050_ACCEL_FS_2;

  /* sending */
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, MPU_ADDR, mpu_tx_data, 3, mpu_rx_data, 0, tmo);
  i2cReleaseBus(&I2CD1);

  if (status != RDY_OK){
    errors = i2cGetErrors(&I2CD1);
  }

  return 0;
}


/**
 *
 */
void request_accel_data(void){
  msg_t status = RDY_OK;
  systime_t tmo = MS2ST(4);

  mpu_tx_data[0] = MPU6050_RA_ACCEL_XOUT_H | AUTO_INCREMENT_BIT; /* register address */
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, MPU_ADDR, mpu_tx_data, 1, mpu_rx_data, 6, tmo);
  i2cReleaseBus(&I2CD1);

  if (status != RDY_OK){
    errors = i2cGetErrors(&I2CD1);
  }

  acceleration_x = mpu_rx_data[0] + (mpu_rx_data[1] << 8);
  acceleration_y = mpu_rx_data[2] + (mpu_rx_data[3] << 8);
  acceleration_z = mpu_rx_data[4] + (mpu_rx_data[5] << 8);
}


/**
 *
 */
void request_gyro_data(void){
  msg_t status = RDY_OK;
  systime_t tmo = MS2ST(4);

  mpu_tx_data[0] = MPU6050_RA_GYRO_XOUT_H | AUTO_INCREMENT_BIT; /* register address */
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, MPU_ADDR, mpu_tx_data, 1, mpu_rx_data, 6, tmo);
  i2cReleaseBus(&I2CD1);

  if (status != RDY_OK){
    errors = i2cGetErrors(&I2CD1);
  }

  rotation_x = mpu_rx_data[0] + (mpu_rx_data[1] << 8);
  rotation_y = mpu_rx_data[2] + (mpu_rx_data[3] << 8);
  rotation_z = mpu_rx_data[4] + (mpu_rx_data[5] << 8);
}

