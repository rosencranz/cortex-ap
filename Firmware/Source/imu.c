/**===========================================================================+
 *
 * $HeadURL: $
 * $Revision: $
 * $Date: 01/11/2014 $
 * $Author: Lorenzo Fraccaro $
 *
 * @brief imu driver
 *
 * @file
 *
 *  Change: ADXL345 set to fullscale range, added sensor saturation detect
 *
 *
 *============================================================================*/

#include "ch.h"
#include "hal.h"

#include "imu.h"
#include "config.h"
#include "adxl345.h"
#include "l3g4200d.h"

/* autoincrement bit */
#define AUTO_INCREMENT_BIT  (1 << 7)

#define MAX_ACCEL           500  // 7500
#define MAX_ROTATION        5000 // 25000

/* I2C1 */
static const I2CConfig i2cfg1 = {
    OPMODE_I2C,
    100000,
    STD_DUTY_CYCLE,
};

static i2cflags_t errors = 0;
static uint8_t saturation = 0;

/* buffers */
static uint8_t imu_rx_data[IMU_RX_DEPTH];
static uint8_t imu_tx_data[IMU_TX_DEPTH];

/* offsets of sensor data */
static int16_t imu_offset[6] = {0,0,0,0,0,0};

/* sign of sensor data */
static const int16_t imu_sign[6] = {
    -1,     /* acceleration X, must be positive forward */
     1,     /* acceleration Y, must be positive rightward */
     1,     /* acceleration Z, must be positive downward */
     1,	    /* roll rate, must be positive when right wing lowers */
    -1,     /* pitch rate, must be positive when tail lowers */
    -1      /* yaw rate, must be positive when turning right */
};

/*----------------------------------------------------------------------------
 *
 * @brief   Initialization of L3G4200D and ADXL345
 * @return  -
 * @remarks -
 *
 *---------------------------------------------------------------------------*/
bool Init_IMU ( void ) {

  uint8_t i, j;
  int16_t * p_sensor_data;
  msg_t status = RDY_OK;
  systime_t tmo = MS2ST(4);

  i2cStart(&I2CD1, &i2cfg1);

  /* tune ports for I2C1*/
  palSetPadMode(IOPORT2, 6, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
  palSetPadMode(IOPORT2, 7, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);

  chThdSleepMilliseconds(100);  /* Just to be safe. */

  /*------------------------------- L3G4200D -------------------------------*/
  /* set the ODR, bandwith, enable axes, NORMAL MODE */
  imu_tx_data[0] = CTRL_REG1 | AUTO_INCREMENT_BIT;/* first register address */
  imu_tx_data[1] = (XEN | YEN | ZEN | PD);    /* CTRL_REG1: 100 Hz, 12.5 BW, all axes, normal */
  imu_tx_data[2] = 0;                         /* CTRL_REG2: default value */
  imu_tx_data[3] = 0;                         /* CTRL_REG3: default value */
  imu_tx_data[4] = FULLSCALE_2000 << 4;       /* CTRL_REG4: set 2000 deg/s range */
  imu_tx_data[5] = 0;                         /* CTRL_REG5: disable FIFO */

  /* sending */
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, L3G4200_ADDR, imu_tx_data, 6, imu_rx_data, 0, tmo);
  i2cReleaseBus(&I2CD1);

  if (status != RDY_OK){
    errors = i2cGetErrors(&I2CD1);
  }

  /*-------------------------------- ADXL345 --------------------------------*/
  /* disable FIFO */
  imu_tx_data[0] = FIFO_CTL;      /* FIFO_CTL register address */
  imu_tx_data[1] = 0;             /* BYPASS_MODE */

  /* sending */
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, ADXL345_ADDR, imu_tx_data, 2, imu_rx_data, 0, tmo);
  i2cReleaseBus(&I2CD1);
  if (status != RDY_OK){
    errors = i2cGetErrors(&I2CD1);
  }

  /* set full scale range */
  imu_tx_data[0] = DATA_FORMAT;   /* DATA_FORMAT register address */
  imu_tx_data[1] = ADXL_FULL_RES | RANGE_16G;   /* full resolution, 16 g */

  /* sending */
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, ADXL345_ADDR, imu_tx_data, 2, imu_rx_data, 0, tmo);
  i2cReleaseBus(&I2CD1);
  if (status != RDY_OK){
    errors = i2cGetErrors(&I2CD1);
  }

  /* set output rate and start measure */
  imu_tx_data[0] = BW_RATE | AUTO_INCREMENT_BIT;  /* BW_RATE register address */
  imu_tx_data[1] = RATE_100HZ;    /* 100 Hz */
  imu_tx_data[2] = MEASURE;       /* POWER_CTL register address, start measure */

  /* sending */
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, ADXL345_ADDR, imu_tx_data, 3, imu_rx_data, 0, tmo);
  i2cReleaseBus(&I2CD1);
  if (status != RDY_OK){
    errors = i2cGetErrors(&I2CD1);
  }

  /* Compute sensor offsets */
  for (i = 0; i < 16; i++) {
    chThdSleepMilliseconds(32);

  /*-------------------------------- ADXL345 --------------------------------*/

    imu_tx_data[0] = DATAX0 | AUTO_INCREMENT_BIT; /* register address */
    i2cAcquireBus(&I2CD1);
    status = i2cMasterTransmitTimeout(&I2CD1, ADXL345_ADDR, imu_tx_data, 1, imu_rx_data, 6, tmo);
    i2cReleaseBus(&I2CD1);
    if (status != RDY_OK) {
      errors = i2cGetErrors(&I2CD1);
    }

  /*------------------------------- L3G4200D -------------------------------*/

    imu_tx_data[0] = OUT_X_L | AUTO_INCREMENT_BIT; /* register address */
    i2cAcquireBus(&I2CD1);
    status = i2cMasterTransmitTimeout(&I2CD1, L3G4200_ADDR, imu_tx_data, 1, &imu_rx_data[6], 6, tmo);
    i2cReleaseBus(&I2CD1);
    if (status != RDY_OK) {
      errors = i2cGetErrors(&I2CD1);
    }

    p_sensor_data = (int16_t *)imu_rx_data;
    for (j = 0; j < 6; j++) {                   /* accumulate */
      imu_offset[j] += *p_sensor_data++;
    }
  }
  for (j = 0; j < 6; j++) {                     /* average */
    imu_offset[j] = imu_offset[j] / 16;
  }

  return 0;
}


/*----------------------------------------------------------------------------
 *
 * @brief   Read acceleration and rotation
 * @return  pointer to 6 elements array containing IMU data
 * @param   -
 * @remarks -
 *
 *---------------------------------------------------------------------------*/
int16_t * Request_IMU_Data( void ) {

  uint8_t j;
  int16_t * p_sensor_data;
  msg_t status = RDY_OK;
  systime_t tmo = MS2ST(4);

  /*-------------------------------- ADXL345 --------------------------------*/

  imu_tx_data[0] = DATAX0 | AUTO_INCREMENT_BIT;     /* register address */
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1,         /* I2C driver 1 */
                                    ADXL345_ADDR,   /* I2C address of sensor */
                                    imu_tx_data,    /* pointer to register buffer */
                                    1,              /* bytes to transmit */
                                    imu_rx_data,    /* pointer to data buffer */
                                    6,              /* bytes to read */
                                    tmo);           /* timeout */
  i2cReleaseBus(&I2CD1);
  if (status != RDY_OK) {
    errors = i2cGetErrors(&I2CD1);
  }

  /*------------------------------- L3G4200D -------------------------------*/

  imu_tx_data[0] = OUT_X_L | AUTO_INCREMENT_BIT;    /* register address */
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1,         /* I2C driver 1 */              
                                    L3G4200_ADDR,   /* I2C address of sensor */     
                                    imu_tx_data,    /* pointer to register buffer */
                                    1,              /* bytes to transmit */                           
                                    &imu_rx_data[6],/* pointer to data buffer */     
                                    6,              /* bytes to read */             
                                    tmo);           /* timeout */                   
  i2cReleaseBus(&I2CD1);
  if (status != RDY_OK) {
    errors = i2cGetErrors(&I2CD1);
  }

  /* Check saturation, add offset, correct sign */
  p_sensor_data = (int16_t *)imu_rx_data;
  saturation = 0;
  for (j = 0; j < 3; j++) {                     /* ADXL345 */
    *p_sensor_data -= imu_offset[j];            /* strip offset */
    *p_sensor_data *= imu_sign[j];              /* correct sign */
    if (j == 2) {                               /* z acceleration */
      *p_sensor_data += (int16_t)GRAVITY;       /* add gravity */
    }
    if ((*p_sensor_data < -MAX_ACCEL) ||        /* check saturation */
        (*p_sensor_data >  MAX_ACCEL)) {
       saturation |= ACCEL_SATURATED;
    } else {
       saturation &= ~ACCEL_SATURATED;
    }

    p_sensor_data++;
  }
  for (j = 3; j < 6; j++) {                     /* L3G4200D */
    *p_sensor_data -= imu_offset[j];            /* strip offset */
    *p_sensor_data *= imu_sign[j];              /* correct sign */
    if ((*p_sensor_data < -MAX_ROTATION) ||     /* check saturation */
        (*p_sensor_data >  MAX_ROTATION)) {
      saturation |= GYRO_SATURATED;
    } else {
      saturation &= ~GYRO_SATURATED;
    }
    p_sensor_data++;
  }

  return (int16_t *)imu_rx_data;
}

/*----------------------------------------------------------------------------
 *
 * @brief   Read saturation status 
 * @return  sensor saturation status
 * @param   -
 * @remarks -
 *
 *---------------------------------------------------------------------------*/

uint8_t Get_IMU_Saturation(void) {
    return saturation;
}
