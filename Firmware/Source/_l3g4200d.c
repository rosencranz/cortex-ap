///----------------------------------------------------------------------------
///
/// File Name: L3G4200D_Driver.c
/// $Revision:$
/// $Date:$
/// L3G4200D driver file
/// Changes: disabled fifo, watermark and interrupts in initialization
///
///----------------------------------------------------------------------------

#include <stdlib.h>

#include "ch.h"
#include "hal.h"
#include "l3g4200d.h"

/* buffers */
static uint8_t gyro_rx_data[GYRO_RX_DEPTH];
static uint8_t gyro_tx_data[GYRO_TX_DEPTH];

static i2cflags_t errors = 0;

static int16_t rotation_x = 0;
static int16_t rotation_y = 0;
static int16_t rotation_z = 0;

///----------------------------------------------------------------------------
///
/// @brief   Read rotation
/// @return  -
/// @param
/// @remarks -
///
///----------------------------------------------------------------------------
void Request_Gyro_Data( void ) {

  msg_t status = RDY_OK;
  systime_t tmo = MS2ST(4);

  gyro_tx_data[0] = OUT_X_L | AUTO_INCREMENT_BIT; /* register address */

  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, L3G4200_ADDR, gyro_tx_data, 1, gyro_rx_data, 6, tmo);
  i2cReleaseBus(&I2CD1);

  if (status != RDY_OK){
    errors = i2cGetErrors(&I2CD1);
  }

  rotation_x = gyro_rx_data[0] + (gyro_rx_data[1] << 8);
  rotation_y = gyro_rx_data[2] + (gyro_rx_data[3] << 8);
  rotation_z = gyro_rx_data[4] + (gyro_rx_data[5] << 8);
}


///----------------------------------------------------------------------------
///
/// @brief   Initialization of L3G4200D gyroscope
/// @return  -
/// @remarks -
///
///----------------------------------------------------------------------------
bool Init_L3G4200 ( void )
{
  msg_t status = RDY_OK;
  systime_t tmo = MS2ST(4);

  /* set the ODR, bandwith, enable axes, NORMAL MODE */
  gyro_tx_data[0] = CTRL_REG1 | AUTO_INCREMENT_BIT;/* CTRL_REG1 register address */
  gyro_tx_data[1] = (XEN | YEN | ZEN | PD);    /* CTRL_REG1: 100 Hz, 12.5 BW, all axes, normal */
  gyro_tx_data[2] = 0;                         /* CTRL_REG2: default value */
  gyro_tx_data[3] = 0;                         /* CTRL_REG3: default value */
  gyro_tx_data[4] = FULLSCALE_2000;            /* CTRL_REG4: set 2000 deg/s range */
  gyro_tx_data[5] = 0;                         /* CTRL_REG5: disable FIFO */

  /* sending */
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, L3G4200_ADDR, gyro_tx_data, 6, gyro_rx_data, 0, tmo);
  i2cReleaseBus(&I2CD1);

  if (status != RDY_OK){
    errors = i2cGetErrors(&I2CD1);
  }

  return 0;
}

