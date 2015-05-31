/*===========================================================================+
 *
 * $HeadURL: $
 * $Revision: $
 * $Date: 01711/2014 $
 * $Author: Lorenzo Fraccaro $
 *
 * @brief imu driver
 *
 * @file
 *
 * Change: added sensor saturation detect
 *
 *============================================================================*/

#ifndef IMU_H_
#define IMU_H_

#define IMU_RX_DEPTH    12
#define IMU_TX_DEPTH    6

#define ACCEL_SATURATED (1 << 0)
#define GYRO_SATURATED  (1 << 1)

bool IMU_Init(void);
int16_t * Request_IMU_Data( void );
uint8_t Get_IMU_Saturation(void);

#endif /* IMU_H_ */
