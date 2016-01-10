/*===========================================================================+
 *
 * @brief imu driver
 *
 * @file
 *
 * Change: 
 *         bool replaced with bool_t
 *
 *============================================================================*/

#ifndef IMU_H_
#define IMU_H_

#define IMU_RX_DEPTH    12
#define IMU_TX_DEPTH    6

#define ACCEL_SATURATED (1 << 0)
#define GYRO_SATURATED  (1 << 1)

bool_t IMU_Init(void);
int16_t * Request_IMU_Data( void );
uint8_t Get_IMU_Saturation(void);

#endif /* IMU_H_ */
