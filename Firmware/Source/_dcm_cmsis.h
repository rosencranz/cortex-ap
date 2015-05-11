/**===========================================================================
 *
 * $HeadURL: $
 * $Revision: $
 * $Date:  $
 * $Author: $
 *
 * @brief Direction Cosine Matrix calculations header file
 *
 * @file
 *
 * Change
 *
 *============================================================================*/

/*--------------------------------- Definitions ------------------------------*/

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*---------------------------------- Interface -------------------------------*/

void Init_DCM( void );
void Normalize( void );
void CompensateDrift( void );
void MatrixUpdate( const int16_t * sensor );
float32_t AHRS_Pitch_Rad( void );
float32_t AHRS_Roll_Rad( void );
float32_t AHRS_Yaw_Rad( void );


