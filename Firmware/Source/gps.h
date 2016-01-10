/**===========================================================================
 *
 * @brief GPS header file
 *
 * @file
 *
 * Change:
 *         added function Gps_Set_Date()
 *         function Gps_Status() renamed Gps_Fix()
 *
 *============================================================================*/

/*--------------------------------- Definitions ------------------------------*/

#define GPS_BUFFER_LENGTH   96      /* length of buffer for USART */

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*------------------------------------ Types ---------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*----------------------------------- Globals --------------------------------*/

/*---------------------------------- Interface -------------------------------*/

    void GPS_Init      ( void );
    void GPS_Parse     ( void );
 uint8_t Gps_Fix       ( void );
  bool_t Gps_Set_Date  ( struct tm * timp );
uint16_t Gps_Speed_Kt  ( void );
uint16_t Gps_Alt_M     ( void );
uint16_t Gps_COG_Deg   ( void );
   float Gps_Latitude  ( void );
   float Gps_Longitude ( void );
