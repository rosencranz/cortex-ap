/**===========================================================================
 *
 * $HeadURL: $
 * $Revision: $
 * $Date:  $
 * $Author: $
 *
 * @brief  Navigation header file
 *
 * @file
 *
 * Change: 
 *         bool replaced with bool_t
 *
 *============================================================================*/

/*--------------------------------- Definitions ------------------------------*/

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*------------------------------------ Types ---------------------------------*/

typedef struct {    /* waypoint structure */
    float lon;      /* longitude */
    float lat;      /* latitude */
    float alt;      /* altitude */
} STRUCT_WPT;

/*---------------------------------- Constants -------------------------------*/

/*----------------------------------- Globals --------------------------------*/

/*---------------------------------- Interface -------------------------------*/

void     Nav_Load_Waypoints  ( void );
void     Nav_Store_Waypoints ( void );
void     Nav_Save_Home       ( void );
void     Navigation          ( void );
float    Nav_Altitude        ( void );
float    Nav_Bearing_Rad     ( void );
float    Nav_Alt_Error       ( void );
uint16_t Nav_Distance        ( void );
uint16_t Nav_Wpt_Number_Get  ( void );
bool_t   Nav_Wpt_Number_Set  ( uint16_t num );
uint16_t Nav_Wpt_Index       ( void );
uint16_t Nav_Wpt_Altitude    ( void );
void     Nav_Wpt_Get         ( uint16_t index, STRUCT_WPT *wpt );
void     Nav_Wpt_Set         ( uint16_t index, STRUCT_WPT *wpt );


