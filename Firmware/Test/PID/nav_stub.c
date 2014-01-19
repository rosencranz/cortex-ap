/**============================================================================+
 *
 * $HeadURL: $
 * $Revision: $
 * $Date:  $
 * $Author: $
 *
 * @brief navigation task
 *
 * @file
 *
 * Change: 
 *
 *============================================================================*/

#include "ch.h"
#include "config.h"
#include "nav.h"

/*--------------------------------- Definitions ------------------------------*/

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

static float f_alt_error = 0.0f;   /* altitude error [m] */
static float f_bearing = 0.0f;     /* angle to destination [�] */

/*--------------------------------- Prototypes -------------------------------*/


/**----------------------------------------------------------------------------
 *
 * @brief   Get computed bearing [�]
 * @param   -
 * @return  bearing angle in radian, between 0 and 2 * PI
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
float Nav_Bearing_Rad ( void ) {
  return f_bearing;
}


/**----------------------------------------------------------------------------
 *
 * @brief   Get altitude error
 * @param   -
 * @return  altitude error
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
float Nav_Alt_Error ( void ) {
  return f_alt_error;
}
