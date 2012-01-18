//============================================================================+
//
// $RCSfile: $ (SOURCE FILE)
// $Revision: $
// $Date: $
// $Author: $
//
/// \brief
/// PPM input driver header file
///
//  CHANGES Added ELEVATOR_CHANNEL
//
//============================================================================*/

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_STATIC
#undef VAR_STATIC
#endif
#define VAR_STATIC static
#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL

#define PPM_SIGNAL_OK   6
#define PPM_NO_SIGNAL   0

#define AILERON_CHANNEL     1   // Aileron control channel
#define ELEVATOR_CHANNEL    2   // Elevator control channel
#define MODE_CHANNEL        4   // Mode selection channel
#define RC_CHANNELS         6   ///< Number of RC channels. Modify according to RC type

#define MODE_UNDEFINED  0
#define MODE_STABILIZE  1
#define MODE_AUTO       2
#define MODE_MANUAL     3
#define MODE_RTL        4

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

/*--------------------------------- Prototypes -------------------------------*/

void PPM_Init(void);
uint8_t PPMSignalStatus(void);
uint8_t PPMGetMode(void);
uint16_t PPMGetChannel(uint8_t ucChannel);

