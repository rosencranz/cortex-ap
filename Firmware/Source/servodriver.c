//============================================================================+
//
// $RCSfile: $
// $Revision: $
// $Date: $
// $Author: $
//
/// \brief  Servo driver
//  CHANGES MODE_STABILIZE: aileron servo = aileron channel + DCM roll angle
//
//============================================================================*/

#include "stm32f10x.h"
#include "led.h"
#include "ppmdriver.h"
#include "servodriver.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_STATIC
#undef VAR_STATIC
#endif
#define VAR_STATIC static
#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL

#define PRESCALER      23
#define PERIOD         19999
#define SERVO_MIN      900   ///< Absolute minimum pulse length (0.9 ms).
#define SERVO_MAX      2100  ///< Absolute maximum pulse length (2.1 ms).
#define SERVO_NEUTRAL  1500  ///< Pulse length of servo neutral position (1.5 ms).

#define SATURATE(p)    if (p < SERVO_MIN) { p = SERVO_MIN; } \
                       if (p > SERVO_MAX) { p = SERVO_MAX; }

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

/*
VAR_STATIC float fElevatorGain = 500.0f;  //!< Elevator servo conversion gain
VAR_STATIC float fRudderGain = 500.0f;    //!< Rudder servo conversion gain
VAR_STATIC float fAileronGain = 500.0f;   //!< Aileron servo conversion gain
*/
/*--------------------------------- Prototypes -------------------------------*/

///----------------------------------------------------------------------------
///
/// \brief   Timer 3 initialization for servo output
/// \return  -
/// \remarks Generate 4 PWM signals
///          TIM3CLK = 24 MHz, Prescaler = 23, TIM3 counter clock = 1 MHz
///          TIM3 ARR Register = 19999 =>
///          TIM3 Frequency = TIM3 counter clock/(ARR + 1) = 50 Hz.
///
///----------------------------------------------------------------------------
void Servo_Init(void) {

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

  /* Clear current configuration */
  TIM_DeInit(TIM3);

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = PRESCALER;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = SERVO_NEUTRAL;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = SERVO_NEUTRAL;

  TIM_OC2Init(TIM3, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = SERVO_NEUTRAL;

  TIM_OC3Init(TIM3, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = SERVO_NEUTRAL;

  TIM_OC4Init(TIM3, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}

///----------------------------------------------------------------------------
///
/// \brief   Set servo position.
/// \param   servo: servo identifier
/// \param   position: desired position
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void Servo_Set(SERVO_TYPE servo, int16_t position) {

   uint8_t ucMode;
   static uint8_t ucBlink = 0;

   ucMode = PPMGetMode();
   SATURATE(position);

   switch (servo) {
      case SERVO_AILERON :
         switch (ucMode) {
            case MODE_RTL:
               LEDOn(RED);
            break;
            case MODE_STABILIZE:
               position -= SERVO_NEUTRAL;
               position += PPMGetChannel(AILERON_CHANNEL);
               SATURATE(position);
               if (++ucBlink >= 5) {
                  ucBlink = 0;
                  LEDToggle(RED);
               }
            break;
            case MODE_AUTO:
               if (++ucBlink == 10) {
                  ucBlink = 0;
                  LEDToggle(RED);
               }
            break;
            default:
               LEDOff(RED);
               position = PPMGetChannel(AILERON_CHANNEL);
            break;
          }
          TIM_SetCompare1(TIM3, position);
       break;
       case SERVO_RUDDER:
           TIM_SetCompare2(TIM3, position);
       break;
       case SERVO_ELEVATOR:
           TIM_SetCompare3(TIM3, position);
       break;
       case SERVO_THROTTLE:
           TIM_SetCompare4(TIM3, position);
       break;
       default:
       break;
   }
}

