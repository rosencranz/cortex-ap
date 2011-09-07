//============================================================================+
//
// $RCSfile: $
// $Revision: $
// $Date: $
// $Author: $
//
/// \brief  Servo driver header file
//  CHANGES Moved servo management to servodriver.c
//
//============================================================================*/

#include "stm32f10x.h"
#include "STM32vldiscovery.h"
#include "servodriver.h"

/** @addtogroup cortex-ap
  * @{
  */

/** @addtogroup main
  * @{
  */

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_STATIC
#undef VAR_STATIC
#endif
#define VAR_STATIC static
#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC int16_t Servo_Position = 1500;
VAR_STATIC int16_t Servo_Delta = 10;
VAR_STATIC ErrorStatus HSEStartUpStatus;

/*--------------------------------- Prototypes -------------------------------*/

void RCC_Configuration(void);
void GPIO_Configuration(void);
void Delay(__IO uint32_t nCount);


///----------------------------------------------------------------------------
///
/// \brief   main
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file     */

  /* System Clocks Configuration */
  RCC_Configuration();

  /* GPIO Configuration */
  GPIO_Configuration();

  /* Initialize Leds LD3 and LD4 mounted on STM32VLDISCOVERY board */
  STM32vldiscovery_LEDInit(LED3);
  STM32vldiscovery_LEDInit(LED4);

  /* Initialize PWM timers as servo outputs */
  Servo_Init();

  while (1)
  {
    if (Servo_Position > 1999) {
      Servo_Delta = -10;
    } else if (Servo_Position < 999) {
      Servo_Delta = 10;
    }
    Servo_Position += Servo_Delta;
    Servo_Set(SERVO_RUDDER, Servo_Position);

    if (Servo_Delta == 10) {
      STM32vldiscovery_LEDOn(LED3);       /* Turn on LD3 */
      Delay(0x57FE);                      /* Insert delay */
      STM32vldiscovery_LEDOff(LED3);      /* Turn off LD3 */
      Delay(0x57FE);                      /* Insert delay */
    } else {
      STM32vldiscovery_LEDOn(LED4);       /* Turn on LD4 */
      Delay(0x57FE);                      /* Insert delay */
      STM32vldiscovery_LEDOff(LED4);      /* Turn off LD4 */
      Delay(0x57FE);                      /* Insert delay */
    }
  }
}

///----------------------------------------------------------------------------
///
/// \brief   Inserts a delay time.
/// \param   nCount: time
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

///----------------------------------------------------------------------------
///
/// \brief   Configure the different system clocks.
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void RCC_Configuration(void)
{
  /* PCLK1 = HCLK/4 */
  RCC_PCLK1Config(RCC_HCLK_Div4);

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* GPIOA and GPIOB clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
}

///----------------------------------------------------------------------------
///
/// \brief   Configure pins.
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOA Configuration:TIM3 Channel 1, 2 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* GPIOC Configuration:TIM3 Channel 3, 4 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;

  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

#ifdef  USE_FULL_ASSERT
///----------------------------------------------------------------------------
///
/// \brief   Reports the name of the source file and the source line number
///          where the assert_param error has occurred.
/// \param   file: pointer to the source file name
/// \param   line: assert_param error line source number
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/*****END OF FILE****/
