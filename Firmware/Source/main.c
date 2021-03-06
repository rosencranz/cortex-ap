/**===========================================================================+
 *
 * @Author rosenkranz
 * @file main.c
 * @brief main 
 *
 * Changes: restored log thread
 *
 *============================================================================*/

#include <stdlib.h>

#include "ch.h"
#include "hal.h"

#include "ff.h"
#include "config.h"
#include "dcm.h"
#include "imu.h"
#include "gps.h"
#include "rc.h"
#include "baro.h"
#include "servo.h"
#include "control.h"
#include "nav.h"
#include "mavlink.h"
#include "log.h"

/*---------------------------------- Globals ---------------------------------*/

/* MMC driver instance. */
MMCDriver MMCD1;

/* FS mounted and ready.*/
bool_t fs_ready = FALSE;

/*----------------------------------- Locals ---------------------------------*/

/* FS object. */
static FATFS MMC_FS;

/* Maximum speed SPI configuration (18MHz, CPHA=0, CPOL=0, MSb first).*/
static SPIConfig hs_spicfg = {NULL, IOPORT2, GPIOB_SPI2NSS, 0};

/* Low speed SPI configuration (281.250kHz, CPHA=0, CPOL=0, MSb first).*/
static SPIConfig ls_spicfg = {NULL, IOPORT2, GPIOB_SPI2NSS,
                              SPI_CR1_BR_2 | SPI_CR1_BR_1};

/* MMC/SD over SPI driver configuration.*/
static MMCConfig mmccfg = {&SPID2, &ls_spicfg, &hs_spicfg};

static int16_t * p_sensor_data;

static volatile float f_temp;

/*--------------------------------- Prototypes -------------------------------*/

/*---------------------------------- Functions -------------------------------*/

/*----------------------------------------------------------------------------
 *
 * @brief   Initialize SD card
 * @return  -
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
static void SD_Init ( void ) {

  FRESULT res;

  /* Initializes the MMC driver to work with SPI2. */
  palSetPadMode(IOPORT2, GPIOB_SPI2NSS, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPad(IOPORT2, GPIOB_SPI2NSS);
  mmcObjectInit(&MMCD1);
  mmcStart(&MMCD1, &mmccfg);

  if (!mmcConnect(&MMCD1)) {
    /* MMC initialization and FS mount. */
    res = f_mount(0, &MMC_FS);
    if (res == FR_OK) {
      fs_ready = TRUE;
    }
  }
}

/*----------------------------------------------------------------------------
 *
 * @brief   Handle LEDs
 * @return  -
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
static void Led_Handler ( void ) {

   if (Gps_Status() == GPS_FIX) {             /* satellite fix */
      palTogglePad(IOPORT3, GPIOC_LED4);      /* toggle blue LED */
   } else {                                   /* no fix */
      palSetPad(IOPORT3, GPIOC_LED4);         /* turn on blue LED */
   }
   palTogglePad(IOPORT3, GPIOC_LED3);         /* always toggle red LED */
}

/*----------------------------------------------------------------------------
 *
 * @brief   Log thread
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
static WORKING_AREA(wa_Log_Thread, 320);
static msg_t Log_Thread(void *arg) {

  chRegSetThreadName("Log_Thread");
  (void)arg;

  /*
   * initialize log file
   */
  Log_Init();

  while (TRUE) {
    chThdSleepMilliseconds(20);

    if (MODE_STAB == Get_RC_Mode()) {
      Log_Write_Str("1, ", 3);
    } else {
      Log_Write_Str("0, ", 3);
    }
    f_temp = AHRS_Pitch_Rad( );
    Log_Write_Var((uint8_t *) &f_temp, sizeof (f_temp));
    f_temp = AHRS_Roll_Rad( );
    Log_Write_Var((uint8_t *) &f_temp, sizeof (f_temp));
    f_temp = AHRS_Yaw_Rad( );
    Log_Write_Var((uint8_t *) &f_temp, sizeof (f_temp));
    Log_Write_Ch('\n');

  }
  return 0;
}

/*----------------------------------------------------------------------------
 *
 * @brief   Attitude thread
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
static WORKING_AREA(wa_AHRS_Thread, 128);
static msg_t AHRS_Thread(void *arg) {

  chRegSetThreadName("AHRS_Thread");
  (void)arg;

  while (TRUE) {
    chThdSleepMilliseconds(20);
    p_sensor_data = Request_IMU_Data();
    MatrixUpdate(p_sensor_data);      /* compute DCM */
    CompensateDrift();                /* compensate	drift */
    Normalize();                      /* normalize DCM */
    Control();                        /* control loop */
  }
  return 0;
}

/*----------------------------------------------------------------------------
 *
 * @brief   Barometer reading thread
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
static WORKING_AREA(wa_Baro_Thread, 128);
static msg_t Baro_Thread(void *arg) {

  chRegSetThreadName("Baro_Thread");
  (void)arg;

  while (TRUE) {
    chThdSleepMilliseconds(300);
    Baro_Handler();
    Led_Handler();
  }
  return 0;
}

/*----------------------------------------------------------------------------
 *
 * @brief   Navigation thread
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
static WORKING_AREA(wa_Nav_Thread, 320);
static msg_t Nav_Thread(void *arg) {
  EventListener elGPSdata;
  flagsmask_t flags;

  chRegSetThreadName("Nav_Thread");
  (void)arg;

  Nav_Load_Waypoints();

  chEvtRegisterMask((EventSource *)chnGetEventSource(&SD2), &elGPSdata, EVENT_MASK(1));
  while (TRUE) {
     (void)chEvtWaitOneTimeout(EVENT_MASK(1), MS2ST(500));
     flags = chEvtGetAndClearFlags(&elGPSdata);
     if (flags & CHN_INPUT_AVAILABLE) {
        GPS_Parse();
     }
     Navigation();
     Nav_Store_Waypoints();
  }
  return 0;
}

/*----------------------------------------------------------------------------
 *
 * @brief   Telemetry thread
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
static WORKING_AREA(wa_Telemetry_Thread, 128);
static msg_t Telemetry_Thread(void *arg) {

  uint32_t ul_cycles = 0;

  chRegSetThreadName("Telemetry_Thread");
  (void)arg;

#if (SIMULATOR != SIM_NONE)

  while (TRUE) {
    chThdSleepMilliseconds(20);
    Simulator_Send_Controls();          /* update simulator controls */
    Simulator_Parse();                  /* parse simulator data */
    switch (++ul_cycles) {
      case 8:
      case 16:
      case 24:
      case 32:
        Simulator_Send_DCM();           /* send attitude */
        break;

      case 40:
        ul_cycles = 0;                  /* reset cycle counter */
        Simulator_Send_Waypoint();      /* send waypoint information */
        break;

      default :
        break;
    }
  }

#elif defined TELEMETRY_MAVLINK

/*  global_data_reset_param_defaults(); */ /* Load default parameters as fallback */

  while (TRUE)  {
    chThdSleepMilliseconds(1);  /* Use any wait function, better not use sleep */
    Mavlink_Receive();          /* 1 KHz: process parameter request, if any */
    if (++ul_cycles == 20) {    /* 50 Hz: */
      Mavlink_Stream_Send();    /* Send data streams */
      Mavlink_Queued_Send();    /* Send parameters, if requested */
      ul_cycles = 0;
    }
  }
#endif
}

/*----------------------------------------------------------------------------
 *
 * @brief   Entry point 
 * @remarks The main() function is already a thread in the system on entry.
 *
 *----------------------------------------------------------------------------*/
int main(void) {

  halInit();
  chSysInit();

  chThdSleepMilliseconds(200);
  Servo_Init();
  IMU_Init();
  Baro_Init();
  RC_Init();
  GPS_Init();
  DCM_Init();
  SD_Init();
  Control_Init();
  Telemetry_Init();

  /*
   * Create log thread.
   * Duration ??? ms
   * Period 20 ms
   */
  (void)chThdCreateStatic(wa_Log_Thread, sizeof(wa_Log_Thread), HIGHPRIO, Log_Thread, NULL);

  /*
   * Create AHRS thread.
   * Duration 2.34 ms
   * Period 20 ms
   */
  (void)chThdCreateStatic(wa_AHRS_Thread, sizeof(wa_AHRS_Thread), HIGHPRIO, AHRS_Thread, NULL);

  /*
   * Creates navigation thread.
   * Duration 0.050 - 0.086 us
   * Period of incoming characters (~ 10 ms)
   * Timeout 500 ms
   */
  (void)chThdCreateStatic(wa_Nav_Thread, sizeof(wa_Nav_Thread), HIGHPRIO - 1, Nav_Thread, NULL);

  /*
   * Creates telemetry thread.
   * Duration 0.150 ms - 7 ms
   * Period 20 ms
   */
  (void)chThdCreateStatic(wa_Telemetry_Thread, sizeof(wa_Telemetry_Thread), HIGHPRIO - 2, Telemetry_Thread, NULL);

  /*
   * Creates barometer thread.
   * Duration 31.6 ms
   * Period 301 ms
   */
  (void)chThdCreateStatic(wa_Baro_Thread, sizeof(wa_Baro_Thread), HIGHPRIO - 3, Baro_Thread, NULL);

  /*
   * RM schedulability test
   * (2.34 / 20) + (31.6 / 301) + (0.086 / 10) + (3.6 / 20) < 4 * (2 ^ 1/4 - 1)
   * 0.117 + 0.104 + 0.008 + 0.18 < 0.756
   * 0.409 < 0.756
   */

  /* main loop that does nothing */
  while (TRUE) {
    chThdSleepMilliseconds(500);
  }

  return 0;
}
