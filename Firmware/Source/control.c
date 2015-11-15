/**===========================================================================+
 *
 * $HeadURL: $
 * $Revision: $
 * $Date:  $
 * $Author: $
 *
 * @file
 *
 * @brief aircraft control
 *
 * Reference point for pitch and roll stabilization is computed as
 *
 *     -asinf(DCM[...][...])
 *
 * instead of
 *
 *     acosf(DCM[...][...]) - PI/2
 *
 * This returns an angle value that's consistent with angle convention for
 * roll and pitch angles, without need to either subtract PI/2 from reference
 * point or to add PI/2 to the set point.
 *
 * @todo
 * replace SERVO_NEUTRAL with RC command center position value
 *
 * Change: increased gear ratio constant for camera tilt
 *
 *============================================================================*/

#include "ch.h"
#include "hal.h"
#include "math.h"

#include "config.h"
#include "dcm.h"
/* #include "simulator.h" */
#include "mavlink.h"
#include "nav.h"
#include "pid.h"
#include "servo.h"
#include "rc.h"
#include "control.h"

/** @addtogroup cortex_ap
  * @{
  */

/** @addtogroup control
  * @{
  */

/*--------------------------------- Definitions ------------------------------*/

#if (SIMULATOR == SIM_NONE)
  #define Get_Gain Telemetry_Get_Gain
#else
  #define Get_Gain Simulator_Get_Gain
#endif

/* RC elevator command [-500,500] to desired pitch [-45°,+45°] conversion factor */
#define ELEV_TO_PITCH   636.61f
/* RC aileron command [-500,500] to desired bank [-60°,60°] conversion factor */
#define AIL_TO_BANK     477.46f

/* Camera tilt gear ratio */
#define TILT_GEAR_RATIO 1.7f

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

static uint8_t uc_current_mode;
static int16_t i_aileron;                            /* aileron servo position */
static int16_t i_elevator;                           /* elevator servo position */
static int16_t i_throttle;                           /* throttle servo position */
static int16_t i_rudder;                             /* rudder servo position */
static xPID Roll_Pid;                                /* roll PID */
static xPID Pitch_Pid;                               /* pitch PID */
static xPID Nav_Pid;                                 /* navigation PID */
static float f_temp;                                 /* */
static float f_camera_slant;                         /* */
static float f_camera_tilt;                          /* */
static float f_ctrl_elevator;                        /* */
static float f_ctrl_aileron;                         /* */
static float f_ctrl_throttle = MINIMUMTHROTTLE;      /* commanded throttle */
static float f_ahrs_pitch = 0.0f;                    /* pitch */
static float f_ahrs_roll = 0.0f;                     /* roll */
static float f_ahrs_heading = 0.0f;                  /* heading */
static float f_nav_pitch = 0.0f;                     /*  */
static float f_nav_roll = 0.0f;                      /*  */
static float f_height_margin = HEIGHT_MARGIN;        /* altitude hold margin */
static float f_throttle_min = ALT_HOLD_THROTTLE_MIN; /* altitude hold min throttle */
static float f_throttle_max = ALT_HOLD_THROTTLE_MAX; /* altitude hold max throttle */

/*--------------------------------- Prototypes -------------------------------*/

__inline static void roll_control( void );
__inline static void pitch_control( void );
__inline static void direction_control( void );
__inline static void altitude_control( void );
__inline static void camera_control( void );

/*--------------------------------- Functions --------------------------------*/


/*----------------------------------------------------------------------------
 *
 * @brief   Initialize control loops.
 * @return  -
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
void Control_Init(void )
{
    Roll_Pid.fGain = 500.0f;            /* limit servo throw */
    Roll_Pid.fMin = -1.0f;              /* */
    Roll_Pid.fMax = 1.0f;               /* */
    Roll_Pid.fKp = ROLL_KP;             /* init gains with default values */
    Roll_Pid.fKi = ROLL_KI;             /* */
    Roll_Pid.fKd = ROLL_KD;             /* */

    Pitch_Pid.fGain = 500.0f;           /* limit servo throw */
    Pitch_Pid.fMin = -1.0f;             /* */
    Pitch_Pid.fMax = 1.0f;              /* */
    Pitch_Pid.fKp = PITCH_KP;           /* init gains with default values */
    Pitch_Pid.fKi = PITCH_KI;           /* */
    Pitch_Pid.fKd = PITCH_KD;           /* */

    Nav_Pid.fGain = DEGTORAD(NAV_BANK); /* limit bank angle during navigation */
    Nav_Pid.fMin = -1.0f;               /* */
    Nav_Pid.fMax = 1.0f;                /* */
    Nav_Pid.fKp = NAV_KP;               /* init gains with default values */
    Nav_Pid.fKi = NAV_KI;               /* */
    Nav_Pid.fKd = NAV_KD;               /* */

    PID_Init(&Roll_Pid);                /* initialize PIDs */
    PID_Init(&Pitch_Pid);
    PID_Init(&Nav_Pid);
}

/*----------------------------------------------------------------------------
 *
 * @brief   Control
 * @return  -
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
void Control ( void ) {

    static uint8_t uc_old_mode = MODE_MANUAL;
    float f_max_roll;

    /* update PID gains from telemetry */
#if (0)
    Pitch_Pid.fKp = Get_Gain(TEL_PITCH_KP);
    Pitch_Pid.fKi = Get_Gain(TEL_PITCH_KI);
    Roll_Pid.fKp = Get_Gain(TEL_ROLL_KP);
    Roll_Pid.fKi = Get_Gain(TEL_ROLL_KI);
    Nav_Pid.fKp = Get_Gain(TEL_NAV_KP);
    Nav_Pid.fKi = Get_Gain(TEL_NAV_KI);
    Nav_Pid.fGain = Get_Gain(TEL_NAV_BANK);
#endif
    f_max_roll = Get_Gain(TEL_NAV_BANK);

    /* update PID gains from RC auxiliary controls */
    Roll_Pid.fKp = (float)(Get_RC_Channel(KP_CHANNEL) - 1000) / 500.0f;
    Roll_Pid.fKi = (float)(Get_RC_Channel(KI_CHANNEL) - 1000) / 1000.0f;
	if (Roll_Pid.fKp < 0.0f ) { Roll_Pid.fKp = 0.0f; }
	if (Roll_Pid.fKi < 0.0f ) { Roll_Pid.fKi = 0.0f; }

    /* read aircraft attitude */
    f_ahrs_heading = AHRS_Yaw_Rad( );
    f_ahrs_pitch = AHRS_Pitch_Rad( );
    f_ahrs_roll = AHRS_Roll_Rad( );

    /* read RC controls */
    i_aileron = Get_RC_Channel(AILERON_CHANNEL);
    i_elevator = Get_RC_Channel(ELEVATOR_CHANNEL);
    i_throttle = Get_RC_Channel(THROTTLE_CHANNEL);
    i_rudder = Get_RC_Channel(RUDDER_CHANNEL);

    /* update controls */
    direction_control();
    altitude_control();
    pitch_control();
    roll_control();
    camera_control();

    /* read RC mode */
    uc_current_mode = Get_RC_Mode();
    switch (uc_current_mode) {

        /* NAVIGATION MODE */
        /* same as stabilized mode, controls altitude too */
        case MODE_NAV:
//            i_throttle = SERVO_NEUTRAL + (int16_t)(500.0f * f_ctrl_throttle);
            i_elevator = SERVO_NEUTRAL + (int16_t)f_ctrl_elevator;
            i_aileron = SERVO_NEUTRAL + (int16_t)f_ctrl_aileron;
            break;

        /* STABILIZED MODE */
        /* keeps aircraft attitude, as requested by RC commands */
        case MODE_STAB:
            i_elevator = SERVO_NEUTRAL + (int16_t)f_ctrl_elevator;
            i_aileron = SERVO_NEUTRAL + (int16_t)f_ctrl_aileron;
            break;

        /* MANUAL MODE */
        /* servo commands = RC commands, PIDs are reset */
        case MODE_MANUAL:
            /* state entry transition */
            if (uc_old_mode != MODE_MANUAL) {
                /* reset PID controllers */
                PID_Init(&Roll_Pid);
                PID_Init(&Pitch_Pid);
                PID_Init(&Nav_Pid);
            }
            break;

        case MODE_RTL:
        default:
            break;
    }

    /*************** BEGIN OF TEST CODE FOR CAMERA CONTROL ***************/
    /*********** REMOVE TO RESTORE THROTTLE AND RUDDER CONTROL ***********/

    i_throttle = SERVO_NEUTRAL + (int16_t)f_camera_tilt;
    i_rudder = SERVO_NEUTRAL + (int16_t)f_camera_slant;

    /**************** END OF TEST CODE FOR CAMERA CONTROL ****************/

    uc_old_mode = uc_current_mode;

    /* update servos */
    Set_Servo(SERVO_AILERON, i_aileron);
    Set_Servo(SERVO_ELEVATOR, i_elevator);
    Set_Servo(SERVO_THROTTLE, i_throttle);
    Set_Servo(SERVO_RUDDER, i_rudder);
}

/*----------------------------------------------------------------------------
 *
 * @brief   control of aircraft direction
 * @return  -
 * @remarks direction is controlled adjusting aircraft roll.
 *          Control implemented as a PI(D) loop.
 *          Input is set to zero, setpoint is the direction error, i.e. the
 *          difference between the heading computed by AHRS and the bearing
 *          to next waypoint, computed by navigation module.
 *          The direction error is limited between -PI, PI and normalized
 *          to -1, 1.
 *          The roll angle is limited inside PID loop to maximum bank angle.
 *
 *                                  +-----------------+
 *                                  |                 |
 *                          0.0 --->| input    output |---> f_nav_roll [rad]
 *                                  |                 |
 *                                  |       PID       |
 *                                  |                 |
 *   AHRS heading [rad] ---( - )--->| setpoint        |
 *                           |      |                 |
 *   bearing to wpt [rad] ---+      +-----------------+
 *
 *----------------------------------------------------------------------------*/
__inline static void direction_control( void ) {

  /* compute direction error */
  f_temp = f_ahrs_heading - Nav_Bearing_Rad();

  /* limit direction error between [-PI, PI] */
  if (f_temp < -PI) {
     f_temp = f_temp + (2.0f * PI);
  } else if (f_temp > PI) {
     f_temp = f_temp - (2.0f * PI);
  }

  /* normalize direction error */
  f_temp = f_temp / PI;

  /* direction PID */
  Nav_Pid.fSetpoint = f_temp;
  Nav_Pid.fInput = 0.0f;
  f_nav_roll = PID_Compute(&Nav_Pid);
}

/*----------------------------------------------------------------------------
 *
 * @brief   control of aircraft altitude
 * @return  -
 * @remarks altitude is controlled by adjusting throttle and pitch.
 *          Both are interpolated between min and max values when actual
 *          altitude lies within desired altitude +/- height margins and
 *          are set at min / max when actual altitude is above / below
 *          desired altitude +/- height margins.
 *
 *          actual altitude           | throttle & pitch
 *          --------------------------+-----------------
 *                                    | min
 *          desired altitude + margin | min
 *                                    | interpolated
 *          desired altitude          | interpolated
 *                                    | interpolated
 *          desired altitude - margin | max
 *                                    | max
 *
 *          Input is the altitude error (in m) computed by navigation module,
 *          outputs are saved in f_ctrl_throttle (in %), and f_nav_pitch
 *          (in radians).
 *
 *----------------------------------------------------------------------------*/
__inline static void altitude_control( void ) {

    /* get altitude error */
    f_temp = Nav_Alt_Error();

    /* interpolate throttle and pitch */
    if (f_temp > f_height_margin) {             /* we're too high */
        f_ctrl_throttle = f_throttle_min;       /* minimum throttle */
        f_nav_pitch = PITCHATMINTHROTTLE;
    } else if (f_temp < -f_height_margin) {     /* we're too low */
        f_ctrl_throttle = f_throttle_max;       /* max throttle */
        f_nav_pitch = PITCHATMAXTHROTTLE;
    } else {                                    /* interpolate */
        f_temp = (f_temp - f_height_margin) / (2.0f * f_height_margin);
        f_ctrl_throttle = f_temp * (f_throttle_min - f_throttle_max) + f_throttle_min;
        f_nav_pitch = f_temp * (PITCHATMINTHROTTLE - PITCHATMAXTHROTTLE) + PITCHATMINTHROTTLE;
    }
}

/*----------------------------------------------------------------------------
 *
 * @brief   control of aircraft pitch
 * @return  -
 * @remarks control implemented as a PI(D) loop.
 *          Input is aircraft pitch computed by AHRS.
 *          Setpoint is either navigation pitch or RC elevator control.
 *          Navigation pitch is used in NAV mode to maintain altitude.
 *          RC elevator control is used in STAB mode, converted to desired
 *          pitch and limited between -45°, +45°.
 *
 *                                      +-----------------+
 *                                      |                 |
 *   AHRS pitch [rad] ----------------->| input    output |---> f_ctrl_elevator
 *                             +--\     |                 |
 *   RC pitch command ---------|   \    |       PID       |
 *     (mode = MAN)            |    \   |                 |
 *                             |     |->| setpoint        |
 *                             |    /   |                 |
 *   navigation pitch [rad] ---|   /    +-----------------+
 *     (mode = NAV)            +--/
 *
 *----------------------------------------------------------------------------*/
__inline static void pitch_control( void ) {

  /* determine setpoint for pitch PID */
  if (uc_current_mode == MODE_NAV) {    /* NAV */
    Pitch_Pid.fSetpoint = f_nav_pitch;
  } else {                              /* STAB, MAN, FPV, CAMERA */
    Pitch_Pid.fSetpoint = ((float)(i_elevator - SERVO_NEUTRAL) / ELEV_TO_PITCH);
  }

  /* pitch PID */
  Pitch_Pid.fInput = f_ahrs_pitch;
  f_ctrl_elevator = PID_Compute(&Pitch_Pid);
}

/*----------------------------------------------------------------------------
 *
 * @brief   control of aircraft roll
 * @return  -
 * @remarks control implemented as a PI(D) loop.
 *          Input is aircraft roll computed by AHRS.
 *          Setpoint is either navigation roll or RC aileron control.
 *          Navigation roll is used in NAV mode to steer toward next waypoint.
 *          RC aileron control is used in STAB mode, converted to desired bank
 *          and limited between -60°, +60°.
 *
 *                                      +-----------------+
 *                                      |                 |
 *   AHRS roll [rad] ------------------>| input    output |---> f_ctrl_aileron
 *                             +--\     |                 |
 *   RC roll command ----------|   \    |       PID       |
 *     (mode = MAN)            |    \   |                 |
 *                             |     |->| setpoint        |
 *                             |    /   |                 |
 *   navigation roll [rad] ----|   /    +-----------------+
 *     (mode = NAV)            +--/
 *
 *----------------------------------------------------------------------------*/
__inline static void roll_control( void ) {

  /* determine setpoint for roll PID */
  if (uc_current_mode == MODE_NAV) {    /* NAV */
    Roll_Pid.fSetpoint = f_nav_roll;
  } else {                              /* STAB, MAN, FPV, CAMERA */
    Roll_Pid.fSetpoint = ((float)(i_aileron - SERVO_NEUTRAL) / AIL_TO_BANK);
  }

  /* roll PID */
  Roll_Pid.fInput = f_ahrs_roll;
  f_ctrl_aileron = PID_Compute(&Roll_Pid);
}

/*----------------------------------------------------------------------------
 *
 * @brief   camera control
 * @return  -
 * @remarks roll and pitch angles determined by AHRS are scaled and copied to 
 *          slant and tilt respectively.
 *
 *----------------------------------------------------------------------------*/
__inline static void camera_control( void ) {

  f_camera_tilt = -(f_ahrs_pitch * 1800.0f) / (PI * TILT_GEAR_RATIO); /* current pitch */
  f_camera_slant = -(f_ahrs_roll * 1800.0f) / PI;   									/* current roll */
}

/**
  * @}
  */

/**
  * @}
  */

/*****END OF FILE****/

