/**===========================================================================+
 *
 * @Author rosenkranz
 * @file mavlink.c
 * @brief tentative Mavlink protocol implementation
 *
 * -------------------- Mavlink message structure --------------------
 * \code
 *  Byte     Name        Content              Value
 * ---------------------------------------------------
 *   0       MAVLINK_STX Start Transmission   0xFE
 *   1       len         Length               0 - 255
 *   2       seq         Sequence             0 - 255
 *   3       SYSID       System identifier    0 - 255
 *   4       COMPID      Component identifier 0 - 255
 *   5       MSGID       Message identifier   0 - 255
 *   6       Payload     Payload
 *   7 + len CRC 1
 *   8 + len CRC 2
 * \endcode
 * ------------- Mavlink identifiers and message lengths -------------
 *
 * All identifiers are prefixed with "MAVLINK_MSG_ID_" in the code,
 * e.g. ID of "HEARTBEAT" message will be "MAVLINK_MSG_ID_HEARTBEAT".
 *
 * \code
 * Identifier (MSGID)    Value Length Status
 * --------------------------------------------------
 * HEARTBEAT                0     9   Verified
 * SYS_STATUS               1    31   TBD
 * PARAM_REQUEST_LIST      21     2   Implemented
 * PARAM_VALUE             22    25   Verified
 * PARAM_SET               23    23   Verified
 * GPS_RAW_INT             24    30   Verified
 * ATTITUDE                30    28   Verified
 * GLOBAL_POSITION_INT     33    28   Verified
 * RC_CHANNELS_RAW         35    22   TBD
 * MISSION_CURRENT         42     2   TBD
 * MISSION_REQUEST_LIST    43     2   Verified
 * MISSION_COUNT           44     4   Verified
 * MISSION_CLEAR_ALL       45     2   TBD
 * MISSION_ACK             47     3   TBD
 * NAV_CONTROLLER_OUTPUT   62    26   TBD
 * REQUEST_DATA_STREAM     66     6   TBD
 * VFR_HUD                 74    20   Verified
 * COMMAND_LONG            76    21?  TBD
 * HIL_STATE               90    56   TBD
 * WIND                   168    12   TBD
 * \endcode
 *
 * ----------- Content of Mavlink message not yet implemented -----------
 *
 * All identifiers are prefixed with "MAVLINK_MSG_ID_" in the code,
 * e.g. ID of "HEARTBEAT" message will be "MAVLINK_MSG_ID_HEARTBEAT".
 *
 * \code
 * Identifier (MSGID)    Content           Offset Type    Note
 * -----------------------------------------------------------------------
 *
 * SYS_STATUS            Battery voltage     14   uint16_t
 *                       Battery current     16   int16_t
 *                       Battery remaining   30   int8_t
 *
 * NAV_CONTROLLER_OUTPUT Nav roll             0   float
 *                       Nav pitch            4   float
 *                       Altitude error       8   float
 *                       Airspeed error      12   float
 *                       Crosstrack error    16   float
 *                       Nav bearing         20   int16_t
 *                       Target bearing      22   int16_t
 *                       Waypoint distance   24   uint16_t
 *
 * MISSION_CURRENT       Waypoint number      0   uint16_t
 *
 * RC_CHANNELS_RAW       Channel 1            4   uint16_t
 *                       Channel 2            6   uint16_t
 *                       Channel 5           12   uint16_t
 *                       Channel 6           14   uint16_t
 *                       Channel 7           16   uint16_t
 *                       Channel 8           18   uint16_t
 *                       RSSI                21   uint8_t
 *
 * MISSION_CLEAR_ALL     target_system        ?   uint8_t  System ID
 *                       target_component     ?   uint8_t  Component ID
 *
 * WIND                  Wind direction       0   float
 *                       Wind speed           4   float
 *                       Wind speed Z         8   float
 *
 * COMMAND_LONG          target_system        ?   uint8_t  System which should execute the command
 *                       target_component     ?   uint8_t  Component which should execute the command, 0 for all components
 *                       command              ?   uint16_t Command ID, as defined by MAV_CMD enum.
 *                       confirmation         ?   uint8_t  0=First transmission. 1-255=Confirmation transmissions (e.g. for kill command)
 *                       param1               ?   float    Parameter 1, as defined by MAV_CMD enum.
 *                       param2               ?   float    Parameter 2, as defined by MAV_CMD enum.
 *                       param3               ?   float    Parameter 3, as defined by MAV_CMD enum.
 *                       param4               ?   float    Parameter 4, as defined by MAV_CMD enum.
 *                       param5               ?   float    Parameter 5, as defined by MAV_CMD enum.
 *                       param6               ?   float    Parameter 6, as defined by MAV_CMD enum.
 *                       param7               ?   float    Parameter 7, as defined by MAV_CMD enum.
 *                                                         Send a command with up to four parameters to the MAV
 *
 * HIL_STATE             time_usec            0   uint64_t Timestamp [microseconds]
 *                       roll                 8   float    Roll angle [rad]
 *                       pitch               12   float    Pitch angle [rad]
 *                       yaw                 16   float    Yaw angle [rad]
 *                       rollspeed           20   float    Roll angular speed [rad/s]
 *                       pitchspeed          24   float    Pitch angular speed [rad/s]
 *                       yawspeed            28   float    Yaw angular speed [rad/s]
 *                       lat                 32   int32_t  Latitude [deg * 1E7]
 *                       lon                 36   int32_t  Longitude [deg * 1E7]
 *                       alt                 40   int32_t  Altitude [mm]
 *                       vx                  44   int16_t  Ground X Speed (Latitude) [m/s * 100]
 *                       vy                  46   int16_t  Ground Y Speed (Longitude) [m/s * 100]
 *                       vz                  48   int16_t  Ground Z Speed (Altitude) [m/s * 100]
 *                       xacc                50   int16_t  X acceleration [mg]
 *                       yacc                52   int16_t  Y acceleration [mg]
 *                       zacc                54   int16_t  Z acceleration [mg]
 *
 * \endcode
 *
 *--------------------- ArduCAM OSD messages taxonomy ------------------------
 *
 * Message              ID Len         Payload
 * ---------------------------------------------------------------------------
 * request data stream  42  6
 *                             rate  sys  comp stream   cmd    note
 *          "                    2    14   C8     1    start   sensors
 *          "                    2    14   C8     2    start   ext status
 *          "                    5    14   C8     3    start   RC channels
 *          "                    2    14   C8     6    start   position
 *          "                    5    14   C8    10    start   extra 1 (attiude)
 *          "                    2    14   C8    11    start   extra 2 (VFR HUD)
 *
 * ------------------------------ Links ------------------------------
 *

 * ArduPilot Mega parameters modifiable by MAVLink
 * http://code.google.com/p/ardupilot-mega/wiki/MAVParam
 *
 * ArduPilot Mega MAVLink commands
 * http://code.google.com/p/ardupilot-mega/wiki/MAVLink
 *
 * MAVLink protocol specifications
 * http://qgroundcontrol.org/mavlink/start
 * http://groundcontrol.org/dev/mavlink_arduino_integration_tutorial
 * http://qgroundcontrol.org/dev/mavlink_onboard_integration_tutorial
 *
 * MAVLink protocol specifications
 * http://qgroundcontrol.org/mavlink/start
 * http://qgroundcontrol.org/dev/mavlink_onboard_integration_tutorial
 *
 * MAVLINK waypoint protocol
 * http://qgroundcontrol.org/mavlink/waypoint_protocol
 *
 * MAVLINK message set
 * https://pixhawk.ethz.ch/mavlink/
 *
 * Change: modified check of all waypoints received
 *
 *============================================================================*/

#include "ch.h"
#include "hal.h"

#include "math.h"
#include "config.h"
#include "dcm.h"
#include "nav.h"
#include "gps.h"
#include "baro.h"
#include "servo.h"
#include "control.h"
#include "mavlink.h"

/*--------------------------------- Definitions ------------------------------*/

#define MAX_STREAM_RATE             50

#define ONBOARD_PARAM_COUNT         ((uint16_t)TEL_GAIN_NUMBER)
#define ONBOARD_PARAM_NAME_LENGTH   16

/*
 * length of mavlink packet, originally MAVLINK_MAX_PACKET_LEN,
 * reduced to 64 bytes due to RAM constraints
 */
#define PACKET_LEN                  64
#define PAYLOAD_LEN                 64
                                                /* Origin */
#define MAVLINK_MSG_ID_HEARTBEAT             0  /* mavlink\common\mavlink_msg_heartbeat.h */
#define MAVLINK_MSG_ID_PARAM_REQUEST_LIST   21  /* mavlink\common\mavlink_msg_param_request_list.h */
#define MAVLINK_MSG_ID_PARAM_VALUE          22  /* mavlink\common\mavlink_msg_param_value.h */
#define MAVLINK_MSG_ID_PARAM_SET            23  /* mavlink\common\mavlink_msg_param_set.h */
#define MAVLINK_MSG_ID_GPS_RAW_INT          24  /* mavlink\common\mavlink_msg_gps_raw_int.h */
#define MAVLINK_MSG_ID_ATTITUDE             30  /* mavlink\common\mavlink_msg_attitude.h */
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT  33  /* mavlink\common\mavlink_msg_global_position_int.h */
#define MAVLINK_MSG_ID_MISSION_ITEM         39  /* mavlink\common\mavlink_msg_mission_item.h */
#define MAVLINK_MSG_ID_MISSION_REQUEST      40  /* mavlink\common\mavlink_msg_mission_request.h */
#define MAVLINK_MSG_ID_MISSION_REQUEST_LIST 43  /* mavlink\common\mavlink_msg_mission_request_list.h */
#define MAVLINK_MSG_ID_MISSION_COUNT        44  /* mavlink\common\mavlink_msg_mission_count.h */
#define MAVLINK_MSG_ID_MISSION_ACK          47  /* mavlink\common\mavlink_msg_mission_ack.h */
#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM  66  /* mavlink\common\mavlink_msg_request_data_stream.h */
#define MAVLINK_MSG_ID_VFR_HUD              74  /* mavlink\common\mavlink_msg_vfr_hud.h */
#define MAVLINK_MSG_ID_COMMAND_LONG         76  /* mavlink\common\mavlink_msg_command_long.h */
#define MAVLINK_MSG_ID_HIL_STATE            90  /* mavlink\common\mavlink_msg_hil_state.h */
#define X25_INIT_CRC                    0xFFFF  /* mavlink\checksum.h */
#if (0)
#define X25_VALIDATE_CRC                0xF0B8  /* mavlink\matrixpilot\mavlink.h */
#endif
#define MAVLINK_STX                       0xFE  /* mavlink\matrixpilot\mavlink.h */
                                                /* mavlink\matrixpilot\matrixpilot.h */
#define MAVLINK_MESSAGE_CRCS {          \
 50, 124, 137,   0, 237, 217, 104, 119, \
  0,   0,   0,  89,   0,   0,   0,   0, \
  0,   0,   0,   0, 214, 159, 220, 168, \
 24,  23, 170, 144,  67, 115,  39, 246, \
185, 104, 237, 244, 222, 212,   9, 254, \
230,  28,  28, 132, 221, 232,  11, 153, \
 41,  39, 214, 223, 141,  33,  15,   3, \
100,  24, 239, 238,  30, 240, 183, 130, \
130,   0, 148,  21,   0,  52, 124,   0, \
  0,   0,  20,   0, 152, 143,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0, 231, 183,  63,  54,   0,   0,   0, \
  0,   0,   0,   0, 175, 102, 158, 208, \
 56,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0, 204,  49, 170,  44,  83,  46,   0 }

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*lint -e753 -e749 -e751 */
enum MAV_AUTOPILOT {
     MAV_AUTOPILOT_GENERIC = 0,    /* 0: Generic autopilot, full support for everything */
     MAV_AUTOPILOT_PIXHAWK,        /* 1: PIXHAWK autopilot, http://pixhawk.ethz.ch */
     MAV_AUTOPILOT_SLUGS,          /* 2: SLUGS autopilot, http://slugsuav.soe.ucsc.edu */
     MAV_AUTOPILOT_ARDUPILOTMEGA,  /* 3: ArduPilotMega / ArduCopter, http://diydrones.com */
     MAV_AUTOPILOT_OPENPILOT,      /* 4: OpenPilot, http://openpilot.org */
     MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY, /* 5: Generic autopilot only supporting simple waypoints */
     MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY, /* 6: Generic autopilot supporting waypoints and other simple navigation commands */
     MAV_AUTOPILOT_GENERIC_MISSION_FULL, /* 7: Generic autopilot supporting the full mission command set */
     MAV_AUTOPILOT_INVALID,        /* 8: No valid autopilot, e.g. a GCS or other MAVLink component */
     MAV_AUTOPILOT_PPZ,            /* 9: PPZ UAV - http://nongnu.org/paparazzi */
     MAV_AUTOPILOT_UDB,            /* 10: UAV Dev Board */
     MAV_AUTOPILOT_FP,             /* 11: FlexiPilot */
     MAV_AUTOPILOT_ENUM_END        /* 12: */
};
/*
 * type of vehicle
 */
enum MAV_TYPE {
     MAV_TYPE_GENERIC = 0,           /* 0: Generic micro air vehicle. */
     MAV_TYPE_FIXED_WING,            /* 1: Fixed wing aircraft. */
     MAV_TYPE_QUADROTOR,             /* 2: Quadrotor */
     MAV_TYPE_COAXIAL,               /* 3: Coaxial helicopter */
     MAV_TYPE_HELICOPTER,            /* 4: Normal helicopter with tail rotor. */
     MAV_TYPE_ANTENNA_TRACKER,       /* 5: Ground installation */
     MAV_TYPE_GCS,                   /* 6: Operator control unit / ground control station */
     MAV_TYPE_AIRSHIP,               /* 7: Airship, controlled */
     MAV_TYPE_FREE_BALLOON,          /* 8: Free balloon, uncontrolled */
     MAV_TYPE_ROCKET,                /* 9: Rocket */
     MAV_TYPE_GROUND_ROVER,          /* 10 Ground rover */
     MAV_TYPE_SURFACE_BOAT,          /* 11: Surface vessel, boat, ship */
     MAV_TYPE_SUBMARINE,             /* 12: Submarine */
     MAV_TYPE_HEXAROTOR,             /* 13: Hexarotor */
     MAV_TYPE_OCTOROTOR,             /* 14: Octorotor */
     MAV_TYPE_TRICOPTER,             /* 15: Octorotor */
     MAV_TYPE_FLAPPING_WING,         /* 16: Flapping wing */
     MAV_TYPE_ENUM_END               /* 17: */
};
/*
 * identifiers of components
 */
enum MAV_COMPONENT {
     MAV_COMP_ID_ALL=0,              /*  */
     MAV_COMP_ID_DUMMY=1,            /*  */
     MAV_COMP_ID_CAMERA=100,         /*  */
     MAV_COMP_ID_SERVO1=140,         /*  */
     MAV_COMP_ID_SERVO2=141,         /*  */
     MAV_COMP_ID_SERVO3=142,         /*  */
     MAV_COMP_ID_SERVO4=143,         /*  */
     MAV_COMP_ID_SERVO5=144,         /*  */
     MAV_COMP_ID_SERVO6=145,         /*  */
     MAV_COMP_ID_SERVO7=146,         /*  */
     MAV_COMP_ID_SERVO8=147,         /*  */
     MAV_COMP_ID_SERVO9=148,         /*  */
     MAV_COMP_ID_SERVO10=149,        /*  */
     MAV_COMP_ID_SERVO11=150,        /*  */
     MAV_COMP_ID_SERVO12=151,        /*  */
     MAV_COMP_ID_SERVO13=152,        /*  */
     MAV_COMP_ID_SERVO14=153,        /*  */
     MAV_COMP_ID_MAPPER=180,         /*  */
     MAV_COMP_ID_MISSIONPLANNER=190, /*  */
     MAV_COMP_ID_PATHPLANNER=195,    /*  */
     MAV_COMP_ID_IMU=200,            /*  */
     MAV_COMP_ID_IMU_2=201,          /*  */
     MAV_COMP_ID_IMU_3=202,          /*  */
     MAV_COMP_ID_IMU_4=203,          /*  */
     MAV_COMP_ID_IMU_5=204,          /*  */
     MAV_COMP_ID_IMU_6=205,          /*  */
     MAV_COMP_ID_GPS=220,            /*  */
     MAV_COMP_ID_UDP_BRIDGE=240,     /*  */
     MAV_COMP_ID_UART_BRIDGE=241,    /*  */
     MAV_COMP_ID_SYSTEM_CONTROL=250, /*  */
     MAV_COMPONENT_ENUM_END=251      /*  */
};

/*
 * Data stream IDs. 
 * A data stream is not a fixed set of messages, but rather a recommendation to the autopilot software.
 * Individual autopilots may or may not obey the recommended messages.
 * Origin: mavlink\common\common.h
 */
enum MAV_DATA_STREAM {
     MAV_DATA_STREAM_ALL = 0,            /* 0: Enable all data streams */
     MAV_DATA_STREAM_RAW_SENSORS,        /* 1: Enable IMU_RAW, GPS_RAW, GPS_STATUS packets. */
     MAV_DATA_STREAM_EXTENDED_STATUS,    /* 2: Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS */
     MAV_DATA_STREAM_RC_CHANNELS,        /* 3: Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW */
     MAV_DATA_STREAM_RAW_CONTROLLER,     /* 4: Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT. */
     MAV_DATA_STREAM_DUMMY_5,            /* 5: */
     MAV_DATA_STREAM_POSITION,           /* 6: Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages. */
     MAV_DATA_STREAM_DUMMY_7,            /* 7: */
     MAV_DATA_STREAM_DUMMY_8,            /* 8: */
     MAV_DATA_STREAM_DUMMY_9,            /* 9: */
     MAV_DATA_STREAM_EXTRA1,             /* 10: Dependent on the autopilot */
     MAV_DATA_STREAM_EXTRA2,             /* 11: Dependent on the autopilot */
     MAV_DATA_STREAM_EXTRA3,             /* 12: Dependent on the autopilot */
     MAV_DATA_STREAM_ENUM_END            /* 13: */
};

/*
 * Data stream types
 * Origin: mavlink\common\common.h
 */
enum MAVLINK_DATA_STREAM_TYPE {
     MAVLINK_DATA_STREAM_IMG_JPEG = 1,   /* 1: */
     MAVLINK_DATA_STREAM_IMG_BMP,        /* 2: */
     MAVLINK_DATA_STREAM_IMG_RAW8U,      /* 3: */
     MAVLINK_DATA_STREAM_IMG_RAW32U,     /* 4: */
     MAVLINK_DATA_STREAM_IMG_PGM,        /* 5: */
     MAVLINK_DATA_STREAM_IMG_PNG,        /* 6: */
     MAVLINK_DATA_STREAM_TYPE_ENUM_END   /* 7: */
};

/*
 * Types of coordinate frames ?
 * Origin: mavlink\common\common.h
 */
enum MAV_FRAME {
     MAV_FRAME_GLOBAL = 0,           /* 0: Global coordinate frame, WGS84. First value x: latitude, second value y: longitude, third value z: positive altitude (MSL) */
     MAV_FRAME_LOCAL_NED,            /* 1: Local coordinate frame, Z-up (x: north, y: east, z: down). */
     MAV_FRAME_MISSION,              /* 2: NOT a coordinate frame, indicates a mission command. */
     MAV_FRAME_GLOBAL_RELATIVE_ALT,  /* 3: Global coordinate frame, WGS84, relative altitude QFE. First value x: latitude, second value y: longitude, third value z: positive altitude QFE. */
     MAV_FRAME_LOCAL_ENU,            /* 4: Local coordinate frame, Z-down (x: east, y: north, z: up) */
     MAV_FRAME_ENUM_END              /* 5:  */
};

/*
 * Commands
 * Origin: mavlink\common\common.h
 */
enum MAV_CMD {
     MAV_CMD_NAV_WAYPOINT=16,        /* Navigate to MISSION. | Hold time in seconds/10. (ignored by fixed wing)| Acceptance radius in meters | 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at MISSION (rotary wing)| Latitude| Longitude| Altitude| */
     MAV_CMD_NAV_LOITER_UNLIM=17,    /* Loiter around this MISSION an unlimited amount of time |Empty| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
     MAV_CMD_NAV_LOITER_TURNS=18,    /* Loiter around this MISSION for X turns |Turns| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
     MAV_CMD_NAV_LOITER_TIME=19,     /* Loiter around this MISSION for X seconds |Seconds (decimal)| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
     MAV_CMD_NAV_RETURN_TO_LAUNCH=20, /* Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
     MAV_CMD_NAV_LAND=21,            /* Land at location |Empty| Empty| Empty| Desired yaw angle.| Latitude| Longitude| Altitude|  */
     MAV_CMD_NAV_TAKEOFF=22,         /* Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer| Latitude| Longitude| Altitude|  */
     MAV_CMD_NAV_ROI=80,             /* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  */
     MAV_CMD_NAV_PATHPLANNING=81,    /* Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| Empty| Yaw angle at goal, in compass degrees, [0..360]| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */
     MAV_CMD_NAV_LAST=95,            /* NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
     MAV_CMD_CONDITION_DELAY=112,    /* Delay mission state machine. |Delay in seconds (decimal)| Empty| Empty| Empty| Empty| Empty| Empty|  */
     MAV_CMD_CONDITION_CHANGE_ALT=113, /* Ascend/descend at rate.  Delay mission state machine until desired altitude reached. |Descent / Ascend rate (m/s)| Empty| Empty| Empty| Empty| Empty| Finish Altitude|  */
     MAV_CMD_CONDITION_DISTANCE=114, /* Delay mission state machine until within desired distance of next NAV point. |Distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  */
     MAV_CMD_CONDITION_YAW=115,      /* Reach a certain target angle. |target angle: [0-360], 0 is north| speed during yaw change:[deg per second]| direction: negative: counter clockwise, positive: clockwise [-1,1]| relative offset or absolute angle: [ 1,0]| Empty| Empty| Empty|  */
     MAV_CMD_CONDITION_LAST=159,     /* NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
     MAV_CMD_DO_SET_MODE=176,        /* Set system mode. |Mode, as defined by ENUM MAV_MODE| Empty| Empty| Empty| Empty| Empty| Empty|  */
     MAV_CMD_DO_JUMP=177,            /* Jump to the desired command in the mission list.  Repeat this action only the specified number of times |Sequence number| Repeat count| Empty| Empty| Empty| Empty| Empty|  */
     MAV_CMD_DO_CHANGE_SPEED=178,    /* Change speed and/or throttle set points. |Speed type (0=Airspeed, 1=Ground Speed)| Speed  (m/s, -1 indicates no change)| Throttle  ( Percent, -1 indicates no change)| Empty| Empty| Empty| Empty|  */
     MAV_CMD_DO_SET_HOME=179,        /* Changes the home location either to the current location or a specified location. |Use current (1=use current location, 0=use specified location)| Empty| Empty| Empty| Latitude| Longitude| Altitude|  */
     MAV_CMD_DO_SET_PARAMETER=180,   /* Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. |Parameter number| Parameter value| Empty| Empty| Empty| Empty| Empty|  */
     MAV_CMD_DO_SET_RELAY=181,       /* Set a relay to a condition. |Relay number| Setting (1=on, 0=off, others possible depending on system hardware)| Empty| Empty| Empty| Empty| Empty|  */
     MAV_CMD_DO_REPEAT_RELAY=182,    /* Cycle a relay on and off for a desired number of cyles with a desired period. |Relay number| Cycle count| Cycle time (seconds, decimal)| Empty| Empty| Empty| Empty|  */
     MAV_CMD_DO_SET_SERVO=183,       /* Set a servo to a desired PWM value. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Empty| Empty| Empty| Empty| Empty|  */
     MAV_CMD_DO_REPEAT_SERVO=184,    /* Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Cycle count| Cycle time (seconds)| Empty| Empty| Empty|  */
     MAV_CMD_DO_CONTROL_VIDEO=200,   /* Control onboard camera system. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|  */
     MAV_CMD_DO_LAST=240,            /* NOP - This command is only used to mark the upper limit of the DO commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
     MAV_CMD_PREFLIGHT_CALIBRATION=241, /* Trigger calibration. This command will be only accepted if in pre-flight mode. |Gyro calibration: 0: no, 1: yes| Magnetometer calibration: 0: no, 1: yes| Ground pressure: 0: no, 1: yes| Radio calibration: 0: no, 1: yes| Empty| Empty| Empty|  */
     MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS=242, /* Set sensor offsets. This command will be only accepted if in pre-flight mode. |Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow| X axis offset (or generic dimension 1), in the sensor's raw units| Y axis offset (or generic dimension 2), in the sensor's raw units| Z axis offset (or generic dimension 3), in the sensor's raw units| Generic dimension 4, in the sensor's raw units| Generic dimension 5, in the sensor's raw units| Generic dimension 6, in the sensor's raw units|  */
     MAV_CMD_PREFLIGHT_STORAGE=245,  /* Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. |Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM| Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM| Reserved| Reserved| Empty| Empty| Empty|  */
     MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN=246, /* Request the reboot or shutdown of system components. |0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot.| 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer.| Reserved| Reserved| Empty| Empty| Empty|  */
     MAV_CMD_OVERRIDE_GOTO=252,      /* Hold / continue the current action |MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with next item in mission plan| MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position| MAV_FRAME coordinate frame of hold point| Desired yaw angle in degrees| Latitude / X position| Longitude / Y position| Altitude / Z position|  */
     MAV_CMD_MISSION_START=300,      /* start running a mission |first_item: the first mission item to run| last_item:  the last mission item to run (after this item is run, the mission ends)|  */
     MAV_CMD_COMPONENT_ARM_DISARM=400, /* Arms / Disarms a component |1 to arm, 0 to disarm|  */
     MAV_CMD_ENUM_END=401            /*  */
};

/*
 * Results in a mission ack message
 * Origin: mavlink\common\common.h
 */
enum MAV_MISSION_RESULT
{
	MAV_MISSION_ACCEPTED = 0,       /* mission accepted OK */
	MAV_MISSION_ERROR,              /* generic error / not accepting mission commands at all right now */
	MAV_MISSION_UNSUPPORTED_FRAME,  /* coordinate frame is not supported */
	MAV_MISSION_UNSUPPORTED,        /* command is not supported */
	MAV_MISSION_NO_SPACE,           /* mission item exceeds storage space */
	MAV_MISSION_INVALID,            /* one of the parameters has an invalid value */
	MAV_MISSION_INVALID_PARAM1,     /* param1 has an invalid value */
	MAV_MISSION_INVALID_PARAM2,     /* param2 has an invalid value */
	MAV_MISSION_INVALID_PARAM3,     /* param3 has an invalid value */
	MAV_MISSION_INVALID_PARAM4,     /* param4 has an invalid value */
	MAV_MISSION_INVALID_PARAM5_X,   /* x / param5 has an invalid value */
	MAV_MISSION_INVALID_PARAM6_Y,   /* y / param6 has an invalid value */
	MAV_MISSION_INVALID_PARAM7,     /* param7 has an invalid value */
	MAV_MISSION_INVALID_SEQUENCE,   /* received waypoint out of sequence */
	MAV_MISSION_DENIED,             /* not accepting any mission commands from this communication partner */
	MAV_MISSION_RESULT_ENUM_END     /* */
};

/*----------------------------------- Types ----------------------------------*/

/*
 * mavlink-specific types
 * Origin: mavlink\mavlink_types.h
 */
typedef enum {
   MAVLINK_TYPE_CHAR     = 0,
   MAVLINK_TYPE_UINT8_T  = 1,
   MAVLINK_TYPE_INT8_T   = 2,
   MAVLINK_TYPE_UINT16_T = 3,
   MAVLINK_TYPE_INT16_T  = 4,
   MAVLINK_TYPE_UINT32_T = 5,
   MAVLINK_TYPE_INT32_T  = 6,
   MAVLINK_TYPE_UINT64_T = 7,
   MAVLINK_TYPE_INT64_T  = 8,
   MAVLINK_TYPE_FLOAT    = 9,
   MAVLINK_TYPE_DOUBLE   = 10
} mavlink_message_type_t;

/*
 * The state machine for the comm parser
 * Origin: mavlink\mavlink_types.h
 */
typedef enum {
    MAVLINK_PARSE_STATE_UNINIT = 0,
    MAVLINK_PARSE_STATE_IDLE,
    MAVLINK_PARSE_STATE_GOT_STX,
    MAVLINK_PARSE_STATE_GOT_SEQ,
    MAVLINK_PARSE_STATE_GOT_LENGTH,
    MAVLINK_PARSE_STATE_GOT_SYSID,
    MAVLINK_PARSE_STATE_GOT_COMPID,
    MAVLINK_PARSE_STATE_GOT_MSGID,
    MAVLINK_PARSE_STATE_GOT_PAYLOAD,
    MAVLINK_PARSE_STATE_GOT_CRC1
} mavlink_parse_state_t;

/*---------------------------------- Constants -------------------------------*/

static const uint8_t ui_mavlink_crc[] = MAVLINK_MESSAGE_CRCS ;

/*
 * Names of parameters
 * Names are specified according Copter GCS standard:
 * GROUP_SUBGROUP_P / _I / _D / _IMAX
 */
static const uint8_t s_parameter_name[ONBOARD_PARAM_COUNT][ONBOARD_PARAM_NAME_LENGTH] = {
    "ROL_ANG_P\0",  /* Roll Kp */
    "ROL_ANG_I\0",  /* Roll Ki */
#if (0)
    "ROL_ANG_D\0",  /* Roll Kd */
#endif
    "PCH_ANG_P\0",  /* Pitch Kp */
    "PCH_ANG_I\0",  /* Pitch Ki */
#if (0)
    "PCH_ANG_D\0",  /* Pitch Kd */
#endif
    "ALT_POS_P\0",  /* Altitude Kp (via throttle) */
    "ALT_POS_I\0",  /* Altitude Ki (via throttle) */
#if (0)
    "ALT_POS_D\0",  /* Altitude Kd (via throttle) */
#endif
    "NAV_ANG_P\0",  /* Navigation Kp (via roll) */
    "NAV_ANG_I\0",  /* Navigation Ki (via roll) */
#if (0)
    "NAV_ANG_D\0",  /* Navigation Kd (via roll) */
#endif
    "NAV_BANK\0"    /* Max bank angle during navigation [deg] */
};

#if (0)
/* 
 * Autopilot capabilities 
 */
static const uint8_t        uc_autopilot_type       = MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY;
/* 
 * Aircraft type 
 */
static const uint8_t        uc_system_type          = MAV_TYPE_FIXED_WING;       
#endif
/* 
 * System ID (original: 20) 
 */
static const uint8_t        uc_system_ID            = 1;
/* 
 * Component ID (original: MAV_COMP_ID_IMU) 
 */
static const enum MAV_COMPONENT e_component_ID      = MAV_COMP_ID_DUMMY; 

/*---------------------------------- Globals ---------------------------------*/

#if (0)
extern struct global_struct global_data;
#endif

/*----------------------------------- Locals ---------------------------------*/

#if (0)
static uint8_t              System_Mode             = MAV_MODE_PREFLIGHT;   /* Booting up */
static uint8_t              System_State            = MAV_STATE_STANDBY;    /* System ready for flight */
static uint8_t              packet_rx_drop_count;
static uint16_t             packet_drops            = 0;
static uint16_t             mode                    = MAV_MODE_UNINIT;
static uint32_t             custom_mode;
static mavlink_system_t     mavlink_system;
static mavlink_param_set_t  set;
static mavlink_status_t     status;
static mavlink_message_t    msg;
#endif

static uint8_t              uc_msg_ID;
static uint8_t              uc_current_tx_seq       = 0;
static uint16_t             ui_parameter_i          = ONBOARD_PARAM_COUNT;
static uint16_t             ui_wpt_get_index        = 0;
static uint16_t             ui_wpt_get_total        = 0;
static uint16_t             ui_crc;
static STRUCT_WPT           wpt;
static bool                 b_wpt_receiving         = FALSE;

/*
 * buffer for incoming messages
 */
static uint8_t uc_rx_msg[PAYLOAD_LEN];

/*
 * buffer for outgoing messages
 */
static uint8_t uc_tx_msg[PACKET_LEN];

/*
 * tick counters for data streams
 */
static uint8_t uc_stream_tick[MAV_DATA_STREAM_ENUM_END] = {
    MAX_STREAM_RATE,    /*  0: all data streams */
    MAX_STREAM_RATE,    /*  1: IMU_RAW, GPS_RAW, GPS_STATUS */
    MAX_STREAM_RATE,    /*  2: GPS_STATUS, CONTROL_STATUS, AUX_STATUS */
    MAX_STREAM_RATE,    /*  3: RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW */
    MAX_STREAM_RATE,    /*  4: ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT */
    MAX_STREAM_RATE,    /*  5: - */
    MAX_STREAM_RATE,    /*  6: LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT */
    MAX_STREAM_RATE,    /*  7: - */
    MAX_STREAM_RATE,    /*  8: - */
    MAX_STREAM_RATE,    /*  9: - */
    MAX_STREAM_RATE,    /* 10: Extra 1, autopilot dependent */
    MAX_STREAM_RATE,    /* 11: Extra 2, autopilot dependent */
    MAX_STREAM_RATE     /* 12: Extra 3, autopilot dependent */
};

/*
 * frequency of data streams
 */
static uint8_t uc_stream_rate[MAV_DATA_STREAM_ENUM_END] = {
    0,  /*  0: all data streams */
    0,  /*  1: IMU_RAW, GPS_RAW, GPS_STATUS */
    0,  /*  2: GPS_STATUS, CONTROL_STATUS, AUX_STATUS */
    0,  /*  3: RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW */
    0,  /*  4: ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT */
    0,  /*  5: - */
    1,  /*  6: LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT */
    0,  /*  7: - */
    0,  /*  8: - */
    0,  /*  9: - */
    5,  /* 10: Extra 1, autopilot dependent */
    2,  /* 11: Extra 2, autopilot dependent */
    0   /* 12: Extra 3, autopilot dependent */
};

static float f_param_value[ONBOARD_PARAM_COUNT] = {
    ROLL_KP,    /* default roll kp */
    ROLL_KI,    /* default roll ki */
    PITCH_KP,   /* default pitch kp */
    PITCH_KI,   /* default pitch ki */
    0.0f,       /* dummy placeholder */
    0.0f,       /* dummy placeholder */
    NAV_KP,     /* default direction kp */
    NAV_KI,     /* default direction ki */
    NAV_BANK    /* default max bank angle */
};              /* gains for PID loops */

/*
 * Serial configuration structure.
 */
static const SerialConfig sdcfg = {
  57600
};


/*--------------------------------- Prototypes -------------------------------*/

static __inline void checksum_init( void );
static __inline void checksum_accumulate( uint8_t data );
static          void Mavlink_Send( uint8_t crc_extra );
static          bool Mavlink_Parse( void );
                bool Mavlink_Stream_Trigger( uint8_t stream );
                void Mavlink_Mission_Item_Get( void );
                void Mavlink_Mission_Count_Send( void );
                void Mavlink_Mission_Count_Get( void );
                void Mavlink_Mission_Ack( uint8_t result );
                void Mavlink_Heartbeat( void );
                void Mavlink_Hud( void );
                void Mavlink_Attitude( void );
                void Mavlink_Gps_Raw( void );
                void Mavlink_Position( void );
                void Mavlink_Param_Send( uint16_t param_index, uint16_t param_count );
                void Mavlink_Param_Set( void );
                void Mavlink_HIL_State( void );

/*---------------------------------- Functions -------------------------------*/

/*----------------------------------------------------------------------------
 *
 * @brief   Initialize telemetry
 * @return  -
 * @remarks -
 *
 *---------------------------------------------------------------------------*/
void Telemetry_Init( void ) {

 /*
  * Activates the Serial driver 1
  * PA9(TX) and PA10(RX) are routed to USART1.
  */
  palSetPadMode(GPIOA, 9, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
  sdStart(&SD1, &sdcfg);
}

/*----------------------------------------------------------------------------
 *
 * @brief   Initialize CRC
 * @return  -
 * @remarks -
 *
 *---------------------------------------------------------------------------*/
static __inline void checksum_init(void)
{
   ui_crc = X25_INIT_CRC;
}

/*----------------------------------------------------------------------------
 *
 * @brief   Accumulate one byte of data into the CRC
 * @return  -
 * @remarks -
 *
 *---------------------------------------------------------------------------*/
static __inline void checksum_accumulate(uint8_t data)
{
    uint8_t tmp;

    tmp = data ^ (uint8_t)(ui_crc & 0xFF);
    tmp ^= (tmp << 4);
    ui_crc = (ui_crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}


/*----------------------------------------------------------------------------
 *
 * @brief   Send communication packets
 * @param   -
 * @returns -
 * @remarks
 *   0          MAVLINK_STX Start Transmission   0xFE
 *   1          len         Message length       0 - 255
 *   2          seq         Sequence number      0 - 255
 *   3          SYSID       System identifier    0 - 255
 *   4          COMPID      Component identifier 0 - 255
 *   5          MSGID       Message identifier   0 - 255
 *
 *---------------------------------------------------------------------------*/
static void Mavlink_Send( uint8_t crc_extra ) {
  uint16_t j;
  uint8_t length = uc_tx_msg[1];

  checksum_init();                              /* Initialize CRC */
  uc_tx_msg[0] =          MAVLINK_STX;          /* Start of message */
  uc_tx_msg[2] =          uc_current_tx_seq++;  /* One sequence number per component ! */
  uc_tx_msg[3] =          uc_system_ID;         /* System identifier */
  uc_tx_msg[4] = (uint8_t)e_component_ID;       /* COmponent identifier */

  for (j = 1; j < length + 6; j++) {
    checksum_accumulate(uc_tx_msg[j]);          /* Compute CRC */
  }
  checksum_accumulate(crc_extra);               /* Finalize CRC */
  uc_tx_msg[j++] = (uint8_t)(ui_crc & 0xFF);    /* Append CRC */
  uc_tx_msg[j  ] = (uint8_t)(ui_crc >> 8);
  chnWrite(&SD1, uc_tx_msg, length + 8);        /* Transmit message */
}

/*----------------------------------------------------------------------------
 *
 * @brief   Send heartbeat packet
 * @param   -
 * @returns -
 * @remarks The heartbeat message shows that a system is present and responding.
 * The type of the MAV and Autopilot hardware allow the receiving system to
 * treat further messages from this system appropriate (e.g. by laying out the
 * user interface based on the autopilot).
 *
 * Name = MAVLINK_MSG_ID_HEARTBEAT, ID = 0, Length = 9
 *
 * Field         Offset  Type    Meaning
 * ----------------------------------------------------------------------------
 * OSD mode         0   uint32_t Bitfield for use for autopilot-specific flags.
 * Heartbeat type   4   uint8_t  Type of the MAV (quadrotor, heli, see MAV_TYPE ENUM)
 * Base mode        6   uint8_t  System mode bitfield, see MAV_MODE_FLAGS ENUM
 * autopilot        ?   uint8_t  Autopilot type / class. see MAV_AUTOPILOT ENUM
 * system_status    ?   uint8_t  System status flag, see MAV_STATE ENUM
 * mavlink_version  ?   uint8_t  MAVLink version, read only,
 *                               gets added by protocol because of magic data type
 *
 *---------------------------------------------------------------------------*/
void Mavlink_Heartbeat( void ) {
  uint16_t j;

  for (j = 0; j < PACKET_LEN; j++) {
    uc_tx_msg[j] = 0;
  }
  uc_tx_msg[ 1] = 9;                              /* Payload length */
  uc_tx_msg[ 5] = MAVLINK_MSG_ID_HEARTBEAT;       /* Heartbeat message ID */
  uc_tx_msg[10] = (uint8_t)MAV_TYPE_FIXED_WING;   /* Type of the MAV, defined in MAV_TYPE ENUM */

  Mavlink_Send(ui_mavlink_crc[MAVLINK_MSG_ID_HEARTBEAT]);
}

/*----------------------------------------------------------------------------
 *
 * @brief   Send VFR HUD data
 * @param   -
 * @returns -
 * @remarks
 * Name = MAVLINK_MSG_ID_VFR_HUD, ID = 74, Length = 20
 *
 * Field        Offset Type    Meaning
 * -------------------------------------
 * Airspeed       0    float
 * Ground speed   4    float
 * Altitude       8    float
 * Climb rate    12    float
 * Heading       16    int16_t
 * Throttle      18    uint16_t
 *
 *---------------------------------------------------------------------------*/
void Mavlink_Hud( void ) {

                  uc_tx_msg[ 1]   = 20;                         /* Payload length */
                  uc_tx_msg[ 5]   = MAVLINK_MSG_ID_VFR_HUD;     /* VFR HUD message ID */
  *((float    *)(&uc_tx_msg[ 6])) = 0.0f;                       /* Airspeed */
  *((float    *)(&uc_tx_msg[10])) = (float)Gps_Speed_Kt();      /* GPS speed */
  *((float    *)(&uc_tx_msg[14])) = (float)Get_Baro_Altitude(); /* Altitude */
  *((float    *)(&uc_tx_msg[18])) = 0.0f;                       /* Climb rate */
  *((uint16_t *)(&uc_tx_msg[22])) = Gps_COG_Deg();              /* Heading */
  *((int16_t  *)(&uc_tx_msg[24])) = Get_Servo(SERVO_THROTTLE);  /* Throttle */

  Mavlink_Send(ui_mavlink_crc[MAVLINK_MSG_ID_VFR_HUD]);
}

/*----------------------------------------------------------------------------
 *
 * @brief   Send attitude data
 * @param   -
 * @returns -
 * @remarks
 * Name = MAVLINK_MSG_ID_ATTITUDE, ID = 30, Length = 28
 *
 * Field      Offset Type     Meaning
 * ----------------------------------------------------------
 * time_boot_ms  0   uint32_t Time since boot in milliseconds
 * roll          4   float    Roll angle []
 * pitch         8   float    Pitch angle []
 * yaw          12   float    Yaw angle []
 * rollspeed    16   float    Roll rate []
 * pitchspeed   20   float    Pitch rate []
 * yawspeed     24   float    Yaw rate []
 *
 *---------------------------------------------------------------------------*/
void Mavlink_Attitude( void ) {

                  uc_tx_msg[ 1]   = 28;                      /* Payload length */
                  uc_tx_msg[ 5]   = MAVLINK_MSG_ID_ATTITUDE; /* Attitude message ID */
  *((uint32_t *)(&uc_tx_msg[ 6])) = 0;                       /* time from boot [ms] */
  *((float    *)(&uc_tx_msg[10])) = AHRS_Roll_Rad();         /* roll */
  *((float    *)(&uc_tx_msg[14])) = AHRS_Pitch_Rad();        /* pitch */
  *((float    *)(&uc_tx_msg[18])) = AHRS_Yaw_Rad();          /* yaw */
  *((float    *)(&uc_tx_msg[22])) = 0.0f;                    /* roll rate */
  *((float    *)(&uc_tx_msg[26])) = 0.0f;                    /* pitch rate */
  *((float    *)(&uc_tx_msg[30])) = 0.0f;                    /* yaw rate */

  Mavlink_Send(ui_mavlink_crc[MAVLINK_MSG_ID_ATTITUDE]);
}

/*----------------------------------------------------------------------------
 *
 * @brief   Send GPS raw data
 * @param   -
 * @returns -
 * @remarks
 * Name = MAVLINK_MSG_ID_GPS_RAW_INT, ID = 24, Length = 30
 *
 * Field             Offset Type     Meaning
 * ------------------------------------------------------------
 * time_usec            0   uint64_t time from boot [usec]
 * lat                  8   int32_t  latitude [deg x 10^7]
 * lon                 12   int32_t  longitude [deg x 10^7]
 * alt                 16   int32_t  altitude [m]
 * eph                 20   uint16_t
 * epv                 22   uint16_t
 * vel                 24   uint16_t ground speed [kt]
 * cog                 26   uint16_t course over ground [deg]
 * fix_type            28   uint8_t  fix type
 * satellites_visible  29   uint8_t  satellites
 *
 *---------------------------------------------------------------------------*/
void Mavlink_Gps_Raw( void ) {

                  uc_tx_msg[ 1]   = 30;                                       /* Payload length */
                  uc_tx_msg[ 5]   = MAVLINK_MSG_ID_GPS_RAW_INT;               /* GPS message ID */
  *((uint64_t *)(&uc_tx_msg[ 6])) = 0;                                        /* time from boot [us] */
  *((int32_t  *)(&uc_tx_msg[14])) = (int32_t)(10000000.0f * Gps_Latitude());  /* latitude */
  *((int32_t  *)(&uc_tx_msg[18])) = (int32_t)(10000000.0f * Gps_Longitude()); /* longitude */
  *((int32_t  *)(&uc_tx_msg[22])) = Get_Baro_Altitude();                      /* altitude */
  *((uint16_t *)(&uc_tx_msg[26])) = 65535;                                    /* eph */
  *((uint16_t *)(&uc_tx_msg[28])) = 65535;                                    /* epv */
  *((uint16_t *)(&uc_tx_msg[30])) = Gps_Speed_Kt();                           /* velocity */
  *((uint16_t *)(&uc_tx_msg[32])) = Gps_COG_Deg();                            /* course over ground */
                  uc_tx_msg[34]   = Gps_Status();                             /* fix type */
                  uc_tx_msg[35]   = 255;                                      /* satellites */

  Mavlink_Send(ui_mavlink_crc[MAVLINK_MSG_ID_GPS_RAW_INT]);
}

/*----------------------------------------------------------------------------
 *
 * @brief   Send position
 * @param   -
 * @returns -
 * @remarks
 * Name = MAVLINK_MSG_ID_GLOBAL_POSITION_INT, ID = 33, Length = 28
 *
 * Field             Offset Type       Meaning
 * --------------------------------------------------------------
 * time_usec            0   uint32_t   time since boot [ms]
 * lat                  8   int32_t    latitude [deg x 10^7]
 * lon                 12   int32_t    longitude [deg x 10^7]
 * alt                 16   int32_t    altitude [m]
 * relative_alt        20   int32_t    altitude above ground [m]
 * vx                  22   uint16_t   ground x speed (latitude)
 * vy                  24   uint16_t   ground y speed (longitude)
 * vz                  26   uint16_t   ground z speed (altitude)
 * hdg                 28   uint16_t   compass heading [deg]
 *
 *---------------------------------------------------------------------------*/
void Mavlink_Position( void ) {

                  uc_tx_msg[ 1]   = 28;                                       /* payload length */
                  uc_tx_msg[ 5]   = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;       /* global position message ID */
  *((uint32_t *)(&uc_tx_msg[ 6])) = 0;                                        /* time from boot [ms] */
  *(( int32_t *)(&uc_tx_msg[10])) = (int32_t)(10000000.0f * Gps_Latitude());  /* latitude */
  *(( int32_t *)(&uc_tx_msg[14])) = (int32_t)(10000000.0f * Gps_Longitude()); /* longitude */
  *(( int32_t *)(&uc_tx_msg[18])) = Get_Baro_Altitude();                      /* altitude */
  *(( int32_t *)(&uc_tx_msg[22])) = Get_Baro_Altitude();                      /* altitude above ground */
  *((uint16_t *)(&uc_tx_msg[26])) = 0;                                        /* ground x speed */
  *((uint16_t *)(&uc_tx_msg[28])) = 0;                                        /* ground y speed */
  *((uint16_t *)(&uc_tx_msg[30])) = 0;                                        /* ground z speed */
  *((uint16_t *)(&uc_tx_msg[32])) = Gps_COG_Deg();                            /* replace with compass heading */

  Mavlink_Send(ui_mavlink_crc[MAVLINK_MSG_ID_GLOBAL_POSITION_INT]);
}

/*----------------------------------------------------------------------------
 *
 * @brief   Send parameter value
 * @param   param_index = index of parameter to be sent
 * @param   param_count = total number of parameters
 * @returns -
 * @remarks
 * Name = MAVLINK_MSG_ID_PARAM_VALUE, ID = 22, Length = 25
 *
 * Field       Offset Type      Meaning
 * --------------------------------------------------------------------------
 * param_value   0    float     Onboard parameter value
 * param_count   4    uint16_t  Total number of onboard parameters
 * param_index   6    uint16_t  Index of current onboard parameter
 * param_id      8    array     Onboard parameter name, null terminated ...
 * param_type    24   uint8_t   Onboard parameter type: see MAVLINK_TYPE enum
 *
 *---------------------------------------------------------------------------*/
void Mavlink_Param_Send( uint16_t param_index, uint16_t param_count ) {

  uint8_t j;
                  uc_tx_msg[     1]   = 25;                               /* Payload length */
                  uc_tx_msg[     5]   = MAVLINK_MSG_ID_PARAM_VALUE;       /* Parameter value message ID */
  *((float    *)(&uc_tx_msg[     6])) = f_param_value[param_index];       /* Parameter value */
  *((uint16_t *)(&uc_tx_msg[    10])) = param_count;                      /* Total number of parameters */
  *((uint16_t *)(&uc_tx_msg[    12])) = param_index;                      /* Parameter index */
  for (j = 0; j < 16; j++) {
                  uc_tx_msg[j + 14]   = s_parameter_name[param_index][j]; /* Parameter name */
  }
                  uc_tx_msg[    30]   = (uint8_t)MAVLINK_TYPE_FLOAT;      /* Parameter type */

  Mavlink_Send(ui_mavlink_crc[MAVLINK_MSG_ID_PARAM_VALUE]);
}

/*----------------------------------------------------------------------------
 *
 * @brief   Set parameter
 * @param   -
 * @returns TRUE if parameter has been set, FALSE otherwise
 * @remarks
 * Name = MAVLINK_MSG_ID_PARAM_SET, ID = 23, Length = 23
 *
 * Field         Offset Type     Meaning
 * ---------------------------------------------------------------------------
 * param_value      0   float    Onboard parameter value
 * target_system    4   uint8_t  System ID
 * target_component 5   uint8_t  Component ID
 * param_id         6   array    Onboard parameter name,
 *                               null terminated if length < 16 chars
 *                               without null termination if length = 16 chars
 * param_type      22   uint8_t  Onboard parameter type: see MAVLINK_TYPE enum
 *
 *---------------------------------------------------------------------------*/
void Mavlink_Param_Set( void ) {

  float f_value;
  bool match;
  uint8_t i, j;

  f_value = *(float *)(&uc_rx_msg[0]);
  if ((uc_rx_msg[4] == uc_system_ID) &&                     /* message is for this system */
      (uc_rx_msg[5] == e_component_ID)) {                   /* message is for this component */
    for (i = 0; i < ONBOARD_PARAM_COUNT; i++) {             /* Search parameter in the list */
      match = TRUE;
      for (j = 0; j < ONBOARD_PARAM_NAME_LENGTH; j++) {
        if (uc_rx_msg[j + 6] != s_parameter_name[i][j]) {   /* compare name */
          match = FALSE;
        }
        if (s_parameter_name[i][j] == '\0') {               /* null termination is reached */
          j = ONBOARD_PARAM_NAME_LENGTH;                    /* force termination of for loop */
        }
      }
      if (match) {                                          /* name matched and */
        if ((f_param_value[i] != f_value) &&                /* there is a difference and */
           !isnan(f_value) &&                               /* new value is a number and */
           !isinf(f_value) &&                               /* is NOT infinity and */
           (uc_rx_msg[22] == (uint8_t)MAVLINK_TYPE_FLOAT)) { /* is a float */
          f_param_value[i] = f_value;                       /* write changes */
          Mavlink_Param_Send(i, 1);                         /* emit changes */
        }
      }
    }
  }
}

/*----------------------------------------------------------------------------
 *
 * @brief   Get HIL status
 * @param   -
 * @returns -
 * @remarks
 * Name = MAVLINK_MSG_ID_HIL_STATE
 *
 * Field       Offset Type     Meaning
 * -------------------------------------
 * time_usec      0   uint64_t
 * roll           8   float
 * pitch         12   float
 * yaw           16   float
 * rollspeed     20   float
 * pitchspeed    24   float
 * yawspeed      28   float
 * lat           32   int32_t
 * lon           36   int32_t
 * alt           40   int32_t
 * vx            44   int16_t
 * vy            46   int16_t
 * vz            48   int16_t
 * xacc          50   int16_t
 * yacc          52   int16_t
 * zacc          54   int16_t
 *
 *---------------------------------------------------------------------------*/
void Mavlink_HIL_State( void ) {
#if (0)
     rollspeed = *((float   *)(&uc_rx_msg[26]));    /*  */
    pitchspeed = *((float   *)(&uc_rx_msg[30]));    /*  */
      yawspeed = *((float   *)(&uc_rx_msg[34]));    /*  */
           lat = *((int32_t *)(&uc_rx_msg[38]));    /*  */
           lon = *((int32_t *)(&uc_rx_msg[42]));    /*  */
           alt = *((int32_t *)(&uc_rx_msg[46]));    /*  */
          xacc = *((int16_t *)(&uc_rx_msg[56]));    /*  */
          yacc = *((int16_t *)(&uc_rx_msg[58]));    /*  */
          zacc = *((int16_t *)(&uc_rx_msg[60]));    /*  */
#endif
}

/*----------------------------------------------------------------------------
 *
 * @brief   Decode "request data stream" message
 * @param   -
 * @returns -
 * @remarks
 * Name = REQUEST_DATA_STREAM, ID = 66, Length = 6
 *
 * Field         Offset Type   Meaning
 * ----------------------------------------------------------------------
 * req_message_rate 0 uint16_t interval between two messages of this type
 * target_system    2 uint8_t  target requested to send the stream
 * target_component 3 uint8_t  component requested to send the stream
 * req_stream_id    4 uint8_t  ID of the requested data stream
 * start_stop       5 uint8_t  1 to start sending, 0 to stop sending
 *
 *---------------------------------------------------------------------------*/
 void Mavlink_Data_Stream( void ) {

    uint8_t freq;

    if (uc_rx_msg[5] == 0) {        /* stop */
        freq = 0;                   /* force frequency = 0 */
    } else if (uc_rx_msg[5] == 1) { /* start */
        freq = uc_rx_msg[0];
    } else {                        /* wrong start / stop field */
        return;
    }

    switch(uc_rx_msg[4]) {          /* stream ID */

        case MAV_DATA_STREAM_ALL:
            uc_stream_rate[MAV_DATA_STREAM_RAW_SENSORS    ] = freq;
            uc_stream_rate[MAV_DATA_STREAM_EXTENDED_STATUS] = freq;
            uc_stream_rate[MAV_DATA_STREAM_RC_CHANNELS    ] = freq;
            uc_stream_rate[MAV_DATA_STREAM_RAW_CONTROLLER ] = freq;
            uc_stream_rate[MAV_DATA_STREAM_POSITION       ] = freq;
            uc_stream_rate[MAV_DATA_STREAM_EXTRA1         ] = freq;
            uc_stream_rate[MAV_DATA_STREAM_EXTRA2         ] = freq;
            uc_stream_rate[MAV_DATA_STREAM_EXTRA3         ] = freq;
            break;

        case MAV_DATA_STREAM_RAW_SENSORS:
            uc_stream_rate[MAV_DATA_STREAM_RAW_SENSORS] = freq;
            break;

        case MAV_DATA_STREAM_EXTENDED_STATUS:
            uc_stream_rate[MAV_DATA_STREAM_EXTENDED_STATUS] = freq;
            break;

        case MAV_DATA_STREAM_RC_CHANNELS:
            uc_stream_rate[MAV_DATA_STREAM_RC_CHANNELS] = freq;
            break;

        case MAV_DATA_STREAM_RAW_CONTROLLER:
            uc_stream_rate[MAV_DATA_STREAM_RAW_CONTROLLER] = freq;
            break;

        case MAV_DATA_STREAM_POSITION:
            uc_stream_rate[MAV_DATA_STREAM_POSITION] = freq;
            break;

        case MAV_DATA_STREAM_EXTRA1:
            uc_stream_rate[MAV_DATA_STREAM_EXTRA1] = freq;
            break;

        case MAV_DATA_STREAM_EXTRA2:
            uc_stream_rate[MAV_DATA_STREAM_EXTRA2] = freq;
            break;

        case MAV_DATA_STREAM_EXTRA3:
            uc_stream_rate[MAV_DATA_STREAM_EXTRA3] = freq;
            break;

        default:
            break;
    }
}

/*----------------------------------------------------------------------------
 *
 * @brief   Send total number of waypoints
 * @param   -
 * @returns -
 * @remarks 
 * This message is sent as response to MISSION_REQUEST_LIST by the vehicle
 * and to initiate a mission write transaction.
 * The GCS can then request the individual mission item based on the knowledge 
 * of the total number of MISSIONs.
 *
 * Name = MAVLINK_MSG_ID_MISSION_COUNT, ID = 44, Length = 4
 *
 * Field       Offset Type     Meaning
 * ----------------------------------------------------------------------
 * count            0 uint16_t number of waypoints
 * target_system    2 uint8_t  target requested to send the stream
 * target_component 3 uint8_t  component requested to send the stream
 *
 *---------------------------------------------------------------------------*/
void Mavlink_Mission_Count_Send( void ) {

  if ((uc_rx_msg[0] == uc_system_ID) &&     /* message is for this system */
      (uc_rx_msg[1] == e_component_ID)) {   /* message is for this component */
                    uc_tx_msg[1]   = 4;                             /* payload length */
                    uc_tx_msg[5]   = MAVLINK_MSG_ID_MISSION_COUNT;  /* mission count message ID */
    *((uint16_t *)(&uc_tx_msg[6])) = Nav_Wpt_Number_Get();          /* number of waypoints */

    Mavlink_Send(ui_mavlink_crc[MAVLINK_MSG_ID_MISSION_COUNT]);
  }
}

/*----------------------------------------------------------------------------
 *
 * @brief   Start receiving mission waypoints
 * @param   -
 * @returns -
 * @remarks 
 * This action is taken in response to a MISSION_COUNT message
 *
 * Name = MAVLINK_MSG_ID_MISSION_COUNT, ID = 44, Length = 4
 *
 * Field       Offset Type     Meaning
 * ----------------------------------------------------------------------
 * count            0 uint16_t number of waypoints
 * target_system    2 uint8_t  target requested to send the stream
 * target_component 3 uint8_t  component requested to send the stream
 *
 *---------------------------------------------------------------------------*/
void Mavlink_Mission_Count_Get( void ) {

  if ((uc_rx_msg[2] != uc_system_ID) ||     /* message is NOT for this system */
      (uc_rx_msg[3] != e_component_ID)) {   /* message is NOT for this component */
    return;
  }
  ui_wpt_get_total = *((uint16_t *)(&uc_rx_msg[0]));  /* number of waypoints to be received */
  if (Nav_Wpt_Number_Set(ui_wpt_get_total)) {         /* set total number of waypoints */
    ui_wpt_get_index = 0;                             /* clear waypoint counter */
    b_wpt_receiving  = TRUE;                          /* we're receiving mission wpts */
  } else {                                            /* waypoints number exceeds storage space */
    Mavlink_Mission_Ack(MAV_MISSION_NO_SPACE);        /* send error */
  }
}

/*----------------------------------------------------------------------------
 *
 * @brief   Send waypoint data
 * @param   -
 * @param   -
 * @returns -
 * @remarks
 * Name = MAVLINK_MSG_ID_MISSION_ITEM, ID = 39, Length = 37
 *
 * Field        Offset Type     Meaning
 * --------------------------------------------------------------------
 * param1            0 float    acceptance radius
 * param2            4 float    permanence time
 * param3            8 float    orbit direction
 * param4           12 float    yaw orientation
 * x                16 float    x position or latitude
 * y                20 float    y position or longitude
 * z                24 float    z position or altitude
 * seq              28 uint16_t sequence
 * command          30 uint16_t scheduled action
 * target_system    32 uint8_t  target requested to send the stream
 * target_component 33 uint8_t  component requested to send the stream
 * frame            34 uint8_t  coordinate system
 * current          35 uint8_t  false:0, true:1
 * auto_continue    36 uint8_t  autocontinue to next wp
 *
 *---------------------------------------------------------------------------*/
void Mavlink_Mission_Item_Send( void ) {

  uint16_t index;

  if ((uc_rx_msg[2] == uc_system_ID) &&     /* message is for this system */
      (uc_rx_msg[3] == e_component_ID)) {   /* message is for this component */

    index = *((uint16_t *)(&uc_rx_msg[0])); /* get waypoint index */
    Nav_Wpt_Get(index, &wpt);               /* get waypoint data */

                    uc_tx_msg[ 1]   = 37;                           /* payload length */
                    uc_tx_msg[ 5]   = MAVLINK_MSG_ID_MISSION_ITEM;  /* message ID */
    *((float    *)(&uc_tx_msg[ 6])) = 100.0f;                       /* radius */
    *((float    *)(&uc_tx_msg[10])) =   0.0f;                       /* time */
    *((float    *)(&uc_tx_msg[14])) =   0.0f;                       /* orbit */
    *((float    *)(&uc_tx_msg[18])) =   0.0f;                       /* yaw */
    *((float    *)(&uc_tx_msg[22])) = wpt.lat;                      /* latitude */
    *((float    *)(&uc_tx_msg[26])) = wpt.lon;                      /* longitude */
    *((float    *)(&uc_tx_msg[30])) = wpt.alt;                      /* altitude */
    *((uint16_t *)(&uc_tx_msg[34])) = index;                        /* sequence */
    *((uint16_t *)(&uc_tx_msg[36])) = (uint8_t)MAV_CMD_NAV_WAYPOINT;/* command */
                    uc_tx_msg[38]   = 1;                            /* target sys */
                    uc_tx_msg[39]   = 1;                            /* target comp */
                    uc_tx_msg[40]   = (uint8_t)MAV_FRAME_GLOBAL;    /* frame */
                    uc_tx_msg[41]   = 0;                            /* current */
                    uc_tx_msg[42]   = 1;                            /* auto continue */

    Mavlink_Send(ui_mavlink_crc[MAVLINK_MSG_ID_MISSION_ITEM]);
  }
}

/*----------------------------------------------------------------------------
 *
 * @brief   Receive waypoint data
 * @param   -
 * @param   -
 * @returns -
 * @remarks
 * Name = MAVLINK_MSG_ID_MISSION_ITEM, ID = 39, Length = 37
 *
 * Field        Offset Type     Meaning
 * --------------------------------------------------------------------
 * param1            0 float    acceptance radius
 * param2            4 float    permanence time
 * param3            8 float    orbit direction
 * param4           12 float    yaw orientation
 * x                16 float    x position or latitude
 * y                20 float    y position or longitude
 * z                24 float    z position or altitude
 * seq              28 uint16_t sequence
 * command          30 uint16_t scheduled action
 * target_system    32 uint8_t  target requested to send the stream
 * target_component 33 uint8_t  component requested to send the stream
 * frame            34 uint8_t  coordinate system
 * current          35 uint8_t  false:0, true:1
 * auto_continue    36 uint8_t  autocontinue to next wp
 *
 *---------------------------------------------------------------------------*/
void Mavlink_Mission_Item_Get( void ) {
  uint16_t j;
  uint16_t result;
  uint16_t command;
  uint8_t  frame;

  result = MAV_MISSION_ACCEPTED;
#if(0)
   wpt.radius = *((float    *)(&uc_rx_msg[ 0]));    /* radius */
     wpt.time = *((float    *)(&uc_rx_msg[ 4]));    /* time */
    wpt.orbit = *((float    *)(&uc_rx_msg[ 8]));    /* orbit */
      wpt.yaw = *((float    *)(&uc_rx_msg[12]));    /* yaw */
     wpt.curr =                 uc_rx_msg[35];      /* current */
     wpt.cont =                 uc_rx_msg[36];      /* auto continue */
#endif
      wpt.lat = *((float    *)(&uc_rx_msg[16]));    /* latitude */
      wpt.lon = *((float    *)(&uc_rx_msg[20]));    /* longitude */
      wpt.alt = *((float    *)(&uc_rx_msg[24]));    /* altitude */
            j = *((uint16_t *)(&uc_rx_msg[28]));    /* sequence */
      command = *((uint16_t *)(&uc_rx_msg[30]));    /* command */
        frame =                 uc_rx_msg[34];      /* reference frame */

  if ((uc_rx_msg[32] != uc_system_ID) ||            /* message is NOT for this system */
      (uc_rx_msg[33] != e_component_ID)) {          /* message is NOT for this component */
    result = MAV_MISSION_DENIED;
  } else if (TRUE != b_wpt_receiving) {             /* mission protocol NOT in progress */
    result = MAV_MISSION_ERROR;
  } else if (j != ui_wpt_get_index) {               /* sequence DOESN'T match expected index */
    result = MAV_MISSION_INVALID_SEQUENCE;
  } else if (isnan(wpt.lat) || isinf(wpt.lat)) {    /* latitude is NOT a number or is infinity */
    result = MAV_MISSION_INVALID;
  } else if (isnan(wpt.lon) || isinf(wpt.lon)) {    /* longitude is NOT a number or is infinity */
    result = MAV_MISSION_INVALID;
  } else if (isnan(wpt.alt) || isinf(wpt.alt)) {    /* altitude is NOT a number or is infinity */
    result = MAV_MISSION_INVALID;
  } else if (command != MAV_CMD_NAV_WAYPOINT) {     /* command is not a navigation waypoint */
    result = MAV_MISSION_UNSUPPORTED;
  } else if ((frame != MAV_FRAME_MISSION) &&        /* reference frame is NOT mission frame */
             (frame != MAV_FRAME_GLOBAL)) {         /* reference frame is NOT global frame */
    result = MAV_MISSION_UNSUPPORTED_FRAME;
  }
  if (result == MAV_MISSION_ACCEPTED) {
    Nav_Wpt_Set(ui_wpt_get_index++, &wpt);          /* set waypoint data */
    if (ui_wpt_get_index == ui_wpt_get_total) {     /* received all waypoints */
      b_wpt_receiving = FALSE;                      /* close mission protocol */
      Mavlink_Mission_Ack(result);
    }
  } else {
    Mavlink_Mission_Ack(result);                    /* send error */
  }
}

/*----------------------------------------------------------------------------
 *
 * @brief   Send mission ack packet
 * @param   type = mission result, see MAV_MISSION_RESULT enum
 * @returns -
 * @remarks 
 *
 * Name = MAVLINK_MSG_ID_MISSION_ACK, ID = 47, Length = 3
 *
 * Field         Offset  Type    Meaning
 * ----------------------------------------------------------------------------
 *
 *---------------------------------------------------------------------------*/
void Mavlink_Mission_Ack( uint8_t type ) {
  uint16_t j;

  for (j = 0; j < PACKET_LEN; j++) {
    uc_tx_msg[j] = 0;
  }

  uc_tx_msg[ 1] = 3;                            /* payload length */
  uc_tx_msg[ 5] = MAVLINK_MSG_ID_MISSION_ACK;   /* ACK message ID */
  uc_tx_msg[ 6] = type;                         /* mission result */

  Mavlink_Send(ui_mavlink_crc[MAVLINK_MSG_ID_MISSION_ACK]);
}

/*----------------------------------------------------------------------------
 *
 * @brief   Request waypoint data
 * @param   -
 * @param   -
 * @returns -
 * @remarks
 * Name = MAVLINK_MSG_ID_MISSION_REQUEST, ID = 40, Length = 4
 *
 * Field        Offset Type     Meaning
 * --------------------------------------------------------------------
 * target_system     uint8_t  target requested to send the stream
 * target_component  uint8_t  component requested to send the stream
 * seq               uint16_t sequence
 *
 *---------------------------------------------------------------------------*/
void Mavlink_Wpt_Request( void ) {

  if ((b_wpt_receiving == TRUE) &&
      (ui_wpt_get_index <= ui_wpt_get_total)) {
                    uc_tx_msg[1]   = 4;                              /* payload length */
                    uc_tx_msg[5]   = MAVLINK_MSG_ID_MISSION_REQUEST; /* mission request message ID */
    *((uint16_t *)(&uc_tx_msg[6])) = ui_wpt_get_index;               /* waypoint index */

    Mavlink_Send(ui_mavlink_crc[MAVLINK_MSG_ID_MISSION_REQUEST]);
  }
}

/*----------------------------------------------------------------------------
 *
 * @brief   Parse communication packets
 * @param   -
 * @returns -
 * @remarks This function decodes packets on the protocol level.
 *
 *---------------------------------------------------------------------------*/
static bool Mavlink_Parse(void) {

  msg_t c;
#if (0)
  static uint8_t magic;
  static uint8_t sysid;
  static uint8_t compid;
  static uint8_t buffer_overrun = 0;
  static uint8_t parse_error = 0;
  static uint8_t packet_rx_success_count = 0;
#endif
  static mavlink_parse_state_t parse_state = MAVLINK_PARSE_STATE_UNINIT;
  static uint8_t len;
  static uint8_t packet_idx;
  static uint8_t seq;
  static uint8_t current_rx_seq;
  bool msg_received = FALSE;

  c = chnGetTimeout(&SD1, TIME_IMMEDIATE);
  while (c != Q_TIMEOUT) {
    switch (parse_state) {
      case MAVLINK_PARSE_STATE_UNINIT:
      case MAVLINK_PARSE_STATE_IDLE:
        if (c == MAVLINK_STX) {
          parse_state = MAVLINK_PARSE_STATE_GOT_STX;
          len = 0;
           /* magic = c; */
          checksum_init();
        }
        break;

      case MAVLINK_PARSE_STATE_GOT_STX:
        if (msg_received || (c > PAYLOAD_LEN)) {
        /* buffer_overrun++; */
        /* parse_error++; */
          msg_received = FALSE;
          parse_state = MAVLINK_PARSE_STATE_IDLE;
        } else {
          len = c; /* NOT counting STX, LENGTH, SEQ, SYSID, COMPID, MSGID, CRC1 and CRC2 */
          packet_idx = 0;
          checksum_accumulate(c);
          parse_state = MAVLINK_PARSE_STATE_GOT_LENGTH;
        }
        break;

      case MAVLINK_PARSE_STATE_GOT_LENGTH:
        seq = c;
        checksum_accumulate(c);
        parse_state = MAVLINK_PARSE_STATE_GOT_SEQ;
        break;

      case MAVLINK_PARSE_STATE_GOT_SEQ:
      /* sysid = c; */
        checksum_accumulate(c);
        parse_state = MAVLINK_PARSE_STATE_GOT_SYSID;
        break;

      case MAVLINK_PARSE_STATE_GOT_SYSID:
      /* compid = c; */
        checksum_accumulate(c);
        parse_state = MAVLINK_PARSE_STATE_GOT_COMPID;
        break;

      case MAVLINK_PARSE_STATE_GOT_COMPID:
        uc_msg_ID = c;
        checksum_accumulate(c);
        if (len == 0) {
          parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
        } else {
          parse_state = MAVLINK_PARSE_STATE_GOT_MSGID;
        }
        break;

      case MAVLINK_PARSE_STATE_GOT_MSGID:
        uc_rx_msg[packet_idx++] = c;
        checksum_accumulate(c);
        if (packet_idx == len) {
          parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
        }
        break;

      case MAVLINK_PARSE_STATE_GOT_PAYLOAD:
        checksum_accumulate(ui_mavlink_crc[uc_msg_ID]);
        if (c != (ui_crc & 0xFF)) { /* Check first checksum byte */
          /* parse_error++; */
          msg_received = FALSE;
          parse_state = MAVLINK_PARSE_STATE_IDLE;
          if (c == MAVLINK_STX) {
            parse_state = MAVLINK_PARSE_STATE_GOT_STX;
            len = 0;
            checksum_init();
          }
        } else {
          parse_state = MAVLINK_PARSE_STATE_GOT_CRC1;
          uc_rx_msg[packet_idx] = c;
        }
        break;

      case MAVLINK_PARSE_STATE_GOT_CRC1:
        if (c != (ui_crc >> 8)) {     /* Check second checksum byte */
         /* parse_error++; */
          msg_received = FALSE;
          parse_state = MAVLINK_PARSE_STATE_IDLE;
          if (c == MAVLINK_STX) {
            parse_state = MAVLINK_PARSE_STATE_GOT_STX;
            len = 0;
            checksum_init();
          }
        } else {                  /* Successfully got message */
          msg_received = TRUE;
          parse_state = MAVLINK_PARSE_STATE_IDLE;
          uc_rx_msg[packet_idx + 1] = c;
          /* memcpy(r_message, rxmsg, sizeof(mavlink_message_t)); */
        }
        break;
      }

      /* If a message has been sucessfully decoded, check index */
      if (msg_received) {
        current_rx_seq = seq;
      /*
       * Initial condition:
       * if no packet has been received so far, drop count is undefined
       */
//    if (packet_rx_success_count == 0) packet_rx_drop_count = 0;
        /* Count this packet as received */
//    packet_rx_success_count++;
      }

      current_rx_seq = current_rx_seq + 1;
//  packet_rx_drop_count = parse_error;
//  parse_error = 0;

    c = chnGetTimeout(&SD1, TIME_IMMEDIATE);
  }

  return (msg_received);
}

/*----------------------------------------------------------------------------
 *
 * @brief   Receive mavlink packets
 * @param   -
 * @returns -
 * @remarks Handles packet value by calling the appropriate functions.
 *          See Ardupilot function handleMessage at following link:
 * https://github.com/diydrones/ardupilot/blob/master/ArduPlane/GCS_Mavlink.pde
 *
 *---------------------------------------------------------------------------*/
void Mavlink_Receive(void)
{
  if (Mavlink_Parse()) {                        /* Received a correct packet */
    switch (uc_msg_ID) {                        /* Handle message */
      case MAVLINK_MSG_ID_HEARTBEAT:            /* Read GCS heartbeat, go into comm lost if time out */					
        break;
      case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:  /* Start sending data stream */
        Mavlink_Data_Stream();
        break;
      case MAVLINK_MSG_ID_COMMAND_LONG:         /* */
        break;
      case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:   /* Start sending parameter list */
        ui_parameter_i = 0;
        break;
      case MAVLINK_MSG_ID_PARAM_SET:            /* Read parameter data and set it */
        Mavlink_Param_Set();
        break;
      case MAVLINK_MSG_ID_PARAM_VALUE:          /* */
        break;
      case MAVLINK_MSG_ID_HIL_STATE:            /* Send HIL state */
        Mavlink_HIL_State();
        break;
      case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: /* Send total waypoint number */
        Mavlink_Mission_Count_Send();
        break;
      case MAVLINK_MSG_ID_MISSION_REQUEST:      /* Send data of specific waypoint */
        Mavlink_Mission_Item_Send();
        break;
      case MAVLINK_MSG_ID_MISSION_COUNT:        /* Start receiving mission waypoints */
        Mavlink_Mission_Count_Get();
        break;
      case MAVLINK_MSG_ID_MISSION_ITEM:         /* Read waypoint data and set it */
        Mavlink_Mission_Item_Get();
        break;
      default:                                  /* Do nothing */
        break;
    }
  }                                             /* */
/*  packet_drops += packet_rx_drop_count; */      /* Update global packet drops counter */
}

/*----------------------------------------------------------------------------
 *
 * @brief   Send low-priority messages at a maximum rate of xx Hertz
 * @param   -
 * @returns -
 * @remarks This function sends messages at a lower rate to not exceed the
 *          wireless bandwidth. It sends one message each time it is called
 *          until the buffer is empty.
 *          Call function at 50 Hertz to implement periods indicated below.
 *          See Ardupilot function queued_param_send() at link:
 * https://github.com/diydrones/ardupilot/blob/master/ArduPlane/GCS_Mavlink.pde
 *
 *---------------------------------------------------------------------------*/
void Mavlink_Queued_Send ( void ) {

  static uint8_t uc_send_period = 0;

  if ((uc_send_period % 10) == 0) {             /* @ 5 Hz       */
    Mavlink_Wpt_Request();                      /* request next mission waypoint */
    if (ui_parameter_i < ONBOARD_PARAM_COUNT) { /* there are parameters to be sent */
      Mavlink_Param_Send(ui_parameter_i++, 
                         ONBOARD_PARAM_COUNT);  /* send parameters */
    }
  } else if ((uc_send_period % 50) == 0) {      /* @ 1 Hz       */
    Mavlink_Heartbeat();                        /* send heartbeat */
  } else if ((uc_send_period % 200) == 0) {     /* @ 0.25 Hz    */
                                                /* do something at 0.25 Hz */
  } else {                                      /* @ 50 Hz      */
                                                /* do something at 50 Hz */
  }
  if (++uc_send_period > 200) {
    uc_send_period = 0;
  }
}

/*----------------------------------------------------------------------------
 *
 * @brief   Send data stream
 * @param   -
 * @returns -
 * @remarks call function @ 50 Hz
 *
 *---------------------------------------------------------------------------*/
void Mavlink_Stream_Send(void)
{
#if (0)
    if (_queued_parameter != NULL) {
        if (streamRateParams.get() <= 0) {
            streamRateParams.set(50);
        }
        if (stream_trigger(STREAM_PARAMS)) {
            send_message(MSG_NEXT_PARAM);
        }
    }
#endif

    if (Mavlink_Stream_Trigger((uint8_t)MAV_DATA_STREAM_RAW_SENSORS)) {
        /* MSG_RAW_IMU1 */
        /* MSG_RAW_IMU2 */
        /* MSG_RAW_IMU3 */
    }

    if (Mavlink_Stream_Trigger((uint8_t)MAV_DATA_STREAM_EXTENDED_STATUS)) {
        /* MSG_EXTENDED_STATUS1 */
        /* MSG_EXTENDED_STATUS2 */
        /* MSG_CURRENT_WAYPOINT */
        /* MSG_GPS_RAW TODO - remove this message after location message is working */
        /* MSG_NAV_CONTROLLER_OUTPUT */
        /* MSG_FENCE_STATUS */
    }

    if (Mavlink_Stream_Trigger((uint8_t)MAV_DATA_STREAM_RC_CHANNELS)) {
        /* MSG_RADIO_OUT */
        /* MSG_RADIO_IN */
    }

    if (Mavlink_Stream_Trigger((uint8_t)MAV_DATA_STREAM_RAW_CONTROLLER)) {
        /* MSG_SERVO_OUT */
    }

    if (Mavlink_Stream_Trigger((uint8_t)MAV_DATA_STREAM_POSITION)) {
        Mavlink_Position();
    }

    if (Mavlink_Stream_Trigger((uint8_t)MAV_DATA_STREAM_EXTRA1)) {
        Mavlink_Attitude();
        /* MSG_SIMSTATE */
    }

    if (Mavlink_Stream_Trigger((uint8_t)MAV_DATA_STREAM_EXTRA2)) {
        Mavlink_Hud();
    }

    if (Mavlink_Stream_Trigger((uint8_t)MAV_DATA_STREAM_EXTRA3)) {
        /* MSG_AHRS */
        /* MSG_HWSTATUS */
        /* MSG_WIND */
    }
}

/*----------------------------------------------------------------------------
 *
 * @brief   Updates stream tick and tells if it must be transmitted
 * @param   stream = ID of strem to be checked
 * @returns TRUE if stream must be transmitted
 * @remarks Working principle:
 *     - if there's a "queued parameter" or we're receiving waypoints,
 *       transmission frequency is divided by 4
 *     - queued parameters are those requested with PARAM_REQUEST_LIST
 *     - slowdown factor is always < 100 and depends on tx buffer space:
 *         buffer space < 20  slowdown +3
 *         buffer space < 50  slowdown +1
 *         buffer space > 95  slowdown -2
 *         buffer space > 90  slowdown -1
 *
 *---------------------------------------------------------------------------*/
bool Mavlink_Stream_Trigger(uint8_t stream)
{
    uint8_t rate = uc_stream_rate[stream];

    /* send at a much lower rate while handling waypoints and parameter sends */
#if (0)
    if (waypoint_receiving || _queued_parameter != NULL ) {
        rate >>= 2;
    }
#endif
    if (rate == 0) {
        return FALSE;
    }

    if (uc_stream_tick[stream] == 0) {   /* we're triggering now, */
        if (rate > MAX_STREAM_RATE) {    /* setup the next trigger point */
            rate = MAX_STREAM_RATE;
        }
        uc_stream_tick[stream] = (MAX_STREAM_RATE / rate) /* + stream_slowdown */;
        return TRUE;
    }

    uc_stream_tick[stream]--;            /* count down at 50Hz */
    return FALSE;
}

/*----------------------------------------------------------------------------
 *
 * @brief   get telemetry sensors
 * @param   -
 * @returns pointer to raw sensor values
 * @remarks
 *
 *---------------------------------------------------------------------------*/
void Telemetry_Get_Sensors(int16_t * piSensors)
{
   (void) piSensors;
}

/*----------------------------------------------------------------------------
 *
 * @brief   get telemetry gains
 * @param[in] gain identifier of gain
 * @returns gain value
 * @remarks
 *
 *---------------------------------------------------------------------------*/
float Telemetry_Get_Gain(ENUM_GAIN gain)
{
    float f_result;

    switch (gain) {
        case TEL_ROLL_KP :
        case TEL_ROLL_KI :
        case TEL_PITCH_KP :
        case TEL_PITCH_KI :
        case TEL_ALT_KP :
        case TEL_ALT_KI :
        case TEL_NAV_KP :
        case TEL_NAV_KI :
            f_result = f_param_value[gain];
            break;

        case TEL_NAV_BANK :
            f_result = DEGTORAD(f_param_value[gain]);
            break;

        case TEL_GAIN_NUMBER :
        default :
            f_result = 0.0f;
            break;
    }
    return f_result;
}

/*----------------------------------------------------------------------------
 *
 * @brief   get telemetry speed
 * @param   -
 * @returns telemetry speed [?]
 * @remarks
 *
 *---------------------------------------------------------------------------*/
float Telemetry_Get_Speed(void)
{
    return 0.0f;    /* fTrueAirSpeed; */
}

/*----------------------------------------------------------------------------
 *
 * @brief   get telemetry altitude
 * @param   -
 * @returns telemetry altitude [m]
 * @remarks
 *
 *---------------------------------------------------------------------------*/
float Telemetry_Get_Altitude(void)
{
    return 0.0f;    /* fAltitude; */
}

