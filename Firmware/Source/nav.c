/**===========================================================================+
 *
 * $HeadURL: $
 * $Revision: $
 * $Date:  $
 * $Author: $
 *
 * @brief navigation task
 *
 * @file
 * - Initialization:
 *   reads from SD card a text file containing waypoints, translates strings
 *   into coordinates and altitude, saves waypoints in the array waypoint[],
 *   updates number of available waypoints.
 *   If SD card is missing or in case of error during file read, the number
 *   of available waypoints is set to 0.
 * - Navigation:
 *   waits for GPS fix, saves coordinates of launch point in the first entry
 *   of array waypoint[], computes heading and distance to next waypoint.
 *   If available waypoints are 0, computes heading and distance to launch
 *   point (RTL).
 *   Navigation error is the difference (heading - bearing), sign corrected
 *   when < -180° or > 180°. Cross product and dot product of heading vector
 *   with bearing vector doesn't work because bearing vector is not a versor.
 *
 * @todo
 * 1) Compute longitude and latitude differences as :
 * \code
 *     Delta Lat = Lat2 - Lat1
 *     Delta Lon = (Lon2 - Lon1) * cos((Lat1 + Lat2)/2)
 * \endcode
 *
 * @todo
 * 2) Compute distance from waypoint as :
 * \code
 *     Distance = sqrt(Delta Lon ^ 2 + Delta Lat ^ 2) * 111320
 * \endcode
 *
 * Change: function print_waypoint() returns line length 
 *         function Nav_Store_Waypoints(): corrected file write
 *
 *============================================================================*/

#include "ch.h"
#include "math.h"
#include "config.h"
#include "rc.h"
#include "gps.h"
#include "baro.h"
/*#include "simulator.h"*/
#include "ff.h"
#include "nav.h"

/*--------------------------------- Definitions ------------------------------*/

#define MAX_WAYPOINTS   8       /* maximum number of waypoints */
#define MIN_DISTANCE    100     /* minimum distance from waypoint [m] */
#define LINE_LENGTH     48      /* length of lines read from file */
#define BUFFER_LENGTH   128     /* length of file buffer */

#if (SIMULATOR == SIM_NONE)                         /* normal mode */
#define Get_Altitude() Get_Baro_Altitude()          /* get barometric altitude */
#else                                               /* simulation mode */
#define Get_Altitude() Simulator_Get_Altitude();    /* get simulator altitude */
#endif

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

extern bool_t fs_ready;

/*----------------------------------- Locals ---------------------------------*/

static       uint8_t    uc_buffer[BUFFER_LENGTH];       /* file buffer */
static       uint8_t    sz_line[LINE_LENGTH];           /* line */
static       STRUCT_WPT waypoint[MAX_WAYPOINTS];        /* waypoints array */
static const uint8_t    sz_wpt_file[16] = "path.txt";   /* waypoint file name */
static       FIL        nav_file;                       /* file structure */
static       UINT       w_bytes       = 0;              /* counter of file bytes */
static       uint16_t   ui_distance   = 0;              /* distance to destination [m] */
static       uint16_t   ui_wpt_number = 0;              /* total number of waypoints */
static       uint16_t   ui_wpt_index  = 0;              /* waypoint index */
static       float      f_alt_error   = 0.0f;           /* altitude error [m] */
static       float      f_curr_lon    = 0.0f;           /* current longitude */
static       float      f_curr_lat    = 0.0f;           /* current latitude */
static       float      f_curr_alt    = 0.0f;           /* current altitude [m] */
static       float      f_dest_lat    = 0.0f;           /* destination latitude */
static       float      f_dest_lon    = 0.0f;           /* destination longitude */
static       float      f_dest_alt    = 0.0f;           /* destination altitude */
static       float      f_bearing     = 0.0f;           /* angle to destination [°] */
static       bool       b_home        = FALSE;          /* home position saved */
static       bool       b_modified    = FALSE;          /* mission moodified by GCS */

/*--------------------------------- Prototypes -------------------------------*/

static bool    parse_waypoint ( const uint8_t * psz_line );
static uint8_t print_waypoint ( uint16_t wpt_num, uint8_t * psz_line );

/*----------------------------------------------------------------------------
 *
 * @brief   navigation
 * @param   -
 * @remarks Bearing is computed as :
 *
 *             PI/2 - atan2(latitude difference, longitude difference)
 *
 *          Rationale:
 *          - the sign is changed because angles returned by atan2() are
 *            positive CCW, whereas bearing angles are positive CW.
 *          - the PI/2 offset is added because atan2 angles are measured
 *            from X axis (East direction), whereas bearing angles are
 *            measured from Y axis (North direction).
 *
 *----------------------------------------------------------------------------*/
void Navigation ( void ) {

  float f_temp, f_dx, f_dy;

  if (!b_home && (GPS_FIX == Gps_Status())) {

    b_home = TRUE;

    /* Save launch position as first waypoint */
    waypoint[0].lon = Gps_Longitude();
    waypoint[0].lat = Gps_Latitude();

    f_dest_lon = waypoint[ui_wpt_index].lon;    /* load destination longitude */
    f_dest_lat = waypoint[ui_wpt_index].lat;    /* load destination latitude */
    f_dest_alt = waypoint[ui_wpt_index].alt;    /* load destination altitude */

  } else {

    f_curr_alt = Get_Altitude();                    /* get altitude */
    f_curr_lon = Gps_Longitude();                   /* get longitude */
    f_curr_lat = Gps_Latitude();                    /* get latitude */

    /* Compute altitude error */
    f_temp = f_curr_alt - f_dest_alt;
    f_alt_error = f_temp;

    /* Get X and Y components of bearing */
    f_dx = f_dest_lon - f_curr_lon;
    f_dy = f_dest_lat - f_curr_lat;

    /* Compute bearing to waypoint */
    f_bearing = atan2f(f_dy, f_dx);
    f_bearing = (PI / 2.0f) - f_bearing;

    /* Compute distance to waypoint */
    f_temp = sqrtf((f_dy * f_dy) + (f_dx * f_dx));
    ui_distance = (uint16_t)(f_temp * 111113.7f);

    /* Check distance to waypoint */
    if (ui_distance < MIN_DISTANCE) {               /* waypoint reached */
      if (ui_wpt_number != 0) {                     /* waypoints do exist */
        if (++ui_wpt_index == ui_wpt_number) {      /* get next waypoint or */
          ui_wpt_index = 1;                         /* go back to first waypoint */
        }
      }
      f_dest_lon = waypoint[ui_wpt_index].lon;      /* new destination longitude */
      f_dest_lat = waypoint[ui_wpt_index].lat;      /* new destination latitude */
      f_dest_alt = waypoint[ui_wpt_index].alt;      /* new destination altitude */
    }
  }
}

/*----------------------------------------------------------------------------
 *
 * @brief   Load waypoints file from SD card
 * @param   -
 * @return  -
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
void Nav_Load_Waypoints( void ) {

    uint8_t * p_buffer;     /* pointer to file buffer */
    uint8_t * psz_line;     /* pointer to line read */
    uint8_t uc_counter;     /* counter of line characters */
    uint8_t c;              /* character read */
    bool_t b_error;         /* file read error */
    bool_t b_open;          /* file opened */

    b_open = FALSE;
    b_error = FALSE;
    psz_line = sz_line;                             /* init line pointer */
    uc_counter = LINE_LENGTH - 1;                   /* init counter of line characters */

    /* Check file system and open waypoints file */
    if (!fs_ready) {                                /* file system not mounted */
        ui_wpt_number = 0;                          /* no waypoint available */
    } else if (FR_OK != f_open(&nav_file, 
                               (const TCHAR *)sz_wpt_file, 
                               FA_READ)) {
                                                    /* error opening file */
        ui_wpt_number = 0;                          /* no waypoint available */
    } else {                                        /* file system mounted and */
        b_open = TRUE;                              /* file succesfully open */
        ui_wpt_number = 1;                          /* waypoint available */
    }

    /* Read waypoint file */
    while ((!b_error) && (b_open) &&
           (FR_OK == f_read(&nav_file, 
                            uc_buffer, 
                            BUFFER_LENGTH, 
                            &w_bytes))) {
        b_open = (w_bytes != 0);                    /* force file closed if end of file */
        p_buffer = uc_buffer;                       /* init buffer pointer */
        while ((w_bytes != 0) && (!b_error)) {      /* buffer not empty and no error */
            w_bytes--;                              /* decrease overall counter */
            c = *p_buffer++;                        /* read another character */
            if ( c == 10 ) {                        /* found new line */
                uc_counter = 0;                     /* reached end of line */
            } else if ( c == 13 ) {                 /* found carriage return */
                uc_counter--;                       /* decrease counter */
            } else {                                /* alphanumeric character */
                *psz_line++ = c;                    /* copy character */
                uc_counter--;                       /* decrease counter */
            }
            if (uc_counter == 0) {                  /* end of line */
                *psz_line = 0;                      /* append line delimiter */
                psz_line = sz_line;                 /* reset line pointer */
                uc_counter = LINE_LENGTH - 1;       /* reset char counter */
                b_error = parse_waypoint(sz_line);  /* parse line for waypoint */
            }
        }
    }
    ( void )f_close( &nav_file );                   /* close file */

    /* errors reading file */
    if (b_error) {
        ui_wpt_number = 0;                          /* no waypoint available */
        ui_wpt_index = 0;                           /* use launch position */
    } else {                                        /* no waypoint file */
        ui_wpt_index = 1;                           /* read first waypoint */
    }
}

/*----------------------------------------------------------------------------
 *
 * @brief   Store waypoints file into SD card
 * @param   -
 * @return  -
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
void Nav_Store_Waypoints( void ) {

  uint8_t * psz_line;         /* pointer to line read */
  uint16_t ui_wpt_counter;    /* waypoint counter */
  bool_t b_error;             /* file read error */
  bool_t b_open;              /* file opened */
  uint8_t length;             /* line length */

  if (!b_modified) {          /* mission hasn't been modified */
     return;
  }

  b_modified = FALSE;
  b_open = FALSE;
  b_error = FALSE;
  psz_line = sz_line;         /* init line pointer */
  ui_wpt_counter = 1;

  /* Check file system and open waypoints file */
  if ((fs_ready) &&                               /* file system mounted */
      (FR_OK == f_open(&nav_file,                 /* opening file */
                       (const TCHAR *)sz_wpt_file, 
                       FA_WRITE|FA_CREATE_ALWAYS))) {
    b_open = TRUE;                                /* file succesfully open */

  /* Write waypoint file */
    do {
      length = print_waypoint(ui_wpt_counter++, 
                              psz_line);
      if (FR_OK != f_write(&nav_file, 
                           psz_line, 
                           length, 
                           &w_bytes)) {
         b_error = TRUE;
      }
      b_open = (w_bytes != 0);                    /* force file closed if end of file */
    } while ((!b_error) && 
              (b_open) &&
              (ui_wpt_counter < ui_wpt_number));  /* init buffer pointer */
  }
  ( void )f_close( &nav_file );                   /* close file */

  /* errors reading file */
  if (b_error) {
    ui_wpt_number = 0;                            /* no waypoint available */
    ui_wpt_index = 0;                             /* use launch position */
  } else {                                        /* no waypoint file */
    ui_wpt_index = 1;                             /* start with first waypoint */
  }
}

/*----------------------------------------------------------------------------
 *
 * @brief   Parse string for waypoint coordinates
 * @return  TRUE if an error occurred, FALSE otherwise
 * @remarks format of waypoint coordinate is:
 *
 *          xx.xxxxxx,[ ]yy.yyyyyy,[ ]aaa[.[a]]\0
 *
 *          where :
 *
 *          x = longitude
 *          y = latitude
 *          a = altitude
 *          [ ] are zero or more spaces
 *          [.[a]] is an optional decimal point with an optional decimal data
 *
 *----------------------------------------------------------------------------*/
static bool parse_waypoint ( const uint8_t * psz_line ) {

  float f_temp;                       /* temporary */
  float f_div;                        /* divisor */
  uint8_t c;                          /* character read */
  uint8_t uc_field = 0;               /* field counter (lat, lon, alt) */
  uint8_t uc_counter = LINE_LENGTH;   /* counter of line characters */

  while (( uc_field < 3 ) && ( uc_counter > 0 )) {
    f_div = 1.0f;                                   /* initialize divisor */
    f_temp = 0.0f;                                  /* initialize temporary */
    c = *psz_line++;                                /* read char */
    /* leading spaces */
    while (( c == ' ' ) && ( uc_counter > 0 )) {
      c = *psz_line++;                              /* next char */
      uc_counter--;                                 /* count characters */
    }
    /* start of integer part */
    if (( c < '0' ) || ( c > '9' )) {               /* first char not numeric */
      return TRUE;                                  /* return error */
    }
    /* integer part */
    while (( c >= '0' ) && ( c <= '9' ) && ( uc_counter > 0 )) {
      f_temp = f_temp * 10.0f + (float)(c - '0');   /* accumulate */
      c = *psz_line++;                              /* next char */
      uc_counter--;                                 /* count characters */
    }
    /* decimal point */
    if (( c != '.' ) && ( uc_field != 2 )) {        /* altitude may lack decimal */
      return TRUE;
    } else {
      c = *psz_line++;                              /* skip decimal point */
      uc_counter--;                                 /* count characters */
    }
    /* fractional part */
    while (( c >= '0' ) && ( c <= '9' ) && ( uc_counter > 0 )) {
      if (f_div < 1000000.0f) {
        f_temp = f_temp * 10.0f + (float)(c - '0'); /* accumulate */
        f_div = f_div * 10.0f;                      /* update divisor */
      }
      c = *psz_line++;                              /* next char */
      uc_counter--;                                 /* count characters */
    }
    /* delimiter */
    if (( c != ',' ) && ( c != 0 )) {
      return TRUE;                                  /* error */
    } else {
      f_temp = f_temp / f_div;                      /* convert */
    }
    /* assign value and update field counter */
    switch ( uc_field++ ) {
      case 0: waypoint[ui_wpt_number  ].lon = f_temp; break;
      case 1: waypoint[ui_wpt_number  ].lat = f_temp; break;
      case 2: waypoint[ui_wpt_number++].alt = f_temp; break;
      default: break;
    }
  }
  return FALSE;
}

/*----------------------------------------------------------------------------
 *
 * @brief   Print waypoint coordinates into string
 * @return  string length
 * @remarks format of waypoint coordinate is:
 *
 *          xx.xxxxxx,[ ]yy.yyyyyy,[ ]aaa[.[a]]\0
 *
 *          where :
 *
 *          x = longitude
 *          y = latitude
 *          a = altitude
 *          [ ] are zero or more spaces
 *          [.[a]] is an optional decimal point with an optional decimal data
 *
 *----------------------------------------------------------------------------*/
static uint8_t print_waypoint ( uint16_t ui_num, uint8_t * psz_line ) {

  float f_temp;                       /* temporary */
  float f_fract;                      /* fractional part */
  int16_t i_whole;                    /* whole part */
  uint8_t i = 0;
  uint8_t j = 0;                      /*  */
  uint8_t uc_field = 0;               /* field counter (lat, lon, alt) */

  do {    /* assign value and update field counter */
    switch ( uc_field++ ) {
      case 0: f_temp = waypoint[ui_num].lon; break;
      case 1: f_temp = waypoint[ui_num].lat; break;
      case 2: f_temp = waypoint[ui_num].alt; break;
      default: break;
    }

    *psz_line++ = ' ';                  /* leading space */
    i++;                                /* count characters */
    i_whole = (int16_t)f_temp;          /* whole part */
    f_fract = f_temp - (float)i_whole;  /* fractional part */
    while (i_whole > 0) {
       *psz_line++ = '0' + (i_whole % 10);
       i++;                             /* count characters */
       i_whole = i_whole / 10;
    }

    *psz_line++ = '.';                  /* decimal point */
    i++;                                /* count characters */

    for (j = 1; j < 6; j++) {           /* precision is 6 digits after decimal point */
        f_fract = f_fract * 10.0f;
        f_temp = floor(f_fract);
        *psz_line++ = '0' + (uint8_t)f_temp;
        i++;                            /* count characters */
        f_fract = f_fract - f_temp;
    }
    if (uc_field < 2) {
        *psz_line++ = ',';              /* separate fields with comma */
        i++;                            /* count characters */
    } else {
        *psz_line++ = '\n';             /* new line */
        i++;                            /* count characters */
    }
  } while ( uc_field < 3 );

  return i;
}

/*----------------------------------------------------------------------------
 *
 * @brief   Get total waypoints number
 * @param   -
 * @return  waypoints number
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
uint16_t Nav_Wpt_Number_Get ( void ) {
  return ui_wpt_number;
}

/*----------------------------------------------------------------------------
 *
 * @brief   Set total waypoints number
 * @param   waypoints number
 * @return  FALSE if waypoints number exceeded maximum capacity, TURE otherwise
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
bool Nav_Wpt_Number_Set ( uint16_t num ) {

  if (num < MAX_WAYPOINTS) {            /* number fits storage */
    ui_wpt_number = num;
    return TRUE;
  } else {                              /* number exceeds storage capability */
    ui_wpt_number = 0;
    return FALSE;
  }
}

/*----------------------------------------------------------------------------
 *
 * @brief   Get waypoint index
 * @param   -
 * @return  waypoint index
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
uint16_t Nav_Wpt_Index ( void ) {
  return ui_wpt_index;
}

/*----------------------------------------------------------------------------
 *
 * @brief   Get waypoint altitude [m]
 * @param   -
 * @return  altitude
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
uint16_t Nav_Wpt_Altitude ( void ) {
  return (uint16_t)waypoint[ui_wpt_index].alt;
}

/*----------------------------------------------------------------------------
 *
 * @brief   Get waypoint data
 * @param   index = waypoint number
 * @return  waypoint data structure
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
void Nav_Wpt_Get ( uint16_t index, STRUCT_WPT * wpt ) {
  if (index > ui_wpt_number) {
     index = 0;
  }
  wpt->lat = waypoint[index].lat;
  wpt->lon = waypoint[index].lon;
  wpt->alt = waypoint[index].alt;
}

/*----------------------------------------------------------------------------
 *
 * @brief   Set waypoint data
 * @param   index = waypoint number
 * @param   wpt = waypoint data structure
 * @return  -
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
void Nav_Wpt_Set ( uint16_t index, STRUCT_WPT * wpt ) {

  if (index < MAX_WAYPOINTS) {
    waypoint[index].lat = wpt->lat;
    waypoint[index].lon = wpt->lon;
    waypoint[index].alt = wpt->alt;
  }
  if (index == ui_wpt_number - 1) {
    b_modified = TRUE;
  }
}

/*----------------------------------------------------------------------------
 *
 * @brief   Get computed bearing [°]
 * @param   -
 * @return  bearing angle in radian, between 0 and 2 * PI
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
float Nav_Bearing_Rad ( void ) {
  return f_bearing;
}

/*----------------------------------------------------------------------------
 *
 * @brief   Get distance to destination [m]
 * @param   -
 * @return  distance in meters
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
uint16_t Nav_Distance ( void ) {
  return ui_distance;
}

/*----------------------------------------------------------------------------
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

