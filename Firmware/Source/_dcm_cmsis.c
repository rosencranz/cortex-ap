/**============================================================================+
 *
 * $HeadURL: $
 * $Revision: $
 * $Date:  $
 * $Author: $
 *
 * @brief Direction Cosine Matrix calculations
 *
 * @file
 * Meaning of f_dcm_matrix rows / columns:
 * \code
 *         col    0              1               2
 *     row
 *                |              |               |
 *      0    -----|--------------|---------------|-----> earth X axis (west)
 *                |              |               |
 *      1    -----|--------------|---------------|-----> earth Y axis (north)
 *                |              |               |
 *      2    -----|--------------|---------------|-----> earth Z axis (down)
 *                |              |               |
 *                v              v               v
 *           plane X axis   plane Y axis    plane Z axis
 *
 * \endcode
 * The first row of the matrix represent the projection of the earth X axis
 * on the X, Y, and Z axes of the aircraft.
 *
 * The second row of the matrix represent the projection of the earth Y axis
 * on the axes of the aircraft.
 *
 * The third row of the matrix represent the projection of the earth Z axis
 * on the axes of the aircraft.
 *
 * Meaning of f_dcm_matrix elements:
 *
 * \code
 *          col    0                1                 2
 *      row
 *
 *       0   cos(Xp ^ Xe)    cos(Yp ^ Xe)     cos(Zp ^ Xe)
 *
 *       1   cos(Xp ^ Ye)    cos(Yp ^ Ye)     cos(Zp ^ Ye)
 *
 *       2   cos(Xp ^ Ze)    cos(Yp ^ Ze)     cos(Zp ^ Ze)
 *
 * \endcode
 *
 * where cos(Xp ^ Xe) is the cosine of the angle between plane X axis and
 * earth X axis, cos(Yp ^ Xe) is the cosine of the angle between plane Y
 * axis and earth X axis, and so on.
 *
 * Following cosines are mostly relevant:
 *
 * DCM[2][0] = cosine of the angle between aircraft X axis and earth Z axis.
 * It is equal to zero when the aircraft X axis is level with earth XY plane
 * i.e. the aircraft is longitudinally levelled, pitch angle = 0.
 *
 * DCM[2][1] = cosine of the angle between aircraft Y axis and earth Z axis.
 * It is equal to zero when the aircraft Y axis is level with earth XY plane
 * i.e. the aircraft is laterally levelled, roll angle = 0.
 *
 * DCM[1][1] = cosine of the angle between aircraft Y axis and earth Y axis.
 * It is equal to one when aircraft Y axis is aligned with earth Y axis.
 *
 * DCM is the identity matrix when the aircraft is sitting level on the ground
 * facing north.
 *
 * Meaning of f_gyro_vector elements:
 *
 * \code
 *
 *     element          meaning     UDB variable
 *
 *     f_gyro_vector[0]   roll rate   omegagyro[1]
 *     f_gyro_vector[1]   pitch rate  omegagyro[0]
 *     f_gyro_vector[2]   yaw rate    omegagyro[2]
 *
 * \endcode
 *
 * Change
 *
 *=============================================================================+*/

#include "ch.h"

#include "config.h"
#include "arm_math.h"
#include "vmath.h"
#include "gps.h"
#include "DCM.h"

/*--------------------------------- Definitions ------------------------------*/

/* Initial P gain for roll/pitch compensation */
/* Typical values 0.1f, 0.015f, 0.01f, 0.0013f */
#define PITCHROLL_KP    0.1f

/* Initial I gain for roll/pitch compensation*/
/* Typical values 0.000005f, 0.000002f */
#define PITCHROLL_KI    0.000005f

/* Initial P gain for yaw compensation */
/* Typical values 0.5f, 0.27f */
#define YAW_KP          0.5f

/* Initial P gain for yaw compensation */
/* Typical values 0.0005f */
#define YAW_KI          0.0005f

#define NULL_VECTOR { 0.0f, 0.0f, 0.0f }

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

const float32_t diag_mat[9] =
{
   1.0f, 0.0f, 0.0f,
   0.0f, 1.0f, 0.0f,
   0.0f, 0.0f, 1.0f
};

const float32_t null_mat[9] =
{
   0.0f, 0.0f, 0.0f,
   0.0f, 0.0f, 0.0f,
   0.0f, 0.0f, 0.0f
};

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

/* Direction Cosine Matrix*/
static arm_matrix_instance_f32 f_dcm_matrix = {3, 3, (float *)diag_mat};

/* Gyros here*/
static arm_matrix_instance_f32 f_upd_matrix = {3, 3, (float *)diag_mat};

/* Temporary matrix*/
static arm_matrix_instance_f32 f_temp_matrix = {3, 3, (float *)null_mat};

/* Raw gyroscope data*/
static float32_t f_gyro_vector[3] = NULL_VECTOR;

/* g-corrected gyroscope data*/
static float32_t f_omega_vector[3] = NULL_VECTOR;

/* Acceleration vector*/
static float32_t f_accel_vector[3] = NULL_VECTOR;

/* Temporary for intermediate calculation*/
static float32_t f_omega[3] = NULL_VECTOR;

/* f_omega proportional correction*/
static float32_t f_omega_p[3] = NULL_VECTOR;

/* f_omega integral correction*/
static float32_t f_omega_i[3] = NULL_VECTOR;

static float32_t f_scaled_omega_p[3] = NULL_VECTOR;

static float32_t f_scaled_omega_i[3] = NULL_VECTOR;

/* roll/pitch error vector*/
static float32_t f_error_rollpitch[3] = NULL_VECTOR;

/* yaw error vector*/
static float32_t f_error_yaw[3] = NULL_VECTOR;

/* Conversion gain from ADC to angular speed in deg/s*/
static float32_t f_gyro_gain = GYRO_GAIN;

/* Conversion gain from ADC to acceleration in m/s/s*/
static float32_t f_accel_gain = ACCEL_GAIN;

/* Proportional gain roll/pitch compensation*/
static float32_t f_pitchroll_Kp = PITCHROLL_KP;

/* Integral gain roll/pitch compensation*/
static float32_t f_pitchroll_Ki = PITCHROLL_KI;

/* Proportional gain yaw compensation*/
static float32_t f_yaw_Kp = YAW_KP;

/* Integral gain yaw compensation*/
static float32_t f_yaw_Ki = YAW_KI;

/* Velocity 3D*/
static float32_t f_ground_speed = 0.0f;

/* course error in deg*/
static float32_t f_error_course = 180.0f;

/* Course overground X axis*/
static float32_t f_cog_x = 1.0f;

/* Course overground Y axis*/
static float32_t f_cog_y = 0.0f;

/* aircraft pitch */
static float32_t f_pitch = 0.0f;

/* aircraft roll */
static float32_t f_roll = 0.0f;

/* aircraft yaw */
static float32_t f_yaw = 0.0f;

/* semaphore */
static Semaphore sem_dcm;

static float32_t f_temp[9];

/*--------------------------------- Prototypes -------------------------------*/

__inline static void accel_adjust(void);

/*---------------------------------- Functions -------------------------------*/

/*----------------------------------------------------------------------------
 *
 * @brief   Initialize DCM
 * @return  -
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
void DCM_Init ( void ) {

  /* Semaphore */
  chSemInit(&sem_dcm, 1);
}

/*----------------------------------------------------------------------------
 *
 * @brief   Normalize DCM matrix
 * @return  -
 * @remarks -
 * @todo    verificare correttezza indici degli arrai .pData :
 *          [0][0] = [0], [1][0] = [3], [2][0] = [6], [2][1] = [7]
 * @todo    verificare ordine degli operandi nel prodotto esterno
 *
 *----------------------------------------------------------------------------*/
void Normalize(void) {

    float32_t error = 0.0f;
    float32_t renorm = 0.0f;
    /*
                          error                         error
       Xorthogonal = X - ------- Y , Yorthogonal = Y - ------- X        Eq. 19
                            2                             2
    */
    arm_dot_prod_f32(f_dcm_matrix.pData, &f_dcm_matrix.pData[3], 3, &error);
    error = -error * 0.5f;
    arm_scale_f32(&f_dcm_matrix.pData[3], error, &f_temp_matrix.pData[0], 3);
    arm_scale_f32(&f_dcm_matrix.pData[0], error, &f_temp_matrix.pData[3], 3);
    arm_add_f32(&f_temp_matrix.pData[0], &f_dcm_matrix.pData[0], &f_temp_matrix.pData[0], 3);
    arm_add_f32(&f_temp_matrix.pData[3], &f_dcm_matrix.pData[3], &f_temp_matrix.pData[3], 3);

    /*
       Zorthogonal = X orthogonal /\ Yorthogonal                        Eq. 20
    */
    arm_mult_f32(&f_temp_matrix.pData[0], &f_temp_matrix.pData[3], &f_temp_matrix.pData[6], 3);

    /*
                    1
      Xnormalized = - (3 - Xorthogonal . Xorthogonal) Xorthogonal
                    2

                    1
      Yormalized =  - (3 - Yorthogonal . Yorthogonal) Yorthogonal      Eq. 21
                    2

                   1
      Znormalized = - (3 - Zorthogonal . Zorthogonal) Zorthogonal
                    2
    */
    arm_dot_prod_f32(&f_temp_matrix.pData[0], &f_temp_matrix.pData[0], 3, &renorm);
    renorm = 0.5f * (3.0f - renorm);
    arm_scale_f32(&f_dcm_matrix.pData[0], renorm, &f_temp_matrix.pData[0], 3);


    arm_dot_prod_f32(&f_temp_matrix.pData[3], &f_temp_matrix.pData[3], 3, &renorm);
    renorm = 0.5f * (3.0f - renorm);
    arm_scale_f32(&f_dcm_matrix.pData[3], renorm, &f_temp_matrix.pData[3], 3);

    arm_dot_prod_f32(&f_temp_matrix.pData[6], &f_temp_matrix.pData[6], 3, &renorm);
    renorm = 0.5f * (3.0f - renorm);
    arm_scale_f32(&f_dcm_matrix.pData[6], renorm, &f_temp_matrix.pData[6], 3);


    /* update pitch, roll, yaw */
    chSemWait(&sem_dcm);
    f_pitch = -asinf(f_dcm_matrix.pData[6]);    /* [2][0]) */
    f_roll = asinf(f_dcm_matrix.pData[7]);      /* [2][1]) */
    f_yaw = atan2f(f_dcm_matrix.pData[3], f_dcm_matrix.pData[0]); /* [1][0] - [0][0] */
    chSemSignal(&sem_dcm);
}


/*----------------------------------------------------------------------------
 *
 * @brief   Adjust acceleration
 * @return  -
 * @remarks Computes reference acceleration as
 *                                                                 \code
 * g         = Accelerometer + A                 (Eq. 26)
 *  reference                   centrifugal                        \endcode
 *
 * where
 *                                                                 \code
 * A           = omega     /\ V                  (Eq. 25)
 *  centrifugal       gyro                                         \endcode
 *
 * and the velocity vector has only the x component
 *                                                                 \code
 *     | velocity |
 *     |          |
 * V = |    0     |
 *     |          |
 *     |    0     |                                                \endcode
 *
 * The acceleration is then scaled from ADC values to g.
 *
 *----------------------------------------------------------------------------*/
__inline static void accel_adjust(void)
{
#if (SIMULATOR == SIM_NONE)
    f_ground_speed = (float32_t)Gps_Speed_Kt();
    f_ground_speed = (f_ground_speed * 1852.0f) / 36000.0f; /* convert [kt] to [m/s] */
#else
    f_ground_speed = Simulator_Get_Speed();
#endif
    f_accel_vector[1] += ((f_ground_speed * f_omega[2] * 9.81f) / GRAVITY);
    f_accel_vector[2] -= ((f_ground_speed * f_omega[1] * 9.81f) / GRAVITY);
}


/*----------------------------------------------------------------------------
 *
 * Compensate for roll / pitch / yaw drift
 * @return  -
 * @remarks La correzione di rollio e beccheggio e' data da: \code
 *                        | Rzx |
 *                        |     |
 * RollPitch correction = | Rzy | /\ g
 *                        |     |     reference
 *                        | Rzz |
 * \endcode
 *
 * La correzione di imbardata e' calcolata prima nel sistema di riferimento di
 * terra: \code
 * Yaw correction (ground) = xb  /\ COG
 *                             p
 * \endcode dove \code
 *
 * xb
 *   p
 *
 * \endcode e' la proiezione dell'asse X dell'aereo sul piano X Y del sistema
 * di riferimento di terra, e \code COG \endcode (Course Over Ground) e' la
 * proiezione della prua dell'aereo sul piano X Y del sistema di riferimento
 * di terra. La correzione di imbardata viene trasformata nel sistema di
 * riferimento dell'aereo:
 * \code
 *                                                       | Rzx |
 *                                                       |     |
 * Yaw correction (aircraft) = Yaw correction (ground) . | Rzy |
 *                                                       |     |
 *                                                       | Rzz |
 * \endcode
 *
 *----------------------------------------------------------------------------*/
void CompensateDrift( void )
{
    float32_t fCourse_Over_Ground;

    /* RollPitch correction */
    arm_mult_f32(f_accel_vector, &f_dcm_matrix.pData[6], f_error_rollpitch, 3);

    arm_scale_f32(f_error_rollpitch, f_pitchroll_Kp, f_omega_p, 3);
    arm_scale_f32(f_error_rollpitch, f_pitchroll_Ki, f_scaled_omega_i, 3);
    arm_add_f32(f_omega_i, f_scaled_omega_i, f_omega_i, 3);

    /* Course over ground */
    fCourse_Over_Ground = (float32_t)Gps_COG_Deg();
    f_cog_x = arm_cos_f32(DEGTORAD(fCourse_Over_Ground));
    f_cog_y = arm_sin_f32(DEGTORAD(fCourse_Over_Ground));

    /* Yaw correction (ground) */
    f_error_course = (f_dcm_matrix.pData[0] * f_cog_y) - (f_dcm_matrix.pData[3] * f_cog_x);

    /* Yaw correction (aircraft) */
    arm_scale_f32(&f_dcm_matrix.pData[6], f_error_course, f_error_yaw, 3);

    /* YAW proportional gain. */
    arm_scale_f32(f_error_yaw, f_yaw_Kp, f_scaled_omega_p, 3);

    /* Adding proportional. */
    arm_add_f32(f_omega_p, f_scaled_omega_p, f_omega_p, 3);

    /* YAW integral gain. */
    arm_scale_f32(f_error_yaw, f_yaw_Ki, f_scaled_omega_i, 3);

    /* Adding integral to the f_omega_i */
    arm_add_f32(f_omega_i, f_scaled_omega_i, f_omega_i, 3);
}


/*----------------------------------------------------------------------------
 *
 * @brief   Update DCM matrix
 * @return  -
 * @remarks
 *
 *----------------------------------------------------------------------------*/
void MatrixUpdate(const int16_t *sensor)
{
    /* Accelerometer signals */
    f_accel_vector[0] = f_accel_gain * (*sensor++);   /* accel x */
    f_accel_vector[1] = f_accel_gain * (*sensor++);   /* accel y */
    f_accel_vector[2] = f_accel_gain * (*sensor++);   /* accel z */

    /* Gyro signals */
    f_gyro_vector[0] = f_gyro_gain * (*sensor++);     /* omega x */
    f_gyro_vector[1] = f_gyro_gain * (*sensor++);     /* omega y */
    f_gyro_vector[2] = f_gyro_gain * (*sensor);       /* omega z */

    /* adding integral */
    arm_add_f32(f_gyro_vector, f_omega_i, f_omega, 3);

    /* adding proportional */
    arm_add_f32(f_omega, f_omega_p, f_omega_vector, 3);

    /* adjust centrifugal acceleration. */
    accel_adjust();

    f_temp[0] = 0.0f;
    f_temp[1] = -DELTA_T * f_omega_vector[2];
    f_temp[2] =  DELTA_T * f_omega_vector[1];
    f_temp[3] =  DELTA_T * f_omega_vector[2];
    f_temp[4] = 0.0f;
    f_temp[5] = -DELTA_T * f_omega_vector[0];
    f_temp[6] = -DELTA_T * f_omega_vector[1];
    f_temp[7] =  DELTA_T * f_omega_vector[0];
    f_temp[8] = 0.0f;
    arm_mat_init_f32(&f_upd_matrix, 3, 3, f_temp);

    /* Update DCM matrix */
    arm_mat_mult_f32(&f_dcm_matrix, &f_upd_matrix, &f_temp_matrix);
    arm_mat_add_f32(&f_dcm_matrix, &f_temp_matrix, &f_dcm_matrix);
}


/*----------------------------------------------------------------------------
 *
 * @brief   Aircraft pitch.
 * @return  aircraft pitch angle [rad]
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
float32_t AHRS_Pitch_Rad(void)
{
  float32_t f_temp;

  chSemWait(&sem_dcm);
  f_temp = f_pitch;
  chSemSignal(&sem_dcm);

  return (f_temp);
}

/*----------------------------------------------------------------------------
 *
 * @brief   Aircraft roll.
 * @return  aircraft roll angle [rad]
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
float32_t AHRS_Roll_Rad(void)
{
  float32_t f_temp;

  chSemWait(&sem_dcm);
  f_temp = f_roll;
  chSemSignal(&sem_dcm);

  return (f_temp);
}

/*----------------------------------------------------------------------------
 *
 * @brief   Aircraft yaw.
 * @return  aircraft roll angle [rad]
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
float32_t AHRS_Yaw_Rad(void)
{
  float32_t f_temp;

  chSemWait(&sem_dcm);
  f_temp = f_yaw;
  chSemSignal(&sem_dcm);

  return (f_temp);
}

