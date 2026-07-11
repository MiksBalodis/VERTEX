#ifndef TVC_CONTROL_H
#define TVC_CONTROL_H

#include "servo.h"
#include <stdbool.h>
#include <stdint.h>

/* =========================================================================
   TVC_CONTROL — Two-axis Thrust Vector Control for a 2-servo rocket
   =========================================================================

   Coordinate frame (matches imu_fusion.h):
     Y  – rocket long axis, pointing up
     X  – rocket right
     Z  – rocket front

   Axes controlled:
     PITCH  – tilt in the X–Y plane  (rotation about X axis)
              Sensor signal: imu_integration.pitch  [deg]
              Angular rate:  imu.gx                 [milli-DPS]
              Actuator:      hservo1 (Servo1_Home = 86 deg center)

     YAW    – tilt in the Z–Y plane  (rotation about Z axis)
              Sensor signal: imu_integration.yaw    [deg]
              Angular rate:  imu.gz                 [milli-DPS]
              Actuator:      hservo2 (Servo2_Home = 75 deg center)

   ROLL (spin about Y) is not controllable with 2-servo TVC and is ignored.

   PID algorithm
   -------------
   Each axis runs a standard PID with:
     P term  –  proportional to attitude angle error (degrees)
     I term  –  integral of attitude angle error (degree-seconds)
                Clamped independently to prevent integrator wind-up.
     D term  –  proportional to angular rate measured directly by gyro
                (milli-DPS). Using the raw rate instead of differentiating
                the angle gives much lower noise on the derivative.

   Output → servo deflection (degrees) added to the mechanical center:
     servo_angle = home_angle + pid_output
     pid_output is clamped to ±TVC_MAX_DEFLECTION_DEG before applying
     so the nozzle cannot over-travel regardless of gains.

   Typical usage in Mission_Update (ASCENT case):
     TVC_Update(&tvc, imu_integration.pitch, imu.gx,
                       imu_integration.yaw,   imu.gz,
                       dt_s);
   ========================================================================= */

/* ------------------------------------------------------------------
   Mechanical limits.
   Adjust TVC_MAX_DEFLECTION_DEG to match your gimbal's physical travel.
   A typical TVC mount allows ±8–12 degrees.
   ------------------------------------------------------------------ */
#define TVC_MAX_DEFLECTION_DEG   10.0f   /* ± degrees from servo center  */

/* ------------------------------------------------------------------
   Single-axis PID state
   ------------------------------------------------------------------ */
typedef struct
{
    /* Gains (set at init, can be changed at runtime) */
    float Kp;          /* Proportional gain  [deg output / deg error]       */
    float Ki;          /* Integral gain      [deg output / (deg·s)]         */
    float Kd;          /* Derivative gain    [deg output / (milli-DPS)]     */

    /* Integrator state */
    float integrator;  /* Running integral of error [deg·s]                 */
    float i_limit;     /* Symmetric clamp on integrator  [deg·s]            */

    /* Output */
    float output;      /* Last computed deflection [deg], clamped           */

} TVC_PID_t;

/* ------------------------------------------------------------------
   Full TVC controller state
   ------------------------------------------------------------------ */
typedef struct
{
    TVC_PID_t pitch;   /* Pitch-axis PID                                    */
    TVC_PID_t yaw;     /* Yaw-axis PID                                      */

    servo_t  *pitch_servo;   /* Pointer to pitch servo handle               */
    servo_t  *yaw_servo;     /* Pointer to yaw servo handle                 */

    float     pitch_home;    /* Servo center angle for pitch [deg]          */
    float     yaw_home;      /* Servo center angle for yaw   [deg]          */

    bool      enabled;       /* false → servos are parked at home           */
} TVC_t;

/* ------------------------------------------------------------------
   Public API
   ------------------------------------------------------------------ */

/*
 * TVC_Init – configure the controller and set servo handle pointers.
 *
 * Parameters
 * ----------
 * tvc          – controller instance to initialise
 * pitch_servo  – pointer to hservo1 (pitch)
 * yaw_servo    – pointer to hservo2 (yaw)
 * pitch_home   – servo center angle for pitch (Servo1_Home = 86)
 * yaw_home     – servo center angle for yaw   (Servo2_Home = 75)
 * Kp, Ki, Kd   – initial PID gains applied to BOTH axes.
 *                Use TVC_SetGains() afterwards if you need per-axis tuning.
 * i_limit      – symmetric clamp on each integrator [deg·s].
 *                Prevents wind-up during long disturbances.
 *                A value of  Kp * TVC_MAX_DEFLECTION_DEG / Ki  is a
 *                reasonable starting point (limits I contribution to the
 *                full deflection authority).
 */
void TVC_Init(TVC_t    *tvc,
              servo_t  *pitch_servo,
              servo_t  *yaw_servo,
              float     pitch_home,
              float     yaw_home,
              float     Kp,
              float     Ki,
              float     Kd,
              float     i_limit);

/*
 * TVC_SetGains – update gains on one axis at runtime.
 *
 * axis – 0 = pitch, 1 = yaw
 */
void TVC_SetGains(TVC_t *tvc, uint8_t axis, float Kp, float Ki, float Kd);

/*
 * TVC_Enable / TVC_Disable
 *
 * While disabled, TVC_Update() parks both servos at their home angles
 * and holds the integrators at zero.  Call TVC_Enable() only during
 * MISSION_ASCENT (or while the motor is burning).
 */
void TVC_Enable (TVC_t *tvc);
void TVC_Disable(TVC_t *tvc);

/*
 * TVC_Reset – zero integrators and reset output without disabling.
 * Call this at the moment of launch detection so the I term starts
 * clean with zero initial angle error.
 */
void TVC_Reset(TVC_t *tvc);

/*
 * TVC_Update – run one PID step and command both servos.
 *
 * Parameters
 * ----------
 * tvc            – controller instance
 * pitch_angle    – current pitch angle  (degrees, from imu_integration.pitch)
 * pitch_rate_mdps – current pitch rate  (milli-DPS, from imu.gx)
 * yaw_angle      – current yaw angle    (degrees, from imu_integration.yaw)
 * yaw_rate_mdps  – current yaw rate     (milli-DPS, from imu.gz)
 * dt_s           – time since last call (seconds).
 *                  Clamped to [0.0001, 0.1] internally to guard against
 *                  a stale dt causing an integrator spike.
 *
 * Call rate: every Mission_Update cycle (~50 Hz with BMP388 at 50 Hz).
 * The D term uses the gyro rate directly so it does not depend on dt.
 */
/* roll_angle_deg: rotation about the rocket's LONG axis, used to de-rotate the
   pitch/yaw commands into the (body-fixed) gimbal frame. Without it, once the
   rocket rolls 90 deg the pitch servo starts correcting yaw and the controller
   fights itself. */
void TVC_Update(TVC_t *tvc,
                float  pitch_angle,
                float  pitch_rate_mdps,
                float  yaw_angle,
                float  yaw_rate_mdps,
                float  roll_angle_deg,
                float  dt_s);

/*
 * TVC_GetPitchOutput / TVC_GetYawOutput
 * Return the last servo deflection commanded [deg], before adding home.
 * Useful for telemetry.
 */
float TVC_GetPitchOutput(const TVC_t *tvc);
float TVC_GetYawOutput  (const TVC_t *tvc);

#endif /* TVC_CONTROL_H */
