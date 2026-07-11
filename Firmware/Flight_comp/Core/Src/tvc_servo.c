#include "tvc_servo.h"
#include <stddef.h>   /* NULL */
#include <math.h>     /* isfinite, sinf, cosf */

/* =========================================================================
   Internal helpers
   ========================================================================= */

/*
 * clampf – symmetric float clamp.
 */
static float clampf(float value, float limit)
{
    if (value >  limit) return  limit;
    if (value < -limit) return -limit;
    return value;
}

/*
 * pid_init – initialise a single-axis PID.
 */
static void pid_init(TVC_PID_t *pid, float Kp, float Ki, float Kd, float i_limit)
{
    pid->Kp          = Kp;
    pid->Ki          = Ki;
    pid->Kd          = Kd;
    pid->integrator  = 0.0f;
    pid->i_limit     = i_limit;
    pid->output      = 0.0f;
}

/*
 * pid_reset – zero integrator and output without touching gains.
 */
static void pid_reset(TVC_PID_t *pid)
{
    pid->integrator = 0.0f;
    pid->output     = 0.0f;
}

/*
 * pid_step – compute one PID step.
 *
 * error   – angle error in degrees  (setpoint is 0 = vertical)
 * rate    – angular rate in milli-DPS (positive = tilting in positive direction)
 * dt_s    – timestep in seconds (already clamped by TVC_Update caller)
 *
 * Returns the signed deflection command in degrees.
 *
 * Sign convention:
 *   Positive angle error (rocket tilted forward/right) →
 *   Positive output → servo deflects nozzle to correct back toward vertical.
 *   The calling code adds this to the home angle; a positive output means
 *   "deflect the nozzle in the direction that produces a corrective torque."
 *   Physical verification of sign is done during benchtop testing by
 *   confirming the nozzle deflects in the corrective direction.
 *
 * D term:
 *   Kd * rate_mdps
 *   We use the measured angular rate directly instead of differentiating
 *   the angle. This is equivalent but avoids amplifying angle measurement
 *   noise. The milli-DPS unit is absorbed by Kd (a typical Kd will be on
 *   the order of 0.001–0.01 for milli-DPS input).
 */
static float pid_step(TVC_PID_t *pid, float error, float rate_mdps, float dt_s)
{
    /* --- P term --- */
    float p_term = pid->Kp * error;

    /* --- I term: integrate error, then clamp --- */
    pid->integrator += error * dt_s;
    pid->integrator  = clampf(pid->integrator, pid->i_limit);
    float i_term     = pid->Ki * pid->integrator;

    /* --- D term: rate feedback directly from gyro ---
       NOTE: the caller passes the NEGATED rate, because d(error)/dt = -rate
       (the setpoint is a constant 0). The old code passed +rate, which made
       the D term positive feedback on angular rate -- it accelerated the
       divergence instead of damping it. */
    float d_term = pid->Kd * rate_mdps;

    /* --- Sum and clamp to mechanical travel limit --- */
    float raw_output = p_term + i_term + d_term;
    float clamped    = clampf(raw_output, TVC_MAX_DEFLECTION_DEG);

    pid->output = clamped;
    return clamped;
}

/* =========================================================================
   Public API
   ========================================================================= */

void TVC_Init(TVC_t    *tvc,
              servo_t  *pitch_servo,
              servo_t  *yaw_servo,
              float     pitch_home,
              float     yaw_home,
              float     Kp,
              float     Ki,
              float     Kd,
              float     i_limit)
{
    if (tvc == NULL) return;

    tvc->pitch_servo = pitch_servo;
    tvc->yaw_servo   = yaw_servo;
    tvc->pitch_home  = pitch_home;
    tvc->yaw_home    = yaw_home;
    tvc->enabled     = false;

    pid_init(&tvc->pitch, Kp, Ki, Kd, i_limit);
    pid_init(&tvc->yaw,   Kp, Ki, Kd, i_limit);
}

void TVC_SetGains(TVC_t *tvc, uint8_t axis, float Kp, float Ki, float Kd)
{
    if (tvc == NULL) return;

    TVC_PID_t *pid = (axis == 0) ? &tvc->pitch : &tvc->yaw;

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    /* Integrator and i_limit are intentionally not touched here so that
       live gain changes during a test don't reset accumulated state. */
}

void TVC_Enable(TVC_t *tvc)
{
    if (tvc == NULL) return;
    tvc->enabled = true;
}

void TVC_Disable(TVC_t *tvc)
{
    if (tvc == NULL) return;

    tvc->enabled = false;

    /* Park servos at home */
    if (tvc->pitch_servo != NULL)
        Servo_SetAngleFine(tvc->pitch_servo, tvc->pitch_home);

    if (tvc->yaw_servo != NULL)
        Servo_SetAngleFine(tvc->yaw_servo, tvc->yaw_home);
}

void TVC_Reset(TVC_t *tvc)
{
    if (tvc == NULL) return;
    pid_reset(&tvc->pitch);
    pid_reset(&tvc->yaw);
}

void TVC_Update(TVC_t *tvc,
                float  pitch_angle,
                float  pitch_rate_mdps,
                float  yaw_angle,
                float  yaw_rate_mdps,
                float  roll_angle_deg,
                float  dt_s)
{
    if (tvc == NULL) return;

    /* --- Guard dt --- */
    if (dt_s < 0.0001f) dt_s = 0.0001f;
    if (dt_s > 0.1f)    dt_s = 0.1f;

    if (!tvc->enabled)
    {
        /* Keep servos at home, reset integrators */
        pid_reset(&tvc->pitch);
        pid_reset(&tvc->yaw);

        if (tvc->pitch_servo != NULL)
            Servo_SetAngleFine(tvc->pitch_servo, tvc->pitch_home);

        if (tvc->yaw_servo != NULL)
            Servo_SetAngleFine(tvc->yaw_servo, tvc->yaw_home);

        return;
    }

    /* ------------------------------------------------------------------
       Run PID for each axis.

       The setpoint is 0 degrees (rocket pointing straight up).
       A positive angle means the rocket has tilted in the positive
       direction, so the error fed to the PID is simply the measured angle
       (with a negated sign so the output opposes the tilt — see note below).

       Sign note:
         If the rocket tilts forward (positive pitch angle), the nozzle
         must deflect backward (negative direction in gimbal terms) to
         push the base of the rocket back.  Whether positive PID output
         corresponds to corrective or wrong-direction deflection depends on
         the gimbal wiring and physical mounting.
         The sign of Kp therefore has physical meaning: if the rocket
         oscillates or diverges, flip the sign of all three gains on that
         axis (or equivalently negate the angle and rate inputs).
         Here we use:   error = -angle
         so that positive tilt → negative error → negative output → the
         servo moves in whatever direction reduces the tilt.
         Verify on the bench: tilt the rocket forward and confirm the
         nozzle deflects to push it back.
       ------------------------------------------------------------------ */
    float pitch_error = -pitch_angle;
    float yaw_error   = -yaw_angle;

    /* d(error)/dt = -rate. Passing +rate here was a sign bug: with any
       Kd > 0 it made the D term REINFORCE the tumble. */
    float pitch_out = pid_step(&tvc->pitch, pitch_error, -pitch_rate_mdps, dt_s);
    float yaw_out   = pid_step(&tvc->yaw,   yaw_error,   -yaw_rate_mdps,   dt_s);

    /* ------------------------------------------------------------------
       Convert PID output to servo angle and command actuators.

       servo_angle = home + pid_output

       pid_output is already clamped to ±TVC_MAX_DEFLECTION_DEG inside
       pid_step, so servo_angle stays within [home - limit, home + limit].
       Both are within the 0–180 deg SG90 range for the home angles used
       (86 and 75 deg), so no second clamp is needed here.
       ------------------------------------------------------------------ */
    /* ------------------------------------------------------------------
       Roll compensation.

       The gimbal axes are body-fixed and the rocket WILL roll (thrust
       misalignment, fin cant). The pitch/yaw errors above are integrated
       from body rates, so once roll is non-zero the 'pitch' servo is no
       longer aligned with the pitch error. Rotate the command back into
       the gimbal frame by the roll angle.
       ------------------------------------------------------------------ */
    float roll_rad = roll_angle_deg * 0.01745329252f;
    float cr = cosf(roll_rad);
    float sr = sinf(roll_rad);

    float cmd_pitch = clampf( pitch_out * cr + yaw_out * sr, TVC_MAX_DEFLECTION_DEG);
    float cmd_yaw   = clampf(-pitch_out * sr + yaw_out * cr, TVC_MAX_DEFLECTION_DEG);

    if (tvc->pitch_servo != NULL)
        Servo_SetAngleFine(tvc->pitch_servo, tvc->pitch_home + cmd_pitch);

    if (tvc->yaw_servo != NULL)
        Servo_SetAngleFine(tvc->yaw_servo, tvc->yaw_home + cmd_yaw);
}

float TVC_GetPitchOutput(const TVC_t *tvc)
{
    if (tvc == NULL) return 0.0f;
    return tvc->pitch.output;
}

float TVC_GetYawOutput(const TVC_t *tvc)
{
    if (tvc == NULL) return 0.0f;
    return tvc->yaw.output;
}
