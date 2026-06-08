
#ifndef SENSOR_FILTER_H
#define SENSOR_FILTER_H

#include <stdint.h>
#include <stdbool.h>

/* -----------------------------------------------------------------------
   Low-pass (exponential moving average) filter
   ----------------------------------------------------------------------- */
typedef struct
{
    float value;
    float alpha;
    bool initialized;
} LowPassFilter_t;

/* -----------------------------------------------------------------------
   GPS position filter
   ----------------------------------------------------------------------- */
typedef struct
{
    float lat;
    float lon;
    bool initialized;
} GpsFilter_t;

/* -----------------------------------------------------------------------
   1-D Kalman filter for a scalar state (e.g. altitude or vertical speed).

   State vector:   x  = [ position ]   (or whichever scalar you track)
   Process model:  x_k = x_{k-1} + w_k,   w_k ~ N(0, q)
   Measurement:    z_k = x_k + v_k,        v_k ~ N(0, r)

   q  – process noise variance  (larger → filter trusts measurements more)
   r  – measurement noise variance (larger → filter trusts model more)
   p  – estimation error covariance (updated each step)
   x  – current state estimate
   ----------------------------------------------------------------------- */
typedef struct
{
    float q;            /* Process noise variance                          */
    float r;            /* Measurement noise variance                      */
    float p;            /* Estimation error covariance                     */
    float x;            /* Current state estimate                          */
    bool initialized;
} KalmanFilter1D_t;

/* -----------------------------------------------------------------------
   2-state (position + velocity) Kalman filter.

   State:   [ altitude, vertical_speed ]
   Process: altitude_{k} = altitude_{k-1} + vel_{k-1}*dt
            vel_{k}       = vel_{k-1}
   Measurement: altitude only (from barometer).

   q_pos – process noise for position
   q_vel – process noise for velocity
   r     – measurement noise for barometric altitude
   ----------------------------------------------------------------------- */
typedef struct
{
    /* State */
    float alt;          /* Estimated altitude  (m)                         */
    float vel;          /* Estimated vertical speed (m/s)                  */

    /* Covariance matrix  P = [[p00, p01], [p10, p11]] */
    float p00;
    float p01;
    float p10;
    float p11;

    /* Noise parameters */
    float q_pos;        /* Process noise – altitude                        */
    float q_vel;        /* Process noise – velocity                        */
    float r_alt;        /* Measurement noise – barometric altitude         */

    bool initialized;
} KalmanAltVel_t;

/* -----------------------------------------------------------------------
   Multi-sensor vertical Kalman filter  (VertKF_t)
   -----------------------------------------------------------------------
   State vector:  s = [ alt_m,  vel_mps ]   (2 states)

   Process model  (IMU acceleration used as control input, not as state):
     alt_{k} = alt_{k-1}  +  vel_{k-1}*dt  +  0.5*a_net*dt²
     vel_{k} = vel_{k-1}  +  a_net*dt

   where a_net is the net world-vertical acceleration in m/s²,
   derived from the IMU and the current attitude quaternion:
     a_world_y_mg  =  R12*rx + R11*ry + R13*rz        (body→world, Y-up)
     a_net_mps2    =  (a_world_y_mg - 1000.0f) * 0.001f * 9.80665f

   Transition matrix F:
     [ 1  dt ]
     [ 0   1 ]

   Control-input matrix G (effect of a_net on state):
     [ 0.5*dt² ]
     [   dt    ]

   Process noise covariance Q  (set by caller via q_alt / q_vel):
     [ q_alt    0   ]
     [   0    q_vel ]

   Sensors fused (all scalar, H = [1, 0]):
     1. BMP388 barometric altitude (primary, every update, r_baro)
     2. GPS altitude AGL           (conditional, r_gps)

   GPS acceptance criteria (all must hold):
     gps.valid == true
     gps.fix   >= 1
     gps.sat   >= GPS_MIN_SATELLITES
     gps.hdop  <= GPS_MAX_HDOP
   ----------------------------------------------------------------------- */

#define VERT_KF_GPS_MIN_SATELLITES   4
#define VERT_KF_GPS_MAX_HDOP         5.0f

typedef struct
{
    /* ----- State estimate ----- */
    float alt;              /* Estimated AGL altitude   (m)                */
    float vel;              /* Estimated vertical speed (m/s)              */

    /* ----- Covariance matrix  P = [[p00, p01],[p10, p11]] ----- */
    float p00;
    float p01;
    float p10;
    float p11;

    /* ----- Tuning ----- */
    float q_alt;            /* Process noise – altitude   (m²)             */
    float q_vel;            /* Process noise – velocity   (m²/s²)          */
    float r_baro;           /* Baro measurement noise     (m²)             */
    float r_gps;            /* GPS measurement noise      (m²)             */

    /* ----- Ground reference ----- */
    float ground_alt_msl;   /* GPS MSL altitude on pad (m), for AGL conv   */
    bool  ground_alt_valid; /* true once gps.alt sampled on pad            */

    bool initialized;
} VertKF_t;

/* -----------------------------------------------------------------------
   Function prototypes – low-pass
   ----------------------------------------------------------------------- */
void  LowPass_Init   (LowPassFilter_t *filter, float alpha);
void  LowPass_Reset  (LowPassFilter_t *filter);
float LowPass_Update (LowPassFilter_t *filter, float input);

/* -----------------------------------------------------------------------
   Function prototypes – GPS position filter
   ----------------------------------------------------------------------- */
void GpsFilter_Init(GpsFilter_t *filter);
bool GpsFilter_Update(
    GpsFilter_t *filter,
    float input_lat,
    float input_lon,
    float alpha,
    float max_jump_meters,
    float *output_lat,
    float *output_lon
);

/* -----------------------------------------------------------------------
   Function prototypes – 1-D Kalman
   ----------------------------------------------------------------------- */
void  Kalman1D_Init   (KalmanFilter1D_t *kf, float q, float r, float initial_x);
float Kalman1D_Update (KalmanFilter1D_t *kf, float measurement);

/* -----------------------------------------------------------------------
   Function prototypes – 2-state altitude/velocity Kalman
   ----------------------------------------------------------------------- */
void  KalmanAltVel_Init   (KalmanAltVel_t *kf,
                            float q_pos, float q_vel, float r_alt);
void  KalmanAltVel_Update (KalmanAltVel_t *kf,
                            float baro_altitude_m, float dt_s);

/* -----------------------------------------------------------------------
   Function prototypes – multi-sensor vertical Kalman filter
   ----------------------------------------------------------------------- */

/*
 * VertKF_Init – initialise the filter and reset all state.
 *
 * q_alt  – altitude process noise variance (m²).
 *           Reflects uncertainty in IMU-driven altitude prediction.
 *           Recommended: 0.5 – 2.0
 * q_vel  – velocity process noise variance (m²/s²).
 *           Reflects how much vertical speed can change per step beyond
 *           what the IMU predicts.
 *           Recommended: 1.0 – 5.0
 * r_baro – BMP388 altitude measurement noise variance (m²).
 *           BMP388 RMS noise ≈ 0.15 m in still air, worse under vibration.
 *           Recommended: 0.5 – 2.0
 * r_gps  – GPS altitude measurement noise variance (m²).
 *           Consumer GPS altitude accuracy is ~3-10 m (1-sigma).
 *           Recommended: 25.0 (=5 m sigma)
 */
void VertKF_Init(VertKF_t *kf,
                 float q_alt,
                 float q_vel,
                 float r_baro,
                 float r_gps);

/*
 * VertKF_Reset – re-seed the filter state (call while on the pad to keep
 * the filter warm before launch).
 *
 * baro_alt_agl – current barometric AGL altitude (m), typically ≈ 0 on pad.
 */
void VertKF_Reset(VertKF_t *kf, float baro_alt_agl);

/*
 * VertKF_UpdateGroundGpsAlt – record the GPS MSL altitude while still on
 * the pad so we can convert gps.alt (MSL) to AGL during flight.
 * Call this repeatedly while mission_state == MISSION_READY.
 */
void VertKF_UpdateGroundGpsAlt(VertKF_t *kf, float gps_alt_msl);

/*
 * VertKF_Update – run one complete filter step.
 *
 * Parameters
 * ----------
 * baro_alt_agl – AGL altitude from BMP388 (m).  Must be valid every call.
 * imu_rx_mg    – IMU accelerometer rocket-right  axis (mg).
 * imu_ry_mg    – IMU accelerometer rocket-up     axis (mg).
 * imu_rz_mg    – IMU accelerometer rocket-front  axis (mg).
 * qw,qx,qy,qz – Current attitude quaternion (body→world, identity at launch).
 * gps          – Pointer to GPS struct.  May be NULL; if non-NULL and the fix
 *                quality criteria are met, the GPS altitude is fused as an
 *                independent measurement after the baro update.
 * dt_s         – Time since last call (seconds).  Clamped internally to
 *                [0.001, 0.5] to guard against timer glitches.
 *
 * Outputs are stored in kf->alt and kf->vel.
 */

/* gngga_t is defined in gps.h — include it here so this header is
   self-contained.  gps.h uses #pragma once so multiple inclusion is safe. */
#include "gps.h"

void VertKF_Update(VertKF_t   *kf,
                   float       baro_alt_agl,
                   float       imu_rx_mg,
                   float       imu_ry_mg,
                   float       imu_rz_mg,
                   float       qw,
                   float       qx,
                   float       qy,
                   float       qz,
                   const gngga_t *gps,
                   float       dt_s);

#endif /* SENSOR_FILTER_H */
