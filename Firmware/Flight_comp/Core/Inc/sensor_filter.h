#ifndef SENSOR_FILTER_H
#define SENSOR_FILTER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    float value;
    float alpha;
    bool initialized;
} LowPassFilter_t;

typedef struct
{
    float lat;
    float lon;
    bool initialized;
} GpsFilter_t;

typedef struct
{
    float q;
    float r;
    float p;
    float x;
    bool initialized;
} KalmanFilter1D_t;

typedef struct
{

    float alt;
    float vel;

    float p00;
    float p01;
    float p10;
    float p11;

    float q_pos;
    float q_vel;
    float r_alt;

    bool initialized;
} KalmanAltVel_t;

#define VERT_KF_GPS_MIN_SATELLITES   4
#define VERT_KF_GPS_MAX_HDOP         5.0f

typedef struct
{

    float alt;
    float vel;

    float p00;
    float p01;
    float p10;
    float p11;

    float q_alt;
    float q_vel;
    float r_baro;
    float r_gps;

    float ground_alt_msl;
    bool  ground_alt_valid;

    bool initialized;
} VertKF_t;

void  LowPass_Init   (LowPassFilter_t *filter, float alpha);
void  LowPass_Reset  (LowPassFilter_t *filter);
float LowPass_Update (LowPassFilter_t *filter, float input);

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

void  Kalman1D_Init   (KalmanFilter1D_t *kf, float q, float r, float initial_x);
float Kalman1D_Update (KalmanFilter1D_t *kf, float measurement);

void  KalmanAltVel_Init   (KalmanAltVel_t *kf,
                            float q_pos, float q_vel, float r_alt);
void  KalmanAltVel_Update (KalmanAltVel_t *kf,
                            float baro_altitude_m, float dt_s);

void VertKF_Init(VertKF_t *kf,
                 float q_alt,
                 float q_vel,
                 float r_baro,
                 float r_gps);

void VertKF_Reset(VertKF_t *kf, float baro_alt_agl);

void VertKF_UpdateGroundGpsAlt(VertKF_t *kf, float gps_alt_msl);

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

#endif
