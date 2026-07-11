#include "sensor_filter.h"

#include <math.h>
#include <stddef.h>

void LowPass_Init(LowPassFilter_t *filter, float alpha)
{
    if (filter == NULL) return;

    filter->value       = 0.0f;
    filter->alpha       = alpha;
    filter->initialized = false;
}

void LowPass_Reset(LowPassFilter_t *filter)
{
    if (filter == NULL) return;

    filter->value       = 0.0f;
    filter->initialized = false;
}

float LowPass_Update(LowPassFilter_t *filter, float input)
{
    if (filter == NULL) return input;

    if (!filter->initialized)
    {
        filter->value       = input;
        filter->initialized = true;
        return filter->value;
    }

    filter->value = filter->value + filter->alpha * (input - filter->value);
    return filter->value;
}

void GpsFilter_Init(GpsFilter_t *filter)
{
    if (filter == NULL) return;

    filter->lat         = 0.0f;
    filter->lon         = 0.0f;
    filter->initialized = false;
}

static bool gps_latlon_valid(float lat, float lon)
{
    return (
        isfinite(lat) &&
        isfinite(lon) &&
        lat >= -90.0f  && lat <= 90.0f &&
        lon >= -180.0f && lon <= 180.0f &&
        !(lat == 0.0f && lon == 0.0f)
    );
}

static float gps_distance_m(float lat1, float lon1, float lat2, float lon2)
{
    const float meters_per_deg_lat = 111320.0f;
    const float meters_per_deg_lon =
        111320.0f * cosf(lat1 * 3.14159265f / 180.0f);

    float dx = (lon2 - lon1) * meters_per_deg_lon;
    float dy = (lat2 - lat1) * meters_per_deg_lat;

    return sqrtf(dx * dx + dy * dy);
}

bool GpsFilter_Update(
    GpsFilter_t *filter,
    float input_lat,
    float input_lon,
    float alpha,
    float max_jump_meters,
    float *output_lat,
    float *output_lon)
{
    if (filter == NULL || output_lat == NULL || output_lon == NULL) return false;
    if (!gps_latlon_valid(input_lat, input_lon)) return false;

    if (!filter->initialized)
    {
        filter->lat         = input_lat;
        filter->lon         = input_lon;
        filter->initialized = true;
        *output_lat         = filter->lat;
        *output_lon         = filter->lon;
        return true;
    }

    float jump = gps_distance_m(filter->lat, filter->lon, input_lat, input_lon);

    if (jump > max_jump_meters)
    {
        *output_lat = filter->lat;
        *output_lon = filter->lon;
        return false;
    }

    filter->lat = filter->lat + alpha * (input_lat - filter->lat);
    filter->lon = filter->lon + alpha * (input_lon - filter->lon);

    *output_lat = filter->lat;
    *output_lon = filter->lon;

    return true;
}

void Kalman1D_Init(KalmanFilter1D_t *kf, float q, float r, float initial_x)
{
    if (kf == NULL) return;

    kf->q           = q;
    kf->r           = r;
    kf->p           = 1.0f;
    kf->x           = initial_x;
    kf->initialized = false;
}

float Kalman1D_Update(KalmanFilter1D_t *kf, float measurement)
{
    if (kf == NULL) return measurement;

    if (!kf->initialized)
    {
        kf->x           = measurement;
        kf->initialized = true;
        return kf->x;
    }

    float p_prior = kf->p + kf->q;
    float K       = p_prior / (p_prior + kf->r);
    kf->x         = kf->x + K * (measurement - kf->x);
    kf->p         = (1.0f - K) * p_prior;

    return kf->x;
}

void KalmanAltVel_Init(KalmanAltVel_t *kf, float q_pos, float q_vel, float r_alt)
{
    if (kf == NULL) return;

    kf->alt   = 0.0f;
    kf->vel   = 0.0f;
    kf->p00   = 1.0f;
    kf->p01   = 0.0f;
    kf->p10   = 0.0f;
    kf->p11   = 1.0f;
    kf->q_pos = q_pos;
    kf->q_vel = q_vel;
    kf->r_alt = r_alt;
    kf->initialized = false;
}

void KalmanAltVel_Update(KalmanAltVel_t *kf, float baro_altitude_m, float dt_s)
{
    if (kf == NULL) return;

    if (!kf->initialized)
    {
        kf->alt         = baro_altitude_m;
        kf->vel         = 0.0f;
        kf->initialized = true;
        return;
    }

    if (dt_s <= 0.0f || dt_s > 1.0f) dt_s = 0.02f;

    float alt_prior = kf->alt + kf->vel * dt_s;
    float vel_prior = kf->vel;

    float fp00 = kf->p00 + dt_s * kf->p10;
    float fp01 = kf->p01 + dt_s * kf->p11;
    float fp10 = kf->p10;
    float fp11 = kf->p11;

    float p00_prior = fp00 + dt_s * fp01 + kf->q_pos;
    float p01_prior = fp01;
    float p10_prior = fp10 + dt_s * fp11;
    float p11_prior = fp11 + kf->q_vel;

    float S   = p00_prior + kf->r_alt;
    float K0  = p00_prior / S;
    float K1  = p10_prior / S;

    float innov = baro_altitude_m - alt_prior;

    kf->alt = alt_prior + K0 * innov;
    kf->vel = vel_prior + K1 * innov;

    kf->p00 = (1.0f - K0) * p00_prior;
    kf->p01 = (1.0f - K0) * p01_prior;
    kf->p10 = p10_prior - K1 * p00_prior;
    kf->p11 = p11_prior - K1 * p01_prior;
}

#define VERT_KF_GRAVITY_MPS2   9.80665f

#define VERT_KF_DT_MIN_S       0.001f
#define VERT_KF_DT_MAX_S       0.500f

void VertKF_Init(VertKF_t *kf,
                 float q_alt,
                 float q_vel,
                 float r_baro,
                 float r_gps)
{
    if (kf == NULL) return;

    kf->alt   = 0.0f;
    kf->vel   = 0.0f;

    kf->p00   = 1.0f;
    kf->p01   = 0.0f;
    kf->p10   = 0.0f;
    kf->p11   = 1.0f;

    kf->q_alt  = q_alt;
    kf->q_vel  = q_vel;
    kf->r_baro = r_baro;
    kf->r_gps  = r_gps;

    kf->ground_alt_msl   = 0.0f;
    kf->ground_alt_valid = false;

    kf->g_ref_mg = 1000.0f;

    kf->initialized = false;
}

void VertKF_SetGravityRef(VertKF_t *kf, float g_ref_mg)
{
    if (kf == NULL) return;
    if (g_ref_mg < 700.0f || g_ref_mg > 1300.0f) return;   /* refuse garbage */
    kf->g_ref_mg = g_ref_mg;
}

void VertKF_Reset(VertKF_t *kf, float baro_alt_agl)
{
    if (kf == NULL) return;

    kf->alt         = baro_alt_agl;
    kf->vel         = 0.0f;

    kf->p00         = 1.0f;
    kf->p01         = 0.0f;
    kf->p10         = 0.0f;
    kf->p11         = 1.0f;

    kf->initialized = true;
}

void VertKF_UpdateGroundGpsAlt(VertKF_t *kf, float gps_alt_msl)
{
    if (kf == NULL)           return;
    if (!isfinite(gps_alt_msl)) return;

    if (!kf->ground_alt_valid)
    {
        kf->ground_alt_msl   = gps_alt_msl;
        kf->ground_alt_valid = true;
    }
    else
    {

        kf->ground_alt_msl =
            kf->ground_alt_msl + 0.1f * (gps_alt_msl - kf->ground_alt_msl);
    }
}

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
                   float       dt_s)
{
    if (kf == NULL) return;

    if (!kf->initialized)
    {
        kf->alt         = baro_alt_agl;
        kf->vel         = 0.0f;
        kf->initialized = true;
        return;
    }

    if (dt_s < VERT_KF_DT_MIN_S) dt_s = VERT_KF_DT_MIN_S;
    if (dt_s > VERT_KF_DT_MAX_S) dt_s = VERT_KF_DT_MAX_S;

    float R10 = 2.0f * (qx * qy + qz * qw);
    float R11 = 1.0f - 2.0f * (qx * qx + qz * qz);
    float R12 = 2.0f * (qy * qz - qx * qw);

    float a_world_y_mg = R10 * imu_rx_mg
                       + R11 * imu_ry_mg
                       + R12 * imu_rz_mg;

    /* Subtract the gravity we actually measured on the pad, not a hard-coded
       1000 mg. A 335 mg error here became a permanent -3.3 m/s^2 bias in the
       process model, which is exactly what drove the velocity negative. */
    float a_net_mps2 = (a_world_y_mg - kf->g_ref_mg) * 0.001f * VERT_KF_GRAVITY_MPS2;

    float dt2 = dt_s * dt_s;

    float alt_pred = kf->alt + kf->vel * dt_s + 0.5f * a_net_mps2 * dt2;
    float vel_pred = kf->vel + a_net_mps2 * dt_s;

    float fp00 = kf->p00 + dt_s * (kf->p01 + kf->p10) + dt2 * kf->p11;
    float fp01 = kf->p01 + dt_s * kf->p11;
    float fp10 = kf->p10 + dt_s * kf->p11;
    float fp11 = kf->p11;

    float p00_pred = fp00 + kf->q_alt;
    float p01_pred = fp01;
    float p10_pred = fp10;
    float p11_pred = fp11 + kf->q_vel;

    {
        float S    = p00_pred + kf->r_baro;
        float K0   = p00_pred / S;
        float K1   = p10_pred / S;

        float innov = baro_alt_agl - alt_pred;

        alt_pred = alt_pred + K0 * innov;
        vel_pred = vel_pred + K1 * innov;

        float new_p00 = (1.0f - K0) * p00_pred;
        float new_p01 = (1.0f - K0) * p01_pred;
        float new_p10 = p10_pred - K1 * p00_pred;
        float new_p11 = p11_pred - K1 * p01_pred;

        p00_pred = new_p00;
        p01_pred = new_p01;
        p10_pred = new_p10;
        p11_pred = new_p11;
    }

    if (gps != NULL && kf->ground_alt_valid)
    {
        bool fix_ok = (gps->valid          == true)
                   && (gps->fix            >= 1)
                   && (gps->sat            >= VERT_KF_GPS_MIN_SATELLITES)
                   && (gps->hdop           <= VERT_KF_GPS_MAX_HDOP)
                   && isfinite(gps->alt);

        if (fix_ok)
        {
            float gps_alt_agl = gps->alt - kf->ground_alt_msl;

            float S    = p00_pred + kf->r_gps;
            float K0   = p00_pred / S;
            float K1   = p10_pred / S;

            float innov = gps_alt_agl - alt_pred;

            alt_pred = alt_pred + K0 * innov;
            vel_pred = vel_pred + K1 * innov;

            float new_p00 = (1.0f - K0) * p00_pred;
            float new_p01 = (1.0f - K0) * p01_pred;
            float new_p10 = p10_pred - K1 * p00_pred;
            float new_p11 = p11_pred - K1 * p01_pred;

            p00_pred = new_p00;
            p01_pred = new_p01;
            p10_pred = new_p10;
            p11_pred = new_p11;
        }
    }

    kf->alt = alt_pred;
    kf->vel = vel_pred;
    kf->p00 = p00_pred;
    kf->p01 = p01_pred;
    kf->p10 = p10_pred;
    kf->p11 = p11_pred;
}
