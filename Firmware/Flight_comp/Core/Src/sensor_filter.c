
#include "sensor_filter.h"
/* gps.h is already included transitively through sensor_filter.h */
#include <math.h>
#include <stddef.h>    /* NULL                                           */

/* =========================================================================
   Low-pass filter
   ========================================================================= */

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

/* =========================================================================
   GPS position filter
   ========================================================================= */

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

/* =========================================================================
   1-D scalar Kalman filter
   ========================================================================= */

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

/* =========================================================================
   2-state altitude + velocity Kalman filter  (constant-velocity model)
   ========================================================================= */

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

    /* --- Predict --- */
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

    /* --- Update (baro, H = [1 0]) --- */
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

/* =========================================================================
   Multi-sensor vertical Kalman filter  (VertKF_t)
   =========================================================================

   State:     s  = [ alt (m),  vel (m/s) ]ᵀ

   Process model with IMU acceleration as control input u = a_net (m/s²):
     alt_k  = alt_{k-1}  +  vel_{k-1}·dt  +  0.5·u·dt²
     vel_k  = vel_{k-1}  +  u·dt

   Transition matrix:    F = [[1, dt], [0, 1]]
   Control-input matrix: G = [[0.5·dt²], [dt]]

   Process noise covariance:
     Q = [[q_alt, 0], [0, q_vel]]

   Baro and GPS are both scalar measurements:
     z_baro = alt + v_baro,   v_baro ~ N(0, r_baro)
     z_gps  = alt + v_gps,    v_gps  ~ N(0, r_gps)
     H = [1, 0]  for both.

   IMU vertical acceleration derivation
   -------------------------------------
   The IMU reports specific force (gravity + kinematic acceleration) in the
   body frame, in units of mg.  The rocket body frame is:
     rx = rocket-right,  ry = rocket-up,  rz = rocket-front.

   The attitude quaternion q represents body→world at identity quaternion,
   where "world Y" is the vertical axis (Y-up world, rocket starts vertical).

   World Y component of any body vector v = [vx, vy, vz]ᵀ:
     R_11 = 1 - 2(qx²+qz²)
     R_10 = 2(qx·qy + qz·qw)
     R_12 = 2(qy·qz - qx·qw)

     v_world_Y = R_10·vx + R_11·vy + R_12·vz

   At identity quat: R_10=0, R_11=1, R_12=0  →  v_world_Y = vy = ry.
   On the pad at rest: ry ≈ +1000 mg (reaction force from ground, i.e. 1 g).
   True kinematic vertical acceleration = (v_world_Y - 1000) · 0.001 · 9.80665

   GPS gating
   ----------
   GPS altitude is accepted only when ALL of the following hold:
     gps->valid == true            (NMEA CRC passed)
     gps->fix   >= 1               (at least a standard GPS fix)
     gps->sat   >= VERT_KF_GPS_MIN_SATELLITES
     gps->hdop  <= VERT_KF_GPS_MAX_HDOP
     ground_alt_valid == true      (AGL conversion reference is ready)

   GPS AGL = gps->alt  -  kf->ground_alt_msl
   ========================================================================= */

/* Gravity (m/s²) */
#define VERT_KF_GRAVITY_MPS2   9.80665f

/* Guard rails on dt */
#define VERT_KF_DT_MIN_S       0.001f
#define VERT_KF_DT_MAX_S       0.500f

/* -------------------------------------------------------------------------
   VertKF_Init
   ------------------------------------------------------------------------- */
void VertKF_Init(VertKF_t *kf,
                 float q_alt,
                 float q_vel,
                 float r_baro,
                 float r_gps)
{
    if (kf == NULL) return;

    kf->alt   = 0.0f;
    kf->vel   = 0.0f;

    /* Start with moderate uncertainty; the filter will converge quickly. */
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

    kf->initialized = false;
}

/* -------------------------------------------------------------------------
   VertKF_Reset
   ------------------------------------------------------------------------- */
void VertKF_Reset(VertKF_t *kf, float baro_alt_agl)
{
    if (kf == NULL) return;

    kf->alt         = baro_alt_agl;
    kf->vel         = 0.0f;

    /* Reset covariance to moderate values so the filter re-converges
       quickly after re-seeding on the pad. */
    kf->p00         = 1.0f;
    kf->p01         = 0.0f;
    kf->p10         = 0.0f;
    kf->p11         = 1.0f;

    kf->initialized = true;   /* mark as ready for Update */
}

/* -------------------------------------------------------------------------
   VertKF_UpdateGroundGpsAlt
   ------------------------------------------------------------------------- */
void VertKF_UpdateGroundGpsAlt(VertKF_t *kf, float gps_alt_msl)
{
    if (kf == NULL)           return;
    if (!isfinite(gps_alt_msl)) return;

    /* Simple low-pass to average out GPS noise while on the pad. */
    if (!kf->ground_alt_valid)
    {
        kf->ground_alt_msl   = gps_alt_msl;
        kf->ground_alt_valid = true;
    }
    else
    {
        /* Alpha = 0.1: slow drift toward the true mean over ~10 samples. */
        kf->ground_alt_msl =
            kf->ground_alt_msl + 0.1f * (gps_alt_msl - kf->ground_alt_msl);
    }
}

/* -------------------------------------------------------------------------
   VertKF_Update
   ------------------------------------------------------------------------- */
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

    /* ------------------------------------------------------------------
       Bootstrap: seed with first baro measurement.
       ------------------------------------------------------------------ */
    if (!kf->initialized)
    {
        kf->alt         = baro_alt_agl;
        kf->vel         = 0.0f;
        kf->initialized = true;
        return;
    }

    /* ------------------------------------------------------------------
       Clamp dt to safe range.
       ------------------------------------------------------------------ */
    if (dt_s < VERT_KF_DT_MIN_S) dt_s = VERT_KF_DT_MIN_S;
    if (dt_s > VERT_KF_DT_MAX_S) dt_s = VERT_KF_DT_MAX_S;

    /* ------------------------------------------------------------------
       1. Compute world-vertical specific force from IMU + attitude.
          R row-1 (Y-axis) of body→world rotation matrix:
            R10 = 2*(qx*qy + qz*qw)
            R11 = 1 - 2*(qx*qx + qz*qz)
            R12 = 2*(qy*qz - qx*qw)
          World-Y component of body accel vector:
            a_world_y_mg = R10*rx + R11*ry + R12*rz
          Net vertical acceleration (subtract gravity reaction force 1g=1000mg):
            a_net_mps2 = (a_world_y_mg - 1000.0f) * 0.001f * GRAVITY
       ------------------------------------------------------------------ */
    float R10 = 2.0f * (qx * qy + qz * qw);
    float R11 = 1.0f - 2.0f * (qx * qx + qz * qz);
    float R12 = 2.0f * (qy * qz - qx * qw);

    float a_world_y_mg = R10 * imu_rx_mg
                       + R11 * imu_ry_mg
                       + R12 * imu_rz_mg;

    /* Net kinematic vertical acceleration in m/s²:
       IMU reads specific force f = a_kinematic + g (reaction frame).
       Subtracting 1 g (= 1000 mg) gives the net inertial acceleration. */
    float a_net_mps2 = (a_world_y_mg - 1000.0f) * 0.001f * VERT_KF_GRAVITY_MPS2;

    /* ------------------------------------------------------------------
       2. PREDICT step.
          alt_pred = alt + vel·dt + 0.5·a·dt²
          vel_pred = vel + a·dt
          P_pred   = F·P·Fᵀ + Q
       ------------------------------------------------------------------ */
    float dt2 = dt_s * dt_s;

    float alt_pred = kf->alt + kf->vel * dt_s + 0.5f * a_net_mps2 * dt2;
    float vel_pred = kf->vel + a_net_mps2 * dt_s;

    /* F = [[1, dt],[0, 1]]
       F·P·Fᵀ:
         [0][0] = p00 + dt*p10 + dt*(p01 + dt*p11)  =  p00 + dt*(p01+p10) + dt²*p11
         [0][1] = p01 + dt*p11
         [1][0] = p10 + dt*p11
         [1][1] = p11
    */
    float fp00 = kf->p00 + dt_s * (kf->p01 + kf->p10) + dt2 * kf->p11;
    float fp01 = kf->p01 + dt_s * kf->p11;
    float fp10 = kf->p10 + dt_s * kf->p11;
    float fp11 = kf->p11;

    /* P_pred = F·P·Fᵀ + Q */
    float p00_pred = fp00 + kf->q_alt;
    float p01_pred = fp01;
    float p10_pred = fp10;
    float p11_pred = fp11 + kf->q_vel;

    /* ------------------------------------------------------------------
       3. UPDATE — barometric altitude (primary, every step).
          H = [1, 0]
          S  = H·P·Hᵀ + r_baro  = p00_pred + r_baro
          K  = P·Hᵀ / S          = [p00_pred, p10_pred]ᵀ / S
          x  = x_pred + K·(z_baro - alt_pred)
          P  = (I - K·H)·P_pred
       ------------------------------------------------------------------ */
    {
        float S    = p00_pred + kf->r_baro;
        float K0   = p00_pred / S;    /* gain for altitude state */
        float K1   = p10_pred / S;    /* gain for velocity state */

        float innov = baro_alt_agl - alt_pred;

        alt_pred = alt_pred + K0 * innov;
        vel_pred = vel_pred + K1 * innov;

        /* Joseph form would be ideal; for a 2×2 the explicit form is fine. */
        float new_p00 = (1.0f - K0) * p00_pred;
        float new_p01 = (1.0f - K0) * p01_pred;
        float new_p10 = p10_pred - K1 * p00_pred;
        float new_p11 = p11_pred - K1 * p01_pred;

        p00_pred = new_p00;
        p01_pred = new_p01;
        p10_pred = new_p10;
        p11_pred = new_p11;
    }

    /* ------------------------------------------------------------------
       4. UPDATE — GPS altitude (conditional).

          Only fuse when all quality gates pass.
          GPS measures MSL altitude; convert to AGL using ground reference.
          This is an independent measurement of altitude (not vel), so
          H = [1, 0] again.

          The GPS update runs on the already-baro-updated state and
          covariance, which is correct — sequential updates are exact for
          independent sensors.
       ------------------------------------------------------------------ */
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

    /* ------------------------------------------------------------------
       5. Commit updated state and covariance.
       ------------------------------------------------------------------ */
    kf->alt = alt_pred;
    kf->vel = vel_pred;
    kf->p00 = p00_pred;
    kf->p01 = p01_pred;
    kf->p10 = p10_pred;
    kf->p11 = p11_pred;
}
