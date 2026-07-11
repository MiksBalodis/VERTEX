#include "imu_fusion.h"
#include "main.h"
#include "lsm6dso.h"

#include <stdint.h>
#include <string.h>
#include <math.h>

static LSM6DSO_Object_t *imu_handle = NULL;

volatile Quaternion quat = {1.0f, 0.0f, 0.0f, 0.0f};

static float gx_bias = 0.0f;   /* mdps */
static float gy_bias = 0.0f;
static float gz_bias = 0.0f;

/* Cached once at init instead of re-reading the full-scale register on EVERY
   single sample the way LSM6DSO_ACC_GetAxes()/GYRO_GetAxes() do. That halves
   the SPI traffic. */
static float acc_sens_mg_lsb   = 0.488f;   /* +-16 g  */
static float gyro_sens_mdps_lsb = 70.0f;   /* +-2000 dps */

/* Gravity as actually measured on the pad, in mg, plus its direction. */
static float grav_ref_mg = 1000.0f;
static float grav_dir_x  = 0.0f;
static float grav_dir_y  = 1.0f;
static float grav_dir_z  = 0.0f;

static float lowpass(float old_value, float new_value, float alpha)
{
    return old_value + alpha * (new_value - old_value);
}

/* --------------------------------------------------------------------------
   Raw reads using the cached sensitivity.
   -------------------------------------------------------------------------- */
static bool imu_read_acc_mg(float *ax, float *ay, float *az)
{
    LSM6DSO_AxesRaw_t raw;
    if (LSM6DSO_ACC_GetAxesRaw(imu_handle, &raw) != LSM6DSO_OK) return false;

    /* sensor -> rocket frame */
    *ax = -((float)raw.x) * acc_sens_mg_lsb;   /* right */
    *ay = -((float)raw.y) * acc_sens_mg_lsb;   /* up    */
    *az =  ((float)raw.z) * acc_sens_mg_lsb;   /* front */
    return true;
}

static bool imu_read_gyro_mdps(float *gx, float *gy, float *gz)
{
    LSM6DSO_AxesRaw_t raw;
    if (LSM6DSO_GYRO_GetAxesRaw(imu_handle, &raw) != LSM6DSO_OK) return false;

    *gx = -((float)raw.x) * gyro_sens_mdps_lsb;
    *gy = -((float)raw.y) * gyro_sens_mdps_lsb;
    *gz =  ((float)raw.z) * gyro_sens_mdps_lsb;
    return true;
}

/* --------------------------------------------------------------------------
   Init
   -------------------------------------------------------------------------- */
void IMU_Fusion_Init(LSM6DSO_Object_t *imu)
{
    imu_handle = imu;

    LSM6DSO_ACC_Enable(imu_handle);
    LSM6DSO_GYRO_Enable(imu_handle);

    /* FIXED: these take the full scale in g / dps, NOT the sensitivity float.
       The old code passed 0.488f -> truncated to int 0 -> +-2 g, and 70.0f ->
       int 70 -> +-125 dps. Both saturated within the first half second of
       boost. */
    LSM6DSO_ACC_SetFullScale(imu_handle, 16);
    LSM6DSO_GYRO_SetFullScale(imu_handle, 2000);

    LSM6DSO_ACC_SetOutputDataRate(imu_handle, IMU_ODR_416_HZ);
    LSM6DSO_GYRO_SetOutputDataRate(imu_handle, IMU_ODR_416_HZ);

    /* Read back what the chip actually ended up in and cache it. If the part
       ever turns out to be an LSM6DSO32 (different FS encoding) this is where
       the mismatch will show up. */
    float_t s;
    if (LSM6DSO_ACC_GetSensitivity(imu_handle, &s) == LSM6DSO_OK)  acc_sens_mg_lsb    = (float)s;
    if (LSM6DSO_GYRO_GetSensitivity(imu_handle, &s) == LSM6DSO_OK) gyro_sens_mdps_lsb = (float)s;

    /* No more DRDY interrupt. The IMU is read from one fixed-rate loop. */
    LSM6DSO_ACC_GYRO_Disable_DRDY_On_INT1(imu_handle);
}

int32_t LSM6DSO_ACC_GYRO_Disable_DRDY_On_INT1(LSM6DSO_Object_t *pObj)
{
    lsm6dso_pin_int1_route_t pin_int1_route;

    if (lsm6dso_pin_int1_route_get(&(pObj->Ctx), &pin_int1_route) != LSM6DSO_OK)
        return LSM6DSO_ERROR;

    pin_int1_route.drdy_xl = 0;
    pin_int1_route.drdy_g  = 0;

    if (lsm6dso_pin_int1_route_set(&(pObj->Ctx), pin_int1_route) != LSM6DSO_OK)
        return LSM6DSO_ERROR;

    return LSM6DSO_OK;
}

/* --------------------------------------------------------------------------
   Gyro bias. Blocking. Main loop only, control loop must be stopped.
   -------------------------------------------------------------------------- */
void IMU_Fusion_CalibrateGyro(uint16_t samples)
{
    if (imu_handle == NULL || samples == 0) return;

    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    uint16_t n = 0;

    gx_bias = gy_bias = gz_bias = 0.0f;

    for (uint16_t i = 0; i < samples; i++)
    {
        float gx, gy, gz;
        if (imu_read_gyro_mdps(&gx, &gy, &gz))
        {
            sum_x += gx;
            sum_y += gy;
            sum_z += gz;
            n++;
        }
        HAL_Delay(3);
    }

    if (n == 0) return;

    gx_bias = sum_x / (float)n;
    gy_bias = sum_y / (float)n;
    gz_bias = sum_z / (float)n;
}

/* --------------------------------------------------------------------------
   Gravity reference. Blocking. Main loop only, control loop must be stopped.

   This replaces the hard-coded 1000 mg the Kalman filter used to subtract. If
   the pad is tilted, or the accel scale is a few percent off, this absorbs it
   instead of turning into a permanent acceleration bias that integrates
   straight into the velocity estimate.
   -------------------------------------------------------------------------- */
float IMU_Fusion_CaptureGravityRef(uint16_t samples)
{
    if (imu_handle == NULL || samples == 0) return grav_ref_mg;

    float sx = 0.0f, sy = 0.0f, sz = 0.0f;
    uint16_t n = 0;

    for (uint16_t i = 0; i < samples; i++)
    {
        float ax, ay, az;
        if (imu_read_acc_mg(&ax, &ay, &az))
        {
            sx += ax; sy += ay; sz += az;
            n++;
        }
        HAL_Delay(3);
    }

    if (n == 0) return grav_ref_mg;

    sx /= (float)n;
    sy /= (float)n;
    sz /= (float)n;

    float mag = sqrtf(sx*sx + sy*sy + sz*sz);

    /* Sanity: a stationary accelerometer must read ~1 g. If it does not, the
       scale or the bus is still broken -- fall back to 1000 rather than bake a
       garbage reference into the filter. */
    if (mag < 700.0f || mag > 1300.0f)
    {
        grav_ref_mg = 1000.0f;
        grav_dir_x  = 0.0f;
        grav_dir_y  = 1.0f;
        grav_dir_z  = 0.0f;
        return grav_ref_mg;
    }

    grav_ref_mg = mag;
    grav_dir_x  = sx / mag;
    grav_dir_y  = sy / mag;
    grav_dir_z  = sz / mag;

    return grav_ref_mg;
}

float IMU_Fusion_GetGravityRefMg(void)
{
    return grav_ref_mg;
}

/* --------------------------------------------------------------------------
   Attitude reset at launch.

   The old code forced quat = identity, which asserts "the board's -Y axis is
   pointing exactly at the sky". Any pad tilt then became a permanent bias in
   the Kalman process model. Instead, seed the quaternion with the shortest-arc
   rotation that maps the MEASURED gravity direction onto world up (0,1,0).
   -------------------------------------------------------------------------- */
void IMU_Fusion_ResetAttitude(IMU_Integration_t *integ, uint32_t time_us)
{
    if (integ != NULL)
    {
        integ->pitch = 0.0f;
        integ->roll  = 0.0f;
        integ->yaw   = 0.0f;
        integ->time  = time_us;
    }

    float ux = grav_dir_x, uy = grav_dir_y, uz = grav_dir_z;

    Quaternion q;
    q.w = 1.0f + uy;      /* 1 + u . (0,1,0) */
    q.x = -uz;            /* u x (0,1,0)     */
    q.y = 0.0f;
    q.z = ux;

    if (q.w < 1e-6f)      /* degenerate: board is upside down */
    {
        q.w = 0.0f; q.x = 1.0f; q.y = 0.0f; q.z = 0.0f;
    }

    quat_normalize(&q);

    quat.w = q.w;
    quat.x = q.x;
    quat.y = q.y;
    quat.z = q.z;
}

/* --------------------------------------------------------------------------
   The one and only IMU read. Called from Mission_ControlTick() at 400 Hz.
   -------------------------------------------------------------------------- */
void IMU_Fusion_Update(IMU_Data_t *imu_data, IMU_Integration_t *integ, uint32_t time_us)
{
    if (imu_handle == NULL || imu_data == NULL) return;

    float ax, ay, az;
    if (imu_read_acc_mg(&ax, &ay, &az))
    {
        imu_data->rx = lowpass(imu_data->rx, ax, IMU_ACC_LP_ALPHA);
        imu_data->ry = lowpass(imu_data->ry, ay, IMU_ACC_LP_ALPHA);
        imu_data->rz = lowpass(imu_data->rz, az, IMU_ACC_LP_ALPHA);
    }

    float gx_m, gy_m, gz_m;
    if (imu_read_gyro_mdps(&gx_m, &gy_m, &gz_m))
    {
        /* RAW, bias-corrected, NOT low-passed. The TVC D term is a derivative
           term -- feeding it a lagged signal turns damping into phase lag,
           which is anti-damping. */
        imu_data->gx = gx_m - gx_bias;
        imu_data->gy = gy_m - gy_bias;
        imu_data->gz = gz_m - gz_bias;

        if (integ != NULL)
        {
            /* unsigned wraparound is well defined, no abs() needed */
            float dt = (float)(time_us - integ->time) * 1e-6f;
            if (dt <= 0.0f || dt > 0.05f) dt = 0.0025f;   /* 400 Hz fallback */

            float gx = imu_data->gx * 1e-3f;   /* dps */
            float gy = imu_data->gy * 1e-3f;
            float gz = imu_data->gz * 1e-3f;

            integ->pitch += gx * dt;
            integ->roll  += gy * dt;
            integ->yaw   += gz * dt;

            Quaternion q = { quat.w, quat.x, quat.y, quat.z };
            quat_update_from_dps(&q, gx, gy, gz, dt);
            quat.w = q.w; quat.x = q.x; quat.y = q.y; quat.z = q.z;

            integ->time = time_us;
        }
    }
}

/* --------------------------------------------------------------------------
   Quaternion helpers (unchanged maths)
   -------------------------------------------------------------------------- */
void quat_normalize(Quaternion *q)
{
    float magnitude = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (magnitude > 0.00001f)
    {
        q->w /= magnitude;
        q->x /= magnitude;
        q->y /= magnitude;
        q->z /= magnitude;
    }
}

void quat_update_from_dps(Quaternion *q, float gx, float gy, float gz, float dt)
{
    float wx = gx * ((float)M_PI / 180.0f);
    float wy = gy * ((float)M_PI / 180.0f);
    float wz = gz * ((float)M_PI / 180.0f);

    float qw = q->w, qx = q->x, qy = q->y, qz = q->z;

    float dw = 0.5f * (-qx * wx - qy * wy - qz * wz);
    float dx = 0.5f * ( qw * wx + qy * wz - qz * wy);
    float dy = 0.5f * ( qw * wy - qx * wz + qz * wx);
    float dz = 0.5f * ( qw * wz + qx * wy - qy * wx);

    q->w += dw * dt;
    q->x += dx * dt;
    q->y += dy * dt;
    q->z += dz * dt;

    quat_normalize(q);
}
