#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#include "lsm6dso.h"
#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------------------
   Output data rate.

   416 Hz is deliberate. The old 3333 Hz was only ever there to feed a DRDY
   interrupt that did blocking SPI reads at 3.3 kHz -- that ISR collided with
   the main loop on SPI2 and corrupted every accel sample. The IMU is now read
   from a single fixed-rate control loop (Mission_ControlTick @ 400 Hz), so the
   ODR only has to be comfortably above that.
   --------------------------------------------------------------------------- */
#define IMU_ODR_416_HZ    416.0f
#define IMU_ODR_833_HZ    833.0f

/* Accel display/telemetry filter. The OLD value was 0.01 => ~100-sample time
   constant, i.e. seconds of lag on the signal that fed launch detection and the
   Kalman process model. The gyro is now left COMPLETELY unfiltered because the
   TVC D term needs it raw. */
#define IMU_ACC_LP_ALPHA  0.25f

typedef struct {
    float rx;   // rocket right = -sensor_x   [mg]
    float ry;   // rocket up    = -sensor_y   [mg]
    float rz;   // rocket front = +sensor_z   [mg]

    float gx;   // gyro about rocket right axis -> pitch rate  [mdps]  RAW
    float gy;   // gyro about rocket up    axis -> roll  rate  [mdps]  RAW
    float gz;   // gyro about rocket front axis -> yaw   rate  [mdps]  RAW
} IMU_Data_t;

typedef struct {
    // Angle (DEG), integrated from body rates, zeroed at launch
    volatile float pitch;   // about rocket right axis
    volatile float roll;    // about rocket up (long) axis
    volatile float yaw;     // about rocket front axis

    volatile uint32_t time; // us, from Mission_GetTick()
} IMU_Integration_t;

typedef struct {
    float w, x, y, z;
} Quaternion;

void  IMU_Fusion_Init(LSM6DSO_Object_t *imu);
void  IMU_Fusion_CalibrateGyro(uint16_t samples);

/* Single read of BOTH sensors + attitude integration. This is the ONLY function
   that is allowed to touch SPI2 once the control loop is running. */
void  IMU_Fusion_Update(IMU_Data_t *imu_data, IMU_Integration_t *integ, uint32_t time_us);

/* Measure |g| on the pad. Blocking - call from the main loop while the control
   loop is stopped. Returns the accel magnitude in mg and stores the gravity
   direction for later quaternion seeding. */
float IMU_Fusion_CaptureGravityRef(uint16_t samples);
float IMU_Fusion_GetGravityRefMg(void);

/* Zero the euler integrals and seed the quaternion from the measured gravity
   direction, so that "world up" really is up instead of assuming the board is
   perfectly vertical. Call once, at launch detection. */
void  IMU_Fusion_ResetAttitude(IMU_Integration_t *integ, uint32_t time_us);

int32_t LSM6DSO_ACC_GYRO_Disable_DRDY_On_INT1(LSM6DSO_Object_t *pObj);

void quat_normalize(Quaternion *q);
void quat_update_from_dps(Quaternion *q, float gx, float gy, float gz, float dt);

#endif
