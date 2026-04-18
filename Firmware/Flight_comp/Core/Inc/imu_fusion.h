#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#include "lsm6dso.h"
#include <stdint.h>

#define IMU_ODR_208_HZ  208.0f
#define IMU_ODR_3333_HZ  3333.0f
#define IMU_ODR_DIV_20  0x02

typedef struct {
    float rx;   // rocket right = -sensor_x
    float ry;   // rocket up = -sensor_y
    float rz;   // rocket front = +sensor_z

    float gx;   // rocket gyro right-axis
    float gy;   // rocket gyro up-axis
    float gz;   // rocket gyro front-axis
} IMU_Data_t;

typedef struct {
    // Speed (m/s)
    volatile float vx;   // rocket right = -sensor_x
    volatile float vy;   // rocket up = -sensor_y
    volatile float vz;   // rocket front = +sensor_z

    // Angle (DEG)
    volatile float pitch;   // rocket gyro right-axis
    volatile float yaw;   // rocket gyro up-axis
    volatile float roll;   // rocket gyro front-axis

    volatile uint32_t time;
} IMU_Integration_t;

void IMU_Fusion_Init(LSM6DSO_Object_t *imu);
void IMU_Fusion_CalibrateGyro(uint16_t samples);
void IMU_Fusion_Update(IMU_Data_t *imu_data);
void IMU_Fusion_IntegrateGyro(IMU_Integration_t *imu_integration, uint32_t time);
void IMU_Fusion_SetGyro(IMU_Integration_t *imu_integration, float gx, float gy, float gz, uint32_t time);
int32_t LSM6DSO_ACC_GYRO_Enable_DRDY_On_INT1(LSM6DSO_Object_t *pObj);
// IMU_Data_t IMU_Fusion_GetData(void);

#endif
