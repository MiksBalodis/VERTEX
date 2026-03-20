#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#include "lsm6dso.h"
#include <stdint.h>

typedef struct {
    float rx;   // rocket right = -sensor_x
    float ry;   // rocket up = -sensor_y
    float rz;   // rocket front = +sensor_z

    float gx;   // rocket gyro right-axis
    float gy;   // rocket gyro up-axis
    float gz;   // rocket gyro front-axis
} IMU_Data_t;

void IMU_Fusion_Init(LSM6DSO_Object_t *imu);
void IMU_Fusion_CalibrateGyro(uint16_t samples);
void IMU_Fusion_Update(IMU_Data_t *imu_data);
// IMU_Data_t IMU_Fusion_GetData(void);

#endif
