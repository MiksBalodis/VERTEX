#include "imu_fusion.h"
#include "main.h"
#include "lsm6dso.h"
#include <string.h>

static LSM6DSO_Object_t *imu_handle = NULL;
// static IMU_Data_t imu_data;


static float gx_bias = 0.0f;
static float gy_bias = 0.0f;
static float gz_bias = 0.0f;

static float lowpass(float old_value, float new_value, float alpha)
{
    return old_value + alpha * (new_value - old_value);
}

void IMU_Fusion_Init(LSM6DSO_Object_t *imu)
{
    imu_handle = imu;
    // memset(&imu_data, 0, sizeof(imu_data));

    LSM6DSO_ACC_SetFullScale(imu_handle, LSM6DSO_ACC_SENSITIVITY_FS_16G);
    LSM6DSO_GYRO_SetFullScale(imu_handle, LSM6DSO_GYRO_SENSITIVITY_FS_2000DPS);

    LSM6DSO_ACC_SetOutputDataRate(imu_handle, IMU_ODR_3333_HZ);
    LSM6DSO_GYRO_SetOutputDataRate(imu_handle, IMU_ODR_3333_HZ);

    LSM6DSO_ACC_Set_Filter_Mode(imu_handle, 0, IMU_ODR_DIV_20);
    LSM6DSO_GYRO_Set_Filter_Mode(imu_handle, 0, IMU_ODR_DIV_20);

    LSM6DSO_ACC_Enable(imu_handle);
    LSM6DSO_GYRO_Enable(imu_handle);

    LSM6DSO_ACC_Enable_DRDY_On_INT1(imu_handle);
}

void IMU_Fusion_CalibrateGyro(uint16_t samples)
{
    if (imu_handle == NULL || samples == 0) return;

    LSM6DSO_Axes_t gyro_raw;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;

    for (uint16_t i = 0; i < samples; i++)
    {
        if (LSM6DSO_GYRO_GetAxes(imu_handle, &gyro_raw) == LSM6DSO_OK)
        {
            float sensor_x = (float)gyro_raw.x;
            float sensor_y = (float)gyro_raw.y;
            float sensor_z = (float)gyro_raw.z;

            float rocket_x = -sensor_x;
            float rocket_y = -sensor_y;
            float rocket_z =  sensor_z;

            sum_x += rocket_x;
            sum_y += rocket_y;
            sum_z += rocket_z;
        }

        HAL_Delay(5);
    }

    gx_bias = sum_x / samples;
    gy_bias = sum_y / samples;
    gz_bias = sum_z / samples;
}

void IMU_Fusion_Update(IMU_Data_t *imu_data)
{
    if (imu_handle == NULL) return;

    LSM6DSO_Axes_t acc_raw;
    LSM6DSO_Axes_t gyro_raw;

    if (LSM6DSO_ACC_GetAxes(imu_handle, &acc_raw) == LSM6DSO_OK)
    {
        float sensor_x = (float)acc_raw.x;
        float sensor_y = (float)acc_raw.y;
        float sensor_z = (float)acc_raw.z;

        float rocket_x = -sensor_x;
        float rocket_y = -sensor_y;
        float rocket_z =  sensor_z;

        imu_data->rx = lowpass(imu_data->rx, rocket_x, 0.01f);
        imu_data->ry = lowpass(imu_data->ry, rocket_y, 0.01f);
        imu_data->rz = lowpass(imu_data->rz, rocket_z, 0.01f);
    }

    if (LSM6DSO_GYRO_GetAxes(imu_handle, &gyro_raw) == LSM6DSO_OK)
    {
        float sensor_x = (float)gyro_raw.x;
        float sensor_y = (float)gyro_raw.y;
        float sensor_z = (float)gyro_raw.z;

        float rocket_x = (-sensor_x - gx_bias)*0.070f;
        float rocket_y = (-sensor_y - gy_bias)*0.070f;
        float rocket_z =  (sensor_z - gz_bias)*0.070f;

        imu_data->gx = lowpass(imu_data->gx, rocket_x, 0.01f);
        imu_data->gy = lowpass(imu_data->gy, rocket_y, 0.01f);
        imu_data->gz = lowpass(imu_data->gz, rocket_z, 0.01f);
    }
}

void IMU_Fusion_IntegrateGyro(IMU_Integration_t *imu_integration,  uint32_t time){
    LSM6DSO_Axes_t gyro_raw;

    float dt = (float)(time - imu_integration->time) * 1e-6f; // us to seconds (0.000300s)

    if (LSM6DSO_GYRO_GetAxes(imu_handle, &gyro_raw) == LSM6DSO_OK) {
        imu_integration->pitch += ((-(float)gyro_raw.x - gx_bias) * 0.070f) * dt;
        imu_integration->yaw   += ((-(float)gyro_raw.y - gy_bias) * 0.070f) * dt;
        imu_integration->roll  += ( ((float)gyro_raw.z - gz_bias) * 0.070f) * dt;

        imu_integration->time = time;
    }
}

// IMU_Data_t IMU_Fusion_GetData(void)
// {
//     return imu_data;
// }
