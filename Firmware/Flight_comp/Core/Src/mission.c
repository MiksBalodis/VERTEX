#include "mission.h"
#include "imu_fusion.h"
#include "main.h"

typedef enum
{
    MISSION_READY = 0,
    MISSION_ASCENT
} MissionState_t;

static MissionState_t mission_state = MISSION_READY;

void Mission_Init(void)
{
    mission_state = MISSION_READY;
}

void Mission_Update(void)
{
    IMU_Fusion_Update();
    IMU_Data_t imu = IMU_Fusion_GetData();

    switch (mission_state)
    {
        case MISSION_READY:
            // LSM6DSO accel values are in mg so 3000 ~ 3 g
            if (imu.ry > 3000.0f)
            {
                mission_state = MISSION_ASCENT;

                HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
                HAL_Delay(100);
                HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
            }
            break;

        case MISSION_ASCENT:
            break;

        default:
            mission_state = MISSION_READY;
            break;
    }
}
