#include "mission.h"
#include "imu_fusion.h"
#include "main.h"
#include "buzzer.h"

extern Buzzer_Handle hbuzz1;

IMU_Data_t imu;

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
    IMU_Fusion_Update(&imu);

    switch (mission_state)
    {
        case MISSION_READY:
            // LSM6DSO accel values are in mg so 3000 ~ 3 g
            if (imu.ry > 3000.0f){
                mission_state = MISSION_ASCENT;
                BUZZ(&hbuzz1, 100);
            }
            break;

        case MISSION_ASCENT:
            break;

        default:
            mission_state = MISSION_READY;
            break;
    }
}
