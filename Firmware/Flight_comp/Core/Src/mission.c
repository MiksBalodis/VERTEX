#include "mission.h"
#include "imu_fusion.h"
#include "main.h"
#include "buzzer.h"
#include "neopixel.h"

extern Buzzer_Handle hbuzz1;

IMU_Data_t imu;

typedef enum
{
    MISSION_READY = 0,
    MISSION_ASCENT,
    MISSION_DECENT,
    MISSION_LANDED,
    MISSION_IDLE,
    MISSION_POST_FAIL
} MissionState_t;

static MissionState_t mission_state = MISSION_READY;

bool POST_fault_flags[FAULT_MAX] = {0};

void Mission_Init(void)
{
    mission_state = MISSION_READY;

    for (uint8_t fault = 0; fault < FAULT_MAX; fault++){
        if(POST_fault_flags[fault]){
            mission_state = MISSION_POST_FAIL;
            break;
        }
    }
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

        case MISSION_POST_FAIL:
            Mission_SafeMode();
            break;

        default:
            mission_state = MISSION_READY;
            break;
    }
}

void Mission_SafeMode(void){
    /*TO DO: show faults*/

    LED_Set_Color(64, 0, 0);

    __disable_irq();

    while(1);
}
