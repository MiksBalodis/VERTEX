#include "mission.h"
#include "imu_fusion.h"
#include "main.h"
#include "buzzer.h"
#include "neopixel.h"
#include "BMP388.h"

extern Buzzer_Handle hbuzz1;

extern float ground_pressure;

IMU_Data_t imu;
extern BMP388_HandleTypeDef hbmp388;

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

typedef struct __attribute__((packed)) {
    float altitude;           // 4 bytes
    int16_t accel;            // 2 bytes (cm/s^2)
    int16_t speed;            // 2 bytes (cm/s)
    int16_t pitch, roll, yaw; // 6 bytes (DEG*10)
    uint8_t flight_state;     // 1 byte
    uint32_t timestamp;       // 4 bytes
} TelemetryData_t; // Total: 19 bytes (MAX for BLE packet is 20 bytes)

TelemetryData_t telemetry;

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

    telemetry.flight_state = mission_state;
}

void Mission_Update(void)
{
    uint32_t rprs, rtemp, time;
    float prs, temp;

    IMU_Fusion_Update(&imu);

    BMP388_ReadRawPressTempTime(&hbmp388, &rprs, &rtemp, &time);
    BMP388_CompensateRawPressTemp(&hbmp388, rprs, rtemp, &prs, &temp);

    telemetry.flight_state = mission_state;
    telemetry.altitude = BMP388_FindAltitude(ground_pressure, prs);

    switch (mission_state)
    {
        case MISSION_READY:
            // LSM6DSO accel values are in mg so 3000 ~ 3 g
            if (imu.ry > 3000.0f){
                mission_state = MISSION_ASCENT;
                BUZZ(&hbuzz1, 100);
            }
            ground_pressure = prs;
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
