#include "mission.h"
#include "imu_fusion.h"
#include "main.h"
#include "buzzer.h"
#include "neopixel.h"
#include "BMP388.h"
#include "fatfs.h"

extern Buzzer_Handle hbuzz1;
extern TIM_HandleTypeDef htim6;

float ground_pressure;

IMU_Data_t imu;
IMU_Integration_t imu_integration;
extern BMP388_HandleTypeDef hbmp388;
extern FATFS FatFs;

volatile uint32_t overflow; // TO DO: add us timer with interrupts

typedef enum
{
    MISSION_READY = 0,
    MISSION_ASCENT,
    MISSION_DECENT,
    MISSION_LANDED,
    MISSION_IDLE,
    MISSION_POST_FAIL
} MissionState_t;

static volatile MissionState_t mission_state = MISSION_READY;

typedef struct __attribute__((packed)) {
    float altitude;           // 4 bytes
    int16_t accel;            // 2 bytes (cm/s^2)
    int16_t speed;            // 2 bytes (cm/s)
    int16_t pitch, roll, yaw; // 6 bytes (DEG*10)
    uint8_t flight_state;     // 1 byte
    uint32_t timestamp;       // 4 bytes (us)
} TelemetryData_t; // Total: 19 bytes (MAX for BLE packet is 20 bytes) + 1 byte for RSSI on receiver side

TelemetryData_t telemetry;

typedef struct __attribute__((packed)) {
    float gx, gy, gz; // 12 bytes 
    float rx, ry, rz; // 12 bytes 
    uint32_t timestamp;       // 4 bytes (us)
} RAW_TelemetryData_t; // Total: 28 bytes + 1 byte for RSSI on receiver side

RAW_TelemetryData_t raw_telemetry;

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
            if (imu.ry > 300.0f){
                mission_state = MISSION_ASCENT;
                HAL_TIM_Base_Start_IT(&htim6); // Start 1 us timer for INS
                IMU_Fusion_SetGyro(&imu_integration, 0, 0, 0, Mission_GetTick()); // Set initial orientation to 0,0,0
                f_mount(&FatFs, "", 1);
                BUZZ(&hbuzz1, 100);
            }
            ground_pressure = prs;
            break;

        case MISSION_ASCENT:
            break;

        case MISSION_POST_FAIL:
            Mission_SafeMode();
            break;

        case MISSION_LANDED:
            HAL_TIM_Base_Stop(&htim6); 
            f_mount(&FatFs, "", 0);
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

void Mission_IncTick(void){
    overflow++;
}

uint32_t Mission_GetTick(void){
    uint32_t high;
    uint16_t low;

    do {
        high = overflow;
        low  = __HAL_TIM_GET_COUNTER(&htim6); // Read the current timer count (16-bit)
    } while (high != overflow);

    return (high << 16) | low;
}


void Mission_BuildTelemetryPacket(uint8_t *buf){
    telemetry.timestamp = Mission_GetTick();
    telemetry.pitch = imu_integration.pitch * 10.0f; // Convert to DEG*10
    telemetry.roll = imu_integration.roll * 10.0f;  // Convert to DEG*10
    telemetry.yaw = imu_integration.yaw * 10.0f;   // Convert to DEG*10

    memcpy(buf, &telemetry, sizeof(TelemetryData_t));
}

void Mission_BuildRAWTelemetryPacket(uint8_t *buf){
    raw_telemetry.timestamp = Mission_GetTick();
    raw_telemetry.gx = imu.gx;
    raw_telemetry.gy = imu.gy;
    raw_telemetry.gz = imu.gz;
    raw_telemetry.rx = imu.rx;
    raw_telemetry.ry = imu.ry;
    raw_telemetry.rz = imu.rz;

    memcpy(buf, &raw_telemetry, sizeof(RAW_TelemetryData_t));
}

void Mission_IMU_DRDY(void){
    if (mission_state == MISSION_ASCENT) {
        IMU_Fusion_IntegrateGyro(&imu_integration, Mission_GetTick());
    } 
}