#include "mission.h"
#include "imu_fusion.h"
#include "main.h"
#include "buzzer.h"
#include "neopixel.h"
#include "BMP388.h"
#include "fatfs.h"
#include "sensor_filter.h"
#include "gps.h"

#include <string.h>
#include <stdbool.h>

extern Buzzer_Handle hbuzz1;
extern TIM_HandleTypeDef htim6;

extern gngga_t gps;

static LowPassFilter_t altitude_filter;
static LowPassFilter_t pressure_filter;
static LowPassFilter_t vertical_speed_filter;
static GpsFilter_t gps_filter;

static float filtered_gps_lat = 0.0f;
static float filtered_gps_lon = 0.0f;

static float previous_altitude_m = 0.0f;
static uint32_t previous_altitude_time_ms = 0;
static float vertical_speed_mps = 0.0f;

#define LAUNCH_ACCEL_THRESHOLD_MG      300.0f
#define LAUNCH_CONFIRMATION_SAMPLES   3

static uint8_t launch_confirm_counter = 0;

float ground_pressure;

IMU_Data_t imu;
IMU_Integration_t imu_integration;
extern BMP388_HandleTypeDef hbmp388;
extern FATFS FatFs;
extern volatile Quaternion quat;

volatile uint32_t overflow;

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
    float altitude;
    int16_t accel;
    int16_t speed;
    int16_t pitch, roll, yaw;
    uint8_t flight_state;
    uint32_t timestamp;
} TelemetryData_t;

TelemetryData_t telemetry;

typedef struct __attribute__((packed)) {
    float w, x, y, z;
    float rx, ry, rz;
    uint8_t flight_state;
    float altitude;
    uint32_t timestamp;
    float lat;
    float lon;
    float vertical_speed;
} RAW_TelemetryData_t; // Total: 49 bytes

RAW_TelemetryData_t raw_telemetry;

bool POST_fault_flags[FAULT_MAX] = {0};

static int16_t clamp_float_to_int16(float value)
{
    if (value > 32767.0f) return 32767;
    if (value < -32768.0f) return -32768;

    return (int16_t)value;
}

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
    raw_telemetry.flight_state = mission_state;

    LowPass_Init(&altitude_filter, 0.15f);
    LowPass_Init(&pressure_filter, 0.10f);
    LowPass_Init(&vertical_speed_filter, 0.25f);
    GpsFilter_Init(&gps_filter);

    filtered_gps_lat = 0.0f;
    filtered_gps_lon = 0.0f;

    previous_altitude_m = 0.0f;
    previous_altitude_time_ms = 0;
    vertical_speed_mps = 0.0f;

    launch_confirm_counter = 0;
}

void Mission_Update(void)
{
    uint32_t rprs, rtemp, time;
    float prs, temp;

    IMU_Fusion_Update(&imu);

    BMP388_ReadRawPressTempTime(&hbmp388, &rprs, &rtemp, &time);
    BMP388_CompensateRawPressTemp(&hbmp388, rprs, rtemp, &prs, &temp);

    telemetry.flight_state = mission_state;
    raw_telemetry.flight_state = mission_state;

    /*
      Pressure / altitude filtering.
    */

    float prs_filtered = LowPass_Update(&pressure_filter, prs);

    if (mission_state == MISSION_READY)
    {
        ground_pressure = prs_filtered;
    }

    float altitude_raw = BMP388_FindAltitude(ground_pressure, prs_filtered);
    float altitude_filtered = LowPass_Update(&altitude_filter, altitude_raw);

    telemetry.altitude = altitude_filtered;
    raw_telemetry.altitude = altitude_filtered;

    uint32_t altitude_time_ms = HAL_GetTick();

    if (previous_altitude_time_ms != 0)
    {
        uint32_t dt_ms = altitude_time_ms - previous_altitude_time_ms;

        if (dt_ms > 0)
        {
            float dt_s = dt_ms / 1000.0f;
            float vertical_speed_raw_mps = (altitude_filtered - previous_altitude_m) / dt_s;

            vertical_speed_mps = LowPass_Update(
                &vertical_speed_filter,
                vertical_speed_raw_mps
            );
        }
    }

    previous_altitude_m = altitude_filtered;
    previous_altitude_time_ms = altitude_time_ms;

    /*
    telemetry.speed is stored as cm/s.
    vertical_speed_mps is m/s.
    m/s * 100 = cm/s.
    */
    telemetry.speed = clamp_float_to_int16(vertical_speed_mps * 100.0f);

    /*
      GPS filtering.
    */
    float gps_lat;
    float gps_lon;

    if (GpsFilter_Update(
            &gps_filter,
            gps.lat,
            gps.lon,
            0.25f,
            75.0f,
            &gps_lat,
            &gps_lon
        ))
    {
        filtered_gps_lat = gps_lat;
        filtered_gps_lon = gps_lon;
    }

    switch (mission_state)
    {
        case MISSION_READY:
            /*
            Launch confirmation
            */
            if (imu.ry > LAUNCH_ACCEL_THRESHOLD_MG)
            {
                if (launch_confirm_counter < LAUNCH_CONFIRMATION_SAMPLES)
                {
                    launch_confirm_counter++;
                }
            }
            else
            {
                launch_confirm_counter = 0;
            }

            if (launch_confirm_counter >= LAUNCH_CONFIRMATION_SAMPLES)
            {
                mission_state = MISSION_ASCENT;
                launch_confirm_counter = 0;

                HAL_TIM_Base_Start_IT(&htim6); // Start 1 us timer for INS

                IMU_Fusion_SetGyro(
                    &imu_integration,
                    0,
                    0,
                    0,
                    Mission_GetTick()
                );

                f_mount(&FatFs, "", 1);
                BUZZ(&hbuzz1, 100);
            }

            ground_pressure = prs_filtered;
            break;

        case MISSION_ASCENT:
            break;

        case MISSION_DESCENT:
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

    while(1){
        for (uint8_t fault = 0; fault < FAULT_MAX; fault++){
            if(POST_fault_flags[fault]){
                for(uint8_t i = 0; i <= fault; i++){
                    LED_Set_Color(64, 0, 0);
                    HAL_Delay(500);
                    LED_Set_Color(0, 0, 0);
                    HAL_Delay(500);
                }
                HAL_Delay(1500);
            }
        }
    }
}

void Mission_IncTick(void){
    overflow++;
}

uint32_t Mission_GetTick(void){
    uint32_t high;
    uint16_t low;

    do {
        high = overflow;
        low  = __HAL_TIM_GET_COUNTER(&htim6);
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
    raw_telemetry.w = quat.w;
    raw_telemetry.x = quat.x;
    raw_telemetry.y = quat.y;
    raw_telemetry.z = quat.z;

    raw_telemetry.rx = imu.rx;
    raw_telemetry.ry = imu.ry;
    raw_telemetry.rz = imu.rz;

    raw_telemetry.flight_state = mission_state;
    raw_telemetry.altitude = telemetry.altitude;

    raw_telemetry.lat = filtered_gps_lat;
    raw_telemetry.lon = filtered_gps_lon;

    raw_telemetry.vertical_speed = vertical_speed_mps;

    memcpy(buf, &raw_telemetry, sizeof(RAW_TelemetryData_t));

}

void Mission_IMU_DRDY(void){
    if (mission_state == MISSION_ASCENT) {
        IMU_Fusion_IntegrateGyro(&imu_integration, Mission_GetTick());
    }
}
