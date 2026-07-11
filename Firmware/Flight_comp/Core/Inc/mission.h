#ifndef MISSION_H
#define MISSION_H

#include <stdbool.h>
#include <stdint.h>

#define MIN_FLASH_SPACE_KB  1024

#define MAX_ASCENT_TIME_S  10
#define MAX_DECENT_TIME_S  50

#define IDLE_TIMEOUT_S     60

#define FEEFALL_TIME_S  0

#define LANDING_DETECTION_ALTITUDE_M  1.0f
#define DECENT_DETECTION_ALTITUDE_M  5.0f

// Power-on self-test
typedef enum
{
    EEPROM_Comm_Fail = 0,
    IMU_Comm_Fail,
    PRS_Comm_Fail,
    FLASH_Comm_Fail,
    GNSS_Comm_Fail,
    LoRa_Comm_Fail,
    Vbat_UVLO,
    FS_Not_Found,
    FS_No_Space,
    FAULT_MAX  // Only to make an array
} POST_t;

void Mission_Init(void);
void Mission_Update(void);
void Mission_SafeMode(void);
void Mission_IncTick(void);
uint32_t Mission_GetTick(void);
void Mission_BuildTelemetryPacket(uint8_t *buf);
void Mission_BuildRAWTelemetryPacket(uint8_t *buf);

/* Fixed-rate control loop (TIM3 ISR). Owns SPI2 / the IMU. */
void Mission_ControlTick(void);
void Mission_StartControlLoop(void);
void Mission_StopControlLoop(void);
float Mission_GetGravityRefMg(void);
void Mission_SaveTelemetry(void);

#endif
