#ifndef MISSION_H
#define MISSION_H

#include <stdbool.h>
#include <stdint.h>

#define MIN_FLASH_SPACE_KB  1024

// Power-on self-test
typedef enum
{
    EEPROM_Comm_Fail = 0,
    IMU_Comm_Fail,
    PRS_Comm_Fail,
    FLASH_Comm_Fail,
    GNSS_Comm_Fail,
    LORA_Comm_Fail,
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
void Mission_IMU_DRDY(void);

#endif
