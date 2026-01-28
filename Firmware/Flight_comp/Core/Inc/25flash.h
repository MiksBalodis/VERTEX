#pragma once

#include "main.h"
#include "stdint.h" 
#include "string.h"

extern SPI_HandleTypeDef hspi1;
#define FLASH_SPI   hspi1

#define FLASH_TIMEOUT 500

#define FLASH_CS_PORT    FLASH_NSS_GPIO_Port
#define FLASH_CS_PIN     FLASH_NSS_Pin

#define FLASH_CS_LOW()   HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_RESET)
#define FLASH_CS_HIGH()  HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET)

void FLASH_SPI_Write(uint8_t *data, uint16_t len);
void FLASH_SPI_Read(uint8_t *data, uint16_t len);

void FLASH_Reset(void);
uint32_t FLASH_ReadID(void);
uint8_t FLASH_Read(uint32_t address);
void FLASH_Read_Sector(uint32_t sector, uint8_t *data[4096]);
void FLASH_Program_Page(uint16_t page, uint8_t data[256]);
void FLASH_Sector_Erase(uint32_t sector);
void FLASH_Program_Sector(uint32_t sector, uint8_t data[4096]);

void FLASH_WFE(void);
void FLASH_WE(void);