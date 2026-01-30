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

#define FL_OK    0U
#define FL_FAIL  1U

void MX25FLASH_SPI_Write(uint8_t *data, uint16_t len);
HAL_StatusTypeDef MX25FLASH_SPI_Read(uint8_t *data, uint16_t len);

void MX25FLASH_Reset(void);
uint32_t MX25FLASH_ReadID(void);
uint8_t MX25FLASH_Read(uint32_t address);
void MX25FLASH_Read_Sector(uint32_t sector, uint8_t *data);
void MX25FLASH_Program_Page(uint16_t page, uint8_t data[256]);
uint8_t MX25FLASH_Sector_Erase(uint32_t sector);
uint8_t MX25FLASH_Program_Sector(uint32_t sector, uint8_t *data);
void MX25FLASH_Read_LogBlock(uint32_t sector, uint8_t *data);

uint8_t MX25FLASH_WFE(void);
void MX25FLASH_WE(void);