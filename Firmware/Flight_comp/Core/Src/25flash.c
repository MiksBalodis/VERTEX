#include "25flash.h"

void MX25FLASH_SPI_Write (uint8_t *data, uint16_t len){
	HAL_SPI_Transmit(&FLASH_SPI, data, len, 2000);
}

HAL_StatusTypeDef MX25FLASH_SPI_Read (uint8_t *data, uint16_t len){
	return HAL_SPI_Receive(&FLASH_SPI, data, len, 5000);
}

void MX25FLASH_Reset(void){
	uint8_t tData[2];

	tData[0] = 0x66;  // enable Reset
	tData[1] = 0x99;  // Reset
    FLASH_CS_LOW();
	MX25FLASH_SPI_Write(tData, 2);
	FLASH_CS_HIGH(); // pull the HIGH
    HAL_Delay(100);
}

uint32_t MX25FLASH_ReadID(void){
	uint8_t tData = 0x9f;  // Read ID
	uint8_t rData[3];
	FLASH_CS_LOW();  // pull the CS LOW
	MX25FLASH_SPI_Write(&tData, 1);
	MX25FLASH_SPI_Read(rData, 3);
	FLASH_CS_HIGH();  // pull the HIGH
	return ((rData[0]<<16)|(rData[1]<<8)|rData[2]); // MFN ID : MEM ID : CAPACITY ID
}

uint8_t MX25FLASH_Read(uint32_t address){
    uint8_t tData[4];
	tData[0] = 0x03;
    tData[1] = (address >> 16) & 0xFF;  // Most significant byte
    tData[2] = (address >> 8) & 0xFF;
    tData[3] = address & 0xFF;          // Least significant byte
	uint8_t rData[1];
	FLASH_CS_LOW();  // pull the CS LOW
    MX25FLASH_SPI_Write(tData, 4);
	MX25FLASH_SPI_Read(rData, 1);
	FLASH_CS_HIGH();  // pull the HIGH
    return rData[0];
}

void MX25FLASH_Program_Page(uint16_t page, uint8_t data[256]){
	MX25FLASH_WE();
    uint8_t tData[260];
	uint32_t address = page * 0x100;
	tData[0] = 0x02;
    tData[1] = (address >> 16) & 0xFF;  // Most significant byte
    tData[2] = (address >> 8) & 0xFF;
    tData[3] = address & 0xFF;          // Least significant byte
	memcpy(&tData[4], data, 256); 
	FLASH_CS_LOW();  // pull the CS LOW
    MX25FLASH_SPI_Write(tData, 260);
	FLASH_CS_HIGH();  // pull the HIGH
	MX25FLASH_WFE();
}

void MX25FLASH_WE(void){
    uint8_t cmd = 0x06;
    FLASH_CS_LOW();
    MX25FLASH_SPI_Write(&cmd, 1);
    FLASH_CS_HIGH();
}

uint8_t MX25FLASH_Sector_Erase(uint32_t sector){
	MX25FLASH_WE();
    uint8_t cmd[4];
    cmd[0] = 0x20;  // 4KB Sector Erase
    cmd[1] = (sector >> 16) & 0xFF;
    cmd[2] = (sector >> 8) & 0xFF;
    cmd[3] = sector & 0xFF;

    FLASH_CS_LOW();
    MX25FLASH_SPI_Write(cmd, 4);
    FLASH_CS_HIGH();
	return MX25FLASH_WFE();
}

uint8_t MX25FLASH_Program_Sector(uint32_t sector, uint8_t *data){
    MX25FLASH_WE();
    for (uint8_t pg = 0; pg < 16; pg++){
        uint8_t tData[260];
        uint32_t address = sector*0x1000 + pg * 0x100;
        tData[0] = 0x02;
        tData[1] = (address >> 16) & 0xFF;  // Most significant byte
        tData[2] = (address >> 8) & 0xFF;
        tData[3] = address & 0xFF;          // Least significant byte
        memcpy(&tData[4], &data[pg*256], 256); 
        FLASH_CS_LOW();  // pull the CS LOW
        MX25FLASH_SPI_Write(tData, 260);
        FLASH_CS_HIGH();  // pull the HIGH
    }
    return MX25FLASH_WFE();
}

void MX25FLASH_Read_LogBlock(uint32_t sector, uint8_t *data){
    for (uint32_t addr = 0; addr < 512; addr++){
        data[addr] = MX25FLASH_Read(sector*0x200+addr);
    }
}

void MX25FLASH_Read_Sector(uint32_t sector, uint8_t *data){
    for (uint32_t addr = 0; addr < 4096; addr++){
        data[addr] = MX25FLASH_Read(sector*0x1000+addr);
    }
}


uint8_t MX25FLASH_WFE(void){
    uint8_t cmd = 0x05;
	uint32_t tmr = HAL_GetTick();
    uint8_t status;
    do {
        FLASH_CS_LOW();
        MX25FLASH_SPI_Write(&cmd, 1);
        MX25FLASH_SPI_Read(&status, 1);
        FLASH_CS_HIGH();
		if(HAL_GetTick() - tmr > FLASH_TIMEOUT){
			return FL_FAIL;
		}
    } while (status & 0x01);  // Wait while WIP bit is set
    return FL_OK;
}