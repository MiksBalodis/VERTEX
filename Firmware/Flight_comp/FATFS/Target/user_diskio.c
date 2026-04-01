/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   This file includes a diskio driver skeleton to be completed by the user.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
 /* USER CODE END Header */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/*
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future.
 * Kept to ensure backward compatibility with previous CubeMx versions when
 * migrating projects.
 * User code previously added there should be copied in the new user sections before
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ff_gen_drv.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Disk status */
#include "25flash.h"
static volatile DSTATUS Stat = STA_NOINIT;

#define BLOCK_SIZE      512     // FatFs logical block
#define SECTOR_SIZE     4096    // SPI flash erase sector
#define PAGE_SIZE       256     // Flash page size
#define BLOCKS_PER_SECTOR (SECTOR_SIZE / BLOCK_SIZE)
#define PAGES_PER_SECTOR  (SECTOR_SIZE / PAGE_SIZE)

#define _USE_WRITE 1

/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
  Stat = STA_NOINIT;
  MX25FLASH_Reset();
  HAL_Delay(1);
  // if(MX25FLASH_ReadID() == 0x00EF4015){Stat = 0;}
  Stat = 0;
  return Stat;
  /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
  // Stat = STA_NOINIT;
   Stat = 0;
    
    return Stat;
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
  UNUSED(pdrv);
  MX25FLASH_Continious_Read(sector * 512, buff, count * 512);
    return RES_OK;
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{
  /* USER CODE BEGIN WRITE */
   static uint8_t sector_buf[4096];
  uint32_t sectors_processed = 0;

  while (sectors_processed < count)
  {
    uint32_t current_lba = sector + sectors_processed;
    
    uint32_t flash_sector_idx = current_lba / 8; // 8 LBA blocks (512b) per 4KB sector
    uint32_t flash_sector_addr = flash_sector_idx * 4096;
    
    uint32_t lba_offset_in_sector = (current_lba % 8) * 512;

    uint32_t blocks_to_copy = 8 - (current_lba % 8);
    if (blocks_to_copy > (count - sectors_processed)) {
        blocks_to_copy = count - sectors_processed;
    }

    MX25FLASH_Continious_Read(flash_sector_addr, sector_buf, 4096);

    memcpy(sector_buf + lba_offset_in_sector, 
           buff + (sectors_processed * 512), 
           blocks_to_copy * 512);

    _MX25FLASH_Sector_Erase(flash_sector_idx);

    for (uint16_t page = 0; page < 16; page++){
        uint32_t page_idx = (flash_sector_idx * 16) + page;
        MX25FLASH_Program_Page(page_idx, sector_buf + (page * 256));

    }

    sectors_processed += blocks_to_copy;
  }
  /* USER CODE HERE */
  return RES_OK;
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
DRESULT res = RES_ERROR;
    switch(cmd)
    {
        case CTRL_SYNC:
            // Wait until flash is ready
            MX25FLASH_WFE();
            res = RES_OK;
            break;

        case GET_SECTOR_COUNT:
            // Total number of 512-byte blocks
            *(DWORD*)buff = (2*1024*1024) / 512; // 2 MB flash
            res = RES_OK;
            break;

        case GET_SECTOR_SIZE:
            *(WORD*)buff = 512; // logical block size
            res = RES_OK;
            break;

        case GET_BLOCK_SIZE:
            // Erase block size in units of sector
            *(DWORD*)buff = 8; // 4 KB sector / 512 B block
            res = RES_OK;
            break;

        default:
            res = RES_PARERR;
    }
    return res;
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

