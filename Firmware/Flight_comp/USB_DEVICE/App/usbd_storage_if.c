/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_storage_if.c
  * @version        : v1.0_Cube
  * @brief          : Memory management layer.
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

/* Includes ------------------------------------------------------------------*/
#include "usbd_storage_if.h"

/* USER CODE BEGIN INCLUDE */
#include "25flash.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define USB_BLOCK_SIZE    512
#define FLASH_SECTOR_SIZE 4096
#define BLOCKS_PER_SECTOR (FLASH_SECTOR_SIZE / USB_BLOCK_SIZE) // 8
#define TOTAL_USB_BLOCKS  0x1000 // 4096 blocks for 2MB
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @defgroup USBD_STORAGE
  * @brief Usb mass storage device module
  * @{
  */

/** @defgroup USBD_STORAGE_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Defines
  * @brief Private defines.
  * @{
  */

#define STORAGE_LUN_NBR                  1
#define STORAGE_BLK_NBR                  0x10000
#define STORAGE_BLK_SIZ                  0x200

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Variables
  * @brief Private variables.
  * @{
  */

/* USER CODE BEGIN INQUIRY_DATA_FS */
/** USB Mass storage Standard Inquiry Data. */
const int8_t STORAGE_Inquirydata_FS[] = {/* 36 */

  /* LUN 0 */
  0x00,
  0x80,
  0x02,
  0x02,
  (STANDARD_INQUIRY_DATA_LEN - 5),
  0x00,
  0x00,
  0x00,
  'S', 'T', 'M', ' ', ' ', ' ', ' ', ' ', /* Manufacturer : 8 bytes */
  'P', 'r', 'o', 'd', 'u', 'c', 't', ' ', /* Product      : 16 Bytes */
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  '0', '.', '0' ,'1'                      /* Version      : 4 Bytes */
};
/* USER CODE END INQUIRY_DATA_FS */

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t STORAGE_Init_FS(uint8_t lun);
static int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size);
static int8_t STORAGE_IsReady_FS(uint8_t lun);
static int8_t STORAGE_IsWriteProtected_FS(uint8_t lun);
static int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_GetMaxLun_FS(void);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_StorageTypeDef USBD_Storage_Interface_fops_FS =
{
  STORAGE_Init_FS,
  STORAGE_GetCapacity_FS,
  STORAGE_IsReady_FS,
  STORAGE_IsWriteProtected_FS,
  STORAGE_Read_FS,
  STORAGE_Write_FS,
  STORAGE_GetMaxLun_FS,
  (int8_t *)STORAGE_Inquirydata_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the storage unit (medium) over USB FS IP
  * @param  lun: Logical unit number.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Init_FS(uint8_t lun)
{
  /* USER CODE BEGIN 2 */
  UNUSED(lun);

  return (USBD_OK);
  /* USER CODE END 2 */
}

/**
  * @brief  Returns the medium capacity.
  * @param  lun: Logical unit number.
  * @param  block_num: Number of total block number.
  * @param  block_size: Block size.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
  /* USER CODE BEGIN 3 */
  UNUSED(lun);

  *block_num  = STORAGE_BLK_NBR;
  *block_size = STORAGE_BLK_SIZ;
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief   Checks whether the medium is ready.
  * @param  lun:  Logical unit number.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_IsReady_FS(uint8_t lun)
{
  /* USER CODE BEGIN 4 */
  UNUSED(lun);

  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Checks whether the medium is write protected.
  * @param  lun: Logical unit number.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_IsWriteProtected_FS(uint8_t lun)
{
  /* USER CODE BEGIN 5 */
  UNUSED(lun);

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Reads data from the medium.
  * @param  lun: Logical unit number.
  * @param  buf: data buffer.
  * @param  blk_addr: Logical block address.
  * @param  blk_len: Blocks number.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  /* USER CODE BEGIN 6 */
  // UNUSED(lun);
  // // UNUSED(buf);
  // // UNUSED(blk_addr);
  // UNUSED(blk_len);

  // MX25FLASH_Read_LogBlock(blk_addr, buf);

  // return (USBD_OK);

      UNUSED(lun);

    uint8_t temp[FLASH_SECTOR_SIZE];

    #define BLOCKS_PER_SECTOR (FLASH_SECTOR_SIZE / USB_BLOCK_SIZE)

    while (blk_len > 0)
    {
        uint32_t sector_num = blk_addr / BLOCKS_PER_SECTOR;
        uint32_t block_index = blk_addr % BLOCKS_PER_SECTOR;

        // Number of USB blocks to read in this sector
        uint32_t blocks_to_read = blk_len;
        if (blocks_to_read > BLOCKS_PER_SECTOR - block_index)
            blocks_to_read = BLOCKS_PER_SECTOR - block_index;

        uint32_t offset = block_index * USB_BLOCK_SIZE;

        // 1. Read full flash sector
        MX25FLASH_Read_Sector(sector_num, temp);

        // 2. Copy requested USB blocks to output buffer
        memcpy(buf, &temp[offset], blocks_to_read * USB_BLOCK_SIZE);

        // Advance pointers/counters
        buf += blocks_to_read * USB_BLOCK_SIZE;
        blk_addr += blocks_to_read;
        blk_len -= blocks_to_read;
    }

    return USBD_OK;
  /* USER CODE END 6 */
}

/**
  * @brief  Writes data into the medium.
  * @param  lun: Logical unit number.
  * @param  buf: data buffer.
  * @param  blk_addr: Logical block address.
  * @param  blk_len: Blocks number.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  /* USER CODE BEGIN 7 */
  // UNUSED(lun);
  // // UNUSED(buf);
  // // UNUSED(blk_addr);
  // UNUSED(blk_len);

  // uint8_t temp[4096];
  // uint8_t sector_num = blk_addr / (4096 / blk_len);
  // uint32_t block_index_in_sector = blk_addr % 8;
  // uint32_t offset = block_index_in_sector * blk_len;
  
  // MX25FLASH_Read_Sector(sector_num, temp);

  // memcpy(&temp[offset], buf, blk_len);

  // if (MX25FLASH_Sector_Erase(sector_num) == 1) return (USBD_FAIL);
  // if (MX25FLASH_Program_Sector(sector_num, temp) == 1) return (USBD_FAIL);

  // return (USBD_OK);

      UNUSED(lun);

    uint8_t temp[FLASH_SECTOR_SIZE];

    #define BLOCKS_PER_SECTOR (FLASH_SECTOR_SIZE / USB_BLOCK_SIZE)

    while (blk_len > 0)
    {
        uint32_t sector_num = blk_addr / BLOCKS_PER_SECTOR;
        uint32_t block_index = blk_addr % BLOCKS_PER_SECTOR;
        uint32_t blocks_to_write = blk_len;

        // Limit to current sector
        if (blocks_to_write > BLOCKS_PER_SECTOR - block_index)
            blocks_to_write = BLOCKS_PER_SECTOR - block_index;

        uint32_t offset = block_index * USB_BLOCK_SIZE;

        // 1. Read full sector
        MX25FLASH_Read_Sector(sector_num, temp);

        // 2. Copy USB blocks into buffer
        memcpy(&temp[offset], buf, blocks_to_write * USB_BLOCK_SIZE);

        // 3. Erase sector
        if (MX25FLASH_Sector_Erase(sector_num) != 0) return USBD_FAIL;

        // 4. Program sector
        if (MX25FLASH_Program_Sector(sector_num, temp) != 0) return USBD_FAIL;

        // Advance
        buf += blocks_to_write * USB_BLOCK_SIZE;
        blk_addr += blocks_to_write;
        blk_len -= blocks_to_write;
    }

    return USBD_OK;
  /* USER CODE END 7 */
}

/**
  * @brief  Returns the Max Supported LUNs.
  * @param  None
  * @retval Lun(s) number.
  */
int8_t STORAGE_GetMaxLun_FS(void)
{
  /* USER CODE BEGIN 8 */
  return (STORAGE_LUN_NBR - 1);
  /* USER CODE END 8 */
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

