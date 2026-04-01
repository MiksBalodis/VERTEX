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

// #define STORAGE_LUN_NBR                  1
// #define STORAGE_BLK_NBR                  0x10000
// #define STORAGE_BLK_SIZ                  0x200

/* USER CODE BEGIN PRIVATE_DEFINES */

#define STORAGE_LUN_NBR                  1
#define STORAGE_BLK_NBR                  0x4000
#define STORAGE_BLK_SIZ                  0x200

#define STORAGE_PG_NBR                   0x2000
#define STORAGE_PG_SIZ                   0x100
#define STORAGE_SEC_NBR                  0x200
#define STORAGE_SEC_SIZ                  0x1000
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
  'R', 'T', 'U', ' ', 'I', 'Z', 'V', ' ', /* Manufacturer : 8 bytes */
  'V', 'E', 'R', 'T', 'E', 'X', ' ', ' ', /* Product      : 16 Bytes */
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
  UNUSED(lun);

  uint32_t blk_start_addr = blk_addr * STORAGE_BLK_SIZ;
  uint32_t offset = blk_len * STORAGE_BLK_SIZ;

  MX25FLASH_Continious_Read(blk_start_addr, buf, offset);

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
// /* W25Q16 Constants */
//   #define STORAGE_BLK_SIZ 0x200   // 512 bytes (USB Block)
//   #define SECTOR_SIZE     0x1000  // 4096 bytes (Flash Sector)
//   #define PAGE_SIZE       0x100   // 256 bytes (Flash Page)

//   // Use a static buffer to avoid stack overflow (4KB is a lot for STM32 stack)
//   static uint8_t sector_scratchpad[SECTOR_SIZE];

//   uint32_t blocks_written = 0;

//   while (blocks_written < blk_len)
//   {
//     uint32_t current_blk_addr = blk_addr + blocks_written;
    
//     // 1. Find which 4KB sector the current block lives in
//     uint32_t sector_num = current_blk_addr / 8; // 8 blocks per sector (4096/512)
//     uint32_t sector_addr = sector_num * SECTOR_SIZE;
    
//     // 2. Find the offset of our block inside that 4KB sector
//     uint32_t block_offset_in_sector = (current_blk_addr % 8) * STORAGE_BLK_SIZ;

//     // 3. READ: Get the existing 4KB data from Flash into RAM
//     MX25FLASH_Continious_Read(sector_addr, sector_scratchpad, SECTOR_SIZE);

//     // 4. MODIFY: Overwrite only the 512 bytes Windows provided
//     // We can optimize this to write multiple blocks if they are in the same sector
//     uint32_t blocks_to_copy = 8 - (current_blk_addr % 8);
//     if (blocks_to_copy > (blk_len - blocks_written)) blocks_to_copy = blk_len - blocks_written;

//     memcpy(sector_scratchpad + block_offset_in_sector, 
//            buf + (blocks_written * STORAGE_BLK_SIZ), 
//            blocks_to_copy * STORAGE_BLK_SIZ);

//     // 5. ERASE: Clear the physical sector on the chip
//     _MX25FLASH_Sector_Erase(sector_num);

//     // 6. WRITE: Program the 4KB back into the chip, page by page (16 pages)
//     for (uint16_t i = 0; i < 16; i++) {
//       MX25FLASH_Program_Page((sector_num * 16) + i, sector_scratchpad + (i * PAGE_SIZE));
//       // Since your new Program_Page calls WFE(), this is now safe!
//     }

//     blocks_written += blocks_to_copy;
//   }

//   return USBD_OK;


static uint8_t sector_scratchpad[4096];
  static uint8_t original_flash_data[4096]; // To compare changes
  uint32_t blocks_written = 0;

  while (blocks_written < blk_len) {
    uint32_t curr_blk = blk_addr + blocks_written;
    uint32_t sector_num = curr_blk / 8;
    uint32_t sector_addr = sector_num * 4096;

    // Read current state
    MX25FLASH_Continious_Read(sector_addr, original_flash_data, 4096);
    memcpy(sector_scratchpad, original_flash_data, 4096);

    // Modify scratchpad with new data
    uint32_t offset = (curr_blk % 8) * 512;
    uint32_t blocks_to_copy = 8 - (curr_blk % 8);
    if (blocks_to_copy > (blk_len - blocks_written)) blocks_to_copy = blk_len - blocks_written;
    
    memcpy(sector_scratchpad + offset, buf + (blocks_written * 512), blocks_to_copy * 512);

    // Check if data actually changed
    if (memcmp(sector_scratchpad, original_flash_data, 4096) == 0) {
        blocks_written += blocks_to_copy;
        continue; // Skip Erase/Write if data is the same
    }

    // Check if we even need to erase
    bool needs_erase = false;
    for (int i = 0; i < 4096; i++) {
        if (original_flash_data[i] != 0xFF) {
            needs_erase = true;
            break;
        }
    }

    if (needs_erase) {
        _MX25FLASH_Sector_Erase(sector_num);
    }

    // Write back
    for (uint16_t i = 0; i < 16; i++) {
      MX25FLASH_Program_Page((sector_num * 16) + i, sector_scratchpad + (i * 256));
    }

    blocks_written += blocks_to_copy;
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

