/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CAM_CTRL_Pin GPIO_PIN_0
#define CAM_CTRL_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_3
#define BUZZER_GPIO_Port GPIOC
#define SERVO1_Pin GPIO_PIN_0
#define SERVO1_GPIO_Port GPIOA
#define SERVO2_Pin GPIO_PIN_1
#define SERVO2_GPIO_Port GPIOA
#define SERVO3_Pin GPIO_PIN_2
#define SERVO3_GPIO_Port GPIOA
#define SERVO4_Pin GPIO_PIN_3
#define SERVO4_GPIO_Port GPIOA
#define BAT_VSENCE_Pin GPIO_PIN_4
#define BAT_VSENCE_GPIO_Port GPIOA
#define FLASH_SCK_Pin GPIO_PIN_5
#define FLASH_SCK_GPIO_Port GPIOA
#define FLASH_MISO_Pin GPIO_PIN_6
#define FLASH_MISO_GPIO_Port GPIOA
#define FLASH_MOSI_Pin GPIO_PIN_7
#define FLASH_MOSI_GPIO_Port GPIOA
#define FLASH_NSS_Pin GPIO_PIN_4
#define FLASH_NSS_GPIO_Port GPIOC
#define FLASH_WP_Pin GPIO_PIN_5
#define FLASH_WP_GPIO_Port GPIOC
#define LORA_DIO1_Pin GPIO_PIN_0
#define LORA_DIO1_GPIO_Port GPIOB
#define LORA_DIO1_EXTI_IRQn EXTI0_IRQn
#define LORA_IO0_Pin GPIO_PIN_1
#define LORA_IO0_GPIO_Port GPIOB
#define BMP_INT_Pin GPIO_PIN_2
#define BMP_INT_GPIO_Port GPIOB
#define BMP_INT_EXTI_IRQn EXTI2_IRQn
#define BMP_SCL_Pin GPIO_PIN_10
#define BMP_SCL_GPIO_Port GPIOB
#define BMP_SDA_Pin GPIO_PIN_11
#define BMP_SDA_GPIO_Port GPIOB
#define IMU_NSS_Pin GPIO_PIN_12
#define IMU_NSS_GPIO_Port GPIOB
#define IMU_SCK_Pin GPIO_PIN_13
#define IMU_SCK_GPIO_Port GPIOB
#define IMU_MISO_Pin GPIO_PIN_14
#define IMU_MISO_GPIO_Port GPIOB
#define IMU_MOSI_Pin GPIO_PIN_15
#define IMU_MOSI_GPIO_Port GPIOB
#define IMU_INT1_Pin GPIO_PIN_6
#define IMU_INT1_GPIO_Port GPIOC
#define IMU_INT1_EXTI_IRQn EXTI9_5_IRQn
#define PYRO1_Pin GPIO_PIN_7
#define PYRO1_GPIO_Port GPIOC
#define PYRO2_Pin GPIO_PIN_8
#define PYRO2_GPIO_Port GPIOC
#define GPS_SDA_Pin GPIO_PIN_9
#define GPS_SDA_GPIO_Port GPIOC
#define GPS_SCL_Pin GPIO_PIN_8
#define GPS_SCL_GPIO_Port GPIOA
#define GPS_TX_Pin GPIO_PIN_9
#define GPS_TX_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_10
#define GPS_RX_GPIO_Port GPIOA
#define LORA_NSS_Pin GPIO_PIN_15
#define LORA_NSS_GPIO_Port GPIOA
#define LORA_SCK_Pin GPIO_PIN_10
#define LORA_SCK_GPIO_Port GPIOC
#define LORA_MISO_Pin GPIO_PIN_11
#define LORA_MISO_GPIO_Port GPIOC
#define LORA_MOSI_Pin GPIO_PIN_12
#define LORA_MOSI_GPIO_Port GPIOC
#define LORA_NRST_Pin GPIO_PIN_2
#define LORA_NRST_GPIO_Port GPIOD
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOB
#define EEP_WP_Pin GPIO_PIN_5
#define EEP_WP_GPIO_Port GPIOB
#define EEP_SCL_Pin GPIO_PIN_6
#define EEP_SCL_GPIO_Port GPIOB
#define EEP_SDA_Pin GPIO_PIN_7
#define EEP_SDA_GPIO_Port GPIOB
#define SERVO5_Pin GPIO_PIN_8
#define SERVO5_GPIO_Port GPIOB
#define SERVO6_Pin GPIO_PIN_9
#define SERVO6_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
