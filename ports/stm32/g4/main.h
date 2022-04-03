/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define nALERT_FAN_Pin GPIO_PIN_2
#define nALERT_FAN_GPIO_Port GPIOE
#define SEL_SONIC0_Pin GPIO_PIN_3
#define SEL_SONIC0_GPIO_Port GPIOE
#define SEL_SONIC1_Pin GPIO_PIN_4
#define SEL_SONIC1_GPIO_Port GPIOE
#define EN_12V0_LS_3_Pin GPIO_PIN_5
#define EN_12V0_LS_3_GPIO_Port GPIOE
#define BLUE_Pin GPIO_PIN_6
#define BLUE_GPIO_Port GPIOE
#define EN_12V0_LOAD1_Pin GPIO_PIN_13
#define EN_12V0_LOAD1_GPIO_Port GPIOC
#define FDCAN1_STB_Pin GPIO_PIN_2
#define FDCAN1_STB_GPIO_Port GPIOC
#define EMERGENCY_Pin GPIO_PIN_3
#define EMERGENCY_GPIO_Port GPIOC
#define nEMERGENCY_Pin GPIO_PIN_2
#define nEMERGENCY_GPIO_Port GPIOF
#define EN_MD_PWR_Pin GPIO_PIN_2
#define EN_MD_PWR_GPIO_Port GPIOA
#define nEN_PC_DIO_Pin GPIO_PIN_3
#define nEN_PC_DIO_GPIO_Port GPIOA
#define SPI1_CS2_Pin GPIO_PIN_4
#define SPI1_CS2_GPIO_Port GPIOA
#define EN_I2C_PWR_Pin GPIO_PIN_4
#define EN_I2C_PWR_GPIO_Port GPIOC
#define SENSOR_GPO_4_BUFF_Pin GPIO_PIN_5
#define SENSOR_GPO_4_BUFF_GPIO_Port GPIOC
#define SENSOR_GPO_3_BUFF_Pin GPIO_PIN_0
#define SENSOR_GPO_3_BUFF_GPIO_Port GPIOB
#define SENSOR_GPO_2_BUFF_Pin GPIO_PIN_1
#define SENSOR_GPO_2_BUFF_GPIO_Port GPIOB
#define SENSOR_GPO_1_BUFF_Pin GPIO_PIN_2
#define SENSOR_GPO_1_BUFF_GPIO_Port GPIOB
#define SPI1_CS1_Pin GPIO_PIN_7
#define SPI1_CS1_GPIO_Port GPIOE
#define RED_Pin GPIO_PIN_9
#define RED_GPIO_Port GPIOE
#define FDCAN2_STB_Pin GPIO_PIN_15
#define FDCAN2_STB_GPIO_Port GPIOB
#define nINVALID_RS232_Pin GPIO_PIN_8
#define nINVALID_RS232_GPIO_Port GPIOD
#define USER_BUTTON_R_Pin GPIO_PIN_9
#define USER_BUTTON_R_GPIO_Port GPIOD
#define SD_DETECT_Pin GPIO_PIN_10
#define SD_DETECT_GPIO_Port GPIOD
#define CARROT_GPO_1_Pin GPIO_PIN_11
#define CARROT_GPO_1_GPIO_Port GPIOD
#define CARROT_GPO_2_Pin GPIO_PIN_12
#define CARROT_GPO_2_GPIO_Port GPIOD
#define CARROT_GPO_3_Pin GPIO_PIN_13
#define CARROT_GPO_3_GPIO_Port GPIOD
#define CARROT_GPO_4_Pin GPIO_PIN_14
#define CARROT_GPO_4_GPIO_Port GPIOD
#define EN_5V0_LOAD_Pin GPIO_PIN_15
#define EN_5V0_LOAD_GPIO_Port GPIOD
#define EN_24V0_MAIN_Pin GPIO_PIN_15
#define EN_24V0_MAIN_GPIO_Port GPIOA
#define FULL_PB_1000_Pin GPIO_PIN_3
#define FULL_PB_1000_GPIO_Port GPIOD
#define nFAIL_PB_1000_Pin GPIO_PIN_4
#define nFAIL_PB_1000_GPIO_Port GPIOD
#define nEN_PB_1000_Pin GPIO_PIN_7
#define nEN_PB_1000_GPIO_Port GPIOD
#define EN_12V0_MAIN_Pin GPIO_PIN_3
#define EN_12V0_MAIN_GPIO_Port GPIOB
#define EN_12V0_LOAD2_Pin GPIO_PIN_4
#define EN_12V0_LOAD2_GPIO_Port GPIOB
#define nFLAG_LS_Pin GPIO_PIN_5
#define nFLAG_LS_GPIO_Port GPIOB
#define INT2_LSM6_Pin GPIO_PIN_6
#define INT2_LSM6_GPIO_Port GPIOB
#define EN_12V0_SONIC_Pin GPIO_PIN_7
#define EN_12V0_SONIC_GPIO_Port GPIOB
#define EN_24V0_LOAD_Pin GPIO_PIN_9
#define EN_24V0_LOAD_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
