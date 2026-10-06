/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Shutter_control_Pin GPIO_PIN_0
#define Shutter_control_GPIO_Port GPIOC
#define LASER_532_trigger_Pin GPIO_PIN_2
#define LASER_532_trigger_GPIO_Port GPIOC
#define Foot_switch_NO_Pin GPIO_PIN_0
#define Foot_switch_NO_GPIO_Port GPIOA
#define Foot_switch_NC_Pin GPIO_PIN_1
#define Foot_switch_NC_GPIO_Port GPIOA
#define LASER_532_DAC_Pin GPIO_PIN_4
#define LASER_532_DAC_GPIO_Port GPIOA
#define LASER_635_DAC_Pin GPIO_PIN_5
#define LASER_635_DAC_GPIO_Port GPIOA
#define Spot_Size_Pin GPIO_PIN_0
#define Spot_Size_GPIO_Port GPIOB
#define Mirror_spi_select_Pin GPIO_PIN_12
#define Mirror_spi_select_GPIO_Port GPIOB
#define Remote_interlock_Pin GPIO_PIN_6
#define Remote_interlock_GPIO_Port GPIOC
#define SBC_strobe_IN_Pin GPIO_PIN_7
#define SBC_strobe_IN_GPIO_Port GPIOC
#define System_fail_Pin GPIO_PIN_8
#define System_fail_GPIO_Port GPIOC
#define LASER_status_Pin GPIO_PIN_9
#define LASER_status_GPIO_Port GPIOC
#define TMS_SWDIO_Pin GPIO_PIN_13
#define TMS_SWDIO_GPIO_Port GPIOA
#define TCK_SWCLK_Pin GPIO_PIN_14
#define TCK_SWCLK_GPIO_Port GPIOA
#define LASER_TEMP_Pin GPIO_PIN_10
#define LASER_TEMP_GPIO_Port GPIOC
#define LASER_READY_Pin GPIO_PIN_11
#define LASER_READY_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
