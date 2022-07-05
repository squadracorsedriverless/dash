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
#include "stm32f3xx_hal.h"
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_system.h"
#include "stm32f3xx_ll_exti.h"
#include "stm32f3xx_ll_cortex.h"
#include "stm32f3xx_ll_utils.h"
#include "stm32f3xx_ll_pwr.h"
#include "stm32f3xx_ll_dma.h"
#include "stm32f3xx_ll_gpio.h"

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
#define AMI3_CMD_Pin GPIO_PIN_0
#define AMI3_CMD_GPIO_Port GPIOA
#define ASB_SERVO_PWM_CMD_Pin GPIO_PIN_1
#define ASB_SERVO_PWM_CMD_GPIO_Port GPIOA
#define EBS_RELAY2_CMD_Pin GPIO_PIN_2
#define EBS_RELAY2_CMD_GPIO_Port GPIOA
#define EBS_RELAY1_CMD_Pin GPIO_PIN_3
#define EBS_RELAY1_CMD_GPIO_Port GPIOA
#define COCK_BTN_Pin GPIO_PIN_5
#define COCK_BTN_GPIO_Port GPIOA
#define MISSION_BTN_Pin GPIO_PIN_6
#define MISSION_BTN_GPIO_Port GPIOA
#define BUZZERAS_CMD_Pin GPIO_PIN_7
#define BUZZERAS_CMD_GPIO_Port GPIOA
#define AMS_CMD_Pin GPIO_PIN_0
#define AMS_CMD_GPIO_Port GPIOB
#define BUZZEREV_CMD_Pin GPIO_PIN_1
#define BUZZEREV_CMD_GPIO_Port GPIOB
#define IMD_CMD_Pin GPIO_PIN_2
#define IMD_CMD_GPIO_Port GPIOB
#define EXT_BTN_Pin GPIO_PIN_10
#define EXT_BTN_GPIO_Port GPIOB
#define ASB_CMD_Pin GPIO_PIN_11
#define ASB_CMD_GPIO_Port GPIOB
#define INVERTER_PUMP_PWM_CMD_Pin GPIO_PIN_12
#define INVERTER_PUMP_PWM_CMD_GPIO_Port GPIOB
#define TSOFF_CMD_Pin GPIO_PIN_13
#define TSOFF_CMD_GPIO_Port GPIOB
#define RTD_CMD_Pin GPIO_PIN_14
#define RTD_CMD_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_8
#define LED2_GPIO_Port GPIOA
#define AMI_OFF_CMD_Pin GPIO_PIN_3
#define AMI_OFF_CMD_GPIO_Port GPIOB
#define AMI2_CMD_Pin GPIO_PIN_4
#define AMI2_CMD_GPIO_Port GPIOB
#define AMI1_CMD_Pin GPIO_PIN_5
#define AMI1_CMD_GPIO_Port GPIOB
#define ASSI_YELLOW_CMD_Pin GPIO_PIN_6
#define ASSI_YELLOW_CMD_GPIO_Port GPIOB
#define ASSI_BLUE_CMD_Pin GPIO_PIN_7
#define ASSI_BLUE_CMD_GPIO_Port GPIOB
#define BAT_FAN_PWM_CMD_Pin GPIO_PIN_8
#define BAT_FAN_PWM_CMD_GPIO_Port GPIOB
#define RADIATOR_FANS_PWM_CMD_Pin GPIO_PIN_9
#define RADIATOR_FANS_PWM_CMD_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
