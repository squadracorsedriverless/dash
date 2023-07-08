/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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
#define BP_FAN_PWM_CMD_Pin GPIO_PIN_1
#define BP_FAN_PWM_CMD_GPIO_Port GPIOA
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
#define AMS_ERR_CMD_Pin GPIO_PIN_0
#define AMS_ERR_CMD_GPIO_Port GPIOB
#define BUZZEREV_CMD_Pin GPIO_PIN_1
#define BUZZEREV_CMD_GPIO_Port GPIOB
#define IMD_ERR_CMD_Pin GPIO_PIN_2
#define IMD_ERR_CMD_GPIO_Port GPIOB
#define EXT_BTN_Pin GPIO_PIN_10
#define EXT_BTN_GPIO_Port GPIOB
#define ASB_ERR_CMD_Pin GPIO_PIN_11
#define ASB_ERR_CMD_GPIO_Port GPIOB
#define TSOFF_CMD_Pin GPIO_PIN_13
#define TSOFF_CMD_GPIO_Port GPIOB
#define RTD_CMD_Pin GPIO_PIN_14
#define RTD_CMD_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_8
#define LED2_GPIO_Port GPIOA
#define INVERTER_PUMP_PWM_CMD_Pin GPIO_PIN_8
#define INVERTER_PUMP_PWM_CMD_GPIO_Port GPIOB
#define ASB_MOTOR_PWM_CMD_Pin GPIO_PIN_9
#define ASB_MOTOR_PWM_CMD_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define BP_FAN_PWM_TIM htim15
#define BP_FAN_PWM_CH TIM_CHANNEL_1

#define INVERTER_PUMP_PWM_TIM htim16
#define INVERTER_PUMP_PWM_CH TIM_CHANNEL_1

#define ASB_MOTOR_PWM_TIM htim17
#define ASB_MOTOR_PWM_CH TIM_CHANNEL_1

#define COUNTER_TIM htim2

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
