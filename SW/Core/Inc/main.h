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
#define EBS_RELAY1_Pin GPIO_PIN_10
#define EBS_RELAY1_GPIO_Port GPIOD
#define EBS_RELAY2_Pin GPIO_PIN_11
#define EBS_RELAY2_GPIO_Port GPIOD
#define COCK_BTN_Pin GPIO_PIN_12
#define COCK_BTN_GPIO_Port GPIOD
#define MISSION_BTN_Pin GPIO_PIN_13
#define MISSION_BTN_GPIO_Port GPIOD
#define BUZZERAS_CMD_Pin GPIO_PIN_14
#define BUZZERAS_CMD_GPIO_Port GPIOD
#define RTD_CMD_Pin GPIO_PIN_6
#define RTD_CMD_GPIO_Port GPIOC
#define BUZZEREV_CMD_Pin GPIO_PIN_7
#define BUZZEREV_CMD_GPIO_Port GPIOC
#define AMS_CMD_Pin GPIO_PIN_8
#define AMS_CMD_GPIO_Port GPIOC
#define EXT_BTN_Pin GPIO_PIN_9
#define EXT_BTN_GPIO_Port GPIOC
#define IMD_CMD_Pin GPIO_PIN_8
#define IMD_CMD_GPIO_Port GPIOA
#define AMI_OFF_CMD_Pin GPIO_PIN_10
#define AMI_OFF_CMD_GPIO_Port GPIOA
#define ASB_SERVO_PWM_CMD_Pin GPIO_PIN_10
#define ASB_SERVO_PWM_CMD_GPIO_Port GPIOC
#define TSOFF_CMD_Pin GPIO_PIN_11
#define TSOFF_CMD_GPIO_Port GPIOC
#define ASB_CMD_Pin GPIO_PIN_12
#define ASB_CMD_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOD
#define AMI3_CMD_Pin GPIO_PIN_4
#define AMI3_CMD_GPIO_Port GPIOD
#define AMI2_CMD_Pin GPIO_PIN_6
#define AMI2_CMD_GPIO_Port GPIOD
#define AMI1_CMD_Pin GPIO_PIN_5
#define AMI1_CMD_GPIO_Port GPIOB
#define ASSI_YELLOW_CMD_Pin GPIO_PIN_7
#define ASSI_YELLOW_CMD_GPIO_Port GPIOB
#define ASSI_BLUE_CMD_Pin GPIO_PIN_8
#define ASSI_BLUE_CMD_GPIO_Port GPIOB
#define BAT_FAN_PWM_CMD_Pin GPIO_PIN_0
#define BAT_FAN_PWM_CMD_GPIO_Port GPIOE
#define POWERTRAIN_COOLING_PWM_CMD_Pin GPIO_PIN_1
#define POWERTRAIN_COOLING_PWM_CMD_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

#define BAT_FAN_PWM_TIM htim16
#define BAT_FAN_PWM_CH TIM_CHANNEL_1

#define POWERTRAIN_COOLING_PWM_TIM htim17
#define POWERTRAIN_COOLING_PWM_CH TIM_CHANNEL_1

#define ASB_MOTOR_PWM_TIM htim8
#define ASB_MOTOR_PWM_CH TIM_CHANNEL_1

#define COUNTER_TIM htim3

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
