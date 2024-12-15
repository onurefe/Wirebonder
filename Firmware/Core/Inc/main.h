/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define CONTACT_SENSORS_RES1_Pin GPIO_PIN_13
#define CONTACT_SENSORS_RES1_GPIO_Port GPIOC
#define CONTACT_SENSORS_RES2_Pin GPIO_PIN_14
#define CONTACT_SENSORS_RES2_GPIO_Port GPIOC
#define CONTACT_SENSORS_RES3_Pin GPIO_PIN_15
#define CONTACT_SENSORS_RES3_GPIO_Port GPIOC
#define CONTACT_SENSORS_TIP_Pin GPIO_PIN_0
#define CONTACT_SENSORS_TIP_GPIO_Port GPIOC
#define CONTACT_SENSORS_YLIM_Pin GPIO_PIN_1
#define CONTACT_SENSORS_YLIM_GPIO_Port GPIOC
#define CONTACT_SENSORS_MOUSE_LEFT_Pin GPIO_PIN_2
#define CONTACT_SENSORS_MOUSE_LEFT_GPIO_Port GPIOC
#define CONTACT_SENSORS_MOUSE_RIGHT_Pin GPIO_PIN_3
#define CONTACT_SENSORS_MOUSE_RIGHT_GPIO_Port GPIOC
#define ULTRASONIC_VSENS_Pin GPIO_PIN_0
#define ULTRASONIC_VSENS_GPIO_Port GPIOA
#define ULTRASONIC_ISENS_Pin GPIO_PIN_1
#define ULTRASONIC_ISENS_GPIO_Port GPIOA
#define FORCE_COIL_ISENS_Pin GPIO_PIN_2
#define FORCE_COIL_ISENS_GPIO_Port GPIOA
#define ZMOTOR_TACHOMETER_Pin GPIO_PIN_3
#define ZMOTOR_TACHOMETER_GPIO_Port GPIOA
#define ULTRASONIC_SIGNAL_Pin GPIO_PIN_4
#define ULTRASONIC_SIGNAL_GPIO_Port GPIOA
#define LVDT_SIGNAL_Pin GPIO_PIN_5
#define LVDT_SIGNAL_GPIO_Port GPIOA
#define LVDT_A_Pin GPIO_PIN_6
#define LVDT_A_GPIO_Port GPIOA
#define LVDT_B_Pin GPIO_PIN_7
#define LVDT_B_GPIO_Port GPIOA
#define DRIVES_SOL1H_Pin GPIO_PIN_4
#define DRIVES_SOL1H_GPIO_Port GPIOC
#define SERIAL_RX_Pin GPIO_PIN_5
#define SERIAL_RX_GPIO_Port GPIOC
#define DRIVES_SOL1L_Pin GPIO_PIN_0
#define DRIVES_SOL1L_GPIO_Port GPIOB
#define DRIVES_SOL2H_Pin GPIO_PIN_1
#define DRIVES_SOL2H_GPIO_Port GPIOB
#define DRIVES_SOL2L_Pin GPIO_PIN_2
#define DRIVES_SOL2L_GPIO_Port GPIOB
#define SERIAL_TX_Pin GPIO_PIN_10
#define SERIAL_TX_GPIO_Port GPIOB
#define DRIVES_SOL3L_Pin GPIO_PIN_12
#define DRIVES_SOL3L_GPIO_Port GPIOB
#define FORCE_COIL_nPWM_Pin GPIO_PIN_13
#define FORCE_COIL_nPWM_GPIO_Port GPIOB
#define ZMOTOR_nPWM_Pin GPIO_PIN_14
#define ZMOTOR_nPWM_GPIO_Port GPIOB
#define DRIVES_SOL3H_Pin GPIO_PIN_15
#define DRIVES_SOL3H_GPIO_Port GPIOB
#define DRIVES_AREALIGHT_nPWM_Pin GPIO_PIN_6
#define DRIVES_AREALIGHT_nPWM_GPIO_Port GPIOC
#define DRIVES_SPOTLIGHT_nPWM_Pin GPIO_PIN_7
#define DRIVES_SPOTLIGHT_nPWM_GPIO_Port GPIOC
#define STEPPER_TEAR_STEP_Pin GPIO_PIN_8
#define STEPPER_TEAR_STEP_GPIO_Port GPIOC
#define STEPPER_TEAR_DIR_Pin GPIO_PIN_9
#define STEPPER_TEAR_DIR_GPIO_Port GPIOC
#define FORCE_COIL_PWM_Pin GPIO_PIN_8
#define FORCE_COIL_PWM_GPIO_Port GPIOA
#define ZMOTOR_PWM_Pin GPIO_PIN_9
#define ZMOTOR_PWM_GPIO_Port GPIOA
#define STEPPER_Y_STEP_Pin GPIO_PIN_10
#define STEPPER_Y_STEP_GPIO_Port GPIOA
#define STEPPER_Y_DIR_Pin GPIO_PIN_11
#define STEPPER_Y_DIR_GPIO_Port GPIOA
#define STEPPER_EN_Pin GPIO_PIN_12
#define STEPPER_EN_GPIO_Port GPIOA
#define STEPPER_FAULT_Pin GPIO_PIN_15
#define STEPPER_FAULT_GPIO_Port GPIOA
#define STEPPER_RESET_Pin GPIO_PIN_10
#define STEPPER_RESET_GPIO_Port GPIOC
#define INTERFACE_INT_Pin GPIO_PIN_5
#define INTERFACE_INT_GPIO_Port GPIOB
#define INTERFACE_LCD_SCL_Pin GPIO_PIN_6
#define INTERFACE_LCD_SCL_GPIO_Port GPIOB
#define INTERFACE_LCD_SDA_Pin GPIO_PIN_7
#define INTERFACE_LCD_SDA_GPIO_Port GPIOB
#define CONTACT_SENSORS_RES0_Pin GPIO_PIN_9
#define CONTACT_SENSORS_RES0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
