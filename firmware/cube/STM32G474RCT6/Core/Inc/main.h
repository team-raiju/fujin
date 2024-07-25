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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUTTON_2_Pin GPIO_PIN_13
#define BUTTON_2_GPIO_Port GPIOC
#define MOT_R_DIR_Pin GPIO_PIN_0
#define MOT_R_DIR_GPIO_Port GPIOC
#define MOT_R_PWM_Pin GPIO_PIN_1
#define MOT_R_PWM_GPIO_Port GPIOC
#define MOT_L_DIR_Pin GPIO_PIN_2
#define MOT_L_DIR_GPIO_Port GPIOC
#define MOT_L_PWM_Pin GPIO_PIN_3
#define MOT_L_PWM_GPIO_Port GPIOC
#define SEN_L_S_Pin GPIO_PIN_0
#define SEN_L_S_GPIO_Port GPIOA
#define SEN_L_F_Pin GPIO_PIN_1
#define SEN_L_F_GPIO_Port GPIOA
#define ENCODER_L_A_Pin GPIO_PIN_2
#define ENCODER_L_A_GPIO_Port GPIOA
#define ENCODER_L_B_Pin GPIO_PIN_3
#define ENCODER_L_B_GPIO_Port GPIOA
#define MOT_L_CURRENT_Pin GPIO_PIN_4
#define MOT_L_CURRENT_GPIO_Port GPIOA
#define ADC_BAT_Pin GPIO_PIN_0
#define ADC_BAT_GPIO_Port GPIOB
#define SPI_CS_1_Pin GPIO_PIN_1
#define SPI_CS_1_GPIO_Port GPIOB
#define MOT_R_CURRENT_Pin GPIO_PIN_2
#define MOT_R_CURRENT_GPIO_Port GPIOB
#define SPI_CS_2_Pin GPIO_PIN_10
#define SPI_CS_2_GPIO_Port GPIOB
#define SEN_R_F_Pin GPIO_PIN_11
#define SEN_R_F_GPIO_Port GPIOB
#define SEN_R_S_Pin GPIO_PIN_12
#define SEN_R_S_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOB
#define ENCODER_R_B_Pin GPIO_PIN_15
#define ENCODER_R_B_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_6
#define BUZZER_GPIO_Port GPIOC
#define ENCODER_R_A_Pin GPIO_PIN_7
#define ENCODER_R_A_GPIO_Port GPIOC
#define I2C3_SCL_EE_Pin GPIO_PIN_8
#define I2C3_SCL_EE_GPIO_Port GPIOC
#define I2C3_SDA_EE_Pin GPIO_PIN_9
#define I2C3_SDA_EE_GPIO_Port GPIOC
#define I2C2_SDA_IMU_Pin GPIO_PIN_8
#define I2C2_SDA_IMU_GPIO_Port GPIOA
#define I2C2_SCL_IMU_Pin GPIO_PIN_9
#define I2C2_SCL_IMU_GPIO_Port GPIOA
#define TIM2_CH1_WS2812_Pin GPIO_PIN_15
#define TIM2_CH1_WS2812_GPIO_Port GPIOA
#define LED_IR_R_F_Pin GPIO_PIN_12
#define LED_IR_R_F_GPIO_Port GPIOC
#define LED_IR_R_S_Pin GPIO_PIN_2
#define LED_IR_R_S_GPIO_Port GPIOD
#define LED_IR_L_S_Pin GPIO_PIN_3
#define LED_IR_L_S_GPIO_Port GPIOB
#define LED_IR_L_F_Pin GPIO_PIN_4
#define LED_IR_L_F_GPIO_Port GPIOB
#define TIM3_CH2_FAN_Pin GPIO_PIN_5
#define TIM3_CH2_FAN_GPIO_Port GPIOB
#define BUTTON_1_Pin GPIO_PIN_9
#define BUTTON_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
