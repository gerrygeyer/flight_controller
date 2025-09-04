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
#include "stm32h7xx_hal.h"

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
#define Diode_D2_Pin GPIO_PIN_1
#define Diode_D2_GPIO_Port GPIOA
#define Debug_Pin GPIO_PIN_3
#define Debug_GPIO_Port GPIOA
#define LIDAR1_xShunt_out_Pin GPIO_PIN_5
#define LIDAR1_xShunt_out_GPIO_Port GPIOA
#define LIDAR2_xShunt_out_Pin GPIO_PIN_4
#define LIDAR2_xShunt_out_GPIO_Port GPIOC
#define LIDAR3_xShunt_out_Pin GPIO_PIN_0
#define LIDAR3_xShunt_out_GPIO_Port GPIOB
#define LIDAR3_INT_EXTI1_Pin GPIO_PIN_1
#define LIDAR3_INT_EXTI1_GPIO_Port GPIOB
#define LIDAR3_INT_EXTI1_EXTI_IRQn EXTI1_IRQn
#define IMU_EXTI2_Pin GPIO_PIN_2
#define IMU_EXTI2_GPIO_Port GPIOB
#define IMU_EXTI2_EXTI_IRQn EXTI2_IRQn
#define GPS_Rx_Pin GPIO_PIN_7
#define GPS_Rx_GPIO_Port GPIOE
#define GPS_Tx_Pin GPIO_PIN_8
#define GPS_Tx_GPIO_Port GPIOE
#define IMU_RESET_Pin GPIO_PIN_14
#define IMU_RESET_GPIO_Port GPIOE
#define IMU_ADDR_Pin GPIO_PIN_15
#define IMU_ADDR_GPIO_Port GPIOE
#define IMU_SCL_Pin GPIO_PIN_10
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_11
#define IMU_SDA_GPIO_Port GPIOB
#define ESP32_Rx_Pin GPIO_PIN_12
#define ESP32_Rx_GPIO_Port GPIOB
#define ESP32_Tx_Pin GPIO_PIN_13
#define ESP32_Tx_GPIO_Port GPIOB
#define Motor1_Tx_Pin GPIO_PIN_14
#define Motor1_Tx_GPIO_Port GPIOB
#define Motor1_Rx_Pin GPIO_PIN_15
#define Motor1_Rx_GPIO_Port GPIOB
#define Motor3_Tx_Pin GPIO_PIN_8
#define Motor3_Tx_GPIO_Port GPIOD
#define Motor3_Rx_Pin GPIO_PIN_9
#define Motor3_Rx_GPIO_Port GPIOD
#define LIDAR3_SCL_Pin GPIO_PIN_12
#define LIDAR3_SCL_GPIO_Port GPIOD
#define LIDAR3_SDA_Pin GPIO_PIN_13
#define LIDAR3_SDA_GPIO_Port GPIOD
#define Motor4_Tx_Pin GPIO_PIN_6
#define Motor4_Tx_GPIO_Port GPIOC
#define Motor4_Rx_Pin GPIO_PIN_7
#define Motor4_Rx_GPIO_Port GPIOC
#define DUMMY_SD_GPIO_INPUT_Pin GPIO_PIN_15
#define DUMMY_SD_GPIO_INPUT_GPIO_Port GPIOA
#define Motor2_Tx_Pin GPIO_PIN_5
#define Motor2_Tx_GPIO_Port GPIOD
#define Motor2_Rx_Pin GPIO_PIN_6
#define Motor2_Rx_GPIO_Port GPIOD
#define MAG_CS_GPIO_OUT_Pin GPIO_PIN_3
#define MAG_CS_GPIO_OUT_GPIO_Port GPIOB
#define MAG_DRDY_EXTI4_Pin GPIO_PIN_4
#define MAG_DRDY_EXTI4_GPIO_Port GPIOB
#define MAG_DRDY_EXTI4_EXTI_IRQn EXTI4_IRQn
#define MAG_INT_EXTI5_Pin GPIO_PIN_5
#define MAG_INT_EXTI5_GPIO_Port GPIOB
#define MAG_INT_EXTI5_EXTI_IRQn EXTI9_5_IRQn
#define MAG_SCL_Pin GPIO_PIN_6
#define MAG_SCL_GPIO_Port GPIOB
#define MAG_SDA_Pin GPIO_PIN_7
#define MAG_SDA_GPIO_Port GPIOB
#define OPTICAL_FLOW_8RX_Pin GPIO_PIN_0
#define OPTICAL_FLOW_8RX_GPIO_Port GPIOE
#define OPTICAL_FLOW_8TX_Pin GPIO_PIN_1
#define OPTICAL_FLOW_8TX_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
