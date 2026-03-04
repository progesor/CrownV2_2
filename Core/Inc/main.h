/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

typedef enum {
	OP_STATE_IDLE,
	OP_STATE_CONT_MODE_INIT,
	OP_STATE_CONT_MODE,
	OP_STATE_CONT_MODE_END,
	OP_STATE_OSC_MODE_VEL_CONTROL_INIT,
	OP_STATE_OSC_MODE_VEL_CONTROL,
	OP_STATE_OSC_MODE_VEL_CONTROL_END,
	OP_STATE_OSC_MODE_POS_CONTROL_INIT,
	OP_STATE_OSC_MODE_POS_CONTROL,
	OP_STATE_OSC_MODE_POS_CONTROL_END,
	OP_STATE_MAN_MODE
} ProgramState_t;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DO_Led_Pin GPIO_PIN_13
#define DO_Led_GPIO_Port GPIOC
#define AI_Pot_Pin GPIO_PIN_0
#define AI_Pot_GPIO_Port GPIOA
#define AI_Motor_V_Pin GPIO_PIN_1
#define AI_Motor_V_GPIO_Port GPIOA
#define AI_Motor_I_Pin GPIO_PIN_2
#define AI_Motor_I_GPIO_Port GPIOA
#define DI_Pedal_Pin GPIO_PIN_3
#define DI_Pedal_GPIO_Port GPIOA
#define DO_E2prom_WC_Pin GPIO_PIN_4
#define DO_E2prom_WC_GPIO_Port GPIOA
#define DO_E2prom_CE_Pin GPIO_PIN_5
#define DO_E2prom_CE_GPIO_Port GPIOA
#define DO_Buzzer_Pin GPIO_PIN_6
#define DO_Buzzer_GPIO_Port GPIOA
#define DI_Pedal_Enable_Pin GPIO_PIN_7
#define DI_Pedal_Enable_GPIO_Port GPIOA
#define DI_HMI_Tus2_Pin GPIO_PIN_0
#define DI_HMI_Tus2_GPIO_Port GPIOB
#define DI_HMI_Tus1_Pin GPIO_PIN_1
#define DI_HMI_Tus1_GPIO_Port GPIOB
#define USART_TX_HMI_Pin GPIO_PIN_10
#define USART_TX_HMI_GPIO_Port GPIOB
#define USART_RX_HMI_Pin GPIO_PIN_11
#define USART_RX_HMI_GPIO_Port GPIOB
#define DI_HMI_Tus8_Pin GPIO_PIN_12
#define DI_HMI_Tus8_GPIO_Port GPIOB
#define DI_HMI_Tus6_Pin GPIO_PIN_13
#define DI_HMI_Tus6_GPIO_Port GPIOB
#define DI_HMI_Tus5_Pin GPIO_PIN_14
#define DI_HMI_Tus5_GPIO_Port GPIOB
#define DI_HMI_Tus4_Pin GPIO_PIN_15
#define DI_HMI_Tus4_GPIO_Port GPIOB
#define DO_Motor_Pwm_Pin GPIO_PIN_8
#define DO_Motor_Pwm_GPIO_Port GPIOA
#define DI_HMI_Tus3_Pin GPIO_PIN_9
#define DI_HMI_Tus3_GPIO_Port GPIOA
#define DI_HMI_Tus7_Pin GPIO_PIN_10
#define DI_HMI_Tus7_GPIO_Port GPIOA
#define DIO_Debug_Serial_Wire_Pin GPIO_PIN_13
#define DIO_Debug_Serial_Wire_GPIO_Port GPIOA
#define CLK_Debug_Serial_Wire_Pin GPIO_PIN_14
#define CLK_Debug_Serial_Wire_GPIO_Port GPIOA
#define DO_Motor_Direction_Pin GPIO_PIN_3
#define DO_Motor_Direction_GPIO_Port GPIOB
#define DO_Motor_Brake_Pin GPIO_PIN_4
#define DO_Motor_Brake_GPIO_Port GPIOB
#define DI_Encoder_B_Pin GPIO_PIN_6
#define DI_Encoder_B_GPIO_Port GPIOB
#define DI_Encoder_A_Pin GPIO_PIN_7
#define DI_Encoder_A_GPIO_Port GPIOB
#define I2C_SCL_E2prom_Pin GPIO_PIN_8
#define I2C_SCL_E2prom_GPIO_Port GPIOB
#define I2C_SDA_E2prom_Pin GPIO_PIN_9
#define I2C_SDA_E2prom_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
