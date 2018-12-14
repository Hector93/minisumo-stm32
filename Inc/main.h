/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#define led_Pin GPIO_PIN_13
#define led_GPIO_Port GPIOC
#define led1_Pin GPIO_PIN_14
#define led1_GPIO_Port GPIOC
#define led2_Pin GPIO_PIN_15
#define led2_GPIO_Port GPIOC
#define ledRGB_Pin GPIO_PIN_0
#define ledRGB_GPIO_Port GPIOD
#define ird_LD_Pin GPIO_PIN_0
#define ird_LD_GPIO_Port GPIOA
#define ird_LI_Pin GPIO_PIN_1
#define ird_LI_GPIO_Port GPIOA
#define ird_FD_Pin GPIO_PIN_2
#define ird_FD_GPIO_Port GPIOA
#define ird_FC_Pin GPIO_PIN_3
#define ird_FC_GPIO_Port GPIOA
#define ird_FI_Pin GPIO_PIN_4
#define ird_FI_GPIO_Port GPIOA
#define irp_FI_Pin GPIO_PIN_5
#define irp_FI_GPIO_Port GPIOA
#define irp_FD_Pin GPIO_PIN_6
#define irp_FD_GPIO_Port GPIOA
#define irp_AI_Pin GPIO_PIN_7
#define irp_AI_GPIO_Port GPIOA
#define irp_AC_Pin GPIO_PIN_0
#define irp_AC_GPIO_Port GPIOB
#define irp_AD_Pin GPIO_PIN_1
#define irp_AD_GPIO_Port GPIOB
#define boton1_Pin GPIO_PIN_2
#define boton1_GPIO_Port GPIOB
#define ph_1A_Pin GPIO_PIN_10
#define ph_1A_GPIO_Port GPIOB
#define ph_1B_Pin GPIO_PIN_11
#define ph_1B_GPIO_Port GPIOB
#define ph_2A_Pin GPIO_PIN_12
#define ph_2A_GPIO_Port GPIOB
#define ph_2B_Pin GPIO_PIN_13
#define ph_2B_GPIO_Port GPIOB
#define PH_ENA_Pin GPIO_PIN_14
#define PH_ENA_GPIO_Port GPIOB
#define PH_ENB_Pin GPIO_PIN_15
#define PH_ENB_GPIO_Port GPIOB
#define BT_START_Pin GPIO_PIN_8
#define BT_START_GPIO_Port GPIOA
#define TX_Pin GPIO_PIN_9
#define TX_GPIO_Port GPIOA
#define RX_Pin GPIO_PIN_10
#define RX_GPIO_Port GPIOA
#define intAcel_Pin GPIO_PIN_15
#define intAcel_GPIO_Port GPIOA
#define intAcel_EXTI_IRQn EXTI15_10_IRQn
#define ird_enFI_Pin GPIO_PIN_3
#define ird_enFI_GPIO_Port GPIOB
#define ird_enFC_Pin GPIO_PIN_4
#define ird_enFC_GPIO_Port GPIOB
#define ird_enFD_Pin GPIO_PIN_5
#define ird_enFD_GPIO_Port GPIOB
#define ird_enLI_Pin GPIO_PIN_6
#define ird_enLI_GPIO_Port GPIOB
#define ird_enLD_Pin GPIO_PIN_7
#define ird_enLD_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_8
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_9
#define SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
