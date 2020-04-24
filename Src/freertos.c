/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
//#include "message.h"
#include "../Application/Inc/message.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId motorDHandle;
osThreadId motorIHandle;
osThreadId usartHandle;
osThreadId sensorFloorHandle;
osThreadId sensorDistHandle;
osThreadId acelerometroHandle;
osThreadId minisumoHandle;
osMessageQId serialQueueHandle;
osMessageQId motorRQueueHandle;
osMessageQId motorLQueueHandle;
osMessageQId sensorsDistQueueHandle;
osMessageQId sensorsFloorQueueHandle;
osMessageQId imuQueueHandle;
osMessageQId miniQueueHandle;
osSemaphoreId serialSemTxHandle;
osSemaphoreId serialSemRxHandle;
osSemaphoreId irdistHandle;
osSemaphoreId imuSemHandle;
osSemaphoreId irflrHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//extern osMessageQId* serialQHandle;
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
extern void motorR(void const * argument);
extern void motorL(void const * argument);
extern void serial(void const * argument);
extern void sensorsFloor(void const * argument);
extern void sensorsDist(void const * argument);
extern void imu(void const * argument);
extern void mini(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  HAL_UART_Transmit(&huart1,(uint8_t *)"Inicializacion de FreeRtos\r\n",28,200);
  HAL_UART_Transmit(&huart1,(uint8_t *)"creacion de semaforos\r\n",23,200);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of serialSemTx */
  osSemaphoreDef(serialSemTx);
  serialSemTxHandle = osSemaphoreCreate(osSemaphore(serialSemTx), 1);

  /* definition and creation of serialSemRx */
  osSemaphoreDef(serialSemRx);
  serialSemRxHandle = osSemaphoreCreate(osSemaphore(serialSemRx), 1);

  /* definition and creation of irdist */
  osSemaphoreDef(irdist);
  irdistHandle = osSemaphoreCreate(osSemaphore(irdist), 1);

  /* definition and creation of imuSem */
  osSemaphoreDef(imuSem);
  imuSemHandle = osSemaphoreCreate(osSemaphore(imuSem), 1);

  /* definition and creation of irflr */
  osSemaphoreDef(irflr);
  irflrHandle = osSemaphoreCreate(osSemaphore(irflr), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  HAL_UART_Transmit(&huart1,(uint8_t *)"Creacion de colas de comunicacion\r\n",35,200);
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of serialQueue */
  osMessageQDef(serialQueue, 4, message);
  serialQueueHandle = osMessageCreate(osMessageQ(serialQueue), NULL);

  /* definition and creation of motorRQueue */
  osMessageQDef(motorRQueue, 1, message);
  motorRQueueHandle = osMessageCreate(osMessageQ(motorRQueue), NULL);

  /* definition and creation of motorLQueue */
  osMessageQDef(motorLQueue, 1, message);
  motorLQueueHandle = osMessageCreate(osMessageQ(motorLQueue), NULL);

  /* definition and creation of sensorsDistQueue */
  osMessageQDef(sensorsDistQueue, 1, message);
  sensorsDistQueueHandle = osMessageCreate(osMessageQ(sensorsDistQueue), NULL);

  /* definition and creation of sensorsFloorQueue */
  osMessageQDef(sensorsFloorQueue, 1, message);
  sensorsFloorQueueHandle = osMessageCreate(osMessageQ(sensorsFloorQueue), NULL);

  /* definition and creation of imuQueue */
  osMessageQDef(imuQueue, 1, message);
  imuQueueHandle = osMessageCreate(osMessageQ(imuQueue), NULL);

  /* definition and creation of miniQueue */
  osMessageQDef(miniQueue, 3, message);
  miniQueueHandle = osMessageCreate(osMessageQ(miniQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  // serialQHandle = &serialQueueHandle;
  HAL_UART_Transmit(&huart1,(uint8_t *)"Creacion de tareas\r\n",20,200);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 64);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of motorD */
  osThreadDef(motorD, motorR, osPriorityNormal, 0, 64);
  motorDHandle = osThreadCreate(osThread(motorD), NULL);

  /* definition and creation of motorI */
  osThreadDef(motorI, motorL, osPriorityNormal, 0, 64);
  motorIHandle = osThreadCreate(osThread(motorI), NULL);

  /* definition and creation of usart */
  osThreadDef(usart, serial, osPriorityRealtime, 0, 64);
  usartHandle = osThreadCreate(osThread(usart), NULL);

  /* definition and creation of sensorFloor */
  osThreadDef(sensorFloor, sensorsFloor, osPriorityHigh, 0, 64);
  sensorFloorHandle = osThreadCreate(osThread(sensorFloor), NULL);

  /* definition and creation of sensorDist */
  osThreadDef(sensorDist, sensorsDist, osPriorityNormal, 0, 64);
  sensorDistHandle = osThreadCreate(osThread(sensorDist), NULL);

  /* definition and creation of acelerometro */
  osThreadDef(acelerometro, imu, osPriorityAboveNormal, 0, 128);
  acelerometroHandle = osThreadCreate(osThread(acelerometro), NULL);

  /* definition and creation of minisumo */
  osThreadDef(minisumo, mini, osPriorityNormal, 0, 64);
  minisumoHandle = osThreadCreate(osThread(minisumo), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  HAL_UART_Transmit(&huart1,(uint8_t *)"iniciando FreeRtos\r\n",20,200);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
