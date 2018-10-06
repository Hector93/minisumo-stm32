
#include "motors.h"
#include "cmsis_os.h" // para retardos
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "message.h"
#include "stdint.h"

uint8_t motorProcessMessage(message a, uint8_t channel);

void motorR(const void* argument){
  //  HAL_UART_Transmit(&huart1,"motorD process active\r\n",22,100);
  message tx;
  message rx;
  tx = createMessage(7,1,7,7);

  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);

  HAL_GPIO_TogglePin(ph_1A_GPIO_Port,ph_1A_Pin);
  HAL_GPIO_TogglePin(ph_2A_GPIO_Port,ph_2B_Pin);
  
  for(;;)
    {
      //xQueueSend(serialQueueHandle,&tx,100);
      //taskYIELD();
      //     HAL_UART_Transmit(&huart1,"hola motors\r\n",13,10);
      if(pdPASS ==(xQueueReceive(motorRQueueHandle,&rx,10))){
	motorProcessMessage(rx,motorRID);
      }
      osDelay(100);
      
    }
}

void motorL(const void* argument){
  //  char saluda[]={"Hola motor"};

  //  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);

  //  HAL_GPIO_TogglePin(ph_1A_GPIO_Port,ph_1A_Pin);
  //  HAL_GPIO_TogglePin(ph_2A_GPIO_Port,ph_2B_Pin);
  
  for(;;)
    {
      //      HAL_UART_Transmit(&huart1,"hola motors\r\n",13,10);
      osDelay(1000);
    }
}

extern TIM_HandleTypeDef htim1;
extern TIM_OC_InitTypeDef sConfigOC;

uint8_t motorProcessMessage(message a, uint8_t channel){
  switch(a.messageUser.type){
  case motorError: return -1;
  case stopHard:
    HAL_GPIO_WritePin(ph_1A_GPIO_Port,ph_1A_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ph_1B_GPIO_Port,ph_1B_Pin,GPIO_PIN_RESET);
    return 1;
  case stopFree:
    HAL_GPIO_WritePin(ph_1A_GPIO_Port,ph_1A_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(ph_1B_GPIO_Port,ph_1B_Pin,GPIO_PIN_SET);
    return 1;
  case startMotor:   
  case setDirection:
    HAL_GPIO_WritePin(ph_1A_GPIO_Port,ph_1A_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(ph_1B_GPIO_Port,ph_1B_Pin,GPIO_PIN_RESET);
    return 1;
  case setSpeed:
    //    HAL_UART_Transmit(&huart1,"hola veloci\r\n",13,100);
    HAL_TIM_SetPWM(a.messageUser.data,channel);
    return 1;
  case getStatus:        return 1;
  case getSpeed:         return 1;
  case test:             return 1;
  default: return -1;
  }
}

