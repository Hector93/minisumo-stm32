
#include "motors.h"
#include "cmsis_os.h" // para retardos
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "message.h"
#include "stdint.h"

void motorR(const void* argument){
  //  HAL_UART_Transmit(&huart1,"motorD process active\r\n",22,100);
  message tx;
  tx = createMessage(7,7,7);

  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);

  HAL_GPIO_TogglePin(ph_1A_GPIO_Port,ph_1A_Pin);
  HAL_GPIO_TogglePin(ph_2A_GPIO_Port,ph_2B_Pin);
  
  for(;;)
    {
       xQueueSend(serialQueueHandle,&tx,100);
       //taskYIELD();
      //     HAL_UART_Transmit(&huart1,"hola motors\r\n",13,10);
      osDelay(100);
      
    }
}

void motorL(const void* argument){
  //  char saluda[]={"Hola motor"};

  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);

  HAL_GPIO_TogglePin(ph_1A_GPIO_Port,ph_1A_Pin);
  HAL_GPIO_TogglePin(ph_2A_GPIO_Port,ph_2B_Pin);
  
  for(;;)
    {
      //      HAL_UART_Transmit(&huart1,"hola motors\r\n",13,10);
      osDelay(1000);
    }
}
