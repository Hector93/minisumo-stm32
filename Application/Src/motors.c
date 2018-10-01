
#include "motors.h"
#include "cmsis_os.h" // para retardos
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "stdint.h"

void motors(const void* argument){
  //  char saluda[]={"Hola motor"};

  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);

  HAL_GPIO_TogglePin(ph_1A_GPIO_Port,ph_1A_Pin);
  HAL_GPIO_TogglePin(ph_2A_GPIO_Port,ph_2B_Pin);
  
  for(;;)
    {
      HAL_UART_Transmit(&huart1,"hola motors\r\n",13,10);
      osDelay(10);
    }
}
