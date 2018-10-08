
#include "motors.h"
#include "cmsis_os.h" // para retardos
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "message.h"
#include "stdint.h"

motorInternalData motorProcessMessage(message msg,motorInternalData data);
motorInternalData motorUpdate(motorInternalData newData, motorInternalData prevData);
void motorDirection(motorInternalData data);
void motorSpeed(motorInternalData data);

void motorR(const void* argument){
  //  HAL_UART_Transmit(&huart1,"motorD process active\r\n",22,100);
  motorInternalData status;
  message tx;
  message rx;
  rx = createMessage(15,2,5,0);
  
  status.motorOpt.velocity = 100;
  status.motorOpt.direction = FORWARD;
  status.motorOpt.channel = motorRID;
  
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
  status = motorUpdate(motorProcessMessage(rx,status),status);
  
  for(;;)
    {
      //xQueueSend(serialQueueHandle,&tx,100);
      //taskYIELD();
      //     HAL_UART_Transmit(&huart1,"hola motors\r\n",13,10);
      if(pdPASS ==(xQueueReceive(motorRQueueHandle,&rx,10))){
	//motorProcessMessage(rx,motorRID);

	status = motorUpdate(motorProcessMessage(rx,status),status);
      }
      osDelay(100);
      
    }
}

void motorL(const void* argument){
  //  HAL_UART_Transmit(&huart1,"motorD process active\r\n",22,100);
  motorInternalData status;
  message tx;
  message rx;
  rx = createMessage(15,2,5,0);
  
  status.motorOpt.velocity = 100;
  status.motorOpt.direction = FORWARD;
  status.motorOpt.channel = motorLID;
  
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
  status = motorUpdate(motorProcessMessage(rx,status),status);
  
  for(;;)
    {
      //xQueueSend(serialQueueHandle,&tx,100);
      //taskYIELD();
      //     HAL_UART_Transmit(&huart1,"hola motors\r\n",13,10);
      if(pdPASS ==(xQueueReceive(motorLQueueHandle,&rx,10))){
	//motorProcessMessage(rx,motorRID);

	status = motorUpdate(motorProcessMessage(rx,status),status);
      }
      osDelay(100);
      
    }
}
uint16_t map(uint8_t velocity){
  return (uint16_t)velocity*(uint16_t)3.21;
}

uint8_t unMap(uint16_t velocity){
  return velocity/3.21;
}

motorInternalData motorProcessMessage(message msg,motorInternalData data){
  motorInternalData aux;
  switch(msg.messageUser.type){
  case motorError:
    //enviar error
    return data;
  case stopHard:     // poner pwm de canal a 100% y poner en cero los dos pines del motor
    data.motorOpt.direction = STOPEDHARD;
    data.motorOpt.velocity = STOPEDHARD;
    return data;
  case stopFree:     //poner pwm de canal en 0;
    data.motorOpt.velocity = STOPED;
    return data;
  case startMotor:
    data.motorData = msg.messageUser.data;
    return data;
  case setDirection:
    aux.motorData = msg.messageUser.data;
    data.motorOpt.direction = map(aux.motorOpt.direction);
    return data;
  case setSpeed:
    aux.motorData = msg.messageUser.data;
    data.motorOpt.velocity = aux.motorOpt.velocity;
    return data;
  case getStatus:
    //enviar mensaje de estado
    return data;
  case getSpeed:
    //enviar mensaje de velocidad
    return data;
  case test:
    //llamar a funcion de prueba
    return data;
  default: return data;
  }
}

motorInternalData motorUpdate(motorInternalData newData, motorInternalData prevData){
  if(newData.motorOpt.channel == prevData.motorOpt.channel){
    if (newData.motorOpt.velocity != prevData.motorOpt.velocity){
      motorSpeed(newData);
    }
    if(newData.motorOpt.direction != prevData.motorOpt.direction){
      motorDirection(newData);
    }
    return newData;
  }else{
    return prevData;
  }
}

void motorSpeed(motorInternalData data){
  if(data.motorOpt.velocity > 0x63){
    HAL_TIM_SetPWM(321,data.motorOpt.channel);
  }else{
    HAL_TIM_SetPWM(map(data.motorOpt.velocity),data.motorOpt.channel);
  }    
}

void motorDirection(motorInternalData data){
  switch(data.motorOpt.direction){
  case FORWARD:
    if(data.motorOpt.channel == motorRID){
      HAL_GPIO_WritePin(ph_1A_GPIO_Port,ph_1A_Pin,GPIO_PIN_SET);
      HAL_GPIO_WritePin(ph_1B_GPIO_Port,ph_1B_Pin,GPIO_PIN_RESET);
    }else if(data.motorOpt.channel == motorLID){
      HAL_GPIO_WritePin(ph_2A_GPIO_Port,ph_2A_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ph_2B_GPIO_Port,ph_2B_Pin,GPIO_PIN_SET);
    }
    break;
  case BACKWARDS:
    if(data.motorOpt.channel == motorRID){
      HAL_GPIO_WritePin(ph_1A_GPIO_Port,ph_1A_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ph_1B_GPIO_Port,ph_1B_Pin,GPIO_PIN_SET);
    }else if(data.motorOpt.channel == motorLID){
      HAL_GPIO_WritePin(ph_2A_GPIO_Port,ph_2A_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ph_2B_GPIO_Port,ph_2B_Pin,GPIO_PIN_SET);
    }
    break;
  case STOPED:
  case STOPEDHARD:    // poner pwm de canal a 100% y poner en cero los dos pines del motor
    if(data.motorOpt.channel == motorRID){
      HAL_GPIO_WritePin(ph_1A_GPIO_Port,ph_1A_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ph_1B_GPIO_Port,ph_1B_Pin,GPIO_PIN_RESET);
    }else if(data.motorOpt.channel == motorLID){
      HAL_GPIO_WritePin(ph_2A_GPIO_Port,ph_2A_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ph_2B_GPIO_Port,ph_2B_Pin,GPIO_PIN_RESET);
    }
    break;
  }
}

