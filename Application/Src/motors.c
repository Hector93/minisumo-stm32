
#include "motors.h"
#include "cmsis_os.h" // para retardos
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "message.h"
#include "stdint.h"

motorInternalData motorProcessMessage(message msg,motorInternalData data);
motorInternalData motorUpdate(motorInternalData newData, motorInternalData prevData);
void motorDirectionInternal(motorInternalData data);
void motorSpeedInternal(motorInternalData data);

void motorR(const void* argument){
  //  HAL_UART_Transmit(&huart1,"motorD process active\r\n",22,100);
  motorInternalData status;
  message tx;
  message rx;
  rx = createMessage(15,2,5,0);
  
  status.motorOpt.speed = 100;
  status.motorOpt.direction = STOPEDHARD;
  status.motorOpt.channel = motorRID;
  
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
  status = motorUpdate(motorProcessMessage(rx,status),status);
  
  for(;;)
    {
      //xQueueSend(serialQueueHandle,&tx,100);
      //taskYIELD();
      if(pdPASS ==(xQueueReceive(motorRQueueHandle,&rx,10))){
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
  rx = createMessage(15,3,5,0);
  
  status.motorOpt.speed = 100;
  status.motorOpt.direction = STOPEDHARD;
  status.motorOpt.channel = motorLID;
  
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
  status = motorUpdate(motorProcessMessage(rx,status),status);
  
  for(;;)
    {
      //xQueueSend(serialQueueHandle,&tx,100);
      //taskYIELD();
      //     HAL_UART_Transmit(&huart1,"hola motors\r\n",13,10);
      if(pdPASS ==(xQueueReceive(motorLQueueHandle,&rx,10))){
	status = motorUpdate(motorProcessMessage(rx,status),status);
      }
      osDelay(100);
      
    }
}
uint16_t map(uint8_t speed){
  return (uint16_t)speed*(uint16_t)3.21;
}

uint8_t unMap(uint16_t speed){
  return speed/3.21;
}

motorInternalData motorProcessMessage(message msg,motorInternalData data){
  motorInternalData aux;
  message response;
  //     HAL_UART_Transmit(&huart1,"hola motors\r\n",13,10);
  switch(msg.messageUser.type){
  case motorError:
    response = createMessage(data.motorOpt.channel,msg.messageUser.IdpO,0,0);
    xQueueSend(serialQueueHandle,&response, 10);
    return data;
  case stopHard:     // poner pwm de canal a 100% y poner en cero los dos pines del motor
    data.motorOpt.direction = STOPEDHARD;
    data.motorOpt.speed = STOPEDHARD;
    return data;
  case stopFree:     //poner pwm de canal en 0;
    data.motorOpt.speed = STOPED;
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
    data.motorOpt.speed = aux.motorOpt.speed;
    return data;
  case getStatus:
    response = createMessage(data.motorOpt.channel,msg.messageUser.IdpO,getStatus,data.motorData);
    xQueueSend(serialQueueHandle,&response,10);    
    return data;
  case getSpeed:
    response = createMessage(data.motorOpt.channel,msg.messageUser.IdpO,getSpeed,data.motorData & 0xFF00);
    xQueueSend(serialQueueHandle,&response,10);    
    return data;
  case getDirection:
    response = createMessage(data.motorOpt.channel,msg.messageUser.IdpO,getDirection,data.motorData & 0x00FF);
    xQueueSend(serialQueueHandle,&response,10);    
    return data;
  case test:
    //llamar a funcion de prueba
    return data;
  default: return data;
  }
}

motorInternalData motorUpdate(motorInternalData newData, motorInternalData prevData){
  if(newData.motorOpt.channel == prevData.motorOpt.channel){
    if (newData.motorOpt.speed != prevData.motorOpt.speed){
      motorSpeedInternal(newData);
    }
    if(newData.motorOpt.direction != prevData.motorOpt.direction){
      motorDirectionInternal(newData);
    }
    return newData;
  }else{
    return prevData;
  }
}

void motorSpeedInternal(motorInternalData data){
  if(data.motorOpt.speed > 0x63){
    HAL_TIM_SetPWM(321,data.motorOpt.channel);
  }else{
    HAL_TIM_SetPWM(map(data.motorOpt.speed),data.motorOpt.channel);
  }    
}

void motorDirectionInternal(motorInternalData data){
  switch(data.motorOpt.direction){
  case FORWARD:
    //    HAL_UART_Transmit(&huart1,"proc motorR\r\n",13,10);
    if(data.motorOpt.channel == motorRID){
      HAL_GPIO_WritePin(ph_1A_GPIO_Port,ph_1A_Pin,GPIO_PIN_SET);
      HAL_GPIO_WritePin(ph_1B_GPIO_Port,ph_1B_Pin,GPIO_PIN_RESET);
    }else if(data.motorOpt.channel == motorLID){
      HAL_GPIO_WritePin(ph_2A_GPIO_Port,ph_2A_Pin,GPIO_PIN_SET);
      HAL_GPIO_WritePin(ph_2B_GPIO_Port,ph_2B_Pin,GPIO_PIN_RESET);
    }
    break;
  case BACKWARDS:
    //    HAL_UART_Transmit(&huart1,"backwards motor \r\n",13,10);
    if(data.motorOpt.channel == motorRID){
      HAL_GPIO_WritePin(ph_1A_GPIO_Port,ph_1A_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ph_1B_GPIO_Port,ph_1B_Pin,GPIO_PIN_SET);
    }else if(data.motorOpt.channel == motorLID){
      HAL_GPIO_WritePin(ph_2A_GPIO_Port,ph_2A_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ph_2B_GPIO_Port,ph_2B_Pin,GPIO_PIN_SET);
    }
    break;
  case STOPED:
    break;
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

uint16_t createMotorData(uint8_t speed,uint8_t direction){
  motorInternalData aux;
  aux.motorOpt.speed = speed;
  aux.motorOpt.direction = direction;
  return aux.motorData;
}
uint16_t motorSpeed(uint8_t speed){
  return createMotorData(speed,0);
}
uint16_t motorDirection(uint8_t direction){
  return createMotorData(0,direction);
}

