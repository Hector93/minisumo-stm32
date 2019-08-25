#include "mini.h"
#include "stdint.h"
#include "message.h"
#include "cmsis_os.h"
#include "usart.h"
#include "imu.h"
#include "packet.h"
#include "sensorsDist.h"
#include "sensorsFloor.h"

volatile long imuHeading = 0;
sensorDistData distSensorData;

typedef struct {
  long heading;
  uint16_t LMotorSpeed;
  uint16_t RMotorSpeed;
  uint8_t irFloor;
  sensorDistData irDist;
}miniStatus;

void miniprocessMessage(message *rx);
void imuMessage(message *rx);
void sensorsFloorMsg(message *rx);
void sensorsDistMsg(message *rx);

char testmini[5];
volatile miniStatus status;

void mini(void const * argument){
  message rx;
  for(;;){
    if(pdPASS == (xQueueReceive(miniQueueHandle, &rx, 100))){
      miniprocessMessage(&rx);
      //status.irFloor = 5;
    }
    //actualizando estado
    //status.heading = imuHeading;
    //vTaskDelay(10);

    
  }  
}

void miniprocessMessage(message *rx){
  switch(rx->messageUser.IdpO){
  case sensorsFloorID:
    sensorsFloorMsg(rx);
    break;
  case sensorsDistID:
    sensorsDistMsg(rx);
    break;
  case imuId:
    imuMessage(rx);
    break;
  default:
    break;
    
  }
}

void imuMessage(message *rx){
  switch(rx->messageUser.type){
  case HEADING:
    status.heading = imuHeading;
    break;
  case TAP_X_U:
    HAL_UART_Transmit(&huart1, (uint8_t*)"tapxu\n", 6, 100);
    break;
  case TAP_X_D:
    HAL_UART_Transmit(&huart1, (uint8_t*)"tapxd\n", 6, 100);
    break;
  case TAP_Y_U:
    HAL_UART_Transmit(&huart1, (uint8_t*)"tapyu\n", 6, 100);
    break;
  case TAP_Y_D:
    HAL_UART_Transmit(&huart1, (uint8_t*)"tapyd\n", 6, 100);
    break;
  case TAP_Z_U:
    HAL_UART_Transmit(&huart1, (uint8_t*)"tapzu\n", 6, 100);
    break;
  case TAP_Z_D:
    HAL_UART_Transmit(&huart1, (uint8_t*)"tapzd\n", 6, 100);
    break;
  case PORTRAIT:
    HAL_UART_Transmit(&huart1, (uint8_t*)"portr\n", 6, 100);
    break;
  case LANDSCAPE:
    HAL_UART_Transmit(&huart1, (uint8_t*)"lands\n", 6, 100);
    break;
  case REV_PORTRAIT:
    HAL_UART_Transmit(&huart1, (uint8_t*)"revpo\n", 6, 100);
    break;
  case REV_LANDSCAPE:
    HAL_UART_Transmit(&huart1, (uint8_t*)"revla\n", 6, 100);
    break;
  default:
    break;
  }
}

void sensorsFloorMsg(message *rx){
  switch(rx->messageUser.type){
  case ALL_SENSORS:
    status.irFloor = rx->messageUser.data;
    break;
  default:
    break;
  }
}

void sensorsDistMsg(message *rx){
  switch(rx->messageUser.type){
  case GALLSENSORS:
    status.irDist.distDataRaw = rx->messageUser.data;
    break;
  default:
    break;
  }
}
