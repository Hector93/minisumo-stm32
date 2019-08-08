#include "mini.h"
#include "stdint.h"
#include "message.h"
#include "cmsis_os.h"
#include "usart.h"
#include "imu.h"
#include "packet.h"
#include "sensorsDist.h"

//extern osMessageQId miniQueueHandle;
//extern osSemaphoreId miniSemHandle;
volatile long imuHeading = 0;
sensorDistData distSensorData;

typedef struct {
  long heading;
  uint16_t LMotorSpeed;
  uint16_t RMotorSpeed;
  uint8_t irFloor;
  sensorDistData irDist;
}miniStatus;

void miniprocessMessage(const message *rx);

char testmini[5];
miniStatus status;

void mini(void const * argument){
  message rx;
  xSemaphoreGive(miniSemHandle);
  for(;;){
    //for(;;){    HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);}
    if(pdPASS == (xQueueReceive(miniQueueHandle, &rx, 0))){
      miniprocessMessage(&rx);
    }
    //actualizando estado
    status.heading = imuHeading;
    if(pdPASS == (xSemaphoreTake(miniSemHandle, 0))){//intenta leer region critica, si no puede se salta la actualizacion
      status.irDist = distSensorData;
      xSemaphoreGive(miniSemHandle);// falta validar que no falle en todos los casos
    }
    vTaskDelay(10);
  }  
}

void miniprocessMessage(const message *rx){
  switch(rx->messageUser.type){
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
