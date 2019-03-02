#include "serial.h"
#include "message.h"
#include "sensorsDist.h"
#include "adc.h"
#include "imu.h"
#include "serial.h"
#include "usart.h"
#include "queue.h"
#include "mini.h"

uint8_t sensorDistStatus;
extern sensorDistData distSensorData;
sensorDistData distSensorDataInternal;
uint32_t sensorData[5];

sensorDistData findOponent();
void irSensorsIoController();
void SensorDistProcessMessage(message msg);
void processAdc();
uint8_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);

void sensorsDist(void const* argument){
  message rx;
  sensorDistStatus = 31; //by default all sensors are started
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1,sensorData,4);
  xSemaphoreGive(irdistHandle);

  for(;;){
    if(pdPASS == (xQueueReceive(sensorsDistQueueHandle,&rx,10))){
      SensorDistProcessMessage(rx);
    }
    if(pdPASS == (xSemaphoreTake(irdistHandle,10))){
      //procesar la informacion antes de volver a leer el adc
      if(pdPASS == (xSemaphoreTake(miniSemHandle, 10))){
	distSensorData = distSensorDataInternal;
	xSemaphoreGive(miniSemHandle);
      }
      processAdc();
      HAL_ADC_Start_DMA(&hadc1,sensorData,5);
    }
  }
}

void SensorDistProcessMessage(message msg){
  message rx;
  switch(msg.messageUser.type){
  case setSensor:
    if((msg.messageUser.data & (LIID | FIID | FCID | FDID | LDID)) != 0){
      uint8_t aux = sensorDistStatus;
      aux ^= msg.messageUser.data;
      if(aux ^ sensorDistStatus){
	sensorDistStatus = aux;
	irSensorsIoController();
      }
    }else{
      rx = createMessage(sensorsDistID, msg.messageUser.IdpO, SENSDISTERROR, 0);
      xQueueSend(imuQueueHandle, &rx, 10);
    }      
    break;
  case getStatus: //regresar estado de sensores y si el adc los esta actualizando
    rx = createMessage(sensorsDistID, msg.messageUser.IdpO, getStatus, 0);
    xQueueSend(imuQueueHandle, &rx, 10);
  default:
    rx = createMessage(sensorsDistID, msg.messageUser.IdpO, SENSDISTERROR, 0);
    xQueueSend(imuQueueHandle, &rx, 10);
  }
  rx = createMessage(sensorsDistID, msg.messageUser.IdpO, SENSDISTERROR, 0);
  xQueueSend(imuQueueHandle, &rx, 10);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if(pdPASS == (xSemaphoreGiveFromISR(irdistHandle,&xHigherPriorityTaskWoken))){
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}
//TODO detectar si hay un oponente en los sensores de distancia
sensorDistData findOponent(){
  sensorDistData aux;  
  aux.distDataRaw = 0;
  return aux;
}

void irSensorsIoController(){  
  if((sensorDistStatus & LIID) == LIID){
    HAL_GPIO_TogglePin(ird_enLI_GPIO_Port,ird_enLI_Pin);
  }else if((sensorDistStatus & FIID) == FIID){
    HAL_GPIO_TogglePin(ird_enFI_GPIO_Port, ird_enFI_Pin);
  }else if((sensorDistStatus & FCID) == FCID){
    HAL_GPIO_TogglePin(ird_enFC_GPIO_Port, ird_enFC_Pin);
  }else if((sensorDistStatus & FDID) == FDID){
    HAL_GPIO_TogglePin(ird_enFD_GPIO_Port, ird_enFD_Pin);
  }else if((sensorDistStatus & LDID) == LDID){
    HAL_GPIO_TogglePin(ird_enLD_GPIO_Port, ird_enLD_Pin);
  }
}

void processAdc(){
  if((sensorDistStatus & LIID) == 0){
    sensorData[LIPOS] = 0;
    distSensorDataInternal.distData.Li = 0;
  }else{
    distSensorDataInternal.distData.Li = map(sensorData[LIPOS], 0, 4095, 0, 7);    
  }

  if((sensorDistStatus & FIID) == 0){
    sensorData[FIPOS] = 0;
    distSensorDataInternal.distData.Fi = 0;
  }else{
    distSensorDataInternal.distData.Fi = map(sensorData[FIPOS], 0, 4095, 0, 7);    
  }

  if((sensorDistStatus & FCID) == 0){
    sensorData[FCPOS] = 0;
    distSensorDataInternal.distData.Fc = 0;
  }else{
    distSensorDataInternal.distData.Fc = map(sensorData[FCPOS], 0, 4095, 0, 7);
  }
  if((sensorDistStatus & LDID) == 0){
    sensorData[LDPOS] = 0;
    distSensorDataInternal.distData.Ld = 0;
  }else{
    distSensorDataInternal.distData.Ld = map(sensorData[LDPOS], 0, 4095, 0, 7);    
  }
  if((sensorDistStatus & FDID) == 0){
    sensorData[FDPOS] = 0;
    distSensorDataInternal.distData.Fd = 0;
  }else{
    distSensorDataInternal.distData.Fd = map(sensorData[FDPOS], 0, 4095, 0, 7);      
  } 
}

uint8_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
