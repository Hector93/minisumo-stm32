#include "serial.h"
#include "message.h"
#include "sensorsDist.h"
#include "adc.h"

typedef union{
  uint16_t rawStatus;
  struct{
      uint8_t enabledSensors;
      uint8_t readingSensors;
    }status;
}sensorDistInternalStatus;

sensorDistInternalStatus sensorDistStatus;
uint32_t sensorData[5];

sensorDistData findOponent();
uint16_t SensorDistProcessMessage(message msg);

void sensorsDist(void const* argument){
  extern osSemaphoreId irdistHandle;
  extern ADC_HandleTypeDef hadc1;

  message rx;
  sensorDistStatus.rawStatus = 0; //by default all sensors are stoped
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1,sensorData,4);
  xSemaphoreGive(irdistHandle);
  for(;;){
    if(pdPASS == (xQueueReceive(sensorsDistQueueHandle,&rx,10))){
      SensorDistProcessMessage(rx);
    }
    if(pdPASS == (xSemaphoreTake(irdistHandle,10))){
      //procesar la informacion antes de volver a leer el adc
      HAL_ADC_Start_DMA(&hadc1,sensorData,5);
    }
  }
}

uint16_t SensorDistProcessMessage(message msg){
  switch(msg.messageUser.type){
  case getSensor:
    // regresar solo la lectura de un sensor
    switch(msg.messageUser.data){
    case LIID:
      return sensorData[0];
    case FIID:
      return sensorData[1];
    case FCID:
      return sensorData[2];
    case FDID:
      return sensorData[3];
    case LDID:
      return sensorData[4];
    default:
      return SENSDISTERROR;
    }
    break;
  case getAllSensors: //regresar sensorDistData
    return SENSDISTERROR;
    break;
  case setReading: //desactivar uno o mas sensores
    sensorDistStatus.status.readingSensors ^= msg.messageUser.data;
    return SENSDISTERROR;
    break;
  case setSensor: //poner pin en 0
    sensorDistStatus.status.enabledSensors ^= msg.messageUser.data;
    return SENSDISTERROR;
    break;
  case getStatus: //regresar estado de sensores y si el adc los esta actualizando
    return sensorDistStatus.rawStatus;
  default:
    return SENSDISTERROR;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if(pdPASS == (xSemaphoreGiveFromISR(irdistHandle,&xHigherPriorityTaskWoken))){
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}
 
sensorDistData findOponent(){
  sensorDistData aux;  
  aux.distDataRaw = 0;
  return aux;
}
