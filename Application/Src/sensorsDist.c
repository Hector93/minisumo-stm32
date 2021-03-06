#include "serial.h"
#include "message.h"
#include "sensorsDist.h"
#include "adc.h"
#include "imu.h"
#include "serial.h"
#include "usart.h"
#include "queue.h"
#include "mini.h"


uint8_t sensorDistStatus; //stores the info of enabled sensors
sensorDistData distSensorDataInternal;
uint16_t sensorDistDataRaw[ADC_CHANELS];

int8_t findOponent();
void irSensorsIoController();
void SensorDistProcessMessage(message msg);
void processAdc();
uint8_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);

void sensorsDist(void const* argument){
  UNUSED(argument);
  message rx;
  taskYIELD();
  const TickType_t xPeriod = pdMS_TO_TICKS( 8);
  xSemaphoreTake(irdistHandle,portMAX_DELAY);
  sensorDistStatus = 31; //by default all sensors are started
  HAL_GPIO_WritePin(ird_enLD_GPIO_Port, ird_enLD_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ird_enLI_GPIO_Port, ird_enLI_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ird_enFI_GPIO_Port, ird_enFI_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ird_enFC_GPIO_Port, ird_enFC_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ird_enFD_GPIO_Port, ird_enFD_Pin, GPIO_PIN_SET);
  HAL_ADCEx_Calibration_Start(&hadc1);
  ADCs_Start();//TODO validar el retorno

  

  for(;;){
    if(pdPASS == (xQueueReceive(sensorsDistQueueHandle,&rx,0))){
      SensorDistProcessMessage(rx);
    }
    //if(pdPASS == (xSemaphoreTake(irdistHandle,100))){
    //procesar la informacion antes de volver a leer el adc
    processAdc();
    //rx = createMessage(sensorsDistID, miniId, DIRECTION, findOponent());
    rx = createMessage(sensorsDistID, miniId, GALLSENSORS, distSensorDataInternal.distDataRaw);
    xQueueSend(miniQueueHandle, &rx, 10);
    message log = messageDinamicArray(sensorsDistID, serialID, ARRAY, sensorDistDataRaw, sizeof(uint16_t)*ADC_CHANELS);
    if(pdPASS != xQueueSend(serialQueueHandle, &log, 0)){
      vPortFree(log.pointer.array);
    }
    while(HAL_OK != ADCs_Start()){
      ADCs_Start();
    }
    //}
    vTaskDelay(xPeriod);
    //taskYIELD();
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
    rx = createMessage(sensorsDistID, msg.messageUser.IdpO, getStatus, sensorDistStatus);
    xQueueSend(imuQueueHandle, &rx, 10);
    break;
  default:
    rx = createMessage(sensorsDistID, msg.messageUser.IdpO, SENSDISTERROR, 0);
    xQueueSend(imuQueueHandle, &rx, 10);
  }
  rx = createMessage(sensorsDistID, msg.messageUser.IdpO, SENSDISTERROR, 0);
  xQueueSend(imuQueueHandle, &rx, 10);
}

//TODO detectar si hay un oponente en los sensores de distancia
int8_t findOponent(){
  int8_t direction = -1;
  uint8_t aux = 0;

  if(aux < distSensorDataInternal.distData.Li){
    aux = distSensorDataInternal.distData.Li;
    direction = LIID;
  }
  
  if(aux < distSensorDataInternal.distData.Fi){
    aux = distSensorDataInternal.distData.Fi;
    direction = FIID;
  }

  if(aux < distSensorDataInternal.distData.Fc){
    aux = distSensorDataInternal.distData.Fc;
    direction = FCID;
  }

  if(aux < distSensorDataInternal.distData.Fd){
    aux = distSensorDataInternal.distData.Fd;
    direction = FDID;
  }

  if(aux < distSensorDataInternal.distData.Ld){
    aux = distSensorDataInternal.distData.Ld;
    direction = LDID;
  }
  
  return direction;
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
    sensorDistDataRaw[LIPOS] = 0;
    distSensorDataInternal.distData.Li = 0;
  }else{
    distSensorDataInternal.distData.Li = map(sensorDistDataRaw[LIPOS], 0, 4095, 0, 7);
  }

  if((sensorDistStatus & FIID) == 0){
    sensorDistDataRaw[FIPOS] = 0;
    distSensorDataInternal.distData.Fi = 0;
  }else{
    distSensorDataInternal.distData.Fi = map(sensorDistDataRaw[FIPOS], 0, 4095, 0, 7);    
  }

  if((sensorDistStatus & FCID) == 0){
    sensorDistDataRaw[FCPOS] = 0;
    distSensorDataInternal.distData.Fc = 0;
  }else{
    distSensorDataInternal.distData.Fc = map(sensorDistDataRaw[FCPOS], 0, 4095, 0, 7);
  }
  if((sensorDistStatus & LDID) == 0){
    sensorDistDataRaw[LDPOS] = 0;
    distSensorDataInternal.distData.Ld = 0;
  }else{
    distSensorDataInternal.distData.Ld = map(sensorDistDataRaw[LDPOS], 0, 4095, 0, 7);    
  }
  if((sensorDistStatus & FDID) == 0){
    sensorDistDataRaw[FDPOS] = 0;
    distSensorDataInternal.distData.Fd = 0;
  }else{
    distSensorDataInternal.distData.Fd = map(sensorDistDataRaw[FDPOS], 0, 4095, 0, 7);      
  } 
}

uint8_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//ADCCALLBACK
void SensorsDistInterrupt(ADC_HandleTypeDef* hadc){
  UNUSED(hadc); //the adc1 always generates the ISR
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if(pdPASS == (xSemaphoreGiveFromISR(irdistHandle,&xHigherPriorityTaskWoken))){
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}
