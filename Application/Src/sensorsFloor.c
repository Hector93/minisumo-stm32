#include "limits.h"
#include "string.h"
#include "serial.h"
#include "message.h"
#include "mini.h"
#include "sensorsFloor.h"
#include "sensorsDist.h"
#include "adc.h"

uint16_t sensorFloorDataRaw[ADC_CHANELS];
uint8_t sensorFloorData;

typedef struct {
  uint16_t High;
  uint16_t Low;
}calibration;

calibration thresholds[ADC_CHANELS];
int16_t deltas[ADC_CHANELS];

uint8_t processRawData; //TODO enviar mensaje a proceso principal con los sensores procesador y procesar los sensores

typedef enum {HIGH = 1, LOW = 0}Type;

int16_t delta(int16_t previous, int16_t current);
uint8_t deltaCalculations(uint16_t* previous, uint16_t* current);
void thresold(calibration irSensor, uint16_t rawValue, Type type);
void thresoldsCalculation(calibration* irSensor, uint16_t* rawValue, Type type);
void procesIrData(const uint16_t* rawData, uint8_t* irData);

void calibrateSensors();
void readyForNewRawData();

void sensorsFloor(void const* argument){
  HAL_ADCEx_Calibration_Start(&hadc2);
  HAL_ADC_Start(&hadc2);
  readyForNewRawData();
  calibrateSensors();
  /* for(;;){ */
  /*   vTaskDelay(1); */
  /* } */
  
  for(;;){
    readyForNewRawData();
    procesIrData(sensorFloorDataRaw, &sensorFloorData);
    message msg = createMessage(sensorsFloorID, miniId, ALL_SENSORS, sensorFloorData);
    xQueueSend(miniQueueHandle, &msg, 10);
    msg = messageDinamicArray(sensorsFloorID, serialID, ARRAY, sensorFloorDataRaw, sizeof(uint16_t)*ADC_CHANELS);
    xQueueSend(miniQueueHandle, &msg, 100);
    vPortFree(msg.pointer.array);
      
      
  }
  
}

void procesIrData(const uint16_t* rawData, uint8_t* irData){
  for(uint8_t i = 0; i < ADC_CHANELS; i++){
    if(rawData[i] < thresholds[i].High){
      *irData = *irData | 1 << i;
    }else if(rawData[i] > thresholds[i].Low){
      *irData &= ~(1 << i);
    }
  }
  //xSemaphoreGive(irflrHandle);
}

void calibrateSensors(){
  readyForNewRawData();
  sensorFloorData = 0;
  for(uint8_t i = 0; i < ADC_CHANELS; i++){
    thresholds[i].Low = 3072;
    thresholds[i].High = 1024;
  }
  /*
    for(uint8_t i = 0; i < 20; i++){
    readyForNewRawData();
    thresoldsCalculation(thresholds, sensorFloorDataRaw, LOW);
    }
    while(1){
    readyForNewRawData();
    memcpy(sensorFloorDataRawPrevious, sensorFloorDataRaw, sizeof(uint16_t) * ADC_CHANELS);
    uint8_t aux = deltaCalculations(sensorFloorDataRawPrevious, sensorFloorDataRaw);
    if(aux & 1){ thresold(thresholds[0], sensorFloorDataRaw[0], HIGH);}
    if(aux & 2){ thresold(thresholds[1], sensorFloorDataRaw[1], HIGH);}
    if(aux & 3){ thresold(thresholds[2], sensorFloorDataRaw[2], HIGH);}
    if(aux & 4){ thresold(thresholds[3], sensorFloorDataRaw[3], HIGH);}
    if(aux & 5){ thresold(thresholds[4], sensorFloorDataRaw[4], HIGH);}
    
    }
  */
}

void thresold(calibration irSensor, uint16_t rawValue, Type type){
  switch(type){
  case HIGH:
    if(rawValue > irSensor.High){ irSensor.High = rawValue;}
    break;
  case LOW:
    if(rawValue < irSensor.Low){irSensor.Low = rawValue;}
    break;
  }
  
}

void thresoldsCalculation(calibration* irSensor, uint16_t* rawValue, Type type){
  for(uint8_t i = 0; i < ADC_CHANELS; i++){
    thresold(irSensor[i], rawValue[i], type);
  }
}

int16_t delta(int16_t previous, int16_t current){
  return current - previous;
}

uint8_t deltaCalculations(uint16_t* previous, uint16_t* current){
  uint8_t flag = 0;
  for(uint8_t i = 0; i < ADC_CHANELS; i++){
    deltas[i] = delta(previous[i], current[i]);
    if(deltas[i] > 2048){ flag += 1<<i;}
  }
  return flag;
}


void readyForNewRawData(){//TODO checar que no se trabe
  xSemaphoreGive(irdistHandle);
  xSemaphoreTake(irflrHandle, 10);
}

void SensorsFloorInterrupt(ADC_HandleTypeDef* hadc){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if(pdPASS == (xSemaphoreGiveFromISR(irflrHandle,&xHigherPriorityTaskWoken))){
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

  }
}
