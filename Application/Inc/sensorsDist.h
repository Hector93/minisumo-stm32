/**
******************************************************************************
* File Name          : sensorsFloor.h
* Description        : function definitions for the floor sensors
******************************************************************************
**/

#ifndef __SENSORS_FLOOR_H
#define __SENSORS_FLOOR_H

#include "FreeRTOS.h"
#include "cmsis_os.h"

//[LD,LI,FD,FC,FI]

#define sensorsDistID 4
#define LIID 1
#define FIID 2
#define FCID 4
#define FDID 8
#define LDID 16

//type definitions
#define getSensor      1
#define getAllSensors  2
#define setReading     3
#define setSensor      4
#define getStatus      5
#define SENSDISTERROR  0

typedef union{
  uint16_t distDataRaw;
  struct{
    uint8_t binDir : 4;
    uint8_t LI : 4;
    uint8_t FI : 4;
    uint8_t FC : 4;
    uint8_t FD : 4;
    uint8_t LD : 4;
  }distData;
}sensorDistData;



//definir queue del proceso y funciones para comunicarse con el proceso
extern osThreadId sensorDistHandle;
extern osSemaphoreId irdistHandle;
extern osMessageQId sensorsDistQueueHandle;
#endif
