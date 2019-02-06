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

#define LIPOS 0
#define FIPOS 1
#define FCPOS 2
#define FDPOS 3
#define LDPOS 4

//type definitions
//#define getSensor      1
//#define getAllSensors  2
//#define setReading     3
#define setSensor      4
#define getStatus      5
#define SENSDISTERROR  1

typedef union{
  uint16_t distDataRaw;
  struct{
    uint8_t EnemyPresence : 1;
    uint8_t Li : 3;
    uint8_t Fi : 3;
    uint8_t Fc : 3;
    uint8_t Fd : 3;
    uint8_t Ld : 3;
  }distData;
}sensorDistData;



//definir queue del proceso y funciones para comunicarse con el proceso
extern osThreadId sensorDistHandle;
extern osSemaphoreId irdistHandle;
extern osMessageQId sensorsDistQueueHandle;
#endif
