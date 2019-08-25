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

#define sensorsFloorID 5

#define sensorsFloorFI 1
#define sensorsFloorFD 2
#define sensorsFloorAI 4
#define sensorsFloorAD 8
#define sensorsFloorAC 16
//message type 
#define ALL_SENSORS 1
//definir queue del proceso
extern osThreadId sensorFloorHandle;
extern osSemaphoreId irflrHandle;
#endif
