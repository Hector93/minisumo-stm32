/**
******************************************************************************
* File Name          : mini.h
* Description        : function definitions for the minisumo, control of the robot
******************************************************************************
**/

#ifndef __MINI_H
#define __MINI_H
#include "cmsis_os.h"
#define miniId 7

extern osMessageQId miniQueueHandle;
extern osSemaphoreId miniSemHandle;
#endif
