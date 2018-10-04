/**
******************************************************************************
* File Name          : serial.h
* Description        : function definitions for the serial port
******************************************************************************
**/

#ifndef __SERIAL_H
#define __SERIAL_H
#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "cmsis_os.h"
  
 
  extern  osMessageQId serialQueueHandle;
  //void serial(void const* argument);


#endif
  
