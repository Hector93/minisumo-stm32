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

#define aceptSERIAL serialID
#define aceptMOTORR
#define aceptMOTORL 

#define serialID  1
  
  //extern const uint8_t serialID;
  extern  osMessageQId serialQueueHandle;
  //void serial(void const* argument);


#endif
  
