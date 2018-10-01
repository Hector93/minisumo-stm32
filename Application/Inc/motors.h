/**
******************************************************************************
* File Name          : motors.h
* Description        : function definitions for the two dc motors 
******************************************************************************
**/

#ifndef __MOTORS_H
#define __MOTORS_H
#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "mensaje.h"

  void motors(void const* argument); //funcion que controla el proceso, esta definida en freertos.c
  mensaje getConfig();

#endif
  
  
