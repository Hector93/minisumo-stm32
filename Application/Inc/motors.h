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

  //#include "tim.h"
  //#include "gpio.h"
  #include "serial.h"

#define motorRID 2
#define motorLID 3
  
  
  extern osMessageQId motorLQueueHandle;
  extern osMessageQId motorRQueueHandle;
  
  void motorR(void const* argument); //funcion que controla el proceso, esta definida en freertos.c
  void motorL(void const* argument); //funcion que controla el proceso, esta definida en freertos.c

#endif
  
  
  
