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

  //type definitions
#define motorError   0
#define stopHard     1
#define stopFree     2
#define startMotor   3
#define setDirection 4
#define setSpeed     5
#define getStatus    6
#define getSpeed     7
#define test         8
  
  
  extern osMessageQId motorLQueueHandle;
  extern osMessageQId motorRQueueHandle;
  
  void motorR(void const* argument); //funcion que controla el proceso, esta definida en freertos.c
  void motorL(void const* argument); //funcion que controla el proceso, esta definida en freertos.c

#endif
  
  
  
