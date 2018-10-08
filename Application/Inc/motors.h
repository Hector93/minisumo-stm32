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
#include "stdint.h"

#define motorRID 2
#define motorLID 3

  //motor directions
#define STOPED     0
#define STOPEDHARD 1
#define FORWARD    2
#define BACKWARDS  3
  
  //type definitions
#define motorError   0
#define stopHard     1 //complete
#define stopFree     2 //complete
#define startMotor   3 //complete
#define setDirection 4 //complete
#define setSpeed     5 //complete
#define getStatus    6 //faltaleer la direccion
#define getSpeed     7 //complete
#define test         8
  
  typedef union{
    uint16_t motorData;
    struct {
      uint8_t speed;
      uint8_t direction;
      uint8_t channel;
    }motorOpt;
  }motorInternalData;
  
  extern osMessageQId motorLQueueHandle;
  extern osMessageQId motorRQueueHandle;
  
  void motorR(void const* argument); //funcion que controla el proceso, esta definida en freertos.c
  void motorL(void const* argument); //funcion que controla el proceso, esta definida en freertos.c
  uint16_t map(uint8_t speed);
  uint8_t unMap(uint16_t speed);
  uint16_t createMotorData(uint8_t speed,uint8_t direction);
  uint16_t motorSpeed(uint8_t speed);
  uint16_t motorDirection(uint8_t direction);
#endif
  
  
  
