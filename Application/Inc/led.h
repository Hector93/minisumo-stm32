/**
******************************************************************************
* File Name          : led.h
* Description        : function definitions for the RGB led
******************************************************************************
**/

#ifndef __LED_H
#define __LED_H
#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "cmsis_os.h"


#define ledID  7

#define T0H 35
#define T0L 9
#define T1H 9
#define T1L 35
#define TRES 1

#define RED    1
#define GREEN  2
#define YELLOW 4
extern osThreadId ledTaskHandle;
     
#endif
