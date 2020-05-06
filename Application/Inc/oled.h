/**
******************************************************************************
* File Name          : oled.h
* Description        : function definitions for the oled display
******************************************************************************
**/

#ifndef __OLED_H
#define __OLED_H
#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32f1xx.h"

#define oledID  7
/* dmp en direccion: 0,0 */
/* fonts6x8.txt con tamaño: 1520 que ocupara: 48 paginas y inicia en la direccion: 12,0 */
/* fonts7x10.txt con tamaño: 1900 que ocupara: 60 paginas y inicia en la direccion: 18,0 */

  extern  osThreadId oledDisHandle;
     
#endif
