/**
******************************************************************************
* File Name          : imu.h
* Description        : function definitions for the accelerometer
******************************************************************************
**/

#ifndef __IMU_H
#define __IMU_H

#include "cmsis_os.h"
#define imuId 6

//type of message
#define startImu   1
#define stopImu    2
#define getStatus  3
#define setStatus  4

#define IMUERROR 255

extern osMessageQId imuQueueHandle;
uint8_t Sensors_I2C_WriteRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
uint8_t Sensors_I2C_ReadRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);

void mdelay(unsigned long nTime);
uint32_t get_tick_count();
#endif
