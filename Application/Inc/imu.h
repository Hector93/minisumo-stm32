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
#define START_IMU     1
#define STOP_IMU      2
#define GET_STATUS    3
#define SET_SATUS     4
#define TAP_X_U       5
#define TAP_X_D       6
#define TAP_Y_U       7
#define TAP_Y_D       8
#define TAP_Z_U       9
#define TAP_Z_D       10
#define PORTRAIT      11
#define LANDSCAPE     12
#define REV_PORTRAIT  13
#define REV_LANDSCAPE 14
#define HEADING       15     
#define QUATERNION    16
#define ROTMATRIX     17
#define IMUERROR      255

extern osMessageQId imuQueueHandle;
uint8_t Sensors_I2C_WriteRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
uint8_t Sensors_I2C_ReadRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);

void mdelay(unsigned long nTime);
uint32_t get_tick_count();
#endif
