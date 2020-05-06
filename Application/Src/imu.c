#include "stm32f1xx.h"
#include "imu.h"
#include "usart.h"
#include "i2c.h"
#include "FreeRTOS.h"
#include "xQueSendWrap.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "packet.h"

#include <stdio.h>
#include <string.h>
#include "message.h"
#include "mini.h"
unsigned char *dmp_memory;

/* Private typedef -----------------------------------------------------------*/
/* Data read from MPL. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)
#define PRINT_EULER     (0x10)
#define PRINT_ROT_MAT   (0x20)
#define PRINT_HEADING   (0x40)
#define PRINT_PEDO      (0x80)
#define PRINT_LINEAR_ACCEL (0x100)
#define PRINT_GRAVITY_VECTOR (0x200)

volatile uint32_t hal_timestamp = 0;
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (20)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

#define PEDO_READ_MS    (1000)
#define TEMP_READ_MS    (500)
#define COMPASS_READ_MS (100)
extern osMessageQId miniQueueHandle;

struct rx_s {
  unsigned char header[3];
  unsigned char cmd;
};
struct hal_s {
  unsigned char lp_accel_mode;
  unsigned char sensors;
  unsigned char dmp_on;
  unsigned char wait_for_tap;
  volatile unsigned char new_gyro;
  unsigned char motion_int_mode;
  unsigned long no_dmp_hz;
  unsigned long next_pedo_ms;
  unsigned long next_temp_ms;
  unsigned long next_compass_ms;
  unsigned int report;
  unsigned short dmp_features;
  struct rx_s rx;
};
static struct hal_s hal = {0};

/* USB RX binary semaphore. Actually, it's just a flag. Not included in struct
 * because it's declared extern elsewhere.
 */
volatile unsigned char rx_new;

unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
  signed char orientation[9];
};

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * TODO: The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static struct platform_data_s gyro_pdata = {
					    .orientation = { 1, 0, 0,
							     0, 1, 0,
							     0, 0, 1}
};

#if defined MPU9150 || defined MPU9250
static struct platform_data_s compass_pdata = {
					       .orientation = { 0, 1, 0,
								1, 0, 0,
								0, 0, -1}
};
#define COMPASS_ENABLED 1
#elif defined AK8975_SECONDARY
static struct platform_data_s compass_pdata = {
					       .orientation = {-1, 0, 0,
							       0, 1, 0,
							       0, 0,-1}
};
#define COMPASS_ENABLED 1
#elif defined AK8963_SECONDARY
static struct platform_data_s compass_pdata = {
					       .orientation = {-1, 0, 0,
							       0,-1, 0,
							       0, 0, 1}
};
#define COMPASS_ENABLED 1
#endif


/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* ---------------------------------------------------------------------------*/
/* Get data from MPL.
 * TODO: Add return values to the inv_get_sensor_type_xxx APIs to differentiate
 * between new and stale data.
 */
long data[9];
extern volatile long imuHeading;
void read_from_mpl(void){
  long msg;
  float float_data[3] = {0};
  int8_t accuracy;
  unsigned long timestamp;



  if (inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*)&timestamp)) {
    /* Sends a quaternion packet to the PC. Since this is used by the Python
     * test app to visually represent a 3D quaternion, it's sent each time
     * the MPL has new data.
     */
    eMPL_send_quat(data);
    /* Specific data packets can be sent or suppressed using USB commands. */
    if (hal.report & PRINT_QUAT)
      eMPL_send_data(PACKET_DATA_QUAT, data);
  }
  
  if (hal.report & PRINT_HEADING) {
    //message tx = createMessage(imuId, miniId, HEADING, 0);
    if (inv_get_sensor_type_heading(data, &accuracy, (inv_time_t*)&timestamp)){
      eMPL_send_data(PACKET_DATA_HEADING, data);
      imuHeading = data[0] * 1.0 / (1<<16);
      //xQueueSend(miniQueueHandle, &tx, 10);
    }
  }
  
  if (hal.report & PRINT_ACCEL) {
    if (inv_get_sensor_type_accel(data, &accuracy,
				  (inv_time_t*)&timestamp))
      eMPL_send_data(PACKET_DATA_ACCEL, data);
  }
  if (hal.report & PRINT_GYRO) {
    if (inv_get_sensor_type_gyro(data, &accuracy,
				 (inv_time_t*)&timestamp))
      eMPL_send_data(PACKET_DATA_GYRO, data);
  }
#ifdef COMPASS_ENABLED
  if (hal.report & PRINT_COMPASS) {
    if (inv_get_sensor_type_compass(data, &accuracy,
				    (inv_time_t*)&timestamp))
      eMPL_send_data(PACKET_DATA_COMPASS, data);
  }
#endif
  if (hal.report & PRINT_EULER) {
    if (inv_get_sensor_type_euler(data, &accuracy,
				  (inv_time_t*)&timestamp))
      eMPL_send_data(PACKET_DATA_EULER, data);
  }
  if (hal.report & PRINT_ROT_MAT) {
    if (inv_get_sensor_type_rot_mat(data, &accuracy,
				    (inv_time_t*)&timestamp))
      eMPL_send_data(PACKET_DATA_ROT, data);
  }
  if (hal.report & PRINT_HEADING) {
    if (inv_get_sensor_type_heading(data, &accuracy,
				    (inv_time_t*)&timestamp))
      eMPL_send_data(PACKET_DATA_HEADING, data);
  }
  if (hal.report & PRINT_LINEAR_ACCEL) {
    if (inv_get_sensor_type_linear_acceleration(float_data, &accuracy, (inv_time_t*)&timestamp)) {
      MPL_LOGI("Linear Accel: %7.5f,%7.5f,%7.5f\r\n",
	       float_data[0], float_data[1], float_data[2]);                                        
    }
  }
  if (hal.report & PRINT_GRAVITY_VECTOR) {
    if (inv_get_sensor_type_gravity(float_data, &accuracy,
				    (inv_time_t*)&timestamp))
      MPL_LOGI("Gravity Vector: %7.5f %7.5f %7.5f\r\n",
	       float_data[0], float_data[1], float_data[2]);
  }
  if (hal.report & PRINT_PEDO) {
    unsigned long timestamp;
    get_tick_count(&timestamp);
    if (timestamp > hal.next_pedo_ms) {
      hal.next_pedo_ms = timestamp + PEDO_READ_MS;
      unsigned long step_count, walk_time;
      dmp_get_pedometer_step_count(&step_count);
      dmp_get_pedometer_walk_time(&walk_time);
      MPL_LOGI("Walked %ld steps over %ld milliseconds..\n", step_count,
	       walk_time);
    }
  }

  /* Whenever the MPL detects a change in motion state, the application can
   * be notified. For this example, we use an LED to represent the current
   * motion state.
   */
  msg = inv_get_message_level_0(INV_MSG_MOTION_EVENT | INV_MSG_NO_MOTION_EVENT);
  if (msg) {
    if (msg & INV_MSG_MOTION_EVENT) {
      MPL_LOGI("Motion!\n");
    } else if (msg & INV_MSG_NO_MOTION_EVENT) {
      MPL_LOGI("No motion!\n");
    }
  }
}

#ifdef COMPASS_ENABLED
void send_status_compass() {
  long data[3] = { 0 };
  int8_t accuracy = { 0 };
  unsigned long timestamp;
  inv_get_compass_set(data, &accuracy, (inv_time_t*) &timestamp);
  MPL_LOGI("Compass: %7.4f %7.4f %7.4f ",
	   data[0]/65536.f, data[1]/65536.f, data[2]/65536.f);
  MPL_LOGI("Accuracy= %d\r\n", accuracy);

}
#endif

/* Handle sensor on/off combinations. */
static void setup_gyro(void)
{
  unsigned char mask = 0, lp_accel_was_on = 0;
  if (hal.sensors & ACCEL_ON)
    mask |= INV_XYZ_ACCEL;
  if (hal.sensors & GYRO_ON) {
    mask |= INV_XYZ_GYRO;
    lp_accel_was_on |= hal.lp_accel_mode;
  }
#ifdef COMPASS_ENABLED
  if (hal.sensors & COMPASS_ON) {
    mask |= INV_XYZ_COMPASS;
    lp_accel_was_on |= hal.lp_accel_mode;
  }
#endif
  /* If you need a power transition, this function should be called with a
   * mask of the sensors still enabled. The driver turns off any sensors
   * excluded from this mask.
   */
  mpu_set_sensors(mask);
  mpu_configure_fifo(mask);
  if (lp_accel_was_on) {
    unsigned short rate;
    hal.lp_accel_mode = 0;
    /* Switching out of LP accel, notify MPL of new accel sampling rate. */
    mpu_get_sample_rate(&rate);
    inv_set_accel_sample_rate(1000000L / rate);
  }
}

static void tap_cb(unsigned char direction, unsigned char count){
  message tx;
  switch (direction) {
  case TAP_X_UP:
    //    MPL_LOGI("Tap X+ ");
    tx = createMessage(imuId, miniId, TAP_X_U, 10);
    xQueueSend(miniQueueHandle, &tx, 10);
    break;
  case TAP_X_DOWN:
    tx = createMessage(imuId, miniId, TAP_X_D, 10);
    xQueueSend(miniQueueHandle, &tx, 10);
    //    MPL_LOGI("Tap X- ");
    break;
  case TAP_Y_UP:
    tx = createMessage(imuId, miniId, TAP_Y_U, 10);
    xQueueSend(miniQueueHandle, &tx, 10);
    //    MPL_LOGI("Tap Y+ ");
    break;
  case TAP_Y_DOWN:
    tx = createMessage(imuId, miniId, TAP_Y_D, 10);
    xQueueSend(miniQueueHandle, &tx, 10);
    //    MPL_LOGI("Tap Y- ");
    break;
  case TAP_Z_UP:
    tx = createMessage(imuId, miniId, TAP_Z_U, 10);
    xQueueSend(miniQueueHandle, &tx, 10);
    //    MPL_LOGI("Tap Z+ ");
    break;
  case TAP_Z_DOWN:
    tx = createMessage(imuId, miniId, TAP_Z_D, 10);
    xQueueSend(miniQueueHandle, &tx, 10);
    //    MPL_LOGI("Tap Z- ");
    break;
  default:
    return;
  }
  //  MPL_LOGI("x%d\n", count);
  return;
}

static void android_orient_cb(unsigned char orientation){
  message tx;
  switch (orientation) {
  case ANDROID_ORIENT_PORTRAIT:
    //      MPL_LOGI("Portrait\n");
    tx = createMessage(imuId, miniId, PORTRAIT, 10);
    xQueueSend(miniQueueHandle, &tx, 0);
    break;
  case ANDROID_ORIENT_LANDSCAPE:
    //      MPL_LOGI("Landscape\n");
    tx = createMessage(imuId, miniId, LANDSCAPE, 10);
    xQueueSend(miniQueueHandle, &tx, 0);
    break;
  case ANDROID_ORIENT_REVERSE_PORTRAIT:
    //      MPL_LOGI("Reverse Portrait\n");
    tx = createMessage(imuId, miniId, REV_PORTRAIT, 10);
    xQueueSend(miniQueueHandle, &tx, 0);
    break;
  case ANDROID_ORIENT_REVERSE_LANDSCAPE:
    //      MPL_LOGI("Reverse Landscape\n");
    tx = createMessage(imuId, miniId, REV_LANDSCAPE, 10);
    xQueueSend(miniQueueHandle, &tx, 0);
    break;
  default:
    return;
  }
}


static inline void run_self_test(void)
{
  int result;
  long gyro[3], accel[3];

#if defined (MPU6500) || defined (MPU9250)
  result = mpu_run_6500_self_test(gyro, accel, 0);
#elif defined (MPU6050) || defined (MPU9150)
  result = mpu_run_self_test(gyro, accel);
#endif
  if (result == 0x7) {
    MPL_LOGI("Passed!\n");
    MPL_LOGI("accel: %7.4f %7.4f %7.4f\n",
	     accel[0]/65536.f,
	     accel[1]/65536.f,
	     accel[2]/65536.f);
    MPL_LOGI("gyro: %7.4f %7.4f %7.4f\n",
	     gyro[0]/65536.f,
	     gyro[1]/65536.f,
	     gyro[2]/65536.f);
    /* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

#ifdef USE_CAL_HW_REGISTERS
    /*
     * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
     * instead of pushing the cal data to the MPL software library
     */
    unsigned char i = 0;

    for(i = 0; i<3; i++) {
      gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
      accel[i] *= 2048.f; //convert to +-16G
      accel[i] = accel[i] >> 16;
      gyro[i] = (long)(gyro[i] >> 16);
    }

    mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
    mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
    mpu_set_accel_bias_6050_reg(accel);
#endif
#else
    /* Push the calibrated data to the MPL library.
     *
     * MPL expects biases in hardware units << 16, but self test returns
     * biases in g's << 16.
     */
    unsigned short accel_sens;
    float gyro_sens;

    mpu_get_accel_sens(&accel_sens);
    accel[0] *= accel_sens;
    accel[1] *= accel_sens;
    accel[2] *= accel_sens;
    inv_set_accel_bias(accel, 3);
    mpu_get_gyro_sens(&gyro_sens);
    gyro[0] = (long) (gyro[0] * gyro_sens);
    gyro[1] = (long) (gyro[1] * gyro_sens);
    gyro[2] = (long) (gyro[2] * gyro_sens);
    inv_set_gyro_bias(gyro, 3);
#endif
  }
  else {
    if (!(result & 0x1))
      MPL_LOGE("Gyro failed.\n");
    if (!(result & 0x2))
      MPL_LOGE("Accel failed.\n");
    if (!(result & 0x4))
      MPL_LOGE("Compass failed.\n");
  }

}

static void handle_input(char opt)
{
  switch (opt) {
    /* These commands turn off individual sensors. */
  case '8':
    hal.sensors ^= ACCEL_ON;
    setup_gyro();
    if (!(hal.sensors & ACCEL_ON))
      inv_accel_was_turned_off();
    break;
  case '9':
    hal.sensors ^= GYRO_ON;
    setup_gyro();
    if (!(hal.sensors & GYRO_ON))
      inv_gyro_was_turned_off();
    break;
#ifdef COMPASS_ENABLED
  case '0':
    hal.sensors ^= COMPASS_ON;
    setup_gyro();
    if (!(hal.sensors & COMPASS_ON))
      inv_compass_was_turned_off();
    break;
#endif
    /* The commands send individual sensor data or fused data to the PC. */
  case 'a':
    hal.report ^= PRINT_ACCEL;
    break;
  case 'g':
    hal.report ^= PRINT_GYRO;
    break;
#ifdef COMPASS_ENABLED
  case 'c':
    hal.report ^= PRINT_COMPASS;
    break;
#endif
  case 'e':
    hal.report ^= PRINT_EULER;
    break;
  case 'r':
    hal.report ^= PRINT_ROT_MAT;
    break;
  case 'q':
    hal.report ^= PRINT_QUAT;
    break;
  case 'h':
    hal.report ^= PRINT_HEADING;
    break;
  case 'i':
    hal.report ^= PRINT_LINEAR_ACCEL;
    break;
  case 'o':
    hal.report ^= PRINT_GRAVITY_VECTOR;
    break;
#ifdef COMPASS_ENABLED
  case 'w':
    send_status_compass();
    break;
#endif
    /* This command prints out the value of each gyro register for debugging.
     * If logging is disabled, this function has no effect.
     */
  case 'd':
    mpu_reg_dump();
    break;
    /* Test out low-power accel mode. */
  case 'p':
    if (hal.dmp_on)
      /* LP accel is not compatible with the DMP. */
      break;
    mpu_lp_accel_mode(20);
    /* When LP accel mode is enabled, the driver automatically configures
     * the hardware for latched interrupts. However, the MCU sometimes
     * misses the rising/falling edge, and the hal.new_gyro flag is never
     * set. To avoid getting locked in this state, we're overriding the
     * driver's configuration and sticking to unlatched interrupt mode.
     *
     * TODO: The MCU supports level-triggered interrupts.
     */
    mpu_set_int_latched(0);
    hal.sensors &= ~(GYRO_ON|COMPASS_ON);
    hal.sensors |= ACCEL_ON;
    hal.lp_accel_mode = 1;
    inv_gyro_was_turned_off();
    inv_compass_was_turned_off();
    break;
    /* The hardware self test can be run without any interaction with the
     * MPL since it's completely localized in the gyro driver. Logging is
     * assumed to be enabled; otherwise, a couple LEDs could probably be used
     * here to display the test results.
     */
  case 't':
    run_self_test();
    /* Let MPL know that contiguity was broken. */
    inv_accel_was_turned_off();
    inv_gyro_was_turned_off();
    inv_compass_was_turned_off();
    break;
    /* Depending on your application, sensor data may be needed at a faster or
     * slower rate. These commands can speed up or slow down the rate at which
     * the sensor data is pushed to the MPL.
     *
     * In this example, the compass rate is never changed.
     */
  case '1':
    if (hal.dmp_on) {
      dmp_set_fifo_rate(10);
      inv_set_quat_sample_rate(100000L);
    } else
      mpu_set_sample_rate(10);
    inv_set_gyro_sample_rate(100000L);
    inv_set_accel_sample_rate(100000L);
    break;
  case '2':
    if (hal.dmp_on) {
      dmp_set_fifo_rate(20);
      inv_set_quat_sample_rate(50000L);
    } else
      mpu_set_sample_rate(20);
    inv_set_gyro_sample_rate(50000L);
    inv_set_accel_sample_rate(50000L);
    break;
  case '3':
    if (hal.dmp_on) {
      dmp_set_fifo_rate(40);
      inv_set_quat_sample_rate(25000L);
    } else
      mpu_set_sample_rate(40);
    inv_set_gyro_sample_rate(25000L);
    inv_set_accel_sample_rate(25000L);
    break;
  case '4':
    if (hal.dmp_on) {
      dmp_set_fifo_rate(50);
      inv_set_quat_sample_rate(20000L);
    } else
      mpu_set_sample_rate(50);
    inv_set_gyro_sample_rate(20000L);
    inv_set_accel_sample_rate(20000L);
    break;
  case '5':
    if (hal.dmp_on) {
      dmp_set_fifo_rate(100);
      inv_set_quat_sample_rate(10000L);
    } else
      mpu_set_sample_rate(100);
    inv_set_gyro_sample_rate(10000L);
    inv_set_accel_sample_rate(10000L);
    break;
  case ',':
    /* Set hardware to interrupt on gesture event only. This feature is
     * useful for keeping the MCU asleep until the DMP detects as a tap or
     * orientation event.
     */
    dmp_set_interrupt_mode(DMP_INT_GESTURE);
    break;
  case '.':
    /* Set hardware to interrupt periodically. */
    dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
    break;
  case '6':
    /* Toggle pedometer display. */
    hal.report ^= PRINT_PEDO;
    break;
  case '7':
    /* Reset pedometer. */
    dmp_set_pedometer_step_count(0);
    dmp_set_pedometer_walk_time(0);
    break;
  case 'f':
    if (hal.lp_accel_mode)
      /* LP accel is not compatible with the DMP. */
      return;
    /* Toggle DMP. */
    if (hal.dmp_on) {
      unsigned short dmp_rate;
      unsigned char mask = 0;
      hal.dmp_on = 0;
      mpu_set_dmp_state(0);
      /* Restore FIFO settings. */
      if (hal.sensors & ACCEL_ON)
	mask |= INV_XYZ_ACCEL;
      if (hal.sensors & GYRO_ON)
	mask |= INV_XYZ_GYRO;
      if (hal.sensors & COMPASS_ON)
	mask |= INV_XYZ_COMPASS;
      mpu_configure_fifo(mask);
      /* When the DMP is used, the hardware sampling rate is fixed at
       * 200Hz, and the DMP is configured to downsample the FIFO output
       * using the function dmp_set_fifo_rate. However, when the DMP is
       * turned off, the sampling rate remains at 200Hz. This could be
       * handled in inv_mpu.c, but it would need to know that
       * inv_mpu_dmp_motion_driver.c exists. To avoid this, we'll just
       * put the extra logic in the application layer.
       */
      dmp_get_fifo_rate(&dmp_rate);
      mpu_set_sample_rate(dmp_rate);
      inv_quaternion_sensor_was_turned_off();
      MPL_LOGI("DMP disabled.\n");
    } else {
      unsigned short sample_rate;
      hal.dmp_on = 1;
      /* Preserve current FIFO rate. */
      mpu_get_sample_rate(&sample_rate);
      dmp_set_fifo_rate(sample_rate);
      inv_set_quat_sample_rate(1000000L / sample_rate);
      mpu_set_dmp_state(1);
      MPL_LOGI("DMP enabled.\n");
    }
    break;
  case 'm':
    /* Test the motion interrupt hardware feature. */
#ifndef MPU6050 // not enabled for 6050 product
    hal.motion_int_mode = 1;
#endif 
    break;

  case 'v':
    /* Toggle LP quaternion.
     * The DMP features can be enabled/disabled at runtime. Use this same
     * approach for other features.
     */
    hal.dmp_features ^= DMP_FEATURE_6X_LP_QUAT;
    dmp_enable_feature(hal.dmp_features);
    if (!(hal.dmp_features & DMP_FEATURE_6X_LP_QUAT)) {
      inv_quaternion_sensor_was_turned_off();
      MPL_LOGI("LP quaternion disabled.\n");
    } else
      MPL_LOGI("LP quaternion enabled.\n");
    break;
  default:
    break;
  }
  hal.rx.cmd = 0;
}

/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
void gyro_data_ready_cb(void)
{
  hal.new_gyro = 1;
}

/*******************************************************************************/

//----------------------------------------------------------------------------------------------------------


extern osMessageQId imuQueueHandle;
extern osSemaphoreId imuSemHandle;

struct int_param_s int_param;
inv_error_t result = 0;
unsigned char accel_fsr,  new_temp = 0;
unsigned short gyro_rate, gyro_fsr;
unsigned long timestamp;
#ifdef COMPASS_ENABLED
unsigned char new_compass = 0;
unsigned short compass_fsr;
#endif

int imuInit(){
  result = mpu_init(&int_param);
  if (result != 0) {
    MPL_LOGE("Could not initialize gyro.\n");
    return -1;
  }
  result = inv_init_mpl();
  if (result != INV_SUCCESS) {
    MPL_LOGE("Could not initialize MPL.\n");
    return -1;
  }
  if(inv_enable_quaternion() != INV_SUCCESS){
    MPL_LOGE("Could not quaternion.\n");
    return -1;
  }
  if(inv_enable_9x_sensor_fusion() != INV_SUCCESS){
    MPL_LOGE("Could not enable 9x sensor fusion.\n");
    return -1;
  }
  if(inv_enable_fast_nomot() != INV_SUCCESS){
    MPL_LOGE("Could not enable fast nomot.\n");
    return -1;
  }
  if(inv_enable_gyro_tc() != INV_SUCCESS){
    MPL_LOGE("Could not enable gyro tc.\n");
    return -1;
  }  
#ifdef COMPASS_ENABLED
  /* Compass calibration algorithms. */
  inv_enable_vector_compass_cal();
  inv_enable_magnetic_disturbance();
#endif
  /* Allows use of the MPL APIs in read_from_mpl. */
  inv_enable_eMPL_outputs();
  result = inv_start_mpl();
  if (result == INV_ERROR_NOT_AUTHORIZED) {
    while (1) {
      MPL_LOGE("Not authorized.\n");
      return -1;
    }
  }
  if (result) {
    MPL_LOGE("Could not start the MPL.\n");
    return -1;
  }
#ifdef COMPASS_ENABLED
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
#else
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
#endif
  /* Push both gyro and accel data into the FIFO. */
  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_sample_rate(DEFAULT_MPU_HZ);
#ifdef COMPASS_ENABLED
  /* The compass sampling rate can be less than the gyro/accel sampling rate.
   * Use this function for proper power management.
   */
  mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
#endif
  mpu_get_sample_rate(&gyro_rate);
  mpu_get_gyro_fsr(&gyro_fsr);
  mpu_get_accel_fsr(&accel_fsr);
#ifdef COMPASS_ENABLED
  mpu_get_compass_fsr(&compass_fsr);
#endif
  /* Sync driver configuration with MPL. */
  /* Sample rate expected in microseconds. */
  inv_set_gyro_sample_rate(1000000L / gyro_rate);
  inv_set_accel_sample_rate(1000000L / gyro_rate);
#ifdef COMPASS_ENABLED
  /* The compass rate is independent of the gyro and accel rates. As long as
   * inv_set_compass_sample_rate is called with the correct value, the 9-axis
   * fusion algorithm's compass correction gain will work properly.
   */
  inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);
#endif
  /* Set chip-to-body orientation matrix.
   * Set hardware units to dps/g's/degrees scaling factor.
   */
  inv_set_gyro_orientation_and_scale(
				     inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
				     (long)gyro_fsr<<15);
  inv_set_accel_orientation_and_scale(
				      inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
				      (long)accel_fsr<<15);
#ifdef COMPASS_ENABLED
  inv_set_compass_orientation_and_scale(
					inv_orientation_matrix_to_scalar(compass_pdata.orientation),
					(long)compass_fsr<<15);
#endif
#ifdef COMPASS_ENABLED
  hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
#else
  hal.sensors = ACCEL_ON | GYRO_ON;
#endif
  hal.dmp_on = 0;
  hal.report = 0;
  hal.rx.cmd = 0;
  hal.next_pedo_ms = 0;
  hal.next_compass_ms = 0;
  hal.next_temp_ms = 0;
  /* Compass reads are handled by scheduler. */
  get_tick_count(&timestamp);
 
  dmp_memory = pvPortMalloc(sizeof(unsigned char) * 3062);
  dmp_memory[0]=0;
  //if(HAL_OK == (HAL_I2C_IsDeviceReady(&hi2c1,0x50 << 1,15,1000))){
  Sensors_I2C_WriteRegister(0x50,0,1,dmp_memory);
  //HAL_I2C_Mem_Write(&hi2c1, 0x50 << 1, 0x0061, 32, dmp_memory, 0, 100);
  //    HAL_I2C_Mem_Write(&hi2c1,0x50 << 1,0,sizeof(uint8_t),dmp_memory,1,10000);
  //}
  //if(HAL_OK == (HAL_I2C_IsDeviceReady(&hi2c1,0x50 << 1,15,1000))){
  //  Sensors_I2C_ReadRegister(0x50,0,(unsigned int)3062,dmp_memory);
  //  HAL_I2C_Mem_Read(&hi2c1,0x50 << 1,0,sizeof(uint8_t),dmp_memory,3062,100000);
  taskENTER_CRITICAL();
  HAL_I2C_Master_Receive(&hi2c1, 0x50 << 1, dmp_memory, 3062, 1000000);
  taskEXIT_CRITICAL();
  //}
  
  if(0 != dmp_load_motion_driver_firmware()){
    return -1;
  }
  vPortFree(dmp_memory);
  //  free(dmp_memory);
  if(0 != dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation))){
    return -1;
  }
  if(0 != dmp_register_tap_cb(tap_cb)){
    return -1;
  }
  if(0 != dmp_register_android_orient_cb(android_orient_cb)){
    return -1;
  }
  
  /*
   * Known Bug -
   * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
   * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
   * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
   * there will be a 25Hz interrupt from the MPU device.
   *
   * There is a known issue in which if you do not enable DMP_FEATURE_TAP
   * then the interrupts will be at 200Hz even if fifo rate
   * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
   *
   * DMP sensor fusion works only with gyro at +-2000dps and accel +-2G
   */
  hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
    DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
    DMP_FEATURE_GYRO_CAL;
  dmp_enable_feature(hal.dmp_features);
  dmp_set_fifo_rate(DEFAULT_MPU_HZ);
  mpu_set_dmp_state(1);
  hal.dmp_on = 1;
  //opciones
  //handle_input('8');
  handle_input('1');
  handle_input('.');
  //handle_input('a');
  // handle_input('g');
  //handle_input('q');
  //handle_input('r');
  handle_input('h');
  //HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  return 0;
}

int imuInvProcessData(){
  unsigned long sensor_timestamp;
  int new_data = 0;
  get_tick_count(&timestamp);
  /* Temperature data doesn't need to be read with every gyro sample.
   * Let's make them timer-based like the compass reads.
   */
  if (timestamp > hal.next_temp_ms) {
    hal.next_temp_ms = timestamp + TEMP_READ_MS;
    new_temp = 1;
  }

  if (hal.new_gyro && hal.dmp_on) {
    short gyro[3], accel_short[3], sensors;
    unsigned char more;
    long accel[3], quat[4], temperature;
    /* This function gets new data from the FIFO when the DMP is in
     * use. The FIFO can contain any combination of gyro, accel,
     * quaternion, and gesture data. The sensors parameter tells the
     * caller which data fields were actually populated with new data.
     * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
     * the FIFO isn't being filled with accel data.
     * The driver parses the gesture data to determine if a gesture
     * event has occurred; on an event, the application will be notified
     * via a callback (assuming that a callback function was properly
     * registered). The more parameter is non-zero if there are
     * leftover packets in the FIFO.
     */
    dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
    if (!more)
      hal.new_gyro = 0;
    if (sensors & INV_XYZ_GYRO) {
      /* Push the new data to the MPL. */
      inv_build_gyro(gyro, sensor_timestamp);
      new_data = 1;
      if (new_temp) {
	new_temp = 0;
	/* Temperature only used for gyro temp comp. */
	mpu_get_temperature(&temperature, &sensor_timestamp);
	inv_build_temp(temperature, sensor_timestamp);
      }
    }
    if (sensors & INV_XYZ_ACCEL) {
      accel[0] = (long)accel_short[0];
      accel[1] = (long)accel_short[1];
      accel[2] = (long)accel_short[2];
      inv_build_accel(accel, 0, sensor_timestamp);
      new_data = 1;
    }
    if (sensors & INV_WXYZ_QUAT) {
      inv_build_quat(quat, 0, sensor_timestamp);
      new_data = 1;
    }
  } else if (hal.new_gyro) {
    short gyro[3], accel_short[3];
    unsigned char sensors, more;
    long accel[3], temperature;
    /* This function gets new data from the FIFO. The FIFO can contain
     * gyro, accel, both, or neither. The sensors parameter tells the
     * caller which data fields were actually populated with new data.
     * For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't
     * being filled with accel data. The more parameter is non-zero if
     * there are leftover packets in the FIFO. The HAL can use this
     * information to increase the frequency at which this function is
     * called.
     */
    hal.new_gyro = 0;
    mpu_read_fifo(gyro, accel_short, &sensor_timestamp,
		  &sensors, &more);
    if (more)
      hal.new_gyro = 1;
    if (sensors & INV_XYZ_GYRO) {
      /* Push the new data to the MPL. */
      inv_build_gyro(gyro, sensor_timestamp);
      new_data = 1;
      if (new_temp) {
	new_temp = 0;
	/* Temperature only used for gyro temp comp. */
	mpu_get_temperature(&temperature, &sensor_timestamp);
	inv_build_temp(temperature, sensor_timestamp);
      }
    }
    if (sensors & INV_XYZ_ACCEL) {
      accel[0] = (long)accel_short[0];
      accel[1] = (long)accel_short[1];
      accel[2] = (long)accel_short[2];
      inv_build_accel(accel, 0, sensor_timestamp);
      new_data = 1;
    }
  }
  if (new_data) {
    inv_execute_on_data();
    return 1;
  }
  return 0;
}

void imu(void const * argument){
  UNUSED(argument);
  //HAL_Delay(100);
  //xSemaphoreGive(imuSemHandle);
  taskENTER_CRITICAL();
  int res = imuInit();
  taskEXIT_CRITICAL();
  if(res < 0){
    while(1){MPL_LOGE("Could not initialize IMU.\r\n");}
  }
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  //while(1);
  
  for(;;){
    //    message rx;
    //    if(pdPASS == (xQueueReceive(imuQueueHandle, &rx, 0))){
    /* A byte has been received via USART. See handle_input for a list of
     * valid commands.
     */
    //handle_input(rx.messageUser.type);
    // }
    if(pdTRUE == xSemaphoreTake(imuSemHandle, portMAX_DELAY)){
      if(imuInvProcessData()){
	/* This function reads bias-compensated sensor data and sensor
	 * fusion outputs from the MPL. The outputs are formatted as seen
	 * in eMPL_outputs.c. This function only needs to be called at the
	 * rate requested by the host.
	 */
	read_from_mpl();      
      }
    }
    //taskYIELD();
  //vTaskDelay(100);
 }
}


uint8_t Sensors_I2C_WriteRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data){
  taskENTER_CRITICAL();
  uint8_t res = (HAL_OK == HAL_I2C_Mem_Write(&hi2c1,slave_addr << 1,reg_addr,sizeof(uint8_t),data,length,10000) ? 0 : 1);
  taskEXIT_CRITICAL();
  return res;
}

uint8_t Sensors_I2C_ReadRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data){
  taskENTER_CRITICAL();
  uint8_t res = (HAL_OK == HAL_I2C_Mem_Read(&hi2c1,slave_addr << 1,reg_addr,sizeof(uint8_t),data,length,100000)? 0 : 1);
  taskEXIT_CRITICAL();
  return res;
}


void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c){
  UNUSED(hi2c);
  while(1){
    
  }
}

void mdelay(unsigned long nTime){
  HAL_Delay(nTime);
  //vTaskDelay(pdMS_TO_TICKS(nTime));
}

uint32_t get_tick_count(){
  return xTaskGetTickCount();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  if(GPIO_Pin == intAcel_Pin){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    gyro_data_ready_cb();
    if(pdPASS == (xSemaphoreGiveFromISR(imuSemHandle,&xHigherPriorityTaskWoken))){
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}
