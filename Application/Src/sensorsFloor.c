#include "serial.h"
#include "message.h"
#include "sensorsFloor.h"
#include "adc.h"

void sensorsFloor(void const* argument){
  for(;;){
    //    HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length)
      osDelay(100);
  }
}
