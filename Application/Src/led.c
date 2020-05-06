#include "led.h"
#include "stm32f1xx.h"
#include "tim.h"
#include <stdbool.h>
void nrz(bool bit){
  UNUSED(bit);
  vTaskDelay(100);

}

void led(void const* argument){
  UNUSED(argument);
  for(;;){
    
    vTaskDelay(100);
  }
}

