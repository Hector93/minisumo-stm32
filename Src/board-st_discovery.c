#include "stm32f1xx.h"
//#include "stm32f1xx_usart.h" 
//#include "stm32f1xx_rcc.h"
//#include "i2c.h"
//#include "uart.h"
//#include "gpio.h"
#include "board-st_discovery.h"

//#define TICK_FREQ (1000u)

/*volatile uint32_t g_ul_ms_ticks=0;
  static volatile uint32_t TimingDelay=0;
  unsigned long idle_time=0;
  extern uint32_t SystemCoreClock; //168000000=168Mhz (original value)
*/

/*void mdelay(unsigned long nTime)
  {
  TimingDelay = nTime;
  while(TimingDelay != 0);
  }*/
/*
  int get_tick_count(unsigned long *count)
  {
  count[0] = g_ul_ms_ticks;
  return 0;
  }

  void TimingDelay_Decrement(void)
  {
  if (TimingDelay != 0x00)
  TimingDelay--;
  }

  void TimeStamp_Increment(void)
  {
  g_ul_ms_ticks++;
  }
*/
