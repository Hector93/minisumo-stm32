
//#include <stdio.h>
#include "stm32f1xx.h"
#include "uart.h"
#include "usart.h"
//#include "stm32f1xx_hal_usart.h"
//#include "stm32f1xx_gpio.h" 
//#include "misc.h" 
//#include "stm32f1xx_exti.h"
//#include "stm32f1xx_hal_rcc.h"

/********************************* Defines ************************************/


//////////////////  USART2
#define USARTx                           USART2
#define USARTx_CLK                       RCC_APB1Periph_USART2
#define USARTx_CLK_INIT                  RCC_APB1PeriphClockCmd
#define USARTx_IRQn                      USART2_IRQn
#define USARTx_IRQHandler                USART2_IRQHandler

#define USARTx_TX_PIN                    GPIO_Pin_2
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define USARTx_TX_SOURCE                 GPIO_PinSource2
#define USARTx_TX_AF                     GPIO_AF_USART2

#define USARTx_RX_PIN                    GPIO_Pin_3
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define USARTx_RX_SOURCE                 GPIO_PinSource3
#define USARTx_RX_AF                     GPIO_AF_USART2

#define USARTx_DMAx_CLK                  RCC_AHBPeriph_DMA1

/********************************* Globals ************************************/
/********************************* Prototypes *********************************/
/*******************************  Function ************************************/

void USART_Config(void)
{
 
}
/*
  int fputcI(int ch )
  {
  //* Place your implementation of fputc here 
  //* e.g. write a character to the USART 
  //USART_SendData(USARTx, (uint8_t) ch);

  //* Loop until the end of transmission 
  //while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
  //  {}
  //HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
  return  HAL_UART_Transmit(&huart1,(uint8_t*)ch,1,1000);
  }
*/
