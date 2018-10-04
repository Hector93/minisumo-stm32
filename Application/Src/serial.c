#include "serial.h"
//#include "queue.h"
//#include "FreeRTOS.h"
#include "usart.h"
#include "message.h"

void serial(void const* argument){
  extern osSemaphoreId serialSemTxHandle;
  extern osSemaphoreId serialSemRxHandle;
  serialPkt rx;
  rx.sync.syncChar = '\n';
  message tx;

  xSemaphoreGive(serialSemTxHandle);
  //  HAL_UART_Transmit(&huart1,"serial process active\r\n",22,100);
  HAL_UART_Receive_DMA(&huart1,rx.data,sizeof(serialPkt) - 1);
  for(;;){
    if((uxQueueMessagesWaiting(serialQueueHandle)) > 0){//existe algo por enviar?
      if(pdPASS == (xSemaphoreTake(serialSemTxHandle,0))){//puedo enviar?
	if(pdPASS == (xQueueReceive(serialQueueHandle,&tx,0))){//envia dato
	  HAL_UART_Transmit_DMA(&huart1,tx.messageRaw.data,sizeof(message));
	}else{//no pude leer dato a enviar D':
	  xSemaphoreGive(serialSemTxHandle);
	}
      }
    }
    if(pdPASS == (xSemaphoreTake(serialSemRxHandle,0))){
      //procesar el mensaje que se acaba de recibir!!!
      //HAL_UART_Transmit(&huart1,rx.data,sizeof(serialPkt),100);
      HAL_UART_Receive_DMA(&huart1,rx.data,sizeof(serialPkt) - 1);
    }
    portYIELD();
  }
 
}
