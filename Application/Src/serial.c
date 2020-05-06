#include "serial.h"
//#include "queue.h"
//#include "FreeRTOS.h"
#include "usart.h"
#include "message.h"
#include "motors.h"

//const uint8_t serialID = 1;
/*
 *
 * las ISR estan definidas en usart.h
 *
 */
serialPkt tx;
void processMessage(message msg);

void serial(void const* argument){
  UNUSED(argument);
  extern osSemaphoreId serialSemTxHandle;
  extern osSemaphoreId serialSemRxHandle;
  serialPkt rx;
  uint8_t freeMem = 0;
  //  rx.mws.syncChar = '\n';
  
  rx.mws.syncChar = tx.mws.syncChar = SYNCCHAR;
  
  xSemaphoreGive(serialSemTxHandle);
  HAL_UART_Transmit(&huart1,(uint8_t *)"serial process active\r\n",22,200);

  //  HAL_UART_Transmit(&huart1,sizeof(message)/sizeof(uint8_t),2,100);
  HAL_UART_Receive_DMA(&huart1,rx.data,sizeof(serialPkt) - 1);
  for(;;){
    //xSemaphoreTake(serialSemHandle, portMAX_DELAY);
    if(pdPASS == (xSemaphoreTake(serialSemTxHandle,0))){//puedo enviar?
      if(freeMem == 1 && tx.mws.msg.pointer.array[0] == ARRAY){
	vPortFree(tx.mws.msg.pointer.array);
	freeMem=0;
      }
      if(pdPASS == (xQueueReceive(serialQueueHandle,&tx,100))){//envia dato
	if (tx.mws.msg.pointer.array[0] == ARRAY){
	  HAL_UART_Transmit_DMA(&huart1,tx.mws.msg.pointer.array,tx.mws.msg.pointer.size);
	  freeMem = 1;
	}else{
	  HAL_UART_Transmit_DMA(&huart1,tx.data,sizeof(serialPkt) - 1);
	}
	  
      }else{//no pude leer dato a enviar D':
	xSemaphoreGive(serialSemTxHandle);
      }
    }
    if(pdPASS == (xSemaphoreTake(serialSemRxHandle,0))){
      if(rx.mws.syncChar == SYNCCHAR){
	processMessage(rx.mws.msg);	
      }else{
	rx.mws.msg.messageUser.IdpD = serialSyncError;
	processMessage(rx.mws.msg);
      }
      //HAL_UART_Transmit(&huart1,rx.data,sizeof(serialPkt),100);
      //HAL_UART_Transmit(&huart1,"hola serial\r\n",13,100);
      HAL_UART_Receive_DMA(&huart1,rx.data,sizeof(serialPkt) - 1);
    }
    taskYIELD();
  }
 
}

void processMessage(message msg){
  message aux;
  if(msg.messageUser.IdpO == externalControllerID){
    switch(msg.messageUser.IdpD){
#ifdef aceptSERIAL
    case serialID:
      HAL_UART_Transmit(&huart1,(uint8_t*)"serial package\r\n",15,1000);
      aux = createMessage(serialID,externalControllerID,MSG_ERROR,NOTIMPLEM);
      //TODO porcesar mensaje para proceso serial
      break;
#endif
#ifdef aceptMOTORR
    case motorRID:
      HAL_UART_Transmit(&huart1,(uint8_t*)"motorR package\r\n",15,1000);
      aux = createMessage(externalControllerID,motorRID,msg.messageUser.type,msg.messageUser.data);
      xQueueSend(motorRQueueHandle,&aux,100);
      break;
#endif
#ifdef aceptMOTORL
    case motorLID:
      HAL_UART_Transmit(&huart1,(uint8_t*)"motorL package\r\n",15,1000);
      aux = createMessage(externalControllerID,motorLID,msg.messageUser.type,msg.messageUser.data);
      xQueueSend(motorLQueueHandle,&aux,100);
      break;
#endif
#ifdef syncError
    case serialSyncError:
      HAL_UART_Transmit(&huart1,(uint8_t*)"syncError messg\r\n",17,1000);
      aux = createMessage(serialID,externalControllerID,MSG_ERROR,SYNCERROR);
      break;
#endif
    default :
      HAL_UART_Transmit(&huart1,(uint8_t*)"unknown package\r\n",17,1000);
      aux = createMessage(serialID,externalControllerID,MSG_ERROR,UNKNOWNDEST);
    }
  }else{
    //TODO responder paquete invalido?
    HAL_UART_Transmit(&huart1,(uint8_t*)"invalido\r\n",10,100);
  }
}
