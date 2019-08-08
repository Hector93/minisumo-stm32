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

void processMessage(message msg);

void serial(void const* argument){
  extern osSemaphoreId serialSemTxHandle;
  extern osSemaphoreId serialSemRxHandle;
  serialPkt rx;
  //  rx.mws.syncChar = '\n';
  serialPkt tx;
  rx.mws.syncChar = tx.mws.syncChar = SYNCCHAR;
  
  xSemaphoreGive(serialSemTxHandle);
  HAL_UART_Transmit(&huart1,(uint8_t *)"serial process active\r\n",22,200);

  //  HAL_UART_Transmit(&huart1,sizeof(message)/sizeof(uint8_t),2,100);
  HAL_UART_Receive_DMA(&huart1,rx.data,sizeof(serialPkt) - 1);
  for(;;){
    if((uxQueueMessagesWaiting(serialQueueHandle)) > 0){//existe algo por enviar?
      if(pdPASS == (xSemaphoreTake(serialSemTxHandle,0))){//puedo enviar?
	if(pdPASS == (xQueueReceive(serialQueueHandle,&tx,0))){//envia dato
	  HAL_UART_Transmit_DMA(&huart1,tx.data,sizeof(serialPkt) - 1);
	}else{//no pude leer dato a enviar D':
	  xSemaphoreGive(serialSemTxHandle);
	}
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
    portYIELD();
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
