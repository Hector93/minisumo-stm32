

extern osSemaphoreId serialSemHandle;
extern  osMessageQId serialQueueHandle;
extern BaseType_t __real_xQueueGenericSend( QueueHandle_t xQueue, const void * const pvItemToQueue, TickType_t xTicksToWait, const BaseType_t xCopyPosition );

BaseType_t __wrap_xQueueGenericSend( QueueHandle_t xQueue, const void * const pvItemToQueue, TickType_t xTicksToWait, const BaseType_t xCopyPosition ){
  BaseType_t aux;
  return __real_xQueueGenericSend(xQueue, pvItemToQueue, xTicksToWait, xCopyPosition);
  if(pvItemToQueue == 0x0){
    return __real_xQueueGenericSend(xQueue, pvItemToQueue, xTicksToWait, xCopyPosition);
  }
  aux = __real_xQueueGenericSend(xQueue, pvItemToQueue, xTicksToWait, xCopyPosition);
  if(xQueue == serialQueueHandle){
    xSemaphoreGive(serialSemHandle);
  }  
  return __real_xQueueGenericSend(xQueue, pvItemToQueue, xTicksToWait, xCopyPosition);
}

