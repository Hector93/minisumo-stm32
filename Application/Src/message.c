#include "message.h"
#include "FreeRTOS.h"
#include <string.h>

message createMessage(uint8_t IdpO, uint8_t IdpD,uint8_t type,uint16_t data){
  message aux;
  aux.messageUser.IdpO = IdpO;
  aux.messageUser.IdpD = IdpD;
  aux.messageUser.type = type;
  aux.messageUser.data = data;
  return aux;
}

//TODO crear funcion que envie el mensaje

message messageDinamicArray(uint8_t IdpO, uint8_t IdpD, uint8_t type, void* array,uint8_t size){
  uint8_t *dest;
  dest = (uint8_t*)pvPortMalloc(size + 6);
  dest[0] = ARRAY;
  dest[1] = IdpO;
  dest[2] = IdpD;
  dest[3] = type;
  memcpy(&dest[4], array, size);
  dest[size + 4] = '\r';
  dest[size + 5] = '\n';
  message aux;
  aux.pointer.array = dest;
  aux.pointer.size = size + 6;
  return aux;
}
