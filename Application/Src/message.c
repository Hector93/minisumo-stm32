#include "message.h"

message createMessage(uint8_t IdpO, uint8_t IdpD,uint8_t type,uint16_t data){
  message aux;
  aux.messageUser.IdpO = IdpO;
  aux.messageUser.IdpD = IdpD;
  aux.messageUser.type = type;
  aux.messageUser.data = data;
  return aux;
}
