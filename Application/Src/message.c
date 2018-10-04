#include "message.h"

message createMessage(uint8_t Idp,uint8_t type,uint16_t data){
  message aux;
  aux.messageUser.Idp = Idp;
  aux.messageUser.type = type;
  aux.messageUser.data = data;
  return aux;
}
