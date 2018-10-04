#include <stdint.h>

typedef union{
  struct {
    uint8_t data[3];
  }messageRaw;
  struct {
    uint8_t Idp;
    uint8_t type;
    uint16_t data;    
  }messageUser;
}message;

typedef union {
  uint8_t data[4];
  struct {
    message data;
    uint8_t syncChar;
  }sync;
}serialPkt;

message createMessage(uint8_t Idp, uint8_t type, uint16_t data);

