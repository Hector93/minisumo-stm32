#include <stdint.h>

typedef union{
  struct {
    uint8_t data[4];
  }messageRaw;
  struct {
    uint8_t IdpD : 4;
    uint8_t IdpO : 4;
    uint8_t type;
    uint16_t data;    
  }messageUser;
}message;

typedef struct {
  message msg;
  uint8_t syncChar;
}msgWSync;

typedef union {
  uint8_t data[5];
  msgWSync mws;
}serialPkt;

#define externalControllerID 15
#define ERROR                255
#define UNKNOWNDEST          1
#define NOTIMPLEM            2

message createMessage(uint8_t IdpO, uint8_t IdpD, uint8_t type, uint16_t data);

