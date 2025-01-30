
#ifndef _MAIN_H_
#define _MAIN_H_

#include "Arduino.h"


enum COMMAND_STATE {
  EXECUTING = 0,
  SUCCESS = 88,
  ERROR = 44
};

typedef struct {
  unsigned int state;
  char node[5];
  char func[6];
  unsigned char crc;
  char msg[255];
} Command;



Command curr_command;

bool unpack_command(Command *command);
String pack_command(Command *command);
uint16_t modbus_crc16(uint8_t *data, uint16_t len);

bool handle_do(Command *command);

void reload_dog(void) ;
void xTask_watchdog(void *xTask1);
void xTask_handle(void *xTask1);


#endif