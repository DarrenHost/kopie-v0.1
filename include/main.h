
#ifndef _APP_MAIN_H_
#define _APP_MAIN_H_

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
  char arg [128];
  char msg [128];
  char res [128];
  unsigned char crc;
  
} Command;


//拆包
bool unpack_command(Command *command);

//打包
String pack_command(Command *command);

//处理 GPIO 操作
bool handle_do(Command *command);

//处理 初始化硬件
bool handle_init(Command *command);

//处理 水路 操作 
bool handle_water(Command *command);

//处理 粉路 操作 
bool handle_granule(Command *command);

//喂狗
void reload_dog(void) ;

//FreeOS 看门狗线程
void xTask_watchdog(void *xTask1);

//FreeOS 命令执行线程
void xTask_handle(void *xTask1);



#endif