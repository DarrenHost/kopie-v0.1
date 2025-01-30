#ifndef _USR_IO_LIB_H_
#define _USR_IO_LIB_H_

#include "Arduino.h"

class USR_IO
{
public:
  void init();
  void do_set(char slot_num, char io_num, char level);
  //设定ao输出是电流还是电压，mode=0输出为电流 mode=1 输出为电压。
  void ao_config(char slot_num, char io_num, char mode);
  //设定ao输出电流大小 current_value的值限定为4 5 10 20 单位是毫安。
  void ao_current_set(char slot_num, char io_num, char current_value);
  void ao_volt_set(char slot_num, char io_num, char volt_level);
  char di_get(char slot_num, char io_num);
  float ai_get(char slot_num, char io_num);
  void log_on(char log);
  void print();
  void debug_list_slot();
private:
  ;
};

#endif