#ifndef _APP_UTILS_H_
#define _APP_UTILS_H_

#include "Arduino.h"

//工具类
class UTILS{
   //CRC16 计算
    public:
    static uint16_t modbus_crc16(uint8_t *data, uint16_t len);
};


#endif