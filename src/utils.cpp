#include "utils.h"

// 计算CRC16函数
uint16_t UTILS::modbus_crc16(uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF; // 初始值
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8; // 将数据字节与CRC值进行XOR操作
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) { // 检查最高位是否为1
        crc = (crc << 1) ^ 0xA001; // 如果是1，进行多项式运算
      } else {
        crc = crc << 1; // 如果不是，则左移
      }
    }
  }
  return crc; // 返回最终的CRC值
}