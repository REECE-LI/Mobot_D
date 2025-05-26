#include "TMC2300.h"

TMC2300::TMC2300(HardwareSerial *ser, uint8_t address):serial(ser), nodeAddress(address) {}

void TMC2300::begin(uint32_t baudrate ) {
  serial->begin(baudrate);
}

uint8_t TMC2300::calculateCRC(uint8_t *data, uint8_t length) {
  uint8_t crc = 0;
  // for (uint8_t i = 0; i < length; i++) {
  //   crc ^= data[i];
  //   for (uint8_t j = 0; j < 8; j++) {
  //     if (crc & 0x80) {
  //       crc = (crc << 1) ^ 0x07;
  //     } else {
  //       crc <<= 1;
  //     }
  //   }
  // }
  uint8_t currentByte;

  for (int i=0; i<(length); i++)
  { // Execute for all bytes of a message
      currentByte = data[i]; // Retrieve a byte to be sent from Array
      for (int j=0; j<8; j++)
      {
          if ((crc >> 7) ^ (currentByte&0x01)) // update CRC based result of XOR operation
          {
              crc = (crc << 1) ^ 0x07;
          }
          else
          {
              crc = (crc << 1);
          }
          currentByte = currentByte >> 1;
      } // for CRC bit
  } // for message byte
  return crc;
}

// 写寄存器（32位数据）
bool TMC2300::writeRegister(uint8_t reg, int32_t data) {
  uint8_t buffer[8];
  // 同步头：0x05（包含保留位）
  buffer[0] = 0x05;
  buffer[1] = nodeAddress;     // 节点地址
  buffer[2] = reg | 0x80;      // 寄存器地址 + 写标志
  buffer[3] = (data >> 24) & 0xFF; // 数据高位在前
  buffer[4] = (data >> 16) & 0xFF;
  buffer[5] = (data >> 8) & 0xFF;
  buffer[6] = data & 0xFF;
  buffer[7] = calculateCRC(buffer, 7); // 计算前7字节的CRC

  serial->write(buffer, 8);
  return true; // 实际应用中应检查ACK
}

// 读寄存器（返回32位数据）
int32_t TMC2300::readRegister(uint8_t reg) {

  uint8_t txBuffer[4];
  txBuffer[0] = 0x05;          // 同步头
  txBuffer[1] = nodeAddress;   // 节点地址
  txBuffer[2] = reg & 0x7F;    // 寄存器地址 + 读标志
  txBuffer[3] = calculateCRC(txBuffer, 3);

  serial->write(txBuffer, 4);
  delay(1); // 等待响应

  // 读取8字节响应
  uint8_t rxBuffer[8];
  if (serial->readBytes(rxBuffer, 8) != 8)
  {
    return 0xFFFFFFFF;
  } 

  // 校验CRC（响应最后1字节）
  if (calculateCRC(rxBuffer, 7) != rxBuffer[7])
  {
    return 0xFFFFFFFF;
  } 

  // 组合32位数据
  return ((uint32_t)rxBuffer[3] << 24 |
        (uint32_t)rxBuffer[4] << 16 |
        (uint32_t)rxBuffer[5] << 8 |
        rxBuffer[6]);
}

// 示例：设置运行电流（IRUN）和保持电流（IHOLD）
void TMC2300::setCurrent(uint8_t ihold, uint8_t irun) {
  uint32_t data = (ihold & 0x1F) | ((irun & 0x1F) << 8);
  writeRegister(TMC2300_IHOLD_IRUN, data);
}

void TMC2300::setVelocity(int32_t velocity) {
  writeRegister(TMC2300_VACTUAL, velocity);
}
// 示例：读取驱动状态
uint32_t TMC2300::getDriverStatus() {
  return readRegister(TMC2300_DRV_STATUS);
}



