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

// д�Ĵ�����32λ���ݣ�
bool TMC2300::writeRegister(uint8_t reg, int32_t data) {
  uint8_t buffer[8];
  // ͬ��ͷ��0x05����������λ��
  buffer[0] = 0x05;
  buffer[1] = nodeAddress;     // �ڵ��ַ
  buffer[2] = reg | 0x80;      // �Ĵ�����ַ + д��־
  buffer[3] = (data >> 24) & 0xFF; // ���ݸ�λ��ǰ
  buffer[4] = (data >> 16) & 0xFF;
  buffer[5] = (data >> 8) & 0xFF;
  buffer[6] = data & 0xFF;
  buffer[7] = calculateCRC(buffer, 7); // ����ǰ7�ֽڵ�CRC

  serial->write(buffer, 8);
  return true; // ʵ��Ӧ����Ӧ���ACK
}

// ���Ĵ���������32λ���ݣ�
int32_t TMC2300::readRegister(uint8_t reg) {

  uint8_t txBuffer[4];
  txBuffer[0] = 0x05;          // ͬ��ͷ
  txBuffer[1] = nodeAddress;   // �ڵ��ַ
  txBuffer[2] = reg & 0x7F;    // �Ĵ�����ַ + ����־
  txBuffer[3] = calculateCRC(txBuffer, 3);

  serial->write(txBuffer, 4);
  delay(1); // �ȴ���Ӧ

  // ��ȡ8�ֽ���Ӧ
  uint8_t rxBuffer[8];
  if (serial->readBytes(rxBuffer, 8) != 8)
  {
    return 0xFFFFFFFF;
  } 

  // У��CRC����Ӧ���1�ֽڣ�
  if (calculateCRC(rxBuffer, 7) != rxBuffer[7])
  {
    return 0xFFFFFFFF;
  } 

  // ���32λ����
  return ((uint32_t)rxBuffer[3] << 24 |
        (uint32_t)rxBuffer[4] << 16 |
        (uint32_t)rxBuffer[5] << 8 |
        rxBuffer[6]);
}

// ʾ�����������е�����IRUN���ͱ��ֵ�����IHOLD��
void TMC2300::setCurrent(uint8_t ihold, uint8_t irun) {
  uint32_t data = (ihold & 0x1F) | ((irun & 0x1F) << 8);
  writeRegister(TMC2300_IHOLD_IRUN, data);
}

void TMC2300::setVelocity(int32_t velocity) {
  writeRegister(TMC2300_VACTUAL, velocity);
}
// ʾ������ȡ����״̬
uint32_t TMC2300::getDriverStatus() {
  return readRegister(TMC2300_DRV_STATUS);
}



