#pragma once
#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "SCServo.h"


class ServoController {
public:
    ServoController();
    ServoController(HardwareSerial *pSerial);
    void attachSerial(HardwareSerial *pSerial);
    void init(void);
    void setVelocity(int16_t velocity);
    void setPosition(uint8_t id, int16_t position, int16_t velocity);
    void setPositionSync(uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Speed[], uint8_t ACC[]);
    int getPosition(uint8_t id);

// private:
    SMS_STS _servo_sts;
    int16_t _targetVelocity;
    int64_t _targetPosition;
    
};