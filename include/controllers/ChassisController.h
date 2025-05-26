#pragma once
#include "MotorController.h"
#include "MotorDriver.h"
#include "Encoder_ESP32.h"
#include "TB67H450_ESP32.h"
#include "ServoController.h"
class ChassisController {
public:
    virtual void init(void) = 0;
    virtual void move(float vx, float vy, float vspin) = 0;
    virtual void turn(float vspin) = 0;
    virtual bool turnTo(float targetAngle) = 0;
    virtual bool moveTo(float targetX, float targetY) = 0;
    virtual bool moveTo(float targetX, float targetY, float targetAngle) = 0;
    void setActualValue(float actualX, float actualY, float actualAngle);
    void setTargetValue(float targetX, float targetY, float targetAngle);
    virtual bool isArrived(void) = 0;
    virtual void stop(void) = 0;
    virtual void disable(void) = 0;

protected:

    float _targetX = 1500.0f;
    float _targetY = 700.0f;
    float _targetAngle = 330.0f;

    float _horizontalAngle = 330.0f;
    float _actualX = 1500.0f;
    float _actualY = 700.0f;
    float _actualAngle = 330.0f;

private:

};