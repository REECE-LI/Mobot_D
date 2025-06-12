#pragma once
#include "MotorController.h"
#include "MotorDriver.h"
#include "Encoder_ESP32.h"
#include "TB67H450_ESP32.h"
#include "ServoController.h"
#include "ChassisController.h"
#include "PID.h"
class RudderWheelController : public ChassisController {
public:
    RudderWheelController(void);
    RudderWheelController(ServoController *servo, MotorController *wheelR, MotorController *wheelL);
   
    void attachServo(ServoController *servo);
    void attachMotor( MotorController *wheelR, MotorController *wheelL);

    void init(void) override;
    void move(float vx, float vy, float vspin) override;
    bool moveTo(float targetX, float targetY) override;
    bool moveTo(float targetX, float targetY, float targetAngle) override;
    void turn(float vspin) override;
    bool turnTo(float targetAngle) override;
    bool isArrived(void) override; 
    void stop(void) override;
    void disable(void) override;

    void run(void);
    uint16_t getPIDCalculationPeriod(void);

    PIDController<float> _velPID;
    PIDController<float> _vspinPID;
// protected:


// private:

    void setWheelAngle(float targetAngle);
    void setWheelAngle(uint8_t id, float targetAngle);
    void setWheelVelocity(float velocity);
    void setWheelVelocity(uint8_t id, float velocity);
    void setWheelAngleVelocity(float angle, float velocity, float vspin);
    void setServoID(uint8_t id);
    void setPIDCalculationPeriod(uint16_t periodMs);
    float getActualPos(void);
    float getActualAngle(void);

    
    enum Wheel
    {
        R = 0,
        L 
    };

    float _targetWheelVelocity = 0;
    float _targetWheelAngle = 90;
    float _targetWheelVspin = 0;

    float _actualPositionErr = 0;

    uint8_t _servoID = 1;

    bool _isStop = false;
    bool _isEnable = false;
    bool _isTurn = false;
    
    uint16_t pidCalculationPeriodMs = 5;
    MotorController *_wheelR = NULL;
    MotorController *_wheelL = NULL;
    ServoController *_servo = NULL;
    TaskHandle_t pidChassisTaskHandle;
};