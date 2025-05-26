#pragma once

#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ServoController.h"
#include "DendoStepper.h"
#include "TMC2300.h"
#include "ArmController.h"
#include <cmath>
#include <cassert>
#include <limits>


class ScaraController : public ArmController{

  public:
    ScaraController(ServoController *servo, DendoStepper *stepper, float l1, float l2);
    ScaraController(ServoController *servo, DendoStepper *stepper, TMC2300 *tmc2300,  float l1, float l2);
        
    void init(void) override;
    bool moveTo(float x, float y, float z) override;
    bool moveTo(float x, float y) override;
    bool moveZ(float z) override;
    void setHomeZ(void) override;
    void setLimitPin(uint8_t pin1, uint8_t pin2);
    bool isArrived(void);
    bool goHomeZ(void) override;
    bool getCoordinate(float *x, float *y, float *z) override;
    bool getCoordinate(void) override;

  private:

    float _L1 = 70;
    float _L2 = 70;
    float xErr = 110;
    uint8_t _servo1ID = 3;
    uint8_t _servo2ID = 4;

    const uint16_t _servo1BaseAngle = (1835+1092);// base 2008 1852
    const uint16_t _servo2BaseAngle = (2365+155);// base 1845 2552

    const int16_t _theta1Base = 135;//130
    const int16_t _theta2Base = 85;

    const static uint8_t _IDN = 2;
    uint8_t _ID[_IDN];
    int16_t _position[_IDN];
    uint16_t _velocity[_IDN];
    uint8_t _acc[_IDN];

    ServoController *_servo  = NULL;
    DendoStepper *_stepper = NULL;
    TMC2300 *_tmc2300 = NULL;

    uint8_t _limitPin1;
    uint8_t _limitPin2;

    bool inverseKinematics(float x, float y);
    bool linearCompensation(float x1, float y1, float x2, float y2);

};

