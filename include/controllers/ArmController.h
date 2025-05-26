#pragma once

#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ServoController.h"
#include "DendoStepper.h"
#include "TMC2300.h"

class ArmController {
  public:

    virtual void init() = 0;
    virtual bool moveTo(float x, float y, float z) = 0;
    virtual bool moveTo(float x, float y) = 0;
    // virtual bool moveX(float x) = 0;
    // virtual bool moveY(float y) = 0;
    virtual void setHomeZ(void) = 0;
    virtual bool goHomeZ(void) = 0;
    virtual bool moveZ(float z) = 0;
    virtual bool getCoordinate(float *x, float *y, float *z) = 0;
    virtual bool getCoordinate() = 0;
  
  protected:

    float _x = 32.0f;
    float _y = 32.0f;
    float _z;

  private:


};

