#pragma once
#include "DriverHAL.h"
#include "EncoderHAL.h"
#include "Arduino.h"
class MotorDriver 
{
public:
    MotorDriver(EncoderHAL *encoder, DriverHAL *driver);
    void init();
    void run(int16_t velocity);
    void stop();
    int16_t getVelocity();
    int64_t getPosition();
    void cleaPosition();

    void attachDriver(DriverHAL *driver);
    void attachEncoder(EncoderHAL *encoder);
private:

    EncoderHAL *_encoder = NULL;
    DriverHAL *_driver = NULL;

    int64_t _position = 0;
    int64_t _lastPosition = 0;
    int64_t _count = 0;
    int64_t _lastCount = 0;
    int16_t _velocity = 0;

};
