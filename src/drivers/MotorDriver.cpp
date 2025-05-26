#include "MotorDriver.h"

MotorDriver::MotorDriver(EncoderHAL *encoder, DriverHAL *driver) 
{
    _encoder = encoder;
    _driver = driver;
    //init();
}


void MotorDriver::attachDriver(DriverHAL *driver)
{
    _driver = driver;
}

void MotorDriver::attachEncoder(EncoderHAL *encoder)
{
    _encoder = encoder;
}

void MotorDriver::init() 
{
    _encoder->init();
    _encoder->cleaCount();
    _driver->init();
    _driver->stop();
}

void MotorDriver::run(int16_t velocity) 
{
   if (velocity >= 0)
   {
        _driver->forward(velocity);
   }
   else 
   {
        _driver->reverse(-velocity);
   }
}

void MotorDriver::stop() 
{
    _driver->stop();
}

int16_t MotorDriver::getVelocity() 
{   
    _count =  _encoder->getCount();
    _velocity = _count - _lastCount;
    _lastCount = _count;
  
    return _velocity;
}

int64_t MotorDriver::getPosition() 
{   
    return _encoder->getCount();
}

void MotorDriver::cleaPosition() 
{
    
    _encoder->cleaCount();
}