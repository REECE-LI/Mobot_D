#include "Encoder_ESP32.h"
#include <driver/gpio.h>
#include <ESP32Encoder.h>

Encoder::Encoder(int aPin,  int bPin)
 {
    _aPin = aPin;
    _bPin = bPin;

    //init();
 }

void Encoder::init() 
{
    encoder.attachFullQuad(_aPin, _bPin);//attachFullQuad
    encoder.setCount(0);
    encoder.clearCount();
    encoder.useInternalWeakPullResistors = puType::up;
}

void Encoder::setPins(int aPin, int bPin)
{
    _aPin =  aPin;
    _bPin =  bPin;      
}

void Encoder::setCount(int64_t vlue) 
{
    encoder.setCount(vlue);   
}

int64_t Encoder::getCount() 
{
    return encoder.getCount();
}

void Encoder::cleaCount() 
{
    encoder.clearCount();
}
