#include "TB67H450_ESP32.h"


TB67H450::TB67H450(uint8_t in1Pin, uint8_t in2Pin, uint8_t currentPin,  uint8_t in1Channel,  uint8_t in2Channel,  uint8_t currentChannel)
{
    _in1Pin = in1Pin;
    _in2Pin = in2Pin;
    _currentPin = currentPin;
    _in1Channel = in1Channel;
    _in2Channel = in2Channel;
    _currentChannel = currentChannel;

    //init();

}

 void TB67H450::init(void)
 {
    ledcSetup(_in1Channel, _in1Freq, _in1resolutionBits);
    ledcAttachPin(_in1Pin, _in1Channel);

    ledcSetup(_in2Channel, _in2Freq, _in2resolutionBits);
    ledcAttachPin(_in2Pin, _in2Channel);

    ledcSetup(_currentChannel, _currentFreq, _currentResolutionBits);
    ledcAttachPin(_currentPin, _currentChannel);

    setCurrent(1<<_currentResolutionBits);
 }

void TB67H450::setIN1PWM(uint16_t duty)
{
   ledcWrite(_in1Channel, duty);

}

void TB67H450::setIN2PWM(uint16_t duty)
{
   ledcWrite(_in2Channel, duty);

}

void TB67H450::setCurrentPWM(uint16_t duty)
{
   ledcWrite(_currentChannel, duty);

}