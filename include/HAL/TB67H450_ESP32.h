#pragma once
#include <cstdint>
#include "TB67H450HAL.h"
#include "Arduino.h"
#include "driver/ledc.h"

//ledc_timer_config_t

class TB67H450 : public TB67H450HAL
{
public:
   
    TB67H450(uint8_t in1Pin, uint8_t in2Pin, uint8_t currentPin,  uint8_t in1Channel,  uint8_t in2Channel,  uint8_t currentChannel);
    TB67H450(ledc_timer_config_t *ledc_timer_config );

    void init(void);

private:

     //void setPins(uint8_t in1Pin, uint8_t in2Pin, uint8_t currentPin) override;
    void setIN1PWM(uint16_t duty) override;
    void setIN2PWM(uint16_t duty) override;
    void setCurrentPWM(uint16_t duty) override;

    uint8_t _in1Pin;
    uint8_t _in2Pin;
    uint8_t _currentPin;

    uint8_t _in1Channel;
    uint8_t _in2Channel;
    uint8_t _currentChannel;

    uint32_t _in1Freq = 10000;
    uint32_t _in2Freq = 10000;
    uint32_t _currentFreq = 10000;

    uint8_t _in1resolutionBits = 10;
    uint8_t _in2resolutionBits = 10;
    uint8_t _currentResolutionBits = 10;

};
