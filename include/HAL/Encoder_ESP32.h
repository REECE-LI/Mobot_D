
#pragma once
#include <cstdint>
#include "EncoderHAL.h"
#include <driver/gpio.h>
#include <ESP32Encoder.h>

class Encoder : public EncoderHAL {
public:
    
    Encoder(int aPin,  int bPin);
    void init() override;
    void setPins(int aPin, int bPin);
    void setCount(int64_t vlue) override;

    int64_t getCount() override;

    void cleaCount() override;

private:
    ESP32Encoder encoder;
    int _aPin;
    int _bPin;
};