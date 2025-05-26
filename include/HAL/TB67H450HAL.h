#pragma once
#include <cstdint>
#include "DriverHAL.h"

class TB67H450HAL : public DriverHAL
{
public:
    explicit TB67H450HAL() = default;
    void init() override;
    void forward(uint16_t) override;
    void reverse(uint16_t) override;
    void stop() override;
    void brake() override;

    void setCurrent(uint16_t);

protected:

    virtual void setIN1PWM(uint16_t duty) = 0;
    virtual void setIN2PWM(uint16_t duty) = 0;
    virtual void setCurrentPWM(uint16_t duty) = 0;

private:

  
};

