#include "TB67H450HAL.h"
#include <driver/mcpwm.h>  // 假设使用ESP32的MCPWM模块控制电机，引入相关库
#include <esp32mcpwmmotor.h>


void TB67H450HAL::init()  
{

    

}

void TB67H450HAL::forward(uint16_t speed)  
{
    setIN1PWM(0);
    setIN2PWM(speed); 
}

void TB67H450HAL::reverse(uint16_t speed)  
{
    setIN1PWM(speed);
    setIN2PWM(0); 
}

void TB67H450HAL::setCurrent(uint16_t current) 
{ 
    setCurrentPWM(current);
}

void TB67H450HAL::stop()
{
    setIN1PWM(0);
    setIN2PWM(0);      
}
void TB67H450HAL::brake()
{

    setIN1PWM(1000);
    setIN2PWM(1000); 
}

