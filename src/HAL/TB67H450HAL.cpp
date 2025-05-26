#include "TB67H450HAL.h"
#include <driver/mcpwm.h>  // ����ʹ��ESP32��MCPWMģ����Ƶ����������ؿ�
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

