#pragma once

#include "CANHAL.h"
#include "driver/twai.h"
// ESP32ƽ̨�µ�CANӲ������ʵ���࣬���ֻ������ܲ���
class CAN : public CANHAL 
{
private:
    twai_general_config_t g_config;
    twai_timing_config_t t_config;
    twai_filter_config_t f_config;
   

public:
    CAN(gpio_num_t tx_pin, gpio_num_t rx_pin, int baud);
    bool init() override ;

    bool sendFrame(const void* p_frame) override;
    bool receiveFrame(void* p_frame) override ;
};