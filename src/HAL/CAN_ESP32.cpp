#include "CAN_ESP32.h"
#include "driver/twai.h"
// ESP32平台下的CAN硬件抽象实现类，保持基本功能不变

  CAN::CAN(gpio_num_t tx_pin, gpio_num_t rx_pin, int baud)
  {
    g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_pin, rx_pin, TWAI_MODE_NORMAL);
    f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    switch (baud)
    {
    case 100:
      t_config = TWAI_TIMING_CONFIG_100KBITS();
      break;
    
    default:
      t_config = TWAI_TIMING_CONFIG_1MBITS();
      break;
    }

    init();
  }

  bool  CAN::init() 
  {
    if (twai_driver_install(&g_config, &t_config, &f_config)!= ESP_OK) 
    {
          return false;
    }
    
    if (twai_start()!= ESP_OK) 
    {    
        return false;
    }

    return true;      
  }

  bool  CAN::sendFrame(const void* p_frame) 
  {
      const twai_message_t *message = static_cast<const twai_message_t *>(p_frame);
      return (twai_transmit(message, pdMS_TO_TICKS(10)));
  }

  bool  CAN::receiveFrame(void* p_frame) 
  {
    twai_message_t *message = static_cast<twai_message_t *>(p_frame);
    if (p_frame == nullptr) 
    {
      return false;
    }
    int res = twai_receive(message, pdMS_TO_TICKS(10));
    return (res == ESP_OK);
  }