#pragma once
#include <cstdint>


class DriverHAL
{
  public:
   virtual void init() = 0;
   virtual void forward(uint16_t) = 0;
   virtual void reverse(uint16_t) = 0;
   virtual void stop() = 0;
   virtual void brake() = 0;
  private:

};

