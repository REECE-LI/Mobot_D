#ifndef _MAIN_H
#define _MAIN_H
#include "SimpleSerialShell.h"
#include <FastLED.h>
#include "SCServo.h"
#include "Servo16.h"
#include "Robot.h"
#include "picture.h"
#include "Wire.h"

#define NUM_LEDS 8
#define DATA_PIN 41
// #define CLOCK_PIN 13

#define GDW_PIN 21

#define MOTOR_EN 6 // IOT Kit is IO32
//#define STANDBY 5 // IOT Kit is IO5
#define MOTOR_STEP 9
#define MOTOR_DIR 10
#define MOTOR_MODE 5
#define MOTOR_MS1 8
#define MOTOR_MS2 7

#define ENCODER1_PIN_A 39
#define ENCODER1_PIN_B 40

#define ENCODER2_PIN_A 47
#define ENCODER2_PIN_B 48

#define DRIVER1_PIN_1 37
#define DRIVER1_PIN_2 38
#define DRIVER1_PIN_C 36

#define DRIVER1_PWM1_CH LEDC_CHANNEL_0
#define DRIVER1_PWM2_CH LEDC_CHANNEL_1
#define DRIVER1_PWM3_CH LEDC_CHANNEL_2

#define DRIVER2_PIN_1 34
#define DRIVER2_PIN_2 35
#define DRIVER2_PIN_C 33

#define DRIVER2_PWM1_CH LEDC_CHANNEL_3
#define DRIVER2_PWM2_CH LEDC_CHANNEL_4
#define DRIVER2_PWM3_CH LEDC_CHANNEL_5

#define SERIAL0_PIN_RX 15
#define SERIAL0_PIN_TX 16

#if 1
#define SERIAL1_PIN_RX 44
#define SERIAL1_PIN_TX 43
#else
#define SERIAL1_PIN_RX 14
#define SERIAL1_PIN_TX 13
#endif

#define I2C_SCK 14
#define I2C_SDA 13

#define SERIAL2_PIN_RX 11
#define SERIAL2_PIN_TX 12

#define LIMIT_PIN_1 17
#define LIMIT_PIN_2 18

#define USER_PIN   0 
#define DOWN_BIT   BIT0
#define UP_BIT     BIT1
#define RUN_BIT    BIT2
#define CHASSISRUN_BIT    BIT3

#define SLAVE_ADDRESS 0x08

typedef struct 
{
  union 
  {
    uint8_t data[24];
    struct 
    {
      int x;
      int y;
      float angle;

      float target_x;
      float target_y;
      float target_z;
    };
    
  };
  /* data */
}CamData_t;

extern SMS_STS sms_sts;
extern CRGB leds[NUM_LEDS];
extern Servo gdw;
extern Robot mobot;//(&chassis, &armR);
extern uint8_t moveTimes;
extern uint8_t fontWidth;
extern uint8_t isWriting;
#endif // !_MAIN_H
