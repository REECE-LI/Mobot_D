#pragma once
#include "MotorController.h"
#include "MotorDriver.h"
#include "Encoder_ESP32.h"
#include "TB67H450_ESP32.h"
#include "ServoController.h"
#include "ChassisController.h"
#include "ArmController.h"
#include <FastLED.h>
#include "Wire.h"


typedef struct 
{
    float x;
    float y;
    float angle;
}Coordinate_t;


typedef enum
{
    BLACK,
    BLUE,
    LIGHT_BLUE,
    YELLOW

}DrawColor_t;

typedef enum
{
    CLOSE,
    OPEN

}PenState_t;

class Robot {
public:
    Robot();
    Robot(ChassisController *chassis, ArmController *arm);
   
    void init(void);

    void moveChassis(float vx, float vy);
    void moveChassis(float vx, float vy, float spin);

    bool moveChassisTo(float x, float y);
    bool moveChassisTo(float x, float y, float angle);

    void stopChassis(void);
    void goHome();
    void goCharge();

    void setChassisActualValue(float actualX, float actualY, float actualAngle);
    void setArmZBound(int up, int down);

    bool moveArmTo(float x, float y, float z);
    bool moveArmTo(float x, float y);
    bool moveArmZ(float z);
    bool getArmCoordinate();

    void attachRGB( CFastLED *RGB, CRGB *leds, uint16_t LEDNum);
    //void attachRGB(uint16_t num);
    void attachChassisController(ChassisController *chassis);
    void attachArmController(ArmController *arm);

    void breatheRGB(uint8_t color, uint16_t time);
    void blinkRGB(uint16_t time);

    void drawPoint(uint16_t x, uint16_t y);
    void draw64x64(uint8_t ((*data)[8]), uint8_t armInterval);
    void draw16x16(uint8_t ((*data)[2]));
    void draw8x8(uint8_t (*data));
    void drawPicture(float startX, float startY, float drawX, float drawY, uint16_t width, uint16_t hight,  uint8_t *data);
    void drawPictureHight16(float startX, float startY, uint16_t width, uint16_t hight, uint8_t drawX, uint8_t *data);
    void drawPictureHight8(float startX, float startY, uint16_t width, uint16_t hight, uint8_t drawX, uint8_t *data);
    void drawLoop(float startX, float startY, uint16_t width, uint16_t hight,float drawX, float drawY,uint8_t pen);
    void takePen(DrawColor_t id);
    void givePen(DrawColor_t penId);
    bool sendChangeCmd(DrawColor_t id, PenState_t state);
    bool receiveChangeACK(char *buff);
    int8_t findStartX(uint16_t width, uint16_t hight, uint8_t drawX, uint8_t *data);

    float ChassisTargetX = 800.0f;
    float ChassisTargetY = 600.0f;
    float ChassisTargetAngle = 330.0f;

    float ChassisActualX = 800.0f;
    float ChassisActualY = 600.0f;
    float ChassisActualAngle = 330.0f;

    float ArmTargetX = 70.0f;
    float ArmTargetY = 70.0f;
    float ArmTargetZ = 0.0f;

    float ArmActualX = 70.0f;
    float ArmActualY = 70.0f;
    float ArmActualZ = 0.0f;

    int ArmUp = -6000;
    int ArmDown = -23000;


     // 取笔位置
    // Coordinate_t _change = {.x = 2115.0f , .y = 225.0f, .angle = ChassisTargetAngle};
    Coordinate_t _change = {.x = 2128.0f , .y = 240.0f, .angle = ChassisTargetAngle};
    Coordinate_t _waitChange = {.x = 2100.0f , .y = 255.0f, .angle = ChassisTargetAngle};
    uint8_t *_drawBuff[4] = {NULL}; 

    
    ChassisController *_chassis = NULL;
    ArmController *_arm = NULL;

private:

    uint8_t drawBuffer[64][8] = {0};
    uint8_t drawBuffer16[16][2] = {0};
    uint8_t drawBuffer8[8] = {0};
    CFastLED *_RGB = NULL ;
    CRGB *_leds = NULL;
    uint16_t _LEDNum = 1;
   
};