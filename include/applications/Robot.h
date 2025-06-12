#pragma once
#include "MotorController.h"
#include "MotorDriver.h"
#include "Encoder_ESP32.h"
#include "TB67H450_ESP32.h"
#include "ServoController.h"
#include "ChassisController.h"
#include "ArmController.h"
#include <FastLED.h>
#include <RudderWheelController.h>

#include "Wire.h"

#define RotateAngle (-4) // 5 degree
#define RotateRad (RotateAngle /180.0f * M_PI) // 5 degree to radian


typedef struct {
    float x;
    float y;
    int16_t angle;
} Coordinate_t;


typedef enum {
    BLACK,
    BLUE,
    LIGHT_BLUE,
    YELLOW
} DrawColor_t;

typedef enum {
    CLOSE,
    OPEN
} PenState_t;

class Robot {
public:
    Robot();

    Robot(RudderWheelController *rudder, ArmController *arm);

    void init(void);


    void goHome();


    bool moveArmTo(float x, float y, float z);

    bool moveArmTo(float x, float y);

    bool moveArmZ(float z);

    bool getArmCoordinate();

    void ArmMoveDown();

    void ArmMoveUp();

    void attachRGB(CFastLED *RGB, CRGB *leds, uint16_t LEDNum);

    //void attachRGB(uint16_t num);
    void attachChassisController(ChassisController *chassis);

    void attachArmController(ArmController *arm);

    void breatheRGB(uint8_t color, uint16_t time);

    void blinkRGB(uint16_t time);

    void WriteWord(uint8_t _ID);


    // float ChassisTargetX = 800.0f;
    // float ChassisTargetY = 600.0f;
    int16_t ChassisTargetAngle = 330.0f;
    //
    // float ChassisActualX = 800.0f;
    // float ChassisActualY = 600.0f;
    int16_t ChassisActualAngle = 0.0f;

    float chassisConvertAngle = 0.0f;

    typedef struct
    {
        float kp, ki, kd;
        float vError, vErrorLast;
        float outputKp, outputKi, outputKd;
        float integralRound;
        float integralRemainder;
        float output;


        float errorBound; // 错误限制
        float outputBound; // 输出限制
    } PID_t;

    struct wheelVel_t {
        int16_t LeftVel = 0;
        int16_t RightVel = 0;
    };

    struct wheelPos_t {
        int64_t LeftPos = 0;
        int64_t RightPos = 0;
    };

    struct Pos_t {
        float x = 0;
        float y = 0;
        bool isWrite = false;
    };

    wheelVel_t targetVel;
    wheelVel_t wheelSpeed;
    Pos_t targetPos;
    Pos_t actualPos;
    Pos_t readyPos;

    float ArmTargetX = 70.0f;
    float ArmTargetY = 70.0f;
    float ArmTargetZ = 0.0f;

    float ArmActualX = 70.0f;
    float ArmActualY = 70.0f;
    float ArmActualZ = 0.0f;


    // 抬笔
    int ArmUp = -1;
    //  写字
    int ArmDown = -8000;


    // 取笔位置
    // Coordinate_t _change = {.x = 2115.0f , .y = 225.0f, .angle = ChassisTargetAngle};
    // Coordinate_t _change = {.x = 2128.0f , .y = 240.0f, .angle = ChassisTargetAngle};
    Coordinate_t _change = {.x = 2045.0f, .y = 245.0f, .angle = ChassisTargetAngle};
    Coordinate_t _waitChange = {.x = 2015.0f, .y = 268.0f, .angle = ChassisTargetAngle};
    uint8_t *_drawBuff[4] = {NULL};


    RudderWheelController *_rudder = NULL;
    ArmController *_arm = NULL;

    float rotateRads[4] = {
        cosf(RotateRad), (-sinf(RotateRad)),
        sinf(RotateRad), cosf(RotateRad)
    };




    PID_t leftVel = {0};
    PID_t rightVel = {0};
    PID_t robotPosUp = {0};
    PID_t robotPosDown = {0};
    // 这个是调整轮子的差速 来控制自身原地旋转的PID
    PID_t robotAngle = {0};
    // 这个是调整舵轮的PID 来控制舵轮的角度
    PID_t servoPos = {0};


    PID_t robotLeftPos = {0};
    PID_t robotRightPos = {0};

    wheelPos_t targetPosWheel;

    wheelPos_t actualPosWheel;


    float PidTick(PID_t *pid, float target, float actual);

    //    float PidTick(PID_t *pid, float target, float actual);

    float Distance;

    float servoAngleOffset = 0;

    int16_t Speed = 0;

    int16_t rollAngle;

    bool dir;

    bool en;

// private:
    uint8_t drawBuffer[64][8] = {0};
    uint8_t drawBuffer16[16][2] = {0};
    uint8_t drawBuffer8[8] = {0};
    CFastLED *_RGB = NULL;
    CRGB *_leds = NULL;
    uint16_t _LEDNum = 1;
};
