#pragma once
#include "MotorDriver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "PID.h"
class MotorController {
public:
    MotorController(void);
    MotorController(MotorDriver *motor);
    void attachMotorDriver(MotorDriver *motor);
    void init(void);
    void setVelocity(int velocity);
    void setPosition(int64_t position, int16_t velocity);
    void setPosition(int64_t position);

    void setPIDCalculationPeriod(uint16_t periodMs);
    int getVelocity(void);
    int getPosition(void);

    uint16_t getPIDCalculationPeriod(void);

    void run();
    
   // void setVelocity(int velocity);
   

    PIDController<int> _velPID;
    PIDController<int> _posPID;
   
// private:

    // double _p = 1.0;
    // double _i = 0;
    // double _d = 0;

    typedef enum Mode
    {
        VELOCITY = 0,
        POSITION ,
        VEL_POSITION

    }Mode_e;

    Mode_e _mode;
    uint16_t _pidCalculationPeriodMs = 5;
    int16_t _targetVelocity;
    int64_t _targetPosition;
    MotorDriver *_motor = NULL;
    // IPIDController *_velocityPID = NULL;
    // IPIDController *_positionPID = NULL;

   
    //PIDController<int> _pos(_p, _i, _d, getVelocity, run);

    TaskHandle_t _pidCalculationTaskHandle;
};