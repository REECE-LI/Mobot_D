//
// Created by L2248 on 2025/6/6.
//

#include "main.h"

void ChassisControlTask(void *pvParameters) // void *pvParameters
{
    const TickType_t xPeriod = pdMS_TO_TICKS(5); // 5ms周期
    TickType_t xLastWakeTime = xTaskGetTickCount(); // 获取当前tick作为初始时间点
    static int16_t leftOutput = 0;
    static int16_t rightOutput = 0;
    static int16_t leftVel = 0;
    static int16_t rightVel = 0;
    static int64_t leftPos = 0;
    static int64_t rightPos = 0;
    static int16_t rollAng = 0;
    static int16_t speedPos = 0;
    static float distance = 0;
    static float angleRad = 0;
    static float angleDeg = 0;
    static float angle = 0;
    static int16_t timeCount = -80;
    static float deltaX = 0;
    static float deltaY = 0;
    static float targetAngle = 0;
    static bool isRetarget = false;

    while (true) {
        // 等待下一个周期（绝对时间）
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        timeCount++;


        leftVel = (int16_t) mobot._rudder->_wheelL->getVelocity();
        rightVel = (int16_t) mobot._rudder->_wheelR->getVelocity();

        // deltaX = mobot.targetPos.x - mobot.actualPos.x;
        //
        // deltaY = mobot.targetPos.y - mobot.actualPos.y;
        //
        // if (abs(deltaY < 5)) {}
        // distance = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
        // mobot.Distance = distance;
        // if (deltaY <= 0)
        //     speedPos = (int16_t) mobot.PidTick(&mobot.robotPosUp, 0, -distance);
        // else
        //     speedPos = (int16_t) mobot.PidTick(&mobot.robotPosDown, 0, +distance);
        //
        // mobot.wheelSpeed.LeftVel = speedPos;
        // mobot.wheelSpeed.RightVel = -speedPos;
        //
        // angle = (int16_t) mobot.PidTick(&mobot.servoPos, 0, mobot.targetPos.x - mobot.actualPos.x);
        // if (deltaY <= 0)
        //     mobot._rudder->setWheelAngle(angle);
        // else
        //     mobot._rudder->setWheelAngle(-angle );

        mobot.wheelSpeed.LeftVel = -mobot.Speed;
        mobot.wheelSpeed.RightVel = mobot.Speed;
        mobot._rudder->setWheelAngle(mobot.rollAngle);

        // 保持水平
        rollAng = (int16_t) mobot.PidTick(&mobot.robotAngle, targetAngle, mobot.ChassisActualAngle);

        rollAng = 0;
        mobot.targetVel.LeftVel = mobot.wheelSpeed.LeftVel + rollAng;
        mobot.targetVel.RightVel = mobot.wheelSpeed.RightVel + rollAng;

        leftOutput = (int16_t) mobot.PidTick(&mobot.leftVel, mobot.targetVel.LeftVel, -leftVel);
        rightOutput = (int16_t) mobot.PidTick(&mobot.rightVel, mobot.targetVel.RightVel, rightVel);
        // print vel


        // if (leftOutput > 1200)
        //     leftOutput = 1200;
        // else if (leftOutput < -1200)
        //     leftOutput = -1200;
        //
        // if (rightOutput > 1200)
        //     rightOutput = 1200;
        // else if (rightOutput < -1200)
        //     rightOutput = -1200;

        mobot._rudder->_wheelL->_motor->run(leftOutput);
        mobot._rudder->_wheelR->_motor->run(rightOutput);


        // Serial1.printf("leftVel = %d, rightVel = %d, leftOutput = %d,rightOutput = %d\r\n",
        //     leftVel, rightVel, leftOutput, rightOutput);

        // Serial1.printf("leftVel = %d, rightVel = %d\r\n",
        //                leftVel, rightVel);
    }
}
