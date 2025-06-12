#include "Robot.h"
#include "main.h"

#include "picture.h"
#include "gcode.h"

Robot::Robot() {
    // init();
}

Robot::Robot(RudderWheelController *rudder, ArmController *arm) {
    _rudder = rudder;
    _arm = arm;
    // init();
}

void Robot::attachArmController(ArmController *arm) { _arm = arm; }



void Robot::attachRGB(CFastLED *RGB, CRGB *leds, uint16_t LEDNum) {
    _LEDNum = LEDNum;
    _leds = leds;
    _RGB = RGB;
}

void Robot::init(void) {
    if (_rudder) {
        _rudder->setTargetValue(targetPos.x, targetPos.y,
                                 ChassisTargetAngle);
        _rudder->setActualValue(actualPos.x, actualPos.y,
                                 ChassisTargetAngle);
        _rudder->init();
    }

    if (_arm) {
        _arm->init();
    }

    // _drawBuff[0] = black_288;
    // _drawBuff[1] = orange_288;
    // _drawBuff[2] = yellow_288;

    _drawBuff[0] = black_144;
    _drawBuff[1] = orange_144;
    _drawBuff[2] = yellow_144;
    _drawBuff[3] = blue_144;

    // _drawBuff[0] = blue_bytes;
    // _drawBuff[1] = red_bytes;
    // _drawBuff[2] = yellow_bytes;

    // _chassis->moveTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);

    leftVel.kp =10.6;
    leftVel.ki = 0.3;
    leftVel.kd = 0;

    leftVel.errorBound = 400;
    leftVel.outputBound = 2000;

    rightVel.kp = 10.6;
    rightVel.ki = 0.3;
    rightVel.kd = 0.0;

    rightVel.errorBound = 400;
    rightVel.outputBound = 2000;

    // 差速调整角度
    robotAngle.kp = 5.8;
    robotAngle.ki = 0.0;
    robotAngle.kd = 0.7;

    robotAngle.errorBound = 80;
    robotAngle.outputBound = 400;

    // 位置调整上升
    robotPosUp.kp = 12.1;
    robotPosUp.ki = 0.0;
    robotPosUp.kd = 2.5;

    robotPosUp.errorBound = 1500;
    robotPosUp.outputBound = 180;


    // 下降
    robotPosDown.kp = 5.0;
    robotPosDown.ki = 0.0;
    robotPosDown.kd = 0.0;

    robotPosDown.errorBound = 1500;
    robotPosDown.outputBound = 100;

    // 舵机角度
    servoPos.kp = 1.8;
    servoPos.ki = 0.0;
    servoPos.kd = 0.9;

    servoPos.errorBound = 100;
    servoPos.outputBound = 45;

    // 左轮位置
    robotLeftPos.kp = 1.6;
    robotLeftPos.ki = 0.0;
    robotLeftPos.kd = 0.05;

    robotLeftPos.errorBound = 32768;
    robotLeftPos.outputBound = 700;

    // 右轮位置
    robotRightPos.kp = 1.5;
    robotRightPos.ki = 0.0;
    robotRightPos.kd = 0.0;

    robotRightPos.errorBound = 32768;
    robotRightPos.outputBound = 700;


}


bool Robot::moveArmTo(float x, float y, float z) {
    return _arm->moveTo(x, y, z);
}

bool Robot::moveArmTo(float x, float y) {

    // x -= -25;
    // y -= 25;
    //
    // x = x * rotateRads[0] + y * rotateRads[1];
    // y = x * rotateRads[2] + y * rotateRads[3];
    //
    // x += -25;
    // y += 25;
    return _arm->moveTo(x, y);
}

bool Robot::moveArmZ(float z) { return _arm->moveZ(z); }

bool Robot::getArmCoordinate() { return _arm->getCoordinate(); }


void Robot::ArmMoveDown() {
    float distance = (-30 - _arm->_x) * (-30 - _arm->_x) + (_arm->_y - 30) * (_arm->_y - 30) * 0.4;
    _arm->moveZ(ArmDown - distance * 1.88);
}

void Robot::ArmMoveUp() {

    _arm->moveZ(ArmUp);
}

void Robot::breatheRGB(uint8_t color, uint16_t time) {
    for (int i = 10; i < 255; i++) {
        for (int j = 0; j < _LEDNum; j++) {
            _leds[j] = CHSV(color, 255, i);
        }
        _RGB->show();
        delay(time);
    }
    for (int i = 255; i > 10; i--) {
        for (int j = 0; j < _LEDNum; j++) {
            _leds[j] = CHSV(color, 255, i);
        }
        _RGB->show();
        delay(time);
    }
}
// extern const GCodeArray gcodeArrays[];
void Robot::WriteWord(uint8_t _ID) {
    // const size_t testGcodeSize = sizeof(testGcode) / sizeof(testGcode[0]);
    for (uint16_t i = 0; i < allGcodes[_ID].size; i++) {
        // Serial1.println(allGcodes[_ID].size);
        if (allGcodes[_ID].data[i].isPen) {
            mobot.ArmMoveDown();
        } else {
            mobot.ArmMoveUp();
            // delay(300);
        }

        // 输入需要补偿的点
        Point_t inputPoint = {allGcodes[_ID].data[i].x - 100, allGcodes[_ID].data[i].y + 20};

        float _x = inputPoint.x;
        float _y = inputPoint.y;
        _x -= -25;
        _y -= 25;

        _x = _x * mobot.rotateRads[0] + _y * mobot.rotateRads[1];
        _y = _x * mobot.rotateRads[2] + _y * mobot.rotateRads[3];

        _x += -25;
        _y += 25;

        Serial1.println(_x);
        Serial1.println(_y);

        mobot.moveArmTo(_x, _y);

        // mobot.ArmMoveDown();
    }
}



float Robot::PidTick(PID_t *pid, float target, float actual) {
    PID_t _pid = *pid;

    _pid.vError = target - actual;

    if (_pid.vError > (_pid.errorBound))
        _pid.vError = (_pid.errorBound);
    if (_pid.vError < (-_pid.errorBound))
        _pid.vError = (-_pid.errorBound);


    _pid.outputKp = ((_pid.kp) * (_pid.vError));

    _pid.integralRound += (_pid.ki * _pid.vError);
    // _pid.integralRemainder = _pid.integralRound >> 10;
    // _pid.integralRound -= (_pid.integralRemainder << 10);
    _pid.outputKi = _pid.integralRound;
    // integralRound limitation is  ratedCurrent*1024
    // if (_pid.outputKi > context->config.motionParams.ratedCurrent << 10)
    //     _pid.outputKi = context->config.motionParams.ratedCurrent << 10;
    // else if (_pid.outputKi < -(context->config.motionParams.ratedCurrent << 10))
    //     _pid.outputKi = -(context->config.motionParams.ratedCurrent << 10);

    _pid.outputKd = _pid.kd * (_pid.vError - _pid.vErrorLast);

    _pid.output = (_pid.outputKp + _pid.outputKi + _pid.outputKd);
    if (_pid.output > _pid.outputBound)
        _pid.output = _pid.outputBound;
    else if (_pid.output < -_pid.outputBound)
        _pid.output = -_pid.outputBound;
    // Serial.printf( "PID: %f, %f, %f, %f\n", _pid.outputKp, _pid.outputKi, _pid.outputKd, _pid.output);
    _pid.vErrorLast = _pid.vError;

    return _pid.output;
}


