#include "ServoController.h"

ServoController::ServoController(void)
{

}

ServoController::ServoController(HardwareSerial *pSerial)
{

    _servo_sts.pSerial = pSerial;
    //init();

}

void ServoController::attachSerial(HardwareSerial *pSerial)
{
    _servo_sts.pSerial = pSerial;
}

 void ServoController::init(void)
 {

 }


void ServoController::setVelocity(int16_t velocity)
{
    _targetVelocity = velocity;
}
// SyncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[]);//同步写多个舵机位置指令
void ServoController::setPosition(uint8_t id, int16_t position, int16_t velocity)
{
    _servo_sts.WritePosEx(id, position, velocity, 0);
}

void ServoController::setPositionSync(uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Speed[], uint8_t ACC[])
{
    _servo_sts.SyncWritePosEx(ID, IDN, Position, Speed, ACC);
}

int ServoController::getPosition(uint8_t id)
{
    return _servo_sts.ReadPos(id);
}


