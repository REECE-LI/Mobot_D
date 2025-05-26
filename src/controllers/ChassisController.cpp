#include "ChassisController.h"



 void ChassisController::setActualValue(float actualX, float actualY, float actualAngle)
 {
   _actualX = actualX;
   _actualY = actualY;
   _actualAngle = actualAngle;

 }

 void ChassisController::setTargetValue(float targetX, float targetY, float targetAngle)
 {
  _targetX = targetX;
  _targetY = targetY;
  _targetAngle = targetAngle;

 }

