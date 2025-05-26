#include "RudderWheelController.h"
#include "main.h"
// #include "main.h"
static void pidChassisTask(void *pvParameters);


RudderWheelController::RudderWheelController():\
_velPID(0, 0, 0,[this](){return this->getActualPos();}, [this](double output){_targetWheelVelocity = output;}),\
_vspinPID(0, 0, 0, [this](){return this->getActualAngle();}, [this](double output){_targetWheelVspin = output;})
{

} 

RudderWheelController::RudderWheelController(ServoController *servo, MotorController *wheelR, MotorController *wheelL):\
_velPID(0, 0, 0,[this](){return this->getActualPos();}, [this](double output){_targetWheelVelocity = output;}),\
_vspinPID(0, 0, 0, [this](){return this->getActualAngle();}, [this](double output){_targetWheelVspin = output;})
{
  _servo = servo;
  _wheelR = wheelR;
  _wheelL = wheelL;
} 


void RudderWheelController::attachServo(ServoController *servo)
{
   _servo = servo;
}

void RudderWheelController::attachMotor( MotorController *wheelR, MotorController *wheelL)
{
   _wheelR = wheelR;
   _wheelL = wheelL;
}

void RudderWheelController::init(void)
{
   // _wheelR->init();
   // _wheelL->init();
   // _servo->init();

   _velPID.setTarget(0);
   _vspinPID.setTarget(_targetAngle);

   if (pidChassisTaskHandle!= NULL) 
   {
      vTaskDelete(pidChassisTaskHandle);
   }
  
   xTaskCreate(pidChassisTask, "pidChassisTask", 4096, this, 1, &pidChassisTaskHandle);
}


// 设置PID计算周期（以毫秒为单位）
void RudderWheelController::setPIDCalculationPeriod(uint16_t periodMs) 
{
   
   pidCalculationPeriodMs = periodMs;

}

uint16_t RudderWheelController::getPIDCalculationPeriod() 
{
   
   return pidCalculationPeriodMs ;

}

void RudderWheelController::turn(float vspin)
{
   _isStop = false;
   _isEnable = false;
   setWheelAngleVelocity(0 ,0 , vspin);
}

void RudderWheelController::move(float vx, float vy, float vspin)
{
  static float angle = 0;

  static int16_t velocity = 0;
  _isStop = false;
  _isEnable = false;

  velocity = -sqrt(vx * vx + vy * vy);
  if (vy != 0)
  {
      angle = atan(vx/vy) * (180.0 / M_PI);
      if (vy > 0)
      {
         velocity = -velocity;
      }   
  }
  else
  {
      angle = 90;
      if (vx > 0)
      {
         velocity = -velocity;
      }    
  }

  float beta =  (vspin - _horizontalAngle) ;
   
  if (abs(beta) > (180))
  {
   
    if(beta > 0)
    {
        beta = ( - _horizontalAngle) + (vspin - 360);
    }
    else
    {
        beta = (360 - _horizontalAngle) + (vspin );
    }
  }
  else
  {
     
  }

  float temp_angle = angle - beta;

  if (temp_angle > 90)
  {
     temp_angle = temp_angle - 180;
     velocity = -velocity;
  }
  else if (temp_angle < -90)
  {
     temp_angle = -(temp_angle + 90);
     velocity = -velocity;
  }
  else
  {

  }
  
  angle = temp_angle;

//   Serial.printf("angle = %f , vx = %f, xy = %f , vel = %d \r\n",angle, vx, vy, velocity);
   setWheelAngleVelocity(angle, velocity, 0);
}

bool RudderWheelController::turnTo(float targetAngle)
{

  //setTargetValue(_actualX, _actualY, targetAngle);
  _targetAngle = targetAngle;
  uint16_t timeout = 20000;
   _isStop = false;
   _isEnable = true;
   _isTurn = true;
   _velPID.setEnabled(false);//目标速度为0，即到达位置后停止
   _vspinPID.setTarget(_targetAngle);
 
   do
   {
       delay(10);
   } while (!isArrived() && timeout--);
   
   if (timeout > 0)
   {
       return true;
   }
   
   return false;

}

bool RudderWheelController::moveTo(float targetX, float targetY)
{

  //move(_positionPID->calculate(targetX, _actualX), _positionPID->calculate(targetY, _actualY), 0);
  _isStop = false;


  return false;

}

bool RudderWheelController::moveTo(float targetX, float targetY, float targetAngle)
{
   uint16_t timeout = 20000;
//   _targetX = targetX;
//   _targetY = targetY;
//   _targetAngle = targetAngle;

  setTargetValue(targetX, targetY, targetAngle);

  _isStop = false;
  _isEnable = true;
  _isTurn = false;
  _velPID.setEnabled(true);
  _velPID.setTarget(0);//目标速度为0，即到达位置后停止
  _vspinPID.setTarget(_targetAngle);

  do
  {
      delay(10);
  } while (!isArrived() && timeout--);
  
  if (timeout > 0)
  {
      return true;
  }
  
  return false;
}

void RudderWheelController::setWheelAngleVelocity(float angle, float velocity, float vspin)
{
  static int16_t lastAngle = 0;
   int16_t timeout = 200;

  int16_t tarPos = (2048 - (int)((angle)*22.76 + 0.5));
  setWheelAngle(angle);
  while (abs(_servo->getPosition(_servoID) - tarPos) > 400 && timeout--)
  {
     setWheelVelocity(R, 0);
     setWheelVelocity(L, 0);
     delay(10);
  }
   setWheelVelocity(R, velocity + vspin );
   setWheelVelocity(L, velocity - vspin );
}

void RudderWheelController::setWheelAngle(float angle)
{
  static int16_t lastAngle = 0;
  int16_t tarPos = (2048 - (int)((angle)*22.76 + 0.5));
  _servo->setPosition(_servoID, tarPos, 4000);
  lastAngle = angle;
}

void RudderWheelController::setWheelAngle(uint8_t id, float angle)
{
  static int16_t lastAngle = 0;
  int16_t tarPos = (2048 - (int)((angle)*22.76 + 0.5));

  switch (id)
  {
   case R:
      _servo->setPosition(_servoID, tarPos, 4000);
      break;
   case L:
      _servo->setPosition(_servoID, tarPos, 4000);
      break; 
   default:
      break;
  }

  lastAngle = angle;
}


void RudderWheelController::setWheelVelocity(uint8_t id, float velocity)
{
  switch (id)
  {
   case R:
      _wheelR->setVelocity(velocity);
      break;
   case L:
      _wheelL->setVelocity(velocity);
      break; 
   default:
      break;
  }
}


void RudderWheelController::setWheelVelocity(float velocity)
{
   _wheelR->setVelocity(velocity);
   _wheelL->setVelocity(velocity);

}

void RudderWheelController::setServoID(uint8_t id)
{
   _servoID = id;
}


//  void RudderWheelController::setActualValue(float actualX, float actualY, float actualAngle)
//  {
//    _actualX = actualX;
//    _actualY = actualY;
//    _actualAngle = actualAngle;

//  }

#if 1

float RudderWheelController::getActualPos(void)
{
   static float positionErr = 0;
   static float wheelAngle = 0;
   static float beta = 0;

   float errX = _targetX -_actualX;
   float errY = _targetY -_actualY;

   positionErr = sqrt(errX * errX + errY * errY);

   if (positionErr > 3)
   {
      if (errY != 0)
      {
         wheelAngle = atan(-errX/errY) * (180.0 / M_PI);
          if (abs(wheelAngle) < 88)
         {
            if (errY < 0)
            {
               positionErr = -positionErr;
            }
         }
         else //if (wheelAngle > 88)
         {
            wheelAngle = 90;
            if (errX > 0)
            {
               positionErr = -positionErr;
            }
         }
         
      }
      else
      {
         wheelAngle = 90;
         
         if (errX > 0)
         {
            positionErr = -positionErr;
         }     
      }

     _actualPositionErr = positionErr;
    
   }
   else
   {
      _actualPositionErr = 0;
   }

   //shell.printf("errX: %f , errY: %f wheelAngle: %f, positionErr:%f\r\n",errX, errY, wheelAngle, _actualPositionErr );
   
   // beta = _horizontalAngle - _actualAngle ;
   
   // if (abs(beta) > (180))
   // {
    
   //   if(beta > 0)
   //   {
   //       beta = ( - _actualAngle) + (_horizontalAngle - 360);
   //   }
   //   else
   //   {
   //       beta = (360 - _actualAngle) + (_horizontalAngle );
   //   }
   // }
   // else
   // {
      
   // }

   // _targetWheelAngle = wheelAngle + beta;


   beta =  (_actualAngle - _horizontalAngle) ;
   
   if (abs(beta) > (180))
   {
    
     if(beta > 0)
     {
         beta = ( - _horizontalAngle) + (_actualAngle - 360);
     }
     else
     {
         beta = (360 - _horizontalAngle) + (_actualAngle );
     }
   }
   else
   {
      
   }
 
   float temp_angle = wheelAngle - beta;
 
   if (temp_angle > 90)
   {
      temp_angle = temp_angle - 180;
      _actualPositionErr = -_actualPositionErr;
   }
   else if (temp_angle < -90)
   {
      temp_angle = -(temp_angle + 90);
      _actualPositionErr = -_actualPositionErr;
   }
   else
   {
      // if (abs(temp_angle) < 88)
      // {
      //    if (errY < 0)
      //    {
      //       _actualPositionErr = -_actualPositionErr;
      //    }
      // }
      // else if (temp_angle > 88)
      // {
      //    temp_angle = 90;
      //    if (errX > 0)
      //    {
      //       _actualPositionErr = -_actualPositionErr;
      //    }
      // }
   }

   // if (condition)
   // {
   //    /* code */
   // }
   
   _targetWheelAngle = temp_angle;

   return _actualPositionErr;
}

#endif


float RudderWheelController::getActualAngle(void)
{
   return _actualAngle;
}

void RudderWheelController::run()
{
#if 1

   if (_isStop)
   {
      setWheelAngleVelocity(_targetWheelAngle, 0, 0);
   }
   else if (_isEnable)
   {
      _velPID.tick();
      _vspinPID.tick();
      // setWheelAngleVelocity(_targetWheelAngle, _targetWheelVelocity, 0);
      // Serial1.printf("ve:%f vo:%f\r\n",_targetWheelVelocity, _targetWheelVspin);
      if (_isTurn)
      {
         setWheelAngleVelocity(0, 0, _targetWheelVspin);
      }
      else
      {
         if (_targetWheelAngle > 75)
         {
            if (abs(_targetWheelVspin) > 5)
            {
               setWheelAngleVelocity(45, _targetWheelVelocity, _targetWheelVspin);
            }
            else
            {
               setWheelAngleVelocity(_targetWheelAngle, _targetWheelVelocity, 0);
            }
         }
         else
         {
            setWheelAngleVelocity(_targetWheelAngle, _targetWheelVelocity, _targetWheelVspin);
         }
      }
      
     
   }
   else
   {

   }
   

#endif
   
}

void RudderWheelController::stop()
{
   _isStop = true;

}

void RudderWheelController::disable()
{
   _isEnable = false;

}

bool RudderWheelController::isArrived(void)
{
   if (abs(_velPID.getError()) <= 3.0f && abs(_vspinPID.getError()) <= 0.5f)
   {
      return true;
   }
   return false;
}


 // PID计算任务函数
static void pidChassisTask(void *pvParameters) 
{
    RudderWheelController *controller = static_cast<RudderWheelController *>(pvParameters);
    BaseType_t xWasDelayed;
    TickType_t lastWakeTime = xTaskGetTickCount();
    while (true) 
    {
        xWasDelayed = xTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(controller->getPIDCalculationPeriod()));

        if (xWasDelayed == pdPASS)
        {
          controller->run();
        }
              
    }

}