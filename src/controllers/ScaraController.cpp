#include "ScaraController.h"
#include "SimpleSerialShell.h"
ScaraController::ScaraController(ServoController *servo, DendoStepper *stepper,  float l1, float l2){

  _servo = servo;
  _stepper =stepper;
  _L1 = l1;
  _L2 = l2;
  _ID[0] = _servo1ID;
  _ID[1] = _servo2ID;

  _velocity[0] = 5000;
  _velocity[1] = 5000;

  _acc[0] = 0;
  _acc[1] = 0;

  //init();

}

ScaraController::ScaraController(ServoController *servo, DendoStepper *stepper, TMC2300 *tmc2300, float l1, float l2){

  _servo = servo;
  _stepper = stepper;
  _tmc2300 = tmc2300;

  _L1 = l1;
  _L2 = l2;
  _ID[0] = _servo1ID;
  _ID[1] = _servo2ID;

  _velocity[0] = 5000;
  _velocity[1] = 5000;

  _acc[0] = 0;
  _acc[1] = 0;

  //init();

}

void ScaraController::init(){

   getCoordinate();

  // if (_tmc2300)
  // {
  //   _tmc2300->begin(115200);
  //   _tmc2300->writeRegister(TMC2300_CHOPCONF, 0x13008001);
  //   _tmc2300->writeRegister(TMC2300_GCONF, 0x00000070);
  //   _tmc2300->setCurrent(8, 30);
  //   _tmc2300->setVelocity(0);
  // }
  
  //setHomeZ();

}

bool ScaraController::moveTo(float x, float y, float z){

  bool ret = false;
  if (_stepper)
  {
    _stepper->runAbsolute(z);
  }

  // tmc2300_writeInt(TMC2300_VACTUAL, 100000);
  // delay(300);
  // tmc2300_writeInt(TMC2300_VACTUAL, 0);

  ret = linearCompensation(_x, _y, x, y);
  //ret = inverseKinematics( x,  y);
  _z = z;
  _x = x;
  _y = y;

  return ret;

}

bool ScaraController::moveTo(float x, float y){

  bool ret = false;

  ret = linearCompensation(_x, _y, x, y);

  _x = x;
  _y = y;

  return ret;

}


bool ScaraController::moveZ(float z){

  bool ret = false;
  uint32_t time = 0;
  uint32_t timeOut = 500;

  _z = z;
  if (_stepper)
  {
    //ret = _stepper->runAbsolute(_z);
    // while (_stepper->getState() != IDLE)
    // {
    //   //isArrived();
    //   Serial.printf("_limitPin: %d %d\r\n",digitalRead(_limitPin1), digitalRead(_limitPin2));
      
    //   delay(5);
    // }

    // while (!isArrived())
    // {
    //   Serial.printf("_limit: %d %d %d %f\r\n",isArrived(), !digitalRead(_limitPin1), !digitalRead(_limitPin2) , _z);
    //   delay(10);
    // }
  }

  if (_tmc2300)
  {
#if 1
    if (_z < -10000)
    {
      if (digitalRead(_limitPin2))
      {
        _tmc2300->setVelocity(-100000);
      }
      else
      {
        // Serial.printf("1:_limitPin1: %d _limitPin2: %d %f\r\n",digitalRead(_limitPin1), digitalRead(_limitPin2), _z);
        _tmc2300->setVelocity(0);
        return true;
      }
      
     
      while (true)
      {
        _tmc2300->setVelocity(-100000);
        if (!digitalRead(_limitPin2))
        {
          //delay(20);
          shell.println(1);
          time = 0;
          _tmc2300->setVelocity(0);
          return true;
        }
        if (time++ >= timeOut)
        {
          time = 0;
          shell.println(0);
          return false;
        }
        delay(1);
      }
    }
    else
    {
      if (digitalRead(_limitPin1))
      {
        _tmc2300->setVelocity(100000);
      }
      else
      {
        _tmc2300->setVelocity(0);
        return true;
      }
      while (true)
      {
        _tmc2300->setVelocity(100000);
        if (!digitalRead(_limitPin1))
        {
         // delay(100);
          time = 0;
          _tmc2300->setVelocity(0);
          return true;
        }
        // if (time++ >= timeOut)
        // {
        //   time = 0;
        //   return false;
        // }
        delay(1);
      }
      
    }
#endif
    // if (_z < -10000)
    // {
    //   _tmc2300->setVelocity(-100000);
    //   delay(300);
    //   _tmc2300->setVelocity(0);
    //   return true;

    // }
    // else
    // {
    //   _tmc2300->setVelocity(100000);
    //   delay(300);
    //   _tmc2300->setVelocity(0);
    //   return true;
    // }

  }
  return true;
}

bool ScaraController::isArrived()
{
  if (_z < -10000)
  {
    if (!digitalRead(_limitPin2))
    {
      // Serial.printf("_limitPin2: %d %f\r\n",!digitalRead(_limitPin2), _z);
      return true;
    }
    
  }
  else
  {
    if (!digitalRead(_limitPin1))
    {
      
      // Serial.printf("_limitPin1: %d %f\r\n",!digitalRead(_limitPin1), _z);
      return true;
    }
  }

  return false;
  
}

void ScaraController::setHomeZ(){

  _stepper->setHome();
}

bool ScaraController::goHomeZ(){

  _tmc2300->setVelocity(20000);

  while (digitalRead(_limitPin1))
  {
    delay(1);
  }
  //delay(600);
  _tmc2300->setVelocity(0);

  return true;
  //_stepper->setHome();
}


void ScaraController::setLimitPin(uint8_t pin1, uint8_t pin2)
{
  _limitPin1 = pin1;
  _limitPin2 = pin2;
}

bool ScaraController::getCoordinate(){

  float theta1,  theta2;
  int pos1, pos2;

  pos1 = _servo->getPosition(_servo1ID);
  pos2 = _servo->getPosition(_servo2ID);

  if (pos1 == -1 || pos2 == -1)
  {
    return false;
  }
  

  theta1 = (_theta1Base - (float)(_servo1BaseAngle - pos1)/11.38) * (M_PI/180);
  theta2 = (_theta2Base - (pos1 - pos2 - (_servo1BaseAngle - _servo2BaseAngle))/11.38) * (M_PI/180) ;

  _x = (_L1 * cos(theta1) + _L2 * cos(theta1 + theta2)) + xErr;
  _y = (_L1 * sin(theta1) + _L2 * sin(theta1 + theta2));
  return true;
}

bool ScaraController::getCoordinate(float *x, float *y, float *z){

  float theta1,  theta2;
  int pos1, pos2;

  pos1 = _servo->getPosition(_servo1ID);
  pos2 = _servo->getPosition(_servo2ID);

  if (pos1 == -1 || pos2 == -1)
  {
    return false;
  }

  theta1 = (_theta1Base - (float)(_servo1BaseAngle - pos1)/11.38) * (M_PI/180);
  theta2 = (_theta2Base - (pos1 - pos2 - (_servo1BaseAngle - _servo2BaseAngle))/11.38) * (M_PI/180) ;

  _x = (_L1 * cos(theta1) + _L2 * cos(theta1 + theta2)) + xErr;
  _y = (_L1 * sin(theta1) + _L2 * sin(theta1 + theta2));

  *x = _x;
  *y = _y;
  *z = _z;

  return true;
}


bool ScaraController::inverseKinematics(float x, float y){

  static float theta1,  theta2, lsatTheta1 ;
 
  static float last_theta1 = 0, last_theta2 = 0;
  static int pos1, pos2, lastPos1, lastPos2;
  // 计算中间变量
  float r = sqrt((x - xErr) * (x - xErr)+ y * y);
  float cos_theta2 = (r * r - _L1 * _L1 - _L2 * _L2) / (2 * _L1 * _L2);

  // 检查解的可行性
  if (abs(cos_theta2) <= 1) {
    theta2 = acos(cos_theta2);
    float k1 = _L1 + _L2 * cos(theta2);
    float k2 = _L2 * sin(theta2);
    theta1 = atan2(y, x - xErr) - atan2(k2, k1);

    theta1 = theta1*(180.0 / M_PI);
    theta2 = theta2*(180.0 / M_PI);
    
    {
     
      pos1 = _servo1BaseAngle - (int)((_theta1Base - theta1)*11.38 + 0.5);
      pos2 = _servo2BaseAngle - (int)((_theta2Base - theta2)*11.38 + 0.5) - (int)((_theta1Base - theta1)*11.38 + 0.5);//+ (pos1 - lastPos1);

      _position[0] = pos1;
      _position[1] = pos2;

      _servo->setPositionSync(_ID, _IDN, _position, _velocity, _acc);
    }

    last_theta1 = theta1;
    last_theta2 = theta2;

    // last_x = x;
    // last_y = y;

    return true;
  } else {
    return false;
  }
}

#if 0
bool ScaraController::linearCompensation(float x1, float y1, float x2, float y2){

  static float x = x1, y = y1, num = 0.1;
  uint8_t delay_ms = 2;
  static float last_x = 0, last_y = 0, now_x = 0, now_y = 0;

  // Serial.printf("x: %f y: %f  x2: %f y2: %f\r\n",x, y, x2, y2);

  if (x2 == x1)
  {
    if (y2 >= y)
    {
      while ((y += num) < y2)
      {
        // y += num;
        inverseKinematics(x,y);
        delay(delay_ms);
        //Serial.println(y);

      }
      return true;
     // Serial.println("ok");
      
    }
    else if (y2 < y)
    {
      while ((y -= num) > y2)
      {
        // y -= num;
        inverseKinematics(x,y);
        delay(delay_ms);
        //Serial.println(y);

      }
      return true;
     // Serial.println("ok");
      
    }

    return false;
    // else
    // {
    //   inverseKinematics(x,y);
    //   //delay(delay_ms);
    //   Serial.println("ok");
    // }

  }
  else
  {
    float k = (y2 - y1) / (x2 - x1);
    //Serial.printf("k:%f \r\n",k);
    if (abs(k) <= 1)
    {
     // Serial.println(k);
      if (x2 > x)
      {
        while ((x += num) < x2)
        {
          //x += num;
          y = k * (x - x1) + y1;
          inverseKinematics(x,y);
          delay(delay_ms);
        }
        return true;
       // Serial.println("ok");
      }
      else if(x2 < x)
      {
        while ((x -= num) > x2)
        {
         // x -= num;
          y = k * (x - x1) + y1;
          inverseKinematics(x,y);
          delay(delay_ms);
        }
        return true;
       // Serial.println("ok");
      }
    }
    else
    {
      if (y2 >= y)
      {
        while ((y += num) < y2)
        {
         // y += num;
          x = ((y - y1) / k) + x1;
          inverseKinematics(x,y);
          delay(delay_ms);

        }
        return true;
       // Serial.println("ok");
      }
      else if(y2 < y)
      {
        while ((y -= num) > y2)
        {
          //y -= num;
          x = ((y - y1) / k) + x1;
          inverseKinematics(x,y);
          delay(delay_ms);

        }
        return true;
       // Serial.println("ok");
      }
      return false;
      // else
      // {
      //   inverseKinematics(x,y);
      //   //delay(delay_ms);
      //   Serial.println("ok");
      // }
    }
  }

  return false;

}


#endif
bool ScaraController::linearCompensation(float x1, float y1, float x2, float y2) {
    constexpr float EPSILON = 1e-6f;
    constexpr float NUM_STEP = 0.2f;
    constexpr uint8_t DELAY_MS = 2;
    static float x = x1, y = y1;
    static float last_x = 0, last_y = 0;  
    // 处理浮点数精度问题的相等判断
    const bool is_vertical = std::fabs(x2 - x1) < EPSILON;
    
    if (is_vertical) {
        // 垂直运动处理
        const bool y_increasing = (y2 - y) > EPSILON;
        const float step = y_increasing ? NUM_STEP : -NUM_STEP;
        while (y_increasing ? (y + step < y2 - EPSILON) 
                           : (y - step > y2 + EPSILON)) 
        {
            y += step;
            inverseKinematics(x, y);
            delay(DELAY_MS);
        }
        // 确保到达终点
        inverseKinematics(x, y2);
        return true;
    } 
    else {
        // 计算斜率（已确保分母不为零）
        const float denominator = x2 - x1;
        assert(std::fabs(denominator) > EPSILON);
        const float k = (y2 - y1) / denominator;
        // 选择步进方向
        const bool use_x_axis = std::fabs(k) <= 1.0f;
        
        if (use_x_axis) {
            const bool x_increasing = (x2 - x) > EPSILON;
            const float step = x_increasing ? NUM_STEP : -NUM_STEP;
            while (x_increasing ? (x + step < x2 - EPSILON) 
                               : (x - step > x2 + EPSILON)) 
            {
                x += step;
                y = k * (x - x1) + y1;
                inverseKinematics(x, y);
                delay(DELAY_MS);
            }
            // 确保到达终点
            inverseKinematics(x2, y2);
        } 
        else {
            const bool y_increasing = (y2 - y) > EPSILON;
            const float step = y_increasing ? NUM_STEP : -NUM_STEP;
            const float inv_k = 1.0f / k;
            while (y_increasing ? (y + step < y2 - EPSILON) 
                               : (y - step > y2 + EPSILON)) 
            {
                y += step;
                x = (y - y1) * inv_k + x1;
                inverseKinematics(x, y);
                delay(DELAY_MS);
            }
            // 确保到达终点
            inverseKinematics(x2, y2);
        }
        return true;
    }
}

