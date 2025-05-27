#include "Robot.h"
#include "main.h"
// #include "ShellFunc.h"

Robot::Robot() {
  // init();
}

Robot::Robot(ChassisController *chassis, ArmController *arm) {
  _chassis = chassis;
  _arm = arm;
  // init();
}

void Robot::attachArmController(ArmController *arm) { _arm = arm; }

void Robot::attachChassisController(ChassisController *chassis) {
  _chassis = chassis;
}

void Robot::attachRGB(CFastLED *RGB, CRGB *leds, uint16_t LEDNum) {
  _LEDNum = LEDNum;
  _leds = leds;
  _RGB = RGB;
}

void Robot::init(void) {
  if (_chassis) {
    _chassis->setTargetValue(ChassisTargetX, ChassisTargetY,
                             ChassisTargetAngle);
    _chassis->setActualValue(ChassisTargetX, ChassisTargetY,
                             ChassisTargetAngle);
    _chassis->init();
  }

  if (_arm) {
    _arm->init();
  }

  // _drawBuff[0] = black_288;
  // _drawBuff[1] = orange_288;
  // _drawBuff[2] = yellow_288;

  // _drawBuff[0] = black_144;
  // _drawBuff[1] = orange_144;
  // _drawBuff[2] = yellow_144;

  _drawBuff[0] = blue_bytes;
  _drawBuff[1] = red_bytes;
  _drawBuff[2] = yellow_bytes;

  // _chassis->moveTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);
}

void Robot::moveChassis(float vx, float vy) { _chassis->move(vx, vy, 0); }

void Robot::moveChassis(float vx, float vy, float vspin) {
  _chassis->move(vx, vy, vspin);
}

bool Robot::moveChassisTo(float x, float y, float angle) {
  return _chassis->moveTo(x, y, angle);
}

void Robot::stopChassis(void) { _chassis->stop(); }

void Robot::setChassisActualValue(float actualX, float actualY,
                                  float actualAngle) {
  _chassis->setActualValue(actualX, actualY, actualAngle);
}

void Robot::setArmZBound(int up, int down) {
  ArmUp = up;
  ArmDown = down;
}

bool Robot::moveArmTo(float x, float y, float z) {
  return _arm->moveTo(x, y, z);
}

bool Robot::moveArmTo(float x, float y) { return _arm->moveTo(x, y); }

bool Robot::moveArmZ(float z) { return _arm->moveZ(z); }

bool Robot::getArmCoordinate() { return _arm->getCoordinate(); }

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

void Robot::drawPoint(uint16_t x, uint16_t y) {
  moveArmZ(ArmUp);
  moveArmTo(x, y);
  moveArmZ(ArmDown);
  moveArmZ(ArmUp);
}

void Robot::draw64x64(uint8_t((*data)[8]), uint8_t armInterval) {
  static uint8_t start = 0;
  moveArmZ(ArmUp);
  moveArmTo(0, 88);
  for (uint8_t y = 0; y < 64; y += armInterval) {
    moveArmZ(ArmUp);
    moveArmTo(0, 85 - y);
    for (uint8_t x = 0; x < 8; x++) {

      for (uint8_t i = start; i < 8; i += armInterval) {
        if ((data[y][x] << i) & 0x80) {

          drawPoint((x * 8) + 5 + i, 85 - y);
        } else {
        }
        // start = 8 - i;
      }
    }
  }
  // mobot.moveArmZ(-1);
}

void Robot::draw8x8(uint8_t(*data)) {
  static uint8_t start = 0;
  static uint8_t armInterval = 8;
  moveArmZ(ArmUp);
  moveArmTo(0, 80);
  for (uint8_t y = 0; y < 8; y++) {
    moveArmZ(ArmUp);
    moveArmTo(0, 80 - (y * armInterval));
    for (uint8_t i = start; i < 8; i++) {
      if ((data[y] << i) & 0x80) {
        // shell.print(1);
        drawPoint((i)*armInterval + 8, 80 - (y * armInterval));
      } else {
        // shell.print(0);
      }
      // start = 8 - i;
    }
    // shell.println();
  }
  // mobot.moveArmZ(-1);
}

void Robot::draw16x16(uint8_t((*data)[2])) {
  static uint8_t start = 0;
  static uint8_t armInterval = 4;
  moveArmZ(ArmUp);
  moveArmTo(0, 85);
  for (uint8_t y = 0; y < 16; y++) {
    moveArmZ(ArmUp);
    moveArmTo(0, 85 - (y * armInterval));
    for (uint8_t x = 0; x < 2; x++) {

      for (uint8_t i = start; i < 8; i++) {
        if ((data[y][x] << i) & 0x80) {

          drawPoint(((x * 8) + i) * armInterval + 15, 85 - (y * armInterval));
        } else {
        }
        // start = 8 - i;
      }
    }
  }
  // mobot.moveArmZ(-1);
}

int16_t takePos4 = 400, takePos3 = 3280;

void Robot::givePen(DrawColor_t penId) {

  // moveChassisTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);
  // 移动到等待换笔点
  moveChassisTo(_waitChange.x, _waitChange.y, _waitChange.angle);
  // stopChassis();

  // 发送换笔指令
  sendChangeCmd(penId, PenState_t::OPEN);
  // 机械臂切换到取笔状态
  sms_sts.WritePosEx(3, takePos3, 1000, 200);
  delay(800);

  sms_sts.WritePosEx(4, takePos4 + 1000, 1000, 200);
  // sms_sts.WritePosEx(3, takePos3, 1000, 200);
  delay(1000);
  // 移动到换笔地点
  moveChassisTo(_change.x, _change.y, _change.angle);
  // stopChassis();

  // 发送换笔结构松开指令
  sms_sts.WritePosEx(4, takePos4, 1000, 200);
  delay(100);
  sms_sts.WritePosEx(4, takePos4, 1000, 200);
  while (abs(takePos4 - sms_sts.ReadPos(4)) > 100) {
    delay(50);
  }

  sendChangeCmd(penId, PenState_t::CLOSE);

  delay(1000);
  // 移动到等待换笔点
  moveChassisTo(_waitChange.x, _waitChange.y, _waitChange.angle);
  stopChassis();
}

void Robot::takePen(DrawColor_t penId) {

  // moveChassisTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);
  // 机械臂切换到取笔状态
  sms_sts.WritePosEx(4, takePos4, 1000, 200);
  delay(100);
  sms_sts.WritePosEx(4, takePos4, 1000, 200);
  sms_sts.WritePosEx(3, takePos3, 1000, 200);
  delay(1000);
  // 移动到等待换笔点
  moveChassisTo(_waitChange.x, _waitChange.y, _waitChange.angle);
  stopChassis();

  // 发送换笔指令
  sendChangeCmd(penId, PenState_t::CLOSE);
  // //机械臂切换到取笔状态
  // sms_sts.WritePosEx(4, takePos4, 1000, 200);
  // delay(100);
  // sms_sts.WritePosEx(4, takePos4, 1000, 200);
  // sms_sts.WritePosEx(3, takePos3, 1000, 200);
  // delay(1000);
  // 移动到换笔地点
  moveChassisTo(_change.x, _change.y, _change.angle);
  stopChassis();

  // 发送换笔结构松开指令
  sendChangeCmd(penId, PenState_t::OPEN);
  delay(100);
  // 机械臂切换到取完笔状态
  sms_sts.WritePosEx(4, takePos4 + 1000, 1000, 200);
  delay(1000);
  // 移动到等待换笔点
  moveChassisTo(_waitChange.x - 100, _waitChange.y + 100, _waitChange.angle);
  stopChassis();

  sms_sts.WritePosEx(3, takePos3 - 1000, 1000, 200);
  delay(1000);
  sms_sts.WritePosEx(4, takePos4 + 2400, 1000, 200);
}

bool Robot::sendChangeCmd(DrawColor_t penId, PenState_t state) {
  uint16_t timeout = 200;
  char buff[8];
  sprintf(buff, "pen %d %d", penId, state);
  do {
    do {
      if (!timeout--) {
        return false;
      }
      Wire.beginTransmission(SLAVE_ADDRESS); // 开始与从设备通信
      Wire.print(buff);                      // 发送数据
      delay(5);

    } while (Wire.endTransmission() != ESP_OK);
    buff[7] = 0;
    delay(10);
  } while (!receiveChangeACK(buff));

  return true;
}

bool Robot::receiveChangeACK(char *buff) {
  Wire.requestFrom(SLAVE_ADDRESS, 7); // 请求最多32个字节
  delay(5);
  char Message[128] = {0};
  int i = 0;
  while (Wire.available()) {
    char c = Wire.read();
    Message[i++] = c;
  }
  Message[i] = 0;
  shell.println(i);
  if (i == 7) {
    shell.printf("Message:%s\tbuff:%s \r\n", Message, buff);
    int result1 = strcmp(Message, buff);
    shell.println(result1);
    if (result1 == 0) {
      // shell.println(receivedMessage);
      Message[0] = '\n';
      return true;
    }
  }
  return false;
}

void Robot::drawLoop(float startX, float startY, uint16_t width, uint16_t hight,
                     float drawX, float drawY, uint8_t pen) {
  ChassisTargetX = startX;
  ChassisTargetY = startY;

  static bool isXfirst = true;
  static bool isPenfirst = true;

  static uint8_t chassisInterval = 64;
  for (int i = drawY; i < 10; i++) {
    // ChassisTargetX = startX ;
    int j = 0, x = 0;
    if (isPenfirst) {
      j = pen;
      x = drawX;
      isPenfirst = false;
    }
    ChassisTargetY = startY + (i * chassisInterval);
    ChassisTargetX = startX + (x * chassisInterval);
    for (; j < 3; j++) {

      // x = findStartX(width, hight, drawX, &_drawBuff[j][(i * 8) * (width / 8)]);
      // if (x == -1) {
      //   continue;
      // }

      ChassisTargetX = startX + (x * chassisInterval);
      // 取笔
      takePen((DrawColor_t)(j + 1));
      // 移动到待打点
      moveChassisTo(_waitChange.x - 100, ChassisTargetY + 50,
                    _waitChange.angle); //
      // 旋转90度
      _chassis->turnTo(285);

      // 移动到起始点下面
      moveChassisTo(ChassisTargetX, ChassisTargetY + 50, 285); //

      // 旋转至水平
      _chassis->turnTo(ChassisTargetAngle);

      // 打点
      // drawPictureHight16(ChassisTargetX, ChassisTargetY, width, hight, x,
      // &_drawBuff[j][(i*16)*20]);
      //  drawPictureHight16(ChassisTargetX, ChassisTargetY, width, hight, x,
      //  &_drawBuff[j][(i*16)*(width/8)]);
      drawPictureHight8(ChassisTargetX, ChassisTargetY, width, hight, x,
                        &_drawBuff[j][(i * 8) * (width / 8)]);

      // 旋转90度
      _chassis->turnTo(15);
      // 移动到待待换笔点
      moveChassisTo(_waitChange.x - 100, ChassisTargetY, 15);

      // 旋转至水平
      _chassis->turnTo(ChassisTargetAngle);

      // 移动到换笔点
      moveChassisTo(_waitChange.x, _waitChange.y, _waitChange.angle);

      delay(1000);
      // 还笔
      givePen((DrawColor_t)(j + 1));

      ChassisTargetX = startX;
      // ChassisTargetY = startY  + (i*chassisInterval);
    }
  }
}

int8_t Robot::findStartX(uint16_t width, uint16_t hight, uint8_t drawX,
                         uint8_t *data) {
  uint32_t dataSum = 0;

  for (int i = drawX; i < (width / 16); i++) {
    for (int y = 0; y < 16; y++) {
      for (int x = 0; x < 2; x++) {
        dataSum += data[(i * 2) + y * (width / 8) + x]; //
      }
    }
    // sumBuff[i] = dataSum;
    if (dataSum) {
      return i;
    }

    dataSum = 0;
  }

  return -1;
}

void Robot::drawPictureHight8(float startX, float startY, uint16_t width,
                              uint16_t hight, uint8_t drawX, uint8_t *data) {
  uint32_t dataSum = 0;
  static uint8_t chassisInterval = 64;
  ChassisTargetX = startX;
  ChassisTargetY = startY;

  moveChassisTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);
  stopChassis();

  for (int i = drawX; i < (width / 8); i++) {

    for (int y = 0; y < 8; y++) {
      // for (int x = 0; x < 2; x++)
      {
        drawBuffer8[y] = data[(i) + y * (width / 8)]; //
        // shell.println(drawBuffer8[y]);
        dataSum += drawBuffer8[y];
      }
    }
    if (dataSum) {
      // moveChassisTo(ChassisTargetX , ChassisTargetY + (chassisInterval) ,
      // ChassisTargetAngle);
      moveChassisTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);
      stopChassis();

      draw8x8(drawBuffer8);
      {
        // ChassisTargetX += chassisInterval;
        moveChassisTo(ChassisTargetX, ChassisTargetY + (chassisInterval),
                      ChassisTargetAngle);
        // ChassisTargetX-=(chassisInterval);
        // ChassisTargetX = startX + (i + 1) * chassisInterval;
        ChassisTargetX += chassisInterval;
        moveChassisTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);
        stopChassis();
      }
    } else {
      // ChassisTargetX = startX + (i + 1) * chassisInterval;
      moveChassisTo(ChassisTargetX, ChassisTargetY + (chassisInterval),
                    ChassisTargetAngle);
      // ChassisTargetX-=(chassisInterval);
      ChassisTargetX = startX + (i + 1) * chassisInterval;
      moveChassisTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);
      stopChassis();
    }
    dataSum = 0;
  }
}

#if 0
void Robot::drawPictureHight16(float startX, float startY, uint16_t width, uint16_t hight, uint8_t drawX, uint8_t *data)
{
  uint32_t dataSum = 0;
  uint32_t sumBuff[18] = {0};
  static uint8_t chassisInterval = 64;
  ChassisTargetX = startX ;
  ChassisTargetY = startY ;

  moveChassisTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);
  stopChassis();

  // for (int i = drawX; i < (width/16); i++)
  // {
  //   for (int y = 0; y < 16; y++)
  //   {
  //     for (int x = 0; x < 2; x++)
  //     {
  //       drawBuffer16[y][x] = data[(i*2)+y*(width/8)+x];//
  //       dataSum +=  drawBuffer16[y][x];
  //     }
  //   }
  //   sumBuff[i] = dataSum;
  //   dataSum = 0;
  // }

  for (int i = drawX; i < (width/16); i++)
  {
    // for (uint8_t j = (i*2*16); j < (16*(width/8)); j++)
    // {
    //   /* code */
    // }

    for (int y = 0; y < 16; y++)
    {
      for (int x = 0; x < 2; x++)
      {
        drawBuffer16[y][x] = data[(i*2)+y*(width/8)+x];//
        dataSum +=  drawBuffer16[y][x];
      }
    }
    if (dataSum)
    {
     // moveChassisTo(ChassisTargetX , ChassisTargetY + (chassisInterval) , ChassisTargetAngle);
     //旋转到水平
      _chassis->turnTo(ChassisTargetAngle);
      //移动到打点处
      moveChassisTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);
      stopChassis();
      //打点
      draw16x16(drawBuffer16);
      //向下移动
      moveChassisTo(ChassisTargetX , ChassisTargetY + (chassisInterval) , ChassisTargetAngle);
      //ChassisTargetX-=(chassisInterval);
      //ChassisTargetX = startX + (i + 1) * chassisInterval;
      //旋转45度
      _chassis->turnTo(15);
      //移动到下一个点
      ChassisTargetX += chassisInterval;
      moveChassisTo(ChassisTargetX, ChassisTargetY + (chassisInterval),15);
      stopChassis();
    }
    else
    {
      // moveChassisTo(ChassisTargetX , ChassisTargetY + (chassisInterval) , ChassisTargetAngle);
      // ChassisTargetX-=(chassisInterval);
      // if (ChassisActualAngle)
      // {
      //   /* code */
      // }

      _chassis->turnTo(15);
      ChassisTargetX = startX + (i + 1) * chassisInterval;
      moveChassisTo(ChassisTargetX, ChassisTargetY + (chassisInterval) ,15);
      stopChassis();
    }
    dataSum = 0;
  }

}
#endif
void Robot::drawPictureHight16(float startX, float startY, uint16_t width,
                               uint16_t hight, uint8_t drawX, uint8_t *data) {
  uint32_t dataSum = 0;
  uint32_t sumBuff[18] = {0};
  static uint8_t chassisInterval = 64;
  ChassisTargetX = startX;
  ChassisTargetY = startY;

  moveChassisTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);
  stopChassis();

  // for (int i = drawX; i < (width/16); i++)
  // {
  //   for (int y = 0; y < 16; y++)
  //   {
  //     for (int x = 0; x < 2; x++)
  //     {
  //       drawBuffer16[y][x] = data[(i*2)+y*(width/8)+x];//
  //       dataSum +=  drawBuffer16[y][x];
  //     }
  //   }

  //   sumBuff[i] = dataSum;
  //   dataSum = 0;
  // }

  for (int i = drawX; i < (width / 16); i++) {
    // for (uint8_t j = (i*2*16); j < (16*(width/8)); j++)
    // {
    //   /* code */
    // }

    for (int y = 0; y < 16; y++) {
      for (int x = 0; x < 2; x++) {
        drawBuffer16[y][x] = data[(i * 2) + y * (width / 8) + x]; //
        dataSum += drawBuffer16[y][x];
      }
    }
    if (dataSum) {
      // moveChassisTo(ChassisTargetX , ChassisTargetY + (chassisInterval) ,
      // ChassisTargetAngle);
      moveChassisTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);
      stopChassis();

      draw16x16(drawBuffer16);
      {
        moveChassisTo(ChassisTargetX, ChassisTargetY + (chassisInterval),
                      ChassisTargetAngle);
        // ChassisTargetX-=(chassisInterval);
        // ChassisTargetX = startX + (i + 1) * chassisInterval;
        ChassisTargetX += chassisInterval;
        moveChassisTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);
        stopChassis();
      }
    } else {
      moveChassisTo(ChassisTargetX, ChassisTargetY + (chassisInterval),
                    ChassisTargetAngle);
      // ChassisTargetX-=(chassisInterval);
      ChassisTargetX = startX + (i + 1) * chassisInterval;
      moveChassisTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);
      stopChassis();
    }
    dataSum = 0;
  }
}

void Robot::drawPicture(float startX, float startY, float drawX, float drawY,
                        uint16_t width, uint16_t hight, uint8_t *data) {
  static uint8_t chassisInterval = 64;
  static uint8_t armInterval = 4;
  uint32_t dataSum = 0;
  uint8_t ratio = 1;
  bool isFirst = true;

  ChassisTargetX = startX + (drawX)*chassisInterval;
  ChassisTargetY = startY + (drawY)*chassisInterval;

  // moveChassisTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);

  moveChassisTo(_waitChange.x, _waitChange.y, _waitChange.angle);
  stopChassis();
  _drawBuff[DrawColor_t::BLACK] = data;

  for (int y = drawY; y * 64 / ratio < hight; y++) {
    if (y % 2) {
      // for (int x = (width/8)/8/ratio - 1 ; x >= 0; x--)//- drawX
      int x = x = (width / 8) / 8 / ratio - 1;
      if (isFirst) {
        x = drawX;
        isFirst = false;
      }
      for (; x >= 0; x--) {
        for (int i = 0; i < 64 / ratio; i++) {
          for (int j = 0; j < 8 / ratio; j++) {
            drawBuffer[i * ratio][j * ratio] =
                data[((y * 64) / ratio + i) * (width / 8) + (x * 8) / ratio +
                     j];
            dataSum += drawBuffer[i][j];
          }
        }

        if (dataSum) {
          draw64x64(drawBuffer, armInterval);
          if ((x - 1) >= 0) {
            moveChassisTo(ChassisTargetX, ChassisTargetY + (chassisInterval),
                          ChassisTargetAngle);
            // ChassisTargetX-=(chassisInterval);
            ChassisTargetX = startX + (x - 1) * chassisInterval;
            moveChassisTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);
            stopChassis();
          } else {
            // ChassisTargetY+=chassisInterval;
            ChassisTargetY = startY + (y + 1) * chassisInterval;
            moveChassisTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);
            stopChassis();
          }
        } else {
          if ((x - 1) >= 0) {
            // moveChassisTo(ChassisTargetX, ChassisTargetY + (chassisInterval)
            // , ChassisTargetAngle); ChassisTargetX-=(chassisInterval);
            ChassisTargetX = startX + (x - 1) * chassisInterval;
            // moveChassisTo(ChassisTargetX, ChassisTargetY,ChassisTargetAngle);
            // stopChassis();
          } else {
            // ChassisTargetY+=chassisInterval;
            ChassisTargetY = startY + (y + 1) * chassisInterval;
            // moveChassisTo(ChassisTargetX, ChassisTargetY,
            // ChassisTargetAngle); stopChassis();
          }
        }

        dataSum = 0;
      }

    } else {
      int x = 0;
      if (isFirst) {
        x = drawX;
        isFirst = false;
      }

      for (; (x * 8) / ratio < (width / 8); x++) {
        for (int i = 0; i < 64 / ratio; i++) {
          for (int j = 0; j < 8 / ratio; j++) {
            drawBuffer[i * ratio][j * ratio] =
                data[((y * 64 / ratio) + i) * (width / 8) + (x * 8) / ratio +
                     j];
            dataSum += drawBuffer[i][j];
          }
        }

        if (dataSum) {
          draw64x64(drawBuffer, armInterval);

#if 0

          moveChassisTo(1980, 380, ChassisTargetAngle);

          sms_sts.WritePosEx(3, 3500, 1000, 200);
          delay(1000);
          sms_sts.WritePosEx(4, 1000, 1000, 200);
          delay(1000);

          moveChassisTo(2060, 348, ChassisTargetAngle);
          stopChassis();
          for (uint i = 0; i < 5; i++)
          {
            Wire.beginTransmission(0x08); // 开始与从设备通信
            Wire.print("Servo 600"); // 发送数据
            byte error = Wire.endTransmission(); // 结束通信并获取错误码
            delay(5);
          }

          delay(2000);
          sms_sts.WritePosEx(4, 30, 2000, 200);
          while (abs(30 - sms_sts.ReadPos(4)) > 100)
          {
            delay(50);
          }
          for (int i = 0; i < 5; i++)
          {
            Wire.beginTransmission(SLAVE_ADDRESS); // 开始与从设备通信
            Wire.print("Servo 1300"); // 发送数据
            byte error = Wire.endTransmission(); // 结束通信并获取错误码
            delay(5);
          }

          delay(1000);

          moveChassisTo(1980, 380, ChassisTargetAngle);
          stopChassis();

          delay(2000);
          moveChassisTo(2060, 348, ChassisTargetAngle);
          stopChassis();
          for (uint i = 0; i < 5; i++)
          {
            Wire.beginTransmission(SLAVE_ADDRESS); // 开始与从设备通信
            Wire.print("Servo 600"); // 发送数据
            byte error = Wire.endTransmission(); // 结束通信并获取错误码
            delay(5);
          }
          {
            delay(200);

          }
          sms_sts.WritePosEx(4, 1000, 2000, 200);

          delay(1000);
          moveChassisTo(1980, 480, ChassisTargetAngle);
          stopChassis();
#endif
          if (((x + 1) * 8) / ratio < (width / 8)) {
            moveChassisTo(ChassisTargetX, ChassisTargetY + (chassisInterval),
                          ChassisTargetAngle);
            ChassisTargetX = startX + (x + 1) * chassisInterval;
            moveChassisTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);
            stopChassis();
          } else {
            ChassisTargetY = startY + (y + 1) * chassisInterval;
            moveChassisTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);
            stopChassis();
          }

        } else {
          if (((x + 1) * 8) / ratio < (width / 8)) {
            // moveChassisTo(ChassisTargetX, ChassisTargetY + (chassisInterval)
            // , ChassisTargetAngle);
            ChassisTargetX = startX + (x + 1) * chassisInterval;
            // moveChassisTo(ChassisTargetX, ChassisTargetY,
            // ChassisTargetAngle); stopChassis();
          } else {
            ChassisTargetY = startY + (y + 1) * chassisInterval;
            // moveChassisTo(ChassisTargetX, ChassisTargetY,
            // ChassisTargetAngle); stopChassis();
          }
        }

        dataSum = 0;
      }
    }
  }
}

#if 0
void Robot::drawPicture(float startX, float startY, float drawX, float drawY, uint16_t width, uint16_t hight,  uint8_t *data)
{
  static uint8_t chassisInterval = 64;
  static uint8_t armInterval = 4;
  uint32_t dataSum = 0;
  uint8_t ratio = 1;
  bool isFirst = true;

  ChassisTargetX = startX + (drawX) * chassisInterval;
  ChassisTargetY = startY + (drawY) * chassisInterval;;

  moveChassisTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);
  stopChassis();
  for (int y = drawY; y*64/ratio < hight; y++)
  {
    if (y%2)
    {
      //for (int x = (width/8)/8/ratio - 1 ; x >= 0; x--)//- drawX
      int x = x = (width/8)/8/ratio - 1;
      if (isFirst)
      {
        x = drawX;
        isFirst = false;
      }
      for ( ; x >= 0; x--)
      {
        for (int i = 0; i < 64/ratio; i++)
        {
          for (int j = 0; j < 8/ratio; j++)
          {
            drawBuffer[i*ratio][j*ratio] = data[((y*64)/ratio+i)*(width/8)+(x*8)/ratio+j];
            dataSum +=  drawBuffer[i][j];
          }
        }

        if (dataSum)
        {
          draw64x64(drawBuffer, armInterval);
          if ((x-1) >= 0)
          {
            moveChassisTo(ChassisTargetX , ChassisTargetY + (chassisInterval) , ChassisTargetAngle);
            // ChassisTargetX-=(chassisInterval);
            ChassisTargetX = startX + (x - 1) * chassisInterval;
            moveChassisTo(ChassisTargetX, ChassisTargetY,ChassisTargetAngle);
            stopChassis();
          }
          else
          {
            // ChassisTargetY+=chassisInterval;
            ChassisTargetY = startY + (y+1) * chassisInterval;
            moveChassisTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);
            stopChassis();
          }
        }
        else
        {
          if ((x-1) >= 0)
          {
           // moveChassisTo(ChassisTargetX, ChassisTargetY + (chassisInterval) , ChassisTargetAngle);
            // ChassisTargetX-=(chassisInterval);
            ChassisTargetX = startX + (x - 1) * chassisInterval;
           // moveChassisTo(ChassisTargetX, ChassisTargetY,ChassisTargetAngle);
           // stopChassis();
          }
          else
          {
            // ChassisTargetY+=chassisInterval;
            ChassisTargetY = startY + (y + 1) * chassisInterval;
           // moveChassisTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);
           // stopChassis();
          }
        }

        dataSum = 0;

      }

    }
    else
    {
      int x = 0;
      if (isFirst)
      {
        x = drawX;
        isFirst = false;
      }

      for (; (x * 8)/ratio < (width / 8); x++)
      {
        for (int i = 0; i < 64/ratio; i++)
        {
          for (int j = 0; j < 8/ratio; j++)
          {
            drawBuffer[i*ratio][j*ratio] = data[((y*64/ratio)+i)*(width/8)+(x*8)/ratio+j];
            dataSum +=  drawBuffer[i][j];
          }
        }

        if (dataSum)
        {
          draw64x64(drawBuffer, armInterval);

#if 1

          moveChassisTo(1980, 380, ChassisTargetAngle);

          sms_sts.WritePosEx(3, 3500, 1000, 200);
          delay(1000);
          sms_sts.WritePosEx(4, 1000, 1000, 200);
          delay(1000);

          moveChassisTo(2060, 348, ChassisTargetAngle);
          stopChassis();
          for (uint i = 0; i < 5; i++)
          {
            Wire.beginTransmission(0x08); // 开始与从设备通信
            Wire.print("Servo 600"); // 发送数据
            byte error = Wire.endTransmission(); // 结束通信并获取错误码
            delay(5);
          }

          delay(2000);
          sms_sts.WritePosEx(4, 30, 2000, 200);
          while (abs(30 - sms_sts.ReadPos(4)) > 100)
          {
            delay(50);
          }
          for (int i = 0; i < 5; i++)
          {
            Wire.beginTransmission(SLAVE_ADDRESS); // 开始与从设备通信
            Wire.print("Servo 1300"); // 发送数据
            byte error = Wire.endTransmission(); // 结束通信并获取错误码
            delay(5);
          }

          delay(1000);

          moveChassisTo(1980, 380, ChassisTargetAngle);
          stopChassis();

          delay(2000);
          moveChassisTo(2060, 348, ChassisTargetAngle);
          stopChassis();
          for (uint i = 0; i < 5; i++)
          {
            Wire.beginTransmission(SLAVE_ADDRESS); // 开始与从设备通信
            Wire.print("Servo 600"); // 发送数据
            byte error = Wire.endTransmission(); // 结束通信并获取错误码
            delay(5);
          }
          {
            delay(200);

          }
          sms_sts.WritePosEx(4, 1000, 2000, 200);

          delay(1000);
          moveChassisTo(1980, 480, ChassisTargetAngle);
          stopChassis();
#endif
          if (((x+1) * 8)/ratio < (width / 8))
          {
            moveChassisTo(ChassisTargetX , ChassisTargetY + (chassisInterval) , ChassisTargetAngle );
            ChassisTargetX = startX + (x + 1) * chassisInterval;
            moveChassisTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);
            stopChassis();
          }
          else
          {
            ChassisTargetY = startY + (y + 1) * chassisInterval;
            moveChassisTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);
            stopChassis();
          }

        }
        else
        {
          if (((x+1) * 8)/ratio < (width / 8))
          {
            //moveChassisTo(ChassisTargetX, ChassisTargetY + (chassisInterval) , ChassisTargetAngle);
            ChassisTargetX = startX + (x + 1) * chassisInterval;
            //moveChassisTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);
            //stopChassis();
          }
          else
          {
            ChassisTargetY = startY + (y + 1) * chassisInterval;
            //moveChassisTo(ChassisTargetX, ChassisTargetY, ChassisTargetAngle);
            //stopChassis();
          }
        }

        dataSum = 0;


      }
    }
  }
}

#endif
