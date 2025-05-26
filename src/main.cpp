#include "Arduino.h"
#include "DendoStepper.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "main.h"
#include "ShellFunc.h"
#include <ESP32Encoder.h>
#include <Wire.h>
#include "MT6701.h"
#include "TMC2300.h"
#include "Robot.h"
#include "CAN_ESP32.h"
#include <GCodeParser.h>
#include <ArduinoJson.h>
#include "ServoController.h"
#include "ArmController.h"
#include "ChassisController.h"
#include "ScaraController.h"
#include "RudderWheelController.h"
#include "OneButton.h"
#include "Wire.h"

Servo gdw;
#define QUEUE_LENGTH 20
#define ITEM_SIZE 256
//CAN CAN_1(GPIO_NUM_1, GPIO_NUM_2, 1000000);
static QueueHandle_t xSerialQueue = NULL;
// 自定义队列数据结构
typedef struct {
    char payload[ITEM_SIZE];
    uint16_t length;
} SerialMessage;

DendoStepper_config_t conf={
    .step_p=MOTOR_STEP,
    .dir_p=MOTOR_DIR,
    .en_p=MOTOR_EN,
    .endSw_p=ENDSW_DISABLED,
    .timer_group=TIMER_GROUP_0,
    .timer_idx=TIMER_0,
};
DendoStepper stepper(&conf);
TMC2300 tmc2300(&Serial0, 0);

Encoder encoder_1(ENCODER1_PIN_A, ENCODER1_PIN_B);
Encoder encoder_2(ENCODER2_PIN_A, ENCODER2_PIN_B);

TB67H450 tb67h450_1(DRIVER1_PIN_1, DRIVER1_PIN_2, DRIVER1_PIN_C, DRIVER1_PWM1_CH, DRIVER1_PWM2_CH, DRIVER1_PWM3_CH);
TB67H450 tb67h450_2(DRIVER2_PIN_1, DRIVER2_PIN_2, DRIVER2_PIN_C, DRIVER2_PWM1_CH, DRIVER2_PWM2_CH, DRIVER2_PWM3_CH);

MotorDriver motor_1(&encoder_1, &tb67h450_1);
MotorDriver motor_2(&encoder_2, &tb67h450_2);

MotorController wheelL(&motor_1);
MotorController wheelR(&motor_2);
ServoController servo(&Serial2);

RudderWheelController chassis(&servo, &wheelL, &wheelR);
ScaraController armR(&servo, &stepper, &tmc2300, 70, 70);

Robot mobot(&chassis, &armR);

CRGB leds[NUM_LEDS];
CFastLED WS2812RGB;
SMS_STS sms_sts;
SemaphoreHandle_t xMutex;
EventGroupHandle_t runEventGroup;

void ChassisControlTask(void *pvParameters);
void SerialTask(void *pvParameters);
void ArmControlTask(void *pvParameters);
void decodeJsonData(char *buffer);
void click(void);
void click_bitmap(void);

int16_t position = 0;
uint16_t G_index = 0;
uint16_t GS_index = 0;
bool runFlag = false;
uint8_t isWriting = 0;
uint8_t moveTimes = 0;
uint8_t fontWidth = 100;

OneButton button(USER_PIN, true);



void setup() 
{
  Serial.begin(115200);
  Serial0.setPins(SERIAL0_PIN_RX, SERIAL0_PIN_TX);
  Serial0.begin(115200);

  Serial1.setPins(SERIAL1_PIN_RX, SERIAL1_PIN_TX);
  Serial1.begin(115200);

  Serial2.setPins(SERIAL2_PIN_RX, SERIAL2_PIN_TX);
  Serial2.begin(1000000);
  sms_sts.pSerial = &Serial2;
  shell.attach(Serial1);
  shellInit();

  Wire.setPins(I2C_SDA, I2C_SCK);
  Wire.begin();

#if 1
  wheelL._velPID.setPID(5, 0.5, 0);
  wheelL._velPID.setOutputBounds(-1000, 1000);
  wheelL._velPID.setMaxIntegralCumulation(2000);
  //wheelL._velPID.registerTimeFunction(millis);
  wheelL._posPID.setPID(0.5, 0, 0);
  wheelL._posPID.setOutputBounds(-100, 100);
  wheelL._posPID.setMaxIntegralCumulation(2000);
  //wheelL._posPID.registerTimeFunction(millis);
  wheelL.init();

  wheelR._velPID.setPID(5, 0.5, 0);
  wheelR._velPID.setOutputBounds(-1000, 1000);
  wheelR._velPID.setMaxIntegralCumulation(2000);
  //wheelR._velPID.registerTimeFunction(millis);
  wheelR._posPID.setPID(0.5, 0, 0);
  wheelR._posPID.setOutputBounds(-100, 100);
  wheelR._posPID.setMaxIntegralCumulation(2000);
  //wheelR._posPID.registerTimeFunction(millis);
  wheelR.init();

  // wheelL.setVelocity(50);
  // wheelR .setVelocity(50);
#if 1
  chassis._velPID.setPID(5, 0.01, 0);
  chassis._velPID.setOutputBounds(-60, 60);
  chassis._velPID.setMaxIntegralCumulation(5);

  chassis._vspinPID.setPID(5, 0.05, 0);
  chassis._vspinPID.setOutputBounds(-30, 30);
  chassis._vspinPID.setMaxIntegralCumulation(8);
  chassis._vspinPID.setFeedbackWrapBounds(0, 360);

  chassis.init();

  armR.setLimitPin(LIMIT_PIN_1, LIMIT_PIN_2);
  armR.init();
  shell.println("armR init");

  WS2812RGB.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS); 
  WS2812RGB.setBrightness(30);  
  WS2812RGB.clear();
  WS2812RGB.show();
  mobot.attachRGB(&WS2812RGB, leds, NUM_LEDS);
  shell.println("RGB init");
      // stepper.init();//有坑
      // stepper.enableMotor();
  pinMode(MOTOR_EN, OUTPUT);
  digitalWrite(MOTOR_EN, HIGH);
  tmc2300.begin(115200);
  tmc2300.writeRegister(TMC2300_CHOPCONF, 0x13008001);
  tmc2300.writeRegister(TMC2300_GCONF, 0x00000070);
  tmc2300.setCurrent(8, 26);//30
  shell.println("tmc2300 init");
  tmc2300.setVelocity(0);
  armR.goHomeZ();

  mobot.init();
 // mobot.moveChassisTo(1000, 500, mobot.ChassisTargetAngle);
  button.attachClick(click);
  //delay(1000);
  // gdw.setPeriodHertz(50);
  // gdw.attach(GDW_PIN, 6, 500, 2500);//21
  // gdw.writeMicroseconds(1300);
  shell.println("init");
#endif
#endif
  // pinMode(MOTOR_EN, OUTPUT);
  // digitalWrite(MOTOR_EN, HIGH);
  // tmc2300.begin(115200);
  // tmc2300.writeRegister(TMC2300_CHOPCONF, 0x13008001);
  // tmc2300.writeRegister(TMC2300_GCONF, 0x00000070);
  // tmc2300.setCurrent(8, 30);
  // shell.println("tmc2300 init");
  // tmc2300.setVelocity(0);
  // button.attachClick(click);
  xMutex = xSemaphoreCreateMutex();
  runEventGroup = xEventGroupCreate();
  xSerialQueue = xQueueCreate(QUEUE_LENGTH, sizeof(SerialMessage));

  if (xMutex!= NULL && runEventGroup !=NULL) {
    xTaskCreate(SerialTask, "Serial", 1024*5, NULL, 3, NULL);
    //xTaskCreate(ChassisControlTask, "ChassisControl", 1024*4, NULL, 2, NULL);
    xTaskCreate(ArmControlTask, "ArmControl", 1024*4, NULL, 3, NULL);
  }
}


void loop()  
{

  //Serial.printf("Encoder1: %d\t Encoder2: %d \r\n", servo.getPosition(3), servo.getPosition(4));
  shell.executeIfInput();
  button.tick();
  delay(10);

#if 0
  Wire.beginTransmission(SLAVE_ADDRESS); // 开始与从设备通信
  Wire.print("Servo 1300"); // 发送数据
  byte error = Wire.endTransmission(); // 结束通信并获取错误码

  if (error == 0) {
    shell.println("Data sent successfully!");
  } else {
    shell.print("Error sending data: ");
    shell.println(error);
  }

  delay(100); // 每隔1秒发送一次数据

  Wire.requestFrom(SLAVE_ADDRESS, 32); // 请求最多32个字节
  String receivedMessage = "";
  while (Wire.available()) {
    char c = Wire.read();
    receivedMessage += c;
  }
  if (!receivedMessage.isEmpty()) {
    shell.print("Received from slave: ");
    shell.println(receivedMessage);
  }
#endif
}



void click(void)
{
#if 1
  static uint16_t num = 0;
  for (uint8_t y = 80; y > 0; y-=8)
  {
    for (uint8_t x = 0; x < 80; x+=8)
    {
      if (x == 0)
      {
        mobot.moveArmZ(-6000);
        mobot.moveArmTo(x, y);
        delay(200);
      }
      else
      {
        // mobot.moveArmZ(-6000);
        // mobot.moveArmTo(x, y);
        // mobot.moveArmZ(-35000);
        mobot.drawPoint(x, y);
      }

    }
  }
  mobot.moveArmZ(-1);
#endif

#if 0
    static uint16_t num = 0;
  for (uint8_t y = 85; y > 0; y-=5)
  {
    for (uint8_t x = 0; x < 90; x+=5)
    {
      if (x == 0)
      {
        tmc2300.setVelocity(100000);
        delay(400);
        mobot.moveArmTo(x, y);
       // delay(200);
      }
      else
      {
        tmc2300.setVelocity(100000);
        delay(400);
        mobot.moveArmTo(x, y);
        tmc2300.setVelocity(-100000);
        delay(400);
      }

    }
  }
 // mobot.moveArmZ(-1);
#endif

#if 0
  // while (true)
  // {
  //   tmc2300.setVelocity(100000);
  //   delay(400);
  //   tmc2300.setVelocity(-100000);
  //   delay(400);
  // }
#endif


#if 0

// mobot.moveArmZ(-1);
// mobot.moveChassisTo(mobot.ChassisTargetX, mobot.ChassisTargetY, mobot.ChassisTargetAngle);

// for (int i = 0; i < 10; i++)
// {
//   mobot.moveArmTo(10, 30);
//   mobot.moveArmZ(-35000);
//   mobot.moveArmTo(70, 30);
//   mobot.moveArmZ(-1);
//   mobot.moveChassisTo(mobot.ChassisTargetX, mobot.ChassisTargetY + fontWidth, mobot.ChassisTargetAngle);
//   mobot.moveChassisTo(mobot.ChassisTargetX, mobot.ChassisTargetY, mobot.ChassisTargetAngle);
//   mobot.ChassisTargetX += fontWidth;
// }

for (int i = 1; i < 4; i++)
{
  
  mobot.moveChassisTo(mobot.ChassisTargetX, mobot.ChassisTargetY, mobot.ChassisTargetAngle);
  mobot.drawPoint(35, 35);
  mobot.moveChassisTo(mobot.ChassisTargetX, mobot.ChassisTargetY + 50, mobot.ChassisTargetAngle);
  mobot.ChassisTargetX += (50);

}

for (int i = 1; i < 5; i++)
{
  
  mobot.moveChassisTo(mobot.ChassisTargetX, mobot.ChassisTargetY, mobot.ChassisTargetAngle);
  mobot.drawPoint(35, 35);
  mobot.moveChassisTo(mobot.ChassisTargetX, mobot.ChassisTargetY + 50, mobot.ChassisTargetAngle);
  mobot.ChassisTargetX += (i*50);

}

for (int i = 1; i < 5; i++)
{
  
  mobot.moveChassisTo(mobot.ChassisTargetX, mobot.ChassisTargetY, mobot.ChassisTargetAngle);
  mobot.drawPoint(35, 35);
 // mobot.moveChassisTo(mobot.ChassisTargetX, mobot.ChassisTargetY + 50, mobot.ChassisTargetAngle);
  mobot.ChassisTargetY += (i*50);

}
#endif

}

void click_bitmap(void)
{

 for (int i = 1; i < 4; i++)
 {
    mobot.moveChassisTo(mobot.ChassisTargetX, mobot.ChassisTargetY + 50, mobot.ChassisTargetAngle);
    mobot.ChassisTargetX += (i*50);//82mm
    mobot.moveChassisTo(mobot.ChassisTargetX, mobot.ChassisTargetY, mobot.ChassisTargetAngle);
    mobot.drawPoint(32, 32);
 }
}


void ArmControlTask(void *pvParameters)
{
   EventBits_t uxBits;
   const TickType_t xTicksToWait = 200 / portTICK_PERIOD_MS;
   static uint8_t times = 0;
    /* Wait a maximum of 100ms for either bit 0 or bit 4 to be set within
       the event group. Clear the bits before exiting. */
    while (1)
    {
#if 1
      if (isWriting)
      {
        /* code */  
       uxBits = xEventGroupWaitBits(
               runEventGroup,   /* The event group being tested. */
               DOWN_BIT | UP_BIT | RUN_BIT, /* The bits within the event group to wait for. */
               pdTRUE,        /* BIT_0 & BIT_4 should be cleared before returning. */
               pdFALSE,       /* Don't wait for both bits, either bit will do. */
               xTicksToWait );/* Wait a maximum of 100ms for either bit to be set. */

      if( ( uxBits & UP_BIT ) != 0 )
      {
        if ((xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE))
        { 
          mobot.moveArmZ(-mobot.ArmTargetZ);
          mobot.ArmTargetZ = 0;
          xSemaphoreGive(xMutex);
          // xEventGroupSetBits(zEventGroup, RUN_BIT);
        }
        else
        {

        }     
      }
      else if(( uxBits & DOWN_BIT ) != 0 )
      {
        if ((xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE))
        {
          
          mobot.moveArmZ(-mobot.ArmTargetZ);
          mobot.ArmTargetZ = 0;
          xSemaphoreGive(xMutex);
        }
        else
        {

        }
      }
      else if ((uxBits & RUN_BIT) != 0)
      {
        if (mobot.ArmTargetZ == 0)
        {
          if (moveTimes == times)
          {
            mobot.moveChassisTo(mobot.ChassisTargetX, mobot.ChassisTargetY + fontWidth, mobot.ChassisTargetAngle);
            mobot.moveChassisTo(mobot.ChassisTargetX, mobot.ChassisTargetY, mobot.ChassisTargetAngle);
            mobot.stopChassis();
            mobot.ChassisTargetX += fontWidth;
            times++;

          }
          if(mobot.moveArmTo(mobot.ArmTargetX, mobot.ArmTargetY))
          {
            Serial.printf("%d",G_index);
            // shell.printf("%d\t%d\r\n",G_index, GS_index);
          }
        }
        
      } 
      else
      {
        // mobot.moveArmZ(-20000);
        // mobot.moveArmZ(0);
      }

      
    }
    else
    {
      delay(10);
    }
    
#endif

    }     

}

uint8_t ask_flag = 0;
void ChassisControlTask(void *pvParameters)//void *pvParameters
{
  TickType_t xLastWakeTime,  xLLastWakeTime;;
  EventBits_t uxBits;
  const TickType_t xFrequency = 10;
  BaseType_t xWasDelayed;
  xLastWakeTime = xTaskGetTickCount ();
  uint8_t seFlag = 0;
  float ang = 0;
  int32_t vel = 0;
  //SemaphoreHandle_t xBinarySemaphore = (SemaphoreHandle_t)pvParameters;
  
  while (1)
  {
    // uxBits = xEventGroupWaitBits(
    //   runEventGroup,   /* The event group being tested. */
    //   CHASSISRUN_BIT, /* The bits within the event group to wait for. */
    //   pdTRUE,        /* BIT_0 & BIT_4 should be cleared before returning. */
    //   pdFALSE,       /* Don't wait for both bits, either bit will do. */
    //   portMAX_DELAY );/* Wait a maximum of 100ms for either bit to be set. */

    // if( ( uxBits & CHASSISRUN_BIT ) != 0 )
    {
     // mobot.moveChassisTo(mobot.ChassisTargetX, mobot.ChassisTargetY, mobot.ChassisTargetAngle);  
      // delay(20);
    //  Serial.println(mobot._chassis->isArrived());
      // while (!mobot._chassis->isArrived())
      // {
      //   Serial.println(mobot._chassis->isArrived());
      // if (mobot.moveChassisTo(mobot.ChassisTargetX, mobot.ChassisTargetY, mobot.ChassisTargetAngle))
      // {
      //   // delay(2000);
      //   // mobot.ChassisTargetY += 100;

      //   if (runFlag)
      //   {

      //     // click();
      //     mobot.moveArmZ(-1);
      //     mobot.moveArmTo(40,40);
      //     mobot.moveArmZ(28000);
      //     mobot.moveChassisTo(mobot.ChassisTargetX, mobot.ChassisTargetY + 100, mobot.ChassisTargetAngle);

      //     if (mobot.ChassisTargetX < 1800)
      //     {
      //       mobot.ChassisTargetX += 200;
      //     }
      //     else
      //     {
      //       mobot.ChassisTargetY+=100;
      //       mobot.ChassisTargetX = 1000;
      //     }
         
      //   }
        
      // }
      
      delay(100);
      // }
      //EventBits_t uxBits = xEventGroupSetBits(runEventGroup, RUN_BIT);

    }

    //xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency);

    // if (xWasDelayed == pdPASS)
    // {
    //   mobot.moveChassisTo(mobot.ChassisTargetX, mobot.ChassisTargetY, mobot.ChassisTargetAngle);  
    //   delay(20);
      
    //   while (!mobot._chassis->isArrived())
    //   {
    //     delay(10);
    //   }
      
    //   EventBits_t uxBits = xEventGroupSetBits(runEventGroup, RUN_BIT);
    //   //Serial.println(pidOut);
    //   //mobot.move(0, 0, SpinPID.calculate(150, mobot.actualAngle));
    //   xLLastWakeTime =  xTaskGetTickCount ();

    // }
    // else
    // {
    // }
    
  }
}


/*

{"G1":{"X":100,"Y":100}}
{"M3":{"S":1}}
*/
#if 1
void SerialTask(void *pvParameters)//void *pvParameters
{
  //uint16_t size = sizeof(CamData.data)/sizeof(CamData.data[0]);
  int ret = 0;
  uint16_t ser_index = 0;
 
  char ser_char;
  //char buffer[100];
  //SemaphoreHandle_t xBinarySemaphore = (SemaphoreHandle_t)pvParameters;
  while (1)
  {
#if 1
   ret = Serial.available();
   //Serial0.println(ret);
   if (ret) {
    char buffer[100];  // 增加缓冲区大小以容纳更多数据
    // 接收数据
    int len = Serial.read(buffer, 99);
    if (len > 0) {
      buffer[len] = 0;

      // shell.println(buffer);
      // Serial1.println("ok");
      decodeJsonData(buffer);
    }
  }
  else
  {
    //delay(1);
  }

//   ret = Serial1.available();
//   //Serial0.println(ret);
//   if (ret) {
//    char buffer[100];  // 增加缓冲区大小以容纳更多数据
//    // 接收数据
//    int len = Serial1.read(buffer, 99);
//    if (len > 0) {
//      buffer[len] = 0;
//      Serial.println(buffer);
//      //logDebug(buffer);
//      decodeJsonData(buffer);
//    }
//  }

#endif
   delay(1);

  }
}

#endif
// #define QUEUE_LENGTH 20
// #define ITEM_SIZE 256

// void SerialTask(void *pvParameters) {
//     constexpr size_t MAX_BUFFER_SIZE = 256;
//     constexpr char FRAME_END_MARKER = '\n';
    
//     // 初始化队列（实际应在任务外创建，此处演示用）
//     xSerialQueue = xQueueCreate(QUEUE_LENGTH, sizeof(SerialMessage));
//     configASSERT(xSerialQueue);
    
//     // 环形缓冲区状态
//     struct {
//         char data[MAX_BUFFER_SIZE];
//         size_t head = 0;
//         size_t tail = 0;
        
//         size_t available() const {
//             return (tail >= head) ? (tail - head) : (MAX_BUFFER_SIZE - head + tail);
//         }
        
//         void push(uint8_t byte) {
//             if(available() < (MAX_BUFFER_SIZE - 1)) {
//                 data[tail] = byte;
//                 tail = (tail + 1) % MAX_BUFFER_SIZE;
//             }
//         }
        
//         bool extractPacket(SerialMessage* msg) {
//             size_t packet_len = 0;
//             while(head != tail && packet_len < ITEM_SIZE-1) {
//                 char c = data[head];
//                 head = (head + 1) % MAX_BUFFER_SIZE;
                
//                 if(c == FRAME_END_MARKER) {
//                     msg->payload[packet_len] = '\0';
//                     msg->length = packet_len;
//                     return true;
//                 }
                
//                 msg->payload[packet_len++] = c;
//             }
//             return false;
//         }
//     } ringBuffer;
//     while(true) {
//         // 第一阶段：数据接收
//         if(Serial.available()) {
//             uint8_t incomingByte = Serial.read();
//             ringBuffer.push(incomingByte);
//         }
        
//         // 第二阶段：协议解析
//         SerialMessage newMsg;
//         if(ringBuffer.extractPacket(&newMsg)) {
//             // 发送到FreeRTOS队列（带超时保护）
//             if(xQueueSend(xSerialQueue, &newMsg, pdMS_TO_TICKS(10)) != pdPASS) {
//                 // 队列满处理策略：丢弃最旧数据
//                 SerialMessage dummy;
//                 xQueueReceive(xSerialQueue, &dummy, 0);
//                 xQueueSend(xSerialQueue, &newMsg, 0);
//             }
//         }
        
//         // 第三阶段：数据处理
//         SerialMessage receivedMsg;
//         if(xQueueReceive(xSerialQueue, &receivedMsg, pdMS_TO_TICKS(5)) == pdPASS) {
//             // 数据完整性校验
//             // if(receivedMsg.length > 2 && 
//             //    receivedMsg.payload[0] == '{' && 
//             //    receivedMsg.payload[receivedMsg.length-1] == '}') {
//             //     decodeJsonData(receivedMsg.payload);
//             // } else {
//             //     // 错误处理：发送重传请求
//             //     Serial.write(0x15);  // NAK
//             // }

//             decodeJsonData(receivedMsg.payload);
//         }
        
//         // 第四阶段：系统节拍
//         vTaskDelay(pdMS_TO_TICKS(1));
//     }
// }

#if 0
void decodeJsonData(char *buffer)
{
  EventBits_t uxBits;
    JsonDocument doc; 
    DeserializationError error = deserializeJson(doc, buffer);
    if (error) {
      // Serial.print(F("deserializeJson() failed: "));
      // Serial.println(error.f_str());
      //return;
    }
    else
    {
      JsonObject data = doc["data"];
      if (data.isNull() == false) 
      {
        mobot.ChassisActualX = data["x"].as<int>();
        mobot.ChassisActualY = data["y"].as<int>();
        mobot.ChassisActualAngle = data["angle"].as<float>();
        mobot.setChassisActualValue(mobot.ChassisActualX, mobot.ChassisActualY, mobot.ChassisActualAngle);
        //Serial0.printf("X:%03d Y:%03d Ang:%06.2f X:%03d Y:%03d \r\n",(int)mobot.ChassisActualX, (int)mobot.ChassisActualY, mobot.actualAngle, (int)mobot.ChassisTargetX, (int)mobot.ChassisTargetY);
      }
      else 
      {
        JsonObject G0 = doc["G0"];
        if (G0.isNull() == false)
        {
#if 1
         // if (xSemaphoreTake(xMutex, 0) == pdTRUE) 
          {
            mobot.ArmTargetZ = G0["Z"].as<float>();
            // if ( mobot.ArmTargetZ == 2)
            // {
            uxBits = xEventGroupSetBits(
                    runEventGroup,    /* The event group being updated. */
                    UP_BIT );

            if (doc["index"].isNull() == false)
            {
              GS_index = doc["index"].as<int>();
            }
             
            Serial.printf("%d",GS_index);
            // }
            // else if ( mobot.ArmTargetZ == 1)
            // {
            //   uxBits = xEventGroupSetBits(
            //           runEventGroup,    /* The event group being updated. */
            //           DOWN_BIT );
            //   if (doc["index"].isNull() == false)
            //   {
            //     GS_index = doc["index"].as<int>();
            //   }
            //   Serial.printf("%d",GS_index);
            // } 

            //mobot.moveArmZ(mobot.ArmTargetZ);
          //  xSemaphoreGive(xMutex);

          }      
#endif       
        } 
        else 
        {
          JsonObject G1 = doc["G1"];
          if (G1.isNull() == false)
          {
            mobot.ArmTargetX = G1["X"].as<float>();
            mobot.ArmTargetY = G1["Y"].as<float>();

            if (doc["index"].isNull() == false)
            {
              G_index = doc["index"].as<int>();
            }
            uxBits = xEventGroupSetBits(
              runEventGroup,    /* The event group being updated. */
              RUN_BIT );
            //Serial0.printf("targetX:%03d targetY:%03d\r\n",(int)mobot.ChassisTargetX, (int)mobot.ChassisTargetY);
            ask_flag = 1;

          }
          else
          {
          //   JsonObject M3 = doc["M3"];
          //   if (M3.isNull() == false)
          //   {
          //     if (mobot.ArmTargetZ == 0)
          //     {
          //       mobot.ArmTargetZ = M3["S"].as<int>();
          //       if ( mobot.ArmTargetZ == 2)
          //       {
          //         uxBits = xEventGroupSetBits(
          //                 runEventGroup,    /* The event group being updated. */
          //                 UP_BIT );
          //         GS_index = doc["index"].as<int>();
          //       }
          //       else if ( mobot.ArmTargetZ == 1)
          //       {
          //         uxBits = xEventGroupSetBits(
          //                 runEventGroup,    /* The event group being updated. */
          //                 DOWN_BIT );
          //         GS_index = doc["index"].as<int>();
          //       }   
          //     }         
          // }
        }
      }

    }
      
    // if (ask_flag == 1)
    // {
    //   if (sqrt((mobot.ChassisTargetX - mobot.ChassisActualX) * (mobot.ChassisTargetX - mobot.ChassisActualX) + \
    //           (mobot.ChassisTargetY -  mobot.ChassisActualY) * (mobot.ChassisTargetY -  mobot.ChassisActualY)) < 5) 
    //   {
    //     //if (ask_flag == 1)
    //     {
    //       Serial.printf("%d",G_index);
    //       //Serial0.printf("X:%d Y:%d Ang:%d X:%d Y:%d \r\n",(int)mobot.ChassisActualX, (int)mobot.ChassisActualY,G_index, (int)mobot.ChassisTargetX, (int)mobot.ChassisTargetY);
    //       ask_flag = 0;
    //     }
    //   }

    // }

  }
  
}

#endif

// 状态定义枚举
enum ArmMovement { ARM_UP = 2, ARM_DOWN = 1 };
//enum EventFlags { UP_BIT = BIT0, DOWN_BIT = BIT1, RUN_BIT = BIT2 };
// 共享资源访问保护
// #define SAFE_ACCESS(_code) do { \
//     if(xSemaphoreTake(xMutex, pdMS_TO_TICKS(10)) == pdTRUE) { \
//         _code \
//         xSemaphoreGive(xMutex); \
//     } else { \
//         logError("Mutex timeout"); \
//     } \
// } while(0)

// 处理数据块更新
void processDataBlock(JsonObject data) {
   // SAFE_ACCESS({
        mobot.ChassisActualX = data["x"].as<int>() * 1.28;// * 1.76;
        mobot.ChassisActualY = data["y"].as<int>() * 1.28;// * 1.76;
        mobot.ChassisActualAngle = data["angle"].as<float>();
        mobot.setChassisActualValue(mobot.ChassisActualX, 
                                  mobot.ChassisActualY, 
                                  mobot.ChassisActualAngle);
        //shell.printf("angle: %f\r\n",mobot.ChassisActualAngle);
  //  });
}
// 处理G0指令
void processG0Command(JsonObject G0, JsonVariant index) {
        EventBits_t uxBits;
   // SAFE_ACCESS({
    if (xSemaphoreTake(xMutex, 0) == pdTRUE) 
    {

      const float targetZ = G0["Z"].as<float>();
      mobot.ArmTargetZ = targetZ;
      
      // const EventFlags flag = (targetZ == ARM_UP) ? UP_BIT : DOWN_BIT;
      // EventBits_t uxBits = xEventGroupSetBits(runEventGroup, flag);
      uxBits = xEventGroupSetBits(
        runEventGroup,    /* The event group being updated. */
        UP_BIT );
      
      if (!index.isNull()) {
          GS_index = index.as<int>();
          Serial.printf("%d", GS_index);
      }

      xSemaphoreGive(xMutex);
    }

        // if(uxBits & flag) {
        //     logDebug("%s指令已生效", (flag == UP_BIT) ? "上升" : "下降");
        // }
   // });
}
// 处理G1指令
void processG1Command(JsonObject G1, JsonVariant index) {
   // SAFE_ACCESS({
        mobot.ArmTargetX = fmodf(G1["X"].as<float>(), fontWidth);
        // mobot.ArmTargetY = fmodf(G1["Y"].as<float>(), 80);
        mobot.ArmTargetY = G1["Y"].as<float>();

        // mobot.ArmTargetX = G1["X"].as<float>();
        // mobot.ArmTargetY = G1["Y"].as<float>();
        
        
        //mobot.ChassisTargetX = 320 + 40*(int(G1["X"].as<float>())/80);
        moveTimes = (int(G1["X"].as<float>())/fontWidth);
        //Serial.printf("mun: %d \r\n",int(G1["X"].as<float>())/80);
        EventBits_t uxBits = xEventGroupSetBits(runEventGroup, RUN_BIT);
        
        if (!index.isNull()) {
            G_index = index.as<int>();
        }
        
        ask_flag = 1;
        // if(uxBits & RUN_BIT) {
        //  //   logDebug("运行指令已激活");
        // }
  //  });
}

void decodeJsonData(char *buffer) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, buffer);
  
  if (error) {
    //  logError("JSON解析失败: %s", error.c_str());
      return;
  }
  // 数据分发处理
  if (doc["data"].isNull() == false) {
      processDataBlock(doc["data"]);
  } 
  else if (doc["G0"].isNull() == false) {
      processG0Command(doc["G0"], doc["index"]);
  } 
  else if (doc["G1"].isNull() == false) {
      processG1Command(doc["G1"], doc["index"]);
  } 
  else {
    //  logWarning("未知的JSON指令格式");
  }
}