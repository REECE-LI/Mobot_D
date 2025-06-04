#include "main.h"
#include "Arduino.h"
#include "ArmController.h"
#include "CAN_ESP32.h"
#include "ChassisController.h"
#include "DendoStepper.h"
#include "MT6701.h"
#include "OneButton.h"
#include "Robot.h"
#include "RudderWheelController.h"
#include "ScaraController.h"
#include "ServoController.h"
#include "ShellFunc.h"
#include "TMC2300.h"
#include "Wire.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ArduinoJson.h>
#include <ESP32Encoder.h>
#include <GCodeParser.h>
#include <Wire.h>

Servo gdw;
#define QUEUE_LENGTH 20
#define ITEM_SIZE 256
// CAN CAN_1(GPIO_NUM_1, GPIO_NUM_2, 1000000);
static QueueHandle_t xSerialQueue = NULL;


typedef struct {
  char payload[ITEM_SIZE];
  uint16_t length;
} SerialMessage;

DendoStepper_config_t conf = {
    .step_p = MOTOR_STEP,
    .dir_p = MOTOR_DIR,
    .en_p = MOTOR_EN,
    .endSw_p = ENDSW_DISABLED,
    .timer_group = TIMER_GROUP_0,
    .timer_idx = TIMER_0,
};
DendoStepper stepper(&conf);
TMC2300 tmc2300(&Serial0, 0);

Encoder encoder_1(ENCODER1_PIN_A, ENCODER1_PIN_B);
Encoder encoder_2(ENCODER2_PIN_A, ENCODER2_PIN_B);

TB67H450 tb67h450_1(DRIVER1_PIN_1, DRIVER1_PIN_2, DRIVER1_PIN_C,
                    DRIVER1_PWM1_CH, DRIVER1_PWM2_CH, DRIVER1_PWM3_CH);
TB67H450 tb67h450_2(DRIVER2_PIN_1, DRIVER2_PIN_2, DRIVER2_PIN_C,
                    DRIVER2_PWM1_CH, DRIVER2_PWM2_CH, DRIVER2_PWM3_CH);

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

void setup() {
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
  // wheelL._velPID.registerTimeFunction(millis);
  wheelL._posPID.setPID(0.5, 0, 0);
  wheelL._posPID.setOutputBounds(-100, 100);
  wheelL._posPID.setMaxIntegralCumulation(2000);
  // wheelL._posPID.registerTimeFunction(millis);
  wheelL.init();

  wheelR._velPID.setPID(5, 0.5, 0);
  wheelR._velPID.setOutputBounds(-1000, 1000);
  wheelR._velPID.setMaxIntegralCumulation(2000);
  // wheelR._velPID.registerTimeFunction(millis);
  wheelR._posPID.setPID(0.5, 0, 0);
  wheelR._posPID.setOutputBounds(-100, 100);
  wheelR._posPID.setMaxIntegralCumulation(2000);
  // wheelR._posPID.registerTimeFunction(millis);
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
  // stepper.init();//�п�
  // stepper.enableMotor();
  pinMode(MOTOR_EN, OUTPUT);
  digitalWrite(MOTOR_EN, HIGH);
  tmc2300.begin(115200);
  tmc2300.writeRegister(TMC2300_CHOPCONF, 0x13008001);
  tmc2300.writeRegister(TMC2300_GCONF, 0x00000070);
  tmc2300.setCurrent(8, 26); // 30
  shell.println("tmc2300 init");
  tmc2300.setVelocity(0);
  armR.goHomeZ();

  mobot.init();
  // mobot.moveChassisTo(1000, 500, mobot.ChassisTargetAngle);
  button.attachClick(click);
  // delay(1000);
  //  gdw.setPeriodHertz(50);
  //  gdw.attach(GDW_PIN, 6, 500, 2500);//21
  //  gdw.writeMicroseconds(1300);
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

  if (xMutex != NULL && runEventGroup != NULL) {
    xTaskCreate(SerialTask, "Serial", 1024 * 5, NULL, 3, NULL);
    // xTaskCreate(ChassisControlTask, "ChassisControl", 1024*4, NULL, 2, NULL);
    xTaskCreate(ArmControlTask, "ArmControl", 1024 * 4, NULL, 3, NULL);
  }
}

void loop() {
  // Serial.printf("Encoder1: %d\t Encoder2: %d \r\n", servo.getPosition(3),
  // servo.getPosition(4));
  shell.executeIfInput();
  button.tick();
  delay(10);

#if 0
  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.print("Servo 1300");
  byte error = Wire.endTransmission();

  if (error == 0) {
    shell.println("Data sent successfully!");
  } else {
    shell.print("Error sending data: ");
    shell.println(error);
  }

  delay(100); // ÿ��1�뷢��һ������

  Wire.requestFrom(SLAVE_ADDRESS, 32);
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

void click(void) {
#if 1
  static uint16_t num = 0;
  for (uint8_t y = 80; y > 0; y -= 8) {
    for (uint8_t x = 0; x < 80; x += 8) {
      if (x == 0) {
        mobot.moveArmZ(-6000);
        mobot.moveArmTo(x, y);
        delay(200);
      } else {
        // mobot.moveArmZ(-6000);
        // mobot.moveArmTo(x, y);
        // mobot.moveArmZ(-35000);
        mobot.drawPoint(x, y);
      }
    }
  }
  mobot.moveArmZ(-1);
#endif
}

void click_bitmap(void) {
  for (int i = 1; i < 4; i++) {
    mobot.moveChassisTo(mobot.ChassisTargetX, mobot.ChassisTargetY + 50,
                        mobot.ChassisTargetAngle);
    mobot.ChassisTargetX += (i * 50); // 82mm
    mobot.moveChassisTo(mobot.ChassisTargetX, mobot.ChassisTargetY,
                        mobot.ChassisTargetAngle);
    mobot.drawPoint(32, 32);
  }
}

void ArmControlTask(void *pvParameters) {
  EventBits_t uxBits;
  const TickType_t xTicksToWait = 200 / portTICK_PERIOD_MS;
  static uint8_t times = 0;
  /* Wait a maximum of 100ms for either bit 0 or bit 4 to be set within
     the event group. Clear the bits before exiting. */
  while (1) {
#if 1
    if (isWriting) {
      /* code */
      uxBits = xEventGroupWaitBits(
          runEventGroup, /* The event group being tested. */
          DOWN_BIT | UP_BIT |
              RUN_BIT,   /* The bits within the event group to wait for. */
          pdTRUE,        /* BIT_0 & BIT_4 should be cleared before returning. */
          pdFALSE,       /* Don't wait for both bits, either bit will do. */
          xTicksToWait); /* Wait a maximum of 100ms for either bit to be set. */

      if ((uxBits & UP_BIT) != 0) {
        if ((xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)) {
          mobot.moveArmZ(-mobot.ArmTargetZ);
          mobot.ArmTargetZ = 0;
          xSemaphoreGive(xMutex);
          // xEventGroupSetBits(zEventGroup, RUN_BIT);
        } else {
        }
      } else if ((uxBits & DOWN_BIT) != 0) {
        if ((xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)) {
          mobot.moveArmZ(-mobot.ArmTargetZ);
          mobot.ArmTargetZ = 0;
          xSemaphoreGive(xMutex);
        } else {
        }
      } else if ((uxBits & RUN_BIT) != 0) {
        if (mobot.ArmTargetZ == 0) {
          if (moveTimes == times) {
            mobot.moveChassisTo(mobot.ChassisTargetX,
                                mobot.ChassisTargetY + fontWidth,
                                mobot.ChassisTargetAngle);
            mobot.moveChassisTo(mobot.ChassisTargetX, mobot.ChassisTargetY,
                                mobot.ChassisTargetAngle);
            mobot.stopChassis();
            mobot.ChassisTargetX += fontWidth;
            times++;
          }
          if (mobot.moveArmTo(mobot.ArmTargetX, mobot.ArmTargetY)) {
            Serial.printf("%d", G_index);
            // shell.printf("%d\t%d\r\n",G_index, GS_index);
          }
        }
      } else {
        // mobot.moveArmZ(-20000);
        // mobot.moveArmZ(0);
      }
    } else {
      delay(10);
    }

#endif
  }
}

uint8_t ask_flag = 0;

void ChassisControlTask(void *pvParameters) // void *pvParameters
{
  TickType_t xLastWakeTime, xLLastWakeTime;
  ;
  EventBits_t uxBits;
  const TickType_t xFrequency = 10;
  BaseType_t xWasDelayed;
  xLastWakeTime = xTaskGetTickCount();
  uint8_t seFlag = 0;
  float ang = 0;
  int32_t vel = 0;
  // SemaphoreHandle_t xBinarySemaphore = (SemaphoreHandle_t)pvParameters;

  while (1) {
    // uxBits = xEventGroupWaitBits(
    //   runEventGroup,   /* The event group being tested. */
    //   CHASSISRUN_BIT, /* The bits within the event group to wait for. */
    //   pdTRUE,        /* BIT_0 & BIT_4 should be cleared before returning. */
    //   pdFALSE,       /* Don't wait for both bits, either bit will do. */
    //   portMAX_DELAY );/* Wait a maximum of 100ms for either bit to be set. */

    // if( ( uxBits & CHASSISRUN_BIT ) != 0 )
    {
      // mobot.moveChassisTo(mobot.ChassisTargetX, mobot.ChassisTargetY,
      // mobot.ChassisTargetAngle); delay(20);
      //  Serial.println(mobot._chassis->isArrived());
      // while (!mobot._chassis->isArrived())
      // {
      //   Serial.println(mobot._chassis->isArrived());
      // if (mobot.moveChassisTo(mobot.ChassisTargetX, mobot.ChassisTargetY,
      // mobot.ChassisTargetAngle))
      // {
      //   // delay(2000);
      //   // mobot.ChassisTargetY += 100;

      //   if (runFlag)
      //   {

      //     // click();
      //     mobot.moveArmZ(-1);
      //     mobot.moveArmTo(40,40);
      //     mobot.moveArmZ(28000);
      //     mobot.moveChassisTo(mobot.ChassisTargetX, mobot.ChassisTargetY +
      //     100, mobot.ChassisTargetAngle);

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
      // EventBits_t uxBits = xEventGroupSetBits(runEventGroup, RUN_BIT);
    }

    // xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency);

    // if (xWasDelayed == pdPASS)
    // {
    //   mobot.moveChassisTo(mobot.ChassisTargetX, mobot.ChassisTargetY,
    //   mobot.ChassisTargetAngle); delay(20);

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
void SerialTask(void *pvParameters) // void *pvParameters
{
  // uint16_t size = sizeof(CamData.data)/sizeof(CamData.data[0]);
  int ret = 0;
  uint16_t ser_index = 0;

  char ser_char;
  // char buffer[100];
  // SemaphoreHandle_t xBinarySemaphore = (SemaphoreHandle_t)pvParameters;
  while (1) {
#if 1
    ret = Serial.available();
    // Serial0.println(ret);
    if (ret) {
      char buffer[100]; // ���ӻ�������С�����ɸ�������
      // ��������
      int len = Serial.read(buffer, 99);
      if (len > 0) {
        buffer[len] = 0;

        // shell.println(buffer);
        // Serial1.println("ok");
        decodeJsonData(buffer);
      }
    } else {
      // delay(1);
    }

    //   ret = Serial1.available();
    //   //Serial0.println(ret);
    //   if (ret) {
    //    char buffer[100];  // ���ӻ�������С�����ɸ�������
    //    // ��������
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

//     // ��ʼ�����У�ʵ��Ӧ�������ⴴ�����˴���ʾ�ã�
//     xSerialQueue = xQueueCreate(QUEUE_LENGTH, sizeof(SerialMessage));
//     configASSERT(xSerialQueue);

//     // ���λ�����״̬
//     struct {
//         char data[MAX_BUFFER_SIZE];
//         size_t head = 0;
//         size_t tail = 0;

//         size_t available() const {
//             return (tail >= head) ? (tail - head) : (MAX_BUFFER_SIZE - head +
//             tail);
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
//         // ��һ�׶Σ����ݽ���
//         if(Serial.available()) {
//             uint8_t incomingByte = Serial.read();
//             ringBuffer.push(incomingByte);
//         }

//         // �ڶ��׶Σ�Э�����
//         SerialMessage newMsg;
//         if(ringBuffer.extractPacket(&newMsg)) {
//             // ���͵�FreeRTOS���У�����ʱ������
//             if(xQueueSend(xSerialQueue, &newMsg, pdMS_TO_TICKS(10)) !=
//             pdPASS) {
//                 // ������������ԣ������������
//                 SerialMessage dummy;
//                 xQueueReceive(xSerialQueue, &dummy, 0);
//                 xQueueSend(xSerialQueue, &newMsg, 0);
//             }
//         }

//         // �����׶Σ����ݴ���
//         SerialMessage receivedMsg;
//         if(xQueueReceive(xSerialQueue, &receivedMsg, pdMS_TO_TICKS(5)) ==
//         pdPASS) {
//             // ����������У��
//             // if(receivedMsg.length > 2 &&
//             //    receivedMsg.payload[0] == '{' &&
//             //    receivedMsg.payload[receivedMsg.length-1] == '}') {
//             //     decodeJsonData(receivedMsg.payload);
//             // } else {
//             //     // �����������ش�����
//             //     Serial.write(0x15);  // NAK
//             // }

//             decodeJsonData(receivedMsg.payload);
//         }

//         // ���Ľ׶Σ�ϵͳ����
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

// ״̬����ö��
enum ArmMovement { ARM_UP = 2, ARM_DOWN = 1 };

// enum EventFlags { UP_BIT = BIT0, DOWN_BIT = BIT1, RUN_BIT = BIT2 };
//  ������Դ���ʱ���
//  #def ine  SAFE_ACCESS(_code) do { \
//     if(xSemaphoreTake(xMutex, pdMS_TO_TICKS(10)) == pdTRUE) { \
//         _code \
//         xSemaphoreGive(xMutex); \
//     } else { \
//         logError("Mutex timeout"); \
//     } \
// } while(0)

// �������ݿ����
void processDataBlock(JsonObject data) {
  // SAFE_ACCESS({
  mobot.ChassisActualX = data["x"].as<int>() * 1.28; // * 1.76;
  mobot.ChassisActualY = data["y"].as<int>() * 1.28; // * 1.76;
  mobot.ChassisActualAngle = data["angle"].as<float>();
  mobot.setChassisActualValue(mobot.ChassisActualX, mobot.ChassisActualY,
                              mobot.ChassisActualAngle);
  // shell.printf("angle: %f\r\n",mobot.ChassisActualAngle);
  //   });
}

// ����G0ָ��
void processG0Command(JsonObject G0, JsonVariant index) {
  EventBits_t uxBits;
  // SAFE_ACCESS({
  if (xSemaphoreTake(xMutex, 0) == pdTRUE) {
    const float targetZ = G0["Z"].as<float>();
    mobot.ArmTargetZ = targetZ;

    // const EventFlags flag = (targetZ == ARM_UP) ? UP_BIT : DOWN_BIT;
    // EventBits_t uxBits = xEventGroupSetBits(runEventGroup, flag);
    uxBits =
        xEventGroupSetBits(runEventGroup, /* The event group being updated. */
                           UP_BIT);

    if (!index.isNull()) {
      GS_index = index.as<int>();
      Serial.printf("%d", GS_index);
    }

    xSemaphoreGive(xMutex);
  }

  // if(uxBits & flag) {
  //     logDebug("%sָ������Ч", (flag == UP_BIT) ? "����" : "�½�");
  // }
  // });
}

// ����G1ָ��
void processG1Command(JsonObject G1, JsonVariant index) {
  // SAFE_ACCESS({
  mobot.ArmTargetX = fmodf(G1["X"].as<float>(), fontWidth);
  // mobot.ArmTargetY = fmodf(G1["Y"].as<float>(), 80);
  mobot.ArmTargetY = G1["Y"].as<float>();

  // mobot.ArmTargetX = G1["X"].as<float>();
  // mobot.ArmTargetY = G1["Y"].as<float>();

  // mobot.ChassisTargetX = 320 + 40*(int(G1["X"].as<float>())/80);
  moveTimes = (int(G1["X"].as<float>()) / fontWidth);
  // Serial.printf("mun: %d \r\n",int(G1["X"].as<float>())/80);
  EventBits_t uxBits = xEventGroupSetBits(runEventGroup, RUN_BIT);

  if (!index.isNull()) {
    G_index = index.as<int>();
  }

  ask_flag = 1;
  // if(uxBits & RUN_BIT) {
  //  //   logDebug("����ָ���Ѽ���");
  // }
  //  });
}

void decodeJsonData(char *buffer) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, buffer);

  if (error) {
    //  logError("JSON����ʧ��: %s", error.c_str());
    return;
  }
  // ���ݷַ�����
  if (doc["data"].isNull() == false) {
    processDataBlock(doc["data"]);
  } else if (doc["G0"].isNull() == false) {
    processG0Command(doc["G0"], doc["index"]);
  } else if (doc["G1"].isNull() == false) {
    processG1Command(doc["G1"], doc["index"]);
  } else {
    //  logWarning("δ֪��JSONָ���ʽ");
  }
}
