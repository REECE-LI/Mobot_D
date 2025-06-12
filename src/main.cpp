#include "main.h"
#include "Arduino.h"
#include "CAN_ESP32.h"
#include "ChassisController.h"
#include "DendoStepper.h"
#include "MT6701.h"
#include "OneButton.h"
#include "Robot.h"
#include "RudderWheelController.h"

#include "ShellFunc.h"
#include "TMC2300.h"
#include "Wire.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ArduinoJson.h>
#include <gcode.h>


DendoStepper_config_t conf = {
    .step_p = MOTOR_STEP,
    .dir_p = MOTOR_DIR,
    .en_p = MOTOR_EN,
    .endSw_p = ENDSW_DISABLED,
    .timer_group = TIMER_GROUP_1,
    .timer_idx = TIMER_1,
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


OneButton button(USER_PIN, true);

void receiveEvent(int numBytes);

void setup() {
    delay(1000);
    Serial.begin(115200);
    Serial0.setPins(SERIAL0_PIN_RX, SERIAL0_PIN_TX);
    Serial0.begin(115200);

    Serial1.setPins(SERIAL1_PIN_RX, SERIAL1_PIN_TX);
    Serial1.begin(115200);


    shell.attach(Serial1);
    shellInit();


    Serial.begin(115200);

    wheelL.init();
    wheelR.init();


    armR.setLimitPin(LIMIT_PIN_1, LIMIT_PIN_2);


    // WS2812RGB.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);
    // WS2812RGB.setBrightness(30);
    // WS2812RGB.clear();
    // WS2812RGB.show();
    // mobot.attachRGB(&WS2812RGB, leds, NUM_LEDS);
    // shell.println("RGB init");

    pinMode(MOTOR_EN, OUTPUT);
    digitalWrite(MOTOR_EN, HIGH);
    tmc2300.begin(115200);
    tmc2300.writeRegister(TMC2300_CHOPCONF, 0x13008001);
    tmc2300.writeRegister(TMC2300_GCONF, 0x00000070);
    tmc2300.setCurrent(8, 30); // 30

    armR.goHomeZ();

    mobot.init();
    button.attachClick(click);

    Wire.begin(SLAVE_ADDRESS, I2C_SDA, I2C_SCK, 1000000);
    Wire.onReceive(receiveEvent);

    Serial1.println("iic init OK!");
    Serial2.setPins(SERIAL2_PIN_RX, SERIAL2_PIN_TX);
    Serial2.begin(1000000);
    sms_sts.pSerial = &Serial2;
    Serial1.println("STS init OK!!");

    // mobot._rudder->_servo->_servo_sts.CalibrationOfs(mobot._rudder->_servoID);


    Serial1.println("Init OK !");

    // mobot.targetPos.x = 269;
    // mobot.targetPos.y = 120;
    // mobot._arm->moveZ(mobot.ArmDown);
    // xTaskCreatePinnedToCore(SerialTask, "Serial", 1024 * 4, NULL, 3, NULL, 0);
    // xTaskCreatePinnedToCore(ArmControlTask, "ArmControl", 1024 * 5, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(ChassisControlTask, "ChassisControl", 1024 * 8, NULL, 3, NULL, 1);
}

uint8_t writeIndex = 0;
bool isWrite = false;

void loop() {
#if 0
    shell.executeIfInput();
    button.tick();
    delay(10);
#else
    if (writeIndex == 1 && !isWrite) {
        mobot.WriteWord(86);
        mobot.ArmMoveUp();
        isWrite = true;
    } else if (writeIndex == 3&& !isWrite) {
        mobot.WriteWord(87);
        mobot.ArmMoveUp();
        isWrite = true;
    } else if (writeIndex == 5&& !isWrite) {
        mobot.WriteWord(88);
        mobot.ArmMoveUp();
        isWrite = true;
    } else if (writeIndex == 7&& !isWrite) {
        mobot.WriteWord(89);
        mobot.ArmMoveUp();
        isWrite = true;
    } else if (writeIndex == 9&& !isWrite) {
        mobot.WriteWord(90);
        mobot.ArmMoveUp();
        isWrite = true;
    } else if (writeIndex == 11&& !isWrite) {
        mobot.WriteWord(22);
        mobot.ArmMoveUp();
        isWrite = true;
    } else if (writeIndex == 13&& !isWrite) {
        mobot.WriteWord(23);
        mobot.ArmMoveUp();
        isWrite = true;
    }

#endif
}

#define POIN_GAP 5
#define X_NUM 15
#define Y_NUM 15


void click(void) {
#if 0
    // 测试机械臂

    static uint16_t num = 0;
    mobot.moveArmZ(mobot.ArmUp);
    for (int8_t y = 25; y <= 95; y += POIN_GAP) {
        for (int8_t x = -20; x >= -95; x -= POIN_GAP) {
#if 1
            float _x = x;
            float _y = y;
            _x -= -25;
            _y -= 25;

           _x = _x * mobot.rotateRads[0] + _y * mobot.rotateRads[1];
           _y = _x * mobot.rotateRads[2] + _y * mobot.rotateRads[3];

            _x += -25;
            _y += 25;

            // mobot.moveArmTo(_x, _y);
            mobot.moveArmTo(_x-Lut[num].deltaX, _y+Lut[num].deltaY);

            // mobot.moveArmTo(_x-Lut[num].deltaX - Lut2[num].deltaX, _y+Lut[num].deltaY + Lut2[num].deltaY);
#else
            float _x = x;
            float _y = y;
            // _x -= -25;
            // _y -= 25;
            //
            // _x = _x * mobot.rotateRads[0] + _y * mobot.rotateRads[1];
            // _y = _x * mobot.rotateRads[2] + _y * mobot.rotateRads[3];
            //
            // _x += -25;
            // _y += 25;


            mobot.moveArmTo(_x, _y);
#endif
            delay(100);
            mobot.ArmMoveDown();
            delay(100);
            mobot.moveArmZ(mobot.ArmUp);
            delay(300);

            if (x != -20)
            num++;
            Serial1.println(num);
        }
    }
    mobot.moveArmZ(-1);
#else
    mobot.WriteWord(86);
#endif
}

void receiveEvent(int numBytes) {
    // Serial1.println("1");
    static int16_t tempAngle;
    static int16_t lastJ;
    static int16_t joyX, joyY, j, k;

    if (numBytes < 6) return; // 确保数据长度正确

    uint8_t rxData[6];
    for (int i = 0; i < 6; i++) {
        rxData[i] = Wire.read();
    }

    joyX = (int16_t) rxData[0];
    joyY = (int16_t) rxData[1];
    j = rxData[2];
    k = rxData[3];
    int16_t angle = (rxData[4] << 8) | rxData[5];

    mobot.Speed = (joyX * joyX + joyY * joyY) * 0.001;
    if (j - lastJ) {
        writeIndex += 1;
        isWrite =false;
    }

    // Serial1.println(atan2(joyX,joyY));
    // if (joyY > 0)
    //     mobot.rollAngle = atan2(joyX, joyY) / PI * 180;
    // else
    //     mobot.rollAngle = -atan2(joyX, joyY) / PI * 180;
    mobot.rollAngle = 90;
    mobot.ChassisActualAngle = angle;

    // 你可以在这里打印或使用这些值
    // Serial1.printf("Received: joyX=%d, joyY=%d, j=%d, k=%d, angle=%d\n", joyX, joyY, j, k, angle);
    lastJ = j;
}
