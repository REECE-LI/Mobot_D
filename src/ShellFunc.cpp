#include "ShellFunc.h"
#include <Arduino.h>
#include <SimpleSerialShell.h>
#include "main.h"

int Pen = 1;

int STScmd(int argc, char **argv) {
    char buff[7];
    if (argc != 4) {
        shell.println("bad argument count");
        return -1;
    }

    auto id = atoi(argv[1]);
    auto pos = atoi(argv[2]);
    auto vel = atoi(argv[3]);

    shell.print("calling ");
    shell.print(id);
    shell.print(" with ");
    shell.print(pos);
    shell.print(" and ");
    shell.println(vel);
    sprintf(buff, "pen %d 0", Pen);

    sms_sts.WritePosEx(id, pos, vel, 200);
    if (id == 4 && pos < 500) {
        while (abs(pos - sms_sts.ReadPos(4)) > 100) {
            delay(50);
        }
        for (int i = 0; i < 5; i++) {
            Wire.beginTransmission(SLAVE_ADDRESS); // ��ʼ����豸ͨ��
            Wire.print(buff); // ��������
            byte error = Wire.endTransmission(); // ����ͨ�Ų���ȡ������
            delay(5);
        }
    }
    return 0;
}


int Pencmd(int argc, char **argv) {
    char buff[7];
    if (argc != 5) {
        shell.println("bad argument count");
        return -1;
    }

    auto id = atoi(argv[1]);
    auto pos = atoi(argv[2]);
    auto vel = atoi(argv[3]);
    Pen = atoi(argv[4]);

    // uint8_t dir = 0;
    // if (pos < 0) {
    //   dir = 1;
    //   pos = -pos;
    // }

    shell.print("calling ");
    shell.print(id);
    shell.print(" with ");
    shell.print(pos);
    shell.print(" and ");
    shell.println(vel);

    sprintf(buff, "pen %d 0", Pen);
    sms_sts.WritePosEx(id, pos, vel, 200);
    if (id == 4 && pos < 500) {
        while (abs(pos - sms_sts.ReadPos(4)) > 100) {
            delay(50);
        }
        for (int i = 0; i < 5; i++) {
            Wire.beginTransmission(SLAVE_ADDRESS); // ��ʼ����豸ͨ��
            Wire.print(buff); // ��������
            byte error = Wire.endTransmission(); // ����ͨ�Ų���ȡ������
            delay(5);
        }
    }
    return 0;
}


int STSto(int argc, char **argv) {
    if (argc != 4) {
        shell.println("bad argument count");
        return -1;
    }

    auto id = atoi(argv[1]);
    auto pos = atoi(argv[2]);
    auto vel = atoi(argv[3]);

    // uint8_t dir = 0;
    // if (pos < 0) {
    //   dir = 1;
    //   pos = -pos;
    // }

    shell.print("calling ");
    shell.print(id);
    shell.print(" with ");
    shell.print(pos);
    shell.print(" and ");
    shell.println(vel);

    sms_sts.WritePosEx(id, pos, vel, 200);
    return 0;
}


int STSreadPos(int argc, char **argv) {
    if (argc != 2) {
        shell.println("bad argument count");
        return -1;
    }

    auto id = atoi(argv[1]);

    shell.printf(" ID:%d, Pos: %d\r\n ", id, sms_sts.ReadPos(id));
    return 0;
}


int LEDTest(int argc, char **argv) {
    if (argc != 2) {
        shell.println("bad argument count");
        return -1;
    }

    auto id = atoi(argv[1]);
    if (id == 1) {
        leds[0] = CRGB::Red;
    } else {
        leds[0] = CRGB::Black;
    }


    FastLED.show();

    return 0;
}

int Servo_Ctrl(int argc, char **argv) {

    return 0;
}

int move_Arm(int argc, char **argv) {
    if (argc == 3) {
        auto x = atoi(argv[1]);
        auto y = atoi(argv[2]);
        mobot.moveArmTo(x, y);
    }
    return 0;
}

int moveZ(int argc, char **argv) {
    if (argc == 2) {
        auto z = atoi(argv[1]);

        mobot.moveArmZ(z);
    }
    return 0;
}


int draw_Pic(int argc, char **argv) {
    if (argc == 4) {
        auto num = atoi(argv[1]);
        auto x = atoi(argv[2]);
        auto y = atoi(argv[3]);
        // mobot.drawLoop(700, 100, 288, 160, x, y, num);
        // mobot.drawLoop(700, 100, 80, 80, x, y, num);
    }
    return 0;
}

int move_To(int argc, char **argv) {
    if (argc == 4) {

    }
    return 0;
}

int setFontWidth(int argc, char **argv) {

    return 0;
}

int setWrite(int argc, char **argv) {

    return 0;
}

int drawLine(int argc, char **argv) {

    return 0;
}

int move_Get(int argc, char **argv) {
    char buff[7];
    if (argc == 4) {
        auto x = atof(argv[1]);
        auto y = atof(argv[2]);
        auto o = atof(argv[3]);
        sprintf(buff, "pen %d 1", Pen);
        // mobot.moveChassisTo(x, y, o);
        // mobot.stopChassis();
        for (uint i = 0; i < 5; i++) {
            Wire.beginTransmission(SLAVE_ADDRESS);
            Wire.print(buff);
            byte error = Wire.endTransmission();
            delay(5);
        } {
            delay(60);
        }
        sms_sts.WritePosEx(4, 1000, 2000, 200);
    }
    return 0;
} //CalibrationOfs

int STScenter(int argc, char **argv) {
    if (argc != 2) {
        shell.println("bad argument count");
        return -1;
    }
    auto id = atoi(argv[1]);
    sms_sts.CalibrationOfs(id);
    return 0;
}


int changePen(int argc, char **argv) {
    // if (argc == 3) {
    //     char buff[7];
    //     Pen = atoi(argv[1]);
    //     auto pos = atoi(argv[2]);
    //
    //     mobot.sendChangeCmd((DrawColor_t) Pen, (PenState_t) pos);
    // } else if (argc == 2) {
    //     Pen = atoi(argv[1]);
    //     mobot.takePen((DrawColor_t) Pen);
    // }
    return 0;
}

int takePen(int argc, char **argv) {
    if (argc == 2) {
        Pen = atoi(argv[1]);
        // mobot.takePen((DrawColor_t) Pen);
    }
    return 0;
}

int givePen(int argc, char **argv) {
    if (argc == 2) {
        Pen = atoi(argv[1]);
        // mobot.givePen((DrawColor_t) Pen);
    }
    return 0;
}


int move(int argc, char **argv) {
    if (argc == 4) {
        auto x = atof(argv[1]);
        auto y = atof(argv[2]);
        auto o = atof(argv[3]);

        // mobot.moveChassis(x, y, o);
    }
    return 0;
}

int turn(int argc, char **argv) {
    if (argc == 2) {
        auto o = atof(argv[1]);


    }
    return 0;
}

int turnTo(int argc, char **argv) {
    if (argc == 2) {
        auto o = atof(argv[1]);


    }
    return 0;
}


// with Serial pointer motor.setTorque(10);
void shellInit(void) {
    shell.addCommand(F("STS <id> <pos> <vel>"), STScmd);
    shell.addCommand(F("STSPOS <id>"), STSreadPos);
    shell.addCommand(F("RGB <id> "), LEDTest);
    shell.addCommand(F("Servo  <vel>"), Servo_Ctrl);
    shell.addCommand(F("Draw <id> <x> <y>"), draw_Pic);
    shell.addCommand(F("Arm <x> <y> "), move_Arm);
    shell.addCommand(F("movez <z>"), moveZ);
    shell.addCommand(F("MoveTo <x> <y> <o>"), move_To);
    shell.addCommand(F("Move <x> <y> <o>"), move);
    shell.addCommand(F("Turn <o>"), turn);
    shell.addCommand(F("TurnTo <o>"), turnTo);
    shell.addCommand(F("MoveGet <x> <y> <o>"), move_Get);
    shell.addCommand(F("STSPen <id> <pos> <vel> <num>"), Pencmd);
    shell.addCommand(F("STSCen <id>"), STScenter);
    shell.addCommand(F("Cpen <id> <pos>"), changePen);
    shell.addCommand(F("takePen <id>"), takePen);
    shell.addCommand(F("givePen <id>"), givePen);
    shell.addCommand(F("setWidth <num>"), setFontWidth);
    shell.addCommand(F("isWriting <num>"), setWrite); //drawLine
    shell.addCommand(F("drawLine <num>"), drawLine); //drawLine

}
