//
// Created by L2248 on 2025/6/6.
//


#include "main.h"

void ArmControlTask(void *pvParameters) {
    Serial1.println("Arm Task Init, Period 1ms");

    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(1); // 20ms周期

    // 初始化 xLastWakeTime
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // 等待到下一个周期
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        // armR._stepper->
    }
}