#ifndef CRAZYFLIE_H
#define CRAZYFLIE_H

#include "FreeRTOS.h"
#include "task.h"
#include "led.h"
#include "debug.h"
#include "motors.h"

static inline void led(uint32_t id, bool value) {
    ledSet(id, value);
}

static inline void delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static inline void task(void (*fn)(void)) {
    xTaskCreate((TaskFunction_t)fn, "TASK", configMINIMAL_STACK_SIZE * 2, NULL, 2, NULL);
}

static inline void print(const char* str) {
    DEBUG_PRINT(str);
}

static inline void motor(uint32_t id, float ratio) {
    motorsSetRatio(id, (uint16_t)(ratio * UINT16_MAX));
}

#include "stabilizer_types.h"
#include "commander.h"

setpoint_t setpoint;
state_t state;

int takeOff() {
    commanderGetSetpoint(&setpoint, &state);
    return setpoint.mode.x;
}

#endif 