#ifndef CRAZYFLIE_H
#define CRAZYFLIE_H

#include "FreeRTOS.h"
#include "task.h"
#include "led.h"
#include "debug.h"
#include "motors.h"
#include "math.h"

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



static float ax = 0.0f, ay = 0.0f, az = 0.0f;
static float gx = 0.0f, gy = 0.0f, gz = 0.0f;

#include "sensors.h"
static sensorData_t sensors;
static inline void readSensors(void) {
    sensorsAcquire(&sensors);
    ax = sensors.acc.x;
    ay = sensors.acc.y;
    az = sensors.acc.z;
    gx = sensors.gyro.x;
    gy = sensors.gyro.y;
    gz = sensors.gyro.z;
}

// #include "imu.h"
// static Axis3f acc, gyr;
// static inline void readSensors(void) {
//     imu6Read(&gyr,&acc);
//     ax = acc.x;
//     ay = acc.y;
//     az = acc.z;
//     gx = gyr.x;
//     gy = gyr.y;
//     gz = gyr.z;
// }

// #include "estimator.h"
// #include "stabilizer.h"
// static inline void readSensors(void) {
//     const sensorData_t* s = stateEstimatorGetSensorData();
//     ax = s->acc.x;
//     ay = s->acc.y;
//     az = s->acc.z;
//     gx = s->gyro.x;
//     gy = s->gyro.y;
//     gz = s->gyro.z;
// }

// #include "imu_bmi088.h"
// static Axis3f gyroRaw, accRaw;
// static inline void readSensors(void) {
//     imuBmi088GetData(&gyroRaw, &accRaw);  // Essa é a função correta para o BMI088

//     ax = accRaw.x;
//     ay = accRaw.y;
//     az = accRaw.z;

//     gx = gyroRaw.x;
//     gy = gyroRaw.y;
//     gz = gyroRaw.z;
// }

#include "supervisor.h"


#endif 