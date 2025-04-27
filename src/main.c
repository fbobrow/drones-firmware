#include "crazyflie.h"

void motorTask(void) {
    while (1) {
        if (takeOff()) {
            motor(MOTOR_M1, 0.2f); 
            motor(MOTOR_M2, 0.2f); 
            motor(MOTOR_M3, 0.2f); 
            motor(MOTOR_M4, 0.2f); 
        }
        else {
            motor(MOTOR_M1, 0.0f);
            motor(MOTOR_M2, 0.0f);
            motor(MOTOR_M3, 0.0f);
            motor(MOTOR_M4, 0.0f);
        }
        delay(100);            
    }
}

void ledTask(void) {
    while (1) {
        led(0, true);  
        delay(500);    
        led(0, false); 
        delay(500);    
    }
}

// #include "stabilizer_types.h"
// #include "commander.h"

// setpoint_t setpoint;
// state_t state;

// void debugPrintVec3(const char *label, const vector_t *v) {
//   DEBUG_PRINT("%s: x=%.3f y=%.3f z=%.3f\n", label, (double)v->x, (double)v->y, (double)v->z);
// }

// void debugPrintAttitude(const char *label, const attitude_t *a) {
//   DEBUG_PRINT("%s: roll=%.3f pitch=%.3f yaw=%.3f\n", label, (double)a->roll, (double)a->pitch, (double)a->yaw);
// }

// void debugPrintQuat(const quaternion_t *q) {
//   DEBUG_PRINT("quat: q0=%.3f q1=%.3f q2=%.3f q3=%.3f\n", (double)q->q0, (double)q->q1, (double)q->q2, (double)q->q3);
// }

// void debugPrintSetpoint(const setpoint_t *sp) {
//   DEBUG_PRINT("---- SETPOINT ----\n");
//   DEBUG_PRINT("timestamp: %lu\n", sp->timestamp);
  
//   debugPrintAttitude("attitude", &sp->attitude);
//   debugPrintAttitude("attitudeRate", &sp->attitudeRate);
  
//   debugPrintQuat(&sp->attitudeQuaternion);
  
//   DEBUG_PRINT("thrust: %.3f\n", (double)sp->thrust);
  
//   debugPrintVec3("position", &sp->position);
//   debugPrintVec3("velocity", &sp->velocity);
//   debugPrintVec3("acceleration", &sp->acceleration);
//   debugPrintVec3("jerk", &sp->jerk);

//   DEBUG_PRINT("velocity_body: %s\n", sp->velocity_body ? "true" : "false");

//   DEBUG_PRINT("mode.x: %d\n", sp->mode.x);
//   DEBUG_PRINT("mode.y: %d\n", sp->mode.y);
//   DEBUG_PRINT("mode.z: %d\n", sp->mode.z);
//   DEBUG_PRINT("mode.roll: %d\n", sp->mode.roll);
//   DEBUG_PRINT("mode.pitch: %d\n", sp->mode.pitch);
//   DEBUG_PRINT("mode.yaw: %d\n", sp->mode.yaw);
//   DEBUG_PRINT("mode.quat: %d\n", sp->mode.quat);
// }

// void printTask(void) {
//     while (1) {
//         commanderGetSetpoint(&setpoint, &state);
//         debugPrintSetpoint(&setpoint);
//         delay(1000);
//     }
// }

void appMain(void *param) {
    task(motorTask);
    //task(ledTask);
    //task(printTask);
}