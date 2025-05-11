#include "crazyflie.h"

// Physical constants
const float pi = 3.1416;
const float g = 9.81; // m/s^2

// Quadcopter dimensions
const float m = 30.0e-3;    // kg
const float I_xx = 16.0e-6; // kg.m^2
const float I_yy = 16.0e-6; // kg.m^2
const float I_zz = 29.0e-6; // kg.m^2
const float l = 33.0e-3;    // m

// Motor constants
const float a_2 = 6.18e-8;
const float a_1 = 2.34e-4;

// Propeller constants
const float kl = 1.726e-08; // N.s^2/rad^2
const float kd = 1.426e-10; // N.m.s^2/rad^2

float f_t = 0.1f;
float tau_phi = 0.0f;
float tau_theta = 0.0f;
float tau_psi = 0.0f;

void mixer()
{
    float pwm_1 = 0.0f;
    float pwm_2 = 0.0f;
    float pwm_3 = 0.0f;
    float pwm_4 = 0.0f;
    if (supervisorIsArmed())
    {
        float omega_1 = (1.0f / 4.0f) * (f_t / kl - tau_phi / (kl * l) - tau_theta / (kl * l) - tau_psi / kd);
        float omega_2 = (1.0f / 4.0f) * (f_t / kl - tau_phi / (kl * l) + tau_theta / (kl * l) + tau_psi / kd);
        float omega_3 = (1.0f / 4.0f) * (f_t / kl + tau_phi / (kl * l) + tau_theta / (kl * l) - tau_psi / kd);
        float omega_4 = (1.0f / 4.0f) * (f_t / kl + tau_phi / (kl * l) - tau_theta / (kl * l) + tau_psi / kd);
        omega_1 = (omega_1 >= 0.0f) ? sqrtf(omega_1) : 0.0f;
        omega_2 = (omega_2 >= 0.0f) ? sqrtf(omega_2) : 0.0f;
        omega_3 = (omega_3 >= 0.0f) ? sqrtf(omega_3) : 0.0f;
        omega_4 = (omega_4 >= 0.0f) ? sqrtf(omega_4) : 0.0f;
        pwm_1 = a_2 * omega_1 * omega_1 + a_1 * omega_1;
        pwm_2 = a_2 * omega_2 * omega_2 + a_1 * omega_2;
        pwm_3 = a_2 * omega_3 * omega_3 + a_1 * omega_3;
        pwm_4 = a_2 * omega_4 * omega_4 + a_1 * omega_4;
        pwm_1 = 0.5f;
        pwm_2 = 0.5f;
        pwm_3 = 0.5f;
        pwm_4 = 0.5f;
    }
    motor(MOTOR_M1, pwm_1);
    motor(MOTOR_M2, pwm_2);
    motor(MOTOR_M3, pwm_3);
    motor(MOTOR_M4, pwm_4);
}

// void attitudeEstimator()
// {
//     readSensors();
//     DEBUG_PRINT("%f %f %f\n", (double)ax, (double)ay, (double)az);
// }

// void motorTask(void)
// {
//     while (1)
//     {
//         if (takeOff())
//         {
//             motor(MOTOR_M1, 0.2f);
//             motor(MOTOR_M2, 0.2f);
//             motor(MOTOR_M3, 0.2f);
//             motor(MOTOR_M4, 0.2f);
//         }
//         else
//         {
//             motor(MOTOR_M1, 0.0f);
//             motor(MOTOR_M2, 0.0f);
//             motor(MOTOR_M3, 0.0f);
//             motor(MOTOR_M4, 0.0f);
//         }
//         delay(100);
//     }
// }

// void ledTask(void)
// {
//     while (1)
//     {
//         led(0, true);
//         delay(500);
//         led(0, false);
//         delay(500);
//     }
// }

#include "stabilizer_types.h"
#include "commander.h"

setpoint_t setpoint;
state_t state;

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

void lab1()
{
    float pwm = 0.0f;
    if (supervisorIsArmed())
    {
        commanderGetSetpoint(&setpoint, &state);
        pwm = (setpoint.position.z) / 5.0f;
        DEBUG_PRINT("PWM: %.1f\n", (double)pwm);
    }
    motorsSetRatio(MOTOR_M1, pwm * UINT16_MAX);
}

void lab2()
{
    float pwm = 0.0f;
    if (supervisorIsArmed())
    {
        commanderGetSetpoint(&setpoint, &state);
        float omega = (setpoint.position.z) * 400.0f;
        DEBUG_PRINT("Omega (rad/s): %.0f\n", (double)omega);
        pwm = a_2 * omega * omega + a_1 * omega;
    }
    motorsSetRatio(MOTOR_M1, pwm * UINT16_MAX);
    motorsSetRatio(MOTOR_M2, pwm * UINT16_MAX);
    motorsSetRatio(MOTOR_M3, pwm * UINT16_MAX);
    motorsSetRatio(MOTOR_M4, pwm * UINT16_MAX);
}

void lab3()
{
    float pwm_1 = 0.0f;
    float pwm_2 = 0.0f;
    if (supervisorIsArmed())
    {
        commanderGetSetpoint(&setpoint, &state);
        if ((setpoint.position.z) > 0)
        {
            float omega_1 = 1000.0f;
            float omega_2 = 2000.0f;
            pwm_1 = a_2 * omega_1 * omega_1 + a_1 * omega_1;
            pwm_2 = a_2 * omega_2 * omega_2 + a_1 * omega_2;
        }
        else
        {
            pwm_1 = 0.1f;
            pwm_2 = 0.1f;
        }
    }
    motorsSetRatio(MOTOR_M1, pwm_1 * UINT16_MAX);
    motorsSetRatio(MOTOR_M2, pwm_2 * UINT16_MAX);
    motorsSetRatio(MOTOR_M3, pwm_1 * UINT16_MAX);
    motorsSetRatio(MOTOR_M4, pwm_2 * UINT16_MAX);
}

void appMain(void *param)
{
    // task(motorTask);
    // task(ledTask);
    // task(printTask);
    while (true)
    {
        // attitudeEstimator();
        // mixer();
        lab3();
        delay(100);
    }
}
