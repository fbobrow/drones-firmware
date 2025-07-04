#include "FreeRTOS.h"   // FreeRTOS core definitions (needed for task handling and timing)
#include "task.h"       // FreeRTOS task functions (e.g., vTaskDelay)
#include "supervisor.h" // Functions to check flight status (e.g., supervisorIsArmed)
#include "commander.h"  // Access to commanded setpoints (e.g., commanderGetSetpoint)
#include "motors.h"     // Low-level motor control interface (e.g., motorsSetRatio)
#include "debug.h"      // Debug printing functions (e.g., DEBUG_PRINT)
#include "math.h"       // Math functions (e.g., sqrtf, roundf, powf)

// Quadcopter parameters (derived in Lab 1)
const float m = 37.0e-3;    // Mass [kg]
const float I_xx = 20.0e-6; // Moment of inertia around X [kg·m^2]
const float I_yy = 20.0e-6; // Moment of inertia around Y [kg·m^2]
const float I_zz = 40.0e-6; // Moment of inertia around Z [kg·m^2]
const float l = 35.0e-3;    // Arm length from center to motor [m]

// Motor parameters (derived in Lab 2)
// These represent coefficients of the quadratic model: PWM = a_2 * omega^2 + a_1 * omega
const float a_2 = 6.18e-8; // Quadratic coefficient [PWM/(rad/s)^2]
const float a_1 = 2.34e-4; // Linear coefficient [PWM/(rad/s)]

// Propeller parameters (derived in Labs 3 and 4)
const float kl = 1.726e-08; // Thrust coefficient [N·s^2/rad^2]
const float kd = 1.426e-10; // Drag (torque) coefficient [N·m·s^2/rad^2]

// Global variables to store the desired setpoint, the current state (not used here),
// the computed PWM value, and the desired angular velocity (omega)
setpoint_t setpoint;
state_t state;

// PWM signals for each motor (normalized [0.0 – 1.0])
float pwm_1, pwm_2, pwm_3, pwm_4;

// Angular velocities for each motor [rad/s]
float omega_1, omega_2, omega_3, omega_4;

// Control efforts: total thrust and torques around X, Y, Z axes
float f_t, tau_phi, tau_theta, tau_psi;

// Computes control efforts based on desired position (from setpoint)
void command()
{
    // Fetch the latest setpoint from the commander and also fetch the current estimated state (not used here)
    commanderGetSetpoint(&setpoint, &state);

    // Compute control efforts (thrust and torques) based on setpoint position
    // These are proportional controllers for demonstration purposes
    f_t = roundf((setpoint.position.z) * 2.0f) / 100.0f;        // Thrust (N)
    tau_phi = -roundf((setpoint.position.y) * 2.0f) / 1000.0f;  // Roll torque (N·m)
    tau_theta = roundf((setpoint.position.x) * 2.0f) / 1000.0f; // Pitch torque (N·m)
    // tau_psi remains 0.0f (no yaw control in this version)

    // Print debug info for the control efforts
    DEBUG_PRINT("F_t (N): %.2f | tau_phi (N.m): %.3f | tau_theta (N.m): %.3f \n",
                (double)f_t, (double)tau_phi, (double)tau_theta);
}

// Mixer function: converts total thrust and torques into individual motor speeds
void mixer()
{
    // Compute squared angular velocities for each motor based on control allocation matrix
    omega_1 = (1.0f / 4.0f) * (f_t / kl - tau_phi / (kl * l) - tau_theta / (kl * l) - tau_psi / kd);
    omega_2 = (1.0f / 4.0f) * (f_t / kl - tau_phi / (kl * l) + tau_theta / (kl * l) + tau_psi / kd);
    omega_3 = (1.0f / 4.0f) * (f_t / kl + tau_phi / (kl * l) + tau_theta / (kl * l) - tau_psi / kd);
    omega_4 = (1.0f / 4.0f) * (f_t / kl + tau_phi / (kl * l) - tau_theta / (kl * l) + tau_psi / kd);

    // Convert to actual angular velocities by taking square roots (ensuring non-negativity)
    omega_1 = (omega_1 >= 0.0f) ? sqrtf(omega_1) : 0.0f;
    omega_2 = (omega_2 >= 0.0f) ? sqrtf(omega_2) : 0.0f;
    omega_3 = (omega_3 >= 0.0f) ? sqrtf(omega_3) : 0.0f;
    omega_4 = (omega_4 >= 0.0f) ? sqrtf(omega_4) : 0.0f;

    // Convert angular velocities to PWM commands using the quadratic motor model
    pwm_1 = a_2 * omega_1 * omega_1 + a_1 * omega_1;
    pwm_2 = a_2 * omega_2 * omega_2 + a_1 * omega_2;
    pwm_3 = a_2 * omega_3 * omega_3 + a_1 * omega_3;
    pwm_4 = a_2 * omega_4 * omega_4 + a_1 * omega_4;
}

// Main application loop
void appMain(void *param)
{
    // Infinite loop (runs forever)
    while (true)
    {
        // Check if the drone is armed (i.e., ready to fly)
        if (supervisorIsArmed())
        {
            // Step 1: compute control efforts based on setpoint
            command();
            // Step 2: compute motor speeds and PWM from control efforts
            mixer();
        }
        else
        {
            // If not armed, stop the motors (set PWM to zero)
            pwm_1 = pwm_2 = pwm_3 = pwm_4 = 0.0f;
        }

        // Step 3: apply PWM signals to motors (scaled to 16-bit range)
        motorsSetRatio(MOTOR_M1, pwm_1 * UINT16_MAX);
        motorsSetRatio(MOTOR_M2, pwm_2 * UINT16_MAX);
        motorsSetRatio(MOTOR_M3, pwm_3 * UINT16_MAX);
        motorsSetRatio(MOTOR_M4, pwm_4 * UINT16_MAX);

        // Wait for 100 milliseconds before running the next iteration (10 Hz control loop)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}