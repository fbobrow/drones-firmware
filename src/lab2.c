#include "FreeRTOS.h"      // FreeRTOS core definitions (needed for task handling and timing)
#include "task.h"          // FreeRTOS task functions (e.g., vTaskDelay)
#include "supervisor.h"    // Functions to check flight status (e.g., supervisorIsArmed)
#include "commander.h"     // Access to commanded setpoints (e.g., commanderGetSetpoint)
#include "motors.h"        // Low-level motor control interface (e.g., motorsSetRatio)

// Global variables to store the desired setpoint, the current state (not used here) and the computed PWM value.
setpoint_t setpoint;
state_t state;
float pwm;

// Main application
void appMain(void *param)
{
    // Infinite loop (runs forever)
    while (true)
    {
        // Check if the drone is armed (i.e., ready to fly)
        if (supervisorIsArmed())
        {
            // Fetch the latest setpoint from the commander and also fetch the current estimated state (not used here)
            commanderGetSetpoint(&setpoint, &state);

            // Compute a PWM value proportional to the commanded altitude (Z axis position)
            // The altitude command increases in 0.5 m steps, and we want the PWM to increase by 0.1 for each step.
            // Therefore, we divide Z by 5.0 so that: 0.5 m → 0.1 PWM
            pwm = (setpoint.position.z) / 5.0f;
        }
        else
        {
            // If not armed, stop the motor (set PWM to zero)
            pwm = 0.0f;
        }
        // Send the PWM signal to motor M1, scaling it to match the expected range [0, UINT16_MAX]
        motorsSetRatio(MOTOR_M1, pwm * UINT16_MAX);
        // Wait for 100 milliseconds before running the next iteration (10 Hz control loop)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
