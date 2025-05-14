#include "FreeRTOS.h"      // FreeRTOS core definitions (needed for task handling and timing)
#include "task.h"          // FreeRTOS task functions (e.g., vTaskDelay)
#include "debug.h"         // Debug printing functions (e.g., DEBUG_PRINT)
#include "sensors.h"
#include "math.h"
#include "log.h"

//
static sensorData_t sensors;

//
float phi, theta, psi;
float log_phi, log_theta, log_psi;

// Main application loop
void appMain(void *param)
{
    // Infinite loop (runs continuously while the drone is powered on)
    while (true)
    {
        //
        sensorsAcquire(&sensors);
        //
        float phi_a = atan2f(sensors.acc.y,sensors.acc.z);
        float phi_g = phi + sensors.gyro.x*0.002f*3.14f/180.0f;
        phi = 0.998f * phi_g + 0.002f * phi_a;
        log_phi = phi*180.0f/3.14f;
        //
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

//
LOG_GROUP_START(stateEstimate)
LOG_ADD_CORE(LOG_FLOAT, roll, &log_phi)
LOG_ADD_CORE(LOG_FLOAT, pitch, &log_theta)
LOG_ADD_CORE(LOG_FLOAT, yaw, &log_psi)
LOG_GROUP_STOP(stateEstimate)

