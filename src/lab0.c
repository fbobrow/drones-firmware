#include "FreeRTOS.h"   // FreeRTOS core definitions (needed for task handling and timing)
#include "task.h"       // FreeRTOS task functions (e.g., vTaskDelay)
#include "supervisor.h" // Functions to check flight status (e.g., supervisorIsArmed)
#include "motors.h"     // Low-level motor control interface (e.g., motorsSetRatio)
#include "debug.h"      // Debug printing functions (e.g., DEBUG_PRINT)

#include "estimator.h"

float ax, ay, az;
float gx, gy, gz;
float d;
float px, py;
float dt;

// Main application loop
void appMain(void *param)
{
    // Infinite loop (runs continuously while the drone is powered on)
    while (true)
    {

        // Wait for 100 milliseconds before checking again (10 Hz loop)
        vTaskDelay(pdMS_TO_TICKS(100));

        measurement_t m;
        while (estimatorDequeue(&m))
        {
            switch (m.type)
            {
            case MeasurementTypeGyroscope:
                gx = m.data.gyroscope.gyro.x;
                gy = m.data.gyroscope.gyro.y;
                gz = m.data.gyroscope.gyro.z;
                break;
            case MeasurementTypeAcceleration:
                ax = m.data.acceleration.acc.x;
                ay = m.data.acceleration.acc.y;
                az = m.data.acceleration.acc.z;
                break;
            case MeasurementTypeTOF:
                d = m.data.tof.distance;
            case MeasurementTypeFlow:
                px = m.data.flow.dpixelx;
                py = m.data.flow.dpixely;
                dt = m.data.flow.dt;
            default:
                break;
            }
        }
        DEBUG_PRINT("Acc: %4.2f %4.2f %4.2f | Gyr: %6.2f %6.2f %6.2f | Dis: %4.2f | Flow: %2.0f %2.0f %6.4f\n",(double)ax,(double)ay,(double)az,(double)gx,(double)gy,(double)gz,(double)d,(double)px,(double)py,(double)dt);
    }
}
