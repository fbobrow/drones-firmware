#include "FreeRTOS.h" // FreeRTOS core definitions (needed for task handling and timing)
#include "task.h"     // FreeRTOS task functions (e.g., vTaskDelay)
#include "debug.h"    // Debug printing functions (e.g., DEBUG_PRINT)
#include "sensors.h"  // Sensor data acquisition (accelerometer and gyroscope)
#include "math.h"     // Math functions (e.g., atan2f, sinf, cosf)
#include "log.h"      // Logging utilities to send data to the CFClient

// Attitude estimator configuration
const float dt = 0.002;            // Time step [s] (2 ms sampling time)
const float omega_c_att = 1.0;     // Cutoff frequency [rad/s] for complementary filter

// Sensor data structure to store IMU measurements
sensorData_t sensors;

// Estimated angles (Euler angles) and angular velocities
float phi, theta, psi;             // Roll (ϕ), Pitch (θ), Yaw (ψ) angles [rad]
float wx, wy, wz;                  // Angular rates [rad/s]
float log_phi, log_theta, log_psi;// Logged values for visualization in degrees

// Define log group for sending roll, pitch, yaw to the client (e.g., CFClient)
LOG_GROUP_START(stateEstimate)
LOG_ADD_CORE(LOG_FLOAT, roll, &log_phi)
LOG_ADD_CORE(LOG_FLOAT, pitch, &log_theta)
LOG_ADD_CORE(LOG_FLOAT, yaw, &log_psi)
LOG_GROUP_STOP(stateEstimate)

void attitudeEstimator()
{
    // Acquire raw accelerometer and gyroscope data from sensors
    sensorsAcquire(&sensors);

    // Convert accelerometer data from g to m/s² and apply sign correction
    float ax = -sensors.acc.x * 9.81f;
    float ay = -sensors.acc.y * 9.81f;
    float az = -sensors.acc.z * 9.81f;

    // Convert gyroscope data from deg/s to rad/s
    float gx = sensors.gyro.x * 3.14f / 180.0f;
    float gy = sensors.gyro.y * 3.14f / 180.0f;
    float gz = sensors.gyro.z * 3.14f / 180.0f;

    // Store angular rates for possible use elsewhere
    wx = gx;
    wy = gy;
    wz = gz;

    // Estimate roll and pitch angles from accelerometer (static tilt)
    float phi_a = atan2f(-ay, -az);                                 // Roll from accelerometer
    float theta_a = atan2f(ax, sqrt(ay * ay + az * az));           // Pitch from accelerometer

    // Integrate angular velocity to get angles from gyroscope
    float phi_g = phi + (wx + wy * sinf(phi) * tanf(theta) + wz * cosf(phi) * tanf(theta)) * dt;
    float theta_g = theta + (wy * cosf(phi) - wz * sinf(phi)) * dt;
    float psi_g = psi + (wy * sinf(phi) / cosf(theta) + wz * cosf(phi) / cosf(theta)) * dt;

    // Complementary filter: fuse accelerometer (long-term) and gyro (short-term)
    phi = (1.0f - (omega_c_att * dt)) * phi_g + (omega_c_att * dt) * phi_a;
    theta = (1.0f - (omega_c_att * dt)) * theta_g + (omega_c_att * dt) * theta_a;
    psi = psi_g; // No yaw correction from accelerometer (unobservable without magnetometer)

    // Convert estimated angles to degrees and adjust sign for display
    log_phi = phi * 180.0f / 3.14f;
    log_theta = -theta * 180.0f / 3.14f;  // Negative sign for CFClient convention
    log_psi = psi * 180.0f / 3.14f;
}

// Main application loop
void appMain(void *param)
{
    // Infinite loop (runs continuously while the drone is powered on)
    while (true)
    {
        // Run the attitude estimator (sample and update orientation)
        attitudeEstimator();

        // Wait for 2 milliseconds before next iteration (500 Hz loop rate)
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
