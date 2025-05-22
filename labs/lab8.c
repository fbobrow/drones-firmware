#include "math.h"        // Math functions (e.g., sqrtf, roundf, powf)
#include "FreeRTOS.h"    // FreeRTOS core definitions (needed for task handling and timing)
#include "task.h"        // FreeRTOS task functions (e.g., vTaskDelay)
#include "supervisor.h"  // Functions to check flight status (e.g., supervisorIsArmed)
#include "commander.h"   // Access to commanded setpoints (e.g., commanderGetSetpoint)
#include "estimator.h"   // Estimation framework for sensor fusion
#include "motors.h"      // Low-level motor control interface (e.g., motorsSetRatio)
#include "debug.h"       // Debug printing functions (e.g., DEBUG_PRINT)
#include "log.h"         // Logging utilities to send data to the CFClient

// -------------------------------
// Global physical and timing constants
// -------------------------------
float pi = 3.1416;
float g = 9.81;   // Gravitational acceleration [m/s^2]
float dt = 0.002; // Control loop time step [s] (2 ms => 500 Hz)

// -------------------------------
// Global state and control variables
// -------------------------------
setpoint_t setpoint;
state_t state;
sensorData_t sensors;

// PWM values [0.0 – 1.0] and corresponding angular speeds for each motor
float pwm_1, pwm_2, pwm_3, pwm_4;
float omega_1, omega_2, omega_3, omega_4;

// Total thrust and torques in body frame
float f_t, tau_phi, tau_theta, tau_psi;

// Estimated Euler angles [rad] and angular rates [rad/s]
float phi, theta, psi;
float wx, wy, wz;

// Position and velocity in world frame
float x, y, z;
float vx, vy, vz;

// Accelerometer and gyroscope readings
float ax, ay, az;
float gx, gy, gz;

// TOF (altimeter) and optical flow sensor measurements
float d;
float px, py;

// Desired attitude and position references
float phi_r, theta_r, psi_r;
float x_r, y_r, z_r;

// Variables to send to CFClient via logging
float log_phi, log_theta, log_psi;
float log_x, log_y, log_z;
float log_vx, log_vy, log_vz;

// -------------------------------
// Logging group declaration
// -------------------------------
LOG_GROUP_START(stateEstimate)
LOG_ADD_CORE(LOG_FLOAT, roll, &log_phi)
LOG_ADD_CORE(LOG_FLOAT, pitch, &log_theta)
LOG_ADD_CORE(LOG_FLOAT, yaw, &log_psi)
LOG_ADD_CORE(LOG_FLOAT, x, &log_x)
LOG_ADD_CORE(LOG_FLOAT, y, &log_y)
LOG_ADD_CORE(LOG_FLOAT, z, &log_z)
LOG_ADD_CORE(LOG_FLOAT, vx, &log_vx)
LOG_ADD_CORE(LOG_FLOAT, vy, &log_vy)
LOG_ADD_CORE(LOG_FLOAT, vz, &log_vz)
LOG_GROUP_STOP(stateEstimate)

// -------------------------------
// Reads reference setpoints from commander
// -------------------------------
void reference()
{
    commanderGetSetpoint(&setpoint, &state);
    z_r = setpoint.position.z;
    x_r = setpoint.position.x;
    y_r = setpoint.position.y;
    psi_r = 0.0f; // Yaw reference is fixed
}

// -------------------------------
// Converts desired force/torques into motor commands
// -------------------------------
void mixer()
{
    // Geometry and motor model parameters
    static const float l = 35.0e-3;  // Distance from center to motor [m]
    static const float a_2 = 6.2e-8; // Quadratic motor model gain
    static const float a_1 = 2.4e-4; // Linear motor model gain
    static const float kl = 2.0e-08; // Thrust coefficient
    static const float kd = 2.0e-10; // Drag (torque) coefficient

    // Compute required motor speeds squared (omega^2)
    omega_1 = (1.0f / 4.0f) * (f_t / kl - tau_phi / (kl * l) - tau_theta / (kl * l) - tau_psi / kd);
    omega_2 = (1.0f / 4.0f) * (f_t / kl - tau_phi / (kl * l) + tau_theta / (kl * l) + tau_psi / kd);
    omega_3 = (1.0f / 4.0f) * (f_t / kl + tau_phi / (kl * l) + tau_theta / (kl * l) - tau_psi / kd);
    omega_4 = (1.0f / 4.0f) * (f_t / kl + tau_phi / (kl * l) - tau_theta / (kl * l) + tau_psi / kd);

    // Clamp to non-negative and take square root
    omega_1 = (omega_1 >= 0.0f) ? sqrtf(omega_1) : 0.0f;
    omega_2 = (omega_2 >= 0.0f) ? sqrtf(omega_2) : 0.0f;
    omega_3 = (omega_3 >= 0.0f) ? sqrtf(omega_3) : 0.0f;
    omega_4 = (omega_4 >= 0.0f) ? sqrtf(omega_4) : 0.0f;

    // Convert angular speed to PWM using motor model
    pwm_1 = a_2 * omega_1 * omega_1 + a_1 * omega_1;
    pwm_2 = a_2 * omega_2 * omega_2 + a_1 * omega_2;
    pwm_3 = a_2 * omega_3 * omega_3 + a_1 * omega_3;
    pwm_4 = a_2 * omega_4 * omega_4 + a_1 * omega_4;
}

// -------------------------------
// Sends PWM signals to motors (only if drone is armed)
// -------------------------------
void motors()
{
    if (supervisorIsArmed())
    {
        if (setpoint.position.z > 0)
        {
            // Apply calculated PWM values
            motorsSetRatio(MOTOR_M1, pwm_1 * UINT16_MAX);
            motorsSetRatio(MOTOR_M2, pwm_2 * UINT16_MAX);
            motorsSetRatio(MOTOR_M3, pwm_3 * UINT16_MAX);
            motorsSetRatio(MOTOR_M4, pwm_4 * UINT16_MAX);
        }
        else
        {
            // Apply minimum thrust for safety
            motorsSetRatio(MOTOR_M1, 0.1f * UINT16_MAX);
            motorsSetRatio(MOTOR_M2, 0.1f * UINT16_MAX);
            motorsSetRatio(MOTOR_M3, 0.1f * UINT16_MAX);
            motorsSetRatio(MOTOR_M4, 0.1f * UINT16_MAX);
        }
    }
    else
    {
        // Motors off if disarmed
        motorsStop();
    }
}

// -------------------------------
// Sensor fusion from estimator queue
// -------------------------------
void readSensors()
{
    measurement_t measurement;
    while (estimatorDequeue(&measurement))
    {
        switch (measurement.type)
        {
        case MeasurementTypeAcceleration:
            ax = -measurement.data.acceleration.acc.x * 9.81f;
            ay = -measurement.data.acceleration.acc.y * 9.81f;
            az = -measurement.data.acceleration.acc.z * 9.81f;
            break;
        case MeasurementTypeGyroscope:
            gx = measurement.data.gyroscope.gyro.x * pi / 180.0f;
            gy = measurement.data.gyroscope.gyro.y * pi / 180.0f;
            gz = measurement.data.gyroscope.gyro.z * pi / 180.0f;
            break;
        case MeasurementTypeTOF:
            d = measurement.data.tof.distance;
            break;
        case MeasurementTypeFlow:
            px = measurement.data.flow.dpixelx;
            py = measurement.data.flow.dpixely;
            break;
        default:
            break;
        }
    }
}

// -------------------------------
// Complementary filter for attitude estimation
// -------------------------------
void attitudeEstimator()
{
    static const float wc = 1.0; // Cutoff frequency for complementary filter

    // Use gyroscope for integration
    wx = gx;
    wy = gy;
    wz = gz;

    // Compute angles from accelerometer
    float phi_a = atan2f(-ay, -az);
    float theta_a = atan2f(ax, sqrt(ay * ay + az * az));

    // Integrate gyroscope rates (Euler)
    float phi_g = phi + (wx + wy * sinf(phi) * tanf(theta) + wz * cosf(phi) * tanf(theta)) * dt;
    float theta_g = theta + (wy * cosf(phi) - wz * sinf(phi)) * dt;
    float psi_g = psi + (wy * sinf(phi) / cosf(theta) + wz * cosf(phi) / cosf(theta)) * dt;

    // Complementary filter: blend gyro (high freq) and accel (low freq)
    phi = (1.0f - wc * dt) * phi_g + wc * dt * phi_a;
    theta = (1.0f - wc * dt) * theta_g + wc * dt * theta_a;
    psi = psi_g; // No absolute reference for yaw
}

// -------------------------------
// PD controller for attitude (roll, pitch, yaw)
// -------------------------------
void attitudeController()
{
    static const float I_xx = 20.0e-6;
    static const float I_yy = 20.0e-6;
    static const float I_zz = 40.0e-6;

    float kp = 240.28;
    float kd = 26.67;

    tau_phi = I_xx * (kp * (phi_r - phi) + kd * (0.0f - wx));
    tau_theta = I_yy * (kp * (theta_r - theta) + kd * (0.0f - wy));
    tau_psi = I_zz * ((kp / 4.0f) * (psi_r - psi) + (kd / 2.0f) * (0.0f - wz));
}

// -------------------------------
// Estimates vertical position using range sensor
// -------------------------------
void verticalEstimator()
{
    static const float ld = 100.0;     // Gain for velocity correction
    static const float lp = 14.14;     // Gain for position correction
    static const float dt_range = 0.05; // Update rate of range sensor

    // Predict z based on last velocity
    z = z + vz * dt;

    // Get range measurement corrected for orientation
    float z_m = d * cosf(phi) * cosf(theta);

    // Correct velocity and position estimates
    vz = vz + (ld * dt_range) * (z_m - z);
    z = z + (lp * dt_range) * (z_m - z);
}

// -------------------------------
// Vertical controller (PD+I)
// -------------------------------
float z_int; // Integral term for vertical position

void verticalController()
{
    float m = 37.0e-3;

    static const float kp = 5.41;
    static const float kd = 4.00;
    static const float ki = 5.41;

    // Compute total thrust required
    f_t = m * (g + ki * z_int + kp * (z_r - z) + kd * (0.0f - vz));
    z_int += (z_r - z) * dt; // Integrate altitude error
}

// -------------------------------
// Estimates horizontal position using flow sensor
// -------------------------------
void horizontalEstimator()
{
    static const float sigma = 0.2654; // Optical flow scaling factor
    static const float wc = 5.0;       // Correction frequency

    // Predict motion
    x += vx * dt;
    y += vy * dt;

    // Get corrected height for flow estimation
    float d = z / (cosf(phi) * cosf(theta));

    // Estimate velocity from optical flow
    float vx_m = (px * sigma + wy) * d;
    float vy_m = (py * sigma - wx) * d;

    // Apply complementary correction
    vx += wc * dt * (vx_m - vx);
    vy += wc * dt * (vy_m - vy);
}

// -------------------------------
// Horizontal controller (PD)
// -------------------------------
void horizontalController()
{
    static const float kp = 5.41;
    static const float kd = 4.00;

    // Inverse dynamics: convert position error to attitude references
    phi_r = -(1.0f / g) * (kp * (y_r - y) + kd * (0.0f - vy));
    theta_r = (1.0f / g) * (kp * (x_r - x) + kd * (0.0f - vx));
}

// -------------------------------
// Updates values to be logged in CFClient
// -------------------------------
void logger()
{
    log_phi = phi * 180.0f / pi;
    log_theta = -theta * 180.0f / pi;
    log_psi = psi * 180.0f / pi;
    log_x = x;
    log_y = y;
    log_z = z;
    log_vx = vx;
    log_vy = vy;
    log_vz = vz;
}

// -------------------------------
// Main application task (runs at 500 Hz)
// -------------------------------
void appMain(void *param)
{
    while (true)
    {
        reference();             // Update references from user commands
        readSensors();           // Read and parse sensor measurements
        attitudeEstimator();     // Estimate orientation
        verticalEstimator();     // Estimate altitude and vertical velocity
        horizontalEstimator();   // Estimate horizontal position and velocity
        horizontalController();  // Compute desired roll/pitch angles
        verticalController();    // Compute thrust
        attitudeController();    // Compute torques
        mixer();                 // Compute motor speeds and PWM
        motors();                // Apply motor commands
        logger();                // Send data to log
        vTaskDelay(pdMS_TO_TICKS(2)); // Wait 2 ms (500 Hz loop rate)
    }
}
