#include "FreeRTOS.h"   // FreeRTOS core definitions (needed for task handling and timing)
#include "task.h"       // FreeRTOS task functions (e.g., vTaskDelay)
#include "supervisor.h" // Functions to check flight status (e.g., supervisorIsArmed)
#include "commander.h"  // Access to commanded setpoints (e.g., commanderGetSetpoint)
#include "sensors.h"    // Sensor data acquisition (e.g., sensorsAcquire)
#include "motors.h"     // Low-level motor control interface (e.g., motorsSetRatio)
#include "debug.h"      // Debug printing functions (e.g., DEBUG_PRINT)
#include "math.h"       // Math functions (e.g., sqrtf, roundf, powf)
#include "log.h"        // Logging utilities to send data to the CFClient

// Physical constants
float pi = 3.1416;
float g = 9.81;       // m/s^2

// Quadcopter parameters (Lab 1)
float m = 37.0e-3;    // Mass [kg]
float I_xx = 20.0e-6; // Moment of inertia around X [kg·m^2]
float I_yy = 20.0e-6; // Moment of inertia around Y [kg·m^2]
float I_zz = 40.0e-6; // Moment of inertia around Z [kg·m^2]
float l = 35.0e-3;    // Arm length from center to motor [m]

// Motor parameters (Lab 2)
float a_2 = 6.18e-8; // Quadratic coefficient [PWM/(rad/s)^2]
float a_1 = 2.34e-4; // Linear coefficient [PWM/(rad/s)]

// Propeller parameters (Labs 3 and 4)
float kl = 1.726e-08; // Lift coefficient [N·s^2/rad^2]
float kd = 1.426e-10; // Drag coefficient [N·m·s^2/rad^2]

// Attitude estimator configuration (Lab 6)
float dt = 0.002;        // Time step [s] (2 ms sampling time)
float wc_att = 1.0; 

// Attitude controller gains (roll/pitch)
float kp_phi = 240.28;       
float kd_phi = 26.67; 
float kp_theta = 240.28;       
float kd_theta = 26.67; 
float kd_psi = 60.07;       
float kp_psi = 13.33; 

// Global variables to store the desired setpoint, the current state (not used here),
// the computed PWM value, and the desired angular velocity (omega)
setpoint_t setpoint;
state_t state;
sensorData_t sensors;

// PWM signals for each motor (normalized [0.0 – 1.0])
float pwm_1, pwm_2, pwm_3, pwm_4;
float omega_1, omega_2, omega_3, omega_4;
float f_t, tau_phi, tau_theta, tau_psi;
float phi, theta, psi; // Roll (ϕ), Pitch (θ), Yaw (ψ) angles [rad]
float wx, wy, wz;      // Angular rates [rad/s]

//
float ax, ay, az;
float gx, gy, gz;

//
float phi_r, theta_r, psi_r;

// Define log group for sending roll, pitch, yaw to the client (e.g., CFClient)
float log_phi, log_theta, log_psi; // Logged values for visualization in degrees
// LOG_GROUP_START(stateEstimate)
// LOG_ADD_CORE(LOG_FLOAT, roll, &log_phi)
// LOG_ADD_CORE(LOG_FLOAT, pitch, &log_theta)
// LOG_ADD_CORE(LOG_FLOAT, yaw, &log_psi)
// LOG_GROUP_STOP(stateEstimate)

// Computes control efforts based on desired position (from setpoint)
void reference()
{
    // Fetch the latest setpoint from the commander and also fetch the current estimated state (not used here)
    commanderGetSetpoint(&setpoint, &state);
    // Compute control efforts (thrust and torques) based on setpoint position
    // These are proportional controllers for demonstration purposes
    f_t = roundf((setpoint.position.z) * 2.0f) / 100.0f;        // Thrust (N)
    phi_r = 0.0f;
    theta_r = 0.0f;
    psi_r = 0.0f;                                             
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

void motors()
{
    // Check if drone is armed (ready to fly)
    if (supervisorIsArmed())
    {
        // Apply PWM to motors (scaled to 16-bit range)
        motorsSetRatio(MOTOR_M1, pwm_1 * UINT16_MAX);
        motorsSetRatio(MOTOR_M2, pwm_2 * UINT16_MAX);
        motorsSetRatio(MOTOR_M3, pwm_3 * UINT16_MAX);
        motorsSetRatio(MOTOR_M4, pwm_4 * UINT16_MAX);
    }
    else
    {
        motorsStop();
    }
}

void imu()
{
    // Acquire raw accelerometer and gyroscope data from sensors
    sensorsAcquire(&sensors);

    // Convert accelerometer data from g to m/s² and apply sign correction
    ax = -sensors.acc.x * 9.81f;
    ay = -sensors.acc.y * 9.81f;
    az = -sensors.acc.z * 9.81f;

    // Convert gyroscope data from deg/s to rad/s
    gx = sensors.gyro.x * 3.14f / 180.0f;
    gy = sensors.gyro.y * 3.14f / 180.0f;
    gz = sensors.gyro.z * 3.14f / 180.0f;
}

// Estimates roll, pitch and yaw from sensor readings using a complementary filter
void attitudeEstimator()
{
    // Store angular rates for possible use elsewhere
    wx = gx;
    wy = gy;
    wz = gz;

    // Estimate roll and pitch angles from accelerometer (static tilt)
    float phi_a = atan2f(-ay, -az);                      // Roll from accelerometer
    float theta_a = atan2f(ax, sqrt(ay * ay + az * az)); // Pitch from accelerometer

    // Integrate angular velocity to get angles from gyroscope
    float phi_g = phi + (wx + wy * sinf(phi) * tanf(theta) + wz * cosf(phi) * tanf(theta)) * dt;
    float theta_g = theta + (wy * cosf(phi) - wz * sinf(phi)) * dt;
    float psi_g = psi + (wy * sinf(phi) / cosf(theta) + wz * cosf(phi) / cosf(theta)) * dt;

    // Complementary filter: fuse accelerometer (long-term) and gyro (short-term)
    phi = (1.0f - (wc_att * dt)) * phi_g + (wc_att * dt) * phi_a;
    theta = (1.0f - (wc_att * dt)) * theta_g + (wc_att * dt) * theta_a;
    psi = psi_g; // No yaw correction from accelerometer (unobservable without magnetometer)

}

//
void attitudeController()
{
    //
    tau_phi = I_xx*(kp_phi*(phi_r-phi)+kd_phi*(0.0f-wx));
    tau_theta = I_yy*(kp_theta*(theta_r-theta)+kd_theta*(0.0f-wy));
    tau_psi = I_zz*(kp_psi*(psi_r-psi)+kd_psi*(0.0f-wz));
}

void logger()
{
    // Convert estimated angles to degrees and adjust sign for display
    log_phi = phi * 180.0f / 3.14f;
    log_theta = -theta * 180.0f / 3.14f; // Negative sign for CFClient convention
    log_psi = psi * 180.0f / 3.14f;
}

// Main application loop
void appMain(void *param)
{
    // Infinite loop (runs forever)
    while (true)
    {
        // Compute control inputs from position setpoint
        reference();
        //
        imu();
        // Estimate attitude based on sensor data
        attitudeEstimator();
        //
        attitudeController();
        // Mix control efforts and send commands to motors
        mixer();
        //
        motors();
        //
        //logger();
        // Wait for 2 milliseconds before running the next iteration (500 Hz control loop)
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}