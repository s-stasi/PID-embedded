#include <stdio.h>
#define PID_CONTROLLER_IMPLEMENTATION
#include "../pid_controller.h"

float simulate_motor(float current_speed, float torque, float dt) {
    const float friction = 0.1f;
    const float inertia = 0.5f;
    return current_speed + ((torque * inertia) - (current_speed * friction)) * dt;
}

int main() {
    PID_Controller_t tc_pid;
    float dt = 0.01f;
    float motor_speed = 0.0f;
    float target_speed = 50.0f;

    PID_Init(&tc_pid, 2.0f, 0.5f, 0.1f, 0.0f, 1000.0f, dt, true);
    PID_EnableAntiWindup(&tc_pid, 0.0f, 1000.0f);
    PID_SetDerivativeFilterHz(&tc_pid, 10.0f);

    printf("Time(s), Mode, Setpoint, Measurement, Output\n");

    PID_SetMode(&tc_pid, false);
    for (int i = 0; i < 50; i++) {
        float time = i * dt;
        float manual_pedal_torque = 300.0f;
        

        PID_SetManualOutput(&tc_pid, manual_pedal_torque);
        
        motor_speed = simulate_motor(motor_speed, manual_pedal_torque, dt);
        
        float output = PID_Compute(&tc_pid, motor_speed);
        printf("%.2f, MANUAL, %.2f, %.2f, %.2f\n", time, tc_pid.setpoint, motor_speed, output);
    }

    printf("--- SWITCH TO AUTO ---\n");
    PID_SetMode(&tc_pid, true); 
    tc_pid.setpoint = target_speed;

    for (int i = 50; i < 200; i++) {
        float time = i * dt;
        
        float torque_out = PID_Compute(&tc_pid, motor_speed);
        
        motor_speed = simulate_motor(motor_speed, torque_out, dt);
        
        printf("%.2f, AUTO, %.2f, %.2f, %.2f\n", time, tc_pid.setpoint, motor_speed, torque_out);
    }

    return 0;
}