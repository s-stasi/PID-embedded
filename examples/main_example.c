#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#define PID_CONTROLLER_IMPLEMENTATION
#define GAIN_SCHEDULING_IMPLEMENTATION
#include "../gain_scheduling.h"

float simulate_motor(float current_speed, float torque, float dt) {
  const float friction = 0.1f;
  const float inertia = 0.5f;
  return current_speed + ((torque * inertia) - (current_speed * friction)) * dt;
}

int main() {
  PID_Controller_t tc_pid;
  GainSchedule_t schedule;
  float dt = 0.01f;
  float motor_speed = 0.0f;

  // Gain Scheduling Walls based on Asphalt Friction Coefficient (mu)
  GainScheduleEntry_t entries[2] = {
    {{1.0f, 0.1f, 0.0f}, 0.0f},   // Zone 0: Wet Asphalt (Low mu) -> Conservative
    {{3.0f, 0.5f, 0.1f}, 0.5f}    // Zone 1: Dry Asphalt (High mu) -> Aggressive
  };

  // Hysteresis of 0.05 on the friction coefficient estimation
  GainSchedule_Init(&schedule, entries, 2, 0.05f);
  schedule.current_index = 1;

  // New torque limits: -200 (Regenerative braking) to +400 (Traction)
  PID_Init(&tc_pid, 3.0f, 0.5f, 0.1f, -200.0f, 400.0f, dt, true);
  PID_EnableAntiWindup(&tc_pid, -200.0f, 200.0f);
  PID_SetDerivativeFilterHz(&tc_pid, 10.0f);

  printf("Time(s), Setpoint, Measurement, Output, Kp, Zone, Mu\n");

  for (int i = 0; i < 2000; i++) {
    float time = i * dt;

    // The driver sets a cruise speed that oscillates
    tc_pid.setpoint = 60.0f + 20.0f * cosf(2.0f * 3.14159f * 0.1f * time);

    // Simulated road condition: mu oscillates between 0.2 (Wet) and 0.8 (Dry)
    float mu = 0.5f + 0.3f * sinf(2.0f * 3.14159f * 0.1f * time);

    // We schedule the gains based on the surface friction, NOT speed
    GainSchedule_Update(&tc_pid, &schedule, mu);
    
    float torque_out = PID_Compute(&tc_pid, motor_speed);
    motor_speed = simulate_motor(motor_speed, torque_out, dt);

    printf("%.2f, %.2f, %.2f, %.2f, %.2f, %d, %.3f\n",
           time, tc_pid.setpoint, motor_speed, torque_out,
           tc_pid.kp, schedule.current_index, mu);
  }

  return 0;
}