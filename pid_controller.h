/*
 * PID-embedded - pid library for embedded systems
 * Copyright (C) 2025 s-stasi
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

// Helper libraries for stronger types
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif
  /**
   * @brief PID Controller Instance Structure.
   * Allocate this in SRAM_DTC using AT_QUICKACCESS_SECTION_DATA for 0-wait-state access.
   */
  typedef struct
  {
    float kp;       /**< Proportional gain */
    float ki;       /**< Integral gain */
    float kd;       /**< Derivative gain */
    float setpoint; /**< Target value */
    float dt;       /**< Sampling time in seconds (e.g., 0.01f for 100Hz) */

    float integral;       /**< Integral accumulator */
    float previous_error; /**< Error from last cycle */

    float output_min; /**< Minimum controller output limit */
    float output_max; /**< Maximum controller output limit */

    float max_integral;  /**< Anti-windup upper limit */
    float min_integral;  /**< Anti-windup lower limit */
    bool anti_windup_en; /**< Anti-windup enablement flag */

    float d_alpha;         /**< Alpha for derivative EMA filter [0.0 - 1.0] */
    float prev_d_filtered; /**< State for the EMA filter */
    bool d_filter_en;      /**< Derivative filter enablement flag */

    bool is_auto;
    float manual_output;
  } PID_Controller_t;

  /**
   * @brief Initializes the PID instance with basic parameters.
   */
  void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float out_min, float out_max, float dt, bool is_auto);

  /**
   * @brief Computes the PID output.
   * @note Mark this with AT_QUICKACCESS_SECTION_CODE to run from SRAM_ITC at 600MHz.
   */
  float PID_Compute(PID_Controller_t *pid, float measurement);

  /** @brief Enables integral clamping (Anti-Windup). */
  void PID_EnableAntiWindup(PID_Controller_t *pid, float min_int, float max_int);

  /** @brief Disables integral clamping. */
  void PID_DisableAntiWindup(PID_Controller_t *pid);

  /** @brief Enables Exponential Moving Average filter on Derivative term. */
  void PID_EnableDerivativeFilter(PID_Controller_t *pid, float alpha);

  /** @brief Sets derivative filter using a cutoff frequency in Hz. */
  void PID_SetDerivativeFilterHz(PID_Controller_t *pid, float cutoff_hz);

  /** * @brief Switches between Manual and Automatic mode.
   * @param auto_mode True for PID control, False for Manual bypass.
   */
  void PID_SetMode(PID_Controller_t *pid, bool auto_mode);

  /**
   * @brief Sets the output value to be used in Manual mode and synchronizes the PID.
   * @param output The desired manual output (e.g., direct pedal torque).
   */
  void PID_SetManualOutput(PID_Controller_t *pid, float manual_out);

  /** @brief Resets the internal state (integral and previous error). */
  void PID_Reset(PID_Controller_t *pid);
#ifdef __cplusplus
}
#endif

#endif // PID_CONTROLLER_H

#ifdef PID_CONTROLLER_IMPLEMENTATION

void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float out_min, float out_max,float dt, bool is_auto)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->setpoint = 0.0f;
  pid->integral = 0.0f;
  pid->min_integral = 0;
  pid->max_integral = 0;
  pid->previous_error = 0.0f;
  pid->output_min = out_min;
  pid->output_max = out_max;
  pid->d_alpha = 0;
  pid->d_filter_en = false;
  pid->prev_d_filtered = 0;
  pid->anti_windup_en = false;
  pid->dt = dt;
  pid->is_auto = is_auto;
}

float PID_Compute(PID_Controller_t *pid, float measurement)
{
  if (!pid->is_auto)
  {
    return pid->manual_output;
  }

  float error = pid->setpoint - measurement;

  // Proportional component
  float P = pid->kp * error;

  // Integral component
  float I = pid->integral + (pid->ki * error * pid->dt);

  if (pid->anti_windup_en)
  {
    if (I > pid->max_integral)
      I = pid->max_integral;
    else if (I < pid->min_integral)
      I = pid->min_integral;
  }

  pid->integral = I;

  // Derivative component
  float D = (error - pid->previous_error) / pid->dt;
  if (pid->d_filter_en)
  {
    D = (pid->d_alpha * D) + ((1.0f - pid->d_alpha) * pid->prev_d_filtered);
    pid->prev_d_filtered = D;
  }
  D = pid->kd * D;

  pid->previous_error = error;

  float result = P + I + D;

  if (result > pid->output_max)
    result = pid->output_max;
  if (result < pid->output_min)
    result = pid->output_min;

  return result;
}

void PID_EnableAntiWindup(PID_Controller_t *pid, float minIntegral, float maxIntegral)
{
  pid->anti_windup_en = true;
  pid->min_integral = minIntegral;
  pid->max_integral = maxIntegral;
}

void PID_DisableAntiWindup(PID_Controller_t *pid)
{
  pid->anti_windup_en = false;
  pid->min_integral = 0;
  pid->max_integral = 0;
}

void PID_EnableDerivativeFilter(PID_Controller_t *pid, float alpha)
{
  pid->d_filter_en = true;
  pid->d_alpha = alpha;
}

void PID_SetDerivativeFilterHz(PID_Controller_t *pid, float cutoffHz)
{
  float tau = 1.0f / (2.0f * 3.14159f * cutoffHz);
  float alpha = pid->dt / (tau + pid->dt);
  PID_EnableDerivativeFilter(pid, alpha);
}

void PID_DisableDerivativeExpFilter(PID_Controller_t *pid)
{
  pid->d_filter_en = false;
  pid->d_alpha = 0;
}

void PID_SetMode(PID_Controller_t *pid, bool auto_mode)
{
  pid->is_auto = auto_mode;
}

void PID_SetManualOutput(PID_Controller_t *pid, float output)
{
  pid->manual_output = output;

  if (!pid->is_auto)
  {
    float error = pid->setpoint - 0;
    float P = pid->kp * error;
    float D = 0;

    pid->integral = output - P - D;

    if (pid->anti_windup_en)
    {
      if (pid->integral > pid->max_integral)
        pid->integral = pid->max_integral;
      else if (pid->integral < pid->min_integral)
        pid->integral = pid->min_integral;
    }
  }
}

void PID_Reset(PID_Controller_t *pid)
{
  pid->integral = 0;
  pid->previous_error = 0;
  pid->prev_d_filtered = 0;
}

#endif // PID_CONTROLLER_IMPLEMENTATION