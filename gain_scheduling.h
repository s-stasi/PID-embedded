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

#ifndef GAIN_SCHEDULING_H
#define GAIN_SCHEDULING_H

#include "pid_controller.h"

#ifdef __cplusplus
extern "C"
{
#endif

  typedef struct {
    float kp;
    float ki;
    float kd;
  } PID_Gains_t;

  typedef struct {
    PID_Gains_t gainSet;
    float startsAtTarget;
  } GainScheduleEntry_t;
  
  typedef struct {
    GainScheduleEntry_t *entries;
    uint16_t numEntries;
    float histeresys;
    float lastMeasurement;
    uint16_t current_index;
  } GainSchedule_t;

  void GainSchedule_Init(GainSchedule_t *schedule, GainScheduleEntry_t *entries, uint16_t numEntries, float histeresys);
  void GainSchedule_InitPidController(PID_Controller_t *pid, GainSchedule_t *schedule, float initialMeasurement);
  void GainSchedule_Update(PID_Controller_t *pid, GainSchedule_t *schedule, float measurement);

#ifdef __cplusplus
}
#endif 
#endif // GAIN_SCHEDULING_H

// Define only for testing purposes
 #define GAIN_SCHEDULING_IMPLEMENTATION
//

#ifdef GAIN_SCHEDULING_IMPLEMENTATION

void GainSchedule_Init(GainSchedule_t *schedule, GainScheduleEntry_t *entries, uint16_t numEntries, float histeresys)
{
  schedule->entries = entries;
  schedule->numEntries = numEntries;
  schedule->histeresys = histeresys;
}

void GainSchedule_InitPidController(PID_Controller_t *pid, GainSchedule_t *schedule, float initialMeasurement){
  if (schedule == 0 || schedule->numEntries == 0 || pid == 0) {
    return;
  }
  GainSchedule_Update(pid, schedule, initialMeasurement);
}

void GainSchedule_Update(PID_Controller_t *pid, GainSchedule_t *schedule, float measurement)
{
  if (schedule == 0 || schedule->numEntries == 0 || pid == 0) {
    return;
  }

  uint16_t target_index = schedule->numEntries - 1;

  for (uint16_t i = 1; i < schedule->numEntries; i++) {
    float threshold = schedule->entries[i].startsAtTarget;
    float effective_threshold = threshold;

    // Logica di Isteresi basata sullo STATO (immune al rumore del sensore)
    if (schedule->current_index < i) {
      // Vogliamo "salire" (superare il muro verso destra). 
      // Dobbiamo battere la soglia PIÙ l'isteresi per sbloccare la nuova zona.
      effective_threshold += schedule->histeresys;
    } else {
      // Siamo già sopra, vogliamo "scendere" (tornare a sinistra).
      // Dobbiamo scendere SOTTO la soglia MENO l'isteresi per ricadere.
      effective_threshold -= schedule->histeresys;
    }

    if (measurement < effective_threshold) {
      target_index = i - 1;
      break;
    }
  }

  // Applichiamo i cambiamenti SOLO se abbiamo effettivamente cambiato zona
  if (target_index != schedule->current_index) {
    float new_kp = schedule->entries[target_index].gainSet.kp;
    float new_ki = schedule->entries[target_index].gainSet.ki;
    float new_kd = schedule->entries[target_index].gainSet.kd;

    // Compensazione integrale anti-kick
    float error = pid->setpoint - measurement;
    pid->integral += (pid->kp - new_kp) * error;
    
    if (pid->anti_windup_en) {
      if (pid->integral > pid->max_integral) pid->integral = pid->max_integral;
      else if (pid->integral < pid->min_integral) pid->integral = pid->min_integral;
    }

    // Aggiorniamo i parametri e lo stato del PID
    PID_SetGains(pid, new_kp, new_ki, new_kd);
    schedule->current_index = target_index; // Aggiorniamo la memoria del Gain Schedule
  }
}

#endif // GAIN_SCHEDULING_IMPLEMENTATION
