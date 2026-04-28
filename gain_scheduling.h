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
  } GainSchedule_t;

  void GainSchedule_Init(GainSchedule_t *schedule, GainScheduleEntry_t *entries, uint16_t numEntries);
  void GainSchedule_InitPidController(PID_Controller_t *pid, GainSchedule_t *schedule, float initialMeasurement){
  void GainSchedule_Update(PID_Controller_t *pid, GainSchedule_t *schedule, float measurement);

#ifdef __cplusplus
}
#endif 
#endif // GAIN_SCHEDULING_H

// Define only for testing purposes
// #define GAIN_SCHEDULING_IMPLEMENTATION
//

#ifdef GAIN_SCHEDULING_IMPLEMENTATION

void GainSchedule_Init(GainSchedule_t *schedule, GainScheduleEntry_t *entries, uint16_t numEntries)
{
  schedule->entries = entries;
  schedule->numEntries = numEntries;
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

  uint16_t selected_index = schedule->numEntries - 1;

  for (uint16_t i = 1; i < schedule->numEntries; i++) {
    if (measurement < schedule->entries[i].startsAtTarget) {
      selected_index = i - 1;
      break;
    }
  }

  PID_SetGains(pid, 
               schedule->entries[selected_index].gainSet.kp, 
               schedule->entries[selected_index].gainSet.ki, 
               schedule->entries[selected_index].gainSet.kd);
}

#endif // GAIN_SCHEDULING_IMPLEMENTATION
