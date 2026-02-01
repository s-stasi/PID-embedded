# PID-Embedded: High-Performance Control for Racing ECUs

A lightweight, single-header PID library specifically engineered for **real-time electric vehicle applications**. Optimized for **NXP i.MX RT1064** (Cortex-M7 @ 600MHz) and critical control loops (Traction Control, Torque Vectoring).

---

## 🏎️ Racing-Specific Features

* **Bumpless Transfer (Auto/Manual):** Smooth transition between manual pilot input and automatic PID control. Synchronizes the integral term to prevent torque spikes during activation.
* **Integral Anti-Windup:** Prevents integral saturation during torque-limiting events (e.g., inverter max current reached), ensuring immediate recovery.
* **Derivative EMA Filter:** Integrated Exponential Moving Average filter on the derivative term to suppress electrical noise from 50-tooth Hall sensors and ADC jitter.
* **Frequency-Based Tuning:** Set your derivative filter using a cutoff frequency in **Hertz** instead of arbitrary alpha coefficients.
* **Deterministic Execution:** Optimized for **SRAM_ITC** (Instruction RAM) and **SRAM_DTC** (Data RAM) to achieve zero-wait-state performance at 600MHz.

---

## 📈 Control Formula

The library implements the parallel PID form with an integrated first-order low-pass filter on the derivative component:

---

## 🛠️ Integration Guide

### 1. Single-Header Implementation

To keep your project clean, this is a single-header library. In one C file, define the implementation macro before including the header:

```c
#define PID_CONTROLLER_IMPLEMENTATION
#include "pid_controller.h"

```

### 2. Proper Memory Mapping (RT1064 Specific)

To ensure deterministic 10ms control loops, place the instance in **DTC** and the logic in **ITC**:

```c
/* Allocate in Fast Data RAM (DTC) */
AT_QUICKACCESS_SECTION_DATA(PID_Controller_t tcPID);

/* Compute function should reside in Instruction RAM (ITC) */
AT_QUICKACCESS_SECTION_CODE(float PID_Compute(PID_Controller_t *pid, float measurement));

```

---

## ⚙️ Tuning Parameters

| Parameter | Description | Typical Value (FSAE) |
| --- | --- | --- |
| **Kp** | Proportional Gain: Immediate response to slip. | 1.0 - 2.5 |
| **Ki** | Integral Gain: Removes steady-state error. | 0.1 - 0.8 |
| **Kd** | Derivative Gain: Dampens oscillations. | 0.01 - 0.1 |
| **Cutoff Hz** | Filter frequency for the D-term. | 10Hz - 30Hz |

---

## 🚀 Usage Example: Traction Control

```c
void init_traction_control() {
    // Kp, Ki, Kd, OutMin, OutMax, Dt
    PID_Init(&tcPID, 1.8f, 0.4f, 0.05f, 0.0f, 1000.0f, 0.01f);
    PID_EnableAntiWindup(&tcPID, 0.0f, 1000.0f);
    PID_SetDerivativeFilterHz(&tcPID, 20.0f); // 20Hz Cutoff
}

void vControlTask(void *pvParameters) {
    for(;;) {
        float current_slip = calculate_slip();
        
        // PID_Compute handles the logic and ensures Bumpless Transfer
        float torque_out = PID_Compute(&tcPID, current_slip);
        
        CAN_SendTorque(torque_out);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

```

---

## 🧪 Simulation Results

The library has been verified using a simulated motor plant. The following log demonstrates the **Bumpless Transfer** during a mode switch at T=0.50s:

```text
Time(s), Mode,   Measurement, Output
0.48,    MANUAL, 71.76,       300.00
0.49,    MANUAL, 73.19,       300.00
--- SWITCH TO AUTO ---
0.50,    AUTO,   73.94,       164.01  <-- Smooth transition (no spike!)
0.51,    AUTO,   74.83,       144.05

```

---

## 📝 License

This project is licensed under the MIT License - see the LICENSE file for details.
