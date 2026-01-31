# PID-embedded
PID library for embedded microcontrollers

## Overview
This library provides a high-performance PID (Proportional-Integral-Derivative) controller implementation specifically tuned for real-time electric vehicle applications. Given the high-speed requirements of our target, the library is designed to execute from SRAM_ITC to ensure zero-wait-state performance and deterministic timing.

## Control Formula
The calculation follows the standard parallel form with an integrated derivative filter:

$$u(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau) d\tau + K_d \frac{de(t)}{dt}$$

## Racing-Specific Features
- Anti-Windup: Prevents integral saturation when the inverter hits torque limits, ensuring a fast recovery when the error signal changes direction.
- Derivative Low-Pass Filter: Includes an integrated filter on the D-term to mitigate electrical noise from ADC and Hall sensors (essential for our 50-tooth wheel setup).
- DTC Data Optimization: Structures are ready to be allocated in SRAM_DTC for the fastest possible data access by the 600MHz core.
- FreeRTOS Compatible: Designed to be called safely within high-priority tasks (e.g., 100Hz or 200Hz Control Loops).

## Integration Guide
### Order of inclusion
To avoid header conflicts and "missing type" errors encountered during development, always follow this inclusion order:

```C
#include <stdint.h>
#include "FreeRTOS.h"
#include "pid_controller.h"
```

### Memory Mapping
To maintain high-speed execution, ensure the compute function is mapped to the SRAM_ITC region:
```C
AT_QUICKACCESS_SECTION_CODE(float PID_Compute(PID_Handle_t *pid, float setpoint, float measurement));
```

### Tuning Parameters

| **Parameter** | **Description**                                | **Typical Value (FSAE)** |
| ------------- | ---------------------------------------------- | ------------------------ |
| **Kp**        | Proportional Gain: Immediate response to slip. | 1.0 - 2.5                |
| **Ki**        | Integral Gain: Removes steady-state error.     | 0.1 - 0.8                |
| **Kd**        | Derivative Gain: Dampens oscillations.         | 0.01 - 0.1               |
| **Ts**        | Sampling Time: Period of the control task.     | 0.01 (10ms)              |

### Usage Example (Traction Control - FreeRTOS)
```C
// Initialize the PID handle
PID_Handle_t tcPID;
PID_Init(&tcPID, 1.5f, 0.2f, 0.05f, 0.01f); // Kp, Ki, Kd, Ts

// In your Control Task (SRAM_ITC)
void vControlTask(void *pvParameters) {
    for(;;) {
        float slip_error = target_slip - current_calculated_slip;
        float torque_reduction = PID_Compute(&tcPID, target_slip, current_calculated_slip);
        
        apply_torque_limit(torque_reduction);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```
