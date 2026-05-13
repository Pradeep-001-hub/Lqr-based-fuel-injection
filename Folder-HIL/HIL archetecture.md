# Hardware-in-Loop (HIL) System for LQR Fuel Injection Control

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      REAL-TIME PC                            │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  MATLAB/Simulink Real-Time Engine Model             │    │
│  │  (Fuel injection dynamics, combustion delay,        │    │
│  │   fuel film, sensor simulation with noise)          │    │
│  └─────────────────────────────────────────────────────┘    │
│         ↕ USB/Serial/CAN at 115200 baud                     │
└─────────────────────────────────────────────────────────────┘
                           ↓↑
                    ┌─────────────┐
                    │    DAQ       │
                    │  (Optional)  │
                    └─────────────┘
                           ↓↑
┌─────────────────────────────────────────────────────────────┐
│               MICROCONTROLLER (ARM/ESP32)                    │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  Real Controller Implementation:                    │    │
│  │  • PID Loop         (20 lines)                      │    │
│  │  • LQR Loop         (15 lines)                      │    │
│  │  • LQG Loop         (25 lines with Kalman filter)   │    │
│  │                                                      │    │
│  │  Runs at 100Hz control loop                         │    │
│  └─────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
```

## HIL Communication Protocol

**Real-Time Loop (100Hz = 10ms cycle):**

```
Host PC (Simulink)          Microcontroller
     ↓ Send State            
     │──────────────────────→ λ, RPM, sensor_noise
     │                        
     │ Compute Control       
     │                       [K * x]
     │← Receive Control ─────┤
     │   u (injector PWM)    
     │
     ├─ Integrate Plant Model
     │  ẋ = Ax + Bu
     │
     └ Wait ~10ms for next cycle
```

**Packet Format (12 bytes, binary):**

```
Host → MCU (State packet):
[0x55][lambda_f32][rpm_f32][0xAA]

MCU → Host (Control packet):
[0x66][u_pwm_u16][status_u16][0xBB]
```

---

## Hardware Requirements

### Minimum Setup:
- **Microcontroller:** Arduino Due, STM32F407, or ESP32-S3
- **Serial Interface:** USB-to-Serial (FTDI FT232RL)
- **Clock:** 84+ MHz ARM (for real-time determinism)
- **RAM:** 32 KB minimum (for state vectors + Kalman filter)

### Optional (Full Professional HIL):
- **Real-Time OS:** xPC Target, VxWorks, OSEK
- **dSPACE/National Instruments HIL:** LabVIEW RT + DAQ hardware
- **CAN Interface:** For automotive-grade communication
- **Power Supply:** Regulated 5V/12V for microcontroller + sensors

---

## Control Loop Frequency & Timing

| Component | Frequency | Period | Notes |
|-----------|-----------|--------|-------|
| Plant Integration (Simulink) | 1000 Hz | 1 ms | High fidelity fuel dynamics |
| Communication | 100 Hz | 10 ms | Control loop rate |
| Controller (MCU) | 100 Hz | 10 ms | Fuel injection cycle |
| Data Logging | 10 Hz | 100 ms | Telemetry to disk |
| Monitor Dashboard | 10 Hz | 100 ms | Real-time visualization |

---

## Validation Metrics

**Expected Results (PID vs LQR vs LQG):**

| Metric | PID | LQR | LQG | Real-Time Overhead |
|--------|-----|-----|-----|-------------------|
| Overshoot (%) | 12-18 | 4-8 | 2-4 | N/A |
| Settling Time (ms) | 400-600 | 200-300 | 150-250 | N/A |
| Control Effort (integral of u²) | 85-120 | 45-65 | 40-55 | N/A |
| Noise Rejection (SNR) | Poor | Good | Excellent | N/A |
| **MCU CPU Load (%)** | **8-12** | **15-22** | **25-35** | Critical |
| **Memory Usage (KB)** | **2-3** | **4-6** | **8-12** | Important |
| **Execution Time (μs)** | **50-100** | **80-150** | **200-300** | Must fit in 10ms |

-



---



