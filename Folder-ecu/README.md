# LQR/LQG Fuel Injection ECU Firmware

Embedded implementation of the integral-augmented LQR (LQI) control law
designed and validated in `mathematical model.md` and
`controller_comparison.py`. This closes the gap in the original project,
which only had MATLAB simulation code (`Folder-ecu/Code.m`,
`lqr design.m`) and no actual embedded/ECU implementation despite the
project name.

## Files
- `config.h` — pins, sensor calibration, control gains, actuator limits, fault thresholds
- `main.ino` — real-time control loop: sensor read → (optional Kalman filter) → LQI control law → injector output → logging

## Hardware assumptions
- **MCU:** ESP32 (Arduino framework). Swap `analogRead`/`ledcWrite` calls for equivalents if targeting Arduino Uno/Mega or STM32.
- **AFR sensing:** analog voltage input from a wideband lambda controller IC (e.g. CJ125 + Bosch LSU 4.9, matching the sensor used elsewhere in your other project's hardware).
- **RPM sensing:** digital pulse input from a crank/ignition trigger, captured via interrupt (`onCrankPulse`), period-based RPM calculation.
- **Actuator output:** PWM-based pulse-width signal to an injector driver stage (MOSFET). Note in the code: this is a simplified academic-prototype mapping — a production ECU synchronizes injector pulses to crank angle via a dedicated hardware timer, not free-running PWM.

## Control architecture
```
AFR sensor ──┐
             ├──► [Kalman Filter, optional] ──► LQI control law ──► Injector PWM
RPM sensor ──┘                                        │
                                            xi = ∫(AFR_ref − AFR) dt
```
Control law (matches the corrected LQI formulation used in the report):
```
u = -Kx1*(AFR - AFR_ref) - Kx2*(RPM - RPM_ref) - Ki*xi
```

Two modes, toggled by `USE_KALMAN_FILTER` in `main.ino`:
- **`1` → LQG:** control law runs on the Kalman-filtered state estimate (recommended — filters sensor noise)
- **`0` → LQR:** control law runs directly on the raw sensor reading

## ⚠️ Before using on a real engine
The gains in `config.h` (`LQI_KX1`, `LQI_KX2`, `LQI_KI`, `KF_L1`, `KF_L2`) come
from the illustrative 2-state model (`A = [-0.4 0.1; 0.05 -0.2]`, `B = [0.3;
0.1]`) used throughout the MATLAB/Python design work — **not** from a
physically identified engine model. To use this on real hardware:

1. Identify real `A`, `B` matrices — either from first-principles engine
   dynamics (manifold filling, fuel puddle dynamics, etc.) or from system
   identification on logged sensor data (e.g. MATLAB `ssest`/`n4sid`).
2. Recompute `Kx1, Kx2, Ki` (MATLAB `lqr()` on the augmented system, or the
   `solve_continuous_are` approach in `controller_comparison.py`) and the
   Kalman gains `L1, L2`.
3. Replace the constants in `config.h` and the plant matrices hardcoded in
   `kalmanUpdate()` in `main.ino`.
4. Validate in simulation/HIL before connecting to an actual injector.
5. Keep a hardware watchdog/kill switch independent of this MCU — the
   firmware's fault-detection (`checkFault`) falls back to a fixed safe
   pulse width, but that alone is not a substitute for hardware safety
   interlocks on a real vehicle.

## Logging
Serial output (115200 baud) streams CSV rows:
```
time_ms, AFR_raw, RPM_raw, AFR_est, RPM_est, u, xi, fault
```
suitable for the same MATLAB/Python post-processing and plotting used
elsewhere in this project (e.g. `analyse_engine_health.m`-style scripts).
