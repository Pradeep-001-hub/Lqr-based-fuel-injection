#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
//  LQR/LQG FUEL INJECTION ECU FIRMWARE - CONFIGURATION
//  Target: ESP32 (adjust pins if using Arduino Uno/Mega/STM32)
// ============================================================

// ---------------- Pin Definitions ----------------
#define PIN_AFR_ANALOG      34   // ADC pin: wideband lambda controller output (e.g. CJ125 UA)
#define PIN_RPM_PULSE        4   // Digital interrupt pin: crank/ignition pulse input
#define PIN_INJECTOR_PWM    25   // PWM output driving injector MOSFET/driver stage
#define PIN_STATUS_LED       2   // Onboard LED for fault/status indication

// ---------------- Sensor Calibration ----------------
// Lambda sensor (e.g. Bosch LSU 4.9 via CJ125) output voltage -> lambda mapping.
// CALIBRATE THESE FOR YOUR ACTUAL SENSOR/IC BEFORE USE. Values below are
// illustrative placeholders based on typical CJ125 UA characteristics.
#define ADC_VREF            3.3f
#define ADC_MAX_COUNTS      4095.0f
#define LAMBDA_AT_VMIN      0.65f   // lambda value at ADC minimum voltage
#define LAMBDA_AT_VMAX      1.50f   // lambda value at ADC maximum voltage
#define AFR_STOICH          14.7f   // stoichiometric AFR (gasoline)

// RPM measurement: pulses per crank revolution (adjust to your trigger wheel/sensor)
#define PULSES_PER_REV       1

// ---------------- Control Loop Timing ----------------
#define CONTROL_LOOP_HZ      100      // control law update rate (Hz)
#define CONTROL_DT_S          (1.0f / CONTROL_LOOP_HZ)

// ---------------- LQR / LQI Gains ----------------
// *** THESE GAINS COME FROM THE ILLUSTRATIVE 2-STATE MODEL IN THE MATLAB/
// PYTHON DESIGN SCRIPTS (A,B,Q,R as used in controller_comparison.py). ***
// They are NOT derived from a physically identified engine model. Before
// running this on real hardware/engine, you MUST re-derive A,B from either
// (a) first-principles engine dynamics, or (b) system identification on
// real sensor data, then recompute these gains (MATLAB lqr()/scipy
// solve_continuous_are) and replace the values below. Running unvalidated
// gains on a real engine can cause lean/rich misfire, knock, or component
// damage -- validate in simulation/HIL first.
#define LQI_KX1              6.5952f   // gain on AFR state
#define LQI_KX2              0.8017f   // gain on RPM state
#define LQI_KI              -3.1623f   // gain on integral-of-error state

// ---------------- Kalman Filter Gains (for LQG mode) ----------------
// From solve_continuous_are on the same illustrative model; recompute
// alongside the LQI gains above if the plant model changes.
#define KF_L1                 0.0516f
#define KF_L2                 0.0199f

// ---------------- Actuator Limits (SAFETY CRITICAL) ----------------
// Control law output u is mapped to injector pulse width in microseconds.
// These bounds MUST reflect your actual injector's safe operating range.
#define INJECTOR_PW_MIN_US    800.0f    // minimum safe pulse width (us)
#define INJECTOR_PW_MAX_US   6000.0f    // maximum safe pulse width (us)
#define INJECTOR_PW_NOMINAL_US 2500.0f  // pulse width corresponding to u=0

// Anti-windup clamp on the integral state (prevents runaway integral term
// during actuator saturation or sensor fault)
#define INTEGRAL_CLAMP        50.0f

// AFR plausibility bounds -- if sensor reads outside this range, treat as
// a fault and fall back to open-loop/safe mode rather than trusting the
// control law.
#define AFR_FAULT_MIN          9.0f
#define AFR_FAULT_MAX         20.0f

// RPM plausibility bounds
#define RPM_FAULT_MIN           0.0f
#define RPM_FAULT_MAX        12000.0f

// ---------------- Logging ----------------
#define SERIAL_BAUD          115200
#define LOG_INTERVAL_MS         50   // how often to print/log a data row

#endif // CONFIG_H
