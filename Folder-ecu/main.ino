/*
 * ============================================================
 *  LQR / LQG-BASED FUEL INJECTION ECU FIRMWARE
 *  Target: ESP32 (Arduino framework)
 * ============================================================
 *
 * Implements the integral-augmented LQR (LQI) control law validated in
 * controller_comparison.py / mathematical model.md, ported to a real-time
 * embedded control loop:
 *
 *      u = -Kx1*(AFR - AFR_ref) - Kx2*(RPM - RPM_ref) - Ki*xi
 *      xi_dot = (AFR_ref - AFR)          [integral of tracking error]
 *
 * A software Kalman filter is included (LQG mode) so the control law can
 * run off a filtered state estimate rather than the raw noisy AFR sensor
 * reading -- toggle with USE_KALMAN_FILTER below.
 *
 * *** SAFETY NOTE ***
 * This firmware is written for an M.Tech simulation/prototype-validation
 * context. The LQR/Kalman gains in config.h come from an illustrative,
 * NOT physically-identified, 2-state model. Do NOT connect this to a real
 * engine's fuel injector without:
 *   1. Re-deriving A, B from real engine dynamics or system ID data.
 *   2. Recomputing gains and validating in simulation/HIL first.
 *   3. Bench-testing actuator limits and fault handling in isolation.
 *   4. Adding a hardware watchdog / kill switch independent of this MCU.
 * Running unvalidated control gains on a real fuel system can cause lean
 * misfire, engine knock, or component/engine damage.
 */

#include "config.h"

// ---------------- Global state ----------------
volatile unsigned long lastPulseMicros = 0;
volatile unsigned long pulsePeriodMicros = 0;
volatile bool newPulse = false;

float xi_integral = 0.0f;      // LQI integral-of-error state
float afr_setpoint = AFR_STOICH;
float rpm_setpoint = 0.0f;     // 0 => no explicit RPM regulation target (see note in loop())

// Kalman filter state estimate (xhat = [AFR_est, RPM_est])
float xhat_afr = AFR_STOICH;
float xhat_rpm = 0.0f;

bool faultActive = false;
unsigned long lastLoopMicros = 0;
unsigned long lastLogMillis = 0;

#define USE_KALMAN_FILTER   1   // 1 = LQG (filtered state feedback), 0 = LQR (raw sensor feedback)

// ---------------- ISR: RPM pulse capture ----------------
void IRAM_ATTR onCrankPulse() {
  unsigned long now = micros();
  if (lastPulseMicros != 0) {
    pulsePeriodMicros = now - lastPulseMicros;
    newPulse = true;
  }
  lastPulseMicros = now;
}

// ---------------- Sensor acquisition ----------------
float readAFR_raw() {
  int counts = analogRead(PIN_AFR_ANALOG);
  float voltage = (counts / ADC_MAX_COUNTS) * ADC_VREF;
  float lambda = LAMBDA_AT_VMIN +
                 (voltage / ADC_VREF) * (LAMBDA_AT_VMAX - LAMBDA_AT_VMIN);
  return lambda * AFR_STOICH;
}

float readRPM_raw() {
  noInterrupts();
  unsigned long period = pulsePeriodMicros;
  bool isNew = newPulse;
  newPulse = false;
  interrupts();

  // If no pulse seen recently (engine stopped/stalled), report 0 RPM
  if (period == 0 || (micros() - lastPulseMicros) > 500000UL) {
    return 0.0f;
  }
  float rev_per_sec = 1.0e6f / (period * PULSES_PER_REV);
  return rev_per_sec * 60.0f;
}

// ---------------- Fault checking ----------------
bool checkFault(float afr, float rpm) {
  if (afr < AFR_FAULT_MIN || afr > AFR_FAULT_MAX) return true;
  if (rpm < RPM_FAULT_MIN || rpm > RPM_FAULT_MAX) return true;
  if (isnan(afr) || isnan(rpm)) return true;
  return false;
}

// ---------------- Kalman filter update ----------------
// Continuous-time filter, discretized with a simple Euler step at the
// control loop rate. Uses the same A,B matrices as the design model.
void kalmanUpdate(float y_meas_afr, float u, float dt) {
  // Illustrative plant matrices (must match the ones used to compute
  // KF_L1/KF_L2 in config.h if you re-derive gains for a real engine).
  const float A11 = -0.4f, A12 = 0.1f;
  const float A21 = 0.05f, A22 = -0.2f;
  const float B1 = 0.3f, B2 = 0.1f;

  float y_hat = xhat_afr;
  float innovation = y_meas_afr - y_hat;

  float xhat_afr_dot = A11 * xhat_afr + A12 * xhat_rpm + B1 * u + KF_L1 * innovation;
  float xhat_rpm_dot = A21 * xhat_afr + A22 * xhat_rpm + B2 * u + KF_L2 * innovation;

  xhat_afr += xhat_afr_dot * dt;
  xhat_rpm += xhat_rpm_dot * dt;
}

// ---------------- LQI control law ----------------
float computeControl(float afr_state, float rpm_state, float dt) {
  float err = afr_setpoint - afr_state;

  // Integral state update with anti-windup clamp
  xi_integral += err * dt;
  if (xi_integral > INTEGRAL_CLAMP) xi_integral = INTEGRAL_CLAMP;
  if (xi_integral < -INTEGRAL_CLAMP) xi_integral = -INTEGRAL_CLAMP;

  float dAFR = afr_state - afr_setpoint;
  float dRPM = rpm_state - rpm_setpoint;

  float u = -(LQI_KX1 * dAFR) - (LQI_KX2 * dRPM) - (LQI_KI * xi_integral);
  return u;
}

// ---------------- Actuator output ----------------
void writeInjector(float u) {
  // Map control effort u onto an injector pulse width around the nominal
  // (u=0) operating point. This is a simplified academic-prototype mapping;
  // a production ECU would synchronize injector pulses to crank angle via
  // a dedicated hardware timer, not a free-running PWM duty cycle.
  float pw_us = INJECTOR_PW_NOMINAL_US + (u * 100.0f); // scale factor: tune per injector

  if (pw_us < INJECTOR_PW_MIN_US) pw_us = INJECTOR_PW_MIN_US;
  if (pw_us > INJECTOR_PW_MAX_US) pw_us = INJECTOR_PW_MAX_US;

  // Convert desired pulse width to a duty cycle for ledc PWM output
  // (period assumed = 1/CONTROL_LOOP_HZ for this simplified prototype).
  float period_us = 1.0e6f / CONTROL_LOOP_HZ;
  float duty_fraction = pw_us / period_us;
  if (duty_fraction > 1.0f) duty_fraction = 1.0f;

  ledcWrite(0, (uint32_t)(duty_fraction * 255));
}

void enterSafeMode() {
  faultActive = true;
  digitalWrite(PIN_STATUS_LED, HIGH);
  // Fall back to a fixed, conservative pulse width rather than trusting
  // the control law on faulty sensor data.
  float period_us = 1.0e6f / CONTROL_LOOP_HZ;
  float duty_fraction = INJECTOR_PW_NOMINAL_US / period_us;
  ledcWrite(0, (uint32_t)(duty_fraction * 255));
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(PIN_AFR_ANALOG, INPUT);
  pinMode(PIN_RPM_PULSE, INPUT_PULLUP);
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  attachInterrupt(digitalPinToInterrupt(PIN_RPM_PULSE), onCrankPulse, RISING);

  ledcSetup(0, CONTROL_LOOP_HZ, 8);   // channel 0, freq = loop rate, 8-bit resolution
  ledcAttachPin(PIN_INJECTOR_PWM, 0);

  xhat_afr = AFR_STOICH;
  xhat_rpm = 0.0f;
  xi_integral = 0.0f;

  Serial.println("time_ms,AFR_raw,RPM_raw,AFR_est,RPM_est,u,xi,fault");
  lastLoopMicros = micros();
}

// ---------------- Main loop ----------------
void loop() {
  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastLoopMicros) / 1.0e6f;

  if (dt < CONTROL_DT_S) {
    return;   // run the control law at a fixed rate (CONTROL_LOOP_HZ)
  }
  lastLoopMicros = nowMicros;

  float afr_raw = readAFR_raw();
  float rpm_raw = readRPM_raw();

  faultActive = checkFault(afr_raw, rpm_raw);

  if (faultActive) {
    enterSafeMode();
  } else {
    float afr_state, rpm_state;

#if USE_KALMAN_FILTER
    // Predict-correct using last commanded u (stored from previous cycle)
    static float u_prev = 0.0f;
    kalmanUpdate(afr_raw, u_prev, dt);
    afr_state = xhat_afr;
    rpm_state = xhat_rpm;
#else
    afr_state = afr_raw;
    rpm_state = rpm_raw;
#endif

    float u = computeControl(afr_state, rpm_state, dt);
    writeInjector(u);

#if USE_KALMAN_FILTER
    u_prev = u;
#endif

    digitalWrite(PIN_STATUS_LED, LOW);

    if (millis() - lastLogMillis >= LOG_INTERVAL_MS) {
      lastLogMillis = millis();
      Serial.print(millis());   Serial.print(",");
      Serial.print(afr_raw, 3); Serial.print(",");
      Serial.print(rpm_raw, 1); Serial.print(",");
      Serial.print(afr_state, 3); Serial.print(",");
      Serial.print(rpm_state, 1); Serial.print(",");
      Serial.print(u, 3); Serial.print(",");
      Serial.print(xi_integral, 3); Serial.print(",");
      Serial.println(faultActive ? 1 : 0);
    }
  }
}
