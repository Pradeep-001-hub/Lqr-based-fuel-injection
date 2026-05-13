/*
 * LQR/LQG Fuel Injection ECU - Hardware-in-the-Loop (HIL)
 * Arduino Compatible (Best on Arduino Due)
 * Control Rate: 100 Hz
 */

#include <math.h>

#define SERIAL_BAUD 115200
#define CONTROL_PERIOD 10     // 100 Hz
#define PWM_PIN 9
#define LED_PIN 13

// === Select Controller (uncomment only one) ===
#define USE_LQG     // Recommended for real ECU
// #define USE_LQR
// #define USE_PID

// ========== LQR / LQG Parameters (from your model) ==========
const float K[4] = {0.8234, 0.3456, 0.1289, 0.0645};   // LQR Gain

// Plant matrices for internal state propagation
const float A[4][4] = {
  {0.0,  1.0,  0.0,  0.0},
  {-0.5, -0.3, 0.1,  0.0},
  {0.0,  0.2, -0.2,  0.05},
  {0.0,  0.0, 0.1, -0.15}
};

const float B[4] = {0.0, 0.5, 0.0, 0.0};

// Kalman Observer Gain for LQG
const float L[4] = {0.2456, 0.1834, 0.0912, 0.0456};

float x_hat[4] = {0};   // Estimated states (LQG)
float x_lqr[4] = {0};   // States for pure LQR

float last_u = 0.0;
unsigned long last_time = 0;
uint32_t loop_count = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  analogWrite(PWM_PIN, 128);

  delay(2000);
  Serial.println("=== LQR/LQG Fuel Injection ECU - HIL Ready ===");
  #ifdef USE_LQG
    Serial.println("Mode: LQG (LQR + Kalman Filter)");
  #elif defined USE_LQR
    Serial.println("Mode: LQR");
  #else
    Serial.println("Mode: PID");
  #endif
  last_time = millis();
}

void loop() {
  unsigned long now = millis();
  
  if (now - last_time >= CONTROL_PERIOD) {
    last_time = now;

    // Read from MATLAB Host
    if (Serial.available() >= 9) {
      uint8_t header = Serial.read();
      if (header == 0x55) {
        float lambda_meas = readFloat();
        float lambda_error = readFloat();
        uint8_t footer = Serial.read();

        if (footer == 0xAA) {
          float u = 0.0;

          #ifdef USE_LQG
            u = computeLQG(lambda_meas, lambda_error, last_u);
          #elif defined USE_LQR
            u = computeLQR(lambda_error, last_u);
          #else
            u = computePID(lambda_error);
          #endif

          last_u = u;
          sendControl(u);

          // Apply to injector (PWM)
          int pwm = constrain((int)((u + 1.0) * 127.5), 0, 255);
          analogWrite(PWM_PIN, pwm);

          loop_count++;
          digitalWrite(LED_PIN, loop_count % 2);
        }
      }
    }
  }
}

// Helper functions
float readFloat() {
  union { float f; uint8_t bytes[4]; } data;
  for (int i = 0; i < 4; i++) {
    while (!Serial.available()) {}
    data.bytes[i] = Serial.read();
  }
  return data.f;
}

void sendControl(float u) {
  uint8_t header = 0x66;
  uint16_t pwm_cmd = (uint16_t)((u + 1.0) * 500);
  uint8_t footer = 0xBB;
  Serial.write(header);
  Serial.write((uint8_t*)&pwm_cmd, 2);
  Serial.write((uint16_t)loop_count & 0xFFFF);
  Serial.write(footer);
}

// Controllers
float computePID(float e) {
  static float integral = 0, prev_e = 0;
  float dt = CONTROL_PERIOD / 1000.0;
  integral = constrain(integral + e * dt, -0.5, 0.5);
  float deriv = (e - prev_e) / dt;
  prev_e = e;
  return constrain(0.8*e + 0.15*integral + 0.05*deriv, -1.0, 1.0);
}

float computeLQR(float lambda_error, float u_prev) {
  float dt = CONTROL_PERIOD / 1000.0;
  float x_new[4] = {0};

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) x_new[i] += A[i][j] * x_lqr[j] * dt;
    x_new[i] += B[i] * u_prev * dt;
  }
  x_new[0] = lambda_error;   // Measurement override

  memcpy(x_lqr, x_new, sizeof(x_new));

  float u = 0;
  for (int i = 0; i < 4; i++) u -= K[i] * x_lqr[i];
  return constrain(u, -1.0, 1.0);
}

float computeLQG(float y_meas, float lambda_error, float u_prev) {
  float dt = CONTROL_PERIOD / 1000.0;
  float x_pred[4] = {0};

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) x_pred[i] += A[i][j] * x_hat[j] * dt;
    x_pred[i] += B[i] * u_prev * dt;
  }

  float innovation = y_meas - x_pred[0];
  for (int i = 0; i < 4; i++) {
    x_hat[i] = x_pred[i] + L[i] * innovation;
  }

  float u = 0;
  for (int i = 0; i < 4; i++) u -= K[i] * x_hat[i];
  return constrain(u, -1.0, 1.0);
}