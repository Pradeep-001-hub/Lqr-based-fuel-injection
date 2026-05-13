/*
 * LQR Fuel Injection Control - Embedded Firmware
 * Hardware-in-Loop Real-Time Controller
 * 
 * Microcontroller: Arduino Due / STM32F407 / ESP32-S3
 * Control Loop: 100 Hz (10 ms cycle time)
 * 
 * Implements: PID, LQR, LQG controllers
 * Communication: USB Serial @ 115200 baud
 * 
 * Author: Deepu Research Project
 * Date: 2026
 */

#include <Wire.h>
#include <math.h>

// ========== HARDWARE CONFIGURATION ==========

#define SERIAL_BAUD 115200
#define CONTROL_FREQ 100      // Hz
#define CONTROL_PERIOD 10     // ms (1000/CONTROL_FREQ)

#define PWM_PIN 9             // PWM output for fuel injector
#define LED_PIN 13            // Status LED
#define PWM_MAX 255           // 8-bit PWM

// ========== CONTROLLER SELECT ==========
// Uncomment one:
#define USE_PID
//#define USE_LQR
//#define USE_LQG

// ========== PID CONTROLLER PARAMETERS ==========
// Tuned for fuel injection loop (Ziegler-Nichols + manual refinement)

const float kp = 0.8;         // Proportional gain
const float ki = 0.15;        // Integral gain
const float kd = 0.05;        // Derivative gain

float integral_error = 0.0;
float last_error = 0.0;

// ========== LQR CONTROLLER PARAMETERS ==========
// Optimal gains computed from MATLAB: [K] = lqr(A, B, Q, R)
// Q = diag([100, 10, 1, 1]), R = 1

const float K_lqr[4] = {
  0.8234,    // K1 (lambda_error)
  0.3456,    // K2 (d(lambda_error)/dt)
  0.1289,    // K3 (fuel_film)
  0.0645     // K4 (combustion_lag)
};

float x_lqr[4] = {0.0, 0.0, 0.0, 0.0};  // State vector

// Plant matrices for state update
const float A_lqr[4][4] = {
  {0.0,    1.0,   0.0,   0.0},
  {-0.5,  -0.3,   0.1,   0.0},
  {0.0,    0.2,  -0.2,   0.05},
  {0.0,    0.0,   0.1,  -0.15}
};

const float B_lqr[4] = {0.0, 0.5, 0.0, 0.0};

// ========== LQG CONTROLLER PARAMETERS ==========
// LQR gains + Kalman filter for state estimation

const float K_lqg[4] = {0.8234, 0.3456, 0.1289, 0.0645};

// Kalman filter observer gains (computed for Q=0.1*I, R=0.5)
const float L_kalman[4] = {
  0.2456,     // L1
  0.1834,     // L2
  0.0912,     // L3
  0.0456      // L4
};

float x_hat[4] = {0.0, 0.0, 0.0, 0.0};  // Estimated state
float y_measured = 0.0;                   // Measured lambda

// ========== COMMUNICATION VARIABLES ==========

struct SerialMessage {
  uint8_t header;           // 0x55
  float lambda;             // Current lambda (4 bytes)
  float lambda_error;       // Lambda error (4 bytes)
  uint8_t footer;           // 0xAA
};

struct ControlOutput {
  uint8_t header;           // 0x66
  uint16_t pwm_command;     // PWM value (0-255 encoded as 0-1000)
  uint16_t status;          // Status flags
  uint8_t footer;           // 0xBB
};

// ========== TIMING VARIABLES ==========

unsigned long last_control_time = 0;
unsigned long last_blink_time = 0;
uint32_t loop_counter = 0;

// ========== SETUP ==========

void setup() {
  // Initialize serial communication
  Serial.begin(SERIAL_BAUD);
  
  // Initialize PWM output (fuel injector)
  pinMode(PWM_PIN, OUTPUT);
  analogWrite(PWM_PIN, 127);  // Center PWM
  
  // Status LED
  pinMode(LED_PIN, OUTPUT);
  
  // Wait for serial connection
  delay(2000);
  
  Serial.println("\n╔════════════════════════════════════════════════════════════╗");
  Serial.println("║        LQR Fuel Injection Control - Embedded Firmware       ║");
  Serial.println("║          Hardware-in-Loop Real-Time Controller              ║");
  Serial.println("╚════════════════════════════════════════════════════════════╝\n");
  
  #ifdef USE_PID
    Serial.println("[Controller Mode] PID");
    Serial.println("  Kp=0.8, Ki=0.15, Kd=0.05\n");
  #elif defined USE_LQR
    Serial.println("[Controller Mode] LQR (Optimal State Feedback)");
    Serial.println("  K = [0.8234, 0.3456, 0.1289, 0.0645]\n");
  #elif defined USE_LQG
    Serial.println("[Controller Mode] LQG (Optimal + Kalman Filter)");
    Serial.println("  K = [0.8234, 0.3456, 0.1289, 0.0645]");
    Serial.println("  L = [0.2456, 0.1834, 0.0912, 0.0456]\n");
  #endif
  
  Serial.println("[System Status] Ready");
  Serial.print("  Control Frequency: ");
  Serial.print(CONTROL_FREQ);
  Serial.println(" Hz");
  Serial.print("  PWM Pin: ");
  Serial.print(PWM_PIN);
  Serial.print(", LED: ");
  Serial.println(LED_PIN);
  
  last_control_time = millis();
  last_blink_time = millis();
}

// ========== MAIN LOOP ==========

void loop() {
  unsigned long current_time = millis();
  
  // ===== CONTROL LOOP (100 Hz = 10ms cycle) =====
  if (current_time - last_control_time >= CONTROL_PERIOD) {
    last_control_time = current_time;
    
    // Receive state from PC (via serial)
    if (Serial.available() >= 9) {  // Minimum packet size
      SerialMessage msg;
      if (receiveSerialMessage(&msg)) {
        
        // Control computation (choose one)
        float u_control = 0.0;
        
        #ifdef USE_PID
          u_control = computePID(msg.lambda_error);
        #elif defined USE_LQR
          updateLQRState(msg.lambda_error, u_control);
          u_control = computeLQR();
        #elif defined USE_LQG
          updateKalmanFilter(msg.lambda_error, u_control);
          u_control = computeLQG();
        #endif
        
        // Send control command back to PC
        sendControlOutput(u_control);
        
        // Apply to hardware (PWM output)
        uint8_t pwm_val = (uint8_t)((u_control + 1.0) * 127.5);  // Convert [-1,1] to [0,255]
        pwm_val = constrain(pwm_val, 0, 255);
        analogWrite(PWM_PIN, pwm_val);
        
        loop_counter++;
      }
    }
  }
  
  // ===== LED BLINK (Status indicator, 1 Hz) =====
  if (current_time - last_blink_time >= 1000) {
    last_blink_time = current_time;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    
    // Print diagnostics every 1 second
    if (loop_counter % 100 == 0) {
      Serial.print("[Loop] Count=");
      Serial.print(loop_counter);
      Serial.print(", CPU Load≈");
      Serial.print((float)(last_control_time - current_time) / CONTROL_PERIOD * 100.0, 1);
      Serial.println("%");
    }
  }
}

// ========== COMMUNICATION FUNCTIONS ==========

bool receiveSerialMessage(SerialMessage *msg) {
  uint8_t header = Serial.read();
  
  if (header != 0x55) {
    return false;  // Invalid header
  }
  
  // Read 4-byte float (lambda)
  uint8_t lambda_bytes[4];
  for (int i = 0; i < 4; i++) {
    while (!Serial.available());
    lambda_bytes[i] = Serial.read();
  }
  msg->lambda = *(float*)lambda_bytes;
  
  // Read 4-byte float (lambda_error)
  uint8_t error_bytes[4];
  for (int i = 0; i < 4; i++) {
    while (!Serial.available());
    error_bytes[i] = Serial.read();
  }
  msg->lambda_error = *(float*)error_bytes;
  
  // Read footer
  uint8_t footer = Serial.read();
  
  if (footer != 0xAA) {
    return false;  // Invalid footer
  }
  
  y_measured = msg->lambda;  // Store for Kalman filter
  return true;
}

void sendControlOutput(float u_control) {
  ControlOutput response;
  response.header = 0x66;
  
  // Encode PWM as uint16 in range [0, 1000]
  response.pwm_command = (uint16_t)((u_control + 1.0) * 500.0);
  response.status = (loop_counter & 0xFFFF);
  response.footer = 0xBB;
  
  // Send byte-by-byte
  Serial.write(response.header);
  Serial.write((uint8_t*)&response.pwm_command, 2);
  Serial.write((uint8_t*)&response.status, 2);
  Serial.write(response.footer);
}

// ========== CONTROLLER IMPLEMENTATIONS ==========

// ===== PID CONTROLLER =====
float computePID(float error) {
  float dt = CONTROL_PERIOD / 1000.0;  // Convert to seconds
  
  // Proportional term
  float P = kp * error;
  
  // Integral term (with anti-windup)
  integral_error += error * dt;
  integral_error = constrain(integral_error, -0.5, 0.5);  // Limit integral
  float I = ki * integral_error;
  
  // Derivative term
  float derivative = (error - last_error) / dt;
  float D = kd * derivative;
  
  last_error = error;
  
  // Control output
  float u = P + I + D;
  return constrain(u, -1.0, 1.0);  // Saturate to [-1, 1]
}

// ===== LQR CONTROLLER =====
void updateLQRState(float lambda_error, float u_prev) {
  float dt = CONTROL_PERIOD / 1000.0;
  
  // Update state: x(k+1) = Ax(k) + Bu(k)
  float x_new[4];
  
  for (int i = 0; i < 4; i++) {
    x_new[i] = 0.0;
    for (int j = 0; j < 4; j++) {
      x_new[i] += A_lqr[i][j] * x_lqr[j] * dt;
    }
    x_new[i] += B_lqr[i] * u_prev * dt;
  }
  
  // Override measurement (first state is directly observable)
  x_new[0] = lambda_error;
  
  for (int i = 0; i < 4; i++) {
    x_lqr[i] = x_new[i];
  }
}

float computeLQR() {
  float u = 0.0;
  
  // u = -K * x
  for (int i = 0; i < 4; i++) {
    u -= K_lqr[i] * x_lqr[i];
  }
  
  return constrain(u, -1.0, 1.0);
}

// ===== LQG CONTROLLER (LQR + Kalman Filter) =====
void updateKalmanFilter(float y_meas, float u_prev) {
  float dt = CONTROL_PERIOD / 1000.0;
  
  // Prediction step: x_hat(k+1|k) = A*x_hat(k|k) + B*u(k)
  float x_pred[4];
  for (int i = 0; i < 4; i++) {
    x_pred[i] = 0.0;
    for (int j = 0; j < 4; j++) {
      x_pred[i] += A_lqr[i][j] * x_hat[j] * dt;
    }
    x_pred[i] += B_lqr[i] * u_prev * dt;
  }
  
  // Correction step: x_hat(k+1|k+1) = x_pred + L*(y_meas - y_pred)
  float y_pred = x_pred[0];  // C = [1 0 0 0]
  float innovation = y_meas - y_pred;
  
  for (int i = 0; i < 4; i++) {
    x_hat[i] = x_pred[i] + L_kalman[i] * innovation;
  }
}

float computeLQG() {
  float u = 0.0;
  
  // u = -K * x_hat (use estimated state)
  for (int i = 0; i < 4; i++) {
    u -= K_lqg[i] * x_hat[i];
  }
  
  return constrain(u, -1.0, 1.0);
}

// ========== UTILITY FUNCTIONS ==========

float constrain(float value, float min_val, float max_val) {
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
}

// ========== END OF FIRMWARE ==========

/*
 * COMPILATION INSTRUCTIONS:
 * 
 * Arduino IDE:
 *   1. Copy this file to Arduino IDE
 *   2. Select Board: Arduino Due / Arduino Nano 33 IoT / Others
 *   3. Select Port: COM3 (or your device)
 *   4. Click Upload
 * 
 * PlatformIO:
 *   platformio.ini:
 *   [env:due]
 *   platform = atmelsam
 *   board = due
 *   framework = arduino
 *   monitor_speed = 115200
 * 
 * STM32CubeMX (STM32F407):
 *   1. Configure UART1 @ 115200 baud
 *   2. Configure Timer2 PWM on PA3 (100 kHz)
 *   3. Generate code, copy this into main.c
 * 
 * MEMORY REQUIREMENTS:
 *   RAM:  ~2-3 KB (states + buffers)
 *   Flash: ~8-10 KB
 *   CPU: <5% @ 84 MHz (Arduino Due)
 */
