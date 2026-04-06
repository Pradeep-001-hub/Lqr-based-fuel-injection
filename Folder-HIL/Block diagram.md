flowchart TD
    A[Engine Model (Simulink / Real-Time)] --> B[Sensor Signals (AFR & RPM)]
    B --> C[Controller (PID / LQR / LQG) on ECU/Microcontroller]
    C --> D[Actuator Commands (Fuel Injector)]
    D --> A
