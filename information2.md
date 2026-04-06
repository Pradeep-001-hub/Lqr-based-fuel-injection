flowchart TD
    A[System: Engine + Fuel Injector] --> B[Classical PID Approach]
    A --> C[LQR / LQG Approach]

    B --> B1[Use Root Locus, Bode, Nyquist]
    B1 --> B2[Manual tuning of gains]
    B2 --> B3[Single-loop SISO only]

    C --> C1[State-space model: x' = Ax + Bu]
    C1 --> C2[Compute optimal gain K using Riccati equation]
    C2 --> C3[Handle multiple states (AFR & RPM) simultaneously]
    C3 --> C4[Automatically ensures stability & performance]