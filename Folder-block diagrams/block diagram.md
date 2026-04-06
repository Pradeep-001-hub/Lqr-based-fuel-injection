flowchart TB
    %% Reference
    style REF fill:#f9f,stroke:#333,stroke-width:2px
    REF[AFR & RPM Reference r(t)\n(Setpoints)] 

    %% Error
    style SUM fill:#fff,stroke:#333,stroke-width:2px
    SUM[Summing Junction (+)\nError e(t) = r(t) - ŷ(t)] 

    %% Controller with Subblocks
    style CTRL fill:#bbf,stroke:#333,stroke-width:2px
    CTRL[Controller Block\n(Computes u(t))]

    style LQR fill:#8fc,stroke:#333,stroke-width:1.5px
    LQR[LQR Controller K\n(State Feedback)]

    style KF fill:#fc8,stroke:#333,stroke-width:1.5px
    KF[Kalman Filter / Observer\n(State Estimation x̂(t))]

    %% Actuator
    style ACT fill:#bfb,stroke:#333,stroke-width:2px
    ACT[Fuel Injector\n(Actuator Input u(t))]

    %% Engine with Subblocks
    style ENG fill:#ffb,stroke:#333,stroke-width:2px
    ENG[Engine Model / Plant]

    style AFR fill:#ffc,stroke:#333,stroke-width:1.5px
    AFR[AFR Dynamics]

    style RPM fill:#ffc,stroke:#333,stroke-width:1.5px
    RPM[RPM Dynamics]

    %% Sensors
    style SENS fill:#fbf,stroke:#333,stroke-width:2px
    SENS[Sensors\n(Measures AFR & RPM)]

    %% Feedback
    style FB fill:#fbb,stroke:#333,stroke-width:2px
    FB[Feedback to Summing Junction ŷ(t)]

    %% Simulation & Performance
    style SIM fill:#bbf,stroke:#333,stroke-width:2px
    SIM[Simulation Engine / ODE Solver]

    style OUT fill:#bfb,stroke:#333,stroke-width:2px
    OUT[Outputs: AFR, RPM, u(t)]

    style PERF fill:#fcf,stroke:#333,stroke-width:2px
    PERF[Performance Evaluation\n(Rise Time, Overshoot, Settling, RMSE)]

    %% Connections
    REF --> SUM
    SUM --> CTRL
    CTRL --> LQR
    CTRL --> KF
    LQR --> ACT
    ACT --> ENG
    ENG --> AFR
    ENG --> RPM
    AFR --> SENS
    RPM --> SENS
    SENS --> FB
    FB --> SUM
    ENG --> SIM
    SIM --> OUT
    OUT --> PERF