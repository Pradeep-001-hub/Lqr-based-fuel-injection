flowchart LR
    A[AFR & RPM Reference] --> B[ECU Controller]
    B --> C[Fuel Injector]
    C --> D[Engine Dynamics]
    D --> E[Sensors]
    E --> B




flowchart LR
    style A fill:#f9f,stroke:#333,stroke-width:2px
    style B fill:#bbf,stroke:#333,stroke-width:2px
    style C fill:#bfb,stroke:#333,stroke-width:2px
    style D fill:#ffb,stroke:#333,stroke-width:2px
    style E fill:#fbf,stroke:#333,stroke-width:2px

    A[AFR & RPM Reference] -->|Reference Signal| B[ECU Controller]
    B -->|Control Signal| C[Fuel Injector]
    C --> D[Engine Dynamics]
    D -->|AFR, RPM Feedback| E[Sensors]
    E -->|Measured Data| B