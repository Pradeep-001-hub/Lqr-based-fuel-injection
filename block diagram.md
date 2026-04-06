## Fuel Injection Control System – Project Flow

+--------------------------+
|  AFR & RPM Reference     |
|       (Setpoints)        |
+--------------------------+
            |
            v
+--------------------------+
|      Controller          |
|  (PID / LQR / LQG)      |
+--------------------------+
            |
            v
+--------------------------+
|     Actuator Input       |
|    (Fuel Injector)       |
+--------------------------+
            |
            v
+--------------------------+
|      Engine Model        |
|   (AFR & RPM Output)     |
+--------------------------+
            |
            v
+--------------------------+
|     Sensors / Feedback   |
+--------------------------+
            |
            v
+--------------------------+
|      Controller          |
|  (Feedback Loop Back)    |
+--------------------------+