## Fuel Injection Control System – Project Flow

```text
          +------------------+
          |  AFR & RPM Ref   |
          |  (Setpoints)     |
          +--------+---------+
                   |
                   v
          +--------+---------+
          |   Controller     |
          | (PID / LQR / LQG)|
          +--------+---------+
                   |
                   v
          +--------+---------+
          |  Actuator Input  |
          | (Fuel Injector)  |
          +--------+---------+
                   |
                   v
          +--------+---------+
          |    Engine Model  |
          | (AFR & RPM Output)|
          +--------+---------+
                   |
                   v
          +--------+---------+
          |   Sensors /      |
          |  Feedback Loop   |
          +--------+---------+
                   |
                   +-----------------+
                   |                 |
                   v                 ^
          +------------------+      |
          | Measured AFR &   |------+
          | RPM Feedback     |
          +------------------+
