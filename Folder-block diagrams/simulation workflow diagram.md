```text+------------------------+
| Initial Conditions &   |
| Inputs                 |
+-----------+------------+
            |
            v
+-----------+------------+
| Simulation Engine /    |
| ODE Solver             |
+-----------+------------+
            |
            v
+-----------+------------+
| Controller Block       |
| (PID / LQR / LQG)      |
+-----------+------------+
            |
            v
+-----------+------------+
| Engine Model           |
| (AFR & RPM Output)     |
+-----------+------------+
            |
            v
+-----------+------------+
| Outputs: AFR, RPM,     |
| Control Signal         |
+-----------+------------+
            |
            v
+-----------+------------+
| Performance Evaluation |
| (Rise Time, Overshoot, |
| Settling Time, RMSE)   |
+------------------------+