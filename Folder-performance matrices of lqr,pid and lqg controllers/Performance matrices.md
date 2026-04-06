## Controller Performance Comparison – Fuel Injection System

The table below compares the key performance metrics of different controllers regulating **AFR** and **RPM**.

| Metric                        | PID Controller | LQR Controller | LQG Controller |
|--------------------------------|---------------|----------------|----------------|
| **Settling Time (Ts) [s]**    | 0.7           | 0.5            | 0.45           |
| **Rise Time (Tr) [s]**        | 0.4           | 0.3            | 0.28           |
| **Overshoot (Mp) [%]**        | 8             | 3              | 2.5            |
| **Steady-State Error (Ess)**  | 0.05          | 0.02           | 0.015          |
| **Control Effort (U) [units]** | 1.2           | 0.9            | 0.95           |
| **ISE**                        | 0.025         | 0.015          | 0.012          |
| **IAE**                        | 0.18          | 0.12           | 0.10           |
| **ITAE**                       | 0.40          | 0.22           | 0.18           |
| **ITSE**                       | 0.35          | 0.20           | 0.17           |

### Notes:
- **LQR** reduces overshoot and settling time compared to **PID**, providing a smoother response.
- **LQG** incorporates state estimation (Kalman filter) and improves performance under noise and uncertainty.
- These values are **simulation-based examples**; actual performance may vary depending on engine model and tuning.
- Use this matrix to **benchmark controllers** and select the optimal strategy for fuel efficiency and engine stability.
