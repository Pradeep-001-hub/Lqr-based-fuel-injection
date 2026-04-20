Objective of Results

The objective of this section is to evaluate the performance of the proposed LQG-based controller for fuel injection and compare it with a conventional PID controller in terms of stability, response time, and robustness to noise.
-----‐
Key Observations:

pid controller:
The PID controller provides acceptable performance but exhibits higher overshoot and longer settling time. Additionally, the controller is sensitive to noise, which leads to fluctuations in the output response.
lqg controller:
The LQG-controlled system shows a fast and smooth response with minimal oscillations. The settling time is significantly reduced, and the system remains stable even in the presence of noise due to the Kalman filter-based state estimation.

ovarall observations:

- LQG provides optimal control by considering system dynamics and noise.
- PID reacts only to error and does not utilize state information.
- The inclusion of the Kalman filter significantly improves robustness.
- LQG is more suitable for complex systems like fuel injection control in ECUs
-------------
 comparision table:
```text
Metric| PID Controller| LQG Controller
Settling Time| Higher| Lower
Overshoot| Moderate| Minimal
Noise Handling| Poor| Excellent
Stability| Moderate| High



