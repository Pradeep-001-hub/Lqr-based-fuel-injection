GRAPH DESCRIPTION: TRUE VS ESTIMATED AFR AND RPM

The graphs represent the performance of a Kalman Filter-based state estimation applied to a dynamic system involving Air-Fuel Ratio (AFR) and Engine Speed (RPM).

In each plot, the true system states are shown along with the estimated states obtained from the Kalman Filter. The true signals contain fluctuations due to process and measurement noise, while the estimated signals are smoother and closely follow the actual values.

The results show that even though the system starts with an initial estimation error, the Kalman Filter quickly converges and accurately tracks the true states over time. This demonstrates the effectiveness of the filter in handling noisy environments and producing reliable estimates.

KALMAN FILTER ROLE IN THE SYSTEM

In practical systems such as fuel injection or thermal control, not all states can be measured directly, and sensor readings are often affected by noise and uncertainty. The Kalman Filter addresses this problem by acting as an optimal state estimator.

It works by combining predictions from the system model with actual noisy measurements to produce the best possible estimate of the system states. At each time step, it performs two main operations: prediction and correction. In the prediction step, it estimates the next state using the system dynamics. In the correction step, it updates this estimate using the measured output and a calculated gain known as the Kalman Gain.

The Kalman Filter intelligently balances between trusting the model and trusting the measurements based on the noise characteristics defined by process noise covariance (Q) and measurement noise covariance (R).

IMPORTANCE IN THE PROJECT

The Kalman Filter improves system performance in several important ways. It reduces noise in AFR and RPM signals, providing smoother and more accurate data. It allows estimation of internal states that may not be directly measurable. It enhances controller performance, especially when used with optimal controllers like LQR, forming an LQG system. Most importantly, it makes the system robust and suitable for real-world applications where disturbances and uncertainties are unavoidable.

FINAL STATEMENT

The Kalman Filter significantly enhances system reliability and accuracy by providing optimal state estimation, making it an essential component in modern control systems such as automotive engines, aerospace systems, and advanced thermal management.