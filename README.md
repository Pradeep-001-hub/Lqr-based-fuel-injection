# Lqr-based-fuel-injection
Simple idea to over come pid issues
uel Injection Control using LQR



introduction:

This project focuses on controlling fuel injection in an engine using modern control techniques.
The aim is to maintain a stable Air-Fuel Ratio (AFR) and engine speed (RPM) for better performance and efficiency.

---

Theory:

PID Controller:

PID (Proportional-Integral-Derivative) controller is a classical control method that works based on error correction.
It adjusts the system using present, past, and predicted errors.

LQR Controller:

LQR (Linear Quadratic Regulator) is an optimal control technique.
It calculates the best control input by minimizing a cost function, ensuring both accuracy and efficiency.
Description of the System:

The engine is modeled using a state-space approach with two main variables:

- Air-Fuel Ratio (AFR)
- Engine Speed (RPM)

The system uses:

- LQR for control
- Kalman Filter for estimating system states

Together, they form an LQG (Linear Quadratic Gaussian) control system.
 Tools Used:

- MATLAB
- Simulink
- Kalman Filter (for state estimation)

---
                
1.detailed Project Overview:
This project implements advanced control strategies for regulating the Air-Fuel Ratio (AFR) in an engine’s fuel injection system.
Goal:
Maintain the stoichiometric AFR (~14.7) under varying engine conditions using optimal control strategies.
Control Methods Implemented:
PID (Proportional-Integral-Derivative) – conventional method in ECUs
LQR (Linear Quadratic Regulator) – optimal state-feedback control
LQG (Linear Quadratic Gaussian) – LQR with state estimation via Kalman filter
Applications:
Automotive ECU fuel injection loops
Aerospace engine simulations
Academic and research demonstrations
_____
2. Problem Statement:
Maintaining AFR is critical for:
Engine efficiency
Reducing emissions
Avoiding engine knock
Challenges with conventional PID:
Tuning gains is manual and sensitive
Overshoot or slow response under varying loads
Cannot optimally balance tracking error vs control effort
Solution:
Use LQR and LQG, which provide optimal control with minimal cost, ensuring fast and smooth AFR regulation.
____
3. System Modeling
3.1 State Variables
3.2 State-Space Representation
Expanded:
{dx_2}/{dt} = 0.05 x_1 - 0.2 x_2 + 0.1
 u 
____
4. PID Control Loop:
PID Control Law:
Characteristics:
Simple and widely used in ECU loops
Sensitive to tuning; may oscillate under disturbances
No explicit cost minimization
Limitations Observed in Project:
Metric
PID Performance
Overshoot
Medium
Settling Time
High
Control Effort
High
Disturbance Rejection
Moderate
----
5. LQR Control Loop:
LQR Cost Function:
Design Steps:
Choose weighting matrices qand rbased on performance priorities
Compute optimal gain:
matlab syntax

K = lqr(A, B, Q, R);
u = -K * (x - x_ref);
Apply control input to fuel injection loop
------
Advantages over PID:
Optimally balances tracking error vs control effort
Faster convergence to AFR reference
Reduced overshoot and smoother control
LQR Performance Metrics (from simulation):
Metric
LQR Performance
Overshoot
Low
Settling Time
Short
Control Effort
Moderate
Disturbance Rejection
High
6. LQG Control Loop
LQG = LQR + Kalman Filter
Real systems may not measure all states 
Kalman filter estimates unmeasured states using noisy AFR measurements
LQR uses estimated states for control
Mathematical Representation:
u = -K \hat{x} 
Advantages over LQR:
Handles sensor noise effectively
Robust under disturbances and measurement uncertainty
LQG Performance Metrics (from simulation):
Metric
LQG Performance
Overshoot
Very Low
Settling Time
Shortest
Control Effort
Moderate
Disturbance Rejection
Very High
7. LQR vs LQG vs PID (Project Comparison)
Metric
PID
LQR
LQG
Overshoot
Medium
Low
Very Low
Settling Time
High
Short
Shortest
Control Effort
High
Moderate
Moderate
Noise Rejection
Moderate
Moderate
Very High
Ease of Implementation
Easy
Moderate
Moderate-High
Optimality
No
Yes
Yes
Realistic for ECUs
Yes
Yes
Yes
Key Takeaways:
LQR reduces overshoot and settling time compared to PID
LQG handles sensor noise and unmeasured states
Both LQR and LQG optimize control cost, unlike PID
8.2 How to Run
Matlab:

% Step 1: Open MATLAB
% Step 2: Navigate to repo folder
% Step 3: Run compare_results.m
% Step 4: Observe plots for AFR tracking and control inputs
9. Simulation Results
Expected Outcomes:
PID: Oscillatory AFR, moderate settling
LQR: Smooth AFR tracking, reduced overshoot
LQG: Best tracking with minimal oscillations and disturbance rejection
(Insert MATLAB plots: AFR vs time, control effort, PID vs LQR vs LQG comparison)
10. Assumptions
Linearized system around operating point
AFR dynamics approximated as LTI
Disturbances represented as additive vectors
Sensor noise modeled for LQG simulation

- Real-time implementation using microcontrollers or ECU systems
- Adaptive LQR for changing engine conditions
- Integration with IoT for smart monitoring
- Nonlinear engine modeling for higher accuracy
- Application in electric and hybrid vehicles
- Use in aerospace and advanced control systems
