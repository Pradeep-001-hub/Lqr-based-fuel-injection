LQR/LQG-Based Fuel Injection Control System

Overview

This project presents an advanced control strategy for fuel injection systems using Linear Quadratic Regulator (LQR) and Linear Quadratic Gaussian (LQG) techniques.

The objective is to achieve optimal Air-Fuel Ratio (AFR) control, ensuring:

Improved fuel efficiency
Reduced emissions
Stable engine performance
The system is extended to include nonlinear engine dynamics, disturbance rejection, and parameter analysis, making it suitable for high-performance applications.

Objectives

Design an optimal fuel injection controller using LQR
Extend to LQG for state estimation under noise
Implement nonlinear AFR-based engine model
Achieve accurate AFR tracking (≈ 14.7)
Analyze system robustness under disturbances
Perform parameter sensitivity study (Q & R tuning)
System Model

Nonlinear Model

Air-Fuel Ratio(AFR):air mass ÷fuelmass

Nonlinear dynamics include:

Combustion effects
Fuel-air interaction
External disturbances
Control Strategy

LQR Controller

Minimizes cost function: [ J = \int (x^T Q x + u^T R u) , dt ]

Provides:

Optimal control
Reduced overshoot
Stable response
LQG Controller

Combines:

LQR (optimal control)
Kalman Filter (state estimation)
Advantages:

Handles noise
Improves robustness
Suitable for real-world systems
Performance Analysis

Time Domain

Rise Time
Settling Time
Overshoot
Peak Time
Frequency Domain

Bode Plot
Nyquist Plot
Root Locus
Nonlinear Analysis

AFR tracking
Disturbance rejection
Control effort smoothness
Results

LQR vs PID

LQR → smoother response, less overshoot
PID → faster but oscillatory
AFR Tracking

Successfully tracks stoichiometric AFR (14.7)
Maintains stability under disturbances
Control Input

Smooth fuel injection signal
Avoids excessive fuel usage
3D Parameter Analysis

A parametric study was performed by varying:

Q (state weighting)
R (control penalty)
Observations:

High Q → aggressive response, higher overshoot
High R → smoother control, slower response
Balanced Q & R → optimal performance
MATLAB Functions Used

Basic

"clear", "clc", "close all"

Control Design

"lqr()", "kalman()", "pid()"

Simulation

"step()", "ode45()"

Analysis

"stepinfo()", "bode()", "nyquist()", "rlocus()"

Visualization

"plot()", "surf()", "subplot()", "meshgrid()"

Key Insights

LQR provides optimal fuel control
LQG improves robustness under noise
Nonlinear modeling reflects real engine behavior
Parameter tuning is critical for performance
Applications

Automotive fuel injection systems
Aerospace propulsion control
Engine management systems
Advanced control system design
Conclusion:

This project demonstrates that LQR/LQG-based control significantly outperforms traditional PID controllers in fuel injection systems by providing:

Optimal performance
Improved stability
Better fuel efficiency
Robust operation under disturbances
Future Scope

Real-time implementation using embedded systems
Integration with Simulink models
Nonlinear LQG (Extended Kalman Filter)
Experimental validation on engine setup