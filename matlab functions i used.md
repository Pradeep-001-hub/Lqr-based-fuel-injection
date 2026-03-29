MATLAB Functions – Detailed Description

Basic Commands:

"clear" Removes all variables from the workspace to ensure a fresh start.

"clc" Clears the command window for clean output display.

"close all" Closes all open figure windows.

System Modeling:

"ss(A,B,C,D)" Creates a state-space representation of the system.

"tf('s')" Defines the Laplace variable for transfer function modeling.

"feedback()" Forms a closed-loop system by connecting controller and plant.

Control Design:

"lqr(A,B,Q,R)" Computes the optimal state feedback gain by minimizing a quadratic cost function.

"kalman(sys,Qn,Rn)" Designs a Kalman filter for state estimation in noisy environments.

"pid(Kp,Ki,Kd)" Creates a PID controller with proportional, integral, and derivative gains.

Simulation:

"step(system,t)" Generates the time response of a system to a step input.

"ode45()" Solves nonlinear ordinary differential equations using a numerical method.

Performance Analysis:

"stepinfo(system)" Provides key performance metrics such as rise time, settling time, overshoot, and peak time.

"max()" Returns the maximum value of a dataset, used for overshoot calculation.

Matrix & Data Handling:

"diag()" Creates a diagonal matrix, typically used for defining Q matrix in LQR.

"eye()" Generates an identity matrix.

"zeros()" Initializes arrays with zeros.

"linspace()" Generates evenly spaced values over a specified interval.

"meshgrid()" Creates grid matrices for 3D surface plotting.

Visualization & Plotting:

"figure" Opens a new figure window.

"plot()" Generates 2D plots.

"surf()" Creates 3D surface plots.

"subplot()" Divides the figure into multiple sections for plotting.

"grid on" Enables grid lines for better readability.

"legend()" Adds a legend to the plot.

"title()" Adds a title to the plot.

"xlabel()", "ylabel()", "zlabel()" Labels the axes of plots.

"xline()", "yline()" Adds reference lines to plots.

"sgtitle()" Adds an overall title to a figure with multiple subplots.

Frequency Analysis:

"bode()" Plots the frequency response of the system.

"nyquist()" Used for stability analysis in frequency domain.

"rlocus()" Plots root locus to analyze system stability with varying gain.

Summary:

These MATLAB functions collectively enable:

Optimal controller design (LQR/LQG)
Nonlinear system simulation
Performance evaluation
Advanced visualization and analysis
They form the computational backbone of this fuel injection control system.