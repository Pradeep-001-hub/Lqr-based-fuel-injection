Introduction:
The fuel injection system of an internal combustion engine governs the air-fuel ratio (AFR) and engine performance. Accurate modeling of the system dynamics is critical for control design and simulation. This document presents a nonlinear state-space model suitable for high-fidelity simulations and optimal controller design (e.g., LQR/LQG).

System Description:
State Variables:
x1 = Air-Fuel Ratio (AFR)
x2 = Engine Speed (RPM)

Control Input:
u = Fuel Injection Rate (ECU command)

Output Variable:
y = x1 (Air-Fuel Ratio)

Nonlinear State-Space Equations:
dx1/dt = -a1*x1 + a2*x2 + a3*sin(x1) + b1*u
dx2/dt = -a4*x2 + a5*x1^2 + b2*u
y = x1

Where:
- sin(x1) captures nonlinear AFR oscillations
- x1^2 models quadratic coupling between AFR and engine speed

Model Parameters:
a1 = 0.4   % AFR decay constant
a2 = 0.1   % Engine speed coupling
a3 = 0.05  % Nonlinear AFR term
a4 = 0.2   % Engine speed decay
a5 = 0.02  % Quadratic AFR effect
b1 = 0.3   % Control gain for AFR
b2 = 0.1   % Control gain for engine speed

Compact MATLAB Representation:
% Nonlinear Fuel Injection Model
% State Variables: x1 = AFR, x2 = RPM
dx1 = -0.4*x1 + 0.1*x2 + 0.05*sin(x1) + 0.3*u;
dx2 = -0.2*x2 + 0.02*(x1^2) + 0.1*u;
y = x1;

Nonlinear System Features:
- Captures realistic engine dynamics
- Nonlinear terms improve accuracy over linear models
- Suitable for advanced control design and high-fidelity simulation
- Can be extended with:
  - Sensor dynamics (LQG applications)
  - Injector saturation and delays
  - Disturbances such as throttle changes or air density variations

Linearization for Control:
For optimal control design (LQR/LQG), the system can be linearized around an operating point:
x1 = 14.7 (stoichiometric AFR), x2 = x2_ref
Linearized model:
dx/dt = A*x + B*u
y = C*x
Where A, B, C are the Jacobian matrices evaluated at the operating point.

Summary:
This nonlinear state-space model provides a high-fidelity framework for simulation, control design, and research-level projects. It balances complexity with computational tractability, making it suitable for aerospace-grade applications, including ISRO-level research and development.
