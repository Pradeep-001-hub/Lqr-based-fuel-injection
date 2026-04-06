1. System Description

The fuel injection system is modeled as a linear time-invariant (LTI) system.
The objective is to control the Air-Fuel Ratio (AFR) using fuel injection input.

------------------------------------------------------------

2. State Variables

x1(t) = Air-Fuel Ratio (AFR)
x2(t) = Rate of change of AFR

State vector:

x(t) = [x1(t);
        x2(t)]

Control input:

u(t) = Fuel injection control input

Output:

y(t) = AFR

------------------------------------------------------------

3. State-Space Model

A = [-0.4   0.1;
      0.05 -0.2]

B = [0.3;
     0.1]

C = [1 0]
D = 0

System equations:

dx/dt = A*x + B*u

y = C*x

Expanded form:

dx1/dt = -0.4*x1 + 0.1*x2 + 0.3*u
dx2/dt =  0.05*x1 - 0.2*x2 + 0.1*u

------------------------------------------------------------

4. LQR Performance Index

Q = [20  0;
      0 10]

R = 0.5

Cost function:

J = ∫(x'Qx + u'Ru) dt   from 0 to ∞

------------------------------------------------------------

5. Optimal Control Law

K = lqr(A, B, Q, R)

u(t) = -K*x(t)

------------------------------------------------------------

6. Closed-Loop System

dx/dt = (A - B*K)*x

------------------------------------------------------------

7. Reference Tracking (Stoichiometric AFR)

AFR_ref = 14.7

x_ref = [14.7;
          0]

Control law for tracking:

u(t) = -K*(x - x_ref)

------------------------------------------------------------

8. Model with Disturbance (Realistic Extension)

dx/dt = A*x + B*u + w

where w = disturbance vector

------------------------------------------------------------

9. Final Compact Representation

dx/dt = [-0.4   0.1;
          0.05 -0.2] x  +  [0.3;
                               0.1] u

y = [1 0] x

u = -Kx

------------------------------------------------------------

10. Assumption

The real fuel injection system is nonlinear.
This model represents a linearized approximation suitable for control design.