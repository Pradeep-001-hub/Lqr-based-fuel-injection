Physics-Based Modeling of AFR Control System

System Definition
The Air-Fuel Ratio (AFR) in an internal combustion engine is defined as:

AFR = (mass of air) / (mass of fuel)

Let:

m_a = mass of air inside intake manifold
m_f = mass of fuel inside cylinder
u = fuel injection rate (control input)
ṁ_a,in = air inflow rate
Air Dynamics (Manifold Filling Model)
From mass conservation:

dm_a/dt = ṁ_a,in - ṁ_a,out

Assuming:

Outflow proportional to manifold pressure (or mass)
ṁ_a,out = k_a * m_a

So,

dm_a/dt = ṁ_a,in - k_a * m_a

Fuel Dynamics
Fuel is injected directly:

dm_f/dt = u - k_f * m_f

Where:

u = fuel injection rate
k_f = fuel consumption constant
AFR Expression
AFR is defined as:

y = AFR = m_a / m_f

This is a nonlinear relationship.

Nonlinear State Model
Define states:

x₁ = m_a (air mass) x₂ = m_f (fuel mass)

State equations:

dx₁/dt = ṁ_a,in - k_a * x₁

dx₂/dt = u - k_f * x₂

Output equation:

y = x₁ / x₂

This is a nonlinear system because of division.

Linearization Around Operating Point
Let operating point:

x₁₀ = m_a0 x₂₀ = m_f0 y₀ = AFR₀ = 14.7

Using small-signal approximation:

Let:

x₁ = x₁₀ + Δx₁ x₂ = x₂₀ + Δx₂

Using Taylor expansion:

Δy ≈ (1 / x₂₀) Δx₁ − (x₁₀ / x₂₀²) Δx₂

Linear State-Space Model
State vector:

x = [Δx₁ Δx₂]

State equations:

d/dt [Δx₁] = -k_a Δx₁ d/dt [Δx₂] = -k_f Δx₂ + u

Matrix form:

ẋ = A x + B u

Where:

A = [-k_a 0 0 -k_f]

B = [0 1]

Output Equation
y = C x

Where:

C = [1/x₂₀ -x₁₀/x₂₀²]

D = 0

Final State-Space Representation
ẋ = A x + B u y = C x + D u

Key Insights
The system is inherently nonlinear due to AFR definition
Linearization is required for controller design
This model enables LQR and LQG implementation
Operating point (AFR = 14.7) ensures stable combustion
MATLAB Representation
A = [-k_a 0; 0 -k_f];

B = [0; 1];

C = [1/x20 -x10/(x20^2)];

D = 0;