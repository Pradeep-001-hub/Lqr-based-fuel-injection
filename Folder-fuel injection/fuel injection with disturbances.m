clear; clc; close all;

%% System Model (Engine AFR dynamics)
A = [0 1; -0.5 -1];
B = [0;1];
C = [1 0];
D = 0;

%% LQR Design
Q = diag([100 10]);
R = 1;
K = lqr(A,B,Q,R);

%% Simulation settings
tspan = [0 10];
dt = 0.01;
t = tspan(1):dt:tspan(2);

x = [10; 0];                 % Initial AFR deviation
AFR_ref = 14.7;              % Desired AFR
u_max = 5;                   % Injector saturation

x_log = [];
y_log = [];
u_log = [];

%% Simulation loop (more realistic than step())
for i = 1:length(t)

    % Output (AFR)
    y = C*x;

    % Add sensor noise
    noise = 0.05*randn;
    y_measured = y + noise;

    % Reference tracking (error-based state)
    e = [y_measured - AFR_ref; x(2)];

    % LQR control law
    u = -K*e;

    % Saturation (injector limits)
    u = max(-u_max, min(u, u_max));

    % Disturbance (engine load change at t = 5s)
    if t(i) > 5
        d = 0.5;
    else
        d = 0;
    end

    % State update (Euler integration)
    x_dot = A*x + B*u + [0; d];
    x = x + x_dot*dt;

    % Store data
    x_log = [x_log x];
    y_log = [y_log y];
    u_log = [u_log u];
end

x_log = x_log';
y_log = y_log';

%% Plotting

figure

subplot(3,1,1)
plot(t, y_log,'b','LineWidth',2)
hold on
yline(AFR_ref,'r--','Reference (14.7)')
grid on
title('AFR Tracking (Realistic)')
xlabel('Time (seconds)')
ylabel('AFR')

subplot(3,1,2)
plot(t, x_log(:,1),'LineWidth',2)
hold on
plot(t, x_log(:,2),'LineWidth',2)
grid on
title('State Variables')
xlabel('Time (seconds)')
ylabel('States')
legend('x1 (AFR)','x2 (Rate)')

subplot(3,1,3)
plot(t, u_log,'LineWidth',2)
grid on
title('Control Effort with Saturation')
xlabel('Time (seconds)')
ylabel('Fuel Input')

sgtitle('Realistic LQR-Based Fuel Injection System')