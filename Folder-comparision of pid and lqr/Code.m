clc; clear; close all;

% SYSTEM 
% States:
% x1 = Engine Speed (RPM)
% x2 = Air-Fuel Ratio (AFR)

A = [-0.4  0.1;
      0.05 -0.2];

B = [0.3;
     0.1];

C = [1 0];   % Output = Engine Speed

%LQR CONTROLLEr
Q = diag([20 10]);   % State weighting
R = 0.5;             % Control weighting

K = lqr(A, B, Q, R);

%% PID CONTROLLER 
Kp = 0.8;
Ki = 0.2;
Kd = 0.05;

%%SIMULATION SETUP
dt = 0.01;
t = 0:dt:10;

x0 = [50; 16];   % Initial conditions: [RPM, AFR]

% Storage
x_lqr = zeros(2, length(t));
x_pid = zeros(2, length(t));

u_lqr = zeros(1, length(t));
u_pid = zeros(1, length(t));

x_lqr(:,1) = x0;
x_pid(:,1) = x0;

% PID variables
e_prev = 0;
integral = 0;

ref = 0;   % Desired RPM

%SIMULATION LOOP 
for i = 1:length(t)-1
    
    %% -LQR CONTROL 
    u_lqr(i) = -K * x_lqr(:,i);
    
    dx_lqr = A * x_lqr(:,i) + B * u_lqr(i);
    x_lqr(:,i+1) = x_lqr(:,i) + dx_lqr * dt;
    
    
    %%  PID CONTROL 
    error = ref - x_pid(1,i);
    integral = integral + error * dt;
    derivative = (error - e_prev)/dt;
    
    u_pid(i) = Kp*error + Ki*integral + Kd*derivative;
    
    dx_pid = A * x_pid(:,i) + B * u_pid(i);
    x_pid(:,i+1) = x_pid(:,i) + dx_pid * dt;
    
    e_prev = error;
end

u_lqr(end) = u_lqr(end-1);
u_pid(end) 

% Engine Speed (LQR only) 
figure;
plot(t, x_lqr(1,:), 'LineWidth', 2);
title('Engine Speed Response using LQR');
xlabel('Time (s)');
ylabel('Engine Speed (RPM)');
grid on;

% RPM Comparison
figure;
plot(t, x_lqr(1,:), 'b', 'LineWidth', 2); hold on;
plot(t, x_pid(1,:), '--', 'LineWidth', 2);
legend('LQR','PID');
title('RPM Comparison: LQR vs PID');
xlabel('Time (s)');
ylabel('Engine Speed (RPM)');
grid on;

%  Control Effort
figure;
plot(t, u_lqr, 'LineWidth', 2); hold on;
plot(t, u_pid, '--', 'LineWidth', 2);
legend('LQR','PID');
title('Control Effort: LQR vs PID');
xlabel('Time (s)');
ylabel('Control Input');
grid on;

%  AFR Comparison
figure;
plot(t, x_lqr(2,:), 'LineWidth', 2); hold on;
plot(t, x_pid(2,:), '--', 'LineWidth', 2);
legend('LQR','PID');
title('AFR Comparison: LQR vs PID');
xlabel('Time (s)');
ylabel('AFR');
grid on;
