clc; clear; close all;
% ECU Simulation: PID vs LQR%

% System Definition (Simplified Engine Model)
% x = [Air-Fuel Ratio (AFR); Fuel Flow Rate]
A = [-0.5 1;
     -0.2 -0.3];   % Engine dynamics
B = [0.1; 0.2];     % Fuel control input
C = [1 0];          % Output = AFR
D = 0;

dt = 0.01;           % Time step (s)
T = 0:dt:10;         % Simulation time (s)
n = length(T);

% Reference AFR (Target)
ref = ones(1,n) * 1;  % AFR = 1

%% PID Controller Setup %
Kp = 2; Ki = 0.5; Kd = 0.1;

x_pid = [0;0];  % Initial state
y_pid = zeros(1,n);
u_pid = zeros(1,n);
e_prev = 0; integral = 0;

%LQR Controller Setup
Q = C'*C;   % State cost (emphasis on AFR)
R = 0.1;    % Control cost
K_lqr = lqr(A,B,Q,R);

x_lqr = [0;0];
y_lqr = zeros(1,n);
u_lqr = zeros(1,n);

%%  Simulation Loop 
for k = 1:n
    % --- PID ---
    y_pid(k) = C*x_pid;
    e = ref(k) - y_pid(k);
    integral = integral + e*dt;
    derivative = (e - e_prev)/dt;
    u_pid(k) = Kp*e + Ki*integral + Kd*derivative;
    
    % ECU input saturation
    u_pid(k) = max(0, min(u_pid(k), 10));
    
    % System update
    x_pid = x_pid + dt*(A*x_pid + B*u_pid(k));
    e_prev = e;
    
    % --- LQR ---
    y_lqr(k) = C*x_lqr;
    u_lqr(k) = -K_lqr * x_lqr;
    u_lqr(k) = max(0, min(u_lqr(k), 10));
    
    x_lqr = x_lqr + dt*(A*x_lqr + B*u_lqr(k));
end

%% ================= Plot Results =================
figure('Position',[100 100 900 600]);

% Air-Fuel Ratio Tracking
subplot(2,1,1);
plot(T, ref, 'k--','LineWidth',1.5); hold on;
plot(T, y_pid, 'b','LineWidth',1.5);
plot(T, y_lqr, 'r','LineWidth',1.5);
grid on; xlabel('Time (s)'); ylabel('Air-Fuel Ratio');
legend('Reference','PID','LQR','Location','best');
title('ECU Simulation: PID vs LQR Control');

% Control Inputs (Fuel Commands)
subplot(2,1,2);
plot(T, u_pid, 'b','LineWidth',1.5); hold on;
plot(T, u_lqr, 'r','LineWidth',1.5);
grid on; xlabel('Time (s)'); ylabel('Control Input (Fuel Rate)');
legend('PID Input','LQR Input','Location','best');
title('ECU Control Inputs');

subplot(2,1,1);
text(3,1.05,'LQR stabilizes faster','Color','r','FontSize',10);
annotation('arrow',[0.5 0.55],[0.6 0.7]);

subplot(2,1,2);
text(4,8,'PID overshoot','Color','b','FontSize',10);
annotation('arrow',[0.6 0.65],[0.35 0.4]);
