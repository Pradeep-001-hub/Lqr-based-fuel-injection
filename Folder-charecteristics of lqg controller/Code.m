clear; clc; close all;


A = [0 1;
    -1 -1.5];
B = [0;1];
C = [1 0];

Q = [10 0; 0 1];
R = 0.5;
K = lqr(A,B,Q,R);

%% KALMAN FILTER
W = 0.2*eye(2);   % Process noise
V = 0.5;          % Measurement noise

L = lqe(A,eye(2),C,W,V);

%SIMULATION 
dt = 0.01;
t = 0:dt:10;

x = [3; 0];        % True state
x_hat = [0; 0];    % Estimated state

for i = 1:length(t)
    
    % Noise
    w = sqrt(0.2)*randn(2,1);
    v = sqrt(0.5)*randn;
    
    % Measurement
    y = C*x + v;
    
    % Control (uses estimated state)
    u = -K*x_hat;
    
    % True system
    x_dot = A*x + B*u + w;
    x = x + x_dot*dt;
    
    % Estimator
    xhat_dot = A*x_hat + B*u + L*(y - C*x_hat);
    x_hat = x_hat + xhat_dot*dt;
    
    % Store
    X(:,i) = x;
    Xhat(:,i) = x_hat;
    Y(i) = y;
    U(i) = u;
end
figure;

%  Noisy Measurement vs True State
subplot(4,1,1);
plot(t, X(1,:), 'b', 'LineWidth',1.5); hold on;
plot(t, Y, 'g.');
title('Noisy Measurement vs True State');
legend('True State','Measured (Noisy)');
grid on;

% Estimation Performance
subplot(4,1,2);
plot(t, X(1,:), 'b', t, Xhat(1,:), 'r--','LineWidth',1.5);
title('Kalman Filter: State Estimation');
legend('True State','Estimated State');
grid on;

%  Estimation Error
subplot(4,1,3);
plot(t, X(1,:) - Xhat(1,:), 'k','LineWidth',1.5);
title('Estimation Error (Converges to Zero)');
grid on;

% Control Effort
subplot(4,1,4);
plot(t, U, 'm','LineWidth',1.5);
title('Optimal Control Input (LQR)');
xlabel('Time (s)');
grid on;

sgtitle('LQG Controller: Key Characteristics Visualization');
