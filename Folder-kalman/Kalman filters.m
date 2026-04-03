clear; clc; close all;

A = [-0.4 0.1; 0.05 -0.2];
B = [0.3; 0.1];
C = eye(2);

Q = 0.01*eye(2);
R = 0.05*eye(2);

dt = 0.01;
t = 0:dt:10;

x = [16; 50];
xhat = [14; 45];
P = eye(2);

u = 0;

N = length(t);
x_hist = zeros(2,N);
xhat_hist = zeros(2,N);

for i = 1:N
    
    % Proper noise scaling
    w = sqrtm(Q) * randn(2,1) * sqrt(dt);
    v = sqrtm(R) * randn(2,1);
    
    % True system
    x = x + (A*x + B*u)*dt + w;
    y = C*x + v;
    
    % Prediction
    xhat = xhat + (A*xhat + B*u)*dt;
    P = P + (A*P + P*A' + Q)*dt;
    
    % Kalman Gain (numerically stable)
    S = C*P*C' + R;
    K = P*C'/S;
    
    % Update
    xhat = xhat + K*(y - C*xhat);
    P = (eye(2) - K*C)*P;
    
    % Store
    x_hist(:,i) = x;
    xhat_hist(:,i) = xhat;
end

% Plot AFR
figure
plot(t, x_hist(1,:), 'b', t, xhat_hist(1,:), 'r','LineWidth',1.5)
xlabel('Time (s)')
ylabel('AFR')
title('True AFR vs Estimated AFR')
legend('True AFR','Estimated AFR')
grid on

% Plot RPM
figure
plot(t, x_hist(2,:), 'b', t, xhat_hist(2,:), 'r','LineWidth',1.5)
xlabel('Time (s)')
ylabel('RPM')
title('True RPM vs Estimated RPM')
legend('True RPM','Estimated RPM')
grid on
