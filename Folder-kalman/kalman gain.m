clear; clc; close all;

A = [-0.4 0.1; 0.05 -0.2];
C = eye(2);
Q = 0.01*eye(2);
R = 0.05*eye(2);

dt = 0.01;
t = 0:dt:10;
N = length(t);

xhat = [14; 45];
P = eye(2);

K_hist = zeros(2,2,N);

for i = 1:N
    v = sqrtm(R)*randn(2,1);
    y = C*xhat + v;

    xhat = xhat + (A*xhat)*dt;
    P = P + (A*P + P*A' + Q)*dt;

    K = P*C'/(C*P*C' + R);

    xhat = xhat + K*(y - C*xhat);
    P = (eye(2) - K*C)*P;

    K_hist(:,:,i) = K;
end

figure
plot(t,squeeze(K_hist(1,1,:)),'LineWidth',1.5); hold on
plot(t,squeeze(K_hist(2,2,:)),'LineWidth',1.5)
title('Kalman Gain Evolution')
legend('K(1,1)','K(2,2)')
grid on