clear; clc; close all;

% (Same system setup)
A = [-0.4 0.1; 0.05 -0.2];
C = eye(2);
Q = 0.01*eye(2);
R = 0.05*eye(2);

dt = 0.01;
t = 0:dt:10;
N = length(t);

x = [16; 50];
xhat = [14; 45];
P = eye(2);

err_hist = zeros(2,N);

for i = 1:N
    w = sqrtm(Q)*randn(2,1)*sqrt(dt);
    v = sqrtm(R)*randn(2,1);

    x = x + (A*x)*dt + w;
    y = C*x + v;

    xhat = xhat + (A*xhat)*dt;
    P = P + (A*P + P*A' + Q)*dt;

    K = P*C'/(C*P*C' + R);

    xhat = xhat + K*(y - C*xhat);
    P = (eye(2) - K*C)*P;

    err_hist(:,i) = x - xhat;
end

figure
plot(t,err_hist(1,:),t,err_hist(2,:),'LineWidth',1.5)
title('Estimation Error')
legend('AFR Error','RPM Error')
grid on