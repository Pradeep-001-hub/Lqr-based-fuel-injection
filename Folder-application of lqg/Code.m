clear; clc; clr
A = [0 1; -0.5 -1];
B = [0; 1];
C = [1 0];

%LQR
Q = diag([200 50]);
R = 1;
K = lqr(A,B,Q,R);

%REFERENCE 
AFR_ref = 14.7;
Nbar = inv(-C * inv(A - B*K) * B);

%KALMAN FILTER 
W = 0.1*eye(2);   % Process noise
V = 0.5;          % Measurement noise

L = lqe(A, eye(2), C, W, V);

%SIMULATION
dt = 0.01;
t = 0:dt:10;

x = [16; 0];       % Initial AFR (rich)
x_hat = [0; 0];

u_max = 5;
u_min = -5;

% Storage
X = zeros(2,length(t));
Xhat = zeros(2,length(t));
U = zeros(1,length(t));
Y = zeros(1,length(t));
Yhat = zeros(1,length(t));

%% LOOP 
for i = 1:length(t)
    
    % Output with noise
    y = C*x + sqrt(V)*randn;
   
    u = -K*x_hat + Nbar*AFR_ref;
    
    % Saturation
    u = max(u_min, min(u, u_max));
    
    % System update
    x_dot = A*x + B*u + 0.01*randn(2,1);
    x = x + x_dot*dt;
    
    % Kalman update
    xhat_dot = A*x_hat + B*u + L*(y - C*x_hat);
    x_hat = x_hat + xhat_dot*dt;
    
    % Store
    X(:,i) = x;
    Xhat(:,i) = x_hat;
    U(i) = u;
    Y(i) = y;
    Yhat(i) = C*x_hat;
end
figure;

subplot(2,2,1)
plot(t, X(1,:), 'b', t, Yhat, 'r--','LineWidth',1.5)
hold on
yline(AFR_ref,'k--','Reference')
title('AFR Tracking (Correct)')
xlabel('Time (s)'); ylabel('AFR')
legend('True AFR','Estimated AFR','Reference')

subplot(2,2,2)
plot(t, abs(X(1,:) - Yhat),'g','LineWidth',1.5)
title('Estimation Error → 0')
xlabel('Time'); ylabel('Error')

subplot(2,2,3)
plot(t, U,'m','LineWidth',1.5)
title('Control Effort with Saturation')
xlabel('Time'); ylabel('u')

subplot(2,2,4)
plot(t, Y,'b', t, Yhat,'r--','LineWidth',1.5)
title('Noise vs Filtered Output')
xlabel('Time'); ylabel('AFR')
legend('Noisy','Filtered')
