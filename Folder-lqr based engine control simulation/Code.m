
A = [-0.4  0.1;
      0.05 -0.2];
B = [0.3; 0.1];

Q = diag([20 10]);
R = 0.5;

K = lqr(A, B, Q, R);
disp('Computed LQR gain K:');
disp(K);

dt = 0.01;
t = 0:dt:10;
N = length(t);

x = [16; 50];           % initial state
afr = zeros(1, N);
rpm = zeros(1, N);
u_hist = zeros(1, N);

for i = 1:N
    u = -K * x;                    % control input
    xdot = A*x + B*u;
    x = x + xdot * dt;             % Euler integration
    
    afr(i) = x(1);
    rpm(i) = x(2);
    u_hist(i) = u;
end

% Plotting - clean and accurate
figure('Position',[100 100 800 600]);

subplot(3,1,1);
plot(t, afr, 'b', 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('AFR');
title('AFR Response');

subplot(3,1,2);
plot(t, rpm, 'r', 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('RPM');
title('RPM Response');

subplot(3,1,3);
plot(t, u_hist, 'g', 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('Control Input u');
title('Control Input');

sgtitle('LQR Control Simulation (Euler Integration)');