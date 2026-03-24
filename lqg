A = [-0.4 0.1; 0.05 -0.2];
B = [0.3; 0.1];
C = eye(2);

Q = diag([20 10]);
R = 0.5;

K = lqr(A,B,Q,R);

Qn = 0.1*eye(2);
Rn = 0.05*eye(2);

L = lqe(A,eye(2),C,Qn,Rn);

dt = 0.01;
t = 0:dt:10;

x = [16;50];
xhat = [14;45];

afr = [];
rpm = [];

for i = 1:length(t)
    u = -K*xhat;

    xdot = A*x + B*u;
    x = x + xdot*dt;

    y = C*x;

    xhat_dot = A*xhat + B*u + L*(y - C*xhat);
    xhat = xhat + xhat_dot*dt;

    afr = [afr x(1)];
    rpm = [rpm x(2)];
end

figure
plot(t,afr)
xlabel('Time')
ylabel('AFR')

figure
plot(t,rpm)
xlabel('Time')
ylabel('RPM')