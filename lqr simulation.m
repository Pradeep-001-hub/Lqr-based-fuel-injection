A = [-0.4 0.1; 0.05 -0.2];
B = [0.3; 0.1];

Q = diag([20 10]);
R = 0.5;

K = lqr(A,B,Q,R);

dt = 0.01;
t = 0:dt:10;

x = [16; 50];

afr = [];
rpm = [];
u_hist = [];

for i = 1:length(t)
    u = -K*x;
    xdot = A*x + B*u;
    x = x + xdot*dt;

    afr = [afr x(1)];
    rpm = [rpm x(2)];
    u_hist = [u_hist u];
end

figure
plot(t,afr)
xlabel('Time')
ylabel('AFR')

figure
plot(t,rpm)
xlabel('Time')
ylabel('RPM')

figure
plot(t,u_hist)
xlabel('Time')
ylabel('Control')