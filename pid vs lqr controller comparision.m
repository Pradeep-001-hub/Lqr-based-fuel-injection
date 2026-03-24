A = [-0.4 0.1; 0.05 -0.2];
B = [0.3; 0.1];

Q = diag([20 10]);
R = 0.5;

K = lqr(A,B,Q,R);

Kp = 0.8;
Ki = 0.2;
Kd = 0.05;

dt = 0.01;
t = 0:dt:10;

x_lqr = [16;50];
x_pid = [16;50];

integral = 0;
prev_error = 0;

afr_lqr = [];
afr_pid = [];
rpm_lqr = [];
rpm_pid = [];

setpoint = 14.7;

for i = 1:length(t)

    u_lqr = -K*x_lqr;
    xdot_lqr = A*x_lqr + B*u_lqr;
    x_lqr = x_lqr + xdot_lqr*dt;

    error = setpoint - x_pid(1);
    integral = integral + error*dt;
    derivative = (error - prev_error)/dt;
    u_pid = Kp*error + Ki*integral + Kd*derivative;
    prev_error = error;

    xdot_pid = A*x_pid + B*u_pid;
    x_pid = x_pid + xdot_pid*dt;

    afr_lqr = [afr_lqr x_lqr(1)];
    afr_pid = [afr_pid x_pid(1)];
    rpm_lqr = [rpm_lqr x_lqr(2)];
    rpm_pid = [rpm_pid x_pid(2)];
end

figure
plot(t,afr_lqr,t,afr_pid)
legend('LQR','PID')
xlabel('Time')
ylabel('AFR')

figure
plot(t,rpm_lqr,t,rpm_pid)
legend('LQR','PID')
xlabel('Time')
ylabel('RPM')