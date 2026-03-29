clear; clc; close all;

A = [0 1; -0.5 -1];
B = [0;1];
C = [1 0];

Q = diag([200 50]);
R = 1;

K = lqr(A,B,Q,R);

AFR_ref = 14.7;

tspan = [0 10];
x0 = [10; 1];

[t,x] = ode45(@(t,x) engine_model(t,x,K,AFR_ref), tspan, x0);

air = x(:,1);
fuel = x(:,2);
afr = air ./ fuel;

u = zeros(length(t),1);
for i = 1:length(t)
    error = AFR_ref - (x(i,1)/x(i,2));
    u(i) = -K*x(i,:)' + 0.5*error;
end

figure

subplot(3,1,1)
plot(t,afr,'b','LineWidth',2)
hold on
yline(AFR_ref,'r--','Reference')
grid on
title('AFR Tracking (Nonlinear System)')
xlabel('Time (seconds)')
ylabel('AFR')

subplot(3,1,2)
plot(t,air,'LineWidth',2)
hold on
plot(t,fuel,'LineWidth',2)
grid on
title('Air and Fuel Dynamics')
xlabel('Time (seconds)')
ylabel('States')
legend('Air','Fuel')

subplot(3,1,3)
plot(t,u,'LineWidth',2)
grid on
title('Fuel Injection Control Input')
xlabel('Time (seconds)')
ylabel('Control Effort')

sgtitle('Advanced LQR-Based Fuel Injection with AFR Tracking')

function dx = engine_model(t,x,K,AFR_ref)

A = [0 1; -0.5 -1];
B = [0;1];

air = x(1);
fuel = x(2);

afr = air / fuel;

error = AFR_ref - afr;

u = -K*x + 0.5*error;

disturbance = 0.3*sin(2*t);

nonlinear = [0; 0.2*air^2];

dx = A*x + B*(u + disturbance) + nonlinear;

end