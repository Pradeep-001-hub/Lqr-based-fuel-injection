clear; clc; close all;

A = [0 1; -0.5 -1];
B = [0;1];
C = [1 0];
D = 0;

sys = ss(A,B,C,D);

Q = diag([100 10]);
R = 1;

K = lqr(A,B,Q,R);

Acl = A - B*K;
sys_lqr = ss(Acl,B,C,D);

t = 0:0.01:10;

[y,t,x] = step(sys_lqr,t);

u = -K*x';

info = stepinfo(sys_lqr);

figure

subplot(3,1,1)
plot(t,y,'b','LineWidth',2)
grid on
title('System Output (Fuel Injection Response)')
xlabel('Time (seconds)')
ylabel('Output')

subplot(3,1,2)
plot(t,x(:,1),'LineWidth',2)
hold on
plot(t,x(:,2),'LineWidth',2)
grid on
title('State Variables')
xlabel('Time (seconds)')
ylabel('States')
legend('x1','x2')

subplot(3,1,3)
plot(t,u,'LineWidth',2)
grid on
title('Control Effort (Fuel Input)')
xlabel('Time (seconds)')
ylabel('Control Input')

sgtitle('LQR-Based Fuel Injection System Performance')
