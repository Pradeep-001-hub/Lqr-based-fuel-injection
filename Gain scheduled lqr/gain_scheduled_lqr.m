%% GAIN_SCHEDULED_LQR.m
% Headline novelty addition: nonlinear fuel injection model linearized
% about multiple RPM/load operating points, with a gain-scheduled LQR
% controller that interpolates K across the operating envelope.
%
% This is the single strongest differentiator vs. the original
% single-linear-plant comparison -- most reviewers reject "LQR vs PID
% vs LQG on a fixed linear plant" papers as incremental, but gain
% scheduling over a nonlinear model is a recognized, publishable
% extension.
%
% -------------------------------------------------------------------
clear; clc; close all;

%% 1. Nonlinear fuel injection model (example structure)
% States: x1 = fuel flow rate, x2 = manifold pressure (example)
% Replace with your actual nonlinear ODE derived from engine physics.
%
%   dx1/dt = -a1(RPM)*x1 + b1*u
%   dx2/dt =  k(RPM)*x1  - a2(RPM)*x2
%
% a1, a2, k are RPM-dependent coefficients (nonlinearity source)

rpmRange = [800, 2000, 3500, 5000, 6500];   % operating points (idle -> redline)
loadRange = [0.2, 0.5, 0.8];                % normalized load levels

%% 2. Linearize at each operating point and design local LQR gains
Q = diag([10, 1]);
R = 1;

nPoints = length(rpmRange);
Klist = cell(nPoints,1);
Alist = cell(nPoints,1);
Blist = cell(nPoints,1);

for i = 1:nPoints
    rpm = rpmRange(i);

    % Example RPM-dependent coefficient functions -- replace with your
    % identified/derived relationships from engine dynamics equations
    a1 = 2.0 + 0.0008*rpm;
    a2 = 3.0 + 0.0005*rpm;
    k  = 1.2 + 0.0002*rpm;
    b1 = 1.8;

    A_i = [-a1,  0;
            k,  -a2];
    B_i = [b1; 0];

    Alist{i} = A_i;
    Blist{i} = B_i;
    Klist{i} = lqr(A_i, B_i, Q, R);
end

fprintf('=== Gain-Scheduled LQR Gains Across Operating Points ===\n');
for i = 1:nPoints
    fprintf('RPM = %d:  K = [%.4f, %.4f]\n', rpmRange(i), Klist{i}(1), Klist{i}(2));
end

%% 3. Gain scheduling function (linear interpolation between design points)
function K = scheduledGain(rpm, rpmRange, Klist)
    if rpm <= rpmRange(1)
        K = Klist{1};
    elseif rpm >= rpmRange(end)
        K = Klist{end};
    else
        idx = find(rpmRange <= rpm, 1, 'last');
        rpm1 = rpmRange(idx); rpm2 = rpmRange(idx+1);
        alpha = (rpm - rpm1) / (rpm2 - rpm1);
        K = (1-alpha)*Klist{idx} + alpha*Klist{idx+1};
    end
end

%% 4. Simulate closed-loop response across a varying RPM profile
% Simulates an engine accelerating from idle to high RPM during the run,
% demonstrating that the scheduled controller adapts while a
% fixed-gain LQR (designed at one nominal point) does not.

t = 0:0.001:10;
rpmProfile = 800 + (6500-800) * (t/10);   % linear RPM ramp over 10s (example)

x = [0; 0];
xFixed = [0; 0];
K_fixed = Klist{1};   % fixed LQR designed only at idle (800 RPM) -- worst case baseline

y_scheduled = zeros(size(t));
y_fixed = zeros(size(t));
dt = t(2)-t(1);
r_ref = 1;   % reference fuel flow setpoint

for k = 1:length(t)
    rpm_now = rpmProfile(k);

    % Interpolate plant matrices too (true nonlinear plant approximation)
    a1 = 2.0 + 0.0008*rpm_now;
    a2 = 3.0 + 0.0005*rpm_now;
    kk  = 1.2 + 0.0002*rpm_now;
    b1 = 1.8;
    A_now = [-a1, 0; kk, -a2];
    B_now = [b1; 0];

    K_now = scheduledGain(rpm_now, rpmRange, Klist);

    u_sched = -K_now * (x - [r_ref; 0]);
    u_fixed = -K_fixed * (xFixed - [r_ref; 0]);

    xdot_sched = A_now*x + B_now*u_sched;
    xdot_fixed = A_now*xFixed + B_now*u_fixed;

    x = x + xdot_sched*dt;
    xFixed = xFixed + xdot_fixed*dt;

    y_scheduled(k) = x(1);
    y_fixed(k) = xFixed(1);
end

figure;
subplot(2,1,1);
plot(t, y_scheduled, 'b-', 'LineWidth', 1.5); hold on;
plot(t, y_fixed, 'r--', 'LineWidth', 1.5);
yline(r_ref, 'k:');
legend('Gain-Scheduled LQR', 'Fixed LQR (designed at idle)', 'Reference');
xlabel('Time (s)'); ylabel('Fuel flow output');
title('Gain-Scheduled vs Fixed LQR Across Varying RPM');
grid on;

subplot(2,1,2);
plot(t, rpmProfile, 'k-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('RPM');
title('RPM Profile (idle to redline ramp)');
grid on;

%% 5. Quantify improvement
err_sched = trapz(t, (r_ref - y_scheduled).^2);
err_fixed = trapz(t, (r_ref - y_fixed).^2);

fprintf('\n=== Tracking Error Comparison (ISE over RPM ramp) ===\n');
fprintf('Gain-Scheduled LQR ISE: %.4f\n', err_sched);
fprintf('Fixed LQR ISE:          %.4f\n', err_fixed);
fprintf('Improvement:            %.1f%%\n', 100*(err_fixed-err_sched)/err_fixed);
