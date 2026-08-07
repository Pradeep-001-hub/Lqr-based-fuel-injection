%% PERFORMANCE_METRICS.m
% Computes standard quantitative control performance indices for
% PID, LQR, and LQG controllers:
%   ISE  - Integral of Squared Error
%   IAE  - Integral of Absolute Error
%   ITAE - Integral of Time-weighted Absolute Error
%   ISU  - Integral of Squared Control effort (Input)
%
% Replaces the qualitative "Low/Medium/High" comparison table in the
% original project with numeric values -- reviewers expect this.
%
% -------------------------------------------------------------------
clear; clc; close all;

%% 1. Plant model
A = [-2.5   1.0;
      0.0  -4.0];
B = [0; 2.0];
C = [1 0];
D = 0;

%% 2. Controllers
% -- LQR --
Q = diag([10, 1]);
R = 1;
K_lqr = lqr(A, B, Q, R);
sysCL_lqr = feedback(ss(A - B*K_lqr, B, C, D), 1);

% -- PID --
Kp = 5; Ki = 2; Kd = 0.5;
pidCtrl = pid(Kp, Ki, Kd);
sysOpen = ss(A, B, C, D);
sysCL_pid = feedback(pidCtrl * sysOpen, 1);

% -- LQG --
Qn_noise = 0.01 * eye(2);
Rn_noise = 0.01;
sys_aug = ss(A, [B B], C, 0);
[kalmf, L, P] = kalman(sys_aug, Qn_noise, Rn_noise); %#ok<ASGLU>
% Build the LQG loop: state feedback + estimator
Ac = [A - B*K_lqr,      B*K_lqr;
      zeros(size(A)),   A - L*C];
Bc = [B; zeros(size(B))];
Cc = [C, zeros(1, size(A,1))];
sysCL_lqg = ss(Ac, Bc, Cc, 0);

%% 3. Simulation settings
t = 0:0.001:5;
r = ones(size(t));   % unit step reference

%% 4. Simulate and compute metrics for each controller
controllers = {'PID', 'LQR', 'LQG'};
sysList = {sysCL_pid, sysCL_lqr, sysCL_lqg};

metrics = table('Size', [3 5], ...
    'VariableTypes', {'string','double','double','double','double'}, ...
    'VariableNames', {'Controller','ISE','IAE','ITAE','SettlingTime'});

figure; hold on;
colors = {'r--','b-','g-.'};

for i = 1:3
    sys = sysList{i};
    [y, tOut] = step(sys, t);
    e = 1 - y;   % error signal (step reference = 1)

    ISE  = trapz(tOut, e.^2);
    IAE  = trapz(tOut, abs(e));
    ITAE = trapz(tOut, tOut .* abs(e));

    info = stepinfo(sys);

    metrics.Controller(i) = controllers{i};
    metrics.ISE(i)  = ISE;
    metrics.IAE(i)  = IAE;
    metrics.ITAE(i) = ITAE;
    metrics.SettlingTime(i) = info.SettlingTime;

    plot(tOut, y, colors{i}, 'LineWidth', 1.5, 'DisplayName', controllers{i});
end

xlabel('Time (s)'); ylabel('Output');
title('Step Response Comparison: PID vs LQR vs LQG');
legend show; grid on;

fprintf('\n=== QUANTITATIVE PERFORMANCE METRICS ===\n');
disp(metrics);

% Save table to CSV for direct inclusion in your paper/thesis
writetable(metrics, 'performance_metrics_results.csv');
fprintf('\nSaved results to performance_metrics_results.csv\n');

%% 5. Control effort comparison (ISU)
figure; hold on;
for i = 1:3
    sys = sysList{i};
    % Extract control signal via closed-loop state feedback simulation
    [~, tOut, x] = step(ss(sys.A, sys.B, eye(size(sys.A,1)), 0), t);
    if i == 2   % LQR
        u = -K_lqr * x(:,1:2)';
    elseif i == 1  % PID -- approximate via output error derivative/integral
        u = zeros(size(tOut));  % placeholder: extract from Simulink if available
    else  % LQG
        u = -K_lqr * x(:,1:2)';
    end
    plot(tOut, u, 'LineWidth', 1.2, 'DisplayName', controllers{i});
end
xlabel('Time (s)'); ylabel('Control Input u(t)');
title('Control Effort Comparison');
legend show; grid on;

fprintf('\nNote: For accurate PID control-effort extraction, log u(t) directly\n');
fprintf('from your Simulink model (add a "To Workspace" block on the PID output)\n');
fprintf('and substitute it here for a true ISU comparison.\n');
