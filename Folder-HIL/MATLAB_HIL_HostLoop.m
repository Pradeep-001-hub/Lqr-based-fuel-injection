%% HIL Real-Time Host Loop - MATLAB
% Simulates fuel injection plant dynamics in real-time
% Communicates with embedded controller via serial port
% Validates PID/LQR/LQG performance

clear; clc; close all;

%% ========== CONFIGURATION ==========

% Serial Port Configuration
COM_PORT = 'COM3';          % Change to your Arduino port
BAUD_RATE = 115200;
TIMEOUT = 5;                % seconds

% Plant Parameters
PLANT_FREQ = 1000;          % Plant integration frequency (Hz)
CONTROL_FREQ = 100;         % Controller frequency (Hz)
SIM_TIME = 20;              % Total simulation time (seconds)

% Initial Conditions
lambda_ref = 1.0;           % Stoichiometric ratio (14.7:1)
rpm_ref = 3000;             % Target RPM

% Disturbance Profile
disturbance_type = 'transient';  % 'step', 'transient', 'noise'

fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║         LQR Fuel Injection Control - HIL System            ║\n');
fprintf('║        Hardware-in-Loop Real-Time Validation               ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

%% ========== PLANT MODEL DEFINITION ==========
% Linearized fuel injection dynamics (4-state)

% State: x = [lambda_error, d(lambda_error)/dt, fuel_film, combustion_lag]
% Output: lambda (wideband O2 sensor measurement)

% System Matrices (from your research)
A = [  0      1      0     0   ;
      -0.5   -0.3    0.1   0   ;
       0      0.2   -0.2   0.05;
       0      0      0.1   -0.15];

B = [  0    ;
       0.5  ;
       0    ;
       0    ];

C = [1 0 0 0];  % Measure lambda error only

D = 0;

% Create state-space system
sys = ss(A, B, C, D);

fprintf('[Plant Model Loaded]\n');
fprintf('  States: [λ_error, dλ/dt, fuel_film, combustion_lag]\n');
fprintf('  Controllable: %d,  Observable: %d\n\n', ...
    rank(ctrb(A, B)) == 4, rank(obsv(A, C)) == 4);

%% ========== NOISE MODEL ==========
% Wideband O2 sensor noise (realistic parameters)

lambda_noise_std = 0.05;     % 5% of sensor range
rpm_noise_std = 10;          % ±10 RPM

%% ========== TIME VECTORS ==========
dt_plant = 1 / PLANT_FREQ;
dt_control = 1 / CONTROL_FREQ;

t_sim = 0:dt_plant:SIM_TIME;
t_control_idx = 0:CONTROL_FREQ:SIM_TIME;

n_samples = length(t_sim);
n_control_steps = floor(SIM_TIME * CONTROL_FREQ) + 1;

%% ========== STATE STORAGE ==========
x = zeros(4, n_samples);            % Plant states
x(:, 1) = [0; 0; 0; 0];             % Initial condition

y = zeros(1, n_samples);             % Lambda measurement
y_noisy = zeros(1, n_samples);       % Noisy lambda

u = zeros(1, n_samples);             % Controller input
u_commanded = 0;                     % Current commanded control

lambda_ref_profile = zeros(1, n_samples);
disturbance = zeros(1, n_samples);

% Define disturbance profile
for i = 1:n_samples
    t = t_sim(i);
    lambda_ref_profile(i) = lambda_ref;
    
    if strcmp(disturbance_type, 'step')
        if t > 5 && t < 10
            disturbance(i) = 0.2;  % +20% rich (load transient)
        elseif t > 10 && t < 15
            disturbance(i) = -0.15;  % -15% lean
        end
        
    elseif strcmp(disturbance_type, 'transient')
        if t > 5 && t < 6
            disturbance(i) = 0.3 * exp(-5*(t-5));  % Fast transient
        end
        
    elseif strcmp(disturbance_type, 'noise')
        disturbance(i) = 0.1 * randn();  % Continuous disturbance
    end
end

%% ========== SERIAL COMMUNICATION SETUP ==========

try
    % Close any existing serial connections
    instrs = instrfind;
    if ~isempty(instrs)
        fclose(instrs);
        delete(instrs);
    end
    
    % Create serial port object
    ser = serial(COM_PORT, 'BaudRate', BAUD_RATE, 'Timeout', TIMEOUT);
    ser.InputBufferSize = 1024;
    ser.OutputBufferSize = 1024;
    fopen(ser);
    
    fprintf('[Serial Port Connected]\n');
    fprintf('  Port: %s @ %d baud\n', COM_PORT, BAUD_RATE);
    pause(2);  % Wait for Arduino to initialize
    fprintf('  Status: Ready\n\n');
    
    serial_connected = true;
    
catch ME
    fprintf('[WARNING] Serial connection failed: %s\n', ME.message);
    fprintf('         Running in SIMULATION MODE (no real hardware)\n\n');
    serial_connected = false;
end

%% ========== MAIN HIL LOOP ==========

fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║              STARTING REAL-TIME SIMULATION                 ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

control_idx = 1;
last_control_time = tic;

for i = 2:n_samples
    t = t_sim(i);
    
    % ===== PLANT INTEGRATION (Euler method) =====
    x_error = x(1, i-1) - lambda_ref;
    
    % Add disturbance
    plant_input = u_commanded + disturbance(i-1);
    
    % dx/dt = Ax + Bu + w (disturbance)
    dx = A * x(:, i-1) + B * plant_input;
    
    % Update state
    x(:, i) = x(:, i-1) + dx * dt_plant;
    
    % Measurement (lambda = first state)
    y(i) = x(1, i) + lambda_ref;  % Absolute lambda
    
    % Add realistic sensor noise
    y_noisy(i) = y(i) + lambda_noise_std * randn();
    
    % ===== CONTROL LOOP (Every 10ms) =====
    if mod(i-1, round(PLANT_FREQ/CONTROL_FREQ)) == 0
        
        elapsed = toc(last_control_time);
        
        % Error signal
        lambda_error = y_noisy(i) - lambda_ref;
        
        % ===== SEND STATE TO MICROCONTROLLER =====
        if serial_connected
            try
                % Pack data: [header][lambda_f32][error_f32][CRC][footer]
                msg_send = [uint8(0x55); ...
                            typecast(single(y_noisy(i)), 'uint8')'; ...
                            typecast(single(lambda_error), 'uint8')'; ...
                            uint8(0xAA)];
                
                fwrite(ser, msg_send);
                
                % ===== RECEIVE CONTROL FROM MICROCONTROLLER =====
                if ser.BytesAvailable > 0
                    msg_recv = fread(ser, min(ser.BytesAvailable, 8), 'uint8');
                    
                    if length(msg_recv) >= 6 && msg_recv(1) == 0x66
                        % Extract PWM control (2 bytes, uint16)
                        u_commanded = double(typecast(uint8(msg_recv(2:3)), 'uint16')) / 1000 - 0.5;
                        u_commanded = max(-1, min(1, u_commanded));  % Saturate
                    end
                end
                
            catch
                % Silent fail - continue simulation
            end
        else
            % Simulation mode - generate synthetic control
            % (In real HIL, this would come from the microcontroller)
            u_commanded = 0;  % Placeholder
        end
        
        u(i) = u_commanded;
        
        last_control_time = tic;
        control_idx = control_idx + 1;
        
    else
        u(i) = u_commanded;  % Hold last control value
    end
    
    % ===== REAL-TIME RATE LIMITING =====
    % This ensures simulation runs at accurate speed
    while (toc(last_control_time) < dt_plant) && serial_connected
        pause(0.0001);  % Busy-wait with small sleep
    end
    
end

%% ========== CLEANUP ==========

if serial_connected
    fclose(ser);
    delete(ser);
    fprintf('\n[Serial Port Closed]\n');
end

fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
fprintf('║         SIMULATION COMPLETE - PROCESSING RESULTS           ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

%% ========== POST-SIMULATION ANALYSIS ==========

% Calculate performance metrics
lambda_error_signal = y_noisy - lambda_ref;

% Overshoot
overshoot = (max(abs(lambda_error_signal(1000:5000))) / lambda_ref) * 100;

% Settling time (within 5% of reference)
settling_idx = find(abs(lambda_error_signal(1000:end)) > 0.05 * lambda_ref, 1, 'last');
if isempty(settling_idx)
    settling_time = t_sim(end);
else
    settling_time = t_sim(1000 + settling_idx);
end

% Control effort
control_effort = sum(u.^2) / length(u);

% Mean absolute error
mae = mean(abs(lambda_error_signal));

fprintf('Performance Metrics (Transient Response):\n');
fprintf('  Overshoot:      %.2f %%\n', overshoot);
fprintf('  Settling Time:  %.2f seconds\n', settling_time);
fprintf('  Control Effort: %.4f\n', control_effort);
fprintf('  Mean Abs Error: %.4f\n\n', mae);

%% ========== VISUALIZATION ==========

figure('Name', 'HIL Fuel Injection Control', 'NumberTitle', 'off', 'Position', [100 100 1400 900]);

% Lambda tracking
subplot(3,2,1);
plot(t_sim, y_noisy, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Measured λ (noisy)');
hold on;
plot(t_sim, y, 'g--', 'LineWidth', 1, 'DisplayName', 'True λ', 'Alpha', 0.7);
plot(t_sim, lambda_ref_profile, 'r--', 'LineWidth', 2, 'DisplayName', 'Reference');
xlabel('Time (s)'); ylabel('Air-Fuel Ratio (λ)');
title('Lambda Tracking Performance');
legend; grid on;

% Lambda error
subplot(3,2,2);
plot(t_sim, lambda_error_signal * 100, 'b-', 'LineWidth', 1.5);
hold on;
plot(t_sim, 5*ones(size(t_sim)), 'r--', 'LineWidth', 1, 'DisplayName', '±5% band');
plot(t_sim, -5*ones(size(t_sim)), 'r--', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Error (%)');
title('Lambda Error vs Tolerance');
grid on; legend;

% Control input
subplot(3,2,3);
plot(t_sim, u, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Injector PWM (Normalized)');
title('Control Signal (Fuel Injection)');
grid on;

% Disturbance
subplot(3,2,4);
plot(t_sim, disturbance, 'r-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Disturbance (Δλ)');
title('Applied Disturbance Profile');
grid on;

% Plant states
subplot(3,2,5);
plot(t_sim, x(1,:), 'LineWidth', 1.5, 'DisplayName', 'λ error');
hold on;
plot(t_sim, x(2,:), 'LineWidth', 1.5, 'DisplayName', 'dλ/dt');
plot(t_sim, x(3,:), 'LineWidth', 1.5, 'DisplayName', 'Fuel film');
plot(t_sim, x(4,:), 'LineWidth', 1.5, 'DisplayName', 'Combustion lag');
xlabel('Time (s)'); ylabel('State Value');
title('Plant State Evolution');
legend; grid on;

% Frequency content
subplot(3,2,6);
[Pxx, f] = periodogram(lambda_error_signal, hann(length(lambda_error_signal)), [], PLANT_FREQ);
semilogy(f, Pxx, 'b-', 'LineWidth', 1.5);
xlabel('Frequency (Hz)'); ylabel('Power');
title('Lambda Error Frequency Content');
xlim([0 10]); grid on;

sgtitle('Hardware-in-Loop Fuel Injection Control System', 'FontSize', 14, 'FontWeight', 'bold');

%% ========== SAVE RESULTS ==========

results.t_sim = t_sim;
results.y_noisy = y_noisy;
results.y_true = y;
results.u = u;
results.lambda_ref = lambda_ref_profile;
results.x = x;
results.disturbance = disturbance;
results.metrics.overshoot = overshoot;
results.metrics.settling_time = settling_time;
results.metrics.control_effort = control_effort;
results.metrics.mae = mae;

save('HIL_Results.mat', 'results');
fprintf('Results saved to: HIL_Results.mat\n');

fprintf('\nDone! Open HIL_Monitor_Dashboard.html for real-time visualization.\n');
