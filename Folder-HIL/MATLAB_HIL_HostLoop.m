%% HIL Real-Time Host Loop for Arduino - LQR/LQG Fuel Injection
% Compatible with Controller_Firmware.ino (100 Hz)
% Pradeep's LQR Research Repo

clear; clc; close all;

%% ========== CONFIGURATION ==========
COM_PORT = 'COM3';          % ← CHANGE TO YOUR ARDUINO PORT (check Device Manager)
BAUD_RATE = 115200;
SIM_TIME = 30;              % seconds
CONTROL_FREQ = 100;         % Must match Arduino CONTROL_PERIOD = 10 ms

% Reference & Test Scenario
lambda_ref = 1.0;           % Stoichiometric (λ = 1)
disturbance_time = 8;       % Time when load change occurs

fprintf('=== LQR/LQG Fuel Injection HIL Started ===\n');
fprintf('Port: %s | Control Rate: %d Hz | Duration: %d s\n\n', COM_PORT, CONTROL_FREQ, SIM_TIME);

%% ========== SERIAL SETUP ==========
try
    ser = serialport(COM_PORT, BAUD_RATE, 'Timeout', 2);
    configureTerminator(ser, "CR/LF");
    pause(2.5);  % Wait for Arduino boot
    disp('✅ Arduino Connected Successfully');
catch ME
    error('Serial connection failed: %s\nCheck port and Arduino connection.', ME.message);
end

%% ========== DATA STORAGE ==========
t = 0:0.01:SIM_TIME;
n = length(t);

lambda_ref_profile = lambda_ref * ones(1, n);
lambda_meas = zeros(1, n);
lambda_error = zeros(1, n);
u_command = zeros(1, n);
states = zeros(4, n);   % For logging if needed

%% ========== REAL-TIME HIL LOOP ==========
disp('🚀 Starting HIL Simulation... (Press Ctrl+C to stop)');

tic;
for i = 2:n
    current_time = t(i);
    
    % Generate disturbance (load change)
    if current_time > disturbance_time
        dist = 0.25 * exp(-0.8*(current_time - disturbance_time));
    else
        dist = 0;
    end
    
    % Read measurement from plant model (or real sensor in future)
    % Here we simulate plant + noise
    lambda_meas(i) = lambda_ref + 0.08*sin(2*pi*0.5*current_time) + dist + 0.04*randn();
    lambda_error(i) = lambda_meas(i) - lambda_ref;
    
    % === Send data to Arduino ECU ===
    header = uint8(0x55);
    lambda_bytes = typecast(single(lambda_meas(i)), 'uint8');
    error_bytes = typecast(single(lambda_error(i)), 'uint8');
    footer = uint8(0xAA);
    
    write(ser, [header; lambda_bytes'; error_bytes'; footer], 'uint8');
    
    % === Read control command from Arduino ===
    if ser.NumBytesAvailable >= 6
        data = read(ser, 6, 'uint8');
        if data(1) == 0x66 && data(6) == 0xBB
            pwm_raw = typecast(uint8(data(2:3)), 'uint16');
            u_command(i) = double(pwm_raw) / 500.0 - 1.0;   % Convert back to -1..1
            u_command(i) = max(-1, min(1, u_command(i)));
        end
    end
    
    % Real-time plotting (live)
    if mod(i, 10) == 0
        subplot(2,1,1);
        plot(t(1:i), lambda_meas(1:i), 'b', 'LineWidth', 1.5);
        hold on;
        plot(t(1:i), lambda_ref_profile(1:i), 'r--', 'LineWidth', 2);
        ylabel('λ (Air-Fuel Ratio)');
        title(sprintf('HIL Real-Time - Time: %.1f s', current_time));
        legend('Measured λ', 'Reference λ=1', 'Location','best');
        grid on;
        
        subplot(2,1,2);
        plot(t(1:i), u_command(1:i), 'g', 'LineWidth', 1.5);
        ylabel('Control Input u (Fuel Command)');
        xlabel('Time (s)');
        title('LQR/LQG Control Effort');
        grid on;
        drawnow;
    end
    
    % Maintain real-time pace
    while toc < t(i)
        pause(0.001);
    end
end

%% ========== CLEANUP & RESULTS ==========
clear ser;
disp('✅ HIL Simulation Completed');

% Final Plots
figure('Position',[100 100 1200 700]);

subplot(2,2,1);
plot(t, lambda_meas, 'b', 'LineWidth',1.5); hold on;
plot(t, lambda_ref_profile, 'r--', 'LineWidth',2);
grid on; title('AFR (λ) Tracking'); ylabel('λ');

subplot(2,2,2);
plot(t, lambda_meas - lambda_ref, 'm', 'LineWidth',1.5);
grid on; title('Tracking Error'); ylabel('Error');

subplot(2,2,3);
plot(t, u_command, 'g', 'LineWidth',1.5);
grid on; title('Fuel Injection Command (u)'); ylabel('u');

subplot(2,2,4);
plot(t, lambda_meas, 'b', t, u_command*0.3+1, 'g');
grid on; title('Control vs AFR'); legend('λ','Control u (scaled)');

save('HIL_Results_Arduino.mat', 't', 'lambda_meas', 'u_command', 'lambda_ref');
fprintf('Results saved to HIL_Results_Arduino.mat\n'); 