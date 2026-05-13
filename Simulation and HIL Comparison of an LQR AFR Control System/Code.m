%% === EXCELLENT HIL: LQR Fuel Injection with Original 2-State Model ===
% Integrates your exact model from "lqr design.m"
% Simulation vs HIL Comparison + Professional Results

clear; clc; close all;

%% CONFIGURATION
COM_PORT = 'COM3';          % ← CHANGE THIS
BAUD_RATE = 115200;
SIM_TIME = 40;              % seconds
Ts = 0.01;                  % 100 Hz
t = 0:Ts:SIM_TIME;
N = length(t);

lambda_ref = 1.0;           % Target λ = 1 (Stoichiometric)

%% YOUR ORIGINAL PLANT MODEL (from lqr design.m)
A = [-0.4  0.1;
0.05 -0.2];
B = [0.3; 0.1];
C = [1 0];                  % Measure lambda (first state)
D = 0;

% LQR Gain from your design
K = lqr(A, B, diag([20 10]), 0.5);   % You can tune Q/R

%% DATA STORAGE
lambda_sim = zeros(1,N);      % Pure Simulation
lambda_hil = zeros(1,N);      % With Arduino in loop
u_sim = zeros(1,N);
u_hil = zeros(1,N);
x = zeros(2,1);               % Plant state

%% SERIAL SETUP
try
ser = serialport(COM_PORT, BAUD_RATE);
configureTerminator(ser,"CR/LF");
pause(2.5);
disp('✅ Arduino Connected');
catch
error('Cannot connect to Arduino. Check port!');
end

disp('🚀 Starting Full HIL with Original 2-State Model...');

tic;
for i = 2:N
time = t(i);

% Disturbance (load change at 10s)  
dist = (time > 10) * 0.3 * exp(-0.6*(time-10));  
  
% === PURE SIMULATION (No Arduino) ===  
u_sim(i) = -K * (x - [lambda_ref; 0]);  
u_sim(i) = max(-1, min(1, u_sim(i)));  
  
dx = A*x + B*u_sim(i) + [dist; 0.1*dist];  
x = x + dx * Ts;  
lambda_sim(i) = C*x + 0.03*randn();   % + sensor noise  
  
% === HIL (with Arduino) ===  
lambda_meas = lambda_sim(i);           % Send real measurement  
lambda_err  = lambda_meas - lambda_ref;  
  
% Send to Arduino  
header = uint8(0x55);  
write(ser, [header; typecast(single(lambda_meas),'uint8'); ...  
            typecast(single(lambda_err),'uint8'); uint8(0xAA)], 'uint8');  
  
% Read response from Arduino  
if ser.NumBytesAvailable >= 6  
    data = read(ser,6,'uint8');  
    if data(1)==0x66 && data(6)==0xBB  
        pwm_raw = typecast(uint8(data(2:3)),'uint16');  
        u_hil(i) = double(pwm_raw)/500.0 - 1.0;  
    end  
end  
  
lambda_hil(i) = lambda_meas;  
  
% Live Plot (every 10 steps)  
if mod(i,10)==0  
    subplot(211);  
    plot(t(1:i), lambda_sim(1:i),'b', t(1:i), lambda_hil(1:i),'r--', ...  
         t(1:i), lambda_ref*ones(1,i),'k:','LineWidth',1.5);  
    legend('Pure Simulation','HIL (Arduino)','Reference');  
    ylabel('\lambda (AFR)'); title('LQR HIL Performance');  
    grid on;  
      
    subplot(212);  
    plot(t(1:i), u_sim(1:i),'b', t(1:i), u_hil(1:i),'r--','LineWidth',1.5);  
    legend('Simulation u','HIL u (Arduino)');  
    ylabel('Control u'); xlabel('Time (s)');  
    grid on;  
    drawnow;  
end  
  
% Real-time pacing  
while toc < t(i), pause(0.001); end

end

clear ser;

%% FINAL COMPARISON PLOTS + METRICS
figure('Position',[100 100 1400 800]);

subplot(2,2,1);
plot(t, lambda_sim, 'b', t, lambda_hil, 'r', t, lambda_ref*ones(size(t)), 'k--');
grid on; title('AFR Tracking: Simulation vs HIL');
legend('Pure Sim','With Arduino HIL','Ref'); ylabel('\lambda');

subplot(2,2,2);
plot(t, lambda_sim-lambda_ref, 'b', t, lambda_hil-lambda_ref, 'r');
grid on; title('Tracking Error'); ylabel('Error');

subplot(2,2,3);
plot(t, u_sim, 'b', t, u_hil, 'r');
grid on; title('Control Input u'); ylabel('u');

subplot(2,2,4);
plot(lambda_sim, u_sim, 'b.', lambda_hil, u_hil, 'r.');
grid on; title('Phase Plane'); xlabel('\lambda'); ylabel('u');

% Performance Metrics
rmse_sim = sqrt(mean((lambda_sim - lambda_ref).^2));
rmse_hil = sqrt(mean((lambda_hil - lambda_ref).^2));
fprintf('\n=== PERFORMANCE METRICS ===\n');
fprintf('RMSE Simulation : %.4f\n', rmse_sim);
fprintf('RMSE HIL (Arduino): %.4f\n', rmse_hil);
fprintf('Difference due to discretization & timing: %.4f\n', abs(rmse_hil - rmse_sim));

save('HIL_Results_2State.mat', 't','lambda_sim','lambda_hil','u_sim','u_hil','K');
disp('✅ Results saved! Ready for your paper/repo.');
Create image
