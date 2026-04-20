## How to Create the Simulink Model (.slx)

Run the following MATLAB script to automatically generate the `engine_afr_lqr_control.slx` model:

```matlab
% create_engine_control_model.m
% Automatically creates a runnable Simulink model for LQR-based fuel injection

clear; clc; close all;

% 1. Define System Parameters (Update with your real values)
A = [-1.5  0.2; 
      0.1 -0.8];                    % State matrix
B = [0.8; 0.3];                      % Input matrix
C = eye(2);                          % Output matrix
D = 0;

Q = diag([100 1]);                   % Weight AFR higher
R = 0.01;                            % Control effort penalty

K = lqr(A, B, Q, R);

% Reference AFR
ref_value = 14.7;

% Create Simulink model
model_name = 'engine_afr_lqr_control';
new_system(model_name);
open_system(model_name);
set_param(model_name, 'StopTime', '10', 'Solver', 'ode45');

% Add blocks and connections here (full script available in repo)
% ... (rest of the script)