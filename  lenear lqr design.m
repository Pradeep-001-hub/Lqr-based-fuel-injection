% State-space matrices from linearized model
A = [-ka 0; 0 -kf];
B = [0; 1];

% Q and R selection: Trial-and-error for good AFR tracking
% Higher weight on fuel mass (x2) for tighter AFR control
Q = diag([20, 200]);   % Example values - tuned via simulation
R = 1;                 % Moderate penalty on fuel injection rate

[K, S, E] = lqr(A, B, Q, R);   % Compute optimal gain K

disp('LQR Gain K:'); disp(K);