A = [-0.4 0.1; 0.05 -0.2];
B = [0.3; 0.1];
C = eye(2);

Q = diag([20 10]);
R = 0.5;

K = lqr(A,B,Q,R);

Qn = 0.1*eye(2);
Rn = 0.05*eye(2);

L = lqe(A,eye(2),C,Qn,Rn);

Acl = [A-B*K B*K; zeros(size(A)) A-L*C];
Bcl = [B; zeros(size(B))];
Ccl = [C zeros(size(C))];
Dcl = 0;

sys = ss(Acl,Bcl,Ccl,Dcl);
step(sys)