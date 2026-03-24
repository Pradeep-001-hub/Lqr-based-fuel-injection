A = [-0.4 0.1; 0.05 -0.2];
C = eye(2);

Qn = 0.1*eye(2);
Rn = 0.05*eye(2);

[L,P,E] = lqe(A,eye(2),C,Qn,Rn)