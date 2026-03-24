A = [-0.4 0.1; 0.05 -0.2];
B = [0.3; 0.1];
C = eye(2);

Q = 0.01*eye(2);
R = 0.05*eye(2);

dt = 0.01;
t = 0:dt:10;

x = [16;50];
xhat = [14;45];
P = eye(2);

u = 0;

x_hist = [];
xhat_hist = [];

for i = 1:length(t)

    w = sqrt(Q)*randn(2,1);
    v = sqrt(R)*randn(2,1);

    x = x + (A*x + B*u)*dt + w*dt;
    y = C*x + v;

    xhat = xhat + (A*xhat + B*u)*dt;
    P = P + (A*P + P*A' + Q)*dt;

    S = C*P*C' + R;
    K = P*C'/S;

    xhat = xhat + K*(y - C*xhat);
    P = (eye(2) - K*C)*P;

    x_hist = [x_hist x];
    xhat_hist = [xhat_hist xhat];
end

figure
plot(t,x_hist(1,:),t,xhat_hist(1,:))
legend('True AFR','Estimated AFR')

figure
plot(t,x_hist(2,:),t,xhat_hist(2,:))
legend('True RPM','Estimated RPM')