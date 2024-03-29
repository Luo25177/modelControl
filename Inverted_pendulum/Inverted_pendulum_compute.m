clc
clear
syms m ml F N P L g t Il
syms x(t) theta(t)
syms x_dot x_ddot theta_dot theta_ddot

m = 0.5;
ml = 0.2;
L = 0.6;
g = 9.805;
Il = 0.006;


func = [F - P * tan(theta) == m * diff(diff(x, t), t);
        N == diff(diff(L * sin(theta) + x, t), t) * ml;
        P - ml * g == ml * diff(diff(L * cos(theta), t), t);
        P * L / 2 * sin(theta) - N * L / 2 * cos(theta) == Il * diff(diff(theta, t), t);];
func = subs(func, [diff(diff(x, t), t), diff(diff(theta, t), t), diff(x, t), diff(theta, t)], ...
    [x_ddot, theta_ddot, x_dot, theta_dot]);

[N, P, x_ddot, theta_ddot] = solve(func, [N, P, x_ddot, theta_ddot]);

X = [x(t); x_dot; theta(t); theta_dot];
X_dot = [x_dot; x_ddot; theta_dot; theta_ddot];
U = [F];
A = jacobian(X_dot, X);
B = jacobian(X_dot, U);
A = subs(A, [theta_dot, theta(t), x_dot, F], [0, 0, 0, 0]);
B = subs(B, [theta_dot, theta(t), x_dot, F], [0, 0, 0, 0]);
A = double(A);
B = double(B);
C = diag([1, 1, 1, 1]);
D = diag(0);
%% LQR
sys = ss(A, B, C, D);
Q = diag([10, 1, 100, 1]);
R = diag(1);
K = lqr(sys, Q, R);

%% MPC
sys_d = c2d(sys, 0.001);
sys_d.A
sys_d.B




