clear all;
close all;
clc;

[A B C D] = tf2ss([1], [1 2 1])

% A = [0 1; -1 -0.5];
n = size(A,1);
% B = [0; 1];
p = size(B, 2);

Ts = 0.1;

[A, B]=c2d(A, B, Ts);

x0 = [0; 0];
x = x0;
xd = [1; 0];
xa = [x; xd];

u0 = 2;
u = u0;

k_steps = 100;

x_history = zeros(n, k_steps);
x_history(:, 1) = x;
u_history = zeros(p, k_steps);
u_history(:, 1) = u;

AD = eye(n);
Ca = [eye(n) -eye(n)];
Q = [1 0; 0 1];
S = [1 0; 0 1];
R = 0.01;
Sa = Ca' * S * Ca;
Qa = Ca' * Q * Ca;
Aa = [A zeros(n); zeros(n) AD];
Ba = [B; zeros(n, 1)];
p_k = Sa;

for k = 1: k_steps
    F = inv(R + Ba'* p_k * Ba) * Ba'* p_k * Aa;
    p_k = (Aa-Ba*F)'*p_k*(Aa-Ba*F)+ (F)'*R*(F)+Qa;

    %计算系统输入
    u = -F * xa;
    %系统输入带入系统方程，计算系统响应
    x = A * x + B * u;
    xa = [x; xd];
    %保存系统状态到预先定义矩阵的相应位置
    x_history(:, k+1) = x;
    %保存系统输入到预先定义矩阵的相应位置
    u_history(:, k) = u;
end
subplot(2, 1, 1);
for i = 1: n
    plot(x_history(i, :));
    hold;
end
legend(num2str((1: n)', 'x %d'));
xlim([1, k_steps]);
grid on;
%系统输入视图
subplot(2, 1, 2);
for i = 1: p
    stairs(u_history(i, :));
    hold;
end
legend(num2str((1: p)', 'u %d'));
xlim([1, k_steps]);
grid on