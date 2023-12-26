% 这是一个通用的 MPC 模型预测算法的代码
clear;
clear all;
clc;

%%%%%系统矩阵
A = [1 0.1; 0 2];
n = size(A, 1);
B = [0.2 1; 0.5 2]; % 大小取决于输入量的数目
p = size(B, 2);

%%%%% 权重矩阵
Q = [100 0; 0 1];
F = [100 0; 0 1];
R = [1 0; 0 0.1]; % 大小取决于输入量
k_step = 100; % 走100步

x_k = zeros(n, k_step);
x_k(:, 1) = [20; -20];
u_k = zeros(p, k_step);

N = 5; % 预测区间
[E, H] = MPC_Matrices(A, B,Q, R, F, N);

for k = 1: k_step
    u_k(:, k) = Prediction(x_k(:, k), E, H, N, p);
    x_k(:, k+1) = (A * x_k(:, k) + B * u_k(:, k));
end

subplot(2, 1, 1);
hold;
for i = 1: size(x_k, 1)
    plot(x_k(1, :));
    plot(x_k(2, :));
end
legend("x1", "x2");
hold off;

subplot(2, 1, 2);
hold;
for i = 1: size(x_k, 1)
    plot(u_k(1, :));    
    plot(u_k(2, :));

end
legend("u1", "u2");