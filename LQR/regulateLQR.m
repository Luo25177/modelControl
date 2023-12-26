clear all;
close all;
clc;

%%%%%%%%%%系统定义
%定义系统矩阵A
A = [0 1; -1 -0.5];
n = size(A, 1);
%定义系统矩阵B
B = [0; 1];
p = size(B,2);

%%%%%%%%系统离散
%离散时间步长
Ts = 0.1;
%连续系统转为离散系统 不是必须的
%sys_d = c2d(ss(A, B), Ts);
%A = sys_d.a;
%B = sys_d.b;
%%%%%%%%%%系统初始化
%初始状态 可以有多个输入值，这个输入值是一个位移和速度
x0 = [1; 0];
x = x0;
%初始输入
u0 = 2;
u = u0;
%%%%%%%%%%初始化参数
%定义运行步数
k_steps = 100;
%定义存储系统状态结果的矩阵 维度为 n * k_step
x_history = zeros(n, k_steps);
%系统状态赋值存储
x_history(:, 1) = x;%对第一次进行赋值
%定义系统输入状态存储矩阵 p*k_step
u_history = zeros(p, k_steps);
%将系统输入初始赋值
u_history(:, 1) = u;
%%%%%%%%%%权重矩阵设计
%系统状态权重矩阵
Q = [1 0; 0 1];
%系统终值权重矩阵
S = [1 0; 0 1];
%系统输入权重矩阵(单输入)
R = 1;
%p矩阵初始化
p_k = S;
%F_N矩阵计算，可以提前计算好，之后使用，F_N的结果与系统的状态无关，与系统的输入输出也无关
N = k_steps;
for k = 1: N
    F = inv(R + B'* p_k * B)*B'* p_k * A;
    p_k = (A - B * F)' * p_k * (A - B * F) + (F)' * R * (F) + Q;
    if k == 1
        F_N = F;
    else 
        F_N = [F; F_N];
    end
end
%%%%%%%%%%开始仿真
for k = 1: k_steps
    %计算系统输入
    u = - F_N((k-1)*p+1:k*p, :) * x;
    %系统输入带入系统方程，计算系统响应
    x = A * x + B * u;
    %保存系统状态到预先定义矩阵的相应位置
    x_history(:, k+1) = x;
    %保存系统输入到预先定义矩阵的相应位置
    u_history(:, k) = u;
end

%%%%%%%%%%结果显示
%系统状态视图
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