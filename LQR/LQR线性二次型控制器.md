# LQR线性二次型控制器

线性指的是系统是线性的，典型的线性系统的状态方程 

$$
x(k+1)=Ax(k)+Bu(k)
$$

二次型是指代价函数是二次型的J是二次型的。前一项中 $N$ 表示末端时刻， $x_d$ 是系统的参考，也就是目标，这个 $J$ 是末端代价， $S$ 为末端状态的权重矩阵，是一个对角阵， $Q$ 是运行过程中的权重矩阵， $R$ 是控制量的权重矩阵， $S$ 和 $Q$ 是 $N\times N$ 的，但是 $R$ 是 $P\times P$的，如果对某个元素要求大时可以对应的将 $R$ 增大，关键的是 $S$ 和 $Q$ 都是半正定阵， $R$ 为正定阵，只有这样系统才会有最小值，即 $s≥0,q≥0,r>0$

对于一个系统来说，当 $x_d$ 为零时，该系统就是一个调节系统，不为零就是一个跟踪系统。

## LQR线性二次型调节器

对应的调节二次型的LQR的雅各比矩阵就是

$$
J=\frac{1}{2} x(N)^TSx(N)+\frac{1}{2}\sum_{k=0}^{N-1}{[x^T(k)Qx(k)+u^T(k)Ru(k)]}
$$

采用逆向分级的方法来进行分析

系统的状态空间方程如下

$$
x(k+1)=Ax(k)+Bu(k)\\k=0:\ x(1)=Ax(0)+Bu(0)\\k=1:\ x(2)=Ax(1)+Bu(1)=A(Ax(0)+Bu(0))+Bu(1)\\...\\k=N-1:\ x(N)=A^{k+1}x(0)+\sum_{i=0}^{k}A^{k-i}Bu(i)
$$

其中 $u(n-1)$ 可以使 $x(n-1)$ 转化为 $x(n)$

那么根据上式也可以得出对应的调节二次型的雅各比矩阵

$$
J=\frac{1}{2} x(N)^TSx(N)+\frac{1}{2}\sum_{k=0}^{N-1}{[x^T(k)Qx(k)+u^T(k)Ru(k)]}\\k=N:J_{N\rightarrow N}^*=\frac{1}{2} x(N)^TSx(N)=\frac{1}{2} x(N)^TP(0)x(N);\ S=P(0)\\k=N-1:J_{N-1\rightarrow N}^* = \frac{1}{2} x(N)^TSx(N)+\frac{1}{2} x(N-1)^TQx(N-1)+\frac{1}{2}u(N-1)^TRu(N-1)=J^*_{N\rightarrow N}+\frac{1}{2} x(N-1)^TQx(N-1)+\frac{1}{2}u(N-1)^TRu(N-1)\\k=N-2:J_{N-2\rightarrow N}^* = \frac{1}{2} x(N)^TSx(N)+\frac{1}{2} x(N-1)^TQx(N-1)+\frac{1}{2}u(N-1)^TRu(N-1)+\frac{1}{2} x(N-2)^TQx(N-2)+\frac{1}{2}u(N-2)^TRu(N-2)\\=J_{N-1\rightarrow N}^*+\frac{1}{2} x(N-2)^TQx(N-2)+\frac{1}{2}u(N-2)^TRu(N-2)
$$

其中标*的表示最优解

根据贝尔曼最优理论：

当 $J_{N-2→N}$为最优时 $J_{N-1→N}$一定为最优的

观察上个式子，对于每一步的最优解都是取决于最后的一部分，因为前一部分的最优是根据上一步的最优来的，所以对于每一步我们只需要求解最后一部分，而且由于 $x(k)$ 是系统状态量，所以是已知的，根本不需要考虑，因此就是求出最优的 $u(k)$ 就可以了。所以对于$J_{k→N}$的最优解求解就是直接求导，导数为0就可以得到最优控制策略，证明二阶导是一个正定矩阵，所以得到的 $u(k)$一定是最优的解。

下面将展示如何求解 $u(N-1)$，因为其他的求解方式与之相同，不再赘述

$$
J_{N-1\rightarrow N}^* = \frac{1}{2} x(N)^TSx(N)+\frac{1}{2} x(N-1)^TQx(N-1)+\frac{1}{2}u(N-1)^TRu(N-1)=J^*_{N\rightarrow N}+\frac{1}{2} x(N-1)^TQx(N-1)+\frac{1}{2}u(N-1)^TRu(N-1)
$$

由上述可知

$$
x(N)=Ax(N-1)+Bu(N-1)
$$

所以对应的雅可比矩阵可以转化为

$$
J_{N-1\rightarrow N}^* = \frac{1}{2} (Ax(N-1)+Bu(N-1))^TP(0)(Ax(N-1)+Bu(N-1))+\frac{1}{2} x(N-1)^TQx(N-1)+\frac{1}{2}u(N-1)^TRu(N-1)
$$

对上式求导

$$
\frac{\partial J_{N-1\rightarrow N}}{\partial u(N-1)}=0\\frist\ part: \frac{\partial \frac{1}{2}x(N)^TP(0)x(N)}{\partial u(N-1)}=\frac{\partial x(N)}{\partial u(N-1)}*\frac{\partial \frac{1}{2}x(N)^TP(0)x(N)}{\partial x(N)}=B^T*P(0)x(N)=B^TP(0)(Ax(N-1)+Bu(N-1))\\second\ part:\frac{\partial \frac{1}{2} x(N-1)^TQx(N-1)}{\partial u(N-1)}=0\\third\ part:\frac{\partial \frac{1}{2}u(N-1)^TRu(N-1)}{\partial u(N-1)}=Ru(N-1)\\\frac{\partial J_{N-1\rightarrow N}}{\partial u(N-1)}=B^TP(0)(Ax(N-1)+Bu(N-1))+Ru(N-1)
$$

解得

$$
B^TP(0)(Ax(N-1)+Bu(N-1))+Ru(N-1)=0\\\Rightarrow u^*(N-1)=-(B^TP(0)B+R)^{-1}B^TP(0)Ax(N-1)
$$

令 $F(N-1)=(B^TP(0)B+R)^{-1}B^TP(0)A$，原式化简为

$$
u^*(N-1)=-F(N-1)x(N-1)
$$

这就是一个全状态的一个反馈控制器。

由于一阶导为0只能证明是极值，所以求解二阶导来验证一下该结果是否为最小值

$$
\frac{\partial^2 J_{N-1\rightarrow N}}{\partial u(N-1)^2}=(B^TP(0)B)^T+R^T
$$

由于 $R$ 是一个正定矩阵，而 $B^TP(0)B$ 是一个半正定矩阵，所以结果是正定的，因此该结果是最小值

将最优的解带入到代价函数中去，可得到最优的代价

$$
J_{N-1\rightarrow N}^* = \frac{1}{2}x(N-1)^T\{(A-BF(N-1))^TP(0)(A-BF(N-1))+F(N-1)^TRF(N-1)+Q\}x(N-1)
$$

令

$$
P(1)=(A-BF(N-1))^TP(0)(A-BF(N-1))+F(N-1)^TRF(N-1)+Q
$$

上式可为

$$
J_{N-1\rightarrow N}^* = \frac{1}{2}x(N-1)^TP(1)x(N-1)
$$

所以最终可以得到

$$
\left\{\begin{aligned}&J^*_{N-k\rightarrow N}=\frac{1}{2}x(N-k)^TP(k)x(N-k)\\&P(k)=(A-BF(N-k))^TP(k-1)(A-BF(N-k))+F(N-k)^TRF(N-k)+Q\\&F(N-k)=(B^TP(k-1)B+R)^{-1}B^TP(k-1)A\end{aligned}\right.
$$

根据递归计算就可以得到所有的状态矩阵了，其中 $F[N-k]$是反馈增益， $u[N-k]$是N-k时的最优控制量，也会得到 $p[k]$进行下一步的计算，最后需要不断递归计算得到 $u[0]$也就是需要使用的输入量。

```matlab
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
```

## LQR线性二次型跟踪器

跟踪二次型系统的LQR的雅各比矩阵为

$$
J=\frac{1}{2} [x(n)-x_d(n))^TS(x(n)-x_d(n)]+\frac{1}{2}\sum_{k=0}^{N-1}{[x(k)-x_d(k))^T\Phi (x(k)-x_d(k)) + u(k)^TRu(k)]}
$$

定义系统误差

$$
e(k)=x(k)-x_d(k)
$$

跟踪器就是对 $e(k)$ 的调节器，也就是 $e_d(k)=0$

可以定义矩阵的系统状态方程

$$
\begin{bmatrix}x(k+1)\\x_d(k+1)\end{bmatrix}=\begin{bmatrix}A&&0\\0&&A_0\end{bmatrix}\begin{bmatrix}x(k)\\x_d(k)\end{bmatrix}+\begin{bmatrix}B\\0\end{bmatrix}u(k)
$$

一般来说跟踪的量是不会改变的，也就是 $A_0=I$

可以用一些符号代表矩阵来简化式子

$$
x_a(k+1)=A_ax_a(k)+B_au(k)
$$

由

$$
e(k)=x(k)-x_d(k)=\begin{bmatrix}I&&-I\end{bmatrix}\begin{bmatrix}x(k)\\x_d(k)\end{bmatrix}=C_ax_a(k)
$$

其中

$$
\left\{\begin{aligned}&A_a=\begin{bmatrix}A&&0\\0&&A_0\end{bmatrix}\\&B_a=\begin{bmatrix}B\\0\end{bmatrix}\\&C_a=\begin{bmatrix}I&&-I\end{bmatrix}\end{aligned}\right.
$$

对应的代价函数为

$$
J=\frac{1}{2} e(N)^TSe(N)+\frac{1}{2}\sum_{k=0}^{N-1}{[e^T(k)Qe(k)+u^T(k)Ru(k)]}
$$

带入上式为

$$
J=\frac{1}{2} x_a(N)^TC_a^TSC_ax_a(N)+\frac{1}{2}\sum_{k=0}^{N-1}{[x_a^T(k)C_a^TQC_ax_a(k)+u^T(k)Ru(k)]}
$$

令 $S_a=C_a^TSC_a$， $Q_a=C_a^TQC_a$，得

$$
J=\frac{1}{2} x_a(N)^TS_ax_a(N)+\frac{1}{2}\sum_{k=0}^{N-1}{[x_a^T(k)Q_ax_a(k)+u^T(k)Ru(k)]}
$$

与LQR调节器求解过程相同，得最终结果为

$$
\left\{\begin{aligned}&J^*_{N-k\rightarrow N}=\frac{1}{2}x_a(N-k)^TP_a(k)x_a(N-k)\\&F_a(N-k)=(B_a^TP_a(k-1)B_a+R)^{-1}B_a^TP_a(k-1)A_a\\&P_a(k)=(A_a-B_aF_a(N-k))^TP_a(k-1)(A_a-B_aF_a(N-k))+F_a(N-k)^TRF_a(N-k)+Q_a\\&u(N-k)=-F_a(N-k)x_a(N-k)\end{aligned}\right.
$$

```matlab
clear all;
close all;
clc;

A = [0 1; -1 -0.5];
n = size(A,1);
B = [0; 1];
p = size(B, 2);

Ts = 0.1;

[A, B]=c2d(A,B,Ts);

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
```

## 总结

$F$ 和 $F_a$ 都是可以提前算好的，实际上的输出结果就是一个 PD 控制器