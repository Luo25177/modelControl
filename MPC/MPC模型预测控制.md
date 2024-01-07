# MPC模型预测控制

### 最优化控制

****************研究动机****************

在一定的约束条件下达到最优的系统表现，最优是综合分析的最优

**代价函数与评判标准**

- 对于单输入单输出系统控制，e(t)为误差， u(t)为输入
    
    当 $\int_0^te^2dt$ 最小时，就可以保证系统的追踪性很好
    
    当 $\int_1^tu^2dt$ 最小时，可以保证系统的输入最小，能耗最低
    
    控制过程的代价函数 
    
    $$
    J = \int_1^t(qe^2+ru^2)dt
    $$
    
    目的就是设计一个u使J达到最小值
    
    q和r就是我们可以调节的参数，如果 $q>>r$ 那么就是设计过程更加注重误差，如果 $r>>q$的话，就是设计过程更加注重能耗最低
    
- 对于多输入多输出的系统控制中
    
    $\frac{dx}{dt} = Ax +Bu$     x是系统的状态变量
    
    $Y=Cx$       Y就是系统的输出
    
    代价方程就是
    
    $$
    J = \int_0^x(E^TQE + u^TRu)dt
    $$
    
    一个栗子：
    
    $\begin{bmatrix} y_1 \\ y_2\\ \end{bmatrix} = \begin{bmatrix} x_1 \\ x_2 \\ \end{bmatrix}$
    
    $R = \begin{bmatrix}r_1\\r_2\\ \end{bmatrix} = \begin{bmatrix}0\\0\\ \end{bmatrix}$
    
    $⇒E = \begin{bmatrix}e_1\\e_2\\\end{bmatrix} = \begin{bmatrix}y_1-r_1\\y_2-r_2\\\end{bmatrix} = \begin{bmatrix}x_1\\x_2\\\end{bmatrix}$
    
    $⇒E^TQE=\begin{bmatrix}x_1&x_2\\\end{bmatrix}\begin{bmatrix}q_1&0\\0&q_2\\\end{bmatrix}\begin{bmatrix}x_1\\x_2\\\end{bmatrix} = q_1x_1^2+q_2x_2^2$
    
    $⇒u^TRu = r_1u_1^2+r_2u_2^2$
    
    Q和R都是调节矩阵， $q_1,q_2,r_1,r_2$都是权重系数，如果过多的关注 $x_1$的话，可以适当的将 $q_1,r_1$加大，加重权重系数
    

### MPC

通过模型来预测系统在某一时间段内的表现来进行最优化控制，多用于数位控制，多采用离散型状态空间的表达形式  $X_{k+1} = AX_k+Bu_k$

****************分析步骤****************

在当前时刻，即k时刻

1. 测量/估计当前状态，输出为 $Y_k$
2. 基于 $u_k,u_{k+1},….u_{k+N}$ 来进行最优化， N为预测出空间，N-1为控制范围
    
    代价函数 
    
    $$
    X_k=\begin{bmatrix}x(k+1|k)\\x(k+2|k)\\...\\x(k+N|k)\end{bmatrix}\\U_k=\begin{bmatrix}u(k|k)\\u(k+1|k)\\...\\u(k+N-1|k)\end{bmatrix}\\output:Y=X\\reference:R\\E=Y-R=X-R
    $$
    
    $$
    J = ∑_k^{N-1}E_k^TQE_k+U_k^TRU_k+E_N^TFE_N
    $$
    
    其中 $E_N^TFE_N$ 代表了最终的结果，就是期望值，也就是最末一点，终端误差，预测区间最后时刻的误差的代价函数，目的就是找到 $J$ 的最小值
    
    并且 Q 与 R 都是对角矩阵
    
3. 需要找到 $u_k,u_{k+1},….u_{k+N}$ 并非要使用所有的 $u_k$，只实施第一个也就是 $u_k$，当时间到达k+1的时候，就要把窗口，也就是预测区间向右移动，每一步都需要去求解一个最优化问题——滚动优化问题

mpc在预测的过程中会考虑到系统的约束

### 二次规划

******************一般形式******************

$$
min\ Z^TQZ+C^TZ
$$

当Q为对角矩阵时，这个式子为

$$
\sum_{i=1}^n q_iz_i^2
$$

就变成了一个最小二乘的形式

对于一个系统状态方程

$$
x(k+1)=Ax(k)+Bu(k)\\state:x=\begin{bmatrix}x_1\\x_2\\...\\x_n\end{bmatrix}\\input:u=\begin{bmatrix}u_1\\u_2\\...\\u_n\end{bmatrix}
$$

利用增广矩阵来表示k时刻的预测的结果

$$
x(k+1|k)~~~u(k|k)\\x(k+2|k)~~~u(k+1|k)\\...\\x(k+N|k)~~~u(k+N-1|k)
$$

其中 N 表示预测的区间

定义

$$
X_k=\begin{bmatrix}x(k|k)\\x(k+1|k)\\...\\x(k+N|k)\end{bmatrix}\\U_k=\begin{bmatrix}u(k|k)\\u(k+1|k)\\...\\u(k+N-1|k)\end{bmatrix}\\output:Y=X\\reference:R=0\\E=Y-R=X-0=X
$$

代价函数为

$$
J=∑_{i=0}^{N-1} \{x(k+i|k)^TQx(k+i|k)+u(k+i|k)^TRu(k+i|k)\} + x(k+N)^TFx(k+N)
$$

其中

- $X(k+i|k)^TQX(k+i|k)$ 误差加权和
- $u(k+i|k)^TRu(k+i|k)$ 输入加权和
- $x(k+N)^TFx(k+N)$ 终端误差

在k时刻时

$$
x(k|k)=x_k~~~该时刻的初始条件\\x(k+1|k)=Ax(k|k)+Bu(k|k)=Ax_k+Bu(k|k)\\x(k+2|k)=Ax(k+1|k)+Bu(k+1|k)=A^2x(k|k)+ABu(k|k)+Bu(k+1|k)\\x(k+N|k)=A^Nx_k+\sum_{i=0}^{N-1}{A^{N-1-i}Bu(k+i|k)}
$$

根据之前定义简化方程

$$
X_k=\begin{bmatrix}I\\A\\A^2\\...\\A^N\end{bmatrix}x_k+\begin{bmatrix}0&0&...&0\\B&0&...&0\\AB&B&...&0\\...\\A^{N-1}B&A^{N-2}B&...&B\end{bmatrix}U_k
$$

可令

$$
M=\begin{bmatrix}I\\A\\A^2\\...\\A^N\end{bmatrix}\\C=\begin{bmatrix}0&0&...&0\\B&0&...&0\\AB&B&...&0\\...\\A^{N-1}B&A^{N-2}B&...&B\end{bmatrix}
$$

其中 $M$ 为 $(N+1)\times 1$， $C$ 为 $(N+1)\times N$ 矩阵

则简化为

$$
X_k=Mx_k+CU_k
$$

代价函数为

$$
J=∑_{i=0}^{N-1} \{x(k+i|k)^TQx(k+i|k)+u(k+i|k)^TRu(k+i|k)\} + x(k+N)^TFx(k+N)
$$

其中第一项与最后一项为

$$
x(k|k)^TQx(k|k)+...+x(k+N-1|k)^TQx(k+N-1|k)+x(k+N|k)^TQx(k+N|k)\\\Rightarrow \begin{bmatrix}x(k|k)\\x(k+1|k)\\...\\x(k+N|k)\end{bmatrix}^T\begin{bmatrix}Q&0&...&0\\0&Q&...&0\\...\\0&0&...&F\end{bmatrix}\begin{bmatrix}x(k|k)\\x(k+1|k)\\...\\x(k+N|k)\end{bmatrix}
$$

令

$$
\overline{Q}=\begin{bmatrix}Q&0&...&0\\0&Q&...&0\\...\\0&0&...&F\end{bmatrix}
$$

则上式为

$$
X_k^T\overline{Q}X_k
$$

中间一项为

$$
u(k|k)^TRu(k|k)+...+u(k+N-1|k)^TRu(k+N-1|k)\\\Rightarrow \begin{bmatrix}u(k|k)\\u(k+1|k)\\...\\u(k+N-1|k)\end{bmatrix}^T\begin{bmatrix}R&0&...&0\\0&R&...&0\\...\\0&0&...&R\end{bmatrix}\begin{bmatrix}u(k|k)\\u(k+1|k)\\...\\u(k+N-1|k)\end{bmatrix}
$$

令

$$
\overline{R}=\begin{bmatrix}R&0&...&0\\0&R&...&0\\...\\0&0&...&R\end{bmatrix}
$$

上式化简为

$$
U_k^T\overline{R}U_k
$$

故而代价函数为

$$
J=X_k^T\overline{Q}X_k+U_k^T\overline{R}U_k
$$

由于

$$
X_k=Mx_k+CU_k
$$

则

$$
J=(Mx_k+CU_k)^T\overline{Q}(Mx_k+CU_k)+U_k^T\overline{R}U_k
$$

分开之后为

$$
J=x_k^TM^T\overline{Q}Mx_k+x_k^TM^T\overline{Q}CU_k+U_k^TC^T\overline{Q}Mx_k+U_k^TC^T\overline{Q}CU_k+U_k^T\overline{R}U_k
$$

对于第2项和第3项，它们互为转置，并且 $\overline{Q}$ 是对称的，所以可以简化合并为

$$
2x_k^TM^T\overline{Q}CU_k
$$

对于第4项和第5项，也可以化简为

$$
U_k^T(C^T\overline{Q}C+\overline{R})U_k
$$

可令

$$
G=M^T\overline{Q}M\\E=M^T\overline{Q}C\\H=C^T\overline{Q}C+\overline{R}
$$

最终，代价函数写作

$$
J=x_k^TGx_k+2x_k^TEU_k+U_k^THU_k
$$

求解最优解，先求导数为0处，即

$$
\frac{\partial J}{\partial U_k}=0
$$

$$
first~part:0\\second~part:2E^Tx_k\\second~part:(H+H^T)U_k
$$

即

$$
\frac{\partial J}{\partial U_k}=2E^Tx_k+(H+H^T)U_k=0
$$

求得

$$
U_k=-2(H+H^T)^{-1}E^Tx_k
$$

验证是否为极小值

$$
\frac{\partial^2 J}{\partial U_k^2}=(H+H^T)^T=H^T+H
$$

由于H中的 $\overline{Q}$, $C$, $\overline{R}$ 都为正定阵，所以结果一定是正定的，故得证

```matlab
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
```

```matlab
function  [E , H]=MPC_Matrices(A,B,Q,R,F,N)
    n=size(A,1);   % A 是 n x n 矩阵, 得到 n
    p=size(B,2);   % B 是 n x p 矩阵, 得到 p
    %%%%%%%%%%%%
    M=[eye(n);zeros(N*n,n)]; % 初始化 M 矩阵. M 矩阵是 (N+1)n x n的， 
                             % 它上面是 n x n 个 "I", 这一步先把下半部
                             % 分写成 0 
    C=zeros((N+1)*n,N*p); % 初始化 C 矩阵, 这一步令它有 (N+1)n x NP 个 0
    % 定义M 和 C 
    tmp=eye(n);  %定义一个n x n 的 I 矩阵
    %　更新Ｍ和C
    for i=1:N % 循环，i 从 1到 N
        rows =i*n+(1:n); %定义当前行数，从i x n开始，共n行 
        C(rows,:)=[tmp*B,C(rows-n, 1:end-p)]; %将c矩阵填满
        tmp= A*tmp; %每一次将tmp左乘一次A
        M(rows,:)=tmp; %将M矩阵写满
    end
    % 定义Q_bar和R_bar
    Q_bar = kron(eye(N),Q);
    Q_bar = blkdiag(Q_bar,F);
    R_bar = kron(eye(N),R);
    % 计算G, E, H
    G=M'*Q_bar*M; % G: n x n
    E=C'*Q_bar*M; % E: NP x n
    H=C'*Q_bar*C+R_bar; % NP x NP
end
```

```matlab
function u_k= Prediction(x_k,E,H,N,p)
    U_k = zeros(N*p,1); % NP x 1
    U_k = quadprog(H,E*x_k); % MPC预测控制函数
    u_k = U_k(1:p,1); % 取第一个结果
end
```