%% 状态反馈 Hinfinty 控制
clc;
clear;

%% 系统建立
A = [0 1; 1 0];
syssize = size(A, 1);
B_1 = [0; 1];
B_2 = [1; 1];
C_1 = [1 0];
D_11 = 0;
D_12 = 0;
C_2 = [1 0];
D_21 = 0;
D_22 = 0;

%% 状态反馈 Hinfinty 控制器
setlmis([]);
X = lmivar(1, [syssize 1]);
W = lmivar(2, [1 syssize]);
lmiterm([1 1 1 X], A, 1, 's'); % AX+(AX)'
lmiterm([1 1 1 W], B_2, 1, 's'); %B_2W+(B_2W)'
lmiterm([1 2 1 0], B_1'); % B1'
lmiterm([1 2 2 0], -1); % -I
lmiterm([1 3 1 X], C_1, 1); % C1X
lmiterm([1 3 1 W], D_12, 1); % D12W
lmiterm([1 3 2 0], D_11); % D11
lmiterm([1 3 3 0], -1); % -I
lmiterm([-2 1 1 X], 1, 1); % X>0	特别注意不能漏掉

lmisys = getlmis;
[tmin, xfeas] = feasp(lmisys);
XX1 = dec2mat(lmisys,xfeas,X);
WW1 = dec2mat(lmisys,xfeas,W);
K1 = WW1 * inv(XX1);

%% 状态反馈次优gamma-Hinfinty 控制器
gamma = 2;
setlmis([]);
X = lmivar(1, [syssize 1]);
W = lmivar(2, [1 syssize]);
lmiterm([1 1 1 X], A, 1, 's'); % AX+(AX)'
lmiterm([1 1 1 W], B_2, 1, 's'); %B_2W+(B_2W)'
lmiterm([1 2 1 0], B_1'); % B1'
lmiterm([1 2 2 0], -1); % -I
lmiterm([1 3 1 X], C_1, 1); % C1X
lmiterm([1 3 1 W], D_12, 1); % D12W
lmiterm([1 3 2 0], D_11); % D11
lmiterm([1 3 3 0], -1 * gamma ^ 2); % -γ^2I
lmiterm([-2 1 1 X], 1, 1); % X>0	特别注意不能漏掉

lmisys = getlmis;
[tmin, xfeas] = feasp(lmisys);
XX2 = dec2mat(lmisys,xfeas,X);
WW2 = dec2mat(lmisys,xfeas,W);
K2 = WW2 * inv(XX2);

%% 状态反馈最优 Hinfinty 控制器
setlmis([]);
X = lmivar(1, [syssize 1]);
W = lmivar(2, [1 syssize]);
rho = lmivar(1, [1 1]);
lmiterm([1 1 1 X], A, 1, 's'); % AX+(AX)'
lmiterm([1 1 1 W], B_2, 1, 's'); %B_2W+(B_2W)'
lmiterm([1 2 1 0], B_1'); % B1'
lmiterm([1 2 2 0], -1); % -I
lmiterm([1 3 1 X], C_1, 1); % C1X
lmiterm([1 3 1 W], D_12, 1); % D12W
lmiterm([1 3 2 0], D_11); % D11
lmiterm([1 3 3 rho], -1, 1); % -ρI
lmiterm([-2 1 1 X], 1, 1); % X>0	特别注意不能漏掉
lmisys = getlmis;

n = decnbr(lmisys); % 系统决策变量个数
c = zeros(n, 1); % 确定向量c的维数
for j = 1 : n
     [r1j] = defcx(lmisys, j, rho);
      c(j) = trace(r1j);
end
% c = mat2dec(lmisys,zeros(4,4),zeros(1,4),eye(1))
[copt, xopt] = mincx(lmisys,c);
XX3 = dec2mat(lmisys, xopt, X);
WW3 = dec2mat(lmisys, xopt, W);
K3 = WW3 * inv(XX3);
rhoo=dec2mat(lmisys, xopt, rho);
