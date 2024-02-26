clear
clc;

A = [0 1; 1 0];
B_1 = [0.1; 0.5];
B_2 = [0; 1];
C_1 = [1 0];
D_11 = 0;
D_12 = 0;
C_2 = [1 0];
D_21 = 1;
D_22 = 0;

P = ltisys(A, [B_1, B_2], [C_1; C_2], [D_11, D_12; D_21, D_22]);
[gopt, K] = hinflmi(P, [1, 1], 10);
[A_k, B_k, C_k, D_k] = ltiss(K)
