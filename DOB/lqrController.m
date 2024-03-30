A = [0 1; -1 -1];
B = [0; 1];
C = [1 0; 0 1];
D = 0;

Q = diag([10 1]);
R = 0.5;
sys = ss(A, B, C, D);

K = lqr(sys, Q, R)

