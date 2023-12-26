%% 提前算出所有的 F
[A B C D] = tf2ss([1], [1 2 1])
Q = [1 0; 0 1];
R = 0.01;
sys = ss(A, B, C, D);
K = lqr(sys, Q, R)