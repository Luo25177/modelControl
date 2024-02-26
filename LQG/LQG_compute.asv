A = [0 1; 1 0];
B = [0; 1];
C = [1 0; 0 0];
D = 0;

lqrsys = ss(A, B, C, D);
G = tf(lqrsys);
p = size(C, 1);
[n, m] = size(B);

Q = [1 0; 0 1];
R = 1;
Klqr = lqr(A, B, Q, R);
