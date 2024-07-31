close all
clear
clc

K_P = 0;
K_R = 1;
W_C = 1;
W_0 = 100;

KR1 = tf([K_P, 2 * K_R * W_C + 2 * K_P * W_C, K_P * W_0 * W_0], [1, 2 * W_C, W_0 * W_0]);
K_R = 10;
KR10 = tf([K_P, 2 * K_R * W_C + 2 * K_P * W_C, K_P * W_0 * W_0], [1, 2 * W_C, W_0 * W_0]);

K_R = 100;
KR100 = tf([K_P, 2 * K_R * W_C + 2 * K_P * W_C, K_P * W_0 * W_0], [1, 2 * W_C, W_0 * W_0]);

K_R = 1000;
KR1000 = tf([K_P, 2 * K_R * W_C + 2 * K_P * W_C, K_P * W_0 * W_0], [1, 2 * W_C, W_0 * W_0]);

hold on
bode(KR1);
bode(KR10);
bode(KR100);
bode(KR1000);

besself