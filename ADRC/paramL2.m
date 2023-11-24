% 传递函数 Gs=(b1) / (a1 * s ^ 2 + a2 * s + s3)

b1 = 1;
a1 = 1;
a2 = 1;
a3 = 1;

b0 = b1 / a1;

wc = 5; % 只调节这一个参数就可以

wo = wc * 5;
kp = wc * wc;
kd = 2 * wc;
beta21 = 3 * wo;
beta22 = 3 * wo ^ 2;
beta23 = wo ^ 3;