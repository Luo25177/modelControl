% 传递函数 G(s) = b1 / (a1 * s + a2)

a1 = 3;
a2 = 1;
b1 = 5;

wc = 10; % 调节这一个参数

wo = 5 * wc;
b0 = b1 / a1;
beta1 = wo * 2;
beta2 = wo * wo;
