close all
clc
clear

fs = 10;
Wp = 2 / fs;
Ws = 5 / fs;
Rp = 1;
Rs = 10;

disp("butter");
[n,Wn] = buttord(Wp,Ws,Rp,Rs);
[b,a] = butter(n, Wn, 'low');
sys = tf(b, a);
sys_z = c2d(sys, 0.001, 'tustin')

disp("besself");
[b,a] = besself(n,Wn,'low');
sys = tf(b, a);
sys_z = c2d(sys, 0.001, 'tustin')

disp("cheby1");
[n,Wn] = cheb1ord(Wp,Ws,Rp,Rs);
[b,a] = cheby1(n,Rp,Wn,'low');
sys = tf(b, a);
sys_z = c2d(sys, 0.001, 'tustin')

disp("cheby2");
[n,Wn] = cheb2ord(Wp,Ws,Rp,Rs);
[b,a] = cheby2(n,Rs,Wn,'low');
sys = tf(b, a);
sys_z = c2d(sys, 0.001, 'tustin')

disp("ellip");
[n,Wn] = ellipord(Wp,Ws,Rp,Rs);
[b,a] = ellip(n,Rp,Rs,Wn,'low');
sys = tf(b, a);
sys_z = c2d(sys, 0.001, 'tustin')

