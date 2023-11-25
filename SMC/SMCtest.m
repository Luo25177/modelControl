 
clear all;
close all;
clc;
 
t=10;
ts=0.01;%ts代表时间间隔
%控制器参数
c=5;
k1=3;%%%参数符号和循环结构里循环符号要区分开来。
D=0.5;
%控制对象初始状态
x(1)=1;
dx(1)=0;
 
%系统参数
b=10;
 
for k=1:1:t/ts+1
  if k==1
    K=(k-1);
    times(k)=K*ts;%序号k代表K*ts时刻
  
    xd(k)=sin(K*ts);
    dxd(k)=cos(K*ts);
    e(k)=x(k)-xd(k);
    ef(k)=0;
    else 
    K=(k-1);
    times(k)=K*ts;%序号k代表K*ts时刻
    
    
    xd(k)=sin(K*ts);
    dxd(k)=cos(K*ts);
    
    e(k)=x(k)-xd(k);
    ef(k)=ef(k-1)+(e(k)+e(k-1))*ts/2;
    end
    
    s(k)=e(k)+c*ef(k);
    ut(k)=1/b*(-c*e(k)+dxd(k)-k1*s(k)-D*sign(s(k)));
    dt(k)=0.5*sin(K*ts);%状态干扰
    dx(k)=(b*ut(k)+dt(k));
    x(k+1)=x(k)+dx(k)*ts;
end
x(:,end)=[];
figure(1)
plot(times,x,times,xd);
figure(2)
plot(times,ut);
figure(3)
 
plot(ef,e,ef,-c*ef);
 