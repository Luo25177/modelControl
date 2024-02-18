clear all;
clc;
tf = 50; 
Q = 10;
w=sqrt(Q)*randn(1,tf); 
R = 1;
v=sqrt(R)*randn(1,tf);
P =eye(1);
x=zeros(1,tf);
Xnew=zeros(1,tf);
x(1,1)=0.1; 
Xnew(1,1)=x(1,1);
z=zeros(1,tf);
z(1)=x(1,1)^2/20+v(1); 
zjian=zeros(1,tf);
zjian(1,1)=z(1);
linear = 0.5;
for k = 2 : tf 
    % 模拟系统 
    x(:,k) = linear * x(:,k-1) + (2.5 * x(:,k-1) / (1 + x(:,k-1)^2)) + 8 * cos(1.2*(k-1)) + w(k-1); %状态值 
    z(k) = (x(:,k)^2 / 20) + v(k);%观测值
    f=@(x)(linear * x + (2.5 * x / (1 + x^2)) + 8 * cos(1.2*(k-1)));
    h=@(x)(x^2 / 20);
   [Xnew(:,k),P(:,:,k)] = ukf(f,Xnew(:,k-1),P(:,:,k-1),h,z(k),Q,R); 
end
   figure;
   t = 2 : tf;
   plot(t,x(1,t),'b',t,Xnew(1,t),'r:');
   legend('真实值','UKF估计值');
