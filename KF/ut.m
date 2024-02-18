function [y,Y,P,Y1] = ut(f,X,Wm,Wc,n,R)  %f:非线性函数，X: sigma点 Wm:均值权值 Wc:方差权值
L = size(X,2);  
y = zeros(n,1);  
Y = zeros(n,L);  
for k=1:L  
    Y(:,k) = f(X(:,k));  %非线性传递后结果 r
    y = y+Wm(k)*Y(:,k);  %均值
end  
Y1 = Y-y(:,ones(1,L));  
P = Y1*diag(Wc)*Y1'+R;    %协方差
