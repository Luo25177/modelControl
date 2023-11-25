% 这个是一个系统函数，其中d表示导数

function d=dxdt(t,x)

d=[x(1) - 3 * x(2); 
  5 * x(1) - 2 * x(2)]; 
end