# high gain高增益反馈 high frequency高频反馈

对于一个系统

$$
\dot{x}=f(x)+u\\x\rightarrow x_d
$$

其中不清楚 $f(x)$ 的具体表达形式，但是 $f(x)$ 是有界的，并且 $|f(x)|<\rho(x)$

引入误差 $e=x_d-x$ ⇒ $\dot{e}=\dot{x}_d-f(x)-u$，定义李雅普诺夫函数 $V(e)=\frac{1}{2}e^2$

不妨令

$$
u=ke+\dot{x}_d+u_{aux}
$$

其中 $u_{aux}$ 是一个辅助的控制器

**滑膜控制器：** $u_{aux}=\rho(x) \frac{|e|}{e}$，而这个会导致系统不断抖动

**High Gain：** $u_{aux}=\frac{1}{\varepsilon}\rho^2e$ 用足够大的输入抵消不确定性

带入 $\dot{V}$，得

$$
\dot{V}=e\dot{e}=e(\dot{x}_d-f(x)-ke-\dot{x}_d-\frac{1}{\varepsilon}\rho^2e)=-ef(x)-ke^2-\frac{1}{\varepsilon}\rho^2e^2\\\Downarrow\\\dot{V}\leq -ke^2+\rho|e|(1-\frac{1}{\varepsilon}\rho |e|)
$$

此时出现两种情况

case 1:

$$
\rho |e| > \varepsilon \\\Downarrow\\\ 1-\frac{1}{\varepsilon}\rho |e|<0 \\\Downarrow \\\rho|e|(1-\frac{1}{\varepsilon}\rho |e|)<0 \\ \Downarrow \\\dot{V} \leq -ke^2 
$$

case 2:

$$
\rho |e| \leq \varepsilon \\ \Downarrow \\ 0\leq 1-\frac{1}{\varepsilon}\rho |e|\leq 1\\\Downarrow \\\rho|e|(1-\frac{1}{\varepsilon}\rho |e|)\leq \rho|e|\leq \varepsilon \\\Downarrow\\ \dot{V}\leq -ke^2+\varepsilon=-2kV+\varepsilon \\Introduce ~~s(t)>0\\\Downarrow\\\dot{V}+2kV=\varepsilon -s(t)\\解微分方程\\\Downarrow\\V=\frac{1}{2}e^2=\frac{1}{2}e^2_{(0)}exp(-2kt)-exp(-2kt)\int_0^texp(2k\tau)s(\tau)d\tau + \frac{\varepsilon}{2k} (1-exp(-2kt))\\由于第二项总是大于0的\\\Downarrow\\\frac{1}{2}e^2\leq\frac{1}{2}e^2_{(0)}exp(-2kt)+ \frac{\varepsilon}{2k} (1-exp(-2kt))\\两边同时乘2开平方\\\Downarrow\\|e_{(t)}|\leq\sqrt{|e_0|exp(-2kt)+\frac{\varepsilon}{k}-\frac{\varepsilon}{k}exp(-2kt)}\\t\rightarrow \infty\\\Downarrow\\|e_{(t)}|\leq\sqrt{\frac{\varepsilon}{k}}
$$

这就是系统最终会保持一个误差，但是当 $\varepsilon$ 足够小，误差就很小，但是这也导致输入 $u_{aux}=\frac{1}{\varepsilon}\rho^2e$ 会非常大，需要做一个权衡

**High Frequency：** $u_{aux}=\frac{\rho^2e}{\rho |e|+\varepsilon}$ 相当于把滑模控制的抖动幅度减小了

带入 $\dot{V}$ 得

$$
\dot{V}=-ef(x)-ke^2-e\frac{\rho^2e}{\rho |e|+\varepsilon}\leq |e||f(x)|-ke^2-\frac{\rho^2e^2}{\rho |e|+\varepsilon}\\\Downarrow\\\dot{V}\leq-ke^2+|e|\rho(1-\frac{\rho |e|}{\rho |e|+\varepsilon})=-ke^2+\varepsilon\frac{\rho |e|}{\rho |e|+\varepsilon}\\0\leq\frac{\rho |e|}{\rho |e|+\varepsilon}\leq 1\\\Downarrow\\\dot{V}\leq -ke^2+\varepsilon=-2kV+\varepsilon \\Introduce ~~s(t)>0\\\Downarrow\\\dot{V}+2kV=\varepsilon -s(t)\\解微分方程\\\Downarrow\\V=\frac{1}{2}e^2=\frac{1}{2}e^2_{(0)}exp(-2kt)-exp(-2kt)\int_0^texp(2k\tau)s(\tau)d\tau + \frac{\varepsilon}{2k} (1-exp(-2kt))\\由于第二项总是大于0的\\\Downarrow\\\frac{1}{2}e^2\leq\frac{1}{2}e^2_{(0)}exp(-2kt)+ \frac{\varepsilon}{2k} (1-exp(-2kt))\\两边同时乘2开平方\\\Downarrow\\|e_{(t)}|\leq\sqrt{|e_0|exp(-2kt)+\frac{\varepsilon}{k}-\frac{\varepsilon}{k}exp(-2kt)}\\t\rightarrow \infty\\\Downarrow\\|e_{(t)}|\leq\sqrt{\frac{\varepsilon}{k}}
$$

但是这个 $\varepsilon$ 过小时，并不会导致系统的输入过大

### 例子

对于一个系统

$$
\dot{x}=ax^2+u=f(x)+u\\x\rightarrow x_d
$$

在先前是设定 $a$ 是一个常数，但在此控制系统中，认为 $a$ 是一个有界（ $|a|\leq |\overline{a}|$，其中 $\overline{a}$ 已知 ）的数，并非是一个常数， $a$ 是一个 $[-1,1]$ 之间的一个随机数

系统的目标依旧是 $x→x_d$，设定 $e=x_d-x$，得到 $\dot{e}=\dot{x}_d-\dot{x}=\dot{x}_d-f(x)-u$

引入一个函数 $\rho(x)$（实际上可以是函数，也可以是一个固定的上界）使得 $|f(x)|\leq\rho (x)$，设定

- high gain
    
    $$
    u=ke+\dot{x}_d+\frac{1}{\varepsilon}\rho^2e
    $$
    
- high frequency
    
    $$
    u=ke+\dot{x}_d+\frac{\rho^2e}{\rho |e|+\varepsilon}
    $$