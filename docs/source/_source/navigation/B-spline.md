## 1  介绍

Bezier曲线有以下几个不足点，所以导致出现了B-spline算法

* 一旦确定特征多边形，就确定了曲线的阶次
* Bezier曲线拼接复杂（需要满足几何连续性，参数连续性等）
* Bezier曲线不能作局部修改（只能整体修改）

B-spline算法是整条曲线用一段一段的曲线连接而成，采用分段连续多段式生成

B-spline的类型划分

* 均匀B样条曲线

  ${0,1,2,3,4,5,6}$： $u_{i+1} - u_i = Constant(常数)$

  ${0,0.2,0.4,0.6,0.8,1}$ ：  $u_{i+1} - u_i = Constant(常数)$

* 准均匀B样条曲线

* 非均匀B样条曲线

* 分段Bezier曲线

## 2 B-spline曲线定义

有n+1个控制点$P_i(i=0,1,...,n)$和一个节点向量$U = [u_0, u_i, \dots, u_m]$，依次连接这些**控制点**可以构成一个特征多边形。k+1阶（k次）B样条曲线的表达式为$（2 \le k \le n+1)$，必须满足**$m=n+k+1$**
$$
P(u) = \sum_{i=0}^{n} P_iB_{i,k}(u), \quad u\in [u_{k-1}， u_{n+1}]其中：
$$

* $P_i$： 特征多边形的顶点
* $B_{i, k}$：称为k阶基函数
* B-spline曲线的定义域为$u ∈ [ u_{k − 1} , u_{n + 1}]$, $u$​为节点向量集合，节点表个数为n + k + 1

B-spline基函数的求出算法应用最广泛的是deBoor-cox递推算法：
$$
\begin{aligned}
    B_{i, k}(u)  &= \frac{u - u_i}{u_{i+k} - u_i} \times B_{i, k-1}(u) + \frac{u_{i + k+1} - u}{u_{i+k+1} - u_{i+1}} \times B_{i+1, k-1}(u)  \\
	
	B_{i, 0}(u) &= 
        \begin{equation}
            \begin{cases}
                1 & \text{  $u_i \lt u \lt u_{i+1}$ } \\
                0 & \text{otherwise}
            \end{cases}
        \end{equation}
\end{aligned}
$$
规定$\frac{0}{0} = 0$

显然，基函数由U定义，其中基函数满足微分方程：
$$
B_{i, k}^{\prime}(t) = \frac{k-1}{t_{i+k-1} - t_i}B_{i，k-1}(t) +  \frac{k-1}{t_{i+k} - t_{i+1}}B_{i+1，k-1}(t)
$$
 

![img](https://img-blog.csdn.net/20180723213149795?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzQwNTk3MzE3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

上图是基函数的运算关系，从左向右，从上往下可以依次计算各个基函数

## 3 性质

* 局部支撑性

  若$u \notin [u_i, u_{i+p+1}]$， 那么$B_{i, p} = 0$

* 非负性

​	  $B_{i, p}(u) \ge 0$

* 规范性
* 凸包包含