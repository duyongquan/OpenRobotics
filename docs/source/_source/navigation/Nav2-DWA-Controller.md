# Nav2-DWA-Controller



## 1 介绍

* 概述

  DWA算法（dynamic window approach），其原理主要是在速度空间$（v,w）$中**采样多组速度**，并模拟出这些速度在一定时间内的**运动轨迹**，并通过**评价函数**对这些轨迹进行评价，**选取最优轨迹**对应的$（v,w）$驱动机器人运动。

* **优点**
  - **计算复杂度低**：考虑到速度和加速度的限制，只有安全的轨迹会被考虑，且每次采样的时间较短，因此轨迹空间较小
  - **可以实现避障**：可以实时避障，但是避障效果一般
  - **适用于两轮差分和全向移动模型**

* **缺点**
  - **前瞻性不足**：只模拟并评价了下一步，如在机器人前段遇见“C”字形障碍时，不能很好的避障
  - **动态避障效果差：** 模拟运动轨迹段，动态避障效果差
  - **非全局最优路径：** 每次都选择下一步的最佳路径，而非全局最优路径
  - **不适用于阿克曼模型**

## 2 原理

### 2.1 **机器人运动学模型**

非全向移动机器人 运动学模型：
$$
\begin{bmatrix}
	x(t + \delta t) \\
	y(t + \delta t) \\
	\theta(t + \delta t)
\end{bmatrix}
=
\begin{bmatrix}
	x(t) + v(t)*\cos(\theta(t) * \delta t) \\
	y(t) + v(t)*\sin(\theta(t) * \delta t) \\
	\theta(t) + \omega(t) \delta t)
\end{bmatrix}
$$
  其中： $\omega(t), v(t)$为当前机器人的角速度和线速度。

### 2.2 速度空间

* 移动机器人受自身最大速度最小速度的限制
  $$
  V_s = \{(v, \omega)| v \in [v_{min}, v_{max}] \cap \omega \in [w_{min}, w_{max}] \}
  $$

* 移动机器人受电机性能的影响

$$
V_d = \{(v, \omega)| v \in [v_c - \dot{v}_b \Delta t, v_c + \dot{v}_a \Delta t] \cap
	\omega_c \in [\omega_c - \dot{\omega}_b \Delta t, \omega_c + \dot{\omega}_a \Delta t ] \}
$$

* 移动机器人受障碍的影响

$$
V_a = \{(v, \omega)| v \le {\sqrt{2dist(v, \omega) \dot{v}_b}} \cap
	\omega  \le {\sqrt{2dist(v, \omega) \dot{\omega}_b}} \}
$$

其中：$dist(v，\omega)$对应的轨迹上里障碍物最近的距离。

在上述三条约束条件的限制下，速度空间$（v,w）$会有一定的范围，另外会随着电机的线加速度、角加速度进行变换，速度空间会动态变化，我们将其称为**动态窗口**。在满足约束条件的情况下，进行采样$（v,w）$，可以得到相应的轨迹:

<center class="half"> 
    <img src="./../../_static/navigation/DWA2.png" width="400"/> 
    <img src="./../../_static/navigation/DWA.png" width="330"/>
</center>

### 2.3 评价函数

 每条轨迹的评价得分：
$$
G(v, \omega) = \sigma(\alpha * heading(v, \omega) + \beta * dist(v, \omega) + \gamma * velocity(v, \omega))
$$

*  方位角评价函数

  $heading(v, \omega)$ 代表**航向角**和实际机器人与目标点连线的角度偏差，目的是**纠正机器人航向**，最终能快速到达目标点。

* 障碍物评价函数

  $dist(v, \omega)$代表机器人轨迹上与最近的障碍物的距离，目的是**与障碍物保持距离**。计算时一般还要设置一个最大值，当超过这个距离时，就让dist恒等于这个最大值，**防止距离障碍物越远越好的情况出现**

* 速度评价函数

  $velocity(v, \omega)$代表机器人的速度，目的是让机器人以较快速度到达目标点，同时防止**为了避障陷入速度极小**的情况。计算时一般用速度的绝对值。

* $\alpha, \beta, \gamma$三个参数代表权重，**不同情况的环境**可能需要不同的权重，自行调参。同时为了消除量纲，采取**权重归一化**。

* $\sigma$ 使得三个部分的权重**更加平滑**，使得轨迹与障碍物之间保持一定的间隙。
* 当机器人陷入局部最优时（即不存在路径可以通过），使其**原地旋转**，直到找到可行路径。
* 安全裕度：在路径规划时，设定一安全裕度，即在路径和障碍物之间**保留一定间隙**，且该间隙随着速度增大线性增长

## 3 参考

* https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8472029
* http://vigir.missouri.edu/~gdesouza/Research/Conference_CDs/IEEE_ICRA_2007/data/papers/1765.pdf
* https://blog.csdn.net/weixin_65089713/article/details/123955584?spm=1001.2014.3001.5506