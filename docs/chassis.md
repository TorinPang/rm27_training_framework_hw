# 学习记录

## 全向轮与麦克纳姆轮

### 组成

由轮毂和辊子组成。

- 轮毂：轮子的主体框架
- 辊子：安装在轮毂上的鼓状物

全向轮与麦克纳姆轮的区别

- 全向轮：轮毂轴与辊子转轴相互垂直
- 麦克纳姆轮：轮毂轴与辊子转轴成45°角

### 麦克纳姆轮的安装

麦克纳姆轮一般是四个一组使用，两个左旋轮，两个右旋轮，左旋轮和右旋轮成手性对称

安装方式有多种，主要分为：X-正方形（X-square）、X-长方形（X-rectangle）、O-正方形（O-square）、O-长方形（O-rectangle）。其中 X 和 O 表示的是与四个轮子地面接触的辊子所形成的图形；正方形与长方形指的是四个轮子与地面接触点所围成的形状。

![麦克纳姆轮的四种安装方式](https://pic4.zhimg.com/710bfe1d35b5d2f82689965087b33681_1440w.jpg)

最常见的安装方式为**O-长方形**。

### 麦克纳姆轮的底盘解算

我们以**O-长方形**为例，记左前轮为FL(front left)，右前轮FR(front right)，左后轮RL(rear left)，右后轮RR(rear right)，记向右为x轴正方向，向前为y轴正方向，逆时针为角速度正方向（即垂直平面向里）。

则FL和RR轮子方向均为右上-左下，FR和RL的轮子均为左上-右下。我们以车向y轴正反向移动时四个轮子的旋转方向为四个轮子的旋转方向的正方向。

记前后轮的距离为 $2l_y$, 左右轮的距离为$2l_x$，即四个轮子的位置依次为 FL:$\mathbf{R}_{FL}=(-l_x,l_y)$, FR:$\mathbf{R}_{FR}=(l_x,l_y)$, RL:$\mathbf{R}_{RL}=(-l_x,-l_y)$, RR:$\mathbf{R}_{RR}=(l_x,-l_y)$。

记车的速度为$\mathbf{v}$，在x轴方向上的速度为$v_x$, 在y轴方向上的速度为$v_y$，角速度为$\omega$，左前轮轴心位置的速度为$\mathbf{v}_{FR}$，在x轴方向上的速度为 $v_{x_{FL}}$，y轴方向上的速度为 $v_{y_{FL}}$，则对左前轮，我们就有 $\mathbf{v}_{FL}=\mathbf{v}+\mathbf{\omega} \times \mathbf{R}_{FL}$，展开，有 $v_{x_{FL}}=v_x-\omega \cdot l_y, v_{x_{FL}}=v_y-\omega\cdot l_x$，在辊子所在的方向上，$v_{x_{FL}}$为正时辊子速度方向为正，$v_{y_{FL}}$为正时辊子速度方向也为正，那么实际上辊子的速度 $v_{FL}' = (v_{x_{FL}}+v_{y_{FL}})\cdot\cos 45^\circ=\frac{v_x+v_y-\omega(l_x+l_y)}{\sqrt{2}}$，由于辊子与轮毂轴之间的夹角为 $45^\circ$，则$v_{FL}=\frac{v_{FL}'}{\cos 45^\circ}=v_y+v_x-\omega(l_x+l_y)$。

同理，我们可以得到$v_{FR}=v_y-v_x+\omega(l_x+l_y)$，$v_{RL}=v_y-v_x-\omega(l_x+l_y)$，$v_{RR}=v_y+v_x+\omega(l_x+l_y)$。记轮子半径为$r$，$l=l_x+l_y$，则由公式$v=\omega\cdot r$可以得到如下结论

$$
\begin{pmatrix}
\omega_{FL} \\
\omega_{FR} \\
\omega_{RL} \\
\omega_{RR} \\
\end{pmatrix}
=
\frac{1}{r}
\begin{pmatrix}
1 & 1 & -1 \\
1 & -1 & 1 \\
1 & -1 & -1 \\
1 & 1 & 1 \\
\end{pmatrix}
\cdot
\begin{pmatrix}
v_y \\
v_x \\
\omega\cdot l
\end{pmatrix}
$$
