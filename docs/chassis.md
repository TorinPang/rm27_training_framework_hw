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

记车的速度为$\mathbf{v}$，在x轴方向上的速度为$v_x$, 在y轴方向上的速度为$v_y$，角速度为$\omega$，左前轮轴心位置的速度为$\mathbf{v}_{FR}$，在x轴方向上的速度为 $v_{x_{FL}}$，y轴方向上的速度为 $v_{y_{FL}}$，则对左前轮，我们就有 $\mathbf{v}_{FL}=\mathbf{v}+\mathbf{\omega} \times \mathbf{R}_{FL}$，展开，有 $v_{x_{FL}}=v_x-\omega \cdot l_y, v_{y_{FL}}=v_y-\omega\cdot l_x$，在辊子所在的方向上，$v_{x_{FL}}$为正时辊子速度方向为正，$v_{y_{FL}}$为正时辊子速度方向也为正，那么实际上辊子的速度 $v_{FL}' = (v_{x_{FL}}+v_{y_{FL}})\cdot\cos 45^\circ=\frac{v_x+v_y-\omega(l_x+l_y)}{\sqrt{2}}$，由于辊子与轮毂轴之间的夹角为 $45^\circ$，则$v_{FL}=\frac{v_{FL}'}{\cos 45^\circ}=v_y+v_x-\omega(l_x+l_y)$。

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

### 全向轮的底盘解算

#### 辊子是如何带来全向移动的

麦克纳姆轮把辊子斜置 $45^\circ$，驱动力沿辊子方向分解；全向轮的辊子**与轮毂轴垂直**（轮毂轴水平、辊子转轴也水平且垂直于轮毂轴），辊子列方向的滑动是完全自由的。于是每个全向轮只提供**一条约束**：轮心速度在自己的**滚动方向** $\mathbf{u}_i$（水平面内，垂直于轮轴）上的投影必须由轮子转动提供；而垂直于 $\mathbf{u}_i$ 的分量被辊子"滑掉"，既不约束底盘、也不产生力。三条约束互相独立，就撑起了 $(v_x,v_y,\omega)$ 三个自由度，这就是全向移动的来源。

沿用上面麦轮的坐标约定（向右为 $x$ 正、向前为 $y$ 正、逆时针为 $\omega$ 正），记第 $i$ 个轮子的轮心位置 $\mathbf{R}_i=(x_i,y_i)$，则轮心速度为 $\mathbf{v}_i=\mathbf{v}+\boldsymbol\omega\times\mathbf{R}_i=(v_x-\omega y_i,\ v_y+\omega x_i)$，投影后得

$$ v_i=\mathbf{v}_i\cdot\mathbf{u}_i=u_{ix}v_x+u_{iy}v_y+\omega\,(\mathbf{R}_i\times\mathbf{u}_i)_z,\qquad \omega_i=\frac{v_i}{r} $$

其中 $(\mathbf{R}_i\times\mathbf{u}_i)_z=x_iu_{iy}-y_iu_{ix}$，$r$ 为轮子半径。把 $N$ 个轮子摞起来写成矩阵：

$$
\begin{pmatrix}
\omega_1 \\
\vdots \\
\omega_N
\end{pmatrix}
=
\frac{1}{r}
\begin{pmatrix}
u_{1x} & u_{1y} & x_1u_{1y}-y_1u_{1x} \\
\vdots & \vdots & \vdots \\
u_{Nx} & u_{Ny} & x_Nu_{Ny}-y_Nu_{Nx}
\end{pmatrix}
\cdot
\begin{pmatrix}
v_x \\
v_y \\
\omega
\end{pmatrix}
$$

**所有底盘解算都是这一张表**，区别只在每个轮子的 $\mathbf{u}_i$ 怎么取。验证一下：把麦轮 FL 的 $\mathbf{u}_{FL}=(1,1)$（辊子方向，即"虚拟轮的滚动方向"）和 $\mathbf{R}_{FL}=(-l_x,l_y)$ 代进去，第三列得 $(-l_x)\cdot 1-l_y\cdot 1=-(l_x+l_y)$，正是上面麦轮矩阵第一行第三列的元素 $-l$；四个轮子依次取 $\mathbf{u}_i=(1,1),(-1,1),(-1,1),(1,1)$，前两列也就得到矩阵里的 $\pm1$ —— 也就是说，**麦轮的解算本质上就是"把每个麦轮看成一个滚动方向沿辊子方向的全向轮"**（这也正是前面那句"FL/RR 沿右上-左下、FR/RL 沿左上-右下"的几何含义），辊子把轮子自身平面内的速度吃掉一半（$\cos45^\circ$ 那一项），所以麦轮的等效 $\mathbf{u}_i$ 是斜 $45^\circ$ 的对角方向。

#### 三轮全向（$120^\circ$）

三个轮子互成 $120^\circ$，取第 1 个轮子在正前方，安装角（轮心方位角）$\alpha_1=90^\circ,\ \alpha_2=210^\circ,\ \alpha_3=330^\circ$，轮心位置 $\mathbf{R}_i=L(\cos\alpha_i,\sin\alpha_i)$，$L$ 为轮心到中心的距离。实际安装时轮轴沿半径方向，所以滚动方向是**切向**：

$$\mathbf{u}_i=(-\sin\alpha_i,\ \cos\alpha_i)$$

（即"轮子正转把底盘往逆时针切向推"，正方向由我们自己定义，与后面舵轮的角度正方向保持一致即可。）代入上式，注意 $(\mathbf{R}_i\times\mathbf{u}_i)_z=L\cos^2\alpha_i+L\sin^2\alpha_i=L$ 对三个轮子恒成立，于是

$$\omega_1=\frac{-v_x+\omega L}{r},\qquad
\omega_2=\frac{\frac{1}{2}v_x-\frac{\sqrt3}{2}v_y+\omega L}{r},\qquad
\omega_3=\frac{\frac{1}{2}v_x+\frac{\sqrt3}{2}v_y+\omega L}{r}$$

$$
\begin{pmatrix}
\omega_1 \\
\omega_2 \\
\omega_3
\end{pmatrix}
=
\frac{1}{r}
\begin{pmatrix}
-1 & 0 & L \\
\frac{1}{2} & -\frac{\sqrt3}{2} & L \\
\frac{1}{2} & \frac{\sqrt3}{2} & L
\end{pmatrix}
\cdot
\begin{pmatrix}
v_x \\
v_y \\
\omega
\end{pmatrix}
$$

行列式 $=\frac{3\sqrt3}{2}L\ne0$，三轮系统是"恰好可解"的（3 个方程对 3 个未知量），求逆得**正解**（由三个轮子的转速反推底盘速度，即里程计）：

$$ v_x=\frac{r}{3}\left(-2\omega_1+\omega_2+\omega_3\right),\qquad
v_y=\frac{r}{\sqrt3}\left(\omega_3-\omega_2\right),\qquad
\omega=\frac{r}{3L}\left(\omega_1+\omega_2+\omega_3\right) $$

验算两种极端工况（$\omega_i$ 指轮子角速度）：

- 纯平移 $v_x$：$\omega_1=-\frac{v_x}{r},\ \omega_2=\omega_3=\frac{v_x}{2r}$ → 代回得 $v_x$、$v_y=0$、$\omega=0$ ✔
- 纯自转 $\omega$：三轮同速同向 $\omega_i=\frac{\omega L}{r}$ → 代回得 $v_x=v_y=0$、$\omega$ 不变 ✔

反过来看也有物理直觉：正前方那个轮子的滚动方向是左右方向，所以**底盘向前平移时它不转**（$v_1$ 里没有 $v_y$ 项），左右平移时它转得最卖力（$v_1=-v_x$）；三个轮子各自"只管一个方向 ＋ 贡献自转"，正是三轮全向能吃满三个自由度的原因。

#### 四轮全向

四个轮子按 $45^\circ/135^\circ/225^\circ/315^\circ$ 对称布置、同样切向驱动时，$\mathbf{u}_i$ 依次为 $\frac{1}{\sqrt2}(-1,1),\frac{1}{\sqrt2}(-1,-1),\frac{1}{\sqrt2}(1,-1),\frac{1}{\sqrt2}(1,1)$，代进同一张表：

$$
\begin{pmatrix}
\omega_1 \\ \omega_2 \\ \omega_3 \\ \omega_4
\end{pmatrix}
=
\frac{1}{r}
\begin{pmatrix}
-\frac{\sqrt2}{2} & \frac{\sqrt2}{2} & L \\
-\frac{\sqrt2}{2} & -\frac{\sqrt2}{2} & L \\
\frac{\sqrt2}{2} & -\frac{\sqrt2}{2} & L \\
\frac{\sqrt2}{2} & \frac{\sqrt2}{2} & L
\end{pmatrix}
\cdot
\begin{pmatrix}
v_x \\ v_y \\ \omega
\end{pmatrix}
$$

轮子数（4）多于自由度数（3），矩阵不再是方阵，**只能求最小二乘解（伪逆）**。这里三列互相正交、模长平方分别为 $2,2,4L^2$，正好可以解析求逆：

$$ v_x=\frac{\sqrt2 r}{4}\left(-\omega_1-\omega_2+\omega_3+\omega_4\right),\qquad
v_y=\frac{\sqrt2 r}{4}\left(\omega_1-\omega_2-\omega_3+\omega_4\right),\qquad
\omega=\frac{r}{4L}\left(\omega_1+\omega_2+\omega_3+\omega_4\right) $$

也就是说：$v_x,v_y$ 取各轮的"切向投影之和"，$\omega$ 取四轮转速的平均（和三轮一样，自转那一列全是 $L$，本身就是正交的）。轮子多于 3 个的好处是**过约束**：某一轮打滑或失效时，用伪逆仍能得到误差最小的估计，不会像三轮那样直接解错。

## 舵轮

### 组成与原理

每个舵轮轮组有两个电机，绕**竖直轴**配合工作：

- 舵向电机：控制轮子的**朝向角** $\delta_i$（轮子平面指向哪里）
- 舵轮电机：控制轮子的**转速** $n_i$

轮子是普通橡胶轮，**没有辊子**，所以轮子与地面之间不能有侧向滑动 —— 轮心速度的方向必须恰好等于轮子朝向，大小必须等于 $n_ir$。对比一下三者的约束强度：麦轮/全向轮只约束轮心速度在一个方向上的投影（1 条约束），舵轮把**方向和大小同时钉死**（2 条约束）。这就是为什么麦轮、全向轮解出来只是一组"投影和"，而舵轮要解出**角度 + 速度**两个量；也是舵轮能实现真正无滑移全向移动的原因。

### 逆解（$v_x,v_y,\omega$ → 舵向角 + 轮速）

轮心位置仍记为 $\mathbf{R}_i=(x_i,y_i)$，轮心速度与前面完全一样：

$$ \mathbf{v}_i=\mathbf{v}+\boldsymbol\omega\times\mathbf{R}_i=(v_x-\omega y_i,\ v_y+\omega x_i) $$

轮子只能沿自己朝向运动，所以直接把 $\mathbf{v}_i$ 分解成"朝 + 大小"即可：

$$ \delta_i=\operatorname{atan2}\!\left(v_y+\omega x_i,\ v_x-\omega y_i\right),\qquad n_i=\frac{|\mathbf{v}_i|}{r}=\frac{\sqrt{(v_x-\omega y_i)^2+(v_y+\omega x_i)^2}}{r} $$

$\delta_i$ 与坐标约定一致：以 $+x$（向右）为 $0^\circ$、逆时针为正。对最常见的**矩形对称四轮**（$\mathbf{R}$ 见上节麦轮），逐个写开就是：

| 轮 | $\mathbf{R}_i$ | 轮心速度 $\mathbf{v}_i$ |
| --- | --- | --- |
| FL | $(-l_x,\ l_y)$ | $\big(v_x-\omega l_y,\ v_y-\omega l_x\big)$ |
| FR | $(\ l_x,\ l_y)$ | $\big(v_x-\omega l_y,\ v_y+\omega l_x\big)$ |
| RL | $(-l_x,-l_y)$ | $\big(v_x+\omega l_y,\ v_y-\omega l_x\big)$ |
| RR | $(\ l_x,-l_y)$ | $\big(v_x+\omega l_y,\ v_y+\omega l_x\big)$ |

自检：纯平移到右 $v_x$ 时四轮 $\delta_i=0^\circ$、$n_i=v_x/r$；纯后退时 $\delta_i=180^\circ$、$n_i=|v_x|/r$（轮子倒着转，或者舵角翻 $180^\circ$，两者等价）；纯自转时每个轮的 $\mathbf{v}_i$ 都垂直于 $\mathbf{R}_i$，舵角沿各自的切线方向、轮速大小都为 $\omega\sqrt{l_x^2+l_y^2}/r$，正是四轮切向驱动绕中心画圆。

### 工程上必须处理的四个细节

解算式很干净，落地时的坑几乎都在角度上：

1. **零速奇异**：$|\mathbf{v}_i|\to0$（比如 $v_x=v_y=\omega=0$，或某个轮心恰好是瞬时转动中心）时 $\operatorname{atan2}(0,0)$ 无意义，舵角会乱跳。做法是**保持上一拍的角度**（或回正到 $0^\circ$），只把轮速置 0；千万不能让 $n_i=0$ 却让舵向电机按一个随机目标角乱转。
2. **最短路径转向 + 轮子反转**：舵向电机一般要多圈连续转，若直接把 $\operatorname{atan2}$ 的结果当目标角，$350^\circ\to10^\circ$ 会反着转 $340^\circ$。工程上要做两件事：① 把目标角解卷绕到离当前角**最近的等价角**（加减 $360^\circ$ 的整数倍）；② 若与当前角相差大于 $90^\circ$，就把目标角加 $180^\circ$、轮速取负 —— 这就是舵轮"倒着走"的由来，能把换向开销从 $180^\circ$ 降到 0。
3. **零点标定**：$\delta_i$ 是轮子的真实朝向，而电机读回来的是**转子机械角**，两者差一个安装偏置、且没有绝对参考。所以每轮的舵向必须有绝对角反馈（绝对式编码器 / 上电找限位），或开机时统一对齐到 $0^\circ$ 建立零位；零点错了，四个轮子会朝不同方向使劲、直接互相打架。
4. **转舵与给速的时序**：轮子没转到目标角就给轮速，等于让轮子斜着"蹭地"。通常先发舵角、等角度误差进入阈值（或对舵角做限速插值）再放开轮速；舵向电机的角度环带宽也要比轮速环慢，否则底盘会有明显的"甩尾"。

### 正解（$\delta_i,n_i$ → 底盘速度）

反过来做里程计：先由舵角、轮速还原每个轮心的速度 $\mathbf{v}_i=n_ir\,(\cos\delta_i,\ \sin\delta_i)$，再解出 $(v_x,v_y,\omega)$。关系式是

$$ v_{ix}=v_x-\omega y_i,\qquad v_{iy}=v_y+\omega x_i $$

这是 $2N$ 个方程对 3 个未知量，一般用**最小二乘**（至少要 3 个轮组才定得出 3 个未知量）。对矩形对称四轮（$\sum x_i=\sum y_i=0$、$\sum x_i^2=4l_x^2$、$\sum y_i^2=4l_y^2$），解是闭式的：

$$ v_x=\frac{1}{4}\sum_{i}v_{ix},\qquad
v_y=\frac{1}{4}\sum_{i}v_{iy},\qquad
\omega=\frac{\sum_i\left(x_iv_{iy}-y_iv_{ix}\right)}{4\left(l_x^2+l_y^2\right)} $$

前两式是"四轮横向/纵向速度取平均"，第三式是"各轮对中心的角动量取平均"，和麦轮位置环里用编码器估计速度是同一个道理。

### 三种构型对比

| | 麦克纳姆轮 | 全向轮（三轮 $120^\circ$） | 舵轮 |
| --- | --- | --- | --- |
| 全向移动的实现方式 | 辊子斜置 $45^\circ$，驱动方向被分解到两个轮子平面上 | 辊子垂直于轮毂轴，只约束滚动方向 | 每个轮组自带转向自由度，无滑移 |
| 单轮约束条数 | 1（投影） | 1（投影） | 2（方向 + 大小） |
| 全向性 | 是 | 是 | 是（真正的无滑移全向） |
| 效率 / 损耗 | 辊子对角滑移，损耗和磨损较大 | 驱动方向纯滚动、仅被动方向滑动，损耗中等 | 无滑移，效率最高 |
| 机构复杂度 | 低（4 轮 + 辊子） | 低（3 轮，但辊子成本高） | 高（每轮 2 个电机 + 绝对角反馈 + 零点标定） |
| 主要工程痛点 | 辊子磨损、过坎打滑导致解算失真 | 三轮过约束少、单轮失效即失解 | 转向延迟、角度解卷绕、零点标定 |

顺带一提：**"半舵半全向"**（前两轮舵轮、后两轮全向轮之类）之所以存在，是在"全向性 / 驱动数 / 重量 / 成本"之间折中 —— 保留舵轮的高效率与主动转向，又用全向轮省掉两个舵向电机。
