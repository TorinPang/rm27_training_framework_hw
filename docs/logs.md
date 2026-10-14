# 调试日志

记录格式：**日期 · 目标 → 现象 → 猜想 → 改动 → 结果**。

---

## 2026-10-11 · 首次上板：验证方向 A 的"工具链 + 运行"部分

**目标**：把方向 A 写好的固件烧进 C 板，先证明"编译 → 烧写 → 运行"这条链是通的，
再去验 USART 收发。

**现象（起点）**：此前只做过编译层验证（`board/build/Debug` 增量 ninja），从未上板。
且代码里**没有任何 `printf` 调用**，所以原始固件即使烧进去，串口也不会有任何输出。

**猜想**：
1. ST-Link 与 OpenOCD 应该能直接工作（`.vscode/tasks.json` 里已有 `flash-openocd` 任务）；
2. 三色灯（PH12/PH11/PH10，**高电平点亮**）是最省事的自检点，不需要任何外接设备；
3. 一旦真的调用 `printf`，`--gc-sections` 就不会再丢掉 printf 整条链，**FLASH 占用应明显上涨**——
   这可以反过来验证"没人调 printf 时它不进固件"这个判断。

**改动**：只改 `board/Core/Src/main.c`，加一段**带开关的上板自检**：

- `USER CODE BEGIN PD`：`#define BSP_SELF_TEST 1`（开关；验完置 0 即可保留代码但不执行）；
- `USER CODE BEGIN Includes`：`#include <stdio.h>`；
- `USER CODE BEGIN 2`（`bsp_Init();` 之后、`MX_ThreadX_Init()` 之前）：
  - 自检 1：三色灯齐亮 500 ms 再灭；
  - 自检 2：经 USART1 打印 `[rm27] boot ok` + `HAL_RCC_GetHCLKFreq()` + `HAL_GetTick()`。

  放在 `MX_ThreadX_Init()` **之前**，因此此时尚未进 ThreadX，阻塞发送与 `HAL_Delay` 都可放心用。
  （`App_ThreadX_Init()` 目前是空的、一个用户线程都没有，所以打印不能放进线程里。）

**结果**：

| 项 | 数值 |
|---|---|
| 编译 | `exit=0`，无告警 |
| RAM | 5624 B (4.31%) → **6056 B (4.62%)** |
| FLASH | 29860 B (2.85%) → **35660 B (3.40%)**，**+5800 B** |
| 烧写器 | `STLINK V2J46M32`，目标电压 3.256 V，`Cortex-M4 r0p1` 识别正常 |
| 烧写 | `device id = 0x10076413`（STM32F407IG，Flash 1024 KiB）→ **Programming Finished → Verified OK → Resetting Target** |
| 上板现象 | **三色灯正常点亮 500 ms 后熄灭**（用户回报） |

**结论**：

1. **时钟 / 烧写 / 运行三者全通** —— 方向 A 的"工具链 + 运行"部分验收通过。
2. FLASH 涨了 5800 B，实测印证了之前的判断：`printf` 整条链（`printf` / `_write_r` / `__io_putchar` / `vfprintf` …）
   在无人调用时会被链接器整段丢弃，一旦被引用就会保留。
3. **串口这一半仍未验收**：手上没有串口线，`[rm27] boot ok` 究竟有没有发出去还没确认。
   这不影响"程序在跑"的结论（灯已经证明了），但 USART 发送/接收链路仍属未验证。

**遗留 / 下一步**：

- 拿到串口线（或直接用自带串口的 ST-Link V2）后：连 PA9=TX / PB7=RX、**115200 / 8N1 无流控**，
  复位看 `[rm27] boot ok`；再做 `USART_RxCallback` 回环验"增量 + 缓冲回绕"。
- 方向 B 的上板验证需要电机 + CAN 分析仪/上位机（确认 0x200/0x1FF、8 字节大端序、RELAX 时电流为 0）。

**排错提示（本次未用到，先记下）**：OpenOCD 的输出走 **stderr**，PowerShell 会把整段标红成
`NativeCommandError`——**那不是错误**，只看 `exit=0` 和日志里的 `Verified OK`。

---

## 2026-10-11 · 第二次上板：验证 ThreadX 内核启动 + `control` 线程在跑（无电机）

**目标**：方向 B 步骤 4 之后新增了"全局 C++ 对象构造 + 内核里创建线程"，这是首次让用户代码
跑进 ThreadX 调度器。**手上只有板子、没有电机、也没有串口线**，所以本轮只验一件事：
**内核能不能正常启动、`control` 线程能不能被调度。** 不验电流、不验电机反馈。

**现象（起点）**：`control` 线程上一轮只做到"编译通过"，镜像里已经有它，但从未上电跑过。

**猜想 / 验证思路**：没有串口线，就不能靠打印。换个办法——**用 SWD 直接读内存**，这不需要任何外设：

1. `ThreadX` 的 `TX_THREAD` 控制块是普通全局变量，能直接读；
2. 其中 `tx_thread_run_count`（偏移 4）每被调度一次就 +1 —— **间隔 1.5 s 读两次、看差值是否 ≈1500**，
   就能同时证明"线程在跑"和"周期是 1 ms"；
3. `_tx_thread_system_state` 为 0 表示内核已脱离初始化、正常调度；
4. `control_motor` 对象里的 `canId` / `hcan` / PID 增益与限幅，能反过来证明构造函数与 `registerMotor` 真的执行了；
5. **无电机时 CAN 必然收不到 ACK**，所以读 `CAN1->ESR` 应看到 BOFF（bus-off）——这本身就能证明
   "control 线程确实走到并执行了发送"，同时读 `CAN1->BTR` 可直接复核波特率；读发送邮箱还能看到**报文内容**。

先取符号地址（`arm-none-eabi-nm --print-size`），再取 `TX_THREAD` 结构布局
（`board/Middlewares/ST/threadx/common/inc/tx_api.h`）：`tx_thread_id`@0、`tx_thread_run_count`@4、
`tx_thread_stack_ptr`@8、`stack_start`@12、`stack_end`@16、`stack_size`@20、`name`@40、`priority`@44、`state`@48。

**改动**：**不改任何代码**，只烧录 + 读内存。

**结果**：

| 项 | 实测 | 判读 |
|---|---|---|
| 烧写 | `Verified OK`、`device id = 0x10076413`、flash 1024 KiB、目标电压 3.261 V | 烧写链正常 |
| `_tx_thread_system_state` @0x20000010 | `0` | **内核已在正常调度**（非初始化态） |
| `_tx_thread_created_count` @0x200000D4 | `2` | 已建 2 个线程（idle + control） |
| `control_thread.tx_thread_id` | `0x54485244` = `'THRD'` | 控制块有效 |
| …`tx_thread_run_count` | 58610 → **60118**（1.5 s 后） | **+1508 ≈ 1005 次/秒 → 周期 1.005 ms** |
| …`tx_thread_priority` / `preempt_threshold` | `8` / `8` | 与 `CONTROL_THREAD_PRIORITY=8` 一致 |
| …`tx_thread_state` | `4` = `TX_SLEEP` | 正睡在 `tx_thread_sleep(1)`，符合预期 |
| …`stack_start` / `stack_end` / `stack_size` | `0x20001110` / `0x2000150F` / `1024` | 与 `control_thread_stack` 符号完全吻合 |
| 栈水位（`stack_end - stack_ptr`） | 0x150F − 0x13BC = **339 B** / 1024 | 余量充足（约用 1/3） |
| `CAN1->ESR` @0x40006418 | `0x00F80057` → **BOFF=1**、TEC=248、LEC=5(Bit0Error) | **发送确实执行过**；无应答→bus-off |
| `CAN1->TSR` @0x40006408 | `0x82000000` → TME2/1/0=0 | 三个发送邮箱全满（帧没发出去） |
| `CAN1 sTxMailBox[0..2]` | TIR=`0x40000001`、TDTR=`0x00000008`、TDLR/TDHR=`0` | **STID=0x200、IDE=0（标准帧）、RTR=0（数据帧）、DLC=8、8 字节全 0** |
| `CAN1->BTR` @0x4000641C | `0x00290002` → BRP=3、TS1=10、TS2=3 | 42 MHz / (3×14) = **1 Mbps** ✅ |
| `control_motor` 对象（0x20001510，`nm` 大小 192 B） | `canId=0x201`、`hcan=&hcan1`；`speedPid`(kp=30, maxOut=10000, maxIOut=3000)、`positionPid`(kp=10, maxOut=20, maxIOut=20)；`speedSet=5.0`、`positionSet=1.0`；`currentSet=0`、`maxCurrent=10000`、`gearBox=1`；`MotorState=0(OFFLINE)`、`controlMode=0(RELAX)` | **构造函数与 `registerMotor` 都执行了，且值与设计一致** |

**结论**：

1. **内核启动正常，`control` 线程以 1 ms 周期稳定运行** —— 方向 B 步骤 4 的"上板启动"这一关过了。
2. 无电机时 **CAN 进入 bus-off 是协议行为，不是软件 bug**：帧已按 `0x200` 组好并放进邮箱，
   只因总线上没有第二个节点回 ACK，TEC 累加到 248 → BOFF。
3. 顺带在**寄存器层**验掉三条 B 的 §3.3 条目：报文 ID `0x200` + DLC 8 ✅、RELAX 时电流字节为 0 ✅、
   CAN 波特率 1 Mbps ✅。（"8 字节**大端序**"仍无法由全 0 数据证明，需等电机接上给非零电流。）

**顺带修正一条笔记**：ARM EABI 下 GCC **默认 `-fshort-enums`**（枚举占 1 字节），
所以 `DJIMotor` 布局是 `MotorState`@4、`controlMode`@5、`motorFeedback`@8(32 B)、`canId`@40、
`hcan`@44、`speedPid`@48、`positionPid`@108、`speedSet`@168、`gearBox`@188、**`sizeof`=192**。
（用 `static_assert`/模板 `Show<N>` 编译报错法量出来的，与 RAM 实测逐字段吻合。）

**遗留 / 下一步**：

- 接上电机 + CAN 分析仪后，才能验：反馈解析/编码器回绕、RELAX 之外的速度环、双环位置、离线检测、
  以及"手动转输出轴一圈"确认编码器 8192 vs 8191。
- `CONTROL_TEST_MODE` 保持 **0（RELAX）**，最安全；整定 PID 时再改 1/2。
- 串口三条仍挂起（无串口线）。

---

## 2026-10-11 · 方向 C 步骤 1：BMI088 的 SPI 读通路（读芯片 ID）

**目标**：写完 BMI088 的寄存器读写封装（TODO #5 + #6 的 `WriteReg`/`ReadReg`），并按"一次只让一条方向亮"的节奏
**只验读通路** —— 读两只芯片的 ID，不碰任何寄存器。这样万一不过，就能确定是 SPI 模式/引脚/片选的问题，与写通路无关。
手上没有串口线，所以验证靠 **SWD 读内存 + 三色灯**。

**现象（起点）**：SPI1 已由 CubeMX 配好（PB3/PB4/PA7、Mode3、预分频 /16 = 5.25 Mbit/s、软件 NSS），
PA4/PB0 作为两组片选输出且初始为高；`VerifyAccChipID()`/`VerifyGyroChipID()` 是老师已给的，就等 `ReadReg` 落地。

**猜想**：

1. 这个方向最大的不确定不是引脚，而是**读协议里那个"哑字节"算在哪一位**：数据手册 §6.1.2 说加速度计在地址之后
   **还会多送 1 个哑字节**（单字节读要按 burst 读 2 字节、丢弃第 1 个），§6.1.1 说陀螺仪**没有**哑字节。
2. 老师已给的 `VerifyAccChipID`（读 2 字节取 `[1]`）与 `VerifyGyroChipID`（读 1 字节取 `[0]`）反过来给出了口径：
   `ReadReg` 应当**按原样回传地址之后的 len 字节、不替调用方丢弃**，否则两处既有代码就会读错位。
3. 没有串口线也能验：`TX_THREAD` 与全局对象都是普通 RAM 变量，SWD 直接读即可（本项目第二次上板已经这么干过）；
   灯效则当作第一眼的判据。

**改动**：

- `modules/include/BMI088.hpp`：`#include "spi.h"`（自带 `main.h`）+ 4 个宏：`BMI088_SPI_HANDLE = hspi1`、
  `BMI088_ACC_CS_*`（PA4）、`BMI088_GYRO_CS_*`（PB0）。
- `modules/src/BMI088.cpp`：`WriteReg`（地址与 `0x7F` 相与清掉读标志位 → 数据）与 `ReadReg`
  （发 `addr|0x80` → 一次 `HAL_SPI_TransmitReceive` 收发 `len+1` 字节 → 只取后 `len` 字节；失败清零）。
- `board/CMakeLists.txt`：加 `modules/src/BMI088.cpp`、`threads/src/imuthread.cpp`，
  以及 **`bsp/src/bsp_pwm.cpp`** —— 这一条不直观：`cBMI088` 有虚函数，一旦构造对象整张 vtable 被引用，
  `TemperatureControl` 就进固件、进而需要 `PWM_SetDutyRatio`（PWM 本身属方向 D，这里只是个空桩）。
- 新建 `threads/src/imuthread.cpp`：最小 imu 线程（静态栈 1024 B、优先级 10、100 ms 周期），
  只做"ID 自检 + 灯效"；`g_imu` 放全局存储以便 SWD 按符号读。`threads.hpp` 加 `ImuThread_Init()`，
  `app_threadx.c` 里在 `ControlThread_Init()` 之后调用。

**结果**：

| 项 | 实测 | 判读 |
|---|---|---|
| 编译 | `exit=0`、无告警；RAM 7456 → **8768 B**、FLASH 39612 → **44008 B** | RAM 增量 1312 = 栈 1024 + `TX_THREAD` 176 + `g_imu` 112，与 `nm` 尺寸逐项吻合 |
| 烧写 | `Verified OK`、`device id = 0x10076413` | 烧写链正常 |
| 第一次上板 | **红灯亮**；`self_test`：`ACC_CHIP_ID_ERR = 1`、`GYRO_CHIP_ID_ERR = 0` | **陀螺仪通了、加速度计没通** → 总线/模式/片选/引脚都没问题 |
| 原始字节取证 | ACC = `23 1E 23 00 10 …`、GYRO = `0F 22 xx …` | ACC 的 ID 在 `[1]`、GYRO 的 ID 在 `[0]`；**顺带用数据证实 PA4=加速度计、PB0=陀螺仪** |
| 第二轮上板（加 `tx_thread_sleep(10)` + 3 轮连读） | 三轮 ACC 全为 `[1]=0x1E`、GYRO 全为 `[0]=0x0F`；`self_test` 两个标志归 0 | 失败**只发生在重置后的第一次访问**，之后每次都正常（第 1 轮哑字节位是 `0x23`、后两轮是 `0x00`） |
| 定稿版连续两次上电 | `self_test` = `00 01 00 01`（两标志均 0）、`GPIOH->ODR = 0x800` | **绿灯亮，2/2 次一致** |
| `imu_thread` | `'THRD'`、priority 10、`state = TX_SLEEP`；`run_count` 1.5 s 内 +15；栈水位 347/1024 B | 100 ms/轮，余量充足 |

**结论**：

1. **读通路成立**：两只芯片 ID 都读对，且取字节的位置与老师既有的两个 `Verify*ChipID` 完全自洽（既有代码一行未改）。
2. **片选归属不再是"按原理图猜"**：PA4 读到 `0x1E`、PB0 读到 `0x0F`，用数据钉死了。
3. 唯一异常是**重置后第一次 SPI 事务不可靠**（加速度计侧）。只有 1 个失败样本，所以机制**没有**下结论；
   采取的是防御式处理：先 `tx_thread_sleep(10)` 等稳定 → 丢弃一次热身读 → 校验失败再重读一轮。
   定稿后连续两次上电稳定通过。若以后再复现，应优先怀疑"首次事务的 CS/SCK 建立时序"而不是协议取位。

**遗留 / 下一步（C 的后续步骤）**：

- 步骤 2：`ReadAccData`/`ReadGyroData`/`ReadAccTemperature` 的拼接与单位换算（`m/s²`、`rad/s`、`℃`），
  `Verify*Data` 用"全 0 / 全 0xFF / NaN"判通信失效，并拔线复现一次。
- 步骤 3：把 `Config()`/`Calibrate()` 接进 imu 线程（注意 `Calibrate()` 采样 4000 次 ≈ 阻塞 4 s），
  并留出给其它线程读数据的出口。
- `TemperatureControl` + `bsp_pwm.cpp` 的实质实现属方向 D，本轮只是让 `bsp_pwm.cpp` 进工程以解链接依赖。

---

## 2026-10-11 · 方向 C 步骤 2：三轴/温度解析与单位换算（含一次"数据恒 0"的定位）

**目标**：实现 `ReadAccData` / `ReadGyroData` / `ReadAccTemperature` 的拼接与换算，以及 `VerifyAccData` / `VerifyGyroData`；
把 `Config()`/`Calibrate()` 接进 imu 线程，把 `imu` 线程的周期从 100 ms 改成 1 ms（与 acc 800 Hz / gyro 2000 Hz 相称）。

**现象（起点）**：第一次烧上去之后，`self_test` 里 **`ACC_DATA_ERR = 1`、`GYRO_DATA_ERR = 0`**，
加速度三轴换算出来恰好等于各自预标定零偏的**负值** —— 这正是"原始计数全 0"的特征。

**猜想**：

1. 陀螺仪数据与温度都正常、只有加速度计数据是 0，说明 SPI 读通路没问题（温度也挂在加速度计接口上）。
   嫌疑落在"加速度计根本没在转换" —— 即它还停在复位默认的 **suspend**（手册：`ACC_PWR_CONF` 复位值 0x03 = suspend，
   且 `ACC_PWR_CTRL` 必须在上电后写 0x04 才会出数据）。
2. 但 `Config()` 里明明写了这两个寄存器。于是先要回答"**写到底有没有生效**"，而不是继续猜协议。
3. 顺带核实一个疑点：`Config()` 写 `ACC_CONF = 0xAB`，而头文件里 `ACC_CONF_BWP_NORM` 定义为 `0x02` —— 两者对不上，
   需要查手册确认哪个是对的。

**改动（诊断阶段）**：在 imu 线程 `Config()` 之后临时挂一段取证，把 6 个加速度计寄存器、3 个陀螺仪寄存器、
加速度原始字节读回到全局 `g_reg_dump`，再用 SWD 读出来；随后**再单独写一次** `ACC_PWR_CTRL = 0x04` 并回读，
以区分"写不进去"与"写进去又被复位"。

**结果**：

| 读序 | 实测（`g_reg_dump`） | 判读 |
|---|---|---|
| `Config()` 之后 | `PWR_CTRL=0x00`、`PWR_CONF=0x00`、`CONF=0xAB`、`RANGE=0x01`、`ERR_REG=0x00`、`STATUS=0x10(drdy=0)` | 写通路**是通的**（`CONF`/`RANGE` 都不是复位值），只有"上电"那一步没落实 |
| 加速度原始字节 | `00 00 00 00 00 00 00` | 数据寄存器恒 0，与"停在 suspend"一致 |
| 重写 `PWR_CTRL=0x04` 后 | 回读 **`0x04`**、`STATUS=0x90(drdy=1)`、原始值 `4b 00 fe ff 3e 15` | **一次就活了**，Z 轴 ≈ **+9.74 m/s²** |
| 最终固件连续两次上电 | `acc = (0.131, −0.012, 9.786)` / `(0.161, −0.010, 9.741)`；温度 24.75 ℃；陀螺 ~1e-3 rad/s；`self_test` 全 0 | **`|a| ≈ 9.79 / 9.74 m/s²`，2/2 次一致** |
| `imu_thread` | `run_count` 1.5 s 内 +1505 → **1.00 ms/轮**；栈水位 **483/1024 B** | 周期与余量都正常 |

**结论**：

1. **根因是 BMI088 的经典坑**：`ACC_PWR_CTRL`/`ACC_PWR_CONF` 这两次写**不保证一次生效**。
   本工程实测 `Config()` 里那一次 `ACC_PWR_CTRL=0x04` 读回仍是 `0x00`，加速度计一直停在 suspend。
   阴险之处在于**芯片 ID 与温度都照常可读**（温度不受 suspend 影响），极易误判成"总线没问题、只是数据不对"。
   → 修法是官方驱动的老套路：**写 → 回读 → 不匹配就重试**，落在新增的 `EnsureAccPowered()` 里（各 10 次上限）。
2. **哑字节口径再次被证实**：ACC 的寄存器值都在返回流的 `[1]`（读 2 字节）或 `[1..6]`（读 7 字节），GYRO 从 `[0]` 起。
3. **`ACC_CONF = 0xAB` 是对的**：手册 §5.3.8 里 `acc_bwp` 的有效值是 `0x08`/`0x09`/`0x0A`（**都带 bit3**），
   `0x0A << 4 | 0x0B(800 Hz) = 0xAB`。反过来，头文件里那三个 `ACC_CONF_BWP_*` 宏（`0x00/0x01/0x02`）少了 bit3，
   本次一并修正为 `0x08/0x09/0x0A`（它们此前未被任何代码使用，改后行为不变）。
4. `Verify*Data` 的判据选的是"**6 个原始字节完全相同**"（通信断线/短接时读回恒定字），
   而不是"`|a| ≈ 9.8`" —— 后者在直线加速运动时会误报，前者与姿态无关。

**遗留 / 下一步**：

- **通信失败复现**（C 的最后一条验收）：把加速度计 MISO 短接到 GND（或 VCC）后，6 字节应全同 → `ACC_DATA_ERR` 置位、灯转蓝。
  拔线悬空时 MISO 电平不定，可能读到随机值而不置位，所以**推荐用短接**来复现。
- 量纲核对已完成，但"绕 Z 轴转板子看符号与量级"这类动态验证属手工项，尚未做。
- 数据共享的出口（`g_imu` 给其它线程读、是否加 `tx_mutex`）与 `imu_alive` 信号量归方向 E。
- `TemperatureControl` 与加热电阻 PWM 归方向 D。

---

## 2026-10-11 · 方向 D 步骤 1：PWM 封装（TODO #2）

**目标**：补完 `bsp_pwm.cpp` 的 5 个空桩（初始化/启停/周期/占空比），把本工程两路 PWM 的用途与量纲钉死。
**本轮只做驱动本身**，不碰温控环（`TemperatureControl` 属 D2）—— 万一不对，能立刻确定问题出在 PWM 通道/时钟换算，而不是控制逻辑。

**现象（起点）**：`board/Core/Src/tim.c` 里 CubeMX 已经配好两路：

- 加热电阻 TIM10_CH1 = PF6，PSC=167 / ARR=999 → **1 kHz**，初始 `Pulse=0`；
- 舵机 TIM1_CH1 = PE9，PSC=167 / ARR=19999 → **50 Hz**，初始 `Pulse=10000`（1.5 ms 中位）。

两者 PSC 都是 167，故 **1 tick = 1 µs**：50 Hz 那路 20000 tick/周期，**CCR 直接就是脉宽微秒数**。
而 `bsp_pwm.cpp` 里 5 个函数全是 TODO 空桩 —— 其中 `PWM_SetDutyRatio` 已被 `BMI088::TemperatureControl` 引用，
所以它一直"进得了固件、但什么都不做"。

**猜想 / 要先想清楚的三个点**：

1. 唯一非平凡的是 `PWM_SetPeriod` 的"秒 → ARR"：得先拿到**定时器的实际输入时钟**。F4 的 APB 定时器在
   APB 预分频 ≠ 1 时是 PCLK 的 **2 倍**；本工程 APB1 = 42 MHz(/4) → 84 MHz、APB2 = 84 MHz(/2) → 168 MHz，
   **两个域都要乘 2**，于是只需区分挂在哪条 APB 上（TIM1 / TIM8~11 在 APB2）。
2. `duty=1` 想表达"全功率"：若满量程取 ARR，PWM1 模式下还会缺最后一个计数，故取 **ARR+1**
   —— `duty=1` 时比较值 > ARR，输出整周期为高，才是真正的 100 %。
3. 没有串口线、也没有示波器，验证只能走 **SWD 读定时器寄存器**
   （TIM1 基址 `0x40010000`、TIM10 基址 `0x40014400`；读 `CR1`/`CCER`/`ARR`/`CCR1`，TIM1 另加 `BDTR`）。

**改动**：

- `bsp/src/bsp_pwm.cpp`：补完 5 个函数。
  - `PWM_TimerClock()` 取定时器时钟。**坑**：一开始用 `switch ((uint32_t)TIMx)` 写，编译报
    `reinterpret_cast<TIM_TypeDef*>(...) is not a constant expression` —— `TIMx` 宏是指针，**C++ 里不能当 case 标签**；
    改成 `*_BASE` 常量 + if 链。
  - `PWM_SetDutyRatio`：先夹到 [0,1]（写成 `!(x > 0)` 顺带挡住 NaN）再写 CCR，满量程 `ARR+1`。
  - `PWM_SetPeriod`：`ARR = period × timer_clk / (PSC+1) − 1`，并夹到 16 位范围（本工程两路都是 16 位定时器）。
  - `PWM_Start`：句柄空就返回；`HAL_TIM_PWM_Start` 失败则退回停止态。**不调 `Error_Handler()`** ——
    它在 `bsp_Init()` 里被调用，死循环会把后面的自检和 ThreadX 一起带走，退回安全状态反而更好定位（同 `CAN_Transmit` 的取舍）。
  - `PWM_Init()`：启动加热(TIM10_CH1)与舵机(TIM1_CH1)两路，初值都取安全侧 —— 加热 **0 %**
    （必须先摆 0 再启动，否则会按 CubeMX 的 `Pulse` 直接出功率）、舵机 **1.5 ms 中位**；
    另留 `PWM_HEAT_ENABLE` / `PWM_SERVO_ENABLE` / `PWM_DERIVE_PERIOD` 三个开关。
- `bsp/src/bsp.cpp`：`bsp_Init()` 里加 `PWM_Init();`（`MX_TIMx_Init` 已在 `main.c` 里先跑过，顺序安全）。
- `board/CMakeLists.txt`：只改了一行注释（`bsp_pwm.cpp` 早就在 `target_sources` 里）。

**关于 `PWM_DERIVE_PERIOD`**：`PWM_Init` 里额外用 `PWM_SetPeriod(&htim10, 1/1000)`、`PWM_SetPeriod(&htim1, 1/50)`
再推导一遍周期。CubeMX 已经把 ARR 配成 999 / 19999，所以**算对了行为不变、算错了 ARR 会变**
—— 等于让这条平时没有调用点的换算每次上电都被执行一次，上板读 ARR 就能判对错。

**结果**：

| 项 | 实测 | 判读 |
|---|---|---|
| 编译 | `exit=0`、**无告警**；RAM 8768 B 不变、FLASH 46260 → **47652 B** | 增量都是被真正调用的 HAL 启动代码 |
| 烧写 | `Verified OK`、`device id = 0x10076413`、目标 3.261 V | 烧写链正常 |
| TIM1（舵机 PE9） | `CR1=0x01`(CEN=1)、`CCER=0x01`(CC1E=1)、**`ARR=0x4E1F=19999`**、**`CCR1=0x5DC=1500`**、**`BDTR=0xA000`(MOE=1)** | 通道真在输出；ARR/CCR 与两条算式逐项吻合 |
| TIM10（加热 PF6） | `CR1=0x01`、`CCER=0x01`、**`ARR=0x3E7=999`**、**`CCR1=0`** | 同上；加热初值确实在安全侧 |
| `RCC->CFGR` | `0x0000940A`：HPRE=/1、**PPRE1=0b101(/4)**、**PPRE2=0b100(/2)** | 反证 `PWM_TimerClock()` 推的 168 / 84 MHz 没错（ARR 算对的前提） |

**结论**：

1. 5 个函数全部实现并**上板验证**：`PWM_SetPeriod` 的"秒 → ARR"（999 / 19999）与 `PWM_SetDutyRatio` 的
   [0,1] → CCR（0 / 1500）都拿到了寄存器证据，不是"看代码觉得对"。
2. TIM1 是**高级定时器**，除 `CEN` 外还必须 `MOE=1` 才有输出 —— 实测 `BDTR` bit15 = 1，
   说明 `HAL_TIM_PWM_Start` 内部替我们开了主输出。
3. 满量程取 `ARR+1` 的理由成立，duty 两端落在真正的 0 % / 100 %。

**遗留 / 下一步（D 的后续步骤）**：

- **D2**：`BMI088::TemperatureControl`（`modules/src/BMI088.cpp:96`）用 `TempPid` 算占空比 —— 注意老师给的
  `TempPid.maxOut = 25000` 与 duty ∈ [0,1] **量纲不一致**，要么在函数里显式归一化再夹紧，要么把 `maxOut` 改成 1.0，需留档说明选哪个。
  接进 imu 线程循环即可，板载加热电阻**无需外部硬件**就能验（设目标 40 ℃，看温度上升 + duty 收敛）。
- **D3（选做）**：舵机角度接口（0°~180° ↔ 500~2500 µs，即 CCR 500~2500）。注意 `tim.c` 目前只配了 TIM1_CH1，
  J16 的 C2~C7（TIM1_CH2~4 / TIM8_CH1~3）要用得先回 CubeMX 加。
- **极性未实测**：若"duty=1 反而变凉"，说明驱动是低有效，需改 OCPolarity 或在 bsp 层取反。

---

## 2026-10-11 · 方向 D 步骤 2：温控闭环（`TemperatureControl` + imu 线程）—— 软件侧通过，硬件侧卡在"不升温"

**目标**：把 PWM 接到温控环上 —— `BMI088::TemperatureControl` 用 `TempPid` 算占空比、下发加热 PWM，
并接进 imu 线程。板载加热电阻，本应**不需要任何外部硬件**就能验。

**现象（起点）**：`BMI088.cpp:96` 原本是无条件 `duty=0` 的安全占位；imu 线程每 1 ms 已经在读
`acc_data.temperature`（实测室温 24.75 ℃），所以调用点现成。

**猜想 / 要先解决的量纲问题**：

1. 老师给的 `TempPid = PID(0.1, 0, 0, maxOut=25000, maxIOut=3)`，而 PWM 要的是 `duty∈[0,1]` ——
   `maxOut=25000` 与 [0,1] **不是一个量纲**，PID 自身的输出限幅在本用途下等于不起作用。
2. 两条路：改 `maxOut` 为 1.0（动老师的参数），或**在函数内显式归一化**。
   选后者：`duty = clamp(TempPid.result, 0, 1)`，于是 `kp=0.1` 的含义就是"每 1 ℃ 误差给 10 % 功率"
   （误差 10 ℃ 即满功率）—— 对加热电阻是合理的 P 增益，也说明 25000 只是个够大的兜底值。
   安全性说明：① 加热只升温不制冷，负误差一律截到 0；② 本轮 `ki=kd=0`（纯 P），`maxIOut` 根本不参与，
   将来若整定出积分项，末端这道夹紧仍保证下发值合法（但届时 `maxIOut` 与 [0,1] 的口径要再对齐一次）。
3. 一个必须堵的坑：若在温度还没读到（`fdb=0 ℃`）时就闭环，误差 40 ℃ → **立刻满功率**。
   除了用 `INIT_ERR` / `ACC_DATA_ERR` 把关，再加一道**温度合理性带**（`-20 ~ +100 ℃`）兜底。

**改动**：

- `modules/include/BMI088.hpp`：新增 `TEMP_SANE_MIN_C (-20.0f)` / `TEMP_SANE_MAX_C (100.0f)`（合理性带）。
- `modules/src/BMI088.cpp`：`TemperatureControl` 实现完成 ——
  不安全工况（`INIT_ERR` / `ACC_DATA_ERR` / 温度出带 / 目标非有限）→ `TempPid.Clear()` + 不加热 + `TEMP_CTRL_ERR=true`；
  否则 `ref/fdb` 赋值 → `UpdateResult()` → **显式归一化到 [0,1]**（`!(duty>0)` 的写法顺带挡 NaN）→ 下发 → `TEMP_CTRL_ERR=false`。
- `threads/src/imuthread.cpp`：循环里**读温之后、`Verify*Data` 之后**调用（这样加速度计通信失效会立刻把加热关掉）；
  顶部加 `TEMP_CTRL_ENABLE` 与 `TEMP_CTRL_TARGET_C 40.0f`。

**结果**：

| 项 | 实测 | 判读 |
|---|---|---|
| 编译 | `exit=0`、**无告警**；RAM 8768 B 不变、FLASH 47652 → **47932 B** | 增量只有温控与 PID 的那点代码 |
| 烧写 | `Verified OK` | 正常 |
| 闭环内部状态（SWD） | `TempPid.ref=0x42200000=40.0`、`fdb=0x41D20000=26.25 ℃`、`result=0x3FB00000=1.375` | `1.375 = 0.1 × (40−26.25)`，**与算式逐位吻合** |
| 下发值 | `TIM10->CCR1=0x3E8=1000` → duty=1.0 | 归一化 + 夹紧生效（1.375 → 1.0） |
| `self_test` | 7 个标志**全 0**（含 `TEMP_CTRL_ERR`） | 初始化、标定、数据、温控四条都良性 |
| 不加热分支 | 手动置 `INIT_ERR=1` → `CCR1=0`、`TEMP_CTRL_ERR=1` | 失败工况确实不加热 |
| 加热极性 | `GPIOF->IDR` bit6：`duty=1` → **1**、`duty=0` → **0** | **MCU 侧极性正常（高=加热）**；AF 模式下 IDR 仍反映引脚实际电平 |
| **温度** | duty=1 连续 **37 s**、duty=0 保持 **25 s**：一直是 `26.25 / 26.5 ℃` 之间按 0.125 ℃ 量化抖动，**无净升温** | **加热回路上没有功率** |

**结论**：

1. **软件侧全部通过**：PID 算得对（`ref/fdb/result` 与算式吻合）、归一化与夹紧生效、寄存器确实被写入、
   失败工况确实不加热、MCU 侧极性正常。D2 的代码可以认为完成。
2. **硬件侧未通过**：温度在任何 duty 下都不动。极性已排除，故问题不在软件也不在极性，
   指向**加热回路上没有功率** —— 最可能是**板子只由 ST-Link 供电、没接自己的主电源**
   （加热电阻挂在 5 V / VM 轨上，ST-Link 只给 3.3 V，不产生这条轨）。
3. 顺带学到一条排查手法：**AF 模式下也能用 `GPIOx->IDR` 读引脚实际电平**，不用把引脚改回 GPIO 输出。
   另一条：想临时改闭环状态时，改 `TempPid.ref` 是没用的 —— 固件每 1 ms 都用入参重写它，
   要改用固件不会主动清的状态位（如 `INIT_ERR`）去驱动分支。

**遗留 / 下一步**：

- **待用户确认供电**（是否接了电池/DC；也可用万用表量 PF6 与加热电阻供电端）后，用**同一份固件**复测：
  目标 40 ℃ 应从室温满功率升温到约 30 ℃ 后线性收力，最终稳定在 40 ℃ 附近。
- 若接了主电源仍不升温，再查硬件侧驱动（MOSFET / 上拉）与电阻是否焊在加热位。
- D3（选做）舵机角度接口；数据共享出口与 `imu_alive` 归方向 E。
