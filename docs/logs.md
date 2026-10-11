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
