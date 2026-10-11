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
