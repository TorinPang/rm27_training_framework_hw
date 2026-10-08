# AGENTS.md — 项目规范与执行计划

本文件是**人机协作契约**：每次新开对话，agent 先读本文件 + `README.md`，再动手。
规则：本文件只记录**结论、约定、当前进度**，不写冗余推导；改动了工程就同步更新第 3 节。

---

## 1. 任务与边界

**必做（开发部分）**

1. CubeMX 配置 → 生成板级代码（**已完成**，见 §3.1）
2. `board/CMakeLists.txt` 启用 C++，接入 `bsp/ libs/ modules/ threads`
3. 补全 `bsp → libs → modules` 的 TODO（§3.2）
4. 自行设计并实现 `threads/`（alive / imu / control），记得 sleep
5. 文档：`docs/spi.md`、`docs/iic.md`、`docs/logs.md`（调试记录）、`docs/chassis.md`

**选做**：IST8310 磁力计读取、6/9 轴 IMU 解算、温控闭环、灯效巧思。

**明确不做**：不重构老师的框架结构，不删 `board/` 生成代码，不动 `board/.mxproject`。

---

## 2. 分层与代码纪律

```
board/(CubeMX 生成)  ←  bsp/  ←  modules/  ←  threads/
                        libs/ 被 bsp/modules/threads 共享
```

- **HAL 调用只能出现在 `bsp/`**（LED、IMU 例外：README 允许直接调 HAL GPIO/SPI）。`modules/` 及以上不得直接包含 `stm32f4xx_hal*.h`。
- **CubeMX 重新生成会覆盖 `board/Core/`**：自己的代码永远写在 `bsp/ libs/ modules/ threads/`，或 `board/` 内的 `USER CODE BEGIN/END` 区。
- 分层已有目录结构（`include/` + `src/`），新增文件**沿用**，不要另起目录。
- 每个功能必须有**开关**（宏或线程是否启动），保证能单独验证。

---

## 3. 当前状态

### 3.1 已完成（2026-10-07）

- **CubeMX 配置全部完成并生成**，引脚映射见 §5。含 ThreadX（tick=1000、静态内存、时基已改 TIM3）。
- 已实现：`libs/src/math.cpp`、`libs/src/crc.cpp`、`bsp/src/bsp_can.cpp` 的 `CAN_Init`、`modules/src/BMI088.cpp` 的 `Config()/Calibrate()`。
- **`AGENTS.md` 即本文件**（README 第 4 步要求）。

### 3.2 待补 TODO（唯一权威清单）

| # | 文件:行 | 内容 | 所属方向 |
|---|---|---|---|
| 1 | `libs/src/pid.cpp:22` | 位置式/增量式 + 积分限幅 `maxIOut` + 输出限幅 `maxOut` | 地基 |
| 2 | `bsp/src/bsp_pwm.cpp:9,14,19,24,29` | PWM 初始化/启停/周期(秒→ARR)/占空比([0,1]→CCR) | B·C |
| 3 | `bsp/src/bsp_usart.cpp:11,16,21,30` | 双串口 DMA+空闲中断、启动接收、按 mode 发送 | A |
| 4 | `bsp/src/bsp_can.cpp:44,50` | `CAN_Transmit` 组帧发送；接收回调按总线+标准帧 ID 分发 | B |
| 5 | `modules/include/BMI088.hpp:21` | SPI 句柄 + ACC/GYRO 片选宏 | C |
| 6 | `modules/src/BMI088.cpp:63,106,111,116,126,208,215,222` | 温控占空比、数据校验、寄存器读写、三轴/温度解析 | C |
| 7 | `modules/src/M2006.cpp:5,11,12,18` | 构造参数、松开/速度/位置双环、`maxCurrent` 限幅、`AliveCheck` | B |
| 8 | `modules/include/DJIMotor.hpp:65,66` | 速度环/位置环 PID 参数 | B |
| 9 | `modules/src/DJIMotorHandler.cpp:5,6,14,15,20,28,29,36` | 注册(0x201~0x208)、0x200/0x1FF 存在标志、组帧大端序、反馈解析+编码器回绕+减速比换算、存活检查 | B |
| 10 | `threads/*`（空） | alive / imu / control 三个线程 + `main.c` 创建 | A~E |
| 11 | LED 模块 | 自行封装 GPIO（`bsp/` 下新增），供 alive 线程做灯效 | E |

---

## 4. 执行计划

### 4.0 策略：横向地基只做一次，之后全部纵切

不允许"把 bsp 全写完再统一调试"——每层单独拿出来都无法验证，且错误会堆到后期爆发。
也不允许"每个方向各自配一遍 CubeMX"——地基是共享的（CubeMX/CMake/ThreadX/PID）。

### 4.1 阶段 0 · 地基（横向，一次性）

1. `board/CMakeLists.txt`：`enable_language(CXX)`，`target_sources` 加 `bsp/src/*.cpp libs/src/*.cpp modules/src/*.cpp threads/src/*.cpp`，`target_include_directories` 加各层 `include/`。
2. 补全 `libs/src/pid.cpp`（TODO #1）。
3. 保证 `cmake --build` 通过（此时功能都没接线，编译过即可）。

**验收**：能编译、能下载、能点灯（LED 最简单，先证明工具链通）。

### 4.2 方向 A · USART 调试通道（最先做，回报最高）

配置（已完成，DMA 已就位）→ 封装 `bsp_usart.cpp`（TODO #3）→ 上板验证 `printf`。

**验收**：串口助手能看到打印；115200、8N1。后续所有方向的验证都靠它。
实现建议：接收用 `HAL_UARTEx_ReceiveToIdle_DMA`（HAL 内部处理 IDLE）+ `HAL_UARTEx_RxEventCallback` 回调；发送 DMA 的 buffer 必须是 static/全局。

### 4.3 方向 B · CAN → 电机（保底分，越早跑通越好）

`bsp_can`（TODO #4）→ `DJIMotorHandler`（TODO #9）→ `M2006` + `DJIMotor.hpp` 参数（TODO #7,#8）→ `control` 线程。

**验收**（对应 README「完成检查」）：
- 电机 `mode = RELAX` 时输出电流为 0；
- 速度环单独可调；位置+速度双环可调；
- 示波器/上位机确认 CAN 报文 ID（0x200 / 0x1FF）与 8 字节大端序正确；
- 反馈解析正确，编码器回绕不跳变。

### 4.4 方向 C · SPI → BMI088

`BMI088.hpp` 宏（TODO #5）→ 寄存器读写/解析（TODO #6）→ `imu` 线程。

**验收**：芯片 ID 读回正确（ACC `0x1E`、GYRO `0x0F`）；三轴单位明确（m/s²、rad/s）；温度 ℃ 合理；拔掉通信时能置位 `*_DATA_ERR`。

### 4.5 方向 D · PWM → 加热 + 舵机

`bsp_pwm.cpp`（TODO #2）→ `BMI088::TemperatureControl`（TODO #6 中的 63 行）→（如需）舵机 PWM。

**验收**：`duty=0` 不加热、`duty=1` 全功率；舵机 50Hz 下 500~2500 对应 0°~180°。

### 4.6 方向 E · alive 线程（最后做，依赖 A~D）

信号量监控三件事 + 灯效（TODO #10,#11）。

**验收**：初始化失败/IMU 线程死/电机线程死，三种情况灯效可区分。

### 4.7 时间顺序

`0 地基 → A USART → B CAN电机 → C IMU → D PWM → E alive`，每条方向内部节奏固定：
**配该外设 → 写该外设封装 → 上板单独验证 → 再接上层模块 → 再验证**。一次只让一条方向"亮"。

---

## 5. 硬件与配置速查表（C 板 STM32F407IGHx / UFBGA176）

时钟：HSE 12MHz → PLLM6/N168/P2 → **SYSCLK 168MHz**；APB1=42M（Tim 84M）、APB2=84M（Tim 168M）。

| 功能 | 引脚 | 关键参数 |
|---|---|---|
| LED_R / G / B | PH12 / PH11 / PH10 | GPIO_Output，**高电平亮**（经三极管） |
| 舵机 PWM | **PE9 = TIM1_CH1** | 50Hz（PSC=167, ARR=19999），Pulse 500~2500 |
| 加热电阻 | **PF6 = TIM10_CH1** | 1kHz（PSC=167, ARR=999），duty=0 不加热 |
| USART1（调试） | PA9=TX / PB7=RX | 115200；DMA2_S2(RX,Circular) / DMA2_S7(TX,Normal) |
| USART6（裁判系统） | PG14=TX / PG9=RX | 115200；DMA2_S1(RX,Circular) / DMA2_S6(TX,Normal) |
| CAN1 | **PD0=RX / PD1=TX** | 1Mbps（PSC3 + BS1 10TQ + BS2 3TQ），开 `CAN1_RX0_IRQn` |
| CAN2 | PB5=RX / **PB6=TX** | 同上，开 `CAN2_RX0_IRQn` |
| SPI1（BMI088） | PB3=SCK / PB4=MISO / PA7=MOSI | Master 2Lines，CPOL=High+CPHA=2Edge(Mode3)，/16 → 5.25Mbit/s |
| BMI088 片选 | **PA4=ACC_CS / PB0=GYRO_CS** | GPIO_Output，**初始高** |
| I2C3（IST8310） | PA8=SCL / PC9=SDA | Fast 400kHz，7bit 地址 0x0E |
| IST8310 复位 | PG6=RSTN | GPIO_Output 初始高；拉低→延时→拉高 |
| Debug | PA13/PA14 | **Serial Wire**（PB3/PB4 是 JTAG 脚，必须用 SWD） |

**J16 的 7 路 PWM 接口**：C1~C4 = TIM1_CH1~CH4（AF1），C5~C7 = TIM8_CH1~CH3（AF4）。
（注意：**没有** TIM3_CH3/PC8 这种说法，PC8 实为 TIM8_CH3。）

**堆栈**：Heap 0x200 / Stack 0x400；ThreadX tick = 1000（`tx_thread_sleep(1)` = 1ms）。

---

## 6. 线程设计约定

ThreadX **数字越小优先级越高**。全部**静态分配**（栈数组 + `TX_THREAD` 控制块定义在自己的 .cpp 里），在 `tx_application_define()` 里 `tx_thread_create`。

| 线程 | 入口 | 优先级 | 栈 | 职责 |
|---|---|---|---|---|
| control | `control_thread_entry` | 8 | 1024 B | 双环 PID + CAN 收发，必须准时 |
| imu | `imu_thread_entry` | 10 | 1024 B | BMI088 初始化、读三轴/温度、共享给其他线程 |
| alive | `alive_thread_entry` | 20 | 512 B | 信号量监控 + 灯效 |

- 每个线程**必须有 `tx_thread_sleep`**，否则低优先级线程会被饿死。
- 信号量至少 3 个：`init_ok`、`imu_alive`、`motor_alive`。
- 线程间共享数据（IMU 数据）要加互斥或保证单写者。

---

## 7. 构建接入（`board/CMakeLists.txt`）

CubeMX 生成的文件**只在首次生成**，之后可安全手改。需要做的：

```cmake
enable_language(C ASM CXX)          # 原为 enable_language(C ASM)
# target_sources 追加 bsp/src、libs/src、modules/src、threads/src 下的 .cpp
# target_include_directories 追加各层的 include/
# target_compile_definitions 可能需要 ARM_MATH_CM4 等（如用 CMSIS-DSP）
```

`board/CMakeLists.txt` 与仓库根 `CMakeLists.txt` 内容目前相同；**以 `board/` 为工程入口**。

---

## 8. 提交与文档规范

- **提交由本人完成，agent 不执行 `git add / commit / push`。** agent 改动文件后只需说明「改了哪些文件、为什么」。
- 提交信息用 **Conventional Commits**：`type(scope): 摘要`，中文。type 取 feat/fix/refactor/docs/chore/style，scope 用模块名（如 `bsp_can`、`pid`、`threads`）。
- **一个功能一个 commit**；只提交必要产物；**保持原有目录结构**；代码/文档简洁，体现自己的思考。
- 调试过程写入 `docs/logs.md`（现象、猜想、改动、结果）；学习笔记写 `docs/spi.md` / `docs/iic.md` / `docs/chassis.md` / `docs/refereces.md`。
- 生成物判定：`git status` 存在**假阳性**（stat 缓存），以 `git add -n -A` 的输出为准；提交前先 `git diff --cached --name-only` 确认暂存区里没有混入不该提交的文件。

---

## 9. 与 agent 协作纪律

1. 动手前先说明"改哪个文件、为什么"，再改。
2. **一次只推进一条方向的一个小步**，做完立刻上板/编译验证，不做无法验证的批量改动。
3. 不擅自扩大范围：不顺带重构、不批量格式化、不删既有注释。
4. 每完成一个可验证的功能，**提醒本人去提交**（agent 不碰 git，见 §8）。
5. 修改过工程状态后，同步更新本文件 §3。
