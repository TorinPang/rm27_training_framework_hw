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
