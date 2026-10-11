# Seekfree TC264 双摄智能车工程

基于**逐飞 TC264 开源库 V3.4.6** 的摄像头循迹智能车：双 MT9V03X（总钻风）灰度摄像头采集 → 大津法自适应二值化 → 边界跟随寻线 → 路口（左/右转）识别 → 中线偏差 → 位置 PD 方向环 + 增量式 PID 速度环 → 双电机差速 PWM；外围配 IMU660RB 陀螺仪、IPS114 屏幕与 VOFA+/逐飞助手调参。

- **主控**：Infineon AURIX **TC264D**（TC26xB，双核 CPU0 / CPU1）
- **视觉**：MT9V03X ×2（总钻风，188 × 120 @ 50 fps，8 位并口 + 场中断 + DMA 搬运）
- **控制**：5 ms 速度环（增量式 PID）、10 ms 方向环（位置 PD）+ 路口状态机
- **调试**：IPS114 实时叠加图像/边界/中线/参数，VOFA+ 与逐飞助手波形，串口在线调 PID
- **开发环境**：AURIX Development Studio **v1.10.2**（TASKING，`cctc -Ctc26xb`）

> ⚠️ 当前代码**还不能直接编译链接**（存在未定义符号），详见[已知问题](#已知问题)；`Debug/` 里的 `.hex`/`.elf` 是 2026-07-25 的旧产物。

## 目录结构

```text
├── code/                 用户应用代码
│   ├── image.c/.h        图像处理：Otsu 二值化、边界跟随、路口判断、屏幕叠加
│   ├── motor.c/.h        电机、编码器、速度环 PID、位置 PD、控制调度
│   ├── connect.c/.h      路口状态机与路径表（path_driver，尚未接入 motor.c）
│   ├── IMU.c/.h          IMU660RB 初始化、零漂校准、偏航角积分
│   ├── vofa.c/.h         调试串口、FIFO、上位机数据解析、逐飞助手
│   ├── justfloat.c/.h    VOFA+ JustFloat 协议
│   └── buzzer.c/.h       蜂鸣器（P33_10）
├── user/                 工程与中断
│   ├── cpu0_main.c       CPU0 主函数：外设初始化 + 图像处理主循环
│   ├── cpu1_main.c       CPU1 主函数（空工程模板）
│   ├── isr.c             全部中断服务函数（PIT / ERU / DMA / UART）
│   └── isr_config.h      中断优先级与服务者配置
├── libraries/            逐飞开源库 V3.4.6（zf_common / zf_components / zf_driver / zf_device + Infineon iLLD）
├── Debug/                ADS 构建输出（makefile、.o/.d、.elf、.hex、.map）
├── .project / .cproject  ADS 工程文件
├── Lcf_Tasking_Tricore_Tc.lsl          链接脚本（内存段分配）
├── 推荐IO分配.txt / 尽量不要使用的引脚.txt   库自带引脚参考（非本工程实际接线）
├── AURIX修改工程名称.bat / 删除临时文件.bat    工程改名 / 清理构建临时文件
└── code/本文件夹作用.txt                 约定：新增源码直接放在 code/ 下
```

## 快速开始

1. **导入**：ADS → `File` → `Import` → `General` → `Existing Projects into Workspace`，选择本目录。
2. **编译**：`Project` → `Build Project`（或工具栏锤子），产物为 `Debug/Seekfree_TC264_DualCam_Opensource_Library.hex`。
3. **烧录**：用 AURIX Flasher 或 ADS 调试器下载，目标芯片选 **TC264D**。

常见清理：`删除临时文件.bat` 删除 `Debug/`、`.ads/`、`*.launch`；`AURIX修改工程名称.bat` 一键改工程名。

## 软件设计

**双核分工**：CPU0 负责全部外设初始化、主循环图像处理，并承担全部 PIT / 摄像头场中断 / DMA 完成中断 / 串口中断；CPU1 目前是空工程模板（关看门狗、开总中断后空转），可用来承接第二摄像头处理或高频运算。

**图像流水线**

```text
摄像头场中断(ERU) → DMA 搬运整帧 → mt9v03x_finish_flag_1
  → CPU0 主循环: Otsu 自适应阈值 → 二值化 → 边界跟随(搜左右边界)
  → 中线 mid_line[] → 路口特征统计 → IPS114 显示
  → Deviation() 中线偏差 → 位置 PD → 左右轮目标速度差
  → 增量式 PID → PWM（17 kHz）
```

**周期任务**（优先级等细节见 `user/isr_config.h`）

| 中断 | 周期 | 内容 |
| --- | --- | --- |
| `CCU60_CH0` | 5 ms | 编码器读取、里程计算、蜂鸣器节拍、`car_start++`；≥600 后执行速度环 `circle_control()` |
| `CCU60_CH1` | 10 ms | 方向环 `line_control()` + 路口状态机 |
| `CCU61_CH0` | 5 ms | 陀螺仪采集与偏航角积分（`CCU61_CH1` 未启用） |
| 摄像头场中断 + DMA | 每帧 | 触发采集与整帧搬运，置帧完成标志 |

上电后 `IMU_init()` 会阻塞约 500 ms 做零漂校准；`car_start` 计到 600（5 ms × 600 ≈ **3 秒**）进入闭环跑车，可当作发车延时。

## 接线与引脚

**双 MT9V03X 摄像头**（定义见 `libraries/zf_device/zf_device_mt9v03x_double.h`）

| 信号 | 摄像头 1 | 摄像头 2 |
| --- | --- | --- |
| 8 位数据 D0~D7 | 自 `P00_0` 起连续 8 位 | 自 `P02_0` 起连续 8 位 |
| VSYNC（场中断） | `P33_7`（ERU CH4） | `P10_3`（ERU CH3） |
| PCLK（触发 DMA） | `P14_3`（ERU CH1） | `P10_2`（ERU CH2） |
| SCCB SCL / SDA | `P33_13` / `P32_4` | `P13_1` / `P13_0` |
| DMA 通道 | ch7 | ch6 |

**电机、编码器与其余外设**（定义见 `code/motor.c`、`code/buzzer.h` 及各库头文件）

| 用途 | 引脚 / 资源 |
| --- | --- |
| 左电机方向 / PWM | `ATOM3_CH6_P11_11` / `ATOM2_CH7_P11_12` |
| 右电机方向 / PWM | `ATOM2_CH4_P11_9` / `ATOM3_CH5_P11_10` |
| 左编码器 | `TIM4_ENCODER`：`P02_8` / `P00_9` |
| 右编码器 | `TIM6_ENCODER`：`P20_3` / `P20_0` |
| IMU660RB | 硬件 `SPI_0`：SCK `P20_11`、MOSI `P20_14`、MISO `P20_12`、CS `P20_13` |
| IPS114 屏幕 | 硬件 `SPI_2`：SCK `P15_3`、MOSI `P15_5`、CS `P15_2`、DC `P15_0`、RST `P15_1`、BLK `P15_4` |
| 蜂鸣器 | `P33_10` |
| 串口 | UART0 `P14_0`/`P14_1` 115200（调试 / VOFA+ / 逐飞助手）；UART1 摄像头参数配置；UART2 无线转串口；UART3 GNSS |

**避坑**：boot 引脚 `P14_2`、`P14_4`、`P14_5`、`P14_6`、`P10_5`、`P10_6` 尽量不要用（注意摄像头 1 的 PCLK 恰在 `P14_3`）；`P20_2` 只能输入；`P21_6` 在 TC264**DA** 上不可用。工程内所有中断优先级必须互不相同。

## 主要参数

| 参数 | 当前值 | 位置 |
| --- | --- | --- |
| 基础速度 | 循迹 70（`motor.c`）；转向 300（`connect.c` `BASE_SPEED`） | `line_control()` / `path_driver()` |
| 位置 PD | 初始 `Kp=1.0`、`Kd=1.0`、限幅 30；`path_driver()` 内改为 `1.1 / 1.2`、限幅 50 | `PositionPid_Init()` / `code/connect.c` |
| 速度环 PID | 左 `5.0 / 0.05 / 0`（±1200）、右 `6.0 / 0.008 / 0`（±1500） | `Pid_Init()` |
| PWM | 17 kHz，上限 2000，起步补偿 左 800 / 右 850 | `code/motor.c` |
| 方向偏差死区 | ±2.0 | `line_control()` |
| 转向差速 / 退出 | 左转按路口序号取 ×1.9 / 2.5 / 2.4 / 2.1，右转 ×2.25 / 2.42；`turn_tick ≥ 12` 且陀螺仪转角变化 ≥85° 退出，随后 cooldown 4 拍 | `code/connect.c` |
| 里程换算 | `total_encoder / 5000` | `Encoder_Data_distance()` |
| 测速低通 | `0.5 × 旧值 + 0.5 × 新值` | `Encoder_Data_Get()` |
| 图像 | 188 × 120；Otsu 阈值搜索 5~244；搜线窗口 ±10 | `code/image.c` |
| 偏差计算 | 第 90~50 行中线偏差均值 ÷ 1.1 | `Deviation()` |

## 调试

- **屏幕**：`ips114_screen()` 显示阈值、左右基点、编码器速度、里程、路口判定；`ips114_show_line()` 用蓝/红/黄叠加左边界、右边界与中线。
- **波形**：UART0（115200）走 JustFloat 接 VOFA+；`seekfree_send_speed_pid_wave()` 以 6 通道接逐飞助手（左右目标速度、编码器速度、PID 输出）。
- **在线调参**：串口按 `P1=1.2!` 格式发送，`USART_PID_Adjust()` 解析 `P1/P2/P3`、`I1/I2/I3`、`D1/D2/D3`（角度环 / 角速度环 / 速度环）。注意解析结果只写进 `NUM`，尚未接回实际 PID 结构体。

调参建议：先开环给固定 PWM 验证电机方向与编码器极性，再调位置 PD 的 `Kp/Kd`，最后调转向差速与退出条件。

## 二次开发

1. 新增源码按 `code/本文件夹作用.txt` 的约定直接放进 `code/`，不要再建子文件夹。
2. 核间内存：用 `#pragma section all "cpu0_dsram"` … `#pragma section all restore` 把变量放进对应核心的 RAM（CPU1 用 `cpu1_dsram`）。
3. 新增中断需在 `user/isr_config.h` 里分配互不相同的优先级，并在 `user/isr.c` 用 `IFX_INTERRUPT` 实现；TC 系列进入中断后硬件自动关中断，需要嵌套时在 ISR 内调用 `interrupt_global_enable(0)`。
4. 帧率不足时，可把算法代码移入 CPU0 RAM（`#pragma section all "cpu0_psram"`）或把部分处理交给 CPU1。

## 已知问题

1. **还不能编译链接**，当前阻塞点：
   - `code/IMU.c:43`：函数定义后多一个 `/`，语法错误（尚未修）。
   - `code/connect.c` 的语法问题已修复（`driver_state` 类型名、`switch` 内的 `case`、单一 `turn_tick`、无孤立注释符）；但 `code/motor.c:272` 仍在调用 `state_machine()`，而 `connect.c` 现在只提供 `path_driver()`，需要改调用点或补一个 `state_machine()` 包装。
   - `road_order_cache`、`road_weighted_error_cache`（`code/image.h:26-27` 声明）被 `connect.c:114/130/137` 使用，但全工程没有定义，需要补上赋值逻辑，否则路口匹配永远不成立。
   - `Image_Use`、`cut_image()`、`image_busy`、`image_ready` 只声明未定义，被 `user/cpu0_main.c` 使用。
2. `circle_control()` 中 `pwm_L` 算出后未使用，`Motor_SetSpeed((int)out_L, pwm_R)` 左轮传的是**未限幅的 PID 原始输出**，左右不对称。
3. `code/buzzer.h` 注释写 ticks 以 10 ms 为单位，但 `buzzer_tick()` 实际在 **5 ms** 的 PIT 中调用，响铃时间是注释值的一半。
4. `code/IMU.c` 陀螺仪灵敏度写死 `14.3f`，而同处注释标注 IMU660 应为 16.4（本工程用的正是 IMU660RB）；且积分出的角度只被计算、未接入转向补偿。
5. `code/vofa.c` 的 `USART_PID_Adjust()` 解析出的 `NUM` 未接回 PID 结构体，在线调参目前不起作用。
6. 双摄库已就绪，但当前只初始化并采集**摄像头 1**（`mt9v03x_double_init(mt9v03x_1)`）；摄像头 2 的引脚与 DMA 通道已配置，可直接接入（注意库说明：DMA 只有一个，双摄同时输出时帧率减半）。
7. 工程源码已统一为 **UTF-8（无 BOM）**；若日后从逐飞官方更新 `libraries/`（官方为 GBK），会出现编码混用，需整体再转一次。

## 开源协议

底层为逐飞科技 TC264 开源库 V3.4.6，采用 **GPL-3.0**：许可证副本见 `libraries/LICENSE`，英文声明见 `libraries/doc/GPL3_permission_statement.txt`，版本记录见 `libraries/doc/version.txt`。使用时请保留「成都逐飞科技有限公司」的版权声明；上层应用代码（`code/`、`user/`）随工程一同以 GPL-3.0 发布。

参考资料：[逐飞科技店铺](https://seekfree.taobao.com/) · [GNU GPL-3.0](https://www.gnu.org/licenses/) · [VOFA+](https://www.vofa.plus/) · [AURIX 改工程名教程](https://www.bilibili.com/video/BV1nBkMYMEwE)
