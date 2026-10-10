# Seekfree TC264 双摄智能车工程

> 基于 **逐飞科技 TC264 开源库 V3.4.6**（Infineon AURIX TC264D / TC26xB，双核）搭建的摄像头循迹智能车工程。
>
> 主线功能：**MT9V03X（总钻风）灰度摄像头采集 → 大津法二值化 → 赛道边界跟随 → 路口（左/右转）识别 → 中线偏差 → 位置 PD 方向环 + 增量式 PID 速度环 → 双电机差速 PWM**；
> 外围配套：IMU660RB 陀螺仪、IPS114 屏幕实时显示、蜂鸣器、VOFA+ / 逐飞助手 上位机调参、GNSS/无线串口驱动预留。

| 项目 | 说明 |
| --- | --- |
| 工程名 | `Seekfree_TC264_DualCam_Opensource_Library` |
| 主控 | Infineon AURIX **TC264D**（TC26xB，双核 CPU0/CPU1） |
| 开发环境 | **AURIX Development Studio (ADS) v1.10.2** + TASKING C/C++ 编译器（`cctc -Ctc26xb`） |
| 底层库 | 逐飞 TC264 开源库 **V3.4.6**（见 `libraries/doc/version.txt`） |
| 构建产物 | `Debug/Seekfree_TC264_DualCam_Opensource_Library.hex` / `.elf` / `.map` |
| 开源协议 | GPL-3.0（`libraries/LICENSE`、`libraries/doc/GPL3_permission_statement.txt`） |

> ⚠️ **先读这一条**：仓库里的源码与最近一次成功编译的产物**不一致**，当前工作区代码**不能直接编译通过**（存在语法错误与未定义符号）。
> 详见文末 [第 10 节 · 当前状态与已知问题](#10-当前状态与已知问题)。

---

## 目录

1. [功能概览](#1-功能概览)
2. [目录结构](#2-目录结构)
3. [软件架构与控制流程](#3-软件架构与控制流程)
4. [硬件接线与引脚分配](#4-硬件接线与引脚分配)
5. [关键模块说明](#5-关键模块说明)
6. [图像算法说明](#6-图像算法说明)
7. [参数速查表](#7-参数速查表)
8. [调试与上位机](#8-调试与上位机)
9. [编译与烧录](#9-编译与烧录)
10. [当前状态与已知问题](#10-当前状态与已知问题)
11. [二次开发须知](#11-二次开发须知)
12. [开源协议与致谢](#12-开源协议与致谢)

---

## 1. 功能概览

| 功能 | 实现位置 | 说明 |
| --- | --- | --- |
| 双摄硬件支持 | `libraries/zf_device/zf_device_mt9v03x_double.*` | 库自带双 MT9V03X 驱动（188×120 @ 50 fps）。**当前工程仅初始化并采集摄像头 1**（`mt9v03x_double_init(mt9v03x_1)`），摄像头 2 的引脚与 DMA 通道已配置好，可直接接入 |
| 图像采集 | `user/isr.c` + 库 DMA | 场中断（ERU）触发、PCLK 触发 DMA 搬运整帧，采集完成置 `mt9v03x_finish_flag_1` |
| 图像处理 | `code/image.c` | 大津法（Otsu）自适应阈值 → 二值化 → 左右边界跟随（迷宫式搜线）→ 中线 → 路口特征统计 |
| 方向控制 | `code/motor.c` `line_control()` | 中线偏差 → 位置 PD → 左右轮目标速度差 |
| 速度控制 | `code/motor.c` `circle_control()` | 增量式 PID 速度环，5 ms 执行一次，输出 PWM |
| 路口状态机 | `code/connect.c` | 直行 / 左转 / 右转 状态切换（**当前为未完成的半成品代码**） |
| 姿态感知 | `code/IMU.c` | IMU660RB 陀螺仪零漂校准 + Z 轴角速度积分（0°~360° / ±180° / 累计角三种规划） |
| 人机交互 | `code/vofa.c`、`code/justfloat.c`、`code/buzzer.c` | VOFA+ JustFloat 波形、串口在线调 PID、逐飞助手示波器、蜂鸣器提示 |
| 屏幕显示 | `code/image.c` `ips114_screen()` / `ips114_show_line()` | 图像、阈值、左右边界、中线、编码器速度、距离等实时叠加显示 |

---

## 2. 目录结构

```text
Seekfree_TC264_DualCam_Opensource_Library/
├── code/                          # ★ 用户应用代码（本工程核心）
│   ├── image.c / image.h          #   图像处理：Otsu、二值化、边界跟随、路口判断、屏幕叠加
│   ├── motor.c / motor.h          #   电机、编码器、增量式 PID 速度环、位置 PD、控制调度
│   ├── connect.c / connect.h      #   路口/路段状态机与路径表（半成品）
│   ├── IMU.c / IMU.h              #   IMU660RB 初始化、零漂校准、偏航角积分
│   ├── vofa.c / vofa.h            #   调试串口、FIFO、上位机数据解析、逐飞助手示波器
│   ├── justfloat.c / justfloat.h  #   VOFA+ JustFloat 协议（float 转 4 字节 + 帧尾）
│   ├── buzzer.c / buzzer.h        #   蜂鸣器（P33_10）
│   └── 本文件夹作用.txt            #   约定：新增 .c 直接放在此目录，不要再建子文件夹
├── user/                          # ★ 工程/中断相关
│   ├── cpu0_main.c                #   CPU0 主函数：外设初始化 + 图像处理主循环
│   ├── cpu0_main.h                #   Infineon 模板头文件
│   ├── cpu1_main.c                #   CPU1 主函数（空工程模板，可接管第二个核心的任务）
│   ├── isr.c                      #   全部中断服务函数（PIT / ERU / DMA / UART）
│   ├── isr.h                      #   中断相关全局变量声明
│   └── isr_config.h               #   中断优先级与服务者（CPU0/CPU1/DMA）配置
├── libraries/                     # 逐飞开源库 V3.4.6
│   ├── zf_common/                 #   公共层（类型、时钟、延时、调试串口、FIFO 等）
│   ├── zf_components/             #   组件层（printf 重定向、逐飞助手、正弦表等）
│   ├── zf_driver/                 #   驱动层（GPIO/SPI/UART/PWM/编码器/EXTI/PIT…）
│   ├── zf_device/                 #   外设层（MT9V03X 双摄、IMU660RB、IPS114、GNSS…）
│   ├── infineon_libraries/         #   Infineon iLLD / TC26B SDK
│   └── doc/                       #   version.txt、GPL 许可说明
├── Debug/                         # ADS 构建输出（makefile、.o/.d、.elf、.hex、.map）
├── .project / .cproject           # Eclipse / ADS 工程描述文件
├── Lcf_Tasking_Tricore_Tc.lsl     # TASKING 链接脚本（内存段分配）
├── 推荐IO分配.txt / 尽量不要使用的引脚.txt   # 库自带引脚参考（非本工程实际接线，见 4.5 节）
├── AURIX修改工程名称.bat           # 一键改工程名（同时清理临时文件）
├── AURIX修改工程名称教程.txt
└── 删除临时文件.bat                # 删除 Debug/、.ads/、*.launch
```

---

## 3. 软件架构与控制流程

### 3.1 双核分工

| 核心 | 入口 | 职责 |
| --- | --- | --- |
| CPU0 | `user/cpu0_main.c` → `core0_main()` | 初始化全部外设；主循环中等待摄像头帧完成标志，执行图像处理与 IPS114 显示；承担全部 PIT 中断、摄像头场中断、DMA 完成中断与串口中断（ERU CH1/CH5、CH2/CH6 即两路 PCLK 的服务者被配置为 DMA，见 `user/isr_config.h`） |
| CPU1 | `user/cpu1_main.c` → `core1_main()` | 目前为空工程模板（关看门狗、开总中断后空转），可把第二摄像头处理或高频运算搬到这里 |

```c
// user/cpu0_main.c（CPU0 初始化顺序）
clock_init();                       // 时钟
debug_init();                       // 调试串口重定向
Init_uart();                        // UART0 115200（调试 / VOFA）
ips114_init();                      // IPS114 屏幕
mt9v03x_double_init(mt9v03x_1);     // 双摄库，当前仅初始化摄像头 1
buzzer_init();                      // 蜂鸣器
Encoder_Init();                     // 编码器（TIM4 左 / TIM6 右）
Encoder_Data_Get_Init();            // 编码器数据清零
motor_init();                       // 4 路 PWM 初始化
Pid_Init();                         // PID 参数初始化
IMU_init();                         // IMU660RB + 零漂校准（约 500 ms）
pit_ms_init(CCU60_CH0, 5);          // 5 ms：编码器 / 距离 / 蜂鸣器 / 速度环
pit_ms_init(CCU61_CH0, 5);          // 5 ms：陀螺仪采集与角度积分
pit_ms_init(CCU60_CH1, 10);         // 10 ms：方向环 + 状态机
cpu_wait_event_ready();             // 等待所有核心初始化完成
```

> 初始化中的 `IMU_init()` 内部会阻塞约 100 × 5 ms ≈ 500 ms 完成零漂采样。

### 3.2 中断与周期任务

| 中断源 | 优先级 | 服务者 | 周期 | 处理内容 |
| --- | --- | --- | --- | --- |
| `CCU60_CH0` | 30 | CPU0 | 5 ms | `Encoder_Data_Get()`、`Encoder_Data_distance()`、`buzzer_tick()`、`car_start++`；`car_start ≥ 600` 后调用 `circle_control()`（速度环） |
| `CCU60_CH1` | 31 | CPU0 | 10 ms | `car_start ≥ 600` 后调用 `line_control()`（方向环 + 状态机） |
| `CCU61_CH0` | 32 | CPU0 | 5 ms | `gyro_Offset_flag == 1` 时执行 `imu660rb_get_gyro()` + `IMU_YAW_integral()` |
| `CCU61_CH1` | 33 | CPU0 | 未启动 | 空（预留，主函数未 `pit_ms_init`） |
| `ERU CH4` = P33_7 | 255 | CPU0 | 摄像头 1 场中断 | `camera_vsync_handler_1()` 启动一帧 DMA 采集 |
| `ERU CH3` = P10_3 | 254 | CPU0 | 摄像头 2 场中断 | `camera_vsync_handler_2()`（当前未采集摄像头 2） |
| `ERU CH1` = P14_3 | 7 | **DMA** | 摄像头 1 PCLK | 触发 DMA 搬运行数据（ISR 体为空） |
| `ERU CH2` = P10_2 | 6 | **DMA** | 摄像头 2 PCLK | 触发 DMA 搬运行数据（ISR 体为空） |
| `DMA ch7` / `ch6` | 0 | CPU0 | 每帧 | `camera_dma_handler_1()` / `camera_dma_handler_2()` 置采集完成标志 |
| `UART0 TX` | 11 | CPU0 | — | `uart_rx_interrupt_handler()`：查询收到的字节并写入调参 FIFO（库模板如此安排） |
| `UART0 RX` | 10 | CPU0 | — | `debug_interrupr_handler()`：调试串口接收（环形缓冲，`DEBUG_UART_USE_INTERRUPT` 打开时生效） |
| `UART1 RX` | 14 | CPU0 | — | `camera_uart_handler_1()` 摄像头 1 参数配置 |
| `UART2 RX` | 17 | CPU0 | — | `uart_rx_interrupt_handler()`（无线转串口；库模板另有 `wireless_module_uart_handler()` 已注释） |
| `UART3 RX` | 20 | CPU0 | — | `gnss_uart_callback()`（GNSS 定位模块） |
| `UART0~3 ER` | 12/15/18/21 | CPU0 | — | `IfxAsclin_Asc_isrError()` 通讯错误处理 |

其他说明：

- `isr.c` 中保留了库模板的 `ERU CH0/CH2/CH5/CH6/CH7` 分支（如 `imu660rc_callback()`、空 PCLK 分支），本工程未使用；CH2/CH6 的中断服务者被配置为 DMA，其 ISR 体不会被执行。
- 库要求**所有中断优先级互不相同**（见 `user/isr_config.h` 顶部说明），新增中断时务必避让上表数值。
- TASKING/TC 系列默认不支持中断嵌套，`isr.c` 中每个 ISR 首行都调用 `interrupt_global_enable(0)` 打开嵌套。
- `isr_config.h` 中 DMA 中断优先级配置为 `0`，按库注释 0 表示该优先级不使能，移植时请结合库说明复核。

### 3.3 起跑时序（软启动）

```text
上电 → 初始化(约 0.5 s 陀螺仪校准) → cpu_wait_event_ready()
     → 主循环持续采集/处理图像并刷新屏幕
     → CCU60_CH0 每 5 ms: car_start++
     → car_start ≥ 600（≈ 3 s 后）→ 同时启用 速度环(5 ms) 与 方向环(10 ms)
```

即上电后约 **3 秒**进入闭环跑车，方便放车、也可用作发车延时。

### 3.4 数据流

```text
摄像头1 场中断 ──► DMA 搬运整帧 ──► mt9v03x_finish_flag_1 = 1
                                            │
              CPU0 主循环检测到标志 ◄────────┘
                                            │
        image_invocation()                  ▼
          ├─ Otsu(mt9v03x_image_1)      自适应阈值 threhold
          ├─ BinaryZation(threhold)      二值化 → imageindex[120][188]
          ├─ follow_bianjie()            左右边界跟随 → left/right_line_save[], mid_line[]
          └─ judge_num_where()           路口特征统计 up/down/left/right.road[]
                                            │
        ips114_screen() / ips114_show_line() 显示
                                            │
        ┌───────────────────────────────────┴────────────────────────────┐
        │ 5 ms   circle_control(): 目标速度 ← 增量式 PID → PWM 输出         │
        │ 10 ms  line_control():  Deviation() → 位置 PD → 左右轮目标速度差  │
        │        └─ state_machine()（connect.c）: 直行/左转/右转状态切换      │
        └────────────────────────────────────────────────────────────────┘
```

---

## 4. 硬件接线与引脚分配

> 以下引脚均取自代码与库头文件中的实际宏定义，可直接对照接线。

### 4.1 双 MT9V03X 摄像头（8 位并口 + 场中断 + PCLK + SCCB 配置口）

| 信号 | 摄像头 1 | 摄像头 2 |
| --- | --- | --- |
| 8 位数据 D0~D7 | 自 `P00_0` 起连续 8 位 | 自 `P02_0` 起连续 8 位 |
| VSYNC（场中断） | `P33_7`（ERU CH4） | `P10_3`（ERU CH3） |
| PCLK（触发 DMA） | `P14_3`（ERU CH1） | `P10_2`（ERU CH2） |
| SCCB/IIC SCL | `P33_13` | `P13_1` |
| SCCB/IIC SDA | `P32_4` | `P13_0` |
| DMA 通道 | `IfxDma_ChannelId_7` | `IfxDma_ChannelId_6` |
| 分辨率 / 帧率 | 188 × 120 / 50 fps | 188 × 120 / 50 fps |
| 曝光时间 | 400 | 512 |
| 增益 / 自动曝光 | 32 / 关闭 | 32 / 关闭 |

定义位置：`libraries/zf_device/zf_device_mt9v03x_double.h`；中断回调见 `user/isr.c`。

### 4.2 电机与编码器

| 用途 | 引脚 / 资源 | 定义位置 |
| --- | --- | --- |
| 左电机方向 | `ATOM3_CH6_P11_11` | `code/motor.c` |
| 左电机 PWM | `ATOM2_CH7_P11_12` | `code/motor.c` |
| 右电机方向 | `ATOM2_CH4_P11_9` | `code/motor.c` |
| 右电机 PWM | `ATOM3_CH5_P11_10` | `code/motor.c` |
| 左编码器 | `TIM4_ENCODER`：CH1 `P02_8`、CH2 `P00_9` | `code/motor.c` `Encoder_Init()` |
| 右编码器 | `TIM6_ENCODER`：CH1 `P20_3`、CH2 `P20_0` | `code/motor.c` `Encoder_Init()` |

PWM 载波 17 kHz，占空比上限 `max_pwm = 2000`；正负号即正反转，`Motor_SetSpeed(pwm1, pwm2)` 中 **pwm1 → 左轮、pwm2 → 右轮**。

### 4.3 IMU、屏幕、蜂鸣器

| 模块 | 接口与引脚 | 定义位置 |
| --- | --- | --- |
| IMU660RB | 硬件 `SPI_0`：SCK `P20_11`、MOSI `P20_14`、MISO `P20_12`、CS `P20_13`，10 MHz | `libraries/zf_device/zf_device_imu660rb.h`（`IMU660RB_USE_SOFT_IIC = 0`） |
| IPS114 屏幕 | 硬件 `SPI_2`：SCK `P15_3`、MOSI `P15_5`、MISO `P15_4`(占位)、CS `P15_2`、DC `P15_0`、RST `P15_1`、BLK `P15_4`，60 MHz | `libraries/zf_device/zf_device_ips114.h` |
| 蜂鸣器 | `P33_10` | `code/buzzer.h` `BUZZ_PIN` |

### 4.4 串口分配

| 串口 | 引脚 | 波特率 | 用途 |
| --- | --- | --- | --- |
| UART0 | TX `P14_0`、RX `P14_1` | 115200 | 调试串口 / VOFA+ / 逐飞助手（`code/vofa.c`、`code/justfloat.c`） |
| UART1 | — | — | 摄像头 1 参数配置（`camera_uart_handler_1()`） |
| UART2 | — | — | 无线转串口模块（可切换为库的 `wireless_module_uart_handler()`） |
| UART3 | — | — | GNSS 定位模块（`gnss_uart_callback()`） |

UART0 的串口号/波特率/引脚由 `libraries/zf_common/zf_common_debug.h` 中的 `DEBUG_UART_*` 宏决定。

### 4.5 关于根目录的两个引脚说明文件

`推荐IO分配.txt` 与 `尽量不要使用的引脚.txt` 是**逐飞开源库自带的通用参考**，其中提到的 ICM20602、TFT180、舵机等并不完全对应本工程（本工程实际使用 IMU660RB + IPS114）。**接线以 4.1~4.4 与代码为准。**

`尽量不要使用的引脚.txt` 的注意事项仍然有效：

- 避免使用 boot 相关引脚：`P14_2`、`P14_3`、`P14_4`、`P14_5`、`P14_6`、`P10_5`、`P10_6`
  （注意：本工程把**摄像头 1 的 PCLK 放在了 `P14_3`**，属于 boot 引脚，硬件上如出现启动异常可优先怀疑此处）
- `P20_2` 只能输入、不能输出
- `P21_6` 在 TC264**DA** 芯片上不可用，在 TC264D 上可用

---

## 5. 关键模块说明

### 5.1 `code/image.c` / `image.h` —— 图像处理

| 函数 | 说明 |
| --- | --- |
| `uint8 Otsu(uint8 img[H][W])` | 大津法求全局阈值，遍历 `i = 5..245` 取最大类间方差，返回阈值 |
| `void BinaryZation(uint8 value)` | 按阈值二值化为 0 / 255，结果存 `imageindex[120][188]` |
| `void follow_bianjie(...)` | 迷宫式边界跟随：底部基行找最长白段定左右基点，再逐行在上一行边界 ±`SEARCH_RANGE` 窗口内搜跳变 |
| `float Deviation()` | 第 90 行到第 50 行中线相对图像中心列的平均偏差 ÷ 1.1 |
| `void judge_num_where()` | 统计前进/左/右三个方向的跳变数与坐标，结果入 `up/down/left/right.road[]` |
| `int judge_where()` | 综合判断路口：返回 `3` 左转、`4` 右转、`0` 直行；转向时用 `calculate_deviation()` 写入 `weighted_error` |
| `float calculate_deviation(x1,y1)` | 由指定点与底行中点连线，求其在图像中央行处的横向偏差 |
| `int judge_up_if_all_black()` | 前方中央是否大片黑（辅助判断） |
| `judge_where_better_left()` / `jude_where_better_right()` | 左右侧连续白点校验，减少误判（分别返回 6 / 7） |
| `void image_invocation()` | 图像处理总入口：Otsu → 二值化 → 边界跟随 → 路口统计 |
| `void ips114_screen()` / `ips114_show_line()` | 屏幕叠加显示文本信息 / 左右边界与中线 |

全局数据：`imageindex`（二值图）、`left_line_save[]`、`right_line_save[]`、`mid_line[]`、`threhold`、`left_jidian`/`right_jidian`、`weighted_error`。

### 5.2 `code/motor.c` / `motor.h` —— 电机与闭环

| 函数 | 说明 |
| --- | --- |
| `Encoder_Init()` / `Encoder_Data_Get_Init()` | 编码器方向计数初始化 / 数据清零 |
| `Encoder_Data_Get()` | 读原始计数（左轮取负），一阶低通 `speed = 0.5*old + 0.5*raw`，累计 `total_encoder` |
| `Encoder_Data_distance()` | `distance_L/R = total_encoder / 5000`，`distance` 取左右均值 |
| `motor_init()` / `Motor_SetSpeed(pwm1,pwm2)` | PWM 初始化（17 kHz）/ 设定左右轮占空比，含 ±`max_pwm` 限幅与方向脚控制 |
| `Increase_pid_init()`、`Speed_pid_left/right()` | 增量式 PID：`Δu = Kp·(e_k-e_{k-1}) + Ki·e_k + Kd·(e_k-2e_{k-1}+e_{k-2})`，带输出限幅 |
| `Motor_Start_Compensate()` | 起步补偿：编码器接近 0 时给最小 PWM，避免 PID 起步力矩不足 |
| `PositionPID_calculate()` | 位置式 PD：`u = Kp·e − Kd·v`，带输出限幅 |
| `line_control()` | 方向环（10 ms）：偏差 ±2 内直行；转向状态时交出控制权；否则位置 PD → 左右轮目标速度 |
| `circle_control()` | 速度环（5 ms）：左右增量式 PID → `Motor_SetSpeed()` |

### 5.3 `code/connect.c` / `connect.h` —— 路口状态机（半成品）

- `driver_state` 枚举：`follow_state=0`（直行循迹）、`turn_left_state=3`、`turn_right_state=4`、`turn_back_state=5`
- `route_table[]`：路段→状态映射表，元素含 `road_type` / `state` / `distance_min` / `distance_max`
- `state_machine()`：转向时按 `0.7 × |weighted_error|`（左）或 `1.0 × |weighted_error|`（右）做左右轮差速，满足退出条件（偏差回到 ±6 内或超时 `TURN_MAX_TICK=110`）后回到直行
- **该文件当前存在语法与类型错误，无法编译**，修复建议见 [第 10 节](#10-当前状态与已知问题)

### 5.4 `code/IMU.c` / `IMU.h` —— 姿态

| 函数 | 说明 |
| --- | --- |
| `IMU_init()` | `imu660rb_init()` + 零漂校准 |
| `IMU_gyro_Offset_Init()` | 采样 100 次求均值作零漂，返回 `gyro_Offset_flag = 1` |
| `IMU_GetValues()` | 原始角速度减零漂，再除以灵敏度（代码写死 `14.3f`，与注释标注的 IMU660 → 16.4 不一致，见 [10.2](#102-逻辑与实现瑕疵)）换算为 rad/s |
| `IMU_YAW_integral()` | 5 ms 周期积分；`|gyro_z| < 0.015` 时视为静止不积分（去零漂抖动） |
| `IMU_Handle_180()` / `IMU_Handle_360()` / `IMU_Handle_0()` | 角度规划：±180°、±360°、累计角（`T_N`） |

### 5.5 `code/vofa.c` / `justfloat.c` —— 调试与上位机

- `Init_uart()`：初始化 UART0 与接收 FIFO
- `uart_rx_interrupt_handler()`：中断中把字节写入 FIFO
- `Get_Data()` / `USART_PID_Adjust()`：解析上位机 `...=值!` 格式的调参指令，支持 `P1/P2/P3`、`I1/I2/I3`、`D1/D2/D3`（角度环 / 角速度环 / 速度环）
- `seekfree_send_speed_pid_wave()`：逐飞助手示波器 6 通道（左右轮目标速度、编码器速度、PID 输出）
- `justfloat.c`：VOFA+ JustFloat 协议，`Float_to_Byte()` 将 float 拆成 4 字节，帧尾固定 `00 00 80 7F`；提供 1/2/3 通道发送函数

### 5.6 `code/buzzer.c` / `buzzer.h`

`buzzer_beep(ticks)` 打开蜂鸣器并置倒计时，由 5 ms PIT 中的 `buzzer_tick()` 递减，到 0 自动关闭 → 实现无阻塞响铃。

> ⚠️ 头注释写的是“ticks 以 10 ms 为单位”，但 `buzzer_tick()` 实际在 **5 ms** 的 `CCU60_CH0` PIT 中调用，**实际 1 tick = 5 ms**（`buzzer_beep(30)` 响 150 ms，而非注释所说的 300 ms）。

---

## 6. 图像算法说明

### 6.1 预处理

1. **大津法自适应阈值** `Otsu()`：统计 256 级灰度直方图，计算累积概率 `PK[]` 与累积一阶矩 `MK[]`，在 `5..244` 内取类间方差最大处作为阈值，能自适应赛道/场地亮度变化。
2. **二值化** `BinaryZation()`：`pixel < threhold → 0（黑/赛道边线）`，否则 `255（白/赛道）`，结果写入 `imageindex`。

### 6.2 边界跟随（搜线）

```text
基行 base_row = start_search_line - 1 = 119
  └─ 从左到右扫描，取最长白段作为左右基点；白段过短则退化为整行 [1, W-2]
逐行向上 i = H-2 … 1
  ├─ 左边界：在 last_left ± SEARCH_RANGE 内找「黑→白」跳变
  ├─ 右边界：在 last_right ± SEARCH_RANGE 内找「白→黑」跳变
  ├─ 找不到则沿用上一行位置（保持连续性，代码中另有拐角不补线的注释版本）
  └─ mid_line[i] = (left + right) / 2
```

相关常量：`SEARCH_RANGE = 10`、`start_search_line = 120`、`end_search_line = 5`、`expand_row = 10`（当前未使用）。

### 6.3 偏差计算

`Deviation()` 对第 90 行到第 50 行的中线与图像中心列之差求平均，再除以 1.1 作为柔化系数：

```c
error = Σ(mid_line[i] - W/2) / (90 - 50) / 1.1;   // W = 188
```

### 6.4 路口识别

- `judge_num_where()` 在三个区域统计黑白跳变：
  - 前进方向：图像中部 9 行，统计「白→黑」跳变
  - 左/右方向：纵向 10 行，统计「黑→白」与「白→黑」跳变并取较大值
- `judge_where()` 依据 `up/left/right.road[]` 组合、跳变点的纵向一致性、以及左右侧连续白点校验判断左/右转，并把转向瞬间的偏差写入 `weighted_error` 供差速使用。
- `image.h` 中的 `road_type_t` 枚举还预留了 `ROAD_CROSS = 8`（十字）、`ROAD_T = 9`（丁字），当前判断逻辑只输出 3/4/0。

---

## 7. 参数速查表

| 参数 | 当前值 | 位置 | 说明 |
| --- | --- | --- | --- |
| 起跑延时 | `car_start ≥ 600` | `user/isr.c` | 5 ms × 600 ≈ **3 s** |
| 速度环周期 | 5 ms | `user/cpu0_main.c` | `CCU60_CH0` |
| 方向环周期 | 10 ms | `user/cpu0_main.c` | `CCU60_CH1` |
| 基础速度 | 70 | `code/connect.c` `BASE_SPEED`；`code/motor.c` `line_control()` 局部变量 | 左右轮目标速度基准 |
| 位置 PD | `Kp=1.0`、`Kd=1.0`、限幅 `30` | `code/motor.c` `PositionPid_Init()` | 越大转向越猛 |
| 速度环 PID 左 | `Kp=5.0`、`Ki=0.05`、`Kd=0`、输出 ±1200 | `code/motor.c` `Pid_Init()` | 增量式 |
| 速度环 PID 右 | `Kp=6.0`、`Ki=0.008`、`Kd=0`、输出 ±1500 | `code/motor.c` `Pid_Init()` | 增量式 |
| 偏差死区 | ±2.0 | `code/motor.c` `line_control()` | 死区内直行 |
| 转向差速系数 | 左 0.7、右 1.0 | `code/connect.c` | 乘 `|weighted_error|` |
| 转向最短/最长节拍 | 20 / 110 | `code/connect.c` | 对应 200 ms / 1100 ms（10 ms 周期） |
| 转向退出偏差 | ±6.0 | `code/connect.c` `TURN_EXIT_ERROR` | 回到死区即退出转向 |
| PWM 上限 | 2000 | `code/motor.c` `max_pwm` | 对应 17 kHz 载波 |
| 起步补偿 PWM | 左 800 / 右 850 | `code/motor.c` `circle_control()` | 编码器接近 0 时生效 |
| 距离换算 | `total_encoder / 5000` | `code/motor.c` `Encoder_Data_distance()` | 量纲由机械结构标定；`connect.c` 用 28~35、48~52 区间触发路段 |
| 测速低通系数 | 0.5 / 0.5 | `code/motor.c` `Encoder_Data_Get()` | 一阶滤波 |
| 图像尺寸 | 188 × 120 | `zf_device_mt9v03x_double.h` | 总钻风全分辨率 |
| 裁剪尺寸 | `CROP_W=168` × `CROP_H=90` | `code/image.h` | 供显示/后续处理 |
| Otsu 搜索范围 | 5 ~ 244 | `code/image.c` | 避免极端阈值 |
| 搜线范围 | ±10 | `code/image.c` `SEARCH_RANGE` | 限幅防止串边界 |
| 陀螺仪灵敏度 | 代码用 14.3 | `code/IMU.c` `IMU_GetValues()` | 同处注释写「IMU660 → 16.4，IMU963 → 14.3」，而本工程用的是 IMU660RB，此处疑似应改为 16.4，请按实测标定 |
| 陀螺仪静止阈值 | 0.015 rad/s | `code/IMU.c` | 低于此值不积分 |
| 蜂鸣器节拍 | 5 ms / tick | `code/buzzer.c` + `user/isr.c` | 与 `buzzer.h` 注释（10 ms）不一致 |

---

## 8. 调试与上位机

### 8.1 IPS114 屏幕

`ips114_screen()` 在屏幕上显示：Otsu 阈值、左右基点、左右编码器速度、距离（左/右/平均）、前方大片黑标志，以及 `up/left/right.road[]` 与 `judge_where()` 的判定结果；`ips114_show_line()` 用蓝/红/黄分别叠加左边界、右边界与中线。这些是现场调试最直接的手段。

### 8.2 VOFA+（JustFloat）

- 使用 UART0（115200），可在 `code/vofa.c` 的 `seekfree_send_speed_pid_wave()` 中把需要观察的变量加入通道（当前 6 通道：左右目标速度、左右编码器速度、左右 PID 输出）。
- `code/justfloat.c` 提供 `JustFloat_Test_one/two/three()` 直接发送 1~3 路 float，帧尾 `00 00 80 7F` 即 VOFA+ 的 JustFloat 协议。

### 8.3 串口在线调参

上位机按 `P1=1.2!` 这类格式发送，`Get_Data()` 解析后由 `USART_PID_Adjust()` 写入 `NUM` 结构体：

| 指令前缀 | 含义 |
| --- | --- |
| `P1/I1/D1` | 角度环 P / I / D |
| `P2/I2/D2` | 角速度环 P / I / D |
| `P3/I3/D3` | 速度环 P / I / D |

> 注意：`USART_PID_Adjust()` 只更新 `NUM` 结构体，`NUM` 到实际 PID 参数的映射需要用户自行在代码中接上（当前工程未把 `NUM` 应用到 `pid_speed_L/R`）。

### 8.4 逐飞助手

`seekfree_send_speed_pid_wave()` 走库的逐飞助手示波器协议（`seekfree_assistant_oscilloscope_*`），配合逐飞助手可直接看速度环跟踪效果。

### 8.5 蜂鸣器

`buzzer_beep(n)` 可用于状态提示（例如进入转向、离开赛道等），不阻塞主循环。

---

## 9. 编译与烧录

### 9.1 环境准备

1. 安装 **AURIX Development Studio**（本工程基于 v1.10.2，TASKING 编译器随 ADS 提供）。
2. 工程名与元器件无需额外配置，`.project` / `.cproject` 已包含 Debug 与 Release 两套配置。

### 9.2 导入与编译

```text
ADS → File → Import → General → Existing Projects into Workspace
     → Select root directory: 选择本仓库根目录 → Finish
ADS → Project → Build Project（或选中工程后点工具栏锤子）
成功后在 Debug/ 下生成 Seekfree_TC264_DualCam_Opensource_Library.elf / .hex
```

命令行方式（已生成 makefile 时）：进入 `Debug/` 目录执行 `make all`（需要 TASKING 工具链在 PATH 中）。

### 9.3 烧录

使用 AURIX Flasher / ADS 内置调试器下载 `Debug/*.hex`（或直接 Debug 运行 `.elf`）。烧录前确认目标芯片型号为 **TC264D**。

### 9.4 常见清理

- 双击 `删除临时文件.bat`：删除 `Debug/`、`.ads/`、`*.launch` 等构建临时文件
- 双击 `AURIX修改工程名称.bat`：按提示修改工程名（同时改写 `.project`/`.cproject` 并清理临时文件）

---

## 10. 当前状态与已知问题

> 本工程的源码目前处于**开发中/未完成**状态：最后一次成功编译的产物时间戳为 **2026-07-25 14:03**，而 `code/image.c`、`code/motor.c`、`code/connect.c`、`code/IMU.c` 的修改时间为 **2026-09-03 20:14**。
> 也就是说 **`Debug/` 里的 `.elf`/`.hex` 来自旧版本代码，与当前源码不对应**；按当前源码编译会在编译/链接阶段报错。

### 10.1 会直接导致编译失败

| # | 位置 | 问题 |
| --- | --- | --- |
| 1 | `code/IMU.c:43` | `void  IMU_GetValues()/` —— 函数定义后多了一个 `/`，语法错误 |
| 2 | `code/connect.c:22` | `Driver_state Go = follow_state;` —— 类型名应为 `driver_state`（`Driver_state` 不存在） |
| 3 | `code/connect.c:25-26` | 孤立文本 `路径表` 后紧跟一个没有配对 `/*` 的 `*/` |
| 4 | `code/connect.c:146-205` | `state_machine()` 中 `case follow_state:` 等 `case` 标签直接写在函数体里，**不在 `switch` 语句内** |
| 5 | `code/connect.c:160` | `if()` 条件为空 |
| 6 | `code/connect.c:23` 与 `:144` | `static uint16 turn_tick = 0;` 重复定义 |
| 7 | 链接阶段 | 以下符号**只有声明、没有定义**：`Image_Use`、`cut_image()`（`image.h:23-24`，被 `user/cpu0_main.c:77` 使用）、`image_busy`、`image_ready`（`motor.h:67-68`，被 `cpu0_main.c:71-75` 使用）、`base_speed`（`motor.h:66`）、`road_order_cache`、`road_weighted_error_cache`（`image.h:26-27`）、`path_index`（`connect.h:25`）、`what_control()`（`connect.h:21`） |

### 10.2 逻辑与实现瑕疵

| # | 位置 | 问题 |
| --- | --- | --- |
| 1 | `code/motor.c:308-310` | `circle_control()` 中 `pwm_L` 算出后未被使用，`Motor_SetSpeed((int)out_L, pwm_R)` 左轮传入的是 PID 原始输出（**未经起步补偿与限幅**），右轮用的是 `pwm_R`，左右不对称 |
| 2 | `code/connect.c:158-163` | 距离区间判断成立时本意应为左转（`order == 3`），却赋值 `turn_right_state`；`route_table[]` 与 `ROUTE_LEN` 定义后从未被使用，`distance_min/distance_max` 也未参与判断 |
| 3 | `code/IMU.c` `IMU_GetValues()` | 灵敏度写死 `14.3f`，但同处注释标注「IMU660 → 16.4」；本工程使用 IMU660RB，灵敏度可能偏小约 13%，会导致积算角度偏小，建议按型号/实测标定 |
| 4 | `code/buzzer.h:19` vs `code/buzzer.c` + `user/isr.c` | 注释称 ticks 以 10 ms 为单位，实际 `buzzer_tick()` 在 5 ms PIT 中调用，响铃时间是注释值的一半 |
| 5 | `code/image.c` `follow_bianjie()` | 拐角不补线的完整版逻辑被注释掉，目前“找不到边界就沿用上一行”，急弯/丢线时可能出现中线跳变；`expand_row` 定义未使用 |
| 6 | `code/vofa.c` `Get_Data()` | 解析仅支持“整数位在前的小数”，未处理科学计数法；`Get_Data()` 依赖调用前已把数据读入 `fifo_get_data` |
| 7 | `code/vofa.c` `USART_PID_Adjust()` | 解析出的 `NUM` 参数未接回实际 PID 结构体，在线调参目前不起作用 |
| 8 | `code/image.c` `Deviation()` | 行区间 90→50 为硬编码，与 `CROP_H=90`、`MT9V03X_1_H=120` 耦合，改分辨率时需同步修改 |
| 9 | `user/isr.c` | 保留了库模板的空中断分支（ERU CH0/CH2/CH5/CH6/CH7），其中 CH0 分支调用的 `imu660rc_callback()` 与本工程使用的 IMU660RB 不一致，属历史残留 |

### 10.3 源码编码不统一（编辑前必看）

`code/`、`user/` 下同时存在 **GBK** 与 **UTF-8** 两种编码的文件，且部分 UTF-8 文件中的中文已被替换为 `U+FFFD`（`�`）——这是早期转码造成的**不可逆丢失**：

| 编码 | 文件 |
| --- | --- |
| GBK | `buzzer.c`、`buzzer.h`、`image.c`、`IMU.c`、`justfloat.c`、`vofa.c`、`vofa.h`、`cpu1_main.c`、`isr.c`、`isr.h`、`isr_config.h`、`本文件夹作用.txt` |
| UTF-8（部分中文为 `�`） | `connect.c`、`connect.h`、`image.h`、`IMU.h`、`justfloat.h`、`motor.c`、`motor.h`、`cpu0_main.c`、`cpu0_main.h` |

建议：编辑前先确认目标文件的编码，**逐个文件按原编码保存**，不要对整个目录做批量转码，否则注释会进一步损坏。

### 10.4 建议的修复顺序

1. 修 `code/IMU.c:43` 的多余 `/`（一行即可）。
2. 修 `code/connect.c`：类型名、`turn_tick` 重复定义、孤立 `*/`、把 `case` 逻辑包进 `switch(Go)`、补全 `if()` 条件（或直接暂时让 `state_machine()` 只做直行）。
3. 补齐或删除未定义符号：在 `image.c` 中实现 `cut_image()` / 定义 `Image_Use`、`image_busy`、`image_ready`、`road_order_cache`、`road_weighted_error_cache`；在 `motor.c` 定义 `base_speed`；删除或实现 `path_index` / `what_control()`。
4. 修 `circle_control()` 的左右轮输出（`Motor_SetSpeed(pwm_L, pwm_R)`）。
5. 统一蜂鸣器节拍单位（改注释或改周期）。
6. 接入摄像头 2：参考双摄库示例把 `mt9v03x_double_init(mt9v03x_double)` 或 `mt9v03x_2` 打开，并在图像处理中区分 `mt9v03x_image_1` / `mt9v03x_image_2`（注意库说明：DMA 只有一个，双摄同时输出时整体帧率会减半并可能逐渐重合同步）。
7. 按 IMU660RB 实际型号标定陀螺仪灵敏度（`code/IMU.c`），并将其输出接入转向补偿（当前 `T_N`/`Z_360`/`T_M` 只被计算、未被控制逻辑使用）。

---

## 11. 二次开发须知

1. **新增代码文件**：按 `code/本文件夹作用.txt` 的约定，直接把 `.c/.h` 放进 `code/`，不要再建子文件夹（否则需手动在工程里添加目录）。
2. **核间内存**：`user/cpu0_main.c` 用
   `#pragma section all "cpu0_dsram"` … `#pragma section all restore` 把全局变量放 CPU0 的 RAM；`cpu1_main.c` 同理用 `cpu1_dsram`。变量放在哪个核执行，就放到对应区域，避免跨核访问开销与一致性问题。
3. **中断优先级**：TC264 有 1~255 级（255 最高，0 表示不使能），**任何两个中断都不能设为同一优先级**，新增中断请修改 `user/isr_config.h` 并在 `user/isr.c` 中实现 `IFX_INTERRUPT`。
4. **中断嵌套**：TC 系列进入中断后硬件自动关中断，如需嵌套要在 ISR 内调用 `interrupt_global_enable(0)`。
5. **引脚避坑**：见 [4.5 节](#45-关于根目录的两个引脚说明文件)。
6. **性能**：主循环中的 `Otsu` 是 120×188 全图直方图统计，`ips114` 刷屏也较耗时；若帧率不足，可先把算法代码移入 CPU0 RAM（`#pragma section all "cpu0_psram"`）或把部分处理交给 CPU1。
7. **调参顺序建议**：先只跑速度环（把 `line_control()` 里目标速度固定、`Motor_SetSpeed` 直接给开环 PWM 验证方向与编码器极性），再调位置 PD 的 `Kp/Kd`，最后调转向差速系数与退出条件。

---

## 12. 开源协议与致谢

- 本工程底层为 **逐飞科技 TC264 开源库 V3.4.6**，采用 **GPL-3.0** 协议：
  - 许可证副本：`libraries/LICENSE`
  - 英文许可声明：`libraries/doc/GPL3_permission_statement.txt`
  - 版本更新记录：`libraries/doc/version.txt`
  - 使用时请保留 `成都逐飞科技有限公司` 的版权声明。
- 上层应用代码（`code/`、`user/`）随工程一同以 GPL-3.0 发布。

参考资料：

- 逐飞科技官网 / 店铺：<https://seekfree.taobao.com/>
- GNU GPL-3.0：<https://www.gnu.org/licenses/>
- VOFA+ 上位机：<https://www.vofa.plus/>
- AURIX 修改工程名称教程视频：<https://www.bilibili.com/video/BV1nBkMYMEwE>
- Infineon AURIX Development Studio / iLLD 文档：工程内 `libraries/infineon_libraries`

---

<sub>本 README 依据仓库当前源码、库头文件与构建产物实际内容整理；如与代码不一致，请以代码为准并及时更新本文档。</sub>
