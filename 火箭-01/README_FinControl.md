# 可动舵面飞控说明

## 硬件连接

- Pitch 舵机：PC6 / TIM3_CH1 FullRemap
- Roll 舵机：PC7 / TIM3_CH2 FullRemap
- 开伞舵机：PB10 / TIM2_CH3 PartialRemap2
- 主点火输出：PC4 / GPIO 推挽输出
- 分离点火输出：SERVO_3 / PC8 / GPIO 推挽输出，与 PC4 同步
- IMU：LSM6DSV16X，PB3 SCK、PB4 MISO、PB5 MOSI、PA11 CS
- 舵机信号：50 Hz PWM，控制舵中位 1500 μs
- 舵机电源：使用独立 5 V 舵机电源，不要从 STM32 的 3.3 V 取电

PB3/PB4/PB5 使用 SPI1 完全重映射。为释放 PB3/PB4，工程关闭 JTAG、保留 PA13/PA14 的 SWD。

## 控制逻辑

LSM6DSV16X 以 120 Hz 输出片上 SFLP 姿态和陀螺 FIFO。驱动将四元数转换为 Roll/Pitch，并把 X/Y 轴角速度送入现有串级控制器：

1. 角度外环每 20 ms 根据 Roll/Pitch 误差生成目标角速度。
2. 角速度内环每 10 ms 根据 X/Y 轴角速度生成舵机微秒偏量。
3. Roll 输出到 PC7，Pitch 输出到 PC6。
4. 开伞触发后，PB10 从 500 μs切换到 1500 μs并保持。

IMU 初始化会校验设备 ID 和关键配置，并在 PID 启动前有界重试。FIFO 溢出或积压超过 6 项会被清空；任一数据流超过 30 ms 未更新即判无效，控制舵在第一次无效采样时立即回中。点火入口要求连续 10 个有效 IMU 样本且姿态安全；入口只决策一次，故障中止后不会在同一窗口重新点火。

## 主要调参位置

参数集中在 `Hardware/RocketConfig.h`：

- `ROCKET_ROLL_ANGLE_KP`、`ROCKET_PITCH_ANGLE_KP`
- `ROCKET_ROLL_RATE_KP/KI/KD`
- `ROCKET_PITCH_RATE_KP/KI/KD`
- `ROCKET_CONTROL_LIMIT_US`
- `ROCKET_ROLL_SERVO_SIGN`、`ROCKET_PITCH_SERVO_SIGN`
- `ROCKET_ROLL_ANGLE_SIGN`、`ROCKET_PITCH_ANGLE_SIGN`
- `ROCKET_ROLL_RATE_SIGN`、`ROCKET_PITCH_RATE_SIGN`

当前 `ROCKET_IGNITION_ENABLE=1`。台架测试前必须先改为 `0`，确认 IMU 轴向和舵机修正方向后，再逐项调整 PID。
