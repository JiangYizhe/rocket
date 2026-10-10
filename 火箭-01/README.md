# STM32F103RC 火箭双环飞控

本工程面向 STM32F103RCT6，使用 Keil MDK、CMSIS 与 STM32F10x 标准外设库。控制程序采用非阻塞任务状态机：TIM1 提供 1 ms 时基，主循环持续调用 `Rocket_Task(now_ms)`。

## 点火模式选择

在 `Hardware/RocketConfig.h` 中修改 `ROCKET_REMOTE_LAUNCH_MODE`：设为 `0` 时使用原先的非远程逻辑，任务时序从飞控上电开始，并在上电后 6.4 s 点火；设为 `1` 时启用远程点火逻辑，等待蓝牙链路校验和地面站发射命令后开始任务计时，并在一级点火后 1.4 s 执行二级点火。该参数只允许为 `0` 或 `1`，其他值会在编译时报错。

> 安全警告：当前 `Hardware/RocketConfig.h` 中的 `ROCKET_IGNITION_ENABLE` 已设为 `1`，PC4 和 PC8 会在满足联锁条件的点火时间窗同步输出高电平。连接点火器前应先改回 `0`，使用假负载完成时序、接线和失效测试；只有断开调试器、完成接线复核并清空危险区域后，才可重新启用。初始 PID 参数只用于低风险台架验证，不能直接用于实箭。

## 工程结构

- `User/main.c`：只执行初始化并轮询火箭任务。
- `Hardware/Rocket.c`：任务编排、点火和控制状态协调。
- `Hardware/RocketState.c`：纯状态机与绝对时序逻辑，可在主机测试。
- `Hardware/FlightControl.c`：Roll/Pitch 角度外环与角速度内环。
- `Hardware/IMU.c`：LSM6DSV16X SPI1、SFLP/FIFO 和姿态换算。
- `Hardware/LSM6DSV16X/`：ST 官方 LSM6DSV16X 寄存器驱动。
- `Hardware/Servo.c`：TIM2/TIM3 的统一 50 Hz 舵机输出。
- `Hardware/Ignition.c`：点火 GPIO 安全初始化和总开关。
- `System/PID.c`：通用浮点 PID、抗积分饱和和 D 项低通。
- `System/Timebase.c`：TIM1 产生 1 ms 系统时基。
- `System/ControlMath.c`：纯控制限幅与舵机脉宽换算。
- `Tests/test_main.c`：PID、控制数学和状态机的 MinGW 主机测试。

## 引脚与定时器

| 功能 | 引脚 | 外设 | 初始输出 | 动作 |
| --- | --- | --- | --- | --- |
| 主点火头 | PC4 | GPIO 推挽输出 | 低电平 | 与分离点火头同步输出 800 ms |
| 分离点火头（SERVO_3） | PC8 | GPIO 推挽输出 | 低电平 | 与主点火头同步输出 800 ms |
| 开伞舵机 | PB10 | TIM2_CH3 PartialRemap2 | 0°，500 μs | 开伞后保持 90°，1500 μs |
| Roll 控制舵机 | PC7 | TIM3_CH2 FullRemap | 中位，1500 μs | PID 偏量限制为 ±300 μs |
| Pitch 控制舵机 | PC6 | TIM3_CH1 FullRemap | 中位，1500 μs | PID 偏量限制为 ±300 μs |
| IMU SCK | PB3 | SPI1_SCK Remap | Mode 3 | LSM6DSV16X |
| IMU MISO | PB4 | SPI1_MISO Remap | 输入 | LSM6DSV16X |
| IMU MOSI | PB5 | SPI1_MOSI Remap | 输出 | LSM6DSV16X |
| IMU CS | PA11 | GPIO 推挽输出 | 高电平 | 低电平选中 |

TIM3 全重映射后在 PC6/PC7 输出控制舵 PWM；TIM2 PartialRemap2 在 PB10 输出开伞 PWM。两个定时器均使用 1 MHz 计数、20000 μs 周期。SPI1 全重映射到 PB3/PB4/PB5，使用 Mode 3、1.125 MHz；JTAG 被禁用以释放 PB3/PB4，但 PA13/PA14 的 SWD 保留。工程按 8 MHz 外部晶振配置，系统时钟为 72 MHz。

> 驱动目标型号为 LSM6DSV16X，`WHO_AM_I` 必须为 `0x70`；其他 LSM6DSV 变体不能直接假定兼容。

> 主点火使用 PC4，分离点火使用 SERVO_3/PC8。连接真实点火负载前，必须先用假负载验证两路同步时序和上电低电平，并为外部驱动增加硬件下拉。

## 状态与绝对时序

| 远程发射任务时间 | 主状态 | 行为 |
| --- | --- | --- |
| 初始化期间 | `ROCKET_STATE_INIT` | 点火拉低、舵机初始化、IMU 初始化 |
| 等待远程命令 | `ROCKET_STATE_WAIT_PID` | 控制舵保持中位，不启动任务计时 |
| 0～1399 ms | `ROCKET_STATE_WAIT_PID` | 一级点火已经触发，飞控倒计时，控制舵保持中位 |
| 1400～2199 ms | `ROCKET_STATE_IGNITION` | 联锁满足且总开关开启时输出二级点火 800 ms 脉冲 |
| 2200 ms以后 | `ROCKET_STATE_FLIGHT` | 确认燃烧结束后启动 PID，并执行当前高度开伞策略 |
| 开伞后 | `ROCKET_STATE_DEPLOYED` | 开伞舵机锁定 90°，清空 PID，控制舵回中 |

当前任务时间从飞控接受远程发射命令开始；基站重发时携带命令年龄，飞控会补偿通信延迟。蜂鸣器在链路就绪后完成三短声，随后持续长响，发射开始后也不会关闭。

`ROCKET_IGNITION_ENABLE=1` 允许 PC4 和 PC8 同步输出，但还必须收到通过链路校验的远程发射命令。点火窗口入口要求最近 10 个 IMU 样本连续有效、当前样本有效、姿态未超过临界角且尚未开伞。入口只决策一次，启动超过 20 ms、任一 IMU 样本失效、倾角超限或开伞都会否决/中止点火，并且本次上电周期内不会迟到或断续重启。总开关为 `0` 时 PC4 和 PC8 始终保持低电平。

LSM6DSV16X 的 SFLP 姿态和陀螺 FIFO 均配置为 120 Hz。驱动允许短暂复用缓存，但任一数据流超过 30 ms 未更新即返回无效；FIFO 满、溢出或积压超过 6 项（约 25 ms 数据）时会执行 BYPASS→STREAM 刷新并清空缓存，不会把队首旧数据当成新样本。FIFO 内各项的本机时间按 240 word/s 和队列位置回推。任一次无效控制样本都会立即使控制舵回中；状态机连续三次读取失败后再将 IMU 诊断健康状态置为故障。

IMU 初始化会先强制返回主寄存器 bank，再校验 `WHO_AM_I=0x70`，并对 SPI/I2C-I3C、BDU、自动递增、量程、ODR、FIFO 和 SFLP 关键配置逐项读回。SPI 事务均有超时，失败后拉高 CS、清理 DR/SR 并重新使能 SPI；连续三次总线失败会退出已初始化状态。PID 启动前，初始化失败会每 500 ms 重试一次，并在 PID 启动前 200 ms 停止阻塞式重试。`IMU_GetStatus()` 可在调试器中区分 ID、复位、配置、总线、FIFO 刷新和数据陈旧等状态。

本工程最初采用的 ST V4.3.0 platform-independent driver 与参考目录逐字节一致。审查发现原版 `lsm6dsv16x_reset_set()` 将 BOOT 和 SW_RESET 的枚举位映射写反，并有两处在寄存器读取失败后仍可能写入未初始化结构体的异常路径；工程已做局部修正，并用寄存器 mock 回环测试覆盖四种复位值。官方 `sflp_game_gbias_set()` 和加速度硬件 offset API 未直接用于飞行路径；角速度零偏通过 `ROCKET_ROLL_RATE_BIAS_DPS`、`ROCKET_PITCH_RATE_BIAS_DPS` 进行软件补偿。

## 串级 PID

Roll 与 Pitch 使用相同结构：

1. 角度外环每 20 ms（50 Hz）运行一次，仅使用 P 控制，将角度误差转换为目标角速度，并限制在 ±90°/s。
2. 角速度内环每 10 ms（100 Hz）运行一次，使用完整 PID，将目标角速度与 LSM6DSV16X 的 X/Y 轴角速度比较，输出舵机微秒偏量。
3. 舵机层统一完成方向变换、±300 μs 控制限幅及 1000～2000 μs 物理限幅。

默认参数集中在 `Hardware/RocketConfig.h`：

- 目标 Roll/Pitch：0°。
- 外环 Kp：4.0；目标角速度限幅：±90°/s。
- 内环 Kp：1.0，Ki：0，Kd：0.004。
- 积分输出限幅：±80 μs；总输出限幅：±300 μs。
- D 项一阶低通系数：0.2。
- `ROCKET_ROLL_SERVO_SIGN`、`ROCKET_PITCH_SERVO_SIGN` 用于改变舵机修正方向。
- `ROCKET_ROLL_ANGLE_SIGN`、`ROCKET_PITCH_ANGLE_SIGN`、`ROCKET_ROLL_RATE_SIGN`、`ROCKET_PITCH_RATE_SIGN` 用于统一 IMU 安装方向。
- LSM6DSV16X 固定配置为 ±2000°/s、±4 g、120 Hz；实装后必须验证 X/Y 轴与 Roll/Pitch 的对应关系。

## 建议调参顺序

1. 保持点火总开关为 `0`，拆下危险负载，只给控制器和舵机供电。
2. 将内环 Ki、Kd 暂设为 0，外环 Kp 设小；手动转动 IMU，先确认 Roll/Pitch 传感器符号与舵机修正方向正确。
3. 固定角度目标，逐步增加内环 Kp，获得快速但不持续振荡的角速度响应。
4. 少量增加内环 Kd 抑制快速振荡和过冲，再缓慢增加 Ki 消除静差；观察积分是否长期触及 ±80 μs。
5. 最后调整外环 Kp 和 ±90°/s 目标限幅，避免外环给内环不可实现的命令。
6. 每次只修改一个参数，并记录电源电压、舵机负载、响应和振荡频率。

## 编译与测试

MinGW 主机测试不依赖 STM32 头文件：

```powershell
C:\mingw64\bin\gcc.exe -std=c99 -Wall -Wextra -Werror -pedantic `
  -ISystem -IHardware Tests\test_main.c System\PID.c System\ControlMath.c `
  Hardware\RocketState.c -o Tests\rocket_tests.exe
Tests\rocket_tests.exe

C:\mingw64\bin\gcc.exe -std=c99 -Wall -Wextra -Werror -pedantic `
  -ISystem -IHardware Tests\test_altitude_control.c `
  System\ControlMath.c Hardware\AltitudeControl.c -lm `
  -o Tests\altitude_control_tests.exe
Tests\altitude_control_tests.exe

C:\mingw64\bin\gcc.exe -std=c99 -Wall -Wextra -Werror -pedantic `
  -ISystem -IHardware Tests\test_flight_control_allocator.c `
  Hardware\FlightControl.c System\PID.c System\ControlMath.c -lm `
  -o Tests\flight_control_allocator_tests.exe
Tests\flight_control_allocator_tests.exe

C:\mingw64\bin\gcc.exe -std=c99 -Wall -Wextra -Werror -pedantic `
  -IHardware\LSM6DSV16X Tests\test_lsm6dsv16x_reset.c `
  Hardware\LSM6DSV16X\lsm6dsv16x_reg.c `
  -o Tests\lsm6dsv16x_reset_tests.exe
Tests\lsm6dsv16x_reset_tests.exe
```

Keil 使用 `火箭-01.uvprojx` 的 `火箭-01` 目标全量 Rebuild。目标器件为 STM32F103RC、宏为 `USE_STDPERIPH_DRIVER,STM32F10X_HD`，编译器为 ARMCC 5。修改设备相关代码后必须达到 0 Error、0 Warning。

## 板级验收清单

- 不连接点火器，先将当前总开关从 `1` 改为 `0` 并确认 PC4、PC8 始终为低；使用两路独立假负载测试同步时间窗后立即恢复为 `0`。
- 确认 PC6 和 PC7 都是 50 Hz，上电脉宽 1500 μs；手动转动 IMU 时，两路舵面均朝减小误差的方向动作。
- 确认 PB10 上电为 500 μs；触发开伞后变为 1500 μs并保持。
- 确认 PB3/PB4/PB5 的 SPI Mode 3 波形、PA11 片选以及 `WHO_AM_I=0x70`，并确认 SWD 仍可连接。
- 断开 IMU，确认第一次无效读取后控制舵立即回中、PC4 和 PC8 保持低电平，同时 12000 ms 强制开伞仍执行。
- 实际装箭前再次检查 LSM6DSV16X 安装轴向、舵机连杆极限、电源压降和点火输出隔离。
