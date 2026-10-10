# 80m 高度控制试飞与标定计划

本文档用于当前 STM32 火箭飞控工程的下一阶段验证：先不开真实高度舵面，试飞采集日志，判断高度预测是否可靠，再决定是否启用可动舵面高度控制。

当前代码状态：

- 高度目标：`ROCKET_ALTITUDE_TARGET_M = 80.0f`
- 候选舵角：`0, 3, 6, ..., 45 deg`
- 高度舵机基础开关：`ROCKET_ALTITUDE_SERVO_ENABLE = 0`
- 完整流程高度混控：已由 `ROCKET_FULL_FLOW_ALTITUDE_MIX_ENABLE = 1` 接入真实舵面
- Flash 日志格式：`FLIGHT_LOG_FORMAT_VERSION = 6`
- 单条日志：`112 bytes`
- 已记录高度预测、推荐舵角、气压速度、IMU 积分速度、竖直加速度
- 已记录实际舵机 PWM、角速度、目标角速度、PID 输出和传感器样本时间
- 256 字节日志头包含本次飞行使用的高度模型和时序配置快照

## 1. 当前不能直接闭环控制的原因

现在可以试飞采集预测数据，但不建议直接打开高度舵面控制。

原因：

- 气压计接口还需要确认传感器型号、驱动稳定性和零点可靠性。
- IMU 竖直加速度投影依赖安装方向，必须通过静态测试确认正负号。
- `ROCKET_ALTITUDE_BASE_CD` 和 `ROCKET_ALTITUDE_FIN_DRAG_K` 目前是估计值，不是实测值。
- 舵机 PWM 到实际舵面角度的比例还没有标定。
- 两个舵机叠加高度舵角时，方向必须通过实物确认。

所以第一轮目标不是控到 80m，而是验证：

```text
代码预测的最高点 和 实际日志最高点 是否接近
```

## 2. 试飞前必须确认的硬件

### 2.1 气压计

新板子预留 I2C1 气压计接口：

- `PB6`：I2C1 SCL
- `PB7`：I2C1 SDA
- 接口上有 3.3V 上拉
- 需要确认气压计模块供电、电平和地址

需要做：

1. 确认实际气压计型号，例如 BMP280、MS5611、DPS310 等。
2. 确认模块是否能用 3.3V I2C 电平。
3. 确认 SCL/SDA 没有接反。
4. 上电后气压高度零点是否能建立。
5. 静置时 `relative_altitude_m` 应该在 0 附近小幅波动。

当前 `ROCKET_BAROMETER_ENABLE` 如果还是 `0`，不会有真实高度预测。试飞前必须先把气压计驱动恢复并验证不阻塞 IMU。

### 2.2 IMU 安装方向

需要确认火箭竖直静止时日志中的：

- `roll_deg`
- `pitch_deg`
- `accel_x_mg`
- `accel_y_mg`
- `accel_z_mg`
- `vertical_accel_mps2`

静止竖直放置时，期望：

```text
roll_deg 接近 0
pitch_deg 接近 0
vertical_accel_mps2 接近 0
```

如果静止时 `vertical_accel_mps2` 接近 `-19.6` 或 `+19.6`，说明竖直方向或 1g 扣除方向错了，需要调整：

```c
ROCKET_ALTITUDE_ACCEL_VERTICAL_SIGN
ROCKET_ALTITUDE_ACCEL_VERTICAL_BIAS_MG
```

### 2.3 舵机和舵面

第一轮试飞建议高度舵面输出保持关闭：

```c
#define ROCKET_ALTITUDE_SERVO_ENABLE 0
```

地面必须先单独标定：

1. 给舵机 `1500us`，记录舵面中立角。
2. 给 `1510us, 1520us, ...`，测量舵面偏角。
3. 给 `1490us, 1480us, ...`，测量反方向偏角。
4. 得到实际比例：

```text
ROCKET_ALTITUDE_FIN_US_PER_DEG = 每 1 度舵面需要多少 us
```

例如实测 `1500us -> 0 deg`，`1600us -> 10 deg`，则：

```c
#define ROCKET_ALTITUDE_FIN_US_PER_DEG 10.0f
```

还要确认混控方向：

```c
ROCKET_ALTITUDE_ROLL_MIX_SIGN
ROCKET_ALTITUDE_PITCH_MIX_SIGN
```

目标是高度舵角为正时，两个可动舵面进入“增加阻力”的方向，而不是产生明显姿态偏转。

### 2.4 点火输出

连接真实点火负载前，必须先使用假负载测试。

地面联调时建议：

```c
#define ROCKET_IGNITION_ENABLE 0
```

确认完整流程和日志正常后，再按安全流程改回：

```c
#define ROCKET_IGNITION_ENABLE 1
```

真实点火测试必须遵守当地法规、场地规则和安全距离要求。

### 2.5 Flash 日志

W25Q64 必须正常工作，否则无法判断预测准不准。

需要确认：

- 正常启动会擦除旧日志。
- 要导出上次日志，必须短接调试跳线后上电/复位。
- PB1 故障灯不能显示存储故障双闪。
- 串口导出脚本能生成 CSV 和 SVG。

## 3. 关键参数清单

## 3.0 当前控制频率和读取频率

当前调度参数来自 `Hardware/RocketConfig.h` 和 `Hardware/DebugConfig.h`。

### 3.0.1 主循环和传感器

```c
#define ROCKET_TIMEBASE_PERIOD_MS           1U
#define ROCKET_IMU_PERIOD_MS                10U
#define ROCKET_BAROMETER_PERIOD_MS          40U
#define ROCKET_BAROMETER_ZERO_SAMPLES       16U
```

对应频率：

```text
Rocket_Task 主调度基准：1ms，约 1000Hz
IMU 读取周期：10ms，100Hz
气压计读取周期：40ms，25Hz
气压计零点样本：16 个样本，约 16 * 40ms = 640ms
```

注意：当前配置里：

```c
#define ROCKET_BAROMETER_ENABLE 0
```

也就是说，现在代码虽然有气压高度预测逻辑，但气压计实际读取仍然关闭。要真实验证高度预测，必须先恢复气压计读取，并确认不会阻塞 IMU。

### 3.0.2 姿态控制频率

```c
#define ROCKET_RATE_LOOP_PERIOD_MS          10U
#define ROCKET_ANGLE_LOOP_PERIOD_MS         20U
```

对应频率：

```text
角速度内环 PID：10ms，100Hz
姿态角外环：20ms，50Hz
```

当前串级结构是：

```text
roll/pitch 角度误差 -> 目标角速度
目标角速度 - 实际角速度 -> 舵机 PWM 偏移
```

### 3.0.3 舵机 PWM 和舵机更新

```c
#define ROCKET_SERVO_PWM_PERIOD_US          20000U
```

对应：

```text
舵机 PWM 周期：20ms，50Hz
```

代码里的舵机比较值会在内环更新时写入，内环是 100Hz；但普通舵机实际接收 PWM 帧是 50Hz，所以物理有效更新按 50Hz 理解更合理。

### 3.0.4 高度预测和速度融合频率

高度控制函数 `AltitudeControl_Update()` 在主任务中每次都会调用，但有效数据源频率不同：

```text
IMU 加速度参与速度积分：使用最近一次 IMU 数据，IMU 新样本约 100Hz
气压高度差分速度：气压计新样本约 25Hz
高度候选舵角计算：状态有效后可每次任务调用刷新
真实高度舵角输出：只有姿态控制更新舵机时才会写入，约 100Hz 代码更新 / 50Hz PWM 帧
```

第一轮试飞时：

```c
#define ROCKET_ALTITUDE_SERVO_ENABLE 0
```

所以高度舵角只记录，不真实输出。

### 3.0.5 Flash 日志频率

```c
#define DEBUG_FLIGHT_LOG_FREQUENCY_HZ       50U
#define DEBUG_FLIGHT_LOG_DURATION_MS        20000UL
```

对应：

```text
Flash 日志记录频率：50Hz
日志周期：20ms
日志时长：20s
```

当前日志格式：

```c
#define FLIGHT_LOG_FORMAT_VERSION           6U
#define FLIGHT_LOG_RECORD_SIZE_BYTES        112UL
```

20 秒 50Hz 共约 1000 条记录：

```text
1000 * 112 bytes + 256 bytes header ≈ 112.3KB
```

W25Q64 容量足够。

### 3.1 高度目标和控制允许条件

```c
#define ROCKET_ALTITUDE_TARGET_M              80.0f
#define ROCKET_ALTITUDE_CONTROL_START_MS      ROCKET_BURNOUT_DETECT_START_MS
#define ROCKET_ALTITUDE_MIN_CONTROL_SPEED_MPS 2.0f
#define ROCKET_ALTITUDE_DEADBAND_M            2.0f
```

含义：

- `TARGET_M`：目标最高高度。
- `CONTROL_START_MS`：允许高度预测/控制的时间起点。
- 高度控制不再以横滚/俯仰角误差作为准入条件。
- `MIN_CONTROL_SPEED_MPS`：上升速度太小时不再控制高度。
- `DEADBAND_M`：预测不超过目标加死区时，舵角保持 0。

### 3.2 火箭模型参数

```c
#define ROCKET_ALTITUDE_STAGE_MASS_KG          0.4074f
#define ROCKET_ALTITUDE_REFERENCE_DIAMETER_M   0.0616f
#define ROCKET_ALTITUDE_AIR_DENSITY_KG_M3      1.20f
#define ROCKET_ALTITUDE_BASE_CD                0.50f
#define ROCKET_ALTITUDE_FIN_DRAG_K             0.02f
#define ROCKET_ALTITUDE_CD_MIN                 0.10f
#define ROCKET_ALTITUDE_CD_MAX                 2.50f
```

最需要实测/拟合的是：

```c
ROCKET_ALTITUDE_BASE_CD
ROCKET_ALTITUDE_FIN_DRAG_K
```

`BASE_CD` 影响不打舵时预测高度。

`FIN_DRAG_K` 影响打舵后预测能降低多少高度。

### 3.3 预测积分参数

```c
#define ROCKET_ALTITUDE_PREDICT_DT_S        0.02f
#define ROCKET_ALTITUDE_PREDICT_MAX_STEPS   500U
```

当前最多预测：

```text
0.02s * 500 = 10s
```

对 80m 小火箭够用。

### 3.4 气压速度估计参数

```c
#define ROCKET_ALTITUDE_VELOCITY_MIN_DT_MS      20U
#define ROCKET_ALTITUDE_VELOCITY_MAX_DT_MS      200U
#define ROCKET_ALTITUDE_VELOCITY_FILTER_ALPHA   0.25f
```

含义：

- 气压高度差分间隔太短会噪声大。
- 间隔太长说明数据断了。
- `FILTER_ALPHA` 越大，气压速度反应越快但噪声越大。

### 3.5 IMU 速度融合参数

```c
#define ROCKET_ALTITUDE_ACCEL_1G_MG             1000.0f
#define ROCKET_ALTITUDE_ACCEL_VERTICAL_SIGN     1.0f
#define ROCKET_ALTITUDE_ACCEL_VERTICAL_BIAS_MG  0.0f
#define ROCKET_ALTITUDE_IMU_VELOCITY_MAX_DT_MS  50U
#define ROCKET_ALTITUDE_IMU_SPEED_LIMIT_MPS     100.0f
#define ROCKET_ALTITUDE_IMU_SPEED_BLEND         0.70f
#define ROCKET_ALTITUDE_BARO_SPEED_CORRECTION   0.05f
```

第一轮重点看：

- 静止时 `vertical_accel_mps2` 是否接近 0。
- `imu_vertical_speed_mps` 是否短时间跟随上升速度。
- `barometer_vertical_speed_mps` 是否慢但稳定。
- `vertical_speed_mps` 是否介于二者之间。

### 3.6 舵机高度混控参数

```c
#define ROCKET_ALTITUDE_SERVO_ENABLE        0
#define ROCKET_ALTITUDE_FIN_US_PER_DEG      5.5556f
#define ROCKET_ALTITUDE_SERVO_LIMIT_DEG     45.0f
#define ROCKET_ALTITUDE_ROLL_MIX_SIGN       1.0f
#define ROCKET_ALTITUDE_PITCH_MIX_SIGN      -1.0f
```

第一轮必须保持：

```c
#define ROCKET_ALTITUDE_SERVO_ENABLE 0
```

只有预测可靠、方向标定完成后，才能考虑改成：

```c
#define ROCKET_ALTITUDE_SERVO_ENABLE 1
```

## 4. 第一次试飞前的软件检查

### 4.1 Keil 编译

在 Keil MDK/ARMCC5 中确认：

```text
0 Error
0 Warning
```

本机没有 `uv4/armcc`，所以需要在你的 Keil 环境里编译。

### 4.2 确认日志格式

当前应为：

```c
#define FLIGHT_LOG_FORMAT_VERSION     6U
#define FLIGHT_LOG_RECORD_SIZE_BYTES  112UL
```

脚本 `Tools/flight_log_plot.py` 已经同步 v6 字段和配置快照。

CSV 里应该能看到：

```text
relative_altitude_m
vertical_speed_mps
barometer_vertical_speed_mps
imu_vertical_speed_mps
vertical_accel_mps2
predicted_apogee_m
suggested_deflection_deg
altitude_estimator_valid
altitude_imu_estimator_valid
altitude_prediction_valid
altitude_control_allowed
roll_rate_dps
pitch_rate_dps
yaw_rate_dps
roll_rate_target_dps
pitch_rate_target_dps
roll_pid_output_us
pitch_pid_output_us
roll_servo_pwm_us
pitch_servo_pwm_us
applied_altitude_deflection_deg
imu_sample_timestamp_ms
barometer_sample_timestamp_ms
imu_sample_age_ms
barometer_sample_age_ms
```

### 4.3 地面静态日志测试

不点火，短时间上电记录。

检查：

- `relative_altitude_m` 静止接近 0。
- `vertical_accel_mps2` 静止接近 0。
- `imu_vertical_speed_mps` 静止不持续漂移。
- `altitude_zero_ready = 1`。
- `imu_valid = 1`。
- `barometer_valid = 1`。

如果静止时 `vertical_accel_mps2` 偏差明显，先不要试飞。

## 5. 第一轮试飞怎么做

第一轮目标：

```text
不开高度舵面，只记录预测。
```

参数建议：

```c
#define ROCKET_ALTITUDE_SERVO_ENABLE 0
#define ROCKET_IGNITION_ENABLE       按你的安全测试阶段决定
```

试飞后立即导出日志：

```bash
python Tools/flight_log_plot.py --port COMx
```

或者如果已有转储文件：

```bash
python Tools/flight_log_plot.py path/to/dump.txt
```

## 6. 怎么判断预测准不准

导出 CSV 后，先找实际最高点：

```text
actual_apogee = max(relative_altitude_m)
```

然后在关键时间段看：

```text
prediction_error = predicted_apogee_m - actual_apogee
```

重点看这些时刻：

- 二级点火后。
- 检测到失去动力后。
- `altitude_control_allowed` 从 0 变 1 后。
- 上升速度还比较大时。
- 接近最高点前。

判断标准建议：

```text
误差稳定在 +/- 3m 到 +/- 5m：可以继续调参，后续考虑小舵角闭环。
误差经常超过 +/- 10m：不能打开真实高度舵面。
预测一会儿高估一会儿低估：速度估计或气压计噪声需要先处理。
```

## 7. 需要我根据日志调整哪些参数

把 CSV 和 SVG 给我后，我主要会看这些字段：

```text
timestamp_ms
relative_altitude_m
vertical_speed_mps
barometer_vertical_speed_mps
imu_vertical_speed_mps
vertical_accel_mps2
roll_deg
pitch_deg
predicted_apogee_m
suggested_deflection_deg
altitude_estimator_valid
altitude_imu_estimator_valid
altitude_prediction_valid
altitude_control_allowed
```

我会根据日志判断：

1. 气压高度是否可用。
2. IMU 竖直加速度正负号是否正确。
3. IMU 速度是否漂移。
4. 融合速度是否比单独气压速度更好。
5. `ROCKET_ALTITUDE_BASE_CD` 是否需要调。
6. 是否需要降低或提高 `ROCKET_ALTITUDE_IMU_SPEED_BLEND`。
7. 推荐舵角是否过于激进。

可能调整的参数：

```c
ROCKET_ALTITUDE_ACCEL_VERTICAL_SIGN
ROCKET_ALTITUDE_ACCEL_VERTICAL_BIAS_MG
ROCKET_ALTITUDE_IMU_SPEED_BLEND
ROCKET_ALTITUDE_BARO_SPEED_CORRECTION
ROCKET_ALTITUDE_VELOCITY_FILTER_ALPHA
ROCKET_ALTITUDE_BASE_CD
ROCKET_ALTITUDE_FIN_DRAG_K
ROCKET_ALTITUDE_COST_DEFLECTION
ROCKET_ALTITUDE_COST_DEFLECTION_CHANGE
```

## 8. 什么时候可以打开真实高度舵面

同时满足以下条件后，才建议打开：

- 气压计稳定，`relative_altitude_m` 没有明显跳变。
- 静止时 `vertical_accel_mps2` 接近 0。
- 试飞日志里 `predicted_apogee_m` 误差大多数在 `+/- 3m 到 +/- 5m`。
- 舵机方向和 `ROCKET_ALTITUDE_FIN_US_PER_DEG` 已经台架标定。
- 高度舵角为正时，实物确认是增加阻力方向。
- 姿态 PID 不会因为高度混控而明显发散。

第一次打开时建议保守：

```c
#define ROCKET_ALTITUDE_SERVO_ENABLE        1
#define ROCKET_ALTITUDE_SERVO_LIMIT_DEG     3.0f
```

单独低风险台架验证时，可先限制最大 3 度；当前完整流程代码的上限为 45 度。

如果日志正常，再逐步提高：

```text
3 deg -> 6 deg -> 9 deg -> ... -> 45 deg
```

## 9. 下一步最需要做的事

优先级从高到低：

1. 确认气压计型号和驱动稳定性。
2. 开启气压计后地面静态记录，验证高度零点。
3. 静态验证 `vertical_accel_mps2` 是否接近 0。
4. Keil 编译确认 0 错误 0 警告。
5. 第一轮试飞只记录，不打开高度舵面。
6. 导出 CSV/SVG。
7. 根据日志调整高度预测和融合参数。
8. 台架标定舵机 PWM 到舵角比例。
9. 确认高度舵角正方向。
10. 小舵角打开真实高度控制。

## 10. 给我日志后我才能继续完善的内容

要把代码完全调好，需要至少一组真实日志。

需要提供：

- `flight_log.csv`
- `flight_log_charts.svg`
- 火箭实际质量
- 火箭外径
- 气压计型号
- IMU 安装方向说明或照片
- 舵面 0/3/5/10 度对应 PWM 实测值
- 第一次试飞的大致环境：风、温度、场地海拔
- 是否真实二级点火，以及点火时间是否符合日志

有这些数据后，才能把：

```c
ROCKET_ALTITUDE_BASE_CD
ROCKET_ALTITUDE_FIN_DRAG_K
ROCKET_ALTITUDE_FIN_US_PER_DEG
ROCKET_ALTITUDE_*_MIX_SIGN
```

从估计值改成接近实物的参数。

## 11. 当前控制代码实际需要的物理参数

这里只列当前代码真正用到、或者打开高度舵面后马上会用到的物理参数。

### 11.1 控制段火箭质量

对应代码：

```c
#define ROCKET_ALTITUDE_STAGE_MASS_KG 0.4074f
```

需要测：

```text
二级失去动力后、可动舵面开始控制时的火箭质量，单位 kg
```

这个值不是整箭起飞质量，而是高度预测开始工作时的剩余质量。

### 11.2 箭体参考直径

对应代码：

```c
#define ROCKET_ALTITUDE_REFERENCE_DIAMETER_M 0.0616f
```

需要测：

```text
箭体最大外径，单位 m
```

代码用它计算参考面积：

```text
reference_area = pi * diameter^2 / 4
```

### 11.3 基础阻力系数

对应代码：

```c
#define ROCKET_ALTITUDE_BASE_CD 0.50f
```

需要得到：

```text
不打高度舵角时，火箭滑行段的等效阻力系数
```

这个不需要你直接提供。第一轮先使用默认值：

```c
#define ROCKET_ALTITUDE_BASE_CD 0.50f
```

然后用“不开高度舵面”的飞行日志反推。需要日志里有：

```text
relative_altitude_m
vertical_speed_mps
predicted_apogee_m
实际最高高度
```

如果预测最高点总是高于实际最高点，说明 `BASE_CD` 偏小。

如果预测最高点总是低于实际最高点，说明 `BASE_CD` 偏大。

### 11.4 舵面附加阻力系数

对应代码：

```c
#define ROCKET_ALTITUDE_FIN_DRAG_K 0.02f
```

当前模型是：

```c
Cd = BASE_CD + FIN_DRAG_K * deflection_rad * deflection_rad
```

需要得到：

```text
舵角增加后，阻力增加多少
```

这个也不需要你第一轮直接提供。第一轮不开高度舵面，所以它暂时不会影响真实飞行，只影响“假设打舵时”的预测。

第一轮先使用默认值：

```c
#define ROCKET_ALTITUDE_FIN_DRAG_K 0.02f
```

真正要拟合它，需要后续打开小舵角，例如 3 度，做一次受控飞行日志。

也就是说：

```text
不开舵日志 -> 拟合 BASE_CD
小舵角日志 -> 拟合 FIN_DRAG_K
```

### 11.5 空气密度

对应代码：

```c
#define ROCKET_ALTITUDE_AIR_DENSITY_KG_M3 1.20f
```

需要记录：

```text
试飞当天温度
场地海拔
大气压力，如果有
```

这个也可以先不给。第一轮直接使用默认值：

```c
#define ROCKET_ALTITUDE_AIR_DENSITY_KG_M3 1.20f
```

80m 级别第一轮验证时，空气密度误差通常不是最大误差源。后续如果要提高预测精度，再用温度、气压和海拔修正。

### 11.6 舵面角度到 PWM 的比例

对应代码：

```c
#define ROCKET_ALTITUDE_FIN_US_PER_DEG 5.5556f
```

需要测：

```text
舵面 0 deg 对应 PWM
舵面 1 deg 对应 PWM
舵面 5 deg 对应 PWM
舵面 10 deg 对应 PWM
舵面 45 deg 对应 PWM
```

最终要得到：

```text
每 1 度舵面偏转需要多少 us
```

### 11.7 高度舵角混控方向

对应代码：

```c
#define ROCKET_ALTITUDE_ROLL_MIX_SIGN   1.0f
#define ROCKET_ALTITUDE_PITCH_MIX_SIGN -1.0f
```

需要确认：

```text
高度舵角为正时，两个舵机实际是不是让舵面进入增加阻力的方向
```

如果方向反了，就改对应的 `MIX_SIGN`。

### 11.8 IMU 竖直加速度方向和偏置

对应代码：

```c
#define ROCKET_ALTITUDE_ACCEL_VERTICAL_SIGN     1.0f
#define ROCKET_ALTITUDE_ACCEL_VERTICAL_BIAS_MG  0.0f
```

需要测：

```text
火箭竖直静止时 accel_x_mg
火箭竖直静止时 accel_y_mg
火箭竖直静止时 accel_z_mg
火箭竖直静止时 roll_deg
火箭竖直静止时 pitch_deg
火箭竖直静止时 vertical_accel_mps2
```

静止时 `vertical_accel_mps2` 应该接近 0。

如果不接近 0，就需要改 `ROCKET_ALTITUDE_ACCEL_VERTICAL_SIGN` 或 `ROCKET_ALTITUDE_ACCEL_VERTICAL_BIAS_MG`。

### 11.9 失去动力检测阈值

对应代码：

```c
#define ROCKET_BURNOUT_ACCEL_Z_ABS_THRESHOLD_MG 200.0f
#define ROCKET_BURNOUT_CONFIRM_MS              100U
```

需要从日志确认：

```text
二级工作时 accel_z_mg 大概是多少
二级失去动力后 accel_z_mg 大概是多少
阈值 200mg 是否能稳定区分有动力/无动力
```

## 12. 给我数据时的最低清单

当前代码要继续调准，至少需要：

```text
1. flight_log.csv
2. flight_log_charts.svg
3. 控制段火箭质量 kg
4. 箭体最大外径 m
5. 竖直静止时 accel_x/y/z、roll/pitch、vertical_accel_mps2
6. 舵面 0/1/5/10 度对应 PWM
7. 高度舵角正方向是否增加阻力
8. 实际最高高度，如果有外部测量
```

不需要你直接给：

```text
BASE_CD
FIN_DRAG_K
AIR_DENSITY
```

处理方式：

```text
BASE_CD：用不开舵飞行日志反推
FIN_DRAG_K：后续用小舵角飞行日志反推
AIR_DENSITY：第一轮先用默认 1.20 kg/m^3
```

这些数据对应当前代码里的质量、参考面积、IMU 竖直加速度、舵角 PWM 比例和混控方向。

## 13. 2026-08-16 实时性修正后的复测重点

本次根据实飞日志完成了以下修正：

```text
1. 最高点预测由 16×500 步数值积分改为解析计算
2. 舵角候选决策固定为每 40 ms 更新一次
3. 只有失去动力确认、姿态控制允许运行后，才计算并应用高度舵角
4. fused vz 改为持续保存的 IMU 积分 + 气压速度缓慢校正
5. 加速度计量程由 ±4 g 改为 ±16 g
```

下一次复测必须确认：

```text
1. 7.2 s 后 IMU 样本间隔仍接近 10 ms，不再出现连续 FIFO reset
2. 加速度不再卡在约 ±4000 mg；若达到约 ±16000 mg，说明仍发生削顶
3. burnout_detected 出现后，flight_control_running 和 altitude_control_allowed 才变为 1
4. predicted_apogee、suggested_deflection 和 applied_altitude_deflection 开始出现有效数据
5. fused vz 与 imu vz 不再完全重合，并且不能随气压速度尖峰瞬间跳变
6. 任何点火实测前先断开真实点火器，用假负载完成一次完整流程
```

## 14. 高度优先舵角分配

当前先完整分配高度舵角，再把姿态PID限制在剩余PWM范围内：

```text
高度算法请求舵角完整执行
高度舵角不再根据姿态误差缩小
高度PWM分配完成后，再限制横滚和俯仰PID
最终混控仍保持±300μs物理限幅
65°失稳开伞保护继续有效
```

RLOG v9新增记录：

```text
高度控制请求舵角
高度控制实际舵角
高度控制实际占用PWM
分配后的横滚PID
分配后的俯仰PID
姿态PID受限原因
```

复测时应确认请求舵角和实际舵角一致；混控接近限幅时，应优先压缩对应姿态PID，而不是压缩高度舵角。
