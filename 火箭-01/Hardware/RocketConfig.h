#ifndef __ROCKET_CONFIG_H
#define __ROCKET_CONFIG_H

/*============================== 安全开?==============================*/

/* 当前已启用；台架调试和连接假负载前必须改??*/
#define ROCKET_IGNITION_ENABLE              1
#define ROCKET_BENCH_SEQUENCE_TEST_ENABLE   0

/*
 * Launch source selection:
 * 0: original non-remote mode; flight timing starts automatically at power-on.
 * 1: remote mode; wait for a qualified Bluetooth link and launch command.
 */
#define ROCKET_REMOTE_LAUNCH_MODE           1

#if (ROCKET_REMOTE_LAUNCH_MODE != 0) && \
	(ROCKET_REMOTE_LAUNCH_MODE != 1)
#error "ROCKET_REMOTE_LAUNCH_MODE must be 0 or 1"
#endif

#if ROCKET_REMOTE_LAUNCH_MODE
#define ROCKET_REMOTE_LAUNCH_ENABLE         1
#define ROCKET_LEGACY_AUTO_LAUNCH_ENABLE    0
#else
#define ROCKET_REMOTE_LAUNCH_ENABLE         0
#define ROCKET_LEGACY_AUTO_LAUNCH_ENABLE    1
#endif

/* 暂时禁用BMP280；驱动代码保留，改为1即可恢复?*/
#define ROCKET_BAROMETER_ENABLE             1

/*============================== 硬件映射 ==============================*/

#define ROCKET_IGNITION_GPIO                GPIOC
#define ROCKET_IGNITION_GPIO_PIN            GPIO_Pin_4
#define ROCKET_IGNITION_GPIO_CLOCK          RCC_APB2Periph_GPIOC

/* H3 pin 1 / SERVO_3 is PC8 and drives the separation igniter. */
#define ROCKET_SEPARATION_IGNITION_GPIO       GPIOC
#define ROCKET_SEPARATION_IGNITION_GPIO_PIN   GPIO_Pin_8
#define ROCKET_SEPARATION_IGNITION_GPIO_CLOCK RCC_APB2Periph_GPIOC

#define ROCKET_FAULT_LED_GPIO               GPIOB
#define ROCKET_FAULT_LED_GPIO_PIN           GPIO_Pin_1
#define ROCKET_FAULT_LED_GPIO_CLOCK         RCC_APB2Periph_GPIOB

/* H6 pin 1 / SERVO_6 is PB11. Connect the active buzzer negative lead to GND. */
#define ROCKET_BUZZER_GPIO                  GPIOB
#define ROCKET_BUZZER_GPIO_PIN              GPIO_Pin_11
#define ROCKET_BUZZER_GPIO_CLOCK            RCC_APB2Periph_GPIOB

#define ROCKET_PARACHUTE_PWM_TIMER          TIM2
#define ROCKET_PARACHUTE_PWM_CHANNEL        3U
#define ROCKET_PARACHUTE_PWM_GPIO           GPIOB
#define ROCKET_PARACHUTE_PWM_GPIO_PIN       GPIO_Pin_10

#define ROCKET_TIM2_GPIO_CLOCK              RCC_APB2Periph_GPIOB
#define ROCKET_TIM2_CLOCK                   RCC_APB1Periph_TIM2

#define ROCKET_PITCH_PWM_TIMER               TIM3
#define ROCKET_PITCH_PWM_CHANNEL             1U
#define ROCKET_PITCH_PWM_GPIO                GPIOC
#define ROCKET_PITCH_PWM_GPIO_PIN            GPIO_Pin_6

#define ROCKET_TIM3_GPIO_CLOCK              RCC_APB2Periph_GPIOC
#define ROCKET_TIM3_CLOCK                   RCC_APB1Periph_TIM3

#define ROCKET_ROLL_PWM_TIMER              TIM3
#define ROCKET_ROLL_PWM_CHANNEL            2U
#define ROCKET_ROLL_PWM_GPIO               GPIOC
#define ROCKET_ROLL_PWM_GPIO_PIN           GPIO_Pin_7

#define ROCKET_IMU_SPI                     SPI1
#define ROCKET_IMU_SPI_CLOCK               RCC_APB2Periph_SPI1
#define ROCKET_IMU_SPI_GPIO                GPIOB
#define ROCKET_IMU_SPI_GPIO_CLOCK          RCC_APB2Periph_GPIOB
#define ROCKET_IMU_SPI_SCK_PIN             GPIO_Pin_3
#define ROCKET_IMU_SPI_MISO_PIN            GPIO_Pin_4
#define ROCKET_IMU_SPI_MOSI_PIN            GPIO_Pin_5
#define ROCKET_IMU_CS_GPIO                 GPIOA
#define ROCKET_IMU_CS_GPIO_CLOCK           RCC_APB2Periph_GPIOA
#define ROCKET_IMU_CS_PIN                  GPIO_Pin_11

#define ROCKET_FLASH_SPI                   SPI2
#define ROCKET_FLASH_SPI_CLOCK             RCC_APB1Periph_SPI2
#define ROCKET_FLASH_SPI_GPIO              GPIOB
#define ROCKET_FLASH_SPI_GPIO_CLOCK        RCC_APB2Periph_GPIOB
#define ROCKET_FLASH_SPI_SCK_PIN           GPIO_Pin_13
#define ROCKET_FLASH_SPI_MISO_PIN          GPIO_Pin_14
#define ROCKET_FLASH_SPI_MOSI_PIN          GPIO_Pin_15
#define ROCKET_FLASH_CS_GPIO               GPIOC
#define ROCKET_FLASH_CS_GPIO_CLOCK         RCC_APB2Periph_GPIOC
#define ROCKET_FLASH_CS_PIN                GPIO_Pin_1

#define ROCKET_PRESSURE_I2C                 I2C1
#define ROCKET_PRESSURE_I2C_CLOCK           RCC_APB1Periph_I2C1
#define ROCKET_PRESSURE_I2C_GPIO            GPIOB
#define ROCKET_PRESSURE_I2C_GPIO_CLOCK      RCC_APB2Periph_GPIOB
#define ROCKET_PRESSURE_I2C_SCL_PIN         GPIO_Pin_6
#define ROCKET_PRESSURE_I2C_SDA_PIN         GPIO_Pin_7
#define ROCKET_PRESSURE_I2C_SPEED_HZ        100000UL

/*
 * W25Q64JV?WP?HOLD(/RESET)必须由硬件拉高? * 驱动使用标准单线SPI，不会置位QE? */

/*============================== 任务时序 ==============================*/

#define ROCKET_TIMEBASE_PERIOD_MS           1U
#define ROCKET_IMU_PERIOD_MS                10U
#define ROCKET_IMU_RETRY_PERIOD_MS          500U
#define ROCKET_IMU_RETRY_STOP_MARGIN_MS     200U
#define ROCKET_BAROMETER_PERIOD_MS          40U
#define ROCKET_BAROMETER_RETRY_PERIOD_MS    500U
#define ROCKET_BAROMETER_ZERO_SAMPLES       16U
#define ROCKET_RATE_LOOP_PERIOD_MS          10U
#define ROCKET_ANGLE_LOOP_PERIOD_MS         20U

/* H8: pin 1 USART2_TX/PA2, pin 2 USART2_RX/PA3, pin 4 GND. */
#define ROCKET_REMOTE_BAUD_RATE             115200UL
#define ROCKET_REMOTE_QUALIFY_COUNT         10U
#define ROCKET_REMOTE_PING_MAX_GAP_MS       400U
#define ROCKET_REMOTE_LINK_TIMEOUT_MS       1000U
#define ROCKET_REMOTE_MAX_SEQUENCE_GAPS     3U
#define ROCKET_REMOTE_MAX_COMMAND_AGE_MS    600U
#define ROCKET_REMOTE_TX_TIMEOUT_LOOPS      100000UL
#define ROCKET_BUZZER_SHORT_ON_MS           120U
#define ROCKET_BUZZER_SHORT_OFF_MS          120U

#define ROCKET_REMOTE_IGNITION_DELAY_MS     1130U
#define ROCKET_NON_REMOTE_IGNITION_DELAY_MS 6400U
#if ROCKET_REMOTE_LAUNCH_MODE
#define ROCKET_IGNITION_DELAY_MS            ROCKET_REMOTE_IGNITION_DELAY_MS
#else
#define ROCKET_IGNITION_DELAY_MS            ROCKET_NON_REMOTE_IGNITION_DELAY_MS
#endif
#define ROCKET_IGNITION_PULSE_MS            800U
#define ROCKET_BURNOUT_ACCEL_Z_ABS_THRESHOLD_MG 1200.0f
#define ROCKET_FORCE_DEPLOY_TIME_MS         11000U
#define ROCKET_IGNITION_START_GRACE_MS      20U
/*
 * After second-stage ignition, movable fins stay centered until axial
 * acceleration magnitude drops below the burnout threshold continuously.
 */
#define ROCKET_BURNOUT_DETECT_START_MS      \
	(ROCKET_IGNITION_DELAY_MS + ROCKET_IGNITION_PULSE_MS)
#define ROCKET_BURNOUT_CONFIRM_MS           100U
/* Movable-fin attitude PID takes over 200 ms after second-stage ignition. */
#define ROCKET_PID_START_DELAY_MS           (ROCKET_IGNITION_DELAY_MS + 200U)
#define ROCKET_TILT_CHECK_TIME_MS           ROCKET_BURNOUT_DETECT_START_MS
#define ROCKET_TILT_CONFIRM_MS              100U
#define ROCKET_TILT_MAX_SAMPLE_GAP_MS       20U

#define ROCKET_CRITICAL_ANGLE_DEG           65
#define ROCKET_IMU_FAIL_LIMIT               3U
#define ROCKET_IMU_IGNITION_GOOD_SAMPLES    10U

/*============================== 开伞方式 ==============================*/

/*
 * 四种开伞机制（代码均保留，用开关使能/关闭）：
 *   ROCKET_TILT_DEPLOY_ENABLE      姿态开伞：滚转/俯仰超临界角并持续确认后开伞
 *   ROCKET_ALTITUDE_DEPLOY_ENABLE  高度开伞：下降段降到指定高度开伞
 *   ROCKET_TIME_DEPLOY_ENABLE      定时强制开伞：到 ROCKET_FORCE_DEPLOY_TIME_MS 无条件开伞（兜底）
 *   ROCKET_SENSOR_FAIL_DEPLOY_ENABLE  高度/速度与IMU通道均失效时定时强制开伞
 *
 * 当前配置：持续超过 65 m 的高弹道在下降到 55 m 时开伞；未达到
 * 65 m 的低弹道在二级点火结束后确认下降 200 ms 时于最高点附近开伞，
 * 期间允许最长 60 ms 的速度抖动或短暂丢样。
 * 气压高度失效时改用IMU下降速度；正常高度/速度与IMU同时失效时，
 * 到 ROCKET_FORCE_DEPLOY_TIME_MS 强制开伞。姿态开伞与普通绝对时间
 * 开伞保留但关闭。
 */
#define ROCKET_TILT_DEPLOY_ENABLE          0
#define ROCKET_ALTITUDE_DEPLOY_ENABLE      1
#define ROCKET_TIME_DEPLOY_ENABLE          0
#define ROCKET_SENSOR_FAIL_DEPLOY_ENABLE   1
#define ROCKET_ALTITUDE_DEPLOY_M           55.0f
#define ROCKET_ALTITUDE_HIGH_FLIGHT_M      65.0f
#define ROCKET_ALTITUDE_HIGH_CONFIRM_MS    120U
#define ROCKET_ALTITUDE_LOW_DESCENT_CONFIRM_MS 200U
#define ROCKET_ALTITUDE_DESCENT_GAP_MS     60U

/*============================== 姿态参?==============================*/

#define ROCKET_TARGET_ROLL_DEG              0.0f
#define ROCKET_TARGET_PITCH_DEG             0.0f
#define ROCKET_IMU_MOUNT_ROLL_OFFSET_DEG    180.0f
#define ROCKET_IMU_MOUNT_X_SIGN             1.0f
#define ROCKET_IMU_MOUNT_Y_SIGN             -1.0f
#define ROCKET_IMU_MOUNT_Z_SIGN             -1.0f
#define ROCKET_ROLL_ANGLE_SIGN              1.0f
#define ROCKET_PITCH_ANGLE_SIGN             1.0f
#define ROCKET_ROLL_RATE_SIGN               1.0f
#define ROCKET_PITCH_RATE_SIGN              1.0f
#define ROCKET_ROLL_RATE_BIAS_DPS           0.0f
#define ROCKET_PITCH_RATE_BIAS_DPS          0.0f
#define ROCKET_GYRO_RANGE_DPS               2000.0f

/*============================== 串级控制 ==============================*/

#define ROCKET_ROLL_ANGLE_KP                4.0f
#define ROCKET_PITCH_ANGLE_KP               4.0f
#define ROCKET_RATE_TARGET_LIMIT_DPS        90.0f

#define ROCKET_ROLL_RATE_KP                 1.0f
#define ROCKET_ROLL_RATE_KI                 0.0f
#define ROCKET_ROLL_RATE_KD                 0.004f
#define ROCKET_PITCH_RATE_KP                1.0f
#define ROCKET_PITCH_RATE_KI                0.0f
#define ROCKET_PITCH_RATE_KD                0.004f
#define ROCKET_RATE_INTEGRAL_LIMIT_US       80.0f
#define ROCKET_CONTROL_LIMIT_US             300.0f
#define ROCKET_RATE_D_FILTER_ALPHA          0.2f

/* 正负号用于适配舵机安装方向?*/
#define ROCKET_ROLL_SERVO_SIGN              1.0f
#define ROCKET_PITCH_SERVO_SIGN             -1.0f

/*============================== 高度预测 ==============================*/

/* Height-prediction controller starts together with the attitude PID. */
#define ROCKET_ALTITUDE_TARGET_M            80.0f
#define ROCKET_ALTITUDE_CONTROL_START_MS    ROCKET_PID_START_DELAY_MS
#define ROCKET_ALTITUDE_CONTROL_PERIOD_MS   40U
#define ROCKET_ALTITUDE_MIN_CONTROL_SPEED_MPS 2.0f
#define ROCKET_ALTITUDE_DEADBAND_M          2.0f

#define ROCKET_ALTITUDE_STAGE_MASS_KG       0.4624f
#define ROCKET_ALTITUDE_REFERENCE_DIAMETER_M 0.0616f
#define ROCKET_ALTITUDE_AIR_DENSITY_KG_M3   1.20f
#define ROCKET_ALTITUDE_GRAVITY_MPS2        9.80665f
/* Run5 validates the no-fin baseline and gives a conservative fin drag delta. */
#define ROCKET_ALTITUDE_BASE_CD             0.50f
#define ROCKET_ALTITUDE_FIN_DRAG_K          5.00f
#define ROCKET_ALTITUDE_CD_MIN              0.10f
#define ROCKET_ALTITUDE_CD_MAX              2.50f

#define ROCKET_ALTITUDE_VELOCITY_MIN_DT_MS  20U
#define ROCKET_ALTITUDE_VELOCITY_MAX_DT_MS  200U
#define ROCKET_ALTITUDE_VELOCITY_FILTER_ALPHA 0.25f
#define ROCKET_ALTITUDE_ACCEL_1G_MG         1000.0f
#define ROCKET_ALTITUDE_ACCEL_VERTICAL_SIGN 1.0f
#define ROCKET_ALTITUDE_ACCEL_VERTICAL_BIAS_MG 0.0f
#define ROCKET_ALTITUDE_IMU_VELOCITY_MAX_DT_MS 50U
#define ROCKET_ALTITUDE_IMU_SPEED_LIMIT_MPS 100.0f
#define ROCKET_ALTITUDE_BARO_SPEED_CORRECTION 0.05f
/* Reject stale pressure data and motor-pressure spikes before prediction. */
#define ROCKET_ALTITUDE_BARO_STALE_MS        120U
#define ROCKET_ALTITUDE_BARO_IMU_SPEED_GATE_MPS 20.0f
#define ROCKET_ALTITUDE_BARO_RECOVERY_SAMPLES 3U

#define ROCKET_ALTITUDE_COST_HEIGHT         1.0f
#define ROCKET_ALTITUDE_COST_DEFLECTION     0.000001f
#define ROCKET_ALTITUDE_COST_DEFLECTION_CHANGE 0.000003f
#define ROCKET_ALTITUDE_MAX_DEFLECTION_STEP_DEG 3.0f

/*============================== ߶ȶ ==============================*/

/*
 * 0: only calculate and log suggested fin deflection.
 * 1: mix suggested altitude deflection into the two control servos.
 */
#define ROCKET_ALTITUDE_SERVO_ENABLE        1
#define ROCKET_ALTITUDE_FIN_US_PER_DEG      5.5556f
#define ROCKET_CONTROL_ATTITUDE_WEIGHT      0.2592f
#define ROCKET_CONTROL_HEIGHT_WEIGHT        0.7408f
/*
 * The 55 g ballast raises the stage mass by 13.5 percent.  Flight-log RMS
 * demand is handled by the runtime attitude-first allocator.  The height cap
 * is 166.7 us so the requested 30 degrees can be reached when the vehicle is
 * level.  Attitude PID still has runtime priority when physical actuator
 * headroom is insufficient.
 */
#define ROCKET_ALTITUDE_SERVO_LIMIT_DEG     30.0f
#define ROCKET_ALTITUDE_ROLL_MIX_SIGN       1.0f
#define ROCKET_ALTITUDE_PITCH_MIX_SIGN      -1.0f
#define ROCKET_ALTITUDE_HEIGHT_PRIORITY_ENABLE 0
/*============================== 舵机参数 ==============================*/

#define ROCKET_SERVO_PWM_PERIOD_US          20000U
#define ROCKET_CONTROL_SERVO_MIN_US         1000U
#define ROCKET_CONTROL_SERVO_CENTER_US      1500U
#define ROCKET_CONTROL_SERVO_MAX_US         2000U

#define ROCKET_PARACHUTE_SERVO_MIN_US       500U
#define ROCKET_PARACHUTE_SERVO_MAX_US       2500U
#define ROCKET_PARACHUTE_INIT_ANGLE_DEG     0U
#define ROCKET_PARACHUTE_DEPLOY_ANGLE_DEG   90U

#endif
