/**
 * @file    app_config.h
 * @brief   下位机所有可调参数集中在这里。改完重新编译烧录。
 *
 * 首次上电台架测试前必须核对的四项：
 *   1. GIMBAL_MOTOR_ID     与电机拨码一致
 *   2. GIMBAL_ZERO_ECD     云台机械零位对应的编码器值
 *   3. GIMBAL_DIR          云台向左转时 yaw 增大（俯视逆时针为正）
 *   4. GIMBAL_OUTPUT_SCALE 首次测试取 0.2，确认方向和零位后再改成 1.0
 */
#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

/* ---------------------------------------------------------------- 电机后端
 * GIMBAL_VIRTUAL_MOTOR = 1：不接电机，在 C 板上跑和 fake_mcu 相同的 GM6020 模型。
 * 只有 C 板时用它验证串口链路、指示灯和反馈回传；接上真电机前改回 0。
 * 也可以在 cmake 时用 -DGIMBAL_VIRTUAL_MOTOR=ON 覆盖。
 */
#ifndef GIMBAL_VIRTUAL_MOTOR
#define GIMBAL_VIRTUAL_MOTOR 0
#endif

/* GM6020 ID（背面拨码），1~4。反馈帧 ID = 0x204 + ID，控制帧 ID = 0x1FF */
#define GIMBAL_MOTOR_ID 1u

/* 机械零位：云台指向"正前方"时读到的编码器值（0~8191）。
   确定方法：卸力状态下把云台摆到正前方，看 C1 反馈或调试器里的 g_motor_raw.ecd */
#define GIMBAL_ZERO_ECD 0u

/* +1：电机正转（编码器增大）时云台向左；-1：相反 */
#define GIMBAL_DIR (+1)

/* 输出缩放 (0,1] */
#define GIMBAL_OUTPUT_SCALE 0.2f

/* ---------------------------------------------------------------- 限位与速度（度） */
#define GIMBAL_YAW_MIN_DEG (-35.0f)
#define GIMBAL_YAW_MAX_DEG (35.0f)
#define GIMBAL_YAW_VEL_MAX_DPS (120.0f)

/* ---------------------------------------------------------------- PID
 * 串级：角度环 → 速度环 → GM6020 电压给定。默认值在仿真模型上整定，实物需要重新调：
 *   1. 先把 ANGLE_KP 设 0，给固定速度指令，只调速度环（KP 加到开始抖再退一半，KI 消稳态误差）
 *   2. 再加 ANGLE_KP，从小往大加到阶跃响应超调 < 5%
 *   3. 最后让上位机带前馈跑完整轨迹，看 C1.yaw 与 yaw_ref 的差
 */
#define ANGLE_KP 20.0f
#define ANGLE_KI 0.0f
#define ANGLE_KD 0.0f
#define ANGLE_OUT_MAX 10.0f /* rad/s */

#define SPEED_KP 3000.0f
#define SPEED_KI 20000.0f
#define SPEED_KD 0.0f
#define SPEED_I_MAX 8000.0f
#define SPEED_OUT_MAX 25000.0f

/* ---------------------------------------------------------------- 通信 */
#define GIMBAL_FB_PERIOD_MS 10u /* C1 回传周期：100 Hz */

#endif /* APP_CONFIG_H_ */
