# 串口驱动 -> 其它节点：下位机回传的一帧 C1，解码后原样发布。

builtin_interfaces/Time stamp   # 上位机收到这一帧的时刻

float32 yaw          # rad，电机实测，云台系
float32 yaw_vel      # rad/s，电机实测
float32 yaw_ref      # rad，下位机插值/限幅后真正送进控制环的参考
int16 current        # GM6020 转矩电流原始值
uint8 temperature    # °C

# 下位机安全状态
uint8 STATE_SAFE = 0     # 上电默认：卸力，等按键解锁
uint8 STATE_ARMED = 1    # 已解锁
uint8 STATE_TIMEOUT = 2  # 指令中断：保持最后角度
uint8 STATE_FAULT = 3    # 故障：卸力，按键清除
uint8 state

# error_flags 位
uint8 ERR_MOTOR_OFFLINE = 1
uint8 ERR_OVER_TEMP = 2
uint8 ERR_CMD_CLAMPED = 4
uint8 ERR_CMD_TIMEOUT = 8
uint8 ERR_BAD_FLOAT = 16
uint8 ERR_CAN = 32
uint8 ERR_UART = 64
uint8 ERR_WDG_RESET = 128
uint8 error_flags

uint8 echo_seq       # 下位机最近一次采纳的 C2 序号
float32 rtt_ms       # 由 echo_host_time_ms 算出的往返时延；没有可用回显时为 -1
