# 跟踪器 -> 串口驱动：云台指令。串口驱动把它编码成一帧 C2 发给下位机。
#
# 角度都在云台系：yaw = 0 时云台指向站点朝向，向左（俯视逆时针）为正。

builtin_interfaces/Time stamp

float32 yaw          # rad，目标 yaw
float32 yaw_vel      # rad/s，角速度前馈（下位机用它在两帧之间插值，并加到速度环参考上）

uint8 MODE_RELAX = 0 # 卸力
uint8 MODE_TRACK = 1 # 跟随 yaw
uint8 MODE_HOLD = 2  # 保持当前角度
uint8 mode

bool target_valid    # 跟踪器认为目标有效
bool laser_on        # 请求打开激光
bool outlier         # 最近一次观测被判为离群点（下位机只用来闪灯）
