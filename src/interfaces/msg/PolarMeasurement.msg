# 站点 S 处测得的目标距离与方位角
#
#   range   = |p - S|                    + 噪声
#   bearing = atan2(py - sy, px - sx)    + 噪声，世界系，wrap 到 (-π, π]
#
# 站点位置随每帧下发，跟踪器不必另读配置。

std_msgs/Header header

float64 range       # m
float64 bearing     # rad

float64 station_x   # m，世界系
float64 station_y   # m，世界系
