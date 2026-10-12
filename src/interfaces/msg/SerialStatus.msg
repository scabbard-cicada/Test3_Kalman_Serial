# 串口驱动的链路统计，1 Hz 发布

builtin_interfaces/Time stamp

bool connected           # 串口当前是否打开

uint32 tx_frames         # 累计发出的 C2
uint32 rx_frames         # 累计收到并校验通过的 C1
uint32 crc_errors        # 累计 CRC 错误
uint32 len_errors        # 累计长度字节错误
# 回显序号跳变之和：下位机回传的 echo_seq 每跳一下，说明中间的 C2/C1 至少丢了一帧。
# 上行（C2 没到下位机）和下行（C1 没到上位机）的丢包都会体现为这个数，无法区分。
uint32 echo_gaps
uint32 reconnects        # 累计重连次数

float32 rx_rate_hz       # 最近 1 s 收到的 C1 帧率（下位机按 100 Hz 发）
float32 rtt_ms_mean      # 最近 1 s 往返时延均值
float32 rtt_ms_max       # 最近 1 s 往返时延最大值
