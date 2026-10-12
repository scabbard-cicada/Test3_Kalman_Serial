<h1 align="center">gimbal_fw —— 单轴 GM6020 云台下位机</h1>

RoboMaster 开发板 C 型（STM32F407IGH6）固件，通过 CAN1 驱动一台 GM6020 云台电机，
通过 USART6 与上位机通信。**本工程完整提供，是"已经写好的被通信对象"，笔试只考上位机。**

```
上位机 ──USART6──▶ 解析 C2 ──▶ 安全状态机 ──▶ 串级 PID ──CAN1──▶ GM6020
                     ▲                             │
                     └────── 编码 C1 ◀── 反馈 ◀────┘
```

## 目录

```
gimbal_fw/
├── gimbal_fw.ioc            CubeMX 配置，可重新生成代码
├── Core/  Drivers/          CubeMX 生成，用户代码只写在 USER CODE 区
├── App/                     与 HAL 打交道的一层
│   ├── app_config.h         所有可调参数集中在这里
│   └── app.c                串口/CAN/定时器/指示灯/按键/看门狗
├── Shared/                  与硬件无关，固件、软件下位机、主机测试共用
│   ├── gimbal_proto.c/.h    C1/C2 协议、CRC8、流式解析
│   ├── pid.c/.h             PID（积分限幅、积分分离、微分先行）
│   ├── gm6020.c/.h          GM6020 编解码 + 电机模型
│   └── gimbal_ctrl.c/.h     安全状态机 + 参考生成 + 串级 PID
└── tests/                   主机端单元测试
```

## 硬件接线

| 功能 | 引脚 | 说明 |
|:---|:---|:---|
| 上位机串口 | USART6：PG14 TX / PG9 RX | C 板 3-pin 口，**外壳丝印是 "UART1"**，线序 GND / TXD / RXD |
| GM6020 | CAN1：PD0 RX / PD1 TX，1 Mbps | 2-pin 口，线序 CANL / CANH |
| RGB 灯 | PH10 B / PH11 G / PH12 R | 共阳，高电平点亮 |
| 蜂鸣器 | PD14（TIM4_CH3） | 无源，4 kHz |
| 激光 | PC8（TIM3_CH3） | 5 V 接口 |
| 按键 | PA0 | 按下为低电平 |

上位机串口：3.3 V 电平 USB-TTL（CH340 / CP2102），**TX 接 C 板 RX、RX 接 C 板 TX**，共地。
**不要接 5 V 那根线**，否则会和 C 板供电互相倒灌。

> C 板外壳丝印和 STM32 实际串口不对应：丝印 `UART1` = USART6，丝印 `UART2` = USART1。

## 构建

需要 ARM GNU 工具链（随 STM32CubeCLT 安装）在 PATH 里：

```bash
export PATH=/path/to/stm32cubeclt/GNU-tools-for-STM32/bin:$PATH
cmake --preset Debug
cmake --build --preset Debug
```

产物：`build/Debug/gimbal_fw.elf`、`.hex`、`.bin`。

不接电机、只用 C 板验证串口链路和指示灯：

```bash
cmake --preset Debug -DGIMBAL_VIRTUAL_MOTOR=ON   # 板上跑电机模型，CAN 不发
```

主机端单元测试（不依赖 HAL）：

```bash
cmake -S tests -B build/host-tests
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
```

重新生成 CubeMX 代码（改了 `.ioc` 之后）：

```bash
printf 'config load %s/gimbal_fw.ioc\nproject generate\nexit\n' "$PWD" > /tmp/gen.txt
STM32CubeMX -q /tmp/gen.txt
```

App/ 和 Shared/ 里的代码不受重新生成影响（不在 CubeMX 管理范围内）。

## 烧录

- **SWD**（推荐）：ST-Link / DAPLink，`STM32_Programmer_CLI -c port=SWD -w build/Release/gimbal_fw.hex -v -rst`
  或 OpenOCD；VS Code 里有 STM32Cube 扩展可直接下载。
- **DFU**（没有调试器时）：BOOT0 跳线置高，接 micro USB，复位进 DFU，
  用 STM32CubeProgrammer 下载 `build/Release/gimbal_fw.bin`。

## 指示灯与蜂鸣器

| 状态 | RGB | 蜂鸣器 | 电机 |
|:---|:---|:---|:---|
| 上电 | 白灯闪一下 | 短鸣 | 卸力 |
| 没有串口数据 | 蓝灯慢闪 | — | 卸力 |
| 有数据、未解锁 | 蓝灯常亮 | — | 卸力 |
| 有数据但电机不在线 | 蓝红交替 | — | 卸力 |
| TRACK、目标有效 | 绿灯，越接近越亮 | — | 跟随 |
| 本帧是离群点 | 黄灯闪一下 | — | 跟随 |
| HOLD / 目标无效 | 黄灯慢闪 | — | 保持角度 |
| 指令超时 | 红灯快闪 | 长鸣 | 保持→卸力 |
| 故障 | 红灯常亮 | 3 短鸣 | 卸力 |

不接电脑也能从灯的颜色判断链路处在哪一步。

## PID 整定

控制是串级：角度环（只用 P）输出目标角速度 → 速度环（PI）输出 GM6020 电压给定。
上位机发来的 yaw 角速度作为前馈加到速度环参考上。参数在 `App/app_config.h`。

默认增益是按 `gm6020.c` 里的仿真模型整定的，**实物必须重新调**。建议步骤（每次只调一个）：

1. 把 `ANGLE_KP` 设 0，`SPEED_KI` 设 0，给一个固定速度指令，只调 `SPEED_KP`：
   从小加大，加到电机开始轻微抖动，再退回一半。
2. 加 `SPEED_KI`：消除稳态误差；出现低频摆动或声音发闷就减小。
3. 加 `ANGLE_KP`：给角度阶跃指令，加到超调小于 5%（约 100 ms 到位）。
4. 让上位机带前馈跑完整条轨迹，看可视化里反馈射线贴不贴指令射线，或看 C1 的
   `yaw` 与 `yaw_ref` 的差。前馈关掉再跑一次对比。

首次上电务必先把 `GIMBAL_OUTPUT_SCALE` 设为 0.2。

## 首次上电前的确认清单

1. **电机 ID**：`GIMBAL_MOTOR_ID` 与 6020 拨码一致；CAN 两端 120 Ω 终端电阻接好
   （只接一台电机时打开 6020 上的终端电阻拨码）。
2. **方向**：`GIMBAL_DIR`。卸力状态下手动把云台向左转，反馈的 yaw 应该增大；
   不对就把 `GIMBAL_DIR` 取反。
3. **零位**：`GIMBAL_ZERO_ECD`。卸力状态下把云台摆到"正前方"，用调试器看 `g_motor_raw.ecd`，
   填进 `GIMBAL_ZERO_ECD`。
4. **限位**：`GIMBAL_YAW_MIN/MAX_DEG` 按实际机构设，默认 ±35°。
5. 架空云台或卸掉负载，确认以上都对，再把 `GIMBAL_OUTPUT_SCALE` 恢复成 1.0。

## 安全说明

- 电机单独接 24 V（电池或带限流的电源）。C 板只插 micro USB 时 STM32 能运行，
  但 PWM 口所在的 VCC_5V_M 网络不供电。
- 上电默认 SAFE，必须按 KEY 才解锁。上位机无法远程解锁。
- 看门狗只在主循环喂；中断卡死约 500 ms 后自动复位，复位原因通过 C1 上报。
- 激光只在上位机请求、且 ARMED + TRACK + 目标有效时点亮。不要对着人眼。

---

祝顺利 😊😊😊

<div align="right">—— 浙江大学 RoboMaster 算法组</div>
