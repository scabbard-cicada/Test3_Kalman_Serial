<h1 align="center">Test3_Kalman_Serial</h1>

RoboMaster 自瞄笔试题：**极坐标观测的 EKF 目标跟踪 + 云台串口闭环**。

- 上位机（ROS 2）：从固定站点测得的极坐标 `(r, θ)`，用扩展卡尔曼滤波（EKF）估计目标
  状态，预测提前量，算出云台 yaw 角，经串口发给下位机。
- 下位机（STM32F407 / C 板）：解析串口命令，用串级 PID 驱动 GM6020 电机指向目标，
  并把电机实际角度回传。
- 没有硬件时，`fake_mcu` 用伪终端模拟下位机，跑的是和固件相同的控制代码。

题目和提交要求见 [`QUESTION.md`](QUESTION.md)。**开始之前先读 `docs/` 下的材料，顺序见 QUESTION。**

## 目录结构

```text
Test3_Kalman_Serial/
├── QUESTION.md                     题面：任务、要改的文件、编译运行、提交要求
├── README.md                       本文件
├── check_env.sh                    环境自检，缺依赖会打印修复命令
├── compile.sh                      Ninja 编译脚本
├── docs/                           七篇教学文档，阅读顺序见 QUESTION
│   ├── Part_of_Architecture.md     全局架构与出题意图
│   ├── Part_of_Frames.md           坐标系、角度约定、硬件与 fake/real 说明
│   ├── Part_of_KF.md               线性卡尔曼滤波
│   ├── Part_of_EKF.md              EKF 与极坐标观测
│   ├── Part_of_Communication.md    系统协作、时延、前馈、安全状态机
│   ├── Part_of_Serial.md           串口协议 C1/C2、CRC、流式解析
│   └── Part_of_General_STM32.md    下位机通识
├── src/
│   ├── interfaces/                 自定义消息（观测、指令、反馈、串口状态）
│   ├── virtual_locator/            极坐标观测回放，充当传感器输入
│   ├── target_tracker/             EKF 与云台指令，算法核心（要改）
│   ├── target_tracker_visualizer/  发布 Marker 与 TF，供 Foxglove 显示
│   ├── gimbal_serial_driver/       串口收发 + 软件下位机 fake_mcu（要改协议）
│   └── tracker_launcher/           一键启动编排 + 三个校验脚本
└── lower_computer/
    ├── COLCON_IGNORE               让 colcon 跳过固件目录
    └── gimbal_fw/                  下位机固件（CubeMX 工程，完整提供）
```

顶层 `build/`、`install/`、`log/` 是编译产物，已被 `.gitignore` 忽略，不用管。

## 快速开始

```bash
# 1. 环境自检（缺什么会打印修复命令）
bash check_env.sh

# 2. 编译
./compile.sh
source install/setup.bash

# 3. 没有 C 板：软件下位机
ros2 launch tracker_launcher tracker.launch.py mcu:=fake

# 4. 接真 C 板（在基地；port 按自己电脑实际设备名填）
ros2 launch tracker_launcher tracker.launch.py mcu:=real port:=/dev/ttyUSB0
```

启动后 Foxglove bridge 自动监听 `ws://localhost:8765`。浏览器打开
<https://app.foxglove.dev>，连接 Foxglove WebSocket，导入布局
`src/tracker_launcher/foxglove/target_tracker_layout.json`。

## 测试

单元测试（完成对应算法文件后应全部通过）：

```bash
source install/setup.bash
colcon test --packages-select target_tracker gimbal_serial_driver
colcon test-result --verbose
```

运行时检查（整套 launch 跑起来后另开终端）：

```bash
ros2 run tracker_launcher verify_serial.py
ros2 run tracker_launcher verify_ekf.py
ros2 run tracker_launcher verify_gimbal.py
```

## 下位机固件

见 [`lower_computer/gimbal_fw/README.md`](lower_computer/gimbal_fw/README.md)：接线、构建、
烧录、指示灯、PID 整定与安全注意事项。

---

祝顺利 😊😊😊

<div align="right">—— 浙江大学 RoboMaster 算法组</div>
