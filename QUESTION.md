<h1 align="center">Kalman and Serial</h1>

## 一、项目简介

一个固定在站点上的设备观测一个运动目标，给出目标的极坐标观测（距离和方位角）。上位机用扩展卡尔曼滤波（EKF）估计目标的位置、速度、加速度，算出云台应该指向的角度，通过串口发给下位机；下位机用串级 PID 驱动 GM6020 电机转向这个角度，并把电机实际角度回传。上位机把指令和反馈都画出来，形成完整闭环。

本仓库是一套可以完整运行的工程。下位机固件、观测回放、可视化、串口收发框架都已经写好，你要写的是上位机的算法与协议几个文件。
## 二、开始之前

### 1. 环境

需要 ROS 2（Jazzy 或 Lyrical 均可）。依赖分两类：

```bash
# 系统库
sudo apt install libeigen3-dev nlohmann-json3-dev libboost-dev

# ROS 2 包
sudo apt install ros-$ROS_DISTRO-rclcpp ros-$ROS_DISTRO-rclcpp-components \
                 ros-$ROS_DISTRO-geometry-msgs ros-$ROS_DISTRO-std-msgs \
                 ros-$ROS_DISTRO-visualization-msgs ros-$ROS_DISTRO-tf2 \
                 ros-$ROS_DISTRO-tf2-ros ros-$ROS_DISTRO-std-srvs \
                 ros-$ROS_DISTRO-foxglove-bridge
```

配套工具：

```bash
sudo apt install python3-colcon-common-extensions ninja-build cmake build-essential
```

仓库根目录有两个脚本：

- `check_env.sh`：环境自检。它逐项检查系统版本、ROS 2 是否已 source、编译工具、ROS 2 依赖包、系统库、串口设备与权限、Foxglove 端口占用，缺什么就打印对应的修复命令。装好依赖后先跑它，确认输出里没有「缺失」项再往下做；遇到任何环境问题，也先跑它，再把输出贴出来提问。
- `compile.sh`：编译整个工作空间。

跑到 `check_env.sh` 显示「环境检查通过」就可以了。如果它报出缺失项，按它给的命令逐条装上，再重跑一次。

### 2. 文档学习顺序

按这个顺序读 `docs/` 下的材料：

| 顺序 | 文档 | 内容 |
|---:|---|---|
| 1 | `Part_of_Architecture.md` | 全局地图：系统组成、功能包划分、题目为什么这样设计 |
| 2 | `Part_of_Frames.md` | 坐标系与角度约定、硬件接线、fake 与 real |
| 3 | `Part_of_KF.md` | 线性卡尔曼滤波：预测、更新、各个矩阵的含义 |
| 4 | `Part_of_EKF.md` | EKF 与本题的极坐标观测：线性与非线性、雅可比、初始化、环绕、门控 |
| 5 | `Part_of_Communication.md` | 上位机与下位机怎么协作：闭环、频率、时延、前馈、安全状态机 |
| 6 | `Part_of_Serial.md` | 串口协议：C1/C2 帧、CRC、字节序、流式解析与重同步 |
| 7 | `Part_of_General_STM32.md` | 下位机通识：读上位机代码时该知道板子在做什么 |

第一篇建立整体印象，第二篇确定「谁在哪、朝哪」，中间三篇对应算法，后两篇对应协议和下位机。

### 3. 两个阶段

阶段一：软件下位机。 先在自己的电脑上把整条链路跑通。用 `mcu:=fake`，软件下位机模拟 C 板，不需要任何硬件。EKF、云台指令、协议都要在这一阶段调对。

**阶段二：来基地用真实下位机和电机联调。** 软件下位机只用一套简化模型，能验证逻辑对不对，但反映不了真实的延迟、噪声和电机特性。前馈、PID 这些要调出效果的参数，必须在真实下位机和电机上做。 所以在软件下位机上跑通、确认逻辑正确之后，来基地接上真实的 C 板和 GM6020 电机，用真机把参数调好、把效果看实。

两阶段的侧重点不同：逻辑在电脑上验证，参数和效果在基地的真机上调。

## 三、需要完成的内容

要改的是四个文件，对应四条考察主线。每个文件它对应的文档见上表。

| 文件 | 要完成的内容 | 对应文档 |
|---|---|---|
| `src/target_tracker/src/kalman_filter.cpp` | 极坐标观测模型、雅可比、首帧初始化、CA 预测、EKF 更新（角度环绕、离群点门控、协方差更新） | KF、EKF |
| `src/target_tracker/src/gimbal_command.cpp` | 由未来状态生成云台 yaw 与角速度前馈；根据观测新鲜度选择命令阶段 | EKF、Frames、Communication |
| `src/target_tracker/src/tracker_node.cpp` | 观测回调里 EKF 的完整生命周期（首帧、时间步、断流重置）；命令定时器里把阶段结果封装成云台指令 | Communication |
| `src/gimbal_serial_driver/src/protocol.cpp` | CRC8、C2 命令帧编码、C1 反馈帧解码、流式解析与重同步 | Serial |

每个文件里都留了 `TODO`，注释说明了要做什么和边界条件，但没有给实现。写完对应算法文件后，可以直接运行仓库已经提供的 C++ 单元测试，不需要自己编写测试文件。

```bash
source install/setup.bash
colcon test --packages-select target_tracker gimbal_serial_driver
colcon test-result --verbose
```

测试与代码的对应关系：

| 测试文件 | 检查内容 |
|---|---|
| `src/target_tracker/test/test_kalman_filter.cpp` | EKF 模型、雅可比、初始化、角度环绕、门控、预测 |
| `src/target_tracker/test/test_gimbal_command.cpp` | yaw、角速度前馈、RELAX/TRACK/HOLD/RETURN_HOME |
| `src/gimbal_serial_driver/test/test_protocol.cpp` | CRC、C1/C2 编解码、半包、粘包、错误帧重同步 |

修改 C++ 文件后，先重新运行 `./compile.sh`，再执行上述测试。单元测试不需要 C 板、电机或真实串口；通过后再启动 fake MCU 做整条链路检查。

## 四、编译与运行

编译：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
./compile.sh
source install/setup.bash
```

运行（软件下位机）：

```bash
ros2 launch tracker_launcher tracker.launch.py mcu:=fake
```

启动后 Foxglove bridge 会自动监听 `ws://localhost:8765`。浏览器打开 <https://app.foxglove.dev>，连接 Foxglove WebSocket，导入布局文件：

```text
src/tracker_launcher/foxglove/target_tracker_layout.json
```

导入后可以看到目标轨迹、云台指令射线和反馈射线的对比。

### 单元性检查脚本

整套 launch 运行后，可以在另一个终端分别运行以下检查。它们只订阅话题并输出结果，不修改系统：

```bash
source install/setup.bash
ros2 run tracker_launcher verify_serial.py
ros2 run tracker_launcher verify_ekf.py
ros2 run tracker_launcher verify_gimbal.py
```

三个脚本分别检查：

- `verify_serial.py`：串口是否连接、C1 回传率、RTT、CRC/长度错误、序号跳变；
- `verify_ekf.py`：观测和滤波输出是否配对、数值是否有限、NIS 是否异常、轨迹是否在运动；
- `verify_gimbal.py`：云台命令和反馈是否持续、下位机是否 ARMED、跟踪误差是否在容差内。

使用 `mcu:=fake` 时软件下位机默认自动解锁；使用真实 C 板时，需要先按 KEY 进入 ARMED，再运行 `verify_gimbal.py`。

接真实下位机时（在基地）：

```bash
ros2 launch tracker_launcher tracker.launch.py mcu:=real port:=<你的串口设备>
```

`port` 是 USB-TTL 模块在你电脑上的设备名，每台电脑和每个 USB 口都可能不一样，不能照抄 `ttyUSB0`。插上模块后用下面任一命令查看：

```bash
ls /dev/ttyUSB*              # 常见结果：/dev/ttyUSB0、/dev/ttyUSB1……
ls /dev/serial/by-id/        # 按模块唯一编号列出，名字更稳定
```

把看到的设备名填进 `port`。比如：

```bash
ros2 launch tracker_launcher tracker.launch.py mcu:=real port:=/dev/ttyUSB1
```

几点提示：

- 同一台电脑插多个 USB 设备时，编号可能变来变去。想固定，用 `by-id` 目录里的名字，它跟的是模块本身而不是插拔顺序。
- 串口默认只有 `dialout` 组能读写。如果启动时报权限不足，把自己加进该组再重新登录：`sudo usermod -aG dialout $USER`。
- 设备名填错或模块没插好时，启动日志里会显示串口连接失败，换一个名字重试即可。运行 `check_env.sh` 也能看到当前有几个串口设备。

## 五、提交要求

在项目根目录建一个 `result/` 目录，放两样东西。

### 1. 录屏视频（`.mp4`）

用 `mcu:=fake` 启动整条链路，录一段 Foxglove 的视频，展示：

- 你的 launch 启动，节点和 bridge 都起来
- Foxglove 已经连接，3D 面板里能看到目标轨迹和云台两条射线
- 绿色滤波轨迹和红色原始观测一致
- 指令射线和反馈射线的关系正常

不要拍屏。录屏时把 Foxglove 窗口和终端放在同一个画面里。

### 2. 报告 `report.md`

Markdown即可 ，放在 `result/` 目录下，回答三个问题即可，没有标准答案，写真实想法：

1. 你这道题是什么时候开始做的？
2. 这道题里你的收获是什么？
3. 培训过程中，哪些地方让你感觉不太好？

如果哪一部分没做完，也一并说明卡在哪里。

## 六、发送

压缩打包时：

- 保留 `result/` 目录及其中内容
- 不要把 `.git/` 目录打包进来
- 不要打包 `docs/`、`build/`、`install/`、`log/`
- 视频太大就先压一下，或者降低分辨率和码率
- 删除散落在各个文件夹里的中间编译产物

提交前建议从零走一遍（`rm -rf build install log` 后重新编译），确认能跑通。

把整个 `.zip` 发到：

```text
scabbardcicada@gmail.com
```

---

祝顺利 😊😊😊

<div align="right">—— 浙江大学 RoboMaster 算法组</div>
