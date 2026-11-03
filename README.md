# MANUS Metagloves Pro / Pro Haptic 与 ROS 2 仿真

Windows MANUS Core → Ubuntu ROS 2 → RViz 手部骨架 / MuJoCo 五指 Shadow Hand。项目保留 Vicon 与触觉的诊断记录，所有遥操作代码**仅用于仿真**。支持 Pro Haptic 作为输入不等于触觉功能已经验证。

## 实际完成情况

- Windows Core 3.2.1 Remote → Ubuntu 24.04、ROS 2 Jazzy：左右手骨架曾经实时跟随。
- MuJoCo Shadow Hand：右手五指曾在现场各自跟随；手腕 IMU 原地旋转三个方向与五指同时跟随已由操作者确认。
- Vicon Tracker 4.3 → Core：标定杆刚体的位姿两侧出现且移动时变化；不是手腕刚体或时间同步验证。MANUS 官方 Vicon 文档以 Shogun Live 为对象，Tracker 4.3 仅为现场结果。
- 触觉：单指尝试无感，Core 对所接模块报告非 Haptic；硬件型号和配套模块仍待核对，**没有成功振动或停止的硬件证据**。
- MR4：仅官方资料研究，未现场联调。见 [实现与验证记录](docs/implementation.md)。

## 安装

Ubuntu 安装 ROS 2 Jazzy、colcon，Windows 安装 MANUS Core 3.2.1。项目文件和构建目录可按需要放在挂载的数据盘上。手套接收器保持接在 Windows；两台设备网络互通。

1. 从 [MANUS 3.2.1 下载页](https://docs.manus-meta.com/3.2.1/Resources/)下载 **MANUS Core 3 SDK (Linux)**，不将 SDK 包、头文件或二进制提交至本仓库。
2. 使用 SDK 压缩包和 Windows 的实际 IPv4 地址：

```bash
./scripts/prepare_sdk.sh /path/to/MANUS_Core_3.2.1_SDK_Linux.tar.gz WINDOWS_CORE_IPV4
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

脚本从压缩包提取官方 ROS 2 包、消息包和 SDK，应用 [Remote 补丁](patches/manus-3.2.1-remote.patch)。补丁在指定 IP 未被发现时拒绝连接其他 Core，并检查 SDK 的非成功返回；该 IP 固定写入本地忽略的 SDK 源码，改 IP 后要重新构建。已有 SDK 目录时脚本拒绝覆盖。Windows 上设备和授权、Core 配置仍须用户自行确认。

3. 模型与 Python 依赖（在仓库目录执行）：

```bash
mkdir -p third_party
git clone https://github.com/google-deepmind/mujoco_menagerie.git third_party/mujoco_menagerie
git -C third_party/mujoco_menagerie checkout f3b6013629c7b94f8ee93fd977b161fa8350f9bc
python3 -m venv --system-site-packages .venv
.venv/bin/pip install 'mujoco==3.17.0' numpy
```

模型版权与许可证以第三方仓库为准。项目不包含 MANUS SDK、模型资产、构建结果、日志、设备 ID 或标定记录。

## ROS 2 与 RViz

Windows 启动 Core，手套上电并完成校准。Ubuntu 终端启动一次发布器：

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch manus_hand_viz live.launch.py
```

该 launch 已启动发布器，请勿重复启动。查看实际话题：`ros2 topic list -t`。可视化使用相对手腕的显示坐标，不是空间定位。无手套时，可分别运行 `ros2 run manus_hand_viz mock_gloves` 和 `ros2 launch manus_hand_viz view.launch.py`；模拟数据不代表设备联调。

## MuJoCo 五指与 IMU

```bash
./scripts/sim.sh test                 # 合成输入的五指检查
./scripts/sim.sh offline              # 离线自动五指演示
export MANUS_RIGHT_TOPIC=/manus/right/glove_YOUR_HEX_ID
./scripts/sim.sh live                 # 真实手指输入
./scripts/sim.sh imu                  # IMU 原地旋转 + 手指输入
```

`MANUS_RIGHT_TOPIC` 改为 `ros2 topic list` 查到的实际右手话题。真实输入需要先运行且只运行一个 MANUS 发布器。点击 MuJoCo 窗口：手指张开按 `O`、握拳按 `F`、`E` 启用；空格暂停。IMU 模式自动设参考并启动旋转，手掌朝下手指向前按 `R` 重新设参考。

IMU 轴映射是一次右手佩戴下的实验标定，跨设备、佩戴姿态和坐标系不保证正确；`R` 只重设朝向，不会重新学习轴映射。手腕位置固定，模型含前臂整体转动，不是前臂与手腕相对角度，更不包含空间平移。五指为每指一个弯曲量的简化映射。联系物体/抓取、力反馈、延迟精度均未验证。

MANUS `ManusGlove` 缺少源时间戳，官方节点可能重复缓存；ROS 消息仍在到达不证明硬件数据新鲜。不可将该演示接到真实机器人。

## 其他诊断

`tools/vicon/inspect_trackers.cpp` 和 `tools/haptics/inspect_capability.cpp` 为只读 SDK Remote 诊断，须先设置 `MANUS_CORE_IP`，具体编译方法见 [实现与验证记录](docs/implementation.md)。`tools/haptics/pulse_test.py` 默认离线；真实发送需明确 `--execute --side ... --finger ... --topic ...`，并先核实对应设备支持触觉且佩戴者就绪。ROS DDS 确认并不证明物理振动或停振；断网/进程故障时停止命令可能丢失。不要把工具用于不明型号的手套或真实系统。

## 上游资料

[MANUS ROS 2 官方文档](https://docs.manus-meta.com/3.2.1/Plugins/SDK/ROS2/getting%20started/) · [MANUS / Shadow Robot 官方案例](https://www.manus-meta.com/use-cases/teleoperating-the-shadow-hand-with-manus-gloves) · [MuJoCo Menagerie Shadow Hand](https://github.com/google-deepmind/mujoco_menagerie/tree/main/shadow_hand) · [MANUS 追踪器偏移](https://docs.manus-meta.com/3.2.0/Software/TrackingSystems/Tracker%20Offsets/)
