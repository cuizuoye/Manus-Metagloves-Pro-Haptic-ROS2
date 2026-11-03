# 实现流程与验证边界

## 数据路径

1. Core 在 Windows 连接和标定手套。Linux 官方 SDK Remote 使用指定 Core IPv4 发现主机，不把手套接收器改接 Ubuntu。
2. ROS 包发布每只手的 raw skeleton、ergonomics 和 raw sensor orientation；话题按 side + glove ID 命名。当前官方 `ManusGlove` 无源时间戳，发布端可能重复缓存，必须用独立来源时间/质量信号验证新鲜度。
3. `manus_hand_viz` 绘制相对手腕的 MarkerArray。`sim/shadow_teleop.py` 使用五指各一个人体工学弯曲量，经 O/F 标定后映射至 Shadow 执行器。`sim/imu_teleop.py` 在此基础上读取手腕原始 IMU，将固定腕点的整只手作相对旋转；`R` 设置零点。
4. IMU 轴矩阵是某次实际右手标定的经验结果，并非 MANUS/Shadow 官方通用标定，初始姿态、佩戴、坐标变化后需重新验证。仿真仅运动学目标演示，不是机械臂控制；前臂也随之旋转。

## Vicon 观察与后续

已有 Tracker 场景中的标定杆移动时，ROS 和 MANUS SDK 追踪流都观察到同名 Vicon Prop 的运动。静止样本位置经过一个水平轴旋转近似匹配；未进行时间同步或完整坐标变换验证。需要另行安装并定义手背刚体，在 Core 分配对应手、标定刚体到手腕偏移，并检查遮挡、源数据新鲜度、轴方向；不能用标定杆代替手腕。ROS 直接接收刚体作为替代路线时，必须独立完成偏移和坐标标定。

## 编译只读 SDK 诊断

在仓库根目录，先完成 README 的 SDK 提取：

```bash
export MANUS_CORE_IP=WINDOWS_CORE_IPV4
sdk="$PWD/ros2_ws/src/ManusSDK"
g++ -std=c++17 -O2 -I"$sdk/include" tools/vicon/inspect_trackers.cpp -L"$sdk/lib/amd64" -Wl,-rpath,"$sdk/lib/amd64" -lManusSDK-amd64 -o /tmp/manus-inspect-trackers
/tmp/manus-inspect-trackers
```

`inspect_capability.cpp` 同样编译。只读连接将查询 Core landscape，不会写入 Tracker 设置、发送触觉或绑定用户。检查结果是采样快照，不自动证明源流一直新鲜。

## 触觉与 MR4

触觉单次尝试的目标虽收到 DDS 应答，但操作者未感觉振动，SDK 报告设备 `isHaptics=false`；这是 SDK 报告而非实物型号鉴定。手套织物标签是 Haptic，配套顶部模块待核实。请先确认 Core 识别结果及模块，不使用非零触觉命令排错。

Noraxon 的官方流程是 Core → MR4 的 Ultium Hardware Setup → MANUS Gloves；官方要求至少一个 Ultium Motion 传感器参与测量，MR 4.2026.2+ 的手指动画依官方条件配置。该流程并未在本项目现场验证。[Noraxon 指南](https://www.noraxon.com/article/how-to-integrate-manus-gloves-with-a-noraxon-ultium-motion-system/)

## 本机复现注意事项

Ubuntu 24.04 / NVIDIA T500 / MuJoCo 3.17.0 环境中，离线窗口可运行，但程序退出时出现过段错误；用与本项目无关的最小 MuJoCo passive viewer 也能复现。原因尚未确定，不影响此前的五指和 IMU 跟随观察，但不能视作退出流程稳定。请避免将它作为长时间无人值守系统，退出问题待单独排查。
