# rbpianist_cl — 双臂机器人钢琴 MuJoCo 仿真与复验

基于 MuJoCo 被动键（passive-key）钢琴场景的双臂演奏仿真包：RM75 双臂 + L20 灵巧手，
回放已标定的双手《两只老虎》40 音计划，并在同一次连续仿真中核对实际落键/释放边沿。
本仓库面向 **sim-to-real**：MuJoCo 仿真/复验子集 + 真机控制驱动包。
仿真侧（`rm_mujoco`/`scripts`）不含真机控制逻辑；真机侧由 `rm_driver`（机械臂）、
`rm_l20_driver`（灵巧手）、`rm_piano_player`（演奏节点）、`rm_ros_interfaces`
（消息/服务）提供。**尚未做真机验证**，已发布结果均为仿真证据。

## 目录结构

标准 ROS2 工作空间：ROS2 功能包位于 `src/`，MuJoCo 仿真资产以源码树形式留在仓库根目录。

```
src/                  ROS2 功能包（colcon 从此处发现并构建）
  rm_driver/          睿尔曼 RM 机械臂 ROS2 驱动（含厂商 libapi_cpp.so）
  rm_l20_driver/      L20 灵巧手 RS485 驱动（单/双手节点）
  rm_piano_player/    真机演奏节点与曲目配置
  rm_ros_interfaces/  驱动所需 msg/srv 定义
  rm_control/         机械臂跟随控制节点（bringup 依赖）
  rm_bringup/         真机一键启动 launch（driver+control+description+MoveIt+RViz）
  rm_description/     URDF/网格/TF 与双臂 joint_states 桥
  rm_moveit2_config/  MoveIt2 规划配置（子包 rm_75_config）
  rm_moveit2/         MoveIt2 运动演示接口（simple_moveit_node）
config/               场景布局（桌面/基座/钢琴框架/键盘/相机）与右手仿真参数
models/               钢琴键盘与 RM75+双 L20 的 MJCF 模板
meshes/               RM75 与 L20 右手网格（STL）
rm_mujoco/            场景构建、键盘边沿、指法、轨迹采样与回放运行时
scripts/              复验、回放与视频渲染入口
examples/             冻结的 40 音基准（score.yaml / performance.yaml / manifest.json）
verification/         一次已通过的复验证据（report.json / run.json / run.log）
```

构建 ROS2 工作空间（在仓库根目录）：

```bash
colcon build                          # 或 colcon build --packages-up-to rm_bringup
source install/setup.bash
```

## 环境依赖

- Ubuntu 22.04、Python 3.10、MuJoCo 3.3.7、NumPy、PyYAML
- 无头渲染（Jetson/SSH）：`MUJOCO_GL=egl`
- 视频渲染另需：`ffmpeg`；配音另需 `fluidsynth` 与 GM 音色库（默认
  `/usr/share/sounds/sf2/FluidR3_GM.sf2`）

以源码树方式使用（无需安装 ROS 包）：

```bash
package="$PWD"
export PYTHONPATH="$package:$package/scripts${PYTHONPATH:+:$PYTHONPATH}"
export MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1   # 无头环境
```

## 入口

### 1. 冻结基准复验（推荐验收方式）

先校验 `manifest.json` 的输入/模型/配置/网格/实现身份与 40 音音高、手别、固定指法、
onset/end，再调用生产回放器在同一次仿真中执行双手。输出目录必须尚不存在：

```bash
python3 scripts/verify_two_tigers.py --package "$package" \
  --output .validation/two_tigers_verified_40
```

成功要求退出码 0 且 `playback/report.json` 的 `passed=true`、`observed=40`。

### 2. 回放任意 `piano_performance/1` 计划

```bash
python3 scripts/run_performance.py --package "$package" \
  --plan examples/two_tigers_flat_palms/verified_40/performance.yaml \
  --output /tmp/two_tigers_playback
```

可选 `--tempo-scale`、`--timing-tolerance`、`--key-press-deg/--key-release-deg`、
`--camera {overview,front,keys}` 等。

### 3. 渲染演奏视频（带钢琴声）

复用回放的模型与轨迹（运动与已验证回放一致），离屏逐帧渲染管道给 ffmpeg，
再由实际落键边沿合成 MIDI 并用 fluidsynth 配音封装：

```bash
python3 scripts/render_video.py --package "$package" --output /tmp/two_tigers_video
```

默认输出 `two_tigers_overview.mp4`（全景）与 `two_tigers_keys.mp4`（键盘特写），
1280x720@30fps，绿色高亮为被按下的键。可用 `--views`、`--fps`、`--width/--height`、
`--no-audio`、`--soundfont` 调整。

## 启动步骤（ROS2 / 真机）

仿真回放/渲染见上文「入口」（源码树模式，无需 ROS）。以下为 ROS2 侧启动；每个终端先
`source install/setup.bash`。

### 1. 构建工作空间

```bash
cd rbpianist_cl
colcon build                       # 全量；真机演奏链也可 --packages-up-to rm_piano_player
source install/setup.bash
```

Jetson(aarch64) 首次需在 `src/rm_driver/lib/` 执行 `sudo ./lib_install.sh` 安装 arm64 厂商库。

### 2. MoveIt fake 演示（无硬件，已验证）

```bash
colcon build --packages-up-to rm_moveit2   # 无需厂商库/sudo
source install/setup.bash
```

终端 A 启动假执行 bringup（rsp + mock ros2_control + move_group + 控制器）：

```bash
ros2 launch rm_75_config demo_6fb_v.launch.py use_rviz:=false
```

待日志出现 “You can start planning now!” 且 `ros2 control list_controllers` 显示
`rm_group_controller`、`joint_state_broadcaster` 均 active，终端 B 运行演示节点：

```bash
ros2 launch rm_moveit2 moveit_rm75.launch.py planning_group:=rm_group
```

`simple_moveit_node` 依次执行：启动关节目标 → 笛卡尔圆弧/直线 → 回零；全程 `mock_components`
假执行，不动真机。

### 3. 真机启动（务必先完成「上机前检查」）

真机演奏由 `rm_piano_player` 统一编排，fail-closed：双手就绪 → 双臂 bringup → 演奏节点。

```bash
ros2 launch rm_piano_player rm_75_dual_l20_piano_real.launch.py \
  left_arm_ip:=<左臂IP> right_arm_ip:=<右臂IP> \
  left_udp_ip:=<主机IP> right_udp_ip:=<主机IP> \
  left_udp_port:=8089 right_udp_port:=8090 \
  midi_file:=<曲目.mid> \
  execute:=false use_rviz:=false
```

分级试跑（任一步异常即急停排查，不得跳步）：

1. `execute:=false` 起步，确认手就绪、臂使能、`/joint_states` 与左右
   `rm_group_controller/follow_joint_trajectory` 动作就位；
2. 单音慢速：`execute:=true tempo_scale:=0.5`，验证落键/释放；
3. 无异常后逐步提速到目标速度。

其他入口：

- 仅双臂 + MoveIt（不含手/演奏）：
  `ros2 launch rm_bringup rm_75_6fb_v_bringup.launch.py left_arm_ip:=<…> right_arm_ip:=<…>`
- 仅双手 RS485：`ros2 launch rm_l20_driver l20_dual_rs485.launch.py`

## 真机控制（sim-to-real）

真机包复制自完整 `ros2_rm_robot` 工作空间，保持原包名：

- `rm_driver`：RM75 机械臂 ROS2 驱动；`config/rm_75_config.yaml` 设 arm_ip/udp_ip；
  依赖 `lib/` 下厂商预编译 `libapi_cpp.so`（x86 与 arm64）。
- `rm_l20_driver`：L20 灵巧手 RS485 驱动（单/双手节点）；
  `config/hand_identity.yaml` 为左右手 UID，上电前须对照物理标签核对 L/R。
- `rm_piano_player`：真机演奏节点与曲目/仿真配置（`config/piano*.yaml`）。
- `rm_ros_interfaces`：驱动所需 msg/srv。
- `rm_control`：机械臂跟随控制节点（bringup 依赖）。
- `rm_bringup`：真机一键启动 launch，串起双臂 driver + control + 状态桥 + MoveIt + RViz。
- `rm_description`：URDF/网格/TF 与双臂 `joint_states` 桥（`dual_arm_joint_state_bridge.py`）。
- `rm_moveit2_config`：MoveIt2 双臂规划配置（子包 `rm_75_config`：SRDF/运动学/控制器；
  `demo_6fb_v` 仿真执行、`real_moveit_demo_6fb_v` 真机执行）。

真机一键启动（IP 按现场覆盖）：

```bash
ros2 launch rm_bringup rm_75_6fb_v_bringup.launch.py \
  left_arm_ip:=<左臂IP> right_arm_ip:=<右臂IP>
```

配置/launch 中的内网 IP 与手 UID 为实验室现场值，部署或公开前请按现场修改；
厂商 `.so` 的再分发请确认许可。**真机链路尚未验证。**

## 上机前检查（真机部署）

**真机链路尚未验证**（仅 MuJoCo 仿真与 MoveIt fake 执行已验证）。上机前逐项确认下列清单，
任何一项不满足都不要使能运动。分离线配置检查与现场必检，最后按分级流程试跑。

### A. 离线配置检查（连接硬件前）

1. **构建与厂商库**：仓库根 `colcon build` 通过；Jetson(aarch64) 在 `src/rm_driver/lib/`
   执行 `sudo ./lib_install.sh` 安装 arm64 `libapi_cpp.so` 到 `/usr/local/lib`（脚本按 `uname -m` 选架构）。
2. **控制器名一致（真机）**：真机由 `rm_control` 在 `left_arm`/`right_arm` 命名空间提供
   `rm_group_controller/follow_joint_trajectory` 动作（`src/rm_control/src/rm_control.cpp`），
   须与 `rm_75_config/config/moveit_controllers_dual.yaml` 的 `left_arm/rm_group_controller`、
   `right_arm/rm_group_controller` 一致。`ros2_controllers_dual.yaml`（`left_arm_controller`，mock JTC）
   仅用于 fake/gazebo，不参与真机。
3. **网络参数统一**：`arm_ip` 左/右（默认 192.168.1.18 / 192.168.1.19）、`tcp_port` 8080；
   `udp_ip` 必须为主机在臂网段的真实 IP，`udp_port` 左 8089 / 右 8090。
   **默认值不一致**：`rm_bringup` 默认 `udp_ip=192.168.1.100`，而
   `rm_piano_player/launch/rm_75_dual_l20_piano_real.launch.py` 默认 `192.168.1.10`——
   上机时按现场统一覆盖 `left_udp_ip`/`right_udp_ip`。
4. **手 UID 与串口**：`src/rm_l20_driver/config/hand_identity.yaml` 的
   `left_hand_uid=LHT20-011-260-L-B-1-D`、`right_hand_uid=LHT20-011-266-R-B-1-D`
   须对照物理标签核对 L/R；RS485 端口默认 `auto`（可用 `left_hand_port`/`right_hand_port` 指定），
   `baudrate=115200`、`slave_id=1`。
5. **标定值复核**：bringup 双臂安装 `left_xyz='0 -0.35 0.5'`、`right_xyz='0 0.35 0.5'`、rpy 均 0；
   `piano_frame` 固定于 `base_root`（`piano_xyz` 默认 0）。player 键几何（`piano_midi.yaml`，均在 piano_frame）：
   `key_x_start=0.30`、`key_x_step=0.014`、`key_z=0.312`、`approach_height=0.04`、`press_depth=0.008`。
   这些为实验室/近似值，须按实物重新标定后再上机。
6. **安全默认**：player `execute=false`（须显式 `execute:=true` 才运动）、
   `velocity_scaling=acceleration_scaling=0.10`。**`tempo_scale` 默认偏高**
   （真机 launch 4.0 / `piano_midi.yaml` 2.0），首次上机降到 ≤1.0（仿真基准为 1×）。

### B. 现场必检（上电/运动前）

- 急停可达且有效；工作区无人/无障碍；臂处于安全起始位姿；抱闸/使能按规程释放。
- 网络连通：`ping <左臂IP>`、`ping <右臂IP>` 通；主机 `udp_ip` 与臂同网段。
- 手 RS485 物理连接正确，L/R 与 UID 一致；`l20_wait_ready` 就绪
  （fail-closed：就绪失败或手驱动退出则不释放臂/演奏）。
- 启动后核对：`/joint_states` 含左右各 7 关节、TF 正确；`ros2 action list` 含
  `/left_arm/rm_group_controller/follow_joint_trajectory` 与右臂同名动作。
- 无头 Jetson 设 `use_rviz:=false`（rviz 无显示会退出，不影响链路）。

### C. 分级上电与试跑

按「启动步骤」§2–§3 执行：fake 预演 → 手就绪 + 臂 bringup（`execute:=false`）→
单音慢速（`tempo_scale:=0.5`、`execute:=true`）→ 整曲逐步提速。任一步异常即急停排查，不得跳步。

## 验收口径

- 40 音 = 右手 32 + 左手 8；原谱时间不变，仅统一加 1 s 准备时间
- 1× 速度、30 ms 时序容差、默认 0.55/0.30 行程按下/释放阈值、固定指法
- 通过条件：无缺音、无多余音、无错误指法/错键、无角度违规、结束无残留键、
  每手同时按下键数 ≤ 5、臂不在持音期间移动

## 输出说明

回放/复验产出 `report.json`（逐音触发、释放、时序误差、指法、错键、角度检查）、
`key_edges.json`、`key_angle_track.json`、`tracking.jsonl`、`raw_contacts.json`、
`played_keys.mid`；复验另含 `plan.yaml`、`run.json`、`run.log`。

## 限制与声明

- 场景为照片推导的近似几何，非实测硬件标定。
- 这是固定曲目的标定计划回放入口，不是任意曲谱自动规划器；不要将基准移用到其他曲谱。
- 模型或实现变化时应重新验收，不能只改 manifest hash 来宣称旧结论仍有效。
- 所有结果均为仿真证据，不代表真机运动或闭环验证。