# rbpianist_cl — 双臂机器人钢琴 MuJoCo 仿真与复验

基于 MuJoCo 被动键（passive-key）钢琴场景的双臂演奏仿真包：RM75 双臂 + L20 灵巧手，
回放已标定的双手《两只老虎》40 音计划，并在同一次连续仿真中核对实际落键/释放边沿。
**仅仿真**，不包含 ROS 控制器、MoveIt 或 RS485 真机代码，也未做真机验证。

## 目录结构

```
config/      场景布局（桌面/基座/钢琴框架/键盘/相机）与右手仿真参数
models/      钢琴键盘与 RM75+双 L20 的 MJCF 模板
meshes/      RM75 与 L20 右手网格（STL）
rm_mujoco/   场景构建、键盘边沿、指法、轨迹采样与回放运行时
scripts/     复验、回放与视频渲染入口
examples/    冻结的 40 音基准（score.yaml / performance.yaml / manifest.json）
verification/ 一次已通过的复验证据（report.json / run.json / run.log）
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