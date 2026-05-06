# 项目架构记忆

## 项目双层结构

```
wheel_robot/
├── control/          # 实机 CAN 控制（达妙 USB-CAN 串口协议）
│   ├── can_tool.py   # CLI 入口
│   ├── damiao_can/   # 达妙电机/IMU 协议编解码
│   ├── damiao_usbcan.py  # USB-CAN 虚拟串口 backend
│   └── fake_bus.py   # 离线测试用 bus
│
├── sim/              # MuJoCo 仿真（纯仿真，不调用 control/）
│   ├── controllers/  # LQR + VMC 控制器
│   ├── rollout.py    # headless rollout 基础设施
│   ├── optimize.py   # 参数优化器（random search）
│   ├── model_xml.py  # XML 模型可控化处理
│   └── launch_mujoco.py  # viewer 启动入口
```

**关键边界**: `sim/` 代码绝不调用 `control/` 模块。仿真通过 `data.ctrl` 驱动执行器，不通过 CAN 帧。

## 控制器架构

### LQR + VMC 组合控制器 (`sim/controllers/combined.py`)

```
SimState ──┬──► LqrController  ──► wheel torques [actuator 0,1]
           │    (pitch balance + forward drive)
           │
           └──► VmcController  ──► leg torques   [actuator 2,3,4,5]
                (height PD + jump phases)
```

- LQR 和 VMC 控制不同执行器通道，天然无冲突
- LQR 首次调用时惰性初始化：在平衡点线性化 → 投影到 4 维子空间 → 求解增益

### LQR 4 维降维 (`sim/controllers/balance_lqr.py`)

状态向量: `[pitch, pitch_rate, avg_wheel_pos, avg_wheel_vel]`

符号约定（由 `base_link` 绕X翻转180° 决定）:
- pitch = -wy_tangent（P[0] 符号 = -1.0）
- pitch_rate = -qvel[wy]（P[1] 符号 = -1.0，balance_tangent_state 也取负）
- 轮子使用 `WHEEL_FORWARD_SIGNS`（两轮均 -1.0，已验证并落实到代码）统一方向后取平均

流程:
1. `linearize_transition` 获取全状态 A(28×28), B(28×4)
2. 构建投影矩阵 P(4×28) 和控制选择矩阵 S(4×2)，均含方向符号
3. A_reduced = P @ A @ P^+, B_reduced = P @ B @ S
4. **单输入 LQR**: B_sum = B_reduced.sum(axis=1), `solve_discrete_lqr(A_reduced, B_sum, Q, R_single)` → K(1×4)
5. 物理增益 = (S @ ones) @ K（S 矩阵含 WHEEL_FORWARD_SIGNS 方向符号）

### VMC 高度控制 (`sim/controllers/vmc.py`)

- 纯 PD 弹簧阻尼（qfrc_bias 在 2 电机五连杆中不适用，已移除）
- 2 个腿电机按固定符号分配力矩（r1=-1, r9=+1）
- 平滑启动 ramp: 从初始高度 0.06 在 1.5s 内过渡到 nominal_height
- 跳跃时通过 `JumpPhaseMachine` 修改 target_height 和 damping

### 速度 PI 前馈 + yaw PI 修正 (`sim/controllers/combined.py`)

- `target_velocity` 可运行时修改，每步通过 LQR feedforward 通道注入
- `velocity_kp`（比例项）驱动轮子力矩跟踪目标速度
- `pitch_lean_gain` 在速度误差时微调目标 pitch 促进加速
- **yaw PI 修正**: `yaw_damping`(P) + `yaw_ki`(I) 通过差速力矩对抗偏航角速度
  - P 项: 即时阻尼偏航角速度
  - I 项: 积分偏航角速度，消除五连杆不对称导致的稳态偏航偏置
  - 差速力矩 = sign * (avg_forward ± yaw_correction)
- viewer 中 `--scenario drive` 使用梯形速度曲线循环演示
- **drive 场景使用 `STAND_THEN_DRIVE_PARAMS`**（非 STAND_PARAMS）

### 跳跃相位机 (`sim/controllers/phase.py`)

STAND → CROUCH → EXTEND → FLIGHT → LAND → RECOVER → STAND (+ FAIL 兜底)

转移条件:
- CROUCH→EXTEND: 时间到
- EXTEND→FLIGHT: 时间到
- FLIGHT→LAND: 有接触且向下速度，或超时
- LAND→RECOVER: 时间到
- RECOVER→STAND: 时间到

## 模型处理流程 (`sim/model_xml.py`)

`prepare_controlled_mujoco_xml` 输入原始 XML，输出可控 XML:

1. `prepare_mujoco_xml` — mesh 预处理（STL 分片）
2. `_make_mesh_paths_absolute` — mesh 路径绝对化
3. `_ensure_root_freejoint` — 添加 `freejoint name="root"`
4. `_mark_mesh_geoms_visual_only` — 所有 mesh geom 设为 contype=0/conaffinity=0
5. `_add_collision_proxies` — 添加 box/cylinder 碰撞代理，使用 contype/conaffinity 位掩码防止自碰撞：
   - 地面 plane: contype=1, conaffinity=1
   - 轮子 cylinder: contype=1, conaffinity=3（与地面和 body 都碰）
   - base/link box: contype=2, conaffinity=1（与地面碰，不互碰）
6. `_replace_actuators` — 生成 6 个 motor actuator（2 轮 + 4 腿）
7. `_expand_keyframes` — 扩展 keyframe qpos（插入 freejoint 7 维）

## 参数管理 (`sim/controllers/default_params.py`)

所有控制器参数的**唯一真值源**。禁止在其他文件硬编码参数。

- `STAND_PARAMS`: 300 轮 Optuna 优化（纯 PD VMC, score=-52.29, 10s 站稳 pitch=0.019）
- `STAND_THEN_DRIVE_PARAMS`: 300 轮 Optuna 优化（纯 PD VMC, score=-200.96, 前进 0.316m, 仍需后续优化）
- `params_to_json()` / `params_from_json()`: JSON 序列化/反序列化

`launch_mujoco.py` 和 `test_combined_rollout.py` 均引用此文件，不直接定义参数。

## 优化器设计 (`sim/optimize.py` & `sim/rollout.py`)

- **算法升级**: 基于 Optuna (TPE) 进行贝叶斯优化，替换了原有的暴力随机搜索。支持 `CombinedParams` 及相位参数的联合搜索。
- **并行加速**: 彻底重构了 `study.optimize` 的执行流，由于 Python GIL 在仿真与控制器交互中造成的瓶颈，多线程 (`n_jobs`) 加速无效。现改为使用 **多进程 (`multiprocessing.Pool`) + Optuna SQLite backend** 的方式。程序会自动检测机器 CPU 核数（如 Apple M5 的 10 个物理核），并发启动对应数量的 Worker 进行评估，实现对计算资源的最大化利用。
- **提前终止 (Early Stopping)**: `sim/rollout.py` 的仿真循环中加入了提前截断机制：
  - 若 `base_position[2] < initial_z - config.fall_height_drop`，触发跌倒截断（`fell`）。使用相对初始高度的下降量，而非绝对 z 阈值。
  - 若俯仰角 `abs(pitch) > 1.5`，触发失控截断（`excessive_pitch`）。
  - 若为 `drive` 场景，1秒后位移小于 0.05m，触发停滞截断（`stagnation`）。
- **分场景评分**: `score_metrics(metrics, scenario)` 按 stand/drive/jump 分别加权：
  - stand: 100*pitch + 50*height_deviation 为主，存活 +10
  - drive: 20*dist 为主，50*pitch 惩罚，存活 +5
  - jump: 200*jump_height 为主，30*pitch 惩罚
- **warm start**: `--warm-start` 会通过 `study.enqueue_trial` 将已知的 `STAND_PARAMS` 和默认相位参数放入第一轮评估。
- CLI: `uv run python -m sim.optimize sim/轮腿v1/轮腿v1.xml --trials N --duration T --scenario stand`

## 执行器与关节映射

| actuator index | 名称 | 关节 | 控制器 |
|----------------|------|------|--------|
| 0 | act_robot_link2_left_r3 | 左轮驱动 | LQR |
| 1 | act_robot_link2_right_r7 | 右轮驱动 | LQR |
| 2 | act_robot_base_link_r1 | 左腿电机1 | VMC |
| 3 | act_robot_base_link_r2 | 左腿电机2 | VMC |
| 4 | act_robot_base_link_r4 | 右腿电机1 | VMC |
| 5 | act_robot_base_link_r9 | 右腿电机2 | VMC |

轮子 `ctrlrange=[-3.0, 3.0]`（基于 DMH6215 T_MAX=10Nm，保守取值）。
腿 `ctrlrange=[-3.5, 3.5]`（基于 DM4310 额定 3.5Nm）。

## VMC 控制方式

VMC 使用纯 PD 控制（已去除 qfrc_bias 重力补偿）：
```python
control[act_idx] = LEG_EFFORT_SIGNS[joint_name] * pd_effort
```
qfrc_bias 在只有 2 个腿电机的闭链五连杆中产生过大的耦合力矩（机器人会飞起来），因此不适用。

## 硬件 CAN 配置

| 设备 | CAN_ID | Master/MST_ID | 控制模式 |
|------|--------|---------------|----------|
| DM-IMU | 0x01 | 0x11 | 读取 |
| DM4310 左 | 0x02 | 0x12 | MIT 控制模式 |
| DM4310 右 | 0x03 | 0x13 | MIT 控制模式 |
| DMH6215 左 | 0x04 | 0x14 | 速度控制模式 |
| DMH6215 右 | 0x05 | 0x15 | 速度控制模式 |

## 仿真日志系统
- **位置**: 所有的日志数据统一保存在项目根目录的 `logs/` 目录下。
- **模式**:
  - `manual` (`sim/launch_mujoco.py`): 会保存系统事件文本日志 `system.log` 以及详细的每帧物理状态和控制量 `telemetry.csv`，按运行时间命名，例如 `logs/manual/run_YYYYMMDD_HHMMSS/`。
  - `auto_tuning` (`sim/optimize.py`): 为了避免数千次迭代导致小文件暴增及磁盘IO瓶颈，默认仅保存汇总评估和参数日志至 `logs/auto_tuning/opt_YYYYMMDD_HHMMSS/optimization.log`。
- **日志模块**: 封装在 `sim/logger.py` 的 `setup_system_logger` 和 `TelemetryLogger` 中，`TelemetryLogger` 支持流式低开销 CSV 写入。
- **Pose 数据**: `telemetry.csv` 包含完整 6-DOF pose: 位置(xyz)、四元数(qw,qx,qy,qz)、欧拉角(roll,pitch,yaw)、线速度(xyz)、角速度(xyz)。

## 碰撞代理系统 (`model_xml.py`)

- **轮子 geom**: `contype=1, conaffinity=3, friction=1.0` — 唯一的地面驱动接触
- **碰撞代理 box**: `contype=2, conaffinity=1, friction=0` — 与地面碰撞提供法向力（跌倒保护），但 friction=0 避免阻碍水平运动
- **`<contact><exclude>`**: 排除同一运动链上所有相邻 body 对的碰撞，防止碰撞代理互穿产生寄生力
- **注意**: `collision_link2` 底面在 keyframe 初始姿态接近地面 (z≈0.01)，VMC 需要数秒才能将机体推过此高度。修改碰撞代理尺寸时需要验证世界坐标系下的投影位置。
