# 项目当前结构记录

本文件记录当前代码状态。历史调参过程见 `docs/lessons_learned.md` 和 `docs/retuning_guide_2026-05-16.md`。

## 项目双层结构

```text
wheel_robot/
├── control/          # 实机 CAN 控制（达妙 USB-CAN 串口协议）
│   ├── can_tool.py   # CLI 入口
│   ├── damiao_can/   # 达妙电机/IMU 协议编解码
│   ├── damiao_usbcan.py
│   └── fake_bus.py
│
├── sim/              # MuJoCo 仿真（纯仿真，不调用 control/）
│   ├── controllers/  # LQR + VMC 控制器
│   ├── rollout.py    # headless rollout 基础设施
│   ├── optimize.py   # Optuna 参数优化器
│   ├── model_xml.py  # URDF→MJCF 后处理
│   └── launch_mujoco.py
```

关键边界：`sim/` 不调用 `control/`。仿真通过 MuJoCo `data.ctrl` 驱动执行器，实机控制通过 CAN 帧。

## 仿真模型来源

当前仿真源文件是 `sim/robot/robot.urdf`。运行时由 `sim/model_xml.py:prepare_controlled_mujoco_xml()` 转成 MJCF，并追加以下内容：

- root freejoint
- visual-only mesh 设置
- wheel sphere collision proxy，半径 `WHEEL_RADIUS = 0.036`
- base/link collision proxy
- link2/link3 的 equality connect 约束
- `cmd_linear_x`、`cmd_angular_z`、`cmd_height`、`cmd_jump` 命令滑块
- `stand` keyframe
- 可选 `single_wheel_trapezoid` 地形

不要用旧 `sim/轮腿v1/` XML 判断当前结构。

## 控制器架构

### LQR + VMC 组合控制器

```text
SimState
  ├─ CombinedController / LqrController
  │    5D balance state -> forward_wheel + roll_diff_leg
  │    -> wheel torques + roll-diff leg torque
  │
  └─ VmcController
       cmd_height -> LUT -> active motor angle target
       -> joint-space PID + gravity feedforward
```

`CombinedController` 会把 LQR 轮输出、LQR roll-diff 腿输出、VMC 腿输出和 yaw 差速合成后写入物理 actuator。命令滑块 actuator 只用于输入，不是物理执行器。

### 5D 平衡 LQR

当前主控制路径使用 5D LQR：

```text
[pitch, pitch_rate, roll, roll_rate, avg_wheel_vel]
```

代码路径：

- `sim/controllers/combined.py` 调用 `compute_balance_lqr_gain_5d()`
- `LqrController` 使用 `balance_tangent_state_5d` 作为状态提供函数
- `q_diag` 长度必须为 5
- `wheel_pos` 不在内环 LQR state 中

`balance_tangent_state_6d()` 和 `compute_balance_lqr_gain()` 仍保留给历史测试和对照，不是 `CombinedController` 当前主路径。

`target[4]` 是 `wheel_vel` 目标：

- STAND：`target_velocity + height feedforward + position anchor 外环输出`
- FLIGHT：释放为 `0`
- LAND：不使用位置目标

静止漂移由慢速 position anchor 外环转成速度目标处理，不把 `wheel_pos` 放回 LQR 内环。

### VMC 高度控制

当前 VMC 使用 2 个主动腿电机：

- 左腿：`base_link_旋转-2`
- 右腿：`base_link_旋转-1`

被动关节：

- `base_link_旋转-4`
- `base_link_旋转-3`
- `link1_left_旋转-6`
- `link1_right_旋转-5`

高度路径：

```text
cmd_height / nominal_height
  -> max_height_rate 限速
  -> LegHeightLUT.motor_angle_from_height()
  -> active motor angle target
  -> kp_motor/kd_motor joint-space PID
  -> gravity feedforward
```

`qfrc_bias` 不用于当前 VMC。闭链几何相关计算通过 `LEG_CLOSED_LOOP`、equality constraint Jacobian 和 `mj_jac` 完成。

### 跳跃相位机

当前 phase：

```text
STAND -> CROUCH -> EXTEND -> FLIGHT -> LAND -> STAND
```

另有终态：

```text
FALLEN
```

没有 `RECOVER`、`SELF_RIGHT`、`FAIL`。进入 `FALLEN` 后保持终态，控制器输出零，需要重启仿真复位。

`cmd_jump` 是命令滑块，上升沿触发一次 `start_jump()`。

### 速度、yaw 和位置外环

- `cmd_linear_x` 写入 `CombinedParams.target_velocity`，单位 m/s。
- `cmd_angular_z` 写入 `CombinedParams.target_yaw_rate`，单位 rad/s。
- yaw 通过左右轮差速实现。
- `target_velocity == 0` 且处于 STAND 时，position anchor 外环会把位置漂移转换成小速度目标。
- 行驶时清空 position anchor，避免位置外环把车拉回原点。

## 当前默认参数

`sim/controllers/default_params.py` 是控制器参数真值源。

`STAND_PARAMS` 当前关键值：

- `q_diag = [1000, 200, 1000, 200, 500]`
- `r_diag = [200, 400]`
- `velocity_ki = 0.3`
- `pitch_lean_gain = 0.15`
- `position_kp = 3.0`
- `position_kd = 1.5`
- `position_velocity_limit = 0.6`
- `yaw_damping = 0.5`
- `fixed_height = False`
- `flight_wheel_q = 1.0`
- `ff_gain = 4.0`
- VMC `nominal_height = 0.142`
- VMC `kp_motor = 30.0`
- VMC `kd_motor = 1.0`
- VMC `roll_level_kp_height = 0.0`
- VMC `roll_level_kd_height = 0.0`
- VMC `roll_level_offset_limit = 0.0`
- VMC `jump_extend_feedforward = 8.0`

`STAND_THEN_DRIVE_PARAMS` 已替换旧 Optuna 参数，当前与 `STAND_PARAMS` 同一量级，`jump_extend_feedforward = 9.0`。

`pitch_trim` 已删除。自然平衡 pitch 由 `equilibrium_pitch_from_geometry()` 按当前几何计算。

## 执行器与命令滑块

物理 actuator 由 `MODEL_SEMANTICS` 决定，不要硬编码 index。

当前物理 actuator：

- 2 个 wheel actuator：LQR forward/yaw
- 2 个 active leg actuator：VMC + LQR roll-diff

当前命令 actuator：

- `cmd_linear_x`，范围 `[-1.0, 1.0]`
- `cmd_angular_z`，范围 `[-3.0, 3.0]`
- `cmd_height`，范围 `[0.118, 0.142]`
- `cmd_jump`，范围 `[0, 1]`

物理 actuator 限幅：

- wheel：`[-1.0, 1.0]`
- leg MJCF actuator：`[-12.5, 12.5]`
- leg 在 `STAND` / `CROUCH` / `LAND` 阶段由控制器软限幅到 `[-3.5, 3.5]`

`EXTEND` 阶段允许腿部短时峰值，用于跳跃。

## 当前启动与优化命令

受控 viewer：

```bash
uv run mjpython sim/launch_mujoco.py sim/robot/robot.urdf
```

平地：

```bash
uv run mjpython sim/launch_mujoco.py sim/robot/robot.urdf --flat-ground
```

参数优化：

```bash
uv run python -m sim.optimize sim/robot/robot.urdf --trials N --duration T --scenario stand
```

默认 controlled 模式会加入 `single_wheel_trapezoid` 地形；`--flat-ground` 会关闭它。

## 硬件 CAN 当前配置

| 设备 | CAN_ID | Master/MST_ID | 控制模式 |
|---|---:|---:|---|
| DM-IMU | `0x01` | `0x11` | 读取 |
| DM4310 左 | `0x02` | `0x12` | MIT |
| DM4310 右 | `0x03` | `0x13` | MIT |
| DMH6215 左 | `0x04` | `0x14` | MIT |
| DMH6215 右 | `0x05` | `0x15` | MIT |

4 台电机当前都走 MIT 帧，`arbitration_id = CAN_ID`。反馈按 Master/MST_ID 路由。

## 日志

- `manual`：`logs/manual/run_YYYYMMDD_HHMMSS/`
- `auto_tuning`：`logs/auto_tuning/opt_YYYYMMDD_HHMMSS/`
- telemetry 包含 pose、速度、轮状态、接触数、控制量等字段。
