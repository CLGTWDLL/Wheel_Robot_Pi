# 高度跟踪重设计：任务空间 → 关节空间 + h↔θ 查表

记录 2026-05-13 的诊断与重构过程：把腿部高度控制从 task-space VMC 改为 joint-space PID + 离线生成的 h↔θ 查表，根治 `verify_cmd_height_tracking.py` 在高档翻倒的问题。

## 背景与诊断

### 原状

启动 `launch_mujoco` 不动 cmd 滑块时，机器人沿 −Y 方向单向漂移（10 s 漂 1.24 m，稳态 `vel_y ≈ −0.14 m/s`）。运行 `verify_cmd_height_tracking.py` 在 nominal=0.16 处于 t=0.274 s 翻倒（pitch=−1.51）。

### 两条独立根因

1. **`pitch_trim` 与模型不匹配**
   - `STAND_PARAMS.pitch_trim = -0.07` 是早期 CoM 偏置的拟合值
   - commit `c3bd654` ("Adjust base_link CoM to perfectly align horizontally with wheel centers") 之后真实平衡 pitch 接近 0
   - 启动瞬间 LQR 误差 `0 − (−0.07) = 0.07` 把 wheel ctrl 立即推到 ±1 饱和；系统稳定在 `pitch ≈ −0.02, vel ≈ −0.15 m/s` 的偏置均衡
   - 证据：`logs/manual/run_20260513_213408/telemetry.csv`

2. **任务空间 VMC 在 5-bar 奇异处放大力矩**
   - 任务空间反馈 `spring_force = kp_height·height_error − damping·vertical_velocity`，再经 `jacobian` 拼回 motor 力矩
   - 5-bar linkage 接近高/低位极限时雅可比奇异 → 力矩被放大
   - 加上 ±5 N·m 余量本就只有 30 %（实测总质量 10.24 kg，初始 `dh/dθ ≈ 0.069 m/rad`，单腿支撑约 3.46 N·m）
   - 高档 nominal=0.16 处的奇异叠加饱和直接导致翻倒

### 通过 probe 测得的机构限位

`scripts/probe_leg_geometry.py`：关重力 + 钉住 base + actuator PID + 末尾 300 步均值。

- 可用 `θ_motor` 范围：[0.438, 0.603] rad（约 9.4°宽）
- 对应 kinematic `h` 范围：[0.134, 0.149] m（跨度仅 0.0146 m）
- `h(θ)` 单调
- 之前 task-space VMC 在重力载荷下能"拉伸" equality constraint，跑出 ~0.04 m 跨度，但代价是奇异处不稳

## 重构方案

```
┌─────────────┐                  ┌───────────┐  θ_target  ┌──────────────┐
│ cmd_height  │ → clamp → LUT →  │ θ_target  │ ──────────→│ 关节 PID +   │ → τ_motor
│ slider      │   (kinematic)    │ per side  │            │ 重力前馈     │
└─────────────┘                  └───────────┘            └──────────────┘
                                                                ↑
                                                          current θ, θ̇
```

不再有"奇异处反馈无穷"问题：θ 空间内 PID 是线性问题；雅可比只用作前馈幅值，不进入反馈环。

## 修改文件清单

| 路径 | 状态 | 用途 |
|---|---|---|
| `scripts/probe_leg_geometry.py` | 新增 | 离线扫 θ 生成 LUT；何时重跑见文件顶部 docstring |
| `sim/controllers/leg_height_lut.json` | 新增（生成物） | `theta_grid` / `h_grid` 与边界 |
| `sim/controllers/leg_height_lut.py` | 新增 | `LegHeightLUT` dataclass + `motor_angle_from_height` |
| `sim/controllers/vmc.py` | 改 | `VmcParams` 用 `kp_motor/kd_motor` 替 `kp_height/kd_height`；`_control` 查表 + 关节 PID + 现有重力前馈 |
| `sim/controllers/default_params.py` | 改 | STAND / STAND_THEN_DRIVE：`nominal_height=0.142`、`kp_motor=50`、`kd_motor=4`；`params_to_dict / from_dict` 同步换字段 |
| `sim/model_xml.py` | 改 | `cmd_height` ctrlrange `"0.04 0.16"` → `"0.135 0.148"` |
| `sim/optimize.py` | 改 | Optuna search space 与 warm-start 切换到 `kp_motor / kd_motor` 并把 `nominal_height` 收到 [0.135, 0.148] |
| `tests/test_vmc_geometry.py` | 改 | `VmcParams` 构造参数名 |
| `scripts/verify_cmd_height_tracking.py` | 改 | `NOMINAL_HEIGHTS` 改为 `np.linspace(LUT.h_min+ε, LUT.h_max−ε, 5)`；`MIN_POS_Z_SPAN` 改为 `0.6 × LUT span` |

## 关键 LUT 数据（写入时）

- 模型实测总质量：**10.24 kg**（不是早期文档里的 7.7 kg）
- 单腿支撑重力所需力矩：**~3.46 N·m**，ctrlrange ±5 余量约 30 %
- LUT θ 范围：**[0.4380, 0.6027] rad**
- LUT h 范围：**[0.1339, 0.1485] m**
- LUT 单调：**True**
- cmd_height ctrlrange：**[0.135, 0.148]**

## 验证

`uv run python scripts/verify_cmd_height_tracking.py` → **PASS**

| nominal | pos_z | leg_h | pitch | roll |
|---|---|---|---|---|
| 0.1349 | 0.1663 | 0.1366 | 0.0342 | 0.0001 |
| 0.1381 | 0.1695 | 0.1397 | 0.0291 | 0.0001 |
| 0.1412 | 0.1727 | 0.1429 | 0.0224 | 0.0001 |
| 0.1444 | 0.1761 | 0.1463 | 0.0132 | 0.0001 |
| 0.1475 | 0.1796 | 0.1498 | 0.0003 | 0.0001 |

- `pos_z_span = 0.0133` ≥ 阈值 0.0076
- `pearson_r = 0.9996`
- `max|pitch| = 0.0342`、`max|roll| = 0.0001`、不翻倒
- 注意到稳态 `leg_h ≈ nominal + 1.7 mm` 的恒定偏置——是 LUT 在零重力下测得的 kinematic h 与加载后实际 h 的固定差，对线性度无影响

`uv run pytest tests/ -q` → **19 passed**（含 `test_vmc_geometry`）

## LUT 失效条件

下面任一变更后需要重跑 `scripts/probe_leg_geometry.py`：

- `sim/robot/robot.urdf` — 关节几何、质量、mesh 引用
- `sim/model_xml.py` — 闭环 equality 锚点、keyframe、actuator 定义
- `sim/mujoco_mesh_preprocess.py` — mesh 处理逻辑

## 未完成 / 后续工作

- **viewer 手测**：本轮没有真正打开 viewer 拖滑块验证 cmd_height 平滑跟踪
- **跳跃相位**：`JumpPhase.CROUCH/EXTEND` 现在用 `crouch_offset/extend_offset` 加到 `target_height` 再经 LUT clamp 转 motor 角，未在 jump 场景实测
- **`docs/lessons_learned.md` 追加**：本次根因（雅可比奇异 + 力矩饱和导致 task-space VMC 不稳）和解决方案值得进入 lessons_learned 主索引

## 2026-05-14 残留问题记录

本轮已完成：

- `scripts/probe_leg_geometry.py` 输出 `dy_grid` / `pitch_eq_grid`
- `LegHeightLUT` 增加 `dy_wheel_dh()` / `pitch_eq_from_height()` / `height_dtheta()`
- `CombinedController` 默认用 LUT 的高度相关 `pitch_trim`
- `VmcController` 增加 `max_height_rate` 和 `last_target_motor_rate`
- `CombinedController` 把升降几何 wheel velocity 注入 LQR target
- 新增 `scripts/verify_startup_drift.py`

通过的验证：

- `uv run pytest tests/test_leg_height_lut.py tests/test_vmc_geometry.py tests/test_balance_roll_6d.py -q` → 19 passed
- `uv run python scripts/verify_startup_drift.py` → PASS，末段 `wheel_vel=0.0000 m/s`、`wheel_pos_drift=0.0000 m`

未通过的验证：

`uv run python scripts/verify_cmd_height_tracking.py` → FAIL

关键输出：

- `max_abs_pitch=0.0362`
- `max_base_xy_drift=0.0561`
- `pos_z_span=0.0133`
- `pearson_r=0.9996`
- `fell=False`

当前残留问题：

- 启动漂移已经被新脚本验证通过，但高度阶跃严格目标还没达到。
- 当前模型 probe 的几何量与原计划假设不一致：
  - `pitch_eq_from_height(0.142) = -0.020507 rad`，不是计划里的 `-0.17 rad`
  - `dy_wheel_dh(0.142) = 0.163775`，不是计划里的约 `+5`
- 因为 `dy_wheel_dh` 实测量级很小，当前几何 wheel target 前馈对高度阶跃 pitch 峰值影响有限。
- 低高度目标下稳态 pitch 本身约 `0.03 rad`，与“任意阶跃 |pitch| < 0.02 rad”的绝对阈值存在冲突。
- `verify_cmd_height_tracking.py` 现在把每段 step 的 `base_xy_drift` 作为严格指标记录；第一段从初始高度切到最低高度时漂移最大，需要再拆分“初始重定位”和“已稳定后的高度 step”。

建议后续只做一件事：

- 先重新定义并验证“高度阶跃期间”的测量窗口：warmup 到当前 nominal 稳定后，再施加单个 `cmd_height` 阶跃，分别记录 step 前后 0.5 s 内的 `pitch_peak_delta` 和 `base_xy_delta`。确认指标后，再决定是否需要改 LQR 权重或 VMC 力矩分配。
