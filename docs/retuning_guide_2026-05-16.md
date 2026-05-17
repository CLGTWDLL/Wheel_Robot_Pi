# 2026-05-16 控制器重调指南（2.2 kg 模型）

## 背景

URDF 总质量从 CAD 默认 10.24 kg 改为实机称重 2.20 kg，触发 LQR/VMC 全套参数失效。本文记录完整诊断流程、关键现象、最终参数，以及未来上机部署时的注意事项。

**改动前后对比**

| 项 | 旧 | 新 |
|---|---|---|
| 总质量 | 10.24 kg | **2.20 kg** |
| base_link | 6.715 | **0.94** |
| wheel each | 0.955 | **0.50** |
| link1/2/3 each | 0.253/0.481/0.072 | **0.04/0.08/0.01** |
| CoM Z（世界） | 0.1423 m | **0.1059 m**（下移 36 mm） |
| 单腿支撑重力力矩 | ~3.46 N·m | **~0.74 N·m** |
| 摆杆长度 L=CoM_z − wheel_z | ~0.107 m | **0.0707 m** |
| LQR B[1,0] = -1/(M·L·r)·dt | -0.0507 | **-0.357**（×7.04） |

## 调参流程（按顺序复现）

### Step 1：被动行为评估

零控制器（`zero_controller`）跑 2s。脚本：`tmp/retune/01_passive.log`。

**现象**：0.488s 内 pitch 增到 1.50 rad（摔倒），pos_y 漂 +0.088 m。

**结论**：keyframe `stand` 不是平衡点。CoM 偏前，系统开环不稳。

### Step 2：原参数 baseline

`STAND_PARAMS`（kp_motor=80, kd_motor=2, Q=[5000,200,5000,200,50,500], R=[1,2]）。

**现象**：5.8s 内 pos_z 飙到 14.9 m、pitch 翻 180°、vel_x 1737 m/s、轮子饱和 41%、腿饱和 25%。**完全爆炸**。

### Step 3：诊断 LQR B 矩阵增益

```
B 矩阵 B[1,0] = -1/(M·L·r)·dt
新/旧 = (M_old × L_old) / (M_new × L_new) = (10.24×0.107) / (2.20×0.0707) = 7.04×
```

K[0,0]=-21.5 → 对 pitch=0.05 rad 直接 1.08 N·m 输出 → 饱和。

**最初尝试**（仅 R×4.65 按质量比例）：还在崩。
**最终发现**：B 取决于 M·L 乘积，需要 R×100 才能让 K[0,0] 不饱和。

### Step 4：发现"软 Q 才稳"

R×100 + Q=[5000,...,500]：站住但 y 漂 1.7 m，satW=83%。
R×100 + Q=[1000,200,1000,200,**50,500**]：**站住、y 漂 0.086 m、satW=4%**。

**关键洞察**：
- Q[0]/Q[2] (pitch/roll 权重) 从 5000 → 1000 让控制器**允许 ±0.03 rad 自然倾斜**以吸收 CoM 偏移
- Q[5] (wheel_vel 权重) 必须 ≤ 500，超过 800 立即崩（强行刹车导致摔倒）
- Q[1]/Q[3] (pitch_rate) 不能低于 200，否则崩

### Step 5：历史 pitch_trim 校准

`pitch_trim=-0.02`：drift 仅 0.027 m（最优）。
等价于 atan(CoM_y_offset / L) = atan(0.012/0.071) ≈ 0.17 rad，符号通过实测确定。

实际稳态 pitch=−0.029 rad（≈ 自然平衡 pitch）。

当前代码已删除 `pitch_trim` 参数，改为在 `CombinedController` 内用 `equilibrium_pitch_from_geometry()` 按当前几何实时计算自然平衡 pitch。

### Step 6：VMC kp_motor 缩放

kp_motor=80 → **30**（对应 0.21× 旧值，与质量比例 0.215 几乎一致）。
kd_motor=2 → **1.0**。
kp_motor=60+ 时再次失稳（同 R×100 下）。

## 当前参数（以 `sim/controllers/default_params.py` 为准）

```python
STAND_PARAMS = CombinedParams(
    vmc=VmcParams(
        nominal_height=0.142,
        kp_motor=30.0,
        kd_motor=1.0,
        crouch_offset=-0.02,
        crouch_target_height=0.09,
        crouch_duration=0.20,
        extend_offset=0.02,
        landing_damping=1.0,
        land_kp_scale=0.5,
        land_kd_scale=2.5,
        gravity_compensation_scale=1.0,
        max_height_rate=0.05,
        jump_extend_feedforward=8.0,
        jump_extend_feedforward_duration=0.05,
    ),
    q_diag=np.array([1000.0, 200.0, 1000.0, 200.0, 500.0]),
    r_diag=np.array([200.0, 400.0]),
    velocity_ki=0.3,
    pitch_lean_gain=0.15,
    position_kp=3.0,
    position_kd=1.5,
    position_velocity_limit=0.6,
    yaw_damping=0.5,
    yaw_ki=0.0,
    fixed_height=False,
    lqr_height_bin_size=0.02,
    flight_wheel_q=1.0,
    ff_gain=4.0,
)
```

当前 LQR 是 5D：`[pitch, pitch_rate, roll, roll_rate, wheel_vel]`。旧 6D 中的 `wheel_pos` 权重已删除。

`STAND_THEN_DRIVE_PARAMS` 已替换旧 Optuna 参数，当前同样使用 5D `q_diag=[1000,200,1000,200,500]`，`jump_extend_feedforward=9.0`。

## 验证结果（headless 5–10 s）

| 场景 | 结果 | 稳态 pitch | 漂移 | satW | satL |
|---|---|---|---|---|---|
| 静站 10 s | OK | −0.028 rad | 0.03 m | 0.8% | 0.1% |
| 前进 0.3 m/s | OK | −0.028 | 1.45 m（Y 方向） | 4.3% | 0.3% |
| 前进 0.5 m/s | OK | −0.028 | 2.40 m | 3.8% | 0.2% |
| 自转 0.5 rad/s | OK | −0.027 | 0.08 m | 4.3% | 0.1% |
| h=0.135 | OK | −0.028 | < 0.05 m | 1.6% | 0.1% |

**注意**：robot 前向是世界 Y 方向（轮子前后布局），不是 X。

## 已知限制 & 上限缩减

### cmd_height 上限：0.148 → **0.142**

`sim/model_xml.py` 的 cmd_height ctrlrange 收到 `[0.118, 0.142]`。原因：

- h=0.144 起出现 z 振幅 12 mm、pitch 振幅 0.42 rad、~3.7 Hz 自激振荡
- h=0.148 直接 0.18 s 摔倒
- LUT `d²h/dθ²` 在 [0.144, 0.149] 区间符号翻转（+0.105 → −0.251 → +0.773），雅可比剧变
- LQR 在 keyframe 处线性化，h 偏离时未重线性化

若要恢复 0.148 上限，需要：① 重训 LQR 在多个 h 下的增益表（gain scheduling）；② 平滑 LUT 边界；或 ③ 让 LQR 每步重线性化（开销大）。

### 跳跃高度恢复：0.045 m → 0.177 m

EXTEND 阶段已改为短时峰值窗口：MJCF 腿 actuator `ctrlrange` 允许 ±12.5 N·m，VMC/CombinedController 在非 EXTEND 相位继续把腿输出软限制到 ±3.5 N·m。

历史手动调通过的参数：
- `crouch_target_height=0.09`
- `crouch_duration=0.20`
- `extend_duration=0.05`
- `jump_extend_feedforward=11.5`

`tmp/jump_step_response.py` 实测 `max_jump_height=0.1770`、首次落地 `touchdown_abs_pitch=0.0830`。但落地恢复阶段仍有 `max_abs_pitch_after_touchdown≈0.18` 的摆动，尚未达到全恢复窗口 `<0.1 rad`。

当前默认参数已改为 `STAND_PARAMS.jump_extend_feedforward=8.0`、`STAND_THEN_DRIVE_PARAMS.jump_extend_feedforward=9.0`；phase 机不再有 `RECOVER`，LAND 满足接触和竖直速度条件后直接回 `STAND`。

## 实机部署注意事项

### 1. 先比对仿真 ↔ 实机几何

上机前用 `mujoco.MjModel.from_xml_path` 跑：
```python
print(f"total mass = {m.body_mass.sum()}")
print(f"CoM(world) = {(m.body_mass[:,None]*d.xipos).sum(0)/m.body_mass.sum()}")
```

应该看到 **total mass = 2.20 kg**。偏离即 URDF 被改过。

### 2. 初始姿态对齐

keyframe `stand` 的 pitch=0（base 水平），但 CoM 自然平衡点是 pitch=−0.028 rad（向 +y 微倾）。实机上电时若 IMU 读数偏差超过 ±0.03 rad，控制器会以为已经在偏移，给出反向修正力矩。

**建议**：上电瞬间机器人应保持在「轮子贴地、base 几乎水平」的姿态，让 IMU 自校准在 pitch≈0 附近，然后控制器自然把它稳到 pitch=−0.028。

### 3. 力矩对应关系

仿真 → 实机映射：

| 仿真量 | CAN 帧 | 注意 |
|---|---|---|
| `data.ctrl[wheel_*]` (N·m) | DMH6215 MIT 帧 `CAN_ID`，`t_ff` 字段 | 6215 在 MIT 模式下接收力矩前馈 |
| `data.ctrl[leg_*]` (N·m) | DM4310 MIT 控制帧 `CAN_ID`，`t_ff` 字段 | 同上 |

✅ **2026-05-16 起仿真和实机控制模式对齐**：4 个电机统一用 MIT 模式。仿真输出的 wheel torque 和 leg torque 可以直接通过 MIT 帧的 `t_ff` 字段下发（配合 kp=0, kd=小阻尼）。

发送建议：
```python
# 仅力矩（与仿真等效）
mit_frame(motor, position=0, velocity=0, kp=0, kd=0, torque=tau_ctrl)
# 加微小阻尼防自激（推荐实机使用）
mit_frame(motor, position=0, velocity=0, kp=0, kd=0.05, torque=tau_ctrl)
```

### 4. 饱和监控

实机控制器循环里应监控饱和率：

```python
sat_wheel = abs(torque_wheel) >= 0.95 * 1.0   # 6215 额定 1 N·m
sat_leg   = abs(torque_leg)   >= 0.95 * 3.5   # 4310 额定 3.5 N·m
```

仿真静站饱和率应 < 5%。若实机饱和率 > 20% 持续 > 1s，立即停机，可能是：
- 平衡 pitch 漂了（IMU 校准偏 / 实机重心与仿真不符）
- yaw 控制振荡
- 5-bar 几何与仿真不一致（机械装配误差）

### 5. 安全检查清单

每次上机前：
- [ ] `model.body_mass.sum() ≈ 2.20`（仿真）
- [ ] 实机 IMU 静置读数 pitch/roll < ±0.02 rad
- [ ] 4 个电机 enable 后立即 disable，确认通信正常
- [ ] 现场人员手持机器人，控制器 enable 后人力感知是否震动 / 嗡鸣异常
- [ ] 第一次驱动用 cmd_height=0.142、cmd_linear_x=0、cmd_angular_z=0（静站模式）
- [ ] 静站 10 s 通过，再尝试 cmd_linear_x=0.1 慢速前进
- [ ] cmd_height 不要超过 0.142；先验证 cmd_height=0.130 等较低高度稳定
- [ ] 全程急停可达

### 6. 失效模式应对

| 现象 | 可能原因 | 处理 |
|---|---|---|
| 静站时 wheel 持续转向某方向 | 实机自然平衡 pitch 与仿真几何不符 | 用 IMU 和几何脚本复测 CoM / wheel_mid，检查 `equilibrium_pitch_from_geometry()` 的输入 |
| 高频抖动（10 Hz+） | LQR 增益过大 / 控制延迟 | 降 q_diag[0]、增 r_diag、检查 CAN 链路时延 |
| 低频前后摇晃（1–3 Hz） | pitch_lean_gain 与 LQR 冲突 | 把 pitch_lean_gain 降到 0.05 |
| 转弯时机体侧倾不停 | yaw 差速符号错或 roll 控制弱 | 检查 yaw_damping、`LEG_ROLL_DIFF_SIGNS` |
| 升高度时震荡 | LUT 边界附近不稳 | 降低 cmd_height 上限 |

## 重要文件指向

- `sim/controllers/default_params.py` — `STAND_PARAMS` / `STAND_THEN_DRIVE_PARAMS` 最终值
- `sim/controllers/balance_lqr.py` — LQR A/B 解析模型，已按新质量自动适应
- `sim/controllers/vmc.py` — VMC 重力补偿用 `total_mass`，自动适应
- `sim/model_xml.py:435` — actuator ctrlrange（轮 ±1.0、腿 ±12.5；非 EXTEND 相位由控制器软限幅 ±3.5）
- `sim/model_xml.py:cmd_height` ctrlrange `[0.118, 0.142]`
- `AGENTS.md` — 电机参数、URDF 质量分配硬约束、已知遗留
- 本文档 — 调参流程 / 上机部署清单

## 未来工作

- 继续调 LAND 落地窗口，目标是把飞行后接触恢复窗口的 `max_abs_pitch_after_touchdown` 降到 `<0.1 rad`
- LQR gain scheduling 支持 cmd_height 全范围 [0.118, 0.148]
- 重新跑 Optuna 检查 `STAND_THEN_DRIVE_PARAMS` 是否还能提升 drive 距离和转弯稳定性
- 实机 IMU 校准流程文档化
