# TODO

跟踪当前未完成的工程问题。新条目放最上面；完成的条目划掉并保留一段时间作为索引。

来源：`docs/height_tracking_lut_redesign.md`（2026-05-13/14 重构记录）+ 用户口头补充。

---

## P0 — 核心阻塞

### [x] 1. cmd_height 下限偏高，机器人还能更矮

**完成状态**：runtime 已接到扩展 LUT，`launch_mujoco` 的 `cmd_height` slider 现在是 `[0.118, 0.148]`，初始值会设为 `STAND_PARAMS.vmc.nominal_height = 0.142`，避免打开 viewer 时 `data.ctrl=0` 被 clamp 到最低高度。

**现象**：旧 `cmd_height` ctrlrange = `[0.135, 0.148]`，机体最低 `pos_z ≈ 0.166`。但机械上腿还能继续收。

**根因**：限制不在硬件而在 probe 脚本。
- `scripts/probe_leg_geometry.py:34` 旧值硬编码 `THETA_MIN = 0.45`
- URDF (`sim/robot/robot.urdf:182,189`) 实际关节限位 `[0.2618, 1.6581]` rad，比旧 probe 下限还低约 0.19 rad
- 旧 LUT 实测 θ_min = 0.4380 rad，几乎就是旧 probe 下界的稳态值

**实测数据**：
- `scripts/probe_leg_geometry.py` 已改为 `THETA_MIN = 0.325`、`THETA_STEP = 0.005`、`SETTLE_STEPS = 3000`、`AVERAGE_WINDOW = 600`
- 新 LUT：θ range `[0.2656, 0.6027]`，h range `[0.1178, 0.1485]`，h span `0.0307m`，严格单调
- `scripts/probe_leg_full_range.py` 扫 URDF 主动关节全范围 `[0.2618, 1.6581]`：最低 h≈`0.1179` 出现在 θ≈`0.2673`（URDF 下限附近）；继续增大关节角不会更低，只会进入升高方向，最高 h≈`0.1505`

**验证**：
- `uv run python scripts/verify_height_sweep.py` → PASS：command range `[0.1188, 0.1475]`，leg_h range `[0.1206, 0.1501]`，pos_z_span `0.0302`，max_abs_pitch `0.1010`，fell=False
- `uv run pytest tests/test_leg_height_lut.py tests/test_vmc_geometry.py tests/test_balance_roll_6d.py -q` → 19 passed
- `build_controlled_model()` 生成模型中 `cmd_height ctrlrange=[0.118, 0.148]`、初始 `data.ctrl=0.142`

**后续风险**：
- `verify_cmd_height_tracking.py` 仍按旧的“绝对 pitch < 0.02 / 每段 drift < 5mm”严格指标失败；这属于 P0-2 的指标定义问题，不阻塞 launch 全范围高度调节。


---

### [ ] 2. `verify_cmd_height_tracking.py` 仍 FAIL（pitch 超阈值）

**最近一次输出**：
- `max_abs_pitch = 0.0362`（阈值 0.02）❌
- `max_base_xy_drift = 0.0561`
- `pos_z_span = 0.0133`、`pearson_r = 0.9996`、`fell = False`

**症结**：
- 几何量与原计划假设不一致：
  - `pitch_eq_from_height(0.142) = -0.0205 rad`（计划 `-0.17`）
  - `dy_wheel_dh(0.142) = 0.164`（计划 `+5`）
  - 实测量级太小 → 几何 wheel velocity 前馈对高度阶跃 pitch 峰值改善有限
- 低高度目标下稳态 pitch ≈ 0.03 rad，本身就和「任意阶跃 |pitch| < 0.02」绝对阈值冲突
- 第一段从初始高度切到最低高度时漂移最大，与「稳态后单步阶跃」混在一起评判

**建议先做的唯一一件事**（docs 原文）：
- [ ] 重新定义「高度阶跃期间」测量窗口
  - warmup 到当前 nominal 稳定 → 施加单个 `cmd_height` 阶跃 → 分别记录 step 前后 0.5 s 内的 `pitch_peak_delta` 和 `base_xy_delta`
  - 用 delta 而非绝对值，把「初始重定位」和「已稳态后高度阶跃」分开评分
- [ ] 指标确认后再决定是否动 LQR 权重 或 VMC 力矩分配

---

## P1 — 次级

### [ ] 3. Viewer 手测 cmd_height 平滑跟踪

本轮重构后还没真正打开 `launch_mujoco` 拖滑块验证 cmd_height 的连续跟踪是否平滑（无颤抖、无饱和段）。

### [ ] 4. Jump 场景未在 LUT 路径下实测

`JumpPhase.CROUCH/EXTEND` 现在通过 `crouch_offset` / `extend_offset` 加到 `target_height` 再经 LUT clamp 转 motor 角，但 jump 场景从未在新路径下跑过。

### [ ] 5. `STAND_THEN_DRIVE_PARAMS` 在新模型下不可用

来自 `docs/lessons_learned.md`：该参数集由 Optuna 在错误的 `WHEEL_FORWARD_SIGNS=+1` 下优化得到，新物理模型下会弹飞。drive 场景目前临时回退到 `STAND_PARAMS`，需要重跑 Optuna 优化。

---

## P2 — 文档/记录

### [ ] 6. 把本次根因回写到 `docs/lessons_learned.md` 主索引

需要追加的两条核心教训：
1. **任务空间 VMC 在 5-bar 奇异处放大力矩** → 雅可比奇异 × ±5 Nm ctrlrange × 单腿 3.46 Nm 重力支撑 → 高档翻倒。解决方案是 joint-space PID + 离线 h↔θ LUT。
2. **`pitch_trim` 与模型脱钩的陷阱**：早期 `STAND_PARAMS.pitch_trim = -0.07` 来自旧 CoM 偏置；commit `c3bd654` 把 CoM 对齐到轮心之后没人更新参数，启动瞬间 `0 − (−0.07) = 0.07` 直接把 wheel ctrl 推到 ±1 饱和，稳态 `vel_y ≈ −0.15 m/s` 单向漂移。

### [ ] 7. 模型/控制器关键常量与文档对齐

- 模型实测总质量 **10.24 kg**（早期文档写的 7.7 kg，已过期）
- 单腿支撑重力所需力矩 **~3.46 N·m**，ctrlrange ±5 余量 ~30 %
- **`project_memory.md` 关于 ctrlrange 的描述已过期**：文档写轮 `[-3.0, 3.0]` / 腿 `[-3.5, 3.5]`，但 `sim/model_xml.py:331` 的真值是轮 `[-1.0, 1.0]` / 腿 `[-5.0, 5.0]`。需要修文档（代码是真值，刚核实过）。

---

## 完成

（暂无；记录格式：`- [x] YYYY-MM-DD 简述`）
