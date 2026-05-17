# AGENTS.md

Behavioral guidelines to reduce common LLM coding mistakes. Merge with project-specific instructions as needed.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:

- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:

- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:

- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:

- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:

```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.

## AI Agent Workflow Requirements (Grok Search)

- **Search Before Execution**: Before solving problems, writing code, or designing solutions—especially when facing uncertain, complex, or experience-dependent tasks—you **must prioritize calling Grok Search MCP related Skills** (e.g., `grok-web-search`, `grok-web-research`) to retrieve information.
- **Avoid Reinventing the Wheel**: Always try to search the web first for existing solutions, official documentation best practices, or proven viable approaches. Use the experience of others to guide and support your ideas.
- **Solve Problems Efficiently**: Formulate execution plans based on authoritative and accurate information gathered from searches to complete tasks in the most efficient and direct way possible.

## 控制算法要求

- 轮腿机器人控制必须使用 **LQR + VMC** 架构。
- 轮子输出必须由 LQR 决定。不要用普通 PD、启发式速度控制、直接轮速控制替代 LQR。
- 腿部高度控制必须由 VMC 决定。高度需要固定时，应通过 VMC 控制目标高度实现。
- 当前 LQR 调参阶段，允许临时使用仿真外力支撑 root z 高度来隔离 VMC 干扰；该模式必须可关闭，且不得改写 `data.qpos` 或 `data.qvel`。
- 不要物理锁死任何自由度。禁止在控制循环中直接改写 `data.qpos`、`data.qvel` 来固定高度、姿态、yaw 或平面位置。
- 可以调整 LQR/VMC 参数、目标量、状态定义、权重矩阵、线性化点和控制分配，但不要绕开 LQR/VMC。

## 控制目标

- 当前阶段目标：通过 MuJoCo Control 面板滑块控制机器人平面移动，同时高度保持固定。
- 高度固定只是为了方便调整 LQR，不代表允许改写 MuJoCo 状态；临时支撑应使用外力/阻尼，不直接写 `qpos/qvel`。
- `cmd_linear_x` 控制线速度。
- `cmd_angular_z` 控制 yaw 角速度。
- 两轮机器人 yaw 控制应使用左右轮差速实现，允许原地旋转。

## 坐标系与符号

- 当前受控模型使用 `link23_connected_stand` keyframe 作为启动姿态。
- 根关节是 MuJoCo freejoint：
  - `qpos[0:3]` 是 root 位置。
  - `qpos[3:7]` 是 root 四元数。
  - `qvel[0:3]` 是 root 线速度。
  - `qvel[3:6]` 是 root 角速度。
- 当前平面前进距离在 rollout 中按 root `x` 方向计算。
- `cmd_linear_x` 的单位是 m/s。
- `cmd_angular_z` 的单位是 rad/s。
- 轮子关节角速度单位是 rad/s。
- 轮半径当前按 `0.036 m` 使用；轮速和线速度换算必须显式处理单位。
- 不要把 m/s 目标直接写成 rad/s 轮速目标，除非除以轮半径完成换算。

## 滑块与执行器

- MuJoCo actuator 顺序中，真实执行器和命令滑块不同：
  - `cmd_linear_x` 是命令输入，不是物理轮子执行器。
  - `cmd_angular_z` 是命令输入，不是物理轮子执行器。
- 读取滑块必须按 actuator 名称读取，不要依赖 UI 显示顺序。
- 控制器写 `data.ctrl` 时，不要把命令滑块值误当作物理执行器输出。
- `cmd_linear_x` 不应影响 yaw 差速项。
- `cmd_angular_z` 不应产生平均前进力矩；它应产生左右轮差速。

## LQR/VMC 调试原则

- 如果零速度下机器人移动，优先检查：
  - LQR 目标状态是否和当前状态一致。
  - LQR 零目标输出是否有偏置。
  - 轮速目标 m/s 到 rad/s 的单位换算。
  - `WHEEL_FORWARD_SIGNS` 是否和实际平面运动方向一致。
  - yaw 差速项是否在物理轮力矩域中相反，而不是变成同向前进力矩。
- 如果高度不稳，优先调整 VMC：
  - `nominal_height`
  - `kp_height`
  - `kd_height`
  - 腿部力矩方向符号
  - 输出限幅
- 不要通过改写 MuJoCo 状态掩盖控制器问题。

## 验证要求

- 修改后按实际改动运行最小相关验证，不要求每次修改都跑完整测试。
- 只有用户明确要求“仿真完整测试”“完整测试”“完整回归”时，才运行完整仿真/完整回归测试。
- 控制器小改优先运行覆盖改动点的 focused tests，例如：
  - `uv run pytest tests/test_balance_state.py tests/test_balance_lqr.py tests/test_combined_controller.py -q`
- 对滑块映射必须验证：
  - `cmd_linear_x` 改变线速度目标。
  - `cmd_angular_z` 改变 yaw 角速度目标。
  - `cmd_angular_z` 的轮子输出是差速，不是同向前进。

## 调参流程

- 调试 LQR、VMC 或诊断严重不稳定时，优先用自动化脚本获取数据。
- 只有用户明确要求完整仿真验证时，才生成并检查完整 `.png` 曲线图和 `.mp4` 视频。
- 普通小改可以先用短时无头 rollout、首步控制输出、focused pytest 验证。

## 架构与模型防坑指南 (AI 防误判必读)

为了防止 AI 在重构或修改代码时引入致命错误，请严格遵守以下隐藏约束：

1. **闭环连杆约束 (五杆机构)**：
   - 机器人的腿部包含五杆机构闭环结构。在 URDF 生成并预处理后的 MJCF 中，这是通过 `<equality>` 标签将 `link2` 和 `link3` 的 `connect_site` 连接起来实现的。
   - **绝对禁止**在修改 XML 或 URDF 时破坏或删除这些 `<equality>` 约束，否则机器人的腿会在物理仿真中直接解体。
2. **四元数与 Pitch 计算约定**：
   - MuJoCo 的四元数顺序为 `[w, x, y, z]`。
   - 代码中的 `pitch` 状态有专门的解析逻辑（见 `sim/state.py` 中的 `_pitch_from_quaternion`），它是基于局部 Y 轴计算的倾角。
   - **禁止**盲目引入 `scipy.spatial.transform` 等外部库的欧拉角转换函数，这极易引入旋转顺序（如 xyz 与 zyx）不一致或万向节死锁问题。
3. **避免位置环“橡皮筋”效应**：
   - `CombinedController` 在计算 LQR 时，会每步动态更新目标轮子位置（`target[2]`），以实现平滑的速度跟踪。
   - **禁止**将 LQR 的目标轮子位置硬编码重置为静态 `0.0`，否则机器人在移动后会受到巨大的反向“橡皮筋”拉力。
4. **积分抗饱和 (Anti-Windup)**：
   - 速度前馈 (`_velocity_integral`) 和 Yaw 修正 (`_yaw_integral`) 的 PI 控制器都实现了积分抗饱和（条件累加和 `np.clip` 限幅）。
   - **禁止**在重构控制逻辑时删除这些抗饱和与限幅逻辑，否则会导致极其剧烈的震荡和失控。
5. **禁止硬编码索引**：
   - 由于加入了 `cmd_` 命令滑块，MuJoCo 的 actuator 索引经常变动。
   - **绝对禁止**使用 `data.ctrl[0]` 或 `data.qpos[7]` 这样的魔法数字硬编码。
   - 所有的关节和执行器必须通过 `sim/model_semantics.py` 中的 `MODEL_SEMANTICS` 和 `sim/state.py` 中的 `model_addresses()` 动态获取索引。
6. **模型预处理管线 (预处理后才是真理)**：
   - 当前机器人仿真源文件是 `sim/robot/robot.urdf`；项目不使用手写 XML 文件作为仿真入口。
   - 真实的物理仿真模型是在 `launch_mujoco.py` 运行时，由 `sim/model_xml.py` 中的 `prepare_controlled_mujoco_xml` 从 URDF 转换并动态生成 MJCF。
   - 该 Python 脚本会执行许多关键“魔改”：比如将所有 mesh 设为纯视觉（visual-only）、强行注入 `sphere` 碰撞体作为轮子（比 cylinder 更利于 LQR 平滑控制）、强行注入 `root` freejoint、以及注入命令滑块等。
   - **禁止**仅凭阅读历史 `.xml` 文件就得出系统结构结论，更不要盲目添加复杂的碰撞网格。任何针对仿真物理环境的判断，必须以 `sim/model_xml.py` 从 URDF 生成并处理后的动态 MJCF 模型为准。
7. **代码注释只是参考，必须用脚本核实**：
   - 仓库经历了多次模型重构（鸟腿 0.47m → 当前五连杆 0.18m 高度），URDF、keyframe、CoM、平衡 pitch、摆杆长度等几何量都已多次变更。
   - 代码与文档中（包括 `model_xml.py`、`default_params.py`、`docs/lessons_learned.md`、本文件）出现的所有具体数值——CoM 偏移、平衡 pitch、摆杆长度、轮距、合理 `pitch_trim` 范围等——都可能是**前一版模型的遗留**，与当前模型不一致。
   - **禁止**在调参或决策时直接引用注释中的数字。任何涉及几何量的判断，必须临时写脚本用 `mj_resetDataKeyframe` + `mj_forward` 重新计算（参考 CoM 偏移测量套路：`(masses[:,None]*data.xipos).sum(0)/total_mass` 减去 `0.5*(wheel_l.xipos + wheel_r.xipos)`）。
   - 发现注释与实测不符时，**先订正注释**再继续工作，避免错误数字继续被下一个 agent 引用。

## 实机硬件 CAN 配置与调试规则

### 电机型号与参数（硬约束，不得擅自修改）

实机使用以下两款达妙电机，参数取自 `control/docs/手册/` 下的官方说明书。**未来 agent 在调整仿真 ctrlrange、URDF 质量、控制器扭矩上限等任何与电机能力相关的数值前，必须先核对本章节，不得依据旧注释、旧 commit、旧文档里的数字行动**。

#### DM-J4310-2EC V1.2（关节电机，24V 版）

| 项 | 值 |
|---|---|
| 额定扭矩 | **3.5 N·m**（仿真 ctrlrange 取此值） |
| 峰值扭矩 | 12.5 N·m（仅允许短时） |
| 额定转速 | 120 rpm |
| 空载最大转速 | 200 rpm（≈ 21 rad/s） |
| 减速比 | 10 : 1 |
| 极对数 | 14 |
| 重量 | **0.325 kg/个**（含一体化驱动器） |
| 编码器 | 双 16 位磁编码器 |
| 控制接口 | CAN @ 1 Mbps |
| 默认控制模式 | MIT |
| 安装位置 | **base_link**（电机定子固定 base，转轴驱动 link1） |

#### DM-H6215（轮毂电机）

| 项 | 值 |
|---|---|
| 额定扭矩 | **1.0 N·m**（仿真 ctrlrange 取此值） |
| 峰值扭矩 | 2.0 N·m（仅允许短时） |
| 额定转速 | 120 rpm |
| 空载最大转速 | 320 rpm（≈ 33.5 rad/s） |
| 减速比 | 1 : 1（直驱） |
| 极对数 | 14 |
| 相电感 / 相电阻 | 2000 µH / 2.5 Ω |
| 重量 | **0.360 kg/个**（电机本体，不含外加轮辋/橡胶） |
| 外径 × 高度 | 68 mm × 44.5 mm |
| 编码器 | 霍尔 |
| 控制接口 | CAN @ 1 Mbps |
| 默认控制模式 | 速度 |
| 安装位置 | **wheel_left / wheel_right**（轮毂内嵌） |

#### 仿真 ctrlrange 规则

- `sim/model_xml.py:_replace_actuators` 中：轮子 `ctrlrange = "-1.0 1.0"`，腿 `ctrlrange = "-3.5 3.5"`。
- **数值必须取额定扭矩，不是峰值**。原因：仿真控制器找到的解会贴着 ctrlrange 上限运行；若仿真用峰值，上机会瞬间过载或过热。
- 任何提案"放大 ctrlrange 以让控制器更易收敛"的修改，必须先通过现场验证与短时峰值限制（duty cycle）的实测支撑；不得擅自把这两个值改大。

#### 常见误读警告

- **6215 的 `T_MAX` 寄存器 ≠ 物理扭矩极限**。`T_MAX` 是 MIT 模式 12-bit 编码的浮点-定点映射范围（例如配置成 10 表示编码刻度，不代表能输出 10 N·m）。6215 的物理扭矩上限就是上表的额定 1 N·m / 峰值 2 N·m。早期 docs 出现的 "DMH6215 T_MAX=10Nm" 是误读，请勿延续。

### URDF 质量分配（硬约束，2026-05-16 校准）

整车实机称重 **2.20 kg**。URDF 中各 link 质量经 mass-balance 推导后固定如下；**任何修改必须同步更新本章节，并解释为何偏离实机称重**。

| Link | mass (kg) | 说明 |
|---|---|---|
| `base_link` | 0.94 | 含 2× DM4310 (0.65) + 电池 + PCB + 主车架 (≈ 0.29) |
| `wheel_left` / `wheel_right` | 0.50（× 2） | 含 1× DMH6215 (0.36) + 轮辋 + 橡胶 (≈ 0.14) |
| `link1_left` / `link1_right` | 0.04（× 2） | 被动连杆 + 法兰，无电机 |
| `link2_left` / `link2_right` | 0.08（× 2） | 被动连杆，无电机 |
| `link3_left` / `link3_right` | 0.01（× 2） | 短被动耦合杆 |
| **TOTAL** | **2.20** | 与实机称重一致 |

- 几何尺寸（`origin xyz`、mesh）保持不变；每个 link 的 inertia 张量按 `I_new = I_old × (m_new / m_old)` 比例缩放。这是恒密度几何不变时的精确缩放，不破坏 inertia 的物理对称性。
- 在仿真里用 `model.body_mass.sum()` 应严格等于 2.20 kg（4 位精度）；偏离即说明 URDF 被改过且未更新本表。
- 如果未来更换电机或改装（如加二自由度云台、换电池），请先：① 实机称重 → ② 更新本表 → ③ 同步重算 inertia → ④ 重新调 LQR/VMC 参数。

#### 已知遗留（2026-05-16 校准后未完成项）

- ~~**LQR/VMC 参数尚未在新质量下重调**~~ ✅ **2026-05-16 已完成**：`STAND_PARAMS` 和 `STAND_THEN_DRIVE_PARAMS` 已重调。**详细流程、调参经验、实机部署清单见 `docs/retuning_guide_2026-05-16.md`**。关键变化：
  - VMC `kp_motor 50→30, kd_motor 4→1.0`（按质量比例缩小，旧值在 0.74 N·m 单腿支撑下过激）
  - LQR `q_diag[0,2] (pitch/roll) 5000→1000`：允许 ±0.03 rad 自然倾斜以吸收 CoM 偏移
  - LQR `r_diag 200×`：补偿 B 矩阵 ~7× 增益（B ∝ 1/(M·L)）防止饱和
  - `pitch_trim 0→-0.02`：CoM y 偏移 / 摆杆长度 ≈ 自然平衡 pitch
  - Headless 验证：stand 10s、drive 0.3/0.5 m/s、yaw 0.5 rad/s、cmd_height 0.135-0.148 全部稳定，饱和率 < 5%
- **跳跃能力下降未调**：`jump_extend_feedforward=120 N·m` 被新 ctrlrange ±3.5 clip 严重，实测跳跃高度从旧 ~0.15 m 降到 ~0.045 m。若需恢复跳跃高度，参见 `docs/retuning_guide_2026-05-16.md` 的「跳跃高度下降」章节。`test_cmd_jump_rollout_reaches_airborne_contact_state` 阈值已下调至 0.03 m 以匹配现状。
- **cmd_height 上限收窄**：从 0.148 → 0.142。h>0.142 时 LUT `d²h/dθ²` 符号翻转 + LQR 线性化漂移导致自激振荡。若需恢复 0.148 全行程，需 LQR gain scheduling 或 LUT 边界平滑。

#### 不要做的事

- 不要把 wheel mass 调回 CAD 原始的 0.955 kg（那是 SolidWorks 默认密度下的虚假值，与实机不符）。
- 不要把 base_link 推回 6.7 kg（同理）。
- 不要在 `_replace_actuators` 里把腿的 ctrlrange 放回 ±320（旧的占位无意义值），也不要写回 ±3.0 或 ±5.0。
- 不要在 docs/代码注释里写 "DMH6215 T_MAX=10Nm" 这种把 CAN 映射范围当成物理极限的表述。

### 电机与 IMU CAN 配置

实际硬件已按此配置完成：PCB 通过 CAN 连接 4 个电机和 DM-IMU。

| 设备       | CAN_ID | Master/MST_ID | 控制模式     |
| ---------- | ------ | ------------- | ------------ |
| DM-IMU     | 0x01   | 0x11          | -            |
| DM4310 左  | 0x02   | 0x12          | MIT 控制模式 |
| DM4310 右  | 0x03   | 0x13          | MIT 控制模式 |
| DMH6215 左 | 0x04   | 0x14          | MIT 控制模式 |
| DMH6215 右 | 0x05   | 0x15          | MIT 控制模式 |

> **参数参考**：详细电机规格见上方「电机型号与参数」章节；硬件手册原文位于 `control/docs/手册/`。

### 达妙电机与 CAN 调试规则

未来 coding agent 在本项目内编写、修改或运行达妙电机相关代码时，必须遵守以下规则。

#### 默认通信配置

- 默认使用 CAN 标准 11-bit 数据帧和数据帧类型，不使用扩展帧或远程帧，除非硬件资料和现场人员明确要求。
- 实机通信默认使用达妙 USB 转 CAN 虚拟串口二进制协议，CLI backend 为 `dm_usbcan`。
- 电机默认 CAN bitrate 为 `1 Mbps`。达妙 USB-CAN 虚拟串口默认 `serial baud` 为 `921600`。项目代码不自动修改设备 CAN bitrate。
- 使用项目 CLI 时优先显式写出通信参数：`--backend dm_usbcan --port <port> --serial-baud 921600 --can-bitrate 1000000`。

#### 上电、接线、调试顺序

1. 先确认机械固定、轮子离地或有可靠约束，并确认急停、断电、拔 CAN/电源的路径可立即执行。
2. 上电前确认线序：`CAN_H` 对 `CAN_H`、`CAN_L` 对 `CAN_L`，电源正负极正确；不要在接线不确定时发送任何实机命令。
3. 先离线验证，再连接硬件：`uv run pytest`、`uv run python -m compileall control tests`。
4. 串口发现：`uv run python -m control.can_tool list-ports`。
5. 只读 sniff：`uv run python -m control.can_tool sniff --backend dm_usbcan --port <port> --serial-baud 921600 --can-bitrate 1000000 --duration 1.0`。
6. 只读 IMU：`uv run python -m control.can_tool imu read --backend dm_usbcan --port <port> --serial-baud 921600 --can-bitrate 1000000 --duration 1.0`。
7. 单电机短暂 enable 后必须立即 disable：`uv run python -m control.can_tool motor enable --backend dm_usbcan --port <port> --serial-baud 921600 --can-bitrate 1000000 --motor dm4310_left --duration 0.1`。
8. 现场确认 DMH6215 当前实际为 MIT 模式（2026-05-16 已切换，之前为速度模式）；4 台电机全部走 MIT 帧 `arbitration_id = CAN_ID`，零输出用 `mit_frame(motor)` 全 0 参数。
9. 最后才允许保守 group 或 pair 测试：`uv run python -m control.can_tool pair conservative-spin --backend dm_usbcan --port <port> --serial-baud 921600 --can-bitrate 1000000 --duration 0.1 --period 0.05 --velocity 0.2 --wheel-velocity 0.2`，以及 `uv run python -m control.can_tool group zero --backend dm_usbcan --port <port> --serial-baud 921600 --can-bitrate 1000000`、`uv run python -m control.can_tool group conservative-test --backend dm_usbcan --port <port> --serial-baud 921600 --can-bitrate 1000000`。

#### 只读与冒烟测试优先

- 不确定电机状态、接线、模式、ID、Master/MST_ID、供电或机械约束时，只允许执行离线测试、串口枚举、sniff、IMU 只读、短暂 enable/disable 冒烟测试。
- 不要在不确定状态下发送参数写入、参数保存、保存零点/设零点、校准、标定、固件升级、修改 CAN_ID、修改 Master/MST_ID、修改 CAN 波特率等命令。
- 达妙电机出厂通常已完成校准和标定；校准、标定、保存零点、固件升级都会带来运动或持久化风险，只能在明确任务和现场确认后执行。

#### CAN 帧规则

- MIT 模式控制帧：`arbitration_id = CAN_ID`，8 字节按位打包 `p/v/kp/kd/t`。
- 速度模式控制帧：`arbitration_id = 0x200 + CAN_ID`，`float32 little-endian` 速度值放在 D0-D3，D4-D7 为 0。
- 力矩模式控制帧：本项目代码使用 `arbitration_id = 0x600 + CAN_ID`，`float32 little-endian` 力矩值放在 D0-D3，D4-D7 为 0。
- `enable`、`disable`、`zero/save-zero-position`、`clear-error` 都发到 `CAN_ID`，数据分别为 `FF FF FF FF FF FF FF FC/FD/FE/FB`。
- 参数读写/保存统一通过 `arbitration_id = 0x7FF`：D0/D1 为 `CAN_ID` little-endian，D2 为 `0x33` 读、`0x55` 写、`0xAA` 保存，D3 为 `RID`，D4-D7 为数据 little-endian；读命令 D4-D7 置 0。

#### 反馈帧解析规则

- 电机反馈帧的 `arbitration_id` 是 Master/MST_ID，不是 CAN_ID。本项目按 `master_id` 路由反馈帧，并校验反馈中的 `motor_id`。
- D0 低 4 位是 `CAN_ID` 低位，高 4 位是状态/错误码：`0` 失能，`1` 使能，`8` 超压，`9` 欠压，`A` 过电流，`B` MOS 过温，`C` 线圈过温，`D` 通讯丢失，`E` 过载。
- D1-D2 为 position，D3-D4 为 velocity，D4-D5 为 torque，D6 为 MOS 温度，D7 为 rotor 温度；position 使用 16 位映射，velocity/torque 使用 12 位映射，映射范围必须与电机配置的 `P_MAX/V_MAX/T_MAX` 一致。

#### 参数与模式规则

- 写寄存器会立即生效，但不保存则掉电丢失。
- 保存参数只能在失能状态下生效，会一次性写入当前全部参数到 flash；不要频繁执行保存参数命令。
- 控制模式寄存器 `RID=0x0A`：`1=MIT`、`2=位置速度`、`3=速度`、`4=力位混控`。通过 CAN 切换模式会清零位置、速度、MIT 扭矩前馈、`kp`、`kd` 等命令值；未保存则掉电丢失。
- 切换到位置类控制前，应尽量在零速状态下进行，并确认当前位置，避免位置突变造成冲击。

## 文件与目录规范

- **测试文件**：所有的测试代码和文件必须统一写入 `tests/` 目录下。
- **临时文件**：所有的临时文件必须写入 `tmp/` 目录下。并且，一旦使用完毕或不再需要，必须在当前会话结束前主动清理/删除这些临时文件，以保持工作区整洁。

## MuJoCo 仿真性能与 mesh 抽稀经验

### 症状
`uv run mjpython sim/launch_mujoco.py` 在 macOS / Apple Silicon 上帧率低、卡顿，但物理本身不慢。

### 判断瓶颈：先跑 headless profile
不要直接猜，先用 `uv run python profile_sim.py` 验证物理速度。本项目实测 headless 跑 ~10x realtime —— 这说明 `mj_step`/控制器都不是瓶颈，**问题在渲染**，再去动其它东西都是错方向。

### 为什么 macOS 上 visual mesh 面数会影响这么大
不是 M 芯片不够强，是几层开销叠加：
1. **macOS OpenGL 是兼容翻译层**：Apple 2018 弃用 OpenGL 封顶 4.1，M 芯片上走 OpenGL → Metal 翻译，per-draw-call CPU 开销远高于 Linux/Windows。
2. **小三角形 GPU "quad 浪费"**：GPU 光栅化按 2×2 像素 quad 为单位，一个亚像素三角形仍跑完整 4 个 fragment shader（3 个被丢弃）。CAD 导出的高面 mesh 大量三角形在 viewer 视角下小于一像素，**外观看不出差别但 GPU 在大量做无效工作**。
3. **主线程渲染 + viewer.lock() 抢锁**：mjpython 强制 viewer 主线程渲染，worker 线程跑物理，两边共享 `viewer.lock()`。`launch_mujoco.py` 的循环是无 sleep 紧凑 while，重 mesh 拖慢主线程会饥饿 worker → catch-up 累积 → 卡顿。

### 检查 mesh 面数
```python
import mujoco; from sim.model_xml import prepare_controlled_mujoco_xml
m = mujoco.MjModel.from_xml_path(str(prepare_controlled_mujoco_xml(Path('sim/robot/robot.urdf'))))
for i in range(m.nmesh):
    print(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, i), m.mesh_vertnum[i], m.mesh_facenum[i])
```
经验阈值：单个 visual mesh 超过 30k 面就值得抽稀，超过 100k 几乎肯定要抽稀。本项目原始 `base_link.stl` 是 190k 面，是唯一的大头。

### 抽稀工具选择（踩过的坑）
- **不要用 `fast-simplification`（pyfqmr）做 CAD mesh 抽稀**：默认 aggressiveness 高时，圆孔/圆柱/倒角会被切平变多边形（quadric error 在曲面上分布太均匀，圆形特征被优先吃掉）；降低 `agg=3` 又过于保守只能减 40%。对于"既有平面又有曲面"的 CAD 模型不合适。
- **首选 `pymeshlab`（MeshLab 的 QEC filter）**：开关够多，能在保圆角的同时大幅压平面区域。
- **不要用 Blender CLI**：本项目尝试过的 quadric 路径里它最慢、质量也不如 pymeshlab。

### 抽稀脚本与正确参数
脚本：`scripts/decimate_base_link.py`，**始终从 `base_link.stl.orig` 重读再写到 `base_link.stl`**，方便反复迭代不累积失真。第一次跑前先 `cp base_link.stl base_link.stl.orig` 保存原始。

PyMeshLab `meshing_decimation_quadric_edge_collapse` 的关键开关（缺一影响很大）：
- `preserveboundary=True` + `boundaryweight=2.0`：双倍权重保护孔洞/外轮廓边界，圆孔不被切平。
- `preservenormal=True`：保护法线方向，防止圆柱面凹凸翻折。
- `planarquadric=True`：平面用平面误差度量，**优先在大平板上抽稀而不是在圆角上**。这是 fast-simplification 没有的能力。
- `optimalplacement=True`：合并后新顶点放在最小误差位置（不只是端点之一）。
- `qualitythr=1.0`：拒绝生成狭长三角形。
- 注意 pymeshlab 2025.7 的 filter 名是 `meshing_decimation_quadric_edge_collapse`（不带 `_decimation` 后缀），老文档里的 `..._decimation` 名字会报"Filter does not exists"。

本项目实测 190k → 30k 面（-84%，9.1 MB → 1.5 MB），圆孔/圆柱外观保住，viewer 流畅明显改善。

### 其它已知开销（次要，必要时再动）
- 方向光 `castshadow="true"` 会让 shadow map pass 把所有 mesh 再渲染一遍。一般保留即可，确认 mesh 已抽稀后还是卡再考虑关。
- `launch_mujoco.py` 的 viewer 循环无 sleep 紧凑 while，可在循环末尾加 `time.sleep(max(0, next_frame - perf_counter()))` 节流到 60Hz 缓解锁竞争。

### 不要做的事
- 不要为了"提速"把 mesh 面数限制 (`prepare_mujoco_xml` 的 `face_limit`) 调低 —— 那个机制只是把单个 mesh 切成多块以绕过 MuJoCo 单 mesh 上限，**不减少总三角形数**，对渲染开销没帮助。
- 不要去动碰撞 mesh —— 本项目 `sim/model_xml.py` 已经用 box/capsule/sphere proxy 替代 mesh 碰撞，渲染慢和碰撞无关。

## 控制器性能陷阱：phase 触发的 LQR 增益冷缓存

### 症状
跳跃落地 (`JumpPhase.LAND`) 期间 viewer 严重卡顿，但站立、行驶时完全流畅。`mj_step` 本身始终在 µs 级，**瓶颈不在物理引擎**。

### 根因（已修复，2026-05-17）
`CombinedController._ensure_lqr_height_bin` 每步用 `(height_bin, roll_bin)` 作为 key 查 `_lqr_gain_cache`，cache miss 触发 `compute_balance_lqr_gain`：
- 3× `mj_forward` 全模型 kinematics 重算
- `solve_discrete_lqr` 跑 500 次迭代的 Python value iteration

bin 粒度 `lqr_height_bin_size=0.02 m`、`roll_bin_size=0.02 rad`。**落地瞬间身体 z 跌 5-10 cm、roll 弹动 5-10°，每 1-2 步跨一个新桶**，单步 controller 耗时从 0.22 ms 飙到 7.26 ms（×30 倍），166 步 land × 7.26 ms = 1.2 s 墙钟卡顿。

### 正确修复
**只需把 `solve_discrete_lqr` 改用 `scipy.linalg.solve_discrete_are`**（Schur 分解一次出解），单次重算从 ~14 ms 跌到 < 0.5 ms。166 次 land cache-miss 总成本变成 ~80 ms 摊到 332 ms 落地窗口，每步多 0.05 ms，完全无感。

### 反面教训：不要跳过落地的 LQR 增益重算
**走过的弯路**：曾经在 FLIGHT / LAND / FAIL 阶段跳过 `_ensure_lqr_height_bin`，假设「接触瞬态脱离倒立摆模型，VMC 主导腿，LQR 增益微调没意义」。**这是错的**。已实测：跳过 LAND 重算后，机器人复用 EXTEND 末态高度（~0.15 m）的增益，落地时实际 pendulum_length 跌到 0.06-0.09 m，倒立摆失稳项 `g/L` 翻倍，**LQR 反馈不足以接住后仰动量**，落地末态留下 -0.17 rad 后仰 → recover 没扳回来 → stand 阶段 pitch 一路漂到 -1.0 rad 触发 FALLEN（注：自起立机制已在 2026-05-17 移除，FALLEN 现为终态零控）。

**结论**：LQR 增益依赖摆杆长度的物理参数，**落地阶段的增益必须按真实低高度重算，否则机器人会倒**。性能优化时不要省「物理上必须做的事」。

### Phase Machine 与倒地处理（2026-05-17 当前状态）
- JumpPhase 现有 7 个：STAND / CROUCH / EXTEND / FLIGHT / LAND / RECOVER / FALLEN。已删除 SELF_RIGHT、FAIL、FallRecoveryParams 及全部自起立逻辑。
- FALLEN 是**终态**：pitch 超过 `pitch_fallen_threshold` (1.0 rad) 即进入，进入后所有控制器输出零（`combined.py:__call__` 和 `vmc.py` 都早返回 zeros），机器人原地不动，需重启仿真才能复位。
- **不要**重新引入自起立 / `SELF_RIGHT` / `recovery_attempts`：实测过 3 次自起立全失败、机器人在三次循环间反复滑动，比一动不动更糟。如果未来需要恢复跌倒后行为，先证明自起立动作能实际把机器人扳直，再加状态机。
- FALLEN 只看 pitch，不看 roll：本项目机器人在仿真里的实际倒地都是前后扑（pitch ±1.5 rad 量级），侧倒（roll 大）不出现。未来如机器人换形态或环境，可能需要 roll 也加入阈值。

### 隐藏陷阱：iterative Riccati 一直没收敛，整套 Q/R 调参是基于伪 LQR
**2026-05-17 实测**：旧 `solve_discrete_lqr` 的 500 步 value iteration 在本项目所有合理 L (0.06-0.15 m) 下**全部跑满 500 步未达 1e-10 tolerance**，与 scipy DARE 真解相对误差 ~6.6%。最致命的是 `K[0,4]`（轮位 → 轮扭矩）项：
- 迭代版 ≈ -0.08（位置反馈被「衰减」掉）
- DARE 真解 ≈ -0.45（**5.7 倍**强）

这意味着**项目里所有 Q/R 调参都是建在「位置反馈被严重弱化的非收敛 LQR」之上的伪调参**。换 scipy DARE 等于把这个隐藏 bug 拿掉，控制律会突然变「严苛」，**起跳累积的小轮位漂移会被强位置反馈翻译成 ~3.5 N·m 轮扭矩瞬态，反作用把机器人前栽到地上**。

### 应对方式（2026-05-17 已采用）
保留 scipy DARE，**在 LAND/RECOVER 阶段把 `target[4] = current_wheel_pos`**（`combined.py:_update_lqr_target` 里 `reset_wheel_pos_target` 集合扩到 FLIGHT/LAND/RECOVER）。物理直觉：落地瞬态不追究你跑了多远，先把姿态接住再说。Q/R 不动，stand 模式下位置反馈仍正常工作。

### 类似陷阱的通用警示
- **任何手写迭代数值算法，先验证它是否收敛**。`solve_discrete_lqr` 这种 200 行的库函数感觉简单，但本项目这版在常见参数下 500 步都不收敛，长期被当成「正确 LQR」使用。
- **替换数值库时务必做 A/B 对比**：DARE vs iterative、`np.linalg.solve` vs `np.linalg.lstsq` 等切换可能在「看起来都对」的情况下产生 ×5 增益差。任何「换更准的库函数」前，先打印两边输出对比一次。
- **基于伪 LQR 调出来的参数不能直接搬到真 LQR**。若未来要重新调 Q/R，先确认底层 solver 是收敛的。

### 通用教训（防止下次类型）
- **profile 之前不要乱猜**：先用 `[step-profile]` 类型的探针把 controller / mj_step / log / 姿态各段分开测，确认哪一段是真瓶颈。本次先验是「mujoco 渲染慢」「mac 没 Metal」，profile 一打全错。
- **任何「状态触发的 cache + 现场重算」组合都要查 phase 触发的雪崩冷缓存**：bin 粒度 × 状态变化速率 = 单位时间 cache miss 数。当变化速率（如落地时 dz/dt）远大于设计预期（如站立时 dz/dt≈0），同一段代码可以从「永远 cache 命中、0 开销」变成「每步重算、O(秒) 开销」。同类候选：力学参数 LUT、IK 解、轨迹规划缓存。
- **Python 循环里跑 LinAlg 是性能死刑**：`solve_discrete_are`、`solve`、`eig` 这些 scipy/numpy 单次调用都走 BLAS / LAPACK 一次出解，几个数量级快于 Python 循环 + numpy matmul。不要手写迭代 Riccati。
- **优先优化「计算密度」而不是「跳过计算」**：当一段计算的物理意义重要（如基于摆杆长度的 LQR 增益）时，宁可换更快的算法（scipy → BLAS），也不要直接跳过。跳过会留下「机器人摔倒」这类难调试的下游 bug。
- **性能优化必须用同样苛刻的功能测试验证**：本次「跳过 LAND 重算」初看测试全过 (`tests/test_balance_roll_6d.py` 是静态稳定性测试，不覆盖跳跃落地)，但实测跳跃时机器人倒地不起。**没有交互/集成测试覆盖的优化都是隐患**。
- **macOS / Apple Silicon 上没有 Metal 加速的 MuJoCo**。MJX 是为批量 RL 训练设计的 JAX 重写，对单机器人交互 viewer **无加速反而更折腾**。不要去找「Metal 版 MuJoCo」，问题大概率在控制器或渲染层而不是物理引擎。
- **macOS 上 viewer 必须用 `mjpython` 启动**（Cocoa 主线程要求），不是 `python`。本项目 `ensure_dependencies` 已正确处理。
