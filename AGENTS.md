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
   - 机器人的腿部包含五杆机构闭环结构。在 `mjmodel.xml` 中，这是通过 `<equality>` 标签将 `link2` 和 `link3` 的 `connect_site` 连接起来实现的。
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
   - 仓库中的原始 XML 文件（如 `sim/轮腿v1/轮腿v1.xml`）仅仅是**模板源文件**。
   - 真实的物理仿真模型是在 `launch_mujoco.py` 运行时，通过 `sim/model_xml.py` 中的 `prepare_controlled_mujoco_xml` 动态生成的。
   - 该 Python 脚本会执行许多关键“魔改”：比如将所有 mesh 设为纯视觉（visual-only）、强行注入 `sphere` 碰撞体作为轮子（比 cylinder 更利于 LQR 平滑控制）、强行注入 `root` freejoint、以及注入命令滑块等。
   - **禁止**仅凭阅读原始的 `.xml` 文件就得出系统结构结论，更不要盲目在原始 XML 中添加复杂的碰撞网格。任何针对仿真物理环境的判断，必须以 `sim/model_xml.py` 处理后的动态模型为准。

## 实机硬件 CAN 配置与调试规则

### 电机与 IMU CAN 配置

实际硬件已按此配置完成：PCB 通过 CAN 连接 4 个电机和 DM-IMU。

| 设备       | CAN_ID | Master/MST_ID | 控制模式     |
| ---------- | ------ | ------------- | ------------ |
| DM-IMU     | 0x01   | 0x11          | -            |
| DM4310 左  | 0x02   | 0x12          | MIT 控制模式 |
| DM4310 右  | 0x03   | 0x13          | MIT 控制模式 |
| DMH6215 左 | 0x04   | 0x14          | 速度控制模式 |
| DMH6215 右 | 0x05   | 0x15          | 速度控制模式 |

> **参数参考**：具体电机参数请参考 `docs/` 目录下的硬件手册。

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
8. 现场确认 DMH6215 当前实际为速度模式；不要对 0x04/0x05 发送 `motor torque` 测试。
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
