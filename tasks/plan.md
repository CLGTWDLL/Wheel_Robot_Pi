# macOS USB-CAN / 达妙电机调通计划

## Context

目标是在 macOS 上通过已连接的 USB-CAN 串口设备调通 4 个达妙电机和 DM-IMU：Python 负责 CLI、设备表、协议组织和安全限幅，C++ 负责底层串口/CAN 帧收发。第一阶段重点不是高性能控制，而是安全确认“4 个电机能否一起接收命令”，并保持测试参数保守。

已读取并核对：

- `SPEC.md`：已确认 Python 主入口、C++ 底层、SLCAN/串口、保守 group 测试边界。
- `CLAUDE.md`：硬件 CAN 配置：DM-IMU `0x01/0x11`，DM4310 左右 `0x02/0x12`、`0x03/0x13` MIT，DMH6215 左右 `0x04/0x14`、`0x05/0x15` 力矩。
- `control/damiao_can/protocol.py`：已有 `CanFrame`、MIT 编码、enable/disable/zero-position、反馈解码、参数帧。
- `control/damiao_can/motors.py`：已有 4 电机表，但 DMH6215 当前是 `velocity`，需要改成 `torque`。
- `pyproject.toml`：当前仅有 `mujoco` 依赖；后续可能需要 `pyserial`、`pytest`、`pybind11`。

关键约束：

- `group zero` 必须表示“4 电机零输出”，不能调用 `zero_position_frame`/`0xFE` 设零点。
- 多电机非零测试必须在单电机测试和 `group zero` 之后。
- 非零命令必须有 duration，结束和异常路径都要发送零输出或 disable。
- 默认测试参数保守：不为了看见明显动作而提高默认力矩/速度/位置范围。

## Dependency graph

```text
SPEC.md + CLAUDE.md
  -> 设备表与模式定义
     -> 协议编码/反馈解码
     -> 安全限幅与 duration 策略
        -> 单电机命令构建
        -> group zero / group conservative-test 命令构建
           -> FakeCanBus 与执行器
           -> macOS C++ 串口/CAN backend
              -> Python slcan wrapper
                 -> CLI: list-ports / sniff / motor / group / imu
                    -> 实机验证流程
```

## Recommended implementation slices

### Phase 1: 离线基础与安全闭环

#### Task 1 — 对齐设备表与硬件配置

改动目标：`control/damiao_can/motors.py`，必要时新增 `control/devices.py`。

验收标准：

- 设备表包含 `dm_imu`、`dm4310_left`、`dm4310_right`、`dmh6215_left`、`dmh6215_right`。
- CAN_ID/MST_ID 与 `CLAUDE.md` 完全一致。
- DM4310 模式为 `mit`。
- DMH6215 模式改为 `torque`。
- 没有暴露参数保存、写 flash 或设零点的默认实机命令。

验证：

- `uv run pytest tests/test_devices.py`
- 测试覆盖设备名、CAN_ID、MST_ID、mode。

#### Task 2 — 补齐协议 command builders

改动目标：复用 `control/damiao_can/protocol.py`，新增 `control/damiao_can/commands.py`。

验收标准：

- 继续复用 enable/disable/MIT 编码。
- 增加 DMH6215 torque-mode frame builder。
- `group zero` 生成 4 帧，顺序固定为 `dm4310_left`、`dm4310_right`、`dmh6215_left`、`dmh6215_right`。
- `group zero` 明确不调用 `zero_position_frame`，不产生 `0xFE`。

验证：

- `uv run pytest tests/test_damiao_protocol.py`
- 测试覆盖帧数据长度、arbitration_id、group 顺序、无 `0xFE`。

#### Task 3 — 集中安全限幅与 duration 策略

改动目标：新增 `control/damiao_can/safety.py`。

验收标准：

- 非零 torque/运动命令没有 duration 时拒绝。
- duration 默认不超过 `0.5 s`，越界拒绝或要求显式调整。
- torque 默认示例 `0.02–0.05 N·m`，硬上限 `0.05 N·m`。
- NaN/inf 全部拒绝。
- `motor torque` 只允许 torque-mode 电机。
- group conservative 参数集中定义，不散落在 CLI 中。
- DM4310 conservative-test 不默认用高增益拉到绝对 0 位。

验证：

- `uv run pytest tests/test_safety.py`

#### Task 4 — FakeCanBus 与命令执行器

改动目标：新增 `control/bus.py`、`control/fake_bus.py`，命令执行逻辑放在 `control/damiao_can/commands.py` 或专门 runner 中。

验收标准：

- FakeCanBus 可记录发送帧、模拟 receive timeout、注入反馈帧/IMU raw frame。
- 非零命令执行器使用 `try/finally`，结束和异常路径都会发送零输出。
- fake 模式可完整跑 `group zero` 和 `group conservative-test`。

验证：

- `uv run pytest tests/test_fake_bus.py tests/test_group_commands.py`
- `uv run python -m control.can_tool group zero --backend fake --duration 0.1`
- `uv run python -m control.can_tool group conservative-test --backend fake --duration 0.1`

Checkpoint A：只有在 `uv run pytest` 通过、设备表一致、DMH6215 模式已修正、`group zero` 确认不是设零点后，才进入真实串口 backend。

### Phase 2: macOS 串口/CAN backend 与只读 CLI

#### Task 5 — 实现 macOS C++ 串口/CAN backend

改动目标：新增 `control/native/slcan_bus.cpp`、`control/native/bindings.cpp`、`control/slcan.py`，并更新 `pyproject.toml` 构建依赖。

验收标准：

- 使用 macOS POSIX/termios，不依赖 Linux-only 头文件。
- open/read/write/close 都有 timeout 或明确错误。
- Python 侧统一暴露 `send(CanFrame)` / `recv(timeout)`。
- 底层 codec 先按实机确认：标准 ASCII SLCAN 或达妙 USB-CAN 二进制串口协议；上层接口保持不变。
- 不存在串口时错误清楚，不 hang。

验证：

- `uv run pytest tests/test_slcan_codec.py`
- `uv run python -m control.can_tool sniff --port /dev/nonexistent --duration 0.2`

#### Task 6 — 实现 `list-ports` 与 `sniff`

改动目标：新增/完善 `control/can_tool.py`。

验收标准：

- `list-ports` 列出 macOS `/dev/cu.*` 和 `/dev/tty.*`。
- `sniff` 支持 `--port`、`--serial-baud`、`--can-bitrate`、`--duration`。
- CLI 打印 backend、serial baud、CAN bitrate。
- 打印 timestamp、arbitration_id、dlc、data。
- timeout 后正常退出。

验证：

- `uv run python -m control.can_tool list-ports`
- `uv run python -m control.can_tool sniff --port /dev/nonexistent --duration 0.2`
- `uv run pytest tests/test_cli.py`

Checkpoint B：确认能找到 USB-CAN 串口，确认设备真实协议，且仍未发送任何非零电机输出。

### Phase 3: 单设备实机闭环

#### Task 7 — 单电机 enable/disable

改动目标：`control/can_tool.py`、`control/damiao_can/commands.py`。

验收标准：

- 只接受已知 motor name。
- 命令打印 motor name、CAN_ID、MST_ID、mode。
- enable/disable 只发送对应标准帧，不附带非零控制。
- 收到反馈时打印 position、velocity、torque、temperature；无反馈时明确 timeout。

验证：

- fake bus 测试 enable/disable frame。
- 实机单电机：`uv run python -m control.can_tool motor enable --port <PORT> --motor dm4310_left` 后立即 disable。

#### Task 8 — 单个 DMH6215 保守 torque 测试

改动目标：`control/can_tool.py`、`control/damiao_can/safety.py`、`control/damiao_can/commands.py`。

验收标准：

- 只允许 `dmh6215_left/right`。
- 默认 torque 不超过 `0.05 N·m`，建议先用 `0.02 N·m`。
- 默认 duration 不超过 `0.5 s`。
- 执行顺序：零输出 -> enable -> 小 torque -> 零输出 -> disable/安全状态。
- Ctrl-C 或异常时仍发送零输出。

验证：

- `uv run pytest tests/test_safety.py tests/test_cli.py`
- 实机：`uv run python -m control.can_tool motor torque --port <PORT> --motor dmh6215_left --torque 0.02 --duration 0.3`

#### Task 9 — IMU raw read

改动目标：`control/can_tool.py`，必要时新增 `control/damiao_can/imu.py`。

验收标准：

- `imu read` 支持 duration。
- 打印 timestamp、CAN ID、dlc、raw data。
- 能标记疑似 DM-IMU 帧；第一版不强求完整姿态解析。
- 不发送任何电机控制帧。

验证：

- `uv run python -m control.can_tool imu read --port <PORT> --duration 5.0`
- fake bus 注入 IMU frame 测试。

Checkpoint C：`sniff` 稳定、单电机 enable/disable 可用、单个 DMH6215 小 torque 能安全退出、IMU read 能运行到 duration 结束后，才进入 4 电机联动。

### Phase 4: 四电机保守联动

#### Task 10 — `group zero`

改动目标：`control/can_tool.py`、`control/damiao_can/commands.py`、`tests/test_group_commands.py`。

验收标准：

- 每个周期发送 4 台电机零输出帧，顺序固定。
- 默认 duration `1.0 s`，period 保守，例如 `50 ms`。
- 打印每台设备 CAN_ID、MST_ID、mode、frame arbitration_id。
- 不使用 `0xFE` 设零点，不发送参数写入/保存。
- 结束时再次发送零输出或 disable，并打印最终状态。

验证：

- `uv run pytest tests/test_group_commands.py`
- fake：`uv run python -m control.can_tool group zero --backend fake --duration 1.0`
- 实机：`uv run python -m control.can_tool group zero --port <PORT> --duration 1.0`

Checkpoint D：人工确认电机安全支撑、急停/断电可用、`group zero` 无异常、4 电机均已发送且无持续输出后，才运行 conservative-test。

#### Task 11 — `group conservative-test`

改动目标：`control/can_tool.py`、`control/damiao_can/commands.py`、`control/damiao_can/safety.py`。

验收标准：

- 启动时提示必须先完成 `group zero`。
- 命令自身内置 pre-zero。
- DM4310 使用零输出或当前位姿低增益 hold；没有可靠反馈时不拉绝对 0 位。
- DMH6215 torque 不超过安全上限，默认建议 `0.02 N·m`，最多 `0.05 N·m`。
- duration 默认不超过 `0.5 s`。
- 每个控制周期 4 帧同一 batch 发送，顺序固定。
- 结束、异常、Ctrl-C 都发送 group zero，并尽量 disable。
- 不自动重试或提高参数。

验证：

- `uv run pytest tests/test_group_commands.py`
- fake：`uv run python -m control.can_tool group conservative-test --backend fake --duration 0.5`
- 实机前必须通过 Checkpoint D。
- 实机：`uv run python -m control.can_tool group conservative-test --port <PORT> --duration 0.5`

Checkpoint E：核心目标完成条件：pytest 通过、list-ports/sniff 可用、单电机可控、单 DMH6215 小 torque 安全、`group zero` 可用、`group conservative-test` 在保守参数下可安全退出，且未写参数、未设零点、未使用激进参数。

### Phase 5: 调试 runbook

#### Task 12 — 硬件调试流程与日志模板

改动目标：`README.md` 或 `control/docs/hardware_bringup.md`。

验收标准：

- 写明实机前检查清单。
- 写明推荐执行顺序：list-ports -> sniff/imu read -> 单电机 enable/disable -> 单 DMH6215 小 torque -> group zero -> group conservative-test。
- 明确 `group zero` 是零输出，不是设零点。
- 明确如需提高参数必须人工确认。
- 提供日志模板：time、port、serial baud、CAN bitrate、command、targets、feedback、abnormal behavior、final safety state。

验证：

- 按 runbook 从 `list-ports` 到 `group conservative-test` 走一遍并记录日志。

## Files to create or modify after approval

- `tasks/plan.md`：保存本计划的执行版。
- `tasks/todo.md`：保存按阶段和任务拆分的勾选列表。
- `control/damiao_can/motors.py`：修正设备表。
- `control/damiao_can/protocol.py`：复用并补齐协议函数。
- `control/damiao_can/commands.py`：新增命令构建与 group 命令。
- `control/damiao_can/safety.py`：新增保守限幅策略。
- `control/bus.py`、`control/fake_bus.py`：新增可测 bus 接口。
- `control/slcan.py`：Python native backend wrapper。
- `control/native/slcan_bus.cpp`、`control/native/bindings.cpp`：macOS C++ 串口/CAN backend。
- `control/can_tool.py`：CLI 主入口。
- `tests/test_devices.py`、`tests/test_damiao_protocol.py`、`tests/test_safety.py`、`tests/test_fake_bus.py`、`tests/test_group_commands.py`、`tests/test_cli.py`、`tests/test_slcan_codec.py`：离线验证。
- `README.md` 或 `control/docs/hardware_bringup.md`：实机 runbook。
- `pyproject.toml`：必要时添加 `pytest`、`pyserial`、`pybind11` 等依赖。

## Task list content for `tasks/todo.md`

```markdown
# TODO: macOS USB-CAN / 达妙电机调通

## Phase 1: Offline Foundation

- [ ] Task 1: 对齐设备表与硬件配置
  - [ ] 添加/确认 `dm_imu`
  - [ ] 确认 DM4310 左右为 MIT
  - [ ] 修正 DMH6215 左右为 torque
  - [ ] 添加设备表测试
  - [ ] 运行 `uv run pytest tests/test_devices.py`

- [ ] Task 2: 补齐协议 command builders
  - [ ] 复用 enable/disable/MIT 编码
  - [ ] 增加 DMH6215 torque frame
  - [ ] 增加 group zero frame builder
  - [ ] 测试 group zero 不产生 `0xFE`
  - [ ] 运行 `uv run pytest tests/test_damiao_protocol.py`

- [ ] Task 3: 建立安全限幅与 duration 策略
  - [ ] 实现 torque/duration/MIT 参数校验
  - [ ] 拒绝 NaN/inf
  - [ ] 限制 torque-mode 命令目标
  - [ ] 集中定义 group conservative 参数
  - [ ] 运行 `uv run pytest tests/test_safety.py`

- [ ] Task 4: 增加 FakeCanBus 与命令执行器
  - [ ] FakeCanBus 记录发送帧
  - [ ] 支持 timeout 和注入反馈
  - [ ] 非零命令 `try/finally` 发送零输出
  - [ ] fake 跑通 group zero/conservative-test
  - [ ] 运行 `uv run pytest tests/test_fake_bus.py tests/test_group_commands.py`

- [ ] Checkpoint A: Offline Safety Ready
  - [ ] `uv run pytest` 通过
  - [ ] 设备表与 `CLAUDE.md` 一致
  - [ ] `group zero` 确认不是设零点
  - [ ] conservative 参数已硬限制

## Phase 2: Local macOS Serial/CAN Backend

- [ ] Task 5: 实现 macOS C++ 串口/CAN backend
  - [ ] 使用 macOS POSIX/termios
  - [ ] open/read/write/close 有错误处理和 timeout
  - [ ] 暴露统一 Python bus 接口
  - [ ] 确认 USB-CAN 是标准 SLCAN 还是达妙二进制协议
  - [ ] 运行 `uv run pytest tests/test_slcan_codec.py`

- [ ] Task 6: 实现 list-ports 与 sniff CLI
  - [ ] `list-ports` 列出 `/dev/cu.*` 和 `/dev/tty.*`
  - [ ] `sniff` 支持 port/serial-baud/can-bitrate/duration
  - [ ] 不存在串口时错误清楚
  - [ ] 运行 `uv run pytest tests/test_cli.py`

- [ ] Checkpoint B: Local Serial Ready
  - [ ] 找到 USB-CAN 串口
  - [ ] nonexistent port 不 hang
  - [ ] 确认真实串口协议
  - [ ] 未发送任何非零电机输出

## Phase 3: Single Device Hardware Slice

- [ ] Task 7: 单电机 enable/disable
  - [ ] fake bus 验证 frame
  - [ ] 实机单电机 enable 后立即 disable
  - [ ] 记录反馈或 timeout

- [ ] Task 8: 单个 DMH6215 保守 torque 测试
  - [ ] 只允许 `dmh6215_left/right`
  - [ ] 默认 torque 建议 `0.02 N·m`，不超过 `0.05 N·m`
  - [ ] duration 不超过 `0.5 s`
  - [ ] 异常路径发送零输出
  - [ ] 实机先跑 `--torque 0.02 --duration 0.3`

- [ ] Task 9: IMU raw read
  - [ ] duration 到时退出
  - [ ] 打印 raw CAN frame
  - [ ] 不发送电机控制帧

- [ ] Checkpoint C: Single Device Hardware Ready
  - [ ] sniff 稳定
  - [ ] enable/disable 可用
  - [ ] 单 DMH6215 小 torque 安全退出
  - [ ] IMU read 能运行到 duration 结束

## Phase 4: Four-Motor Vertical Slice

- [ ] Task 10: group zero
  - [ ] 固定顺序发送 4 台电机零输出
  - [ ] 不使用 `0xFE`
  - [ ] 不发送参数写入/保存
  - [ ] fake 和实机均验证

- [ ] Checkpoint D: Group Zero Passed
  - [ ] 电机处于安全支撑状态
  - [ ] 急停或断电手段可用
  - [ ] group zero 无异常动作
  - [ ] 已记录 port、baud、bitrate、反馈情况

- [ ] Task 11: group conservative-test
  - [ ] 内置 pre-zero
  - [ ] 默认 duration 不超过 `0.5 s`
  - [ ] DM4310 不默认拉绝对 0 位
  - [ ] DMH6215 不超过 torque 上限
  - [ ] 结束/异常都发送 group zero
  - [ ] 不自动提高参数

- [ ] Checkpoint E: Four-Motor Conservative Slice Passed
  - [ ] 4 台电机同 batch 发送命令
  - [ ] 测试安全退出
  - [ ] 未写参数、未设零点、未使用激进参数

## Phase 5: Documentation and Runbook

- [ ] Task 12: 补充硬件调试 runbook 与日志模板
  - [ ] 写明执行顺序
  - [ ] 写明 `group zero` 不是设零点
  - [ ] 写明提高参数需要人工确认
  - [ ] 提供测试日志模板
```

## End-to-end verification

无硬件：

```bash
uv run pytest
uv run python -m control.can_tool group zero --backend fake --duration 0.1
uv run python -m control.can_tool group conservative-test --backend fake --duration 0.1
uv run python -m control.can_tool imu read --backend fake --duration 0.1
```

本机串口：

```bash
uv run python -m control.can_tool list-ports
uv run python -m control.can_tool sniff --port /dev/nonexistent --duration 0.2
```

实机顺序：

```bash
uv run python -m control.can_tool sniff --port <PORT> --duration 5.0
uv run python -m control.can_tool imu read --port <PORT> --duration 5.0
uv run python -m control.can_tool motor enable --port <PORT> --motor dm4310_left
uv run python -m control.can_tool motor disable --port <PORT> --motor dm4310_left
uv run python -m control.can_tool motor torque --port <PORT> --motor dmh6215_left --torque 0.02 --duration 0.3
uv run python -m control.can_tool group zero --port <PORT> --duration 1.0
uv run python -m control.can_tool group conservative-test --port <PORT> --duration 0.5
```

## Risks

- USB-CAN 可能不是标准 ASCII SLCAN，而是达妙二进制串口协议；先用 backend codec checkpoint 确认。
- 现有 C++ 示例可能 Linux-only；macOS 需用 POSIX/termios 重新实现。
- DMH6215 当前代码模式与硬件配置不一致；必须先修正并测试。
- `group zero` 容易误解为设零点；必须测试禁止 `0xFE`。
- DM4310 若用绝对 0 位 + kp 可能意外动作；conservative-test 默认不这样做。
- 参数太保守可能看不见动作；第一目标是通信与同周期命令，不是明显运动。
