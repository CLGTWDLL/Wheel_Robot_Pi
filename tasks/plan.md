# Implementation Plan: 达妙四电机 USB-CAN 控制器

## Context

本计划基于 `/Users/cqc/mojoco_lqr/SPEC.md`。目标是在 macOS 上通过达妙官方 USB-CAN 工具同步控制四个达妙电机：DM4310 左右腿部关节使用 MIT 模式，DMH6215 左右轮毂电机第一阶段使用速度模式。运行时控制目标是四电机同步执行和反馈聚合；单电机测试仅作为首次实机验证的安全 bring-up 步骤。实现必须支持 dry-run、反馈帧解析、Master_ID 匹配、安全 gating，并在任何硬件冒烟测试结束前失能电机。

用户要求最终在仓库中保存：

- `/Users/cqc/mojoco_lqr/tasks/plan.md`
- `/Users/cqc/mojoco_lqr/tasks/todo.md`

本文件已按批准计划保存到仓库，供后续实现任务执行。

## Architecture Decisions

- 新增 `damiao_can/` 包，不改动现有 MuJoCo 仿真代码。
- 协议层保持纯函数，不导入 `python-can`，便于无硬件测试。
- `motors.py` 固化四电机配置和安全限制。
- `bus.py` 是唯一打开硬件的模块，使用 `can.Bus(interface="damiao", channel=<serial>, bitrate=1000000)`。
- CLI 默认 dry-run 优先；真实非零命令必须显式 `--allow-motion`。
- 运行时控制目标是四电机同步命令执行和反馈聚合，而不是主要逐个电机控制。
- 反馈判断以 `msg.arbitration_id == motor.master_id` 为准，不能把发送成功确认当作电机反馈。
- 第一阶段不做 MuJoCo/LQR 闭环集成，不写入电机 flash 参数。

## Dependency Graph

```text
pyproject dependencies: python-can, python-can-damiao
  └── damiao_can package
      ├── motors.py
      │   └── four motor registry + master_id lookup
      ├── protocol.py
      │   ├── command frames: enable / disable / clear-error / zero
      │   ├── MIT packing for DM4310
      │   ├── velocity packing for DMH6215
      │   ├── parameter frames
      │   └── feedback decoding
      ├── bus.py
      │   ├── create damiao serial bus
      │   ├── send can.Message
      │   └── recv feedback by Master_ID
      └── cli.py
          ├── dry-run status
          ├── enable-all / disable-all / clear-error
          ├── DM4310 mit
          ├── DMH6215 velocity
          └── guarded smoke-test

tests
  ├── test_damiao_protocol.py depends on motors.py + protocol.py
  └── test_damiao_cli.py depends on cli.py dry-run paths
```

## Task List

### Phase 1: Planning Artifacts and Protocol Foundation

#### Task 1: Save repo task documents

**Description:** Create the requested planning files in the repository so future agents can execute from stable artifacts.

**Acceptance criteria:**
- [ ] `/Users/cqc/mojoco_lqr/tasks/plan.md` contains the approved implementation plan.
- [ ] `/Users/cqc/mojoco_lqr/tasks/todo.md` contains an ordered checkbox task list.
- [ ] No source code is changed in this task.

**Verification:**
- [ ] Read both files and confirm content matches the approved plan.

**Dependencies:** Plan approval.

**Files likely touched:**
- `/Users/cqc/mojoco_lqr/tasks/plan.md`
- `/Users/cqc/mojoco_lqr/tasks/todo.md`

**Estimated scope:** S

#### Task 2: Add package skeleton and motor registry

**Description:** Add `damiao_can` package with static four-motor configuration and lookup helpers.

**Acceptance criteria:**
- [ ] `dm4310_left`: CAN_ID `0x02`, Master_ID `0x12`, mode `mit`.
- [ ] `dm4310_right`: CAN_ID `0x03`, Master_ID `0x13`, mode `mit`.
- [ ] `dmh6215_left`: CAN_ID `0x04`, Master_ID `0x14`, mode `velocity`.
- [ ] `dmh6215_right`: CAN_ID `0x05`, Master_ID `0x15`, mode `velocity`.
- [ ] Lookup by motor name and Master_ID works with clear errors for unknown values.

**Verification:**
- [ ] `uv run pytest tests/test_damiao_protocol.py`
- [ ] `uv run pytest`

**Dependencies:** Task 1.

**Files likely touched:**
- `/Users/cqc/mojoco_lqr/damiao_can/__init__.py`
- `/Users/cqc/mojoco_lqr/damiao_can/motors.py`
- `/Users/cqc/mojoco_lqr/tests/test_damiao_protocol.py`

**Estimated scope:** S

#### Task 3: Implement pure protocol packing and feedback decoding

**Description:** Implement deterministic CAN frame helpers with no hardware dependency.

**Acceptance criteria:**
- [ ] Enable payload is `FF FF FF FF FF FF FF FC`.
- [ ] Disable payload is `FF FF FF FF FF FF FF FD`.
- [ ] Clear-error payload is `FF FF FF FF FF FF FF FB`.
- [ ] MIT packing uses frame ID `CAN_ID` and correct 16/12/12/12/12-bit packing.
- [ ] Velocity mode uses frame ID `0x200 + CAN_ID`, little-endian float in bytes 0-3, zeros in bytes 4-7.
- [ ] Feedback decoding extracts payload ID, state, position, velocity, torque, MOS temperature, rotor temperature.
- [ ] Parameter read/write/save helpers use arbitration ID `0x7FF` and command bytes `0x33`, `0x55`, `0xAA` if included in this phase.

**Verification:**
- [ ] `uv run pytest tests/test_damiao_protocol.py`
- [ ] `uv run pytest`

**Dependencies:** Task 2.

**Files likely touched:**
- `/Users/cqc/mojoco_lqr/damiao_can/protocol.py`
- `/Users/cqc/mojoco_lqr/tests/test_damiao_protocol.py`

**Estimated scope:** M

### Checkpoint: Protocol Foundation

- [ ] Protocol tests pass without CAN hardware.
- [ ] `protocol.py` does not import `can`, `serial`, or hardware modules.
- [ ] Frame bytes match `SPEC.md` and docs references.
- [ ] Feedback routing design uses Master_ID.

### Phase 2: Dry-Run CLI Vertical Slice

#### Task 4: Implement dry-run status command

**Description:** Provide a no-hardware CLI path that prints configured motors and representative frames.

**Acceptance criteria:**
- [ ] `uv run python -m damiao_can.cli --dry-run status` works without CAN hardware.
- [ ] Output includes motor name, CAN_ID, Master_ID, mode, and representative frame info.
- [ ] Dry-run does not import/open `can.Bus`.
- [ ] Unknown command or motor returns a clear nonzero error.

**Verification:**
- [ ] `uv run pytest tests/test_damiao_cli.py`
- [ ] `uv run pytest`
- [ ] Manual: `uv run python -m damiao_can.cli --dry-run status`

**Dependencies:** Tasks 2-3.

**Files likely touched:**
- `/Users/cqc/mojoco_lqr/damiao_can/cli.py`
- `/Users/cqc/mojoco_lqr/tests/test_damiao_cli.py`

**Estimated scope:** M

#### Task 5: Extend dry-run for all planned commands and safety gates

**Description:** Make every hardware command previewable before real CAN sends.

**Acceptance criteria:**
- [ ] Dry-run supports `enable-all`, `disable-all`, `clear-error`, `mit`, `velocity`, and `smoke-test`.
- [ ] DM4310 `mit` defaults to zero command.
- [ ] DMH6215 `velocity` defaults to zero command.
- [ ] Real nonzero position, velocity, torque, `kp`, or `kd` requires `--allow-motion`.
- [ ] Dry-run `smoke-test` includes final disable operation.

**Verification:**
- [ ] `uv run pytest tests/test_damiao_cli.py`
- [ ] `uv run pytest`
- [ ] Manual dry-runs for `status`, `smoke-test`, `mit --motor dm4310_left`, and `velocity --motor dmh6215_left --velocity 0`.

**Dependencies:** Task 4.

**Files likely touched:**
- `/Users/cqc/mojoco_lqr/damiao_can/cli.py`
- `/Users/cqc/mojoco_lqr/tests/test_damiao_cli.py`

**Estimated scope:** M

### Checkpoint: Safe No-Hardware Usability

- [ ] `uv run pytest tests/test_damiao_protocol.py tests/test_damiao_cli.py` passes.
- [ ] `uv run pytest` passes.
- [ ] Every command can be inspected via dry-run.
- [ ] Safety gate blocks accidental real nonzero motion.

### Phase 3: CAN Bus and Feedback Slice

#### Task 6: Add dependencies and macOS Damiao bus adapter

**Description:** Add `python-can` and `python-can-damiao`, then isolate hardware access in `bus.py`.

**Acceptance criteria:**
- [ ] `pyproject.toml` and `uv.lock` include `python-can` and `python-can-damiao`.
- [ ] Bus creation uses `interface="damiao"`, explicit `--channel`, and default bitrate `1000000`.
- [ ] No Linux SocketCAN `can0` assumption is introduced.
- [ ] CLI can list or instruct how to list serial ports.

**Verification:**
- [ ] `uv run pytest`
- [ ] `uv run python -c "import can; import damiao_can.bus"`
- [ ] `uv run python -m serial.tools.list_ports -v`

**Dependencies:** Tasks 4-5.

**Files likely touched:**
- `/Users/cqc/mojoco_lqr/pyproject.toml`
- `/Users/cqc/mojoco_lqr/uv.lock`
- `/Users/cqc/mojoco_lqr/damiao_can/bus.py`
- `/Users/cqc/mojoco_lqr/damiao_can/cli.py`

**Estimated scope:** M

#### Task 7: Implement feedback receive by Master_ID

**Description:** Decode only real feedback messages whose arbitration ID matches configured Master_ID.

**Acceptance criteria:**
- [ ] Feedback helper ignores unrelated arbitration IDs.
- [ ] `0x12`, `0x13`, `0x14`, `0x15` map to the correct motors.
- [ ] Sending success is not considered motor response.
- [ ] CLI can print decoded timestamp, motor name, state, position, velocity, torque, MOS temp, rotor temp.

**Verification:**
- [ ] Unit tests with fake messages.
- [ ] `uv run pytest tests/test_damiao_protocol.py tests/test_damiao_cli.py`
- [ ] `uv run pytest`

**Dependencies:** Task 6.

**Files likely touched:**
- `/Users/cqc/mojoco_lqr/damiao_can/bus.py`
- `/Users/cqc/mojoco_lqr/damiao_can/protocol.py`
- `/Users/cqc/mojoco_lqr/tests/test_damiao_protocol.py`

**Estimated scope:** M

### Checkpoint: Hardware Boundary Ready

- [ ] Tests pass without hardware.
- [ ] CAN imports are isolated to bus/CLI paths.
- [ ] Feedback matching is proven by tests using Master_ID.
- [ ] macOS serial discovery command is documented and works.

### Phase 4: Conservative Hardware Commands

#### Task 8: Wire real enable/disable/clear-error commands

**Description:** Connect safe all-motor command paths to the bus wrapper.

**Acceptance criteria:**
- [ ] `enable-all`, `disable-all`, and `clear-error` send standard 8-byte frames to all four CAN_IDs.
- [ ] Commands print sent arbitration ID and payload.
- [ ] Errors are clear and do not hide partial send state.
- [ ] `disable-all` remains a simple cleanup command.

**Verification:**
- [ ] `uv run pytest`
- [ ] Manual hardware only after explicit approval: dry-run → real `disable-all` → staged enable/zero feedback check if needed → synchronized zero-command check → `disable-all`.

**Dependencies:** Tasks 6-7.

**Files likely touched:**
- `/Users/cqc/mojoco_lqr/damiao_can/cli.py`
- `/Users/cqc/mojoco_lqr/damiao_can/bus.py`

**Estimated scope:** S

#### Task 9: Wire bring-up-only single-motor MIT and velocity commands with safety gates

**Description:** Add real single-motor paths for DM4310 MIT and DMH6215 wheel velocity mode as safety bring-up tools, not as the final runtime control model.

**Acceptance criteria:**
- [ ] DM4310 `mit` rejects DMH6215 motors.
- [ ] DMH6215 `velocity` rejects DM4310 motors.
- [ ] Zero commands can run without `--allow-motion`.
- [ ] Nonzero command values require `--allow-motion` and conservative limit checks.
- [ ] After send, command attempts to read matching Master_ID feedback and reports whether the target responded.

**Verification:**
- [ ] CLI tests for allowed/rejected motor-mode combinations.
- [ ] CLI tests for nonzero safety gate.
- [ ] `uv run pytest`

**Dependencies:** Task 8.

**Files likely touched:**
- `/Users/cqc/mojoco_lqr/damiao_can/cli.py`
- `/Users/cqc/mojoco_lqr/damiao_can/bus.py`
- `/Users/cqc/mojoco_lqr/tests/test_damiao_cli.py`

**Estimated scope:** M

#### Task 10: Implement guarded synchronized four-motor command path

**Description:** Add synchronized four-motor command execution and feedback aggregation as the runtime control target, while keeping the first physical validation conservative: enable, send zero-safe commands, read feedback, and always disable on exit.

**Acceptance criteria:**
- [ ] Default first-validation path sends only zero MIT commands to DM4310 and zero velocity commands to DMH6215.
- [ ] Runtime command path can send one synchronized command set to all four motors and aggregate feedback by Master_ID.
- [ ] Any nonzero wheel motion requires `--allow-motion` and explicit approval.
- [ ] Optional first physical motion can still be staged one motor at a time, low speed, short duration.
- [ ] `disable-all` is sent in cleanup on success, failure, timeout, or interrupt.
- [ ] Success is based on aggregated Master_ID feedback, state, velocity sign/threshold when moving, temperature, and absence of fault state.

**Verification:**
- [ ] Dry-run synchronized four-motor path shows all command frames and final disable for first validation.
- [ ] Mocked exception test verifies cleanup disable is attempted.
- [ ] `uv run pytest tests/test_damiao_protocol.py tests/test_damiao_cli.py`
- [ ] `uv run pytest`
- [ ] Manual hardware smoke-test only after explicit user approval and physical safety checks; nonzero motion requires approval.

**Dependencies:** Tasks 7-9.

**Files likely touched:**
- `/Users/cqc/mojoco_lqr/damiao_can/cli.py`
- `/Users/cqc/mojoco_lqr/damiao_can/bus.py`
- `/Users/cqc/mojoco_lqr/tests/test_damiao_cli.py`

**Estimated scope:** M

### Checkpoint: First Hardware Prototype Complete

- [ ] Full test suite passes.
- [ ] Dry-run output reviewed before hardware sends.
- [ ] Real hardware commands require explicit channel and nonzero motion approval.
- [ ] Smoke-test always disables on exit.
- [ ] No motor flash writes occur by default.

## Verification Matrix

| Area | Command / Check |
|---|---|
| Protocol tests | `uv run pytest tests/test_damiao_protocol.py` |
| CLI dry-run tests | `uv run pytest tests/test_damiao_cli.py` |
| Full suite | `uv run pytest` |
| Dry-run status | `uv run python -m damiao_can.cli --dry-run status` |
| Dry-run smoke-test | `uv run python -m damiao_can.cli --dry-run smoke-test` |
| macOS serial discovery | `uv run python -m serial.tools.list_ports -v` |
| Import check | `uv run python -c "import can; import damiao_can.bus"` |
| Manual hardware prerequisite | IDs/modes confirmed; verify wiring, 1 Mbps, secured motors, emergency stop |
| Manual real command order | dry-run → disable-all → staged zero bring-up if needed → synchronized zero command → aggregated feedback check → disable-all |

## Risks and Mitigations

| Risk | Impact | Mitigation |
|---|---:|---|
| Wrong CAN_ID or Master_ID | High | Hard-code values from SPEC in `motors.py`; test registry and feedback routing. |
| Treating send ack as feedback | High | Only accept `bus.recv()` messages whose arbitration ID equals configured Master_ID. |
| Accidental nonzero motion | High | Require `--allow-motion`; clamp to conservative limits. |
| Smoke-test leaves motor enabled | High | Use cleanup path that always sends `disable-all`; add mocked exception test. |
| macOS channel mismatch | Medium | Require explicit `--channel`; provide serial listing command. |
| Unknown exact safe motor ranges | Medium | Centralize limits; clamp commands; do not write motor params without approval. |
| Flash wear | Medium | Do not expose save-by-default; require explicit confirmation for `0xAA`. |

## Open Questions Before Real Hardware Use

- What is the exact macOS serial channel, e.g. `/dev/tty.usbmodemXXXX`?
- What conservative phase-1 wheel velocity limit should be used if `--allow-motion` is approved?
- Should `clear-error` support all-motor only, single-motor only, or both?

Resolved: CAN_ID、Master_ID、控制模式和 1 Mbps 波特率已确认；运行时控制目标为四电机同步控制。

## Repository todo.md Content To Create After Approval

```markdown
# Todo: 达妙四电机 USB-CAN 控制器

- [ ] Save approved plan to `tasks/plan.md` and this checklist to `tasks/todo.md`.
- [ ] Add `damiao_can` package skeleton and static motor registry.
- [ ] Implement pure protocol frame packing and feedback decoding.
- [ ] Add protocol tests for command frames, MIT packing, velocity packing, and feedback decoding.
- [ ] Implement dry-run CLI status and command previews.
- [ ] Add dry-run CLI tests and safety-gate tests.
- [ ] Add `python-can` and `python-can-damiao` dependencies.
- [ ] Implement CAN bus adapter for macOS `damiao` serial channel.
- [ ] Implement feedback receive by Master_ID matching.
- [ ] Wire real enable/disable/clear-error commands.
- [ ] Wire single-motor MIT and velocity commands with nonzero approval gate for bring-up only.
- [ ] Implement synchronized four-motor command execution and feedback aggregation.
- [ ] Implement guarded smoke-test with unconditional disable cleanup.
- [ ] Verify with `uv run pytest tests/test_damiao_protocol.py tests/test_damiao_cli.py`.
- [ ] Verify with `uv run pytest`.
- [ ] Perform manual hardware validation only after explicit approval.
```
