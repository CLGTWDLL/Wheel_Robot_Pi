# Wheel Robot CAN Hardware Bring-up Runbook

## 安全检查清单

- 机械结构已安全支撑，轮子离地或有可靠约束。
- 急停、断电、拔 CAN/电源的路径已确认，现场有人值守。
- 确认设备表：DM-IMU `0x01/0x11`；DM4310 左右 `0x02/0x12`、`0x03/0x13` 为 MIT；DMH6215 左右 `0x04/0x14`、`0x05/0x15` 为速度控制；pair 测试中的 DMH6215 左 `0x04/0x14` 使用速度控制帧 `0x204`；反馈按对应 MST/MasterID（如 `0x12`、`0x14`）查看。
- 先使用 fake backend 和 nonexistent port 错误路径验证命令，不在不确定状态下连接实机发送命令。
- 不运行参数写入、参数保存、设零点命令。
- 不要让电机长期停在 enabled 状态；任何 enable 冒烟检查都必须自动或人工立即 disable。

## 推荐执行顺序

1. 离线验证：`uv run pytest`、`uv run python -m compileall control tests`。
2. 串口发现：`uv run python -m control.can_tool list-ports`。
3. 只读 sniff：`uv run python -m control.can_tool sniff --backend dm_usbcan --port <port> --serial-baud 921600 --can-bitrate 1000000 --duration 1.0`。
4. 只读 IMU：`uv run python -m control.can_tool imu read --backend dm_usbcan --port <port> --serial-baud 921600 --can-bitrate 1000000 --duration 1.0`。
5. 单电机 enable 后立即 disable，仅在机械安全和人员确认后执行；CLI `uv run python -m control.can_tool motor enable --backend dm_usbcan --port <port> --serial-baud 921600 --can-bitrate 1000000 --motor dm4310_left --duration 0.1` 是短暂 enable 后自动 disable 的冒烟命令，非持续使能工具。
6. 现场确认 DMH6215 当前实际为速度模式；不要对 0x04/0x05 发送 `motor torque` 测试。
7. 仅 0x02/0x04 双电机保守同步测试：仅在当前 CAN 总线确实只有 `0x02` 与 `0x04`，或已确认不会影响其他电机时，才运行 `uv run python -m control.can_tool pair conservative-spin --backend dm_usbcan --port <port> --serial-baud 921600 --can-bitrate 1000000 --duration 0.1 --period 0.05 --velocity 0.2 --wheel-velocity 0.2`。该命令只针对 `0x02` 与 `0x04`，其中 `0x04` 使用速度控制帧 `0x204`；结束和异常路径会发送 pair zero 并 disable 两台电机，反馈查看 MST `0x12`/`0x14`。
8. group zero：确认四电机零输出帧，无异常后再考虑下一步。推荐命令：`uv run python -m control.can_tool group zero --backend dm_usbcan --port <port> --serial-baud 921600 --can-bitrate 1000000`。
9. group conservative-test：仅使用默认保守参数，结束和异常路径都应回到 group zero。推荐命令：`uv run python -m control.can_tool group conservative-test --backend dm_usbcan --port <port> --serial-baud 921600 --can-bitrate 1000000`。

## group zero 说明

`group zero` 表示按固定顺序给四台电机发送零输出控制帧：DM4310 使用 MIT 零输出，DMH6215 使用 velocity `0.0`。它不是设零点，不会发送 `0xFE` zero-position 命令，也不写参数或保存参数。

## 提高参数的人工确认

默认测试力矩为 `0.02 N·m`，最大测试力矩硬限制为 `0.05 N·m`，但当前 0x04/0x05 为速度模式，不适用 `motor torque`；非零命令持续时间硬限制为 `0.5 s`。任何提高力矩/速度、延长持续时间、改变周期、改变 MIT 参数的动作，都必须先由现场人员确认机械状态、供电、急停和预期运动方向；不要在脚本中自动提高参数。

## 测试日志模板

- 日期/时间：
- 操作人/观察人：
- 机械支撑状态：
- 急停/断电手段：
- 串口：
- serial baud / CAN bitrate：
- 命令：
- 参数：
- 预期结果：
- 实际反馈或 timeout：
- 是否出现异常动作：
- 结束时是否已发送 zero/disable：
- 后续处理：
