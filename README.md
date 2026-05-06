# wheel_robot

## 实机步骤

列出串口：

```bash
uv run python -m control.can_tool list-ports
```

只读监听 CAN 总线：

```bash
uv run python -m control.can_tool sniff --backend dm_usbcan --duration 1.0
```

## 双电机保守测试命令

同时测试 `0x02 dm4310_left` 和 `0x04 dmh6215_left`：

```bash
uv run python -m control.can_tool pair conservative-spin --backend dm_usbcan --duration 9 --period 0.05 --velocity 0.5 --wheel-velocity 3 --torque 0.5 --telemetry
```

只测试 `0x04 dmh6215_left` 轮电机速度控制，`0x02 dm4310_left` 发送零速度：

```bash
uv run python -m control.can_tool pair conservative-spin --backend dm_usbcan --duration 0.5 --period 0.05 --velocity 0.0 --wheel-velocity 0.5 --torque 0.0 --telemetry
```

手动发送零输出收尾：

```bash
uv run python -m control.can_tool group zero --backend dm_usbcan --duration 0.2 --period 0.05 --telemetry
```

参数含义：

- `--backend dm_usbcan`：使用达妙 USB-CAN 串口后端，默认端口为 `/dev/cu.usbmodem00000000050C1`。
- `--duration`：非零控制持续时间，当前硬限制为 `0 < duration <= 0.5` 秒。
- `--period`：控制帧发送周期，单位秒；建议实机保守测试先用 `0.05`。
- `--velocity`：`0x02 dm4310_left` 的 MIT 速度命令，当前硬限制为 `abs(velocity) <= 5.0`。
- `--wheel-velocity`：`0x04 dmh6215_left` 的速度模式命令，当前硬限制为 `abs(wheel_velocity) <= 5.0`。
- `--torque`：`0x02 dm4310_left` 的 MIT torque feed-forward，当前安全上限为 `abs(torque) <= 1.0`，实机建议先从 `0.2` 开始。
- `--telemetry`：控制期间打印反馈帧，包含 `state`、`position`、`velocity`、`torque`。

注意：多行 shell 命令中每行末尾的 `\` 后面不能有空格，否则 zsh 会把下一行参数当成单独命令。
