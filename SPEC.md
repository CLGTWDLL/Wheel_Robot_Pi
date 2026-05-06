# SPEC: macOS USB-CAN 调通与 Python 控制入口

## 1. Objective

第一阶段目标是在 macOS 上通过已连接的达妙 USB 转 CAN 设备调通达妙电机与 DM-IMU 通信：Python 主入口通过 `control/damiao_usbcan.py` 使用达妙 USB-CAN 虚拟串口二进制协议收发 CAN 帧。

目标使用者是当前项目开发者本人，优先服务实机硬件调通，而不是先做完整机器人控制框架。

成功标准：

- 可以从 Python 打开 macOS 串口形式的达妙 USB-CAN 设备，例如 `/dev/tty.*` 或 `/dev/cu.*`，默认串口波特率 `921600`，CAN bitrate `1000000`。
- 可以监听并打印原始 CAN 帧，用于确认总线上 DM-IMU 和电机是否有响应。
- 可以对 4 个电机发送最小必要控制命令，包括 enable、disable，以及小范围 MIT/速度测试命令。
- 可以执行保守的多电机联动冒烟测试，确认 4 个电机能否在同一控制周期内一起接收命令。
- 可以读取 DM-IMU 帧；第一版可以先打印原始帧和基础字段，完整姿态解码可后续补充。
- 默认控制策略偏开发效率但测试参数保守：允许命令行直接发送小范围测试命令，但必须有强制限幅和命令超时停机。

硬件 CAN 配置以 [CLAUDE.md](CLAUDE.md) 为准：

| 设备 | CAN_ID | Master/MST_ID | 控制模式 |
| --- | --- | --- | --- |
| DM-IMU | `0x01` | `0x11` | 读取 |
| DM4310 左 | `0x02` | `0x12` | MIT 控制模式 |
| DM4310 右 | `0x03` | `0x13` | MIT 控制模式 |
| DMH6215 左 | `0x04` | `0x14` | 速度控制模式 |
| DMH6215 右 | `0x05` | `0x15` | 速度控制模式 |

## 2. Commands

使用 `uv` 管理 Python 项目和命令执行。

建议第一版提供这些命令入口：

```bash
uv run python -m control.can_tool list-ports
uv run python -m control.can_tool sniff --port /dev/cu.usbmodemXXXX --serial-baud 921600 --can-bitrate 1000000
uv run python -m control.can_tool motor enable --port /dev/cu.usbmodemXXXX --motor dm4310_left
uv run python -m control.can_tool motor disable --port /dev/cu.usbmodemXXXX --motor dm4310_left
uv run python -m control.can_tool pair conservative-spin --port /dev/cu.usbmodemXXXX --velocity 0.2 --wheel-velocity 0.2 --duration 0.5
uv run python -m control.can_tool group zero --port /dev/cu.usbmodemXXXX --duration 1.0
uv run python -m control.can_tool group conservative-test --port /dev/cu.usbmodemXXXX --duration 0.5
uv run python -m control.can_tool imu read --port /dev/cu.usbmodemXXXX --duration 5.0
```

不再使用 C++ native wrapper；第一版保持 Python 为主要用户入口，串口二进制协议封装放在 `control/damiao_usbcan.py`，协议细节参考达妙 ROS1 例程。

核心命令行为：

- `list-ports`：列出 macOS 可用串口，帮助用户找到 USB-CAN 设备。
- `sniff`：打开达妙 USB-CAN 虚拟串口，校验/声明 CAN bitrate，项目不自动修改设备 CAN bitrate；打印 timestamp、arbitration_id、dlc、data。
- `motor enable/disable`：按达妙协议发送电机使能/失能命令。
- `motor torque`：只对真实配置为力矩模式的电机发送限幅后的短时测试力矩；当前 0x04/0x05 为速度模式，不适用该命令。
- `pair conservative-spin`：只针对 `0x02` 与 `0x04`；DM4310 使用 MIT 小速度，DMH6215 使用速度控制帧 `0x204` 和 `--wheel-velocity`。
- `group zero`：对 4 个电机一起发送零输出命令，用于验证多电机命令路径和总线写入顺序；DMH6215 使用速度 `0.0`。
- `group conservative-test`：按保守参数对 4 个电机一起发送短时命令；DM4310 使用零位置/零速度/低增益 MIT 命令，DMH6215 使用速度 `0.0`，不向速度模式电机发送 torque 帧。
- `imu read`：读取并打印 IMU 相关 CAN 帧。

## 3. Project Structure

建议保持当前仓库简单，新增控制相关代码时使用下面结构：

```text
control/
  __init__.py
  can_tool.py              # Python CLI 入口
  devices.py               # 设备表：CAN_ID、MST_ID、控制模式
  safety.py                # 限幅、duration、超时策略
  damiao_protocol.py       # 达妙电机/IMU 帧编码解码
  damiao_usbcan.py         # 达妙 USB-CAN 虚拟串口二进制协议封装

tests/
  test_damiao_protocol.py
  test_safety.py
  test_devices.py
```

结构原则：

- Python 负责用户入口、设备配置、参数校验、安全限幅和协议组织。
- `control/damiao_usbcan.py` 负责达妙 USB-CAN 虚拟串口二进制协议、CAN 帧读写和必要的超时处理。
- 设备 CAN_ID/MST_ID 不散落在命令实现里，统一放在设备表。
- 多电机联动命令应复用单电机协议编码和安全校验，不单独维护另一套参数规则。
- 协议编码解码应可脱离硬件进行单元测试。

## 4. Code Style

Python：

- 使用 Python 3.13+ 和 `uv`。
- CLI 参数保持显式，不自动猜测危险参数。
- 类型标注覆盖公共函数和协议编码解码函数。
- 协议编码、限幅、设备表应写成小函数或 dataclass，避免全局可变状态。
- 不为未来 ROS、仿真或复杂控制器预留抽象，等实际需要再加。

达妙 USB-CAN 虚拟串口：

- 默认串口波特率为 `921600`，CAN bitrate 为 `1000000`。
- 二进制协议封装参考达妙 ROS1 例程。
- 读写函数必须支持 timeout，不能无限阻塞。
- 对串口 open/config/read/write 的错误返回清晰异常或错误码。

协议与命名：

- 使用硬件配置中的设备名：`dm4310_left`、`dm4310_right`、`dmh6215_left`、`dmh6215_right`、`dm_imu`。
- CAN ID 使用整数常量或 dataclass 字段，显示为十六进制。
- 所有实机写命令都应通过同一个安全校验路径。

## 5. Testing Strategy

第一版测试分三层。

### 5.1 无硬件单元测试

必须覆盖：

- 达妙电机 enable/disable 帧编码。
- MIT 控制帧编码的边界值与限幅行为。
- 速度模式控制帧编码的边界值与限幅行为。
- 多电机 group 命令生成的帧数量、顺序、设备 ID 和保守参数。
- 设备表是否与 [CLAUDE.md](CLAUDE.md) 中 CAN 配置一致。
- duration、torque、position、velocity、kp、kd 等用户输入的安全校验。

运行命令：

```bash
uv run pytest
```

### 5.2 本机接口测试

无需接电机也可运行：

- `list-ports` 能列出 macOS 串口。
- `sniff` 对不存在串口给出明确错误。
- 达妙 USB-CAN 虚拟串口 open timeout/read timeout 行为可控。

### 5.3 实机冒烟测试

接入 USB-CAN 和 CAN 总线后运行：

1. `list-ports` 找到 USB-CAN 串口。
2. `sniff` 能看到总线帧或至少能稳定打开设备。
3. 对单个电机执行 `enable`，观察是否有响应帧。
4. 执行 `pair conservative-spin --wheel-velocity 0.2 --duration 0.5`，确认 0x04 按速度模式接收保守速度命令。
5. 执行 `group zero --duration 1.0`，确认 4 个电机可以一起接收零输出命令。
6. 执行 `group conservative-test --duration 0.5`，确认多电机联动路径可用但不使用激进参数。
7. 执行 `disable`，确认命令退出时不会保持非零输出。
8. `imu read --duration 5.0` 能打印 IMU 相关帧。

实机测试记录应包含：串口路径、CAN bitrate、执行命令、是否收到帧、异常现象。

## 6. Boundaries

### Always do

- 默认从零输出或 disable 状态开始。
- 非零力矩/运动命令必须设置 duration，命令结束后发送零力矩或 disable。
- 对 torque、position、velocity、kp、kd 做强制限幅；第一版使用保守默认值。
- 多电机一起控制时先执行 `group zero`，再执行短时保守测试，不直接进入较大输出。
- 所有串口/CAN 读写必须有 timeout。
- 实机命令打印即将发送的目标设备、CAN_ID、MST_ID、控制模式和限幅后参数。
- 协议编码先用无硬件单元测试验证，再实机发送。

### Ask first

- 增大默认力矩、速度、位置范围。
- 添加连续控制循环或高频闭环控制。
- 让多个电机同时输出超过 `group conservative-test` 默认范围的非零命令。
- 修改硬件 CAN_ID/MST_ID 配置。
- 引入 ROS、仿真联动、后台 daemon 或长期运行服务。

### Never do

- 不发送无限时长的非零力矩或运动命令。
- 不在异常退出时保持电机输出。
- 不在未完成单电机和 `group zero` 冒烟测试前发送多电机非零命令。
- 不绕过限幅直接发送用户输入。
- 不把 macOS 上不存在的 SocketCAN 当作默认实现。
- 不默认写入厂商固件参数、零点或持久配置。
- 不自动删除或重置用户现有仓库改动。

## Open Questions

- 达妙 USB-CAN 虚拟串口二进制协议封装以 ROS1 例程为参考，需要用实机确认完整收发细节。
- 达妙 IMU 帧格式是否已有官方文档或现成解析逻辑；若没有，第一版先打印原始帧。
