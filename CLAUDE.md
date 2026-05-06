# wheel_robot 项目说明

## 电机与 IMU CAN 配置

实际硬件已按此配置完成：PCB 通过 CAN 连接 4 个电机和 DM-IMU。

| 设备       | CAN_ID | Master/MST_ID | 控制模式     |
| ---------- | ------ | ------------- | ------------ |
| DM-IMU     | 0x01   | 0x11          | -            |
| DM4310 左  | 0x02   | 0x12          | MIT 控制模式 |
| DM4310 右  | 0x03   | 0x13          | MIT 控制模式 |
| DMH6215 左 | 0x04   | 0x14          | 速度控制模式 |
| DMH6215 右 | 0x05   | 0x15          | 速度控制模式 |

## 达妙电机与 CAN 调试规则

未来 coding agent 在本项目内编写、修改或运行达妙电机相关代码时，必须遵守以下规则。

### 默认通信配置

- 默认使用 CAN 标准 11-bit 数据帧和数据帧类型，不使用扩展帧或远程帧，除非硬件资料和现场人员明确要求。
- 实机通信默认使用达妙 USB 转 CAN 虚拟串口二进制协议，CLI backend 为 `dm_usbcan`。
- 电机默认 CAN bitrate 为 `1 Mbps`。达妙 USB-CAN 虚拟串口默认 `serial baud` 为 `921600`。项目代码不自动修改设备 CAN bitrate。
- 使用项目 CLI 时优先显式写出通信参数：`--backend dm_usbcan --port <port> --serial-baud 921600 --can-bitrate 1000000`。

### 上电、接线、调试顺序

1. 先确认机械固定、轮子离地或有可靠约束，并确认急停、断电、拔 CAN/电源的路径可立即执行。
2. 上电前确认线序：`CAN_H` 对 `CAN_H`、`CAN_L` 对 `CAN_L`，电源正负极正确；不要在接线不确定时发送任何实机命令。
3. 先离线验证，再连接硬件：`uv run pytest`、`uv run python -m compileall control tests`。
4. 串口发现：`uv run python -m control.can_tool list-ports`。
5. 只读 sniff：`uv run python -m control.can_tool sniff --backend dm_usbcan --port <port> --serial-baud 921600 --can-bitrate 1000000 --duration 1.0`。
6. 只读 IMU：`uv run python -m control.can_tool imu read --backend dm_usbcan --port <port> --serial-baud 921600 --can-bitrate 1000000 --duration 1.0`。
7. 单电机短暂 enable 后必须立即 disable：`uv run python -m control.can_tool motor enable --backend dm_usbcan --port <port> --serial-baud 921600 --can-bitrate 1000000 --motor dm4310_left --duration 0.1`。
8. 现场确认 DMH6215 当前实际为速度模式；不要对 0x04/0x05 发送 `motor torque` 测试。
9. 最后才允许保守 group 或 pair 测试：`uv run python -m control.can_tool pair conservative-spin --backend dm_usbcan --port <port> --serial-baud 921600 --can-bitrate 1000000 --duration 0.1 --period 0.05 --velocity 0.2 --wheel-velocity 0.2`，以及 `uv run python -m control.can_tool group zero --backend dm_usbcan --port <port> --serial-baud 921600 --can-bitrate 1000000`、`uv run python -m control.can_tool group conservative-test --backend dm_usbcan --port <port> --serial-baud 921600 --can-bitrate 1000000`。

### 只读与冒烟测试优先

- 不确定电机状态、接线、模式、ID、Master/MST_ID、供电或机械约束时，只允许执行离线测试、串口枚举、sniff、IMU 只读、短暂 enable/disable 冒烟测试。
- 不要在不确定状态下发送参数写入、参数保存、保存零点/设零点、校准、标定、固件升级、修改 CAN_ID、修改 Master/MST_ID、修改 CAN 波特率等命令。
- 达妙电机出厂通常已完成校准和标定；校准、标定、保存零点、固件升级都会带来运动或持久化风险，只能在明确任务和现场确认后执行。

### CAN 帧规则

- MIT 模式控制帧：`arbitration_id = CAN_ID`，8 字节按位打包 `p/v/kp/kd/t`。
- 速度模式控制帧：`arbitration_id = 0x200 + CAN_ID`，`float32 little-endian` 速度值放在 D0-D3，D4-D7 为 0。
- 力矩模式控制帧：本项目代码使用 `arbitration_id = 0x600 + CAN_ID`，`float32 little-endian` 力矩值放在 D0-D3，D4-D7 为 0。
- `enable`、`disable`、`zero/save-zero-position`、`clear-error` 都发到 `CAN_ID`，数据分别为 `FF FF FF FF FF FF FF FC/FD/FE/FB`。
- 参数读写/保存统一通过 `arbitration_id = 0x7FF`：D0/D1 为 `CAN_ID` little-endian，D2 为 `0x33` 读、`0x55` 写、`0xAA` 保存，D3 为 `RID`，D4-D7 为数据 little-endian；读命令 D4-D7 置 0。

### 反馈帧解析规则

- 电机反馈帧的 `arbitration_id` 是 Master/MST_ID，不是 CAN_ID。本项目按 `master_id` 路由反馈帧，并校验反馈中的 `motor_id`。
- D0 低 4 位是 `CAN_ID` 低位，高 4 位是状态/错误码：`0` 失能，`1` 使能，`8` 超压，`9` 欠压，`A` 过电流，`B` MOS 过温，`C` 线圈过温，`D` 通讯丢失，`E` 过载。
- D1-D2 为 position，D3-D4 为 velocity，D4-D5 为 torque，D6 为 MOS 温度，D7 为 rotor 温度；position 使用 16 位映射，velocity/torque 使用 12 位映射，映射范围必须与电机配置的 `P_MAX/V_MAX/T_MAX` 一致。

### 参数与模式规则

- 写寄存器会立即生效，但不保存则掉电丢失。
- 保存参数只能在失能状态下生效，会一次性写入当前全部参数到 flash；不要频繁执行保存参数命令。
- 控制模式寄存器 `RID=0x0A`：`1=MIT`、`2=位置速度`、`3=速度`、`4=力位混控`。通过 CAN 切换模式会清零位置、速度、MIT 扭矩前馈、`kp`、`kd` 等命令值；未保存则掉电丢失。
- 切换到位置类控制前，应尽量在零速状态下进行，并确认当前位置，避免位置突变造成冲击。
