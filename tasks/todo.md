# TODO: macOS USB-CAN / 达妙电机调通

## Phase 1: Offline Foundation

- [x] Task 1: 对齐设备表与硬件配置
  - [x] 添加/确认 `dm_imu`
  - [x] 确认 DM4310 左右为 MIT
  - [x] 修正 DMH6215 左右为 velocity
  - [x] 添加设备表测试
  - [x] 运行 `uv run pytest tests/test_devices.py`

- [x] Task 2: 补齐协议 command builders
  - [x] 复用 enable/disable/MIT 编码
  - [x] 使用 DMH6215 velocity frame
  - [x] 增加 group zero frame builder
  - [x] 测试 group zero 不产生 `0xFE`
  - [x] 运行 `uv run pytest tests/test_damiao_protocol.py`

- [x] Task 3: 建立安全限幅与 duration 策略
  - [x] 实现 torque/duration/MIT 参数校验
  - [x] 拒绝 NaN/inf
  - [x] 限制 torque-mode 命令目标
  - [x] 集中定义 group conservative 参数
  - [x] 运行 `uv run pytest tests/test_safety.py`

- [x] Task 4: 增加 FakeCanBus 与命令执行器
  - [x] FakeCanBus 记录发送帧
  - [x] 支持 timeout 和注入反馈
  - [x] 非零命令 `try/finally` 发送零输出
  - [x] fake 跑通 group zero/conservative-test
  - [x] 运行 `uv run pytest tests/test_fake_bus.py tests/test_group_commands.py`

- [x] Checkpoint A: Offline Safety Ready
  - [x] `uv run pytest` 通过
  - [x] 设备表与 `CLAUDE.md` 一致
  - [x] `group zero` 确认不是设零点
  - [x] conservative 参数已硬限制

## Phase 2: Local macOS Damiao USB-CAN Backend

- [x] Task 5: 实现 macOS 达妙 USB-CAN 虚拟串口 backend（离线实现）
  - [x] Python 主入口通过 `control/damiao_usbcan.py`
  - [x] 默认串口波特率 `921600`，CAN bitrate `1000000`
  - [x] open/read/write/close 有错误处理和 timeout
  - [x] 暴露统一 Python bus 接口
  - [ ] 实机确认达妙二进制协议收发细节（参考 ROS1 例程）
  - [x] 运行相关 USB-CAN codec/CLI 测试

- [x] Task 6: 实现 list-ports 与 sniff CLI
  - [x] `list-ports` 列出 `/dev/cu.*` 和 `/dev/tty.*`
  - [x] `sniff` 支持 port/serial-baud/can-bitrate/duration
  - [x] 不存在串口时错误清楚
  - [x] 运行 `uv run pytest tests/test_cli.py`

- [ ] Checkpoint B: Local Serial Ready
  - [ ] 找到 USB-CAN 串口
  - [x] nonexistent port 不 hang
  - [ ] 确认达妙二进制协议收发细节
  - [x] 未发送任何非零电机输出

## Phase 3: Single Device Hardware Slice

- [x] Task 7: 单电机 enable/disable 离线实现
  - [x] fake bus 验证 enable 后自动 disable frame
  - [ ] 实机单电机 enable 后立即 disable
  - [ ] 记录反馈或 timeout

- [x] Task 8: DMH6215 速度模式保守测试（离线/fake 验证）
  - [x] 只允许 `dmh6215_left/right`
  - [x] 使用速度模式零输出或保守小速度，不发送 torque 帧
  - [x] duration 不超过 `0.5 s`
  - [x] 异常路径发送零输出
  - [ ] 实机先跑 `--wheel-velocity 0.2 --duration 0.3`

- [x] Task 9: IMU raw read（离线/fake 验证）
  - [x] duration 到时退出
  - [x] 打印 raw CAN frame
  - [x] 不发送电机控制帧

- [ ] Checkpoint C: Single Device Hardware Ready
  - [ ] sniff 稳定
  - [ ] enable/disable 可用
  - [ ] 单 DMH6215 速度模式安全退出
  - [ ] IMU read 能运行到 duration 结束

## Phase 4: Four-Motor Vertical Slice

- [x] Task 10: group zero（离线/fake 验证）
  - [x] 固定顺序发送 4 台电机零输出
  - [x] 不使用 `0xFE`
  - [x] 不发送参数写入/保存
  - [x] fake 验证
  - [ ] 实机验证

- [ ] Checkpoint D: Group Zero Passed
  - [ ] 电机处于安全支撑状态
  - [ ] 急停或断电手段可用
  - [ ] group zero 无异常动作
  - [ ] 已记录 port、baud、bitrate、反馈情况

- [x] Task 11: group conservative-test（离线/fake 验证）
  - [x] 内置 pre-zero
  - [x] 默认 duration 不超过 `0.5 s`
  - [x] DM4310 不默认拉绝对 0 位
  - [x] DMH6215 不发送 torque 帧
  - [x] 结束/异常都发送 group zero
  - [x] 不自动提高参数

- [ ] Checkpoint E: Four-Motor Conservative Slice Passed
  - [ ] 4 台电机同 batch 发送命令
  - [ ] 测试安全退出
  - [ ] 未写参数、未设零点、未使用激进参数

## Phase 5: Documentation and Runbook

- [x] Task 12: 补充硬件调试 runbook 与日志模板
  - [x] 写明执行顺序
  - [x] 写明 `group zero` 不是设零点
  - [x] 写明提高参数需要人工确认
  - [x] 提供测试日志模板
