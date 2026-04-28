# 规格说明：四个达妙电机的 USB-CAN 控制

## 目标

为两组左右轮腿结构中的四个达妙电机编写一个简单的 Python USB-CAN 控制器。运行时目标是同步控制四个电机并聚合反馈；单电机测试仅作为首次实机 bring-up 的安全步骤：

- `DM4310 左`：CAN_ID `0x02`，Master_ID `0x12`，MIT 模式，用作左侧腿部关节电机
- `DM4310 右`：CAN_ID `0x03`，Master_ID `0x13`，MIT 模式，用作右侧腿部关节电机
- `DMH6215 左`：CAN_ID `0x04`，Master_ID `0x14`，用作左侧轮毂电机
- `DMH6215 右`：CAN_ID `0x05`，Master_ID `0x15`，用作右侧轮毂电机

以上 CAN_ID、Master_ID、控制模式和 1 Mbps 波特率均已确认。

第一阶段交付一个保守的硬件控制原型，能够：

1. 通过达妙官方 USB 转 CAN 工具打开 CAN 总线。
2. 在 macOS 上用 Python 控制，当前也兼容你正在用 Windows 电脑进行的官方上位机测试。
3. 使用 CAN 2.0 标准帧，默认 1 Mbps。
4. 使能和失能四个电机。
5. 同步向两个 DM4310 发送 MIT 模式命令。
6. 同步向两个 DMH6215 发送适合作为轮毂电机的速度控制命令。
7. 聚合解析四个电机反馈帧，得到位置、速度、力矩、状态和温度。
8. 提供 dry-run 模式，在没有硬件的情况下测试报文格式。

目标用户是在本仓库中做轮足机器人调试、台架测试和后续控制算法接入的开发者。

成功状态：开发者可以在 macOS 上通过一条命令向四个已配置电机同步发送低风险命令，并观察聚合解码后的反馈；在没有硬件或不想真实发送时，可以通过 dry-run 看到将要发送的 CAN 帧。

## 技术栈

- 语言：Python `>=3.13`，与 [pyproject.toml](pyproject.toml) 保持一致。
- 包管理：`uv`。
- CAN 抽象库：`python-can`，用户已允许添加。
- 达妙官方 USB-CAN 适配层：`python-can-damiao`，用户已允许添加。
  - 这是社区插件，不是达妙官方 SDK。
  - 插件把达妙官方 USB-CAN 工具注册成 `python-can` 的 `damiao` interface。
  - 插件底层通过串口协议访问 USB-CAN 工具，依赖 `pyserial`。
- 保留现有依赖：`mujoco>=3.7.0`。

建议依赖安装命令：

```bash
uv add python-can python-can-damiao
```

## USB-CAN 接入方案

### 当前确认

- 硬件：达妙官方 USB 转 CAN 工具。
- 目标控制系统：macOS。
- 当前测试系统：Windows 官方上位机/工具已确认硬件、ID、模式、1 Mbps 波特率和接线。
- Python 接入：优先使用 `python-can` + `python-can-damiao`。

### macOS 下的访问方式

macOS 没有 Linux SocketCAN，因此不能把达妙官方 USB-CAN 当作 `can0` 使用。更合适的方案是把它当作 USB 串口设备，由 `python-can-damiao` 封装成 `python-can` 总线。

推荐用法：

```python
import can

bus = can.Bus(
    interface="damiao",
    channel="/dev/tty.usbmodemXXXX",
    bitrate=1000000,
)
```

其中：

- `interface="damiao"` 来自 `python-can-damiao`。
- `channel` 在 macOS 上应是实际串口设备路径，例如 `/dev/tty.usbmodemXXXX` 或 `/dev/tty.usbserialXXXX`。
- `bitrate=1000000` 对应达妙电机文档中的默认 1 Mbps CAN 波特率。
- 插件内部串口波特率默认是 `2000000`，通常不需要暴露给 CLI 用户。

可用以下方式辅助确认设备：

```bash
python -m serial.tools.list_ports -v
system_profiler SPUSBDataType
```

### 达妙 USB-CAN 工具的上位机侧协议

根据 `python-can-damiao` 的公开实现，主机到 USB-CAN 工具不是 SocketCAN 协议，而是串口上的私有帧协议：

- 设置 CAN 波特率命令：`55 05 <bitrate_index> AA 55`。
- 发送 CAN 帧时，串口侧封装为 30 字节帧：
  - 帧头：`55 AA`
  - 长度：`1E`
  - 命令：`03`
  - 包含发送次数、间隔、标准/扩展帧标志、CAN ID、帧类型、DLC、8 字节 CAN 数据和 CRC。
- 接收 CAN 帧时，串口侧封装为 16 字节帧：
  - 帧头：`AA`
  - 命令：`11` 表示接收到 CAN 帧，`12` 表示发送成功确认
  - 包含 info、CAN ID、8 字节数据
  - 帧尾：`55`

本项目不直接重写这层 USB-CAN 私有协议；优先通过 `python-can-damiao` 调用。项目内只实现达妙电机 CAN 报文的打包/解析。

### Windows 当前测试建议

Windows 阶段已用达妙官方上位机确认：四个电机的 CAN_ID、Master_ID、控制模式、1 Mbps 波特率、硬件接线和反馈读取与本规格一致。

后续在 macOS 上用 Python dry-run 和真实总线测试复现。首次物理验证可以先逐个电机安全 bring-up；最终运行时控制模型仍是四电机同步控制。

## 控制模式建议

### DM4310 腿部关节电机

推荐使用 MIT 模式。

原因：腿部关节通常需要位置、速度、刚度、阻尼和前馈力矩一起控制。MIT 模式的五元组 `p_des`、`v_des`、`kp`、`kd`、`t_ff` 更适合做关节阻抗控制、站立姿态控制和后续 LQR/全身控制接入。

第一阶段默认命令必须保守：

- `position = 0`
- `velocity = 0`
- `kp = 0`
- `kd = 0`
- `torque = 0`

后续台架确认机械零点和方向后，再逐步加入 `kp/kd` 或小幅前馈力矩。

### DMH6215 轮毂电机

第一阶段确定使用速度模式，而不是默认直接用力矩控制。

原因：DMH6215 在轮足机器人里作为轮子时，最先需要稳定、可控、低风险地验证轮子转向、速度反馈和左右轮方向。速度模式由电机内部控制速度闭环，调试风险低，适合：

- 台架验证左右轮方向。
- 验证轮速反馈和里程计方向。
- 做低速行走/滚动测试。
- 在 LQR 或上层控制尚未闭环前避免空载力矩导致轮子快速加速。

速度模式发送规则来自文档：

- 帧 ID：`0x200 + CAN_ID`
- Data[0:4]：目标速度 `v_des` 的 32 位浮点数，小端序
- Data[4:8]：全 0

例如：

- 左轮 DMH6215：CAN_ID `0x04`，速度模式帧 ID 为 `0x204`
- 右轮 DMH6215：CAN_ID `0x05`，速度模式帧 ID 为 `0x205`

第二阶段如果要做更高性能的平衡/动态控制，再考虑 MIT 模式下的力矩前馈 `t_ff`：

- `kp = 0`
- `kd = 0`
- `p_des = 0`
- `v_des = 0`
- 只给小幅 `t_ff`

但这必须加安全限制：力矩限幅、斜坡限制、超时失能、急停命令和单轮台架测试。空载直接给力矩会让轮子快速加速，不作为默认方案。

## 反馈获取与“电机转起来”的判定

### 反馈来源

达妙电机的反馈不是连续广播，而是问询式反馈：驱动器收到与自身 CAN_ID 匹配的控制帧后，会向总线发送当前状态。反馈帧的仲裁 ID 由电机配置里的 Master_ID 决定。

本项目四个反馈 ID：

- `DM4310 左`：Master_ID `0x12`
- `DM4310 右`：Master_ID `0x13`
- `DMH6215 左`：Master_ID `0x14`
- `DMH6215 右`：Master_ID `0x15`

因此接收逻辑必须按 `msg.arbitration_id == motor.master_id` 区分电机，不能只看 payload 里的低 4 位 ID。

### python-can-damiao 接收路径

`python-can-damiao` 源码里，USB-CAN 工具通过串口返回 16 字节私有帧：

- `cmd == 0x11`：真实 CAN 接收帧，会被转换成 `can.Message` 并从 `bus.recv()` 返回。
- `cmd == 0x12`：发送成功确认，会被插件忽略，不会作为 `bus.recv()` 的业务消息返回。

所以实现里可以使用标准 `python-can` 接口：

```python
msg = bus.recv(timeout=0.2)
if msg is not None and msg.arbitration_id in master_id_to_motor:
    feedback = decode_feedback(msg.data, motor_config)
```

注意：发送确认不等于电机反馈。只有收到对应 Master_ID 的反馈帧，才能证明电机驱动器实际响应了 CAN 控制帧。

### 反馈帧解析

反馈 payload 固定为 8 字节：

```text
Data[0]      = ID | ERR << 4
Data[1:3]    = POS，16 位无符号映射值
Data[3:5]    = VEL，12 位无符号映射值
Data[4:6]    = T，12 位无符号映射值
Data[6]      = MOS 温度
Data[7]      = Rotor 温度
```

解析规则：

```python
motor_id = data[0] & 0x0F
state = data[0] >> 4
p_int = (data[1] << 8) | data[2]
v_int = (data[3] << 4) | (data[4] >> 4)
t_int = ((data[4] & 0x0F) << 8) | data[5]
```

再用 `uint_to_float()` 按每个电机配置的 `p_max`、`v_max`、`t_max` 映射回物理量。

状态码：

- `0`：失能
- `1`：使能
- `8`：超压
- `9`：欠压
- `A`：过电流
- `B`：MOS 过温
- `C`：电机线圈过温
- `D`：通讯丢失
- `E`：过载

### 成功判定

agent 不能只凭“命令已发送”判断电机成功转起来。必须满足以下反馈证据：

1. 发送控制帧后，在超时时间内收到目标电机 Master_ID 的反馈帧。
2. `state == 1`，表示电机处于使能状态。
3. 对 DMH6215 轮毂电机发送小幅速度命令后，反馈 `velocity` 的符号与命令方向一致。
4. 反馈速度绝对值超过低速阈值，例如 `abs(velocity) > 0.05 rad/s`，才判定为“已转动”。
5. 温度字段在合理范围内，且状态码不是故障码。

第一阶段建议新增同步四电机 `smoke-test`，并保留单电机 bring-up 入口作为安全验证步骤：

- 默认同步测试四个电机，只发送 DM4310 MIT 零命令和 DMH6215 速度 0 命令。
- 首次物理验证可先逐个电机测试使能、零命令和反馈读取。
- 任何非零速度、非零位置或非零力矩命令都必须显式批准，并可按单电机、低速、短时长 staged validation。
- 连续采集并聚合反馈，打印 `timestamp`、`master_id`、`state`、`position`、`velocity`、`torque`、`temperature`。
- 最后无条件发送失能。

### agent 自动化边界

可以让 agents 完成：

- 协议打包/解包实现。
- dry-run 验证。
- 串口设备枚举。
- `bus.recv()` 接收与反馈解码。
- 硬件测试脚本编排。
- 根据反馈判断是否成功使能、是否转动、是否故障。

但真实电机首次转动仍需要你在现场确认：

- 电机固定可靠。
- 轮子悬空或机器人处于安全支撑状态。
- 电源、电流限制、急停可用。
- 左右轮方向不会造成机器人突然运动。

未经你明确批准，agents 不应自动发送非零速度、非零位置或非零力矩命令。

## 相关文档来源

- [docs/DM-J4310-2EC V1.2减速电机说明书V1.2定稿.md](docs/DM-J4310-2EC%20V1.2减速电机说明书V1.2定稿.md)
  - CAN 标准帧、默认 1 Mbps：[405-412 行](docs/DM-J4310-2EC%20V1.2减速电机说明书V1.2定稿.md#L405-L412)
  - 反馈帧和 Master_ID：[421-467 行](docs/DM-J4310-2EC%20V1.2减速电机说明书V1.2定稿.md#L421-L467)
  - MIT 控制帧打包：[468-510 行](docs/DM-J4310-2EC%20V1.2减速电机说明书V1.2定稿.md#L468-L510)
  - 使能、失能、保存零点、清错命令帧：[560-641 行](docs/DM-J4310-2EC%20V1.2减速电机说明书V1.2定稿.md#L560-L641)
  - 参数读取、写入、保存和模式寄存器：[642-763 行](docs/DM-J4310-2EC%20V1.2减速电机说明书V1.2定稿.md#L642-L763)
- [docs/DM-H6215轮毂电机使用教程V2.md](docs/DM-H6215轮毂电机使用教程V2.md)
  - CAN 标准帧、固定/默认 1 Mbps：[549-554 行](docs/DM-H6215轮毂电机使用教程V2.md#L549-L554)
  - 反馈帧解析：[555-650 行](docs/DM-H6215轮毂电机使用教程V2.md#L555-L650)
  - 使能、失能、保存零点、清错命令帧：[700-760 行](docs/DM-H6215轮毂电机使用教程V2.md#L700-L760)
  - MIT 控制帧打包：[776-845 行](docs/DM-H6215轮毂电机使用教程V2.md#L776-L845)
  - 位置速度模式和速度模式：[846-909 行](docs/DM-H6215轮毂电机使用教程V2.md#L846-L909)
  - 参数读取、写入、保存：[1018-1094 行](docs/DM-H6215轮毂电机使用教程V2.md#L1018-L1094)

外部资料：

- 达妙官方工具仓库：`https://gitee.com/kit-miao/dm-tools`
- 达妙官方资料仓库：`https://gitee.com/kit-miao/damiao`
- 社区 `python-can-damiao` 插件：`https://github.com/gaoyichuan/python-can-damiao`
- 社区达妙 Python 控制库：`https://gitee.com/chenmo1/DM_Control_Python`

## 命令

当前仓库已有命令：

```bash
uv run pytest
uv run python launch_mujoco.py
uv run python mujoco_mesh_preprocess.py
```

实现后建议提供的控制命令：

```bash
# 添加依赖。
uv add python-can python-can-damiao

# 显示已配置电机和将要发送的 CAN 帧；不需要 CAN 硬件。
uv run python -m damiao_can.cli --dry-run status

# 列出 macOS 串口设备，辅助找到达妙 USB-CAN 的 channel。
uv run python -m serial.tools.list_ports -v

# 使能四个电机。
uv run python -m damiao_can.cli --channel /dev/tty.usbmodemXXXX --bitrate 1000000 enable-all

# 失能四个电机。
uv run python -m damiao_can.cli --channel /dev/tty.usbmodemXXXX --bitrate 1000000 disable-all

# 向四个电机同步发送安全零命令并聚合反馈。
uv run python -m damiao_can.cli --channel /dev/tty.usbmodemXXXX --bitrate 1000000 command-all --dm4310-zero --dmh6215-velocity 0

# 首次 bring-up 时向单个 DM4310 发送一次安全的 MIT 零命令。
uv run python -m damiao_can.cli --channel /dev/tty.usbmodemXXXX --bitrate 1000000 mit --motor dm4310_left --position 0 --velocity 0 --kp 0 --kd 0 --torque 0

# 首次 bring-up 时向单个 DMH6215 发送低速轮速命令；非零命令需要批准。
uv run python -m damiao_can.cli --channel /dev/tty.usbmodemXXXX --bitrate 1000000 velocity --motor dmh6215_left --velocity 0.5 --allow-motion

# 执行保守的四电机冒烟测试：使能、同步发送零命令、聚合读取反馈、失能。
uv run python -m damiao_can.cli --channel /dev/tty.usbmodemXXXX --bitrate 1000000 smoke-test
```

## 项目结构

建议新增：

```text
damiao_can/
  __init__.py              -> 包导出
  protocol.py              -> CAN 帧打包、解包、浮点/整数映射
  motors.py                -> 四个电机的配置和安全限制
  bus.py                   -> python-can 总线创建、发送和接收辅助函数
  cli.py                   -> 命令行入口

tests/
  test_damiao_protocol.py  -> 协议打包/解包的确定性测试
  test_damiao_cli.py       -> dry-run CLI 行为测试

docs/
  现有说明书仍作为协议细节的权威来源
```

除非用户明确要求把仿真和硬件控制集成起来，否则本功能不重构现有 MuJoCo 文件。

## 代码风格

协议逻辑使用小型带类型的 Python dataclass 和纯函数实现，这样报文格式可以在没有硬件的情况下测试。

示例风格：

```python
from dataclasses import dataclass

@dataclass(frozen=True)
class MotorConfig:
    name: str
    can_id: int
    master_id: int
    p_max: float = 12.5
    v_max: float = 45.0
    t_max: float = 10.0


def float_to_uint(value: float, minimum: float, maximum: float, bits: int) -> int:
    value = min(max(value, minimum), maximum)
    span = maximum - minimum
    return int((value - minimum) * ((1 << bits) - 1) / span)
```

约定：

- 协议打包保持为纯函数。
- 硬件 I/O 隔离在一个小的 bus 封装里。
- 电机名称显式使用：`dm4310_left`、`dm4310_right`、`dmh6215_left`、`dmh6215_right`。
- 打包前将命令值限制在配置的安全范围内。
- 默认命令值必须是零力矩、零速度、零增益，除非用户显式传入。
- DMH6215 轮毂电机第一阶段默认使用速度模式。
- 在手动台架验证前，不加入后台循环控制。

## 测试策略

- 测试框架：`pytest`。
- 协议单元测试不依赖 CAN 硬件。
- dry-run CLI 测试不依赖 CAN 硬件。
- 硬件冒烟测试必须是手动/显式启用，默认跳过。

实现完成前必须通过：

```bash
uv run pytest tests/test_damiao_protocol.py tests/test_damiao_cli.py
uv run pytest
```

协议测试应覆盖：

- 使能帧为 `FF FF FF FF FF FF FF FC`。
- 失能帧为 `FF FF FF FF FF FF FF FD`。
- 清错帧为 `FF FF FF FF FF FF FF FB`。
- MIT 零命令可以确定性打包。
- 速度模式帧 ID 为 `0x200 + CAN_ID`。
- 速度模式 payload 使用目标速度的 32 位小端浮点数，后 4 字节为 0。
- 反馈帧可解析出 `id = data[0] & 0x0F` 和 `state = data[0] >> 4`。
- 反馈接收按 Master_ID 的仲裁 ID 匹配目标电机。
- 反馈速度、位置、力矩按 16/12/12 位映射回浮点值。
- 硬件成功判定必须基于目标 Master_ID 的反馈帧，而不是发送成功确认。
- 参数读取/写入/保存帧使用仲裁 ID `0x7FF`，命令字节为 `0x33`、`0x55`、`0xAA`。

手动硬件验证仅在批准后进行：

1. Windows 官方上位机已确认四个电机 ID、模式、反馈和接线。
2. 检查电机电源和 CAN_H/CAN_L 接线。
3. CAN 波特率已确认为 1 Mbps。
4. macOS 上运行 `serial.tools.list_ports` 找到 USB-CAN 串口路径。
5. 先运行 dry-run status，检查帧 ID。
6. 如需首次安全 bring-up，可先逐个电机使能、发送零命令并读取反馈。
7. 默认物理验证使用四电机同步零命令并聚合读取反馈。
8. 任何小幅非零轮速命令都必须在明确批准后执行。
9. 在配置的 Master_ID 上读取反馈。
10. 结束前失能所有电机。

## 边界

- 总是：
  - 真实硬件发送前先生成 dry-run 帧。
  - 使用 CAN 标准帧、8 字节数据、默认 1 Mbps，除非用户明确修改。
  - macOS 上默认使用达妙 USB-CAN 串口路径，不使用 Linux-only 的 `can0`/SocketCAN 假设。
  - 提供清晰的 `disable-all` 命令。
  - 集中、可见地维护电机配置。
  - 将空载力矩命令视为危险操作。
  - 声称完成前运行测试。

- 先询问：
  - 添加 `python-can-damiao` 之外的任何新依赖。
  - 向已上电电机发送任何真实命令。
  - 使用 `0xAA` 命令保存参数到电机 flash。
  - 修改电机 CAN_ID、Master_ID、控制模式、波特率、P_MAX、V_MAX 或 T_MAX。
  - 将 DMH6215 从速度模式改为 MIT 力矩控制作为默认模式。
  - 将该控制器与 MuJoCo 或 LQR 代码集成。

- 永不：
  - 永不在默认冒烟测试中发送非零力矩或非零位置命令。
  - 永不在 DMH6215 默认冒烟测试中发送高速轮速命令。
  - 永不反复保存电机参数；flash 擦写寿命有限。
  - 永不在多设备总线上做常规调试时修改 CAN 波特率。
  - 永不假设 DM4310 和 DMH6215 在 CAN 帧格式之外拥有相同的安全机械限制。
  - 永不因为本功能删除或覆盖现有机器人描述、网格或仿真文件。

## 建议控制方案

推荐第一阶段实现路径：

1. 实现纯协议打包和解析。
   - 覆盖命令帧、MIT 帧、速度模式帧、参数帧和反馈解析。
   - 可以完全在无硬件条件下测试。

2. 实现静态四电机配置。
   - 使用图中的 ID：`0x02/0x12`、`0x03/0x13`、`0x04/0x14`、`0x05/0x15`。

3. 实现 dry-run CLI。
   - 第一版可见输出应展示仲裁 ID 和 payload 字节。

4. 添加 CAN 总线适配层。
   - 使用 `python-can` + `python-can-damiao`。
   - CLI 参数使用 macOS 串口路径作为 `--channel`。

5. 添加保守的硬件命令。
   - `enable-all`、`disable-all`、`clear-error`、`command-all`、`smoke-test`、用于 bring-up 的 DM4310 单次 `mit` 和 DMH6215 单次 `velocity`。

6. 实现四电机同步命令执行和按 Master_ID 聚合反馈，作为运行时控制模型。

7. 暂缓闭环步态/LQR 集成。
   - 先证明单帧控制和反馈解析是安全、正确的。

8. 第二阶段再评估 DMH6215 力矩控制。
   - 如果后续 LQR/平衡控制确实需要轮端力矩输入，再加入 MIT `t_ff` 控制。
   - 加入前必须先实现力矩限幅、命令斜坡、超时保护、急停和单轮台架测试流程。

## 成功标准

- `uv run pytest` 通过。
- dry-run 模式为四个已配置电机打印正确 CAN 帧。
- macOS 上可以枚举并打开达妙官方 USB-CAN 串口设备。
- 在用户确认硬件接线并批准真实发送后，实现可以通过达妙官方 USB-CAN 工具使能和失能已配置电机。
- 实现可以向两个 DM4310 同步发送 MIT 模式零命令。
- 实现可以向两个 DMH6215 同步发送速度 0，批准后发送小幅速度命令。
- 实现可以解析并聚合四个 Master_ID 上的反馈帧。
- 实现可以用反馈 `state`、`velocity`、故障码和温度判断四电机是否真实响应；单电机判定仅用于 bring-up。
- `smoke-test` 结束前无论成功或失败都发送失能命令。
- 除非用户明确请求，否则不会发生任何电机参数 flash 写入。

## 待确认问题

1. macOS 上达妙 USB-CAN 工具显示的串口路径是什么？可以用 `python -m serial.tools.list_ports -v` 查看。
2. 如果批准 `--allow-motion`，DMH6215 第一阶段安全轮速上限应设为多少？
3. `clear-error` 应只支持全电机、只支持单电机，还是两者都支持？
