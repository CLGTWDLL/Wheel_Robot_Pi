本项目用于达妙电机和 DM-IMU-L1 的联合调试。主程序把电机反馈、IMU 姿态、状态机和平衡控制统一到一个 Rich 终端面板中，方便观察机器人当前运行状态。

## 运行方式

先安装基础依赖：

```bash
pip install -r MOTORS/requirements.txt
```

主入口是 `Contorl/main.py`，运行时启动这个文件即可。默认电机串口为 `COM3`，波特率为 `921600`；默认 IMU 串口为 `COM8`，波特率为 `921600`。如果端口不同，可以改代码

如果直接用命令行运行时出现模块找不到的问题，需要确认 `Contorl`、`MOTORS`、`IMU`、`Robor_State` 和 `Safe` 这几个目录已经加入 Python 的导入路径；在 IDE 中运行时也建议把项目根目录作为工作目录。

## MOTORS

`MOTORS/DM_CAN.py` 是达妙电机通信和控制相关的库文件，里面定义了 `Motor`、`MotorControl`、电机型号、控制模式和参数读写等底层接口。项目里其他电机代码基本都调用这里的函数。这个文件属于底层库函数，不建议随意修改。

`MOTORS/DM_Motor_Test copy.py` 是四电机联调测试脚本。它会创建两个 `DM4310` 和两个 `DMH6215` 电机对象，通过修改脚本里的目标位置、刚度、阻尼和力矩参数，可以同时控制四个电机运动。运行时会把电机位置、速度、力矩等参数通过 Rich 表格显示在终端。

`MOTORS/DM_Motor_Test.py` 是较早的双电机测试示例，主要演示位置速度模式和速度模式控制。现在四电机调试更建议看 `DM_Motor_Test copy.py`。

`MOTORS/motor_dashboard.py` 是旧版单独电机 Rich 面板。当前项目已经把电机和 IMU 显示合并到 `Contorl/robot_state.py`，新代码优先使用 `RobotDashboard`。

`MOTORS/requirements.txt` 记录项目依赖，主要包括 `numpy`、`pyserial` 和 `rich`。

## IMU

`IMU/imu_can.py` 负责解析 DM-IMU-L1 的 USB 主动输出帧，也保留了 CAN 数据帧解析接口。当前主程序主要使用 USB 串口读取 IMU 数据，并支持可选 CRC 校验和 IMU 输出内容配置。

`IMU/balance_signal.py` 把 IMU 原始状态转换成平衡控制需要的观测量。目前用于控制的是 pitch 和 roll 两个姿态角，以及对应角速度。轴向、零偏和方向符号都集中在 `BalanceSignalConfig` 中配置，后续如果 IMU 安装方向变化，可以优先在这里调整。

`IMU/run_imu_monitor.py` 是 IMU 单独监视脚本。它可以只读取 IMU 数据，把 pitch、roll、角速度显示到 Rich 面板，并可选择记录到 `IMU/logs` 下的 CSV 文件。

`IMU/logs` 存放 IMU 调试时生成的 CSV 日志，用于回看姿态和角速度数据。

## Contorl

`Contorl/main.py` 是主函数入口。它会初始化四个电机、打开电机串口、读取 IMU、运行状态机、执行安全检查，并把电机和 IMU 状态绘制到 Rich 表格中。主循环中还会持续读取键盘输入，用于切换机器人状态。

`Contorl/Balance.py` 是原地平衡控制代码。它根据 pitch 和 pitch 角速度计算轮电机力矩，并让两个轮电机输出相反方向的力矩，同时保留关节电机的目标角度。

`Contorl/Manage.py` 是运行时参数管理文件。主程序运行时会定期检查这个文件，修改并保存里面的平衡参数后，可以不用重启程序就更新到状态机里。常调的是Kp和Kd轮电机阻尼参数。

`Contorl/robot_state.py` 是统一状态和 Rich 面板代码。`RobotState` 保存电机和 IMU 的最新状态，`RobotDashboard` 同时绘制电机状态、IMU 状态、安全状态、循环次数和当前模式。

## Robor_State

`Robor_State/State.py` 是机器人状态机。目前包含四个主要状态：`IDLE`、`LEG_HOLD`、`FAULT` 和 `BALANCE`。

按键 `1` 切换到 `IDLE`，四个电机输出零命令，主要用于查看状态。

按键 `2` 切换到 `LEG_HOLD`，关节电机保持指定目标角度，轮电机保持零输出，适合做电机运动示例和关节保持测试。

按键 `3` 切换到 `FAULT`，程序进入停止保护流程。

按键 `4` 切换到 `BALANCE`，关节电机保持目标角度，轮电机根据 IMU 的 pitch 误差输出平衡力矩。

按键 `q`、`Esc` 或空格也会进入故障停止状态，方便调试时快速停机。

## Safe

`Safe/safe.py` 是电机安全保护模块。`SafetyMonitor` 负责判断是否触发故障，包括 IMU 超时、电机反馈超时、roll 角过大和力矩命令限幅等；`MotorSafetyActuator` 负责故障发生后的实际动作，会先发送零力矩命令，再逐个失能电机。

当前 pitch 角保护在代码中暂时被注释掉，roll 角保护仍然启用。调试平衡控制时，应先确认安全阈值和力矩限幅适合当前机械结构。

建议修改范围

日常调参优先修改 `Contorl/Manage.py`、`Contorl/Balance.py` 和 `Robor_State/State.py`。如果只是观察状态或改显示内容，可以修改 `Contorl/robot_state.py`。

不建议直接修改 `MOTORS/DM_CAN.py` 和 `MOTORS/DM_Motor_Test copy.py` 中来自底层库或验证过的通信逻辑，除非明确是在做电机协议适配或硬件接口变更。
