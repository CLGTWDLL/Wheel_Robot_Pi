# wheel_robot 项目说明

## 电机与 IMU CAN 配置

实际硬件已按此配置完成：PCB 通过 CAN 连接 4 个电机和 DM-IMU。

| 设备 | CAN_ID | Master/MST_ID | 控制模式 |
| --- | --- | --- | --- |
| DM-IMU | 0x01 | 0x11 | - |
| DM4310 左 | 0x02 | 0x12 | MIT 控制模式 |
| DM4310 右 | 0x03 | 0x13 | MIT 控制模式 |
| DMH6215 左 | 0x04 | 0x14 | 力矩控制模式 |
| DMH6215 右 | 0x05 | 0x15 | 力矩控制模式 |
