import math
from DM_CAN import *
# from motor_dashboard import MotorDashboard
from pathlib import Path
import sys
import serial
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot_state import RobotDashboard, RobotState


Motor1=Motor(DM_Motor_Type.DM4310,0x01,0x11)
Motor2=Motor(DM_Motor_Type.DM4310,0x02,0x12)
Motor3 = Motor(DM_Motor_Type.DMH6215,0x03,0x13)
Motor4 = Motor(DM_Motor_Type.DMH6215,0x04,0x14)
serial_device = serial.Serial('COM3', 921600, timeout=0.5)

MotorControl1=MotorControl(serial_device)
MotorControl1.change_limit_param(DM_Motor_Type.DM4310, 0.25, 30, 10)

MotorControl1.addMotor(Motor1)
MotorControl1.addMotor(Motor2)
MotorControl1.addMotor(Motor3)
MotorControl1.addMotor(Motor4)

MotorControl1.switchControlMode(Motor1, Control_Type.MIT)
MotorControl1.switchControlMode(Motor2, Control_Type.MIT)
MotorControl1.switchControlMode(Motor3, Control_Type.MIT)
MotorControl1.switchControlMode(Motor4, Control_Type.MIT)

print("sub_ver:",MotorControl1.read_motor_param(Motor1,DM_variable.sub_ver))
print("Gr:",MotorControl1.read_motor_param(Motor1,DM_variable.Gr))

# if MotorControl1.change_motor_param(Motor1,DM_variable.KP_APR,54):
#     print("write success")
print("PMAX:",MotorControl1.read_motor_param(Motor1,DM_variable.PMAX))
# print("MST_ID:",MotorControl1.read_motor_param(Motor1,DM_variable.MST_ID))
# print("VMAX:",MotorControl1.read_motor_param(Motor1,DM_variable.VMAX))
# print("TMAX:",MotorControl1.read_motor_param(Motor1,DM_variable.TMAX))
print("Motor2:")
print("PMAX:",MotorControl1.read_motor_param(Motor2,DM_variable.PMAX))
# print("MST_ID:",MotorControl1.read_motor_param(Motor2,DM_variable.MST_ID))
# # print("VMAX:",MotorControl1.read_motor_param(Motor2,DM_variable.VMAX))
# # print("TMAX:",MotorControl1.read_motor_param(Motor2,DM_variable.TMAX))
# MotorControl1.enable(Motor3)

MotorControl1.save_motor_param(Motor1)
MotorControl1.save_motor_param(Motor2)
MotorControl1.save_motor_param(Motor3)
MotorControl1.save_motor_param(Motor4)

MotorControl1.enable(Motor1)
MotorControl1.enable(Motor2)
MotorControl1.enable(Motor3)
MotorControl1.enable(Motor4)
i=0
motors = [Motor1, Motor2, Motor3, Motor4]
robot_state = RobotState()
with RobotDashboard(robot_state, refresh_per_second=10) as dashboard:
    while i<10000:
        # q=0.2 * math.sin(time.time())
        q = 0.2
        i=i+1
        # MotorControl1.control_pos_force(Motor1, 10, 1000,100)
        # MotorControl1.control_Vel(Motor1, q*5)
        MotorControl1.controlMIT(Motor1, 6, 0.5, q, 0, 0)
        MotorControl1.controlMIT(Motor2, 6, 0.5, -q, 0, 0)
        MotorControl1.controlMIT(Motor3, 0, 0.5, q, 0, 0.3)
        MotorControl1.controlMIT(Motor4, 0, 0.5, q, 0, -0.3)
        robot_state.update_motors_from_dm(motors)

        if i % 50 == 0:
            dashboard.update(loop_count=i, target_position=q)

        time.sleep(0.001)
        # MotorControl1.control(Motor3, 50, 0.3, q, 0, 0)

#语句结束关闭串口
serial_device.close()
