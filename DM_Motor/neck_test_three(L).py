import time
import numpy as np
from DM_CAN import *
from linear import *
from import_math import *
def setup_robot_and_motors():
    """
    初始化所有硬件和参数，并返回它们。
    """
    print("初始化机器人和电机")
    # 1. 创建电机对象
    Motor1 = Motor(DM_Motor_Type.DM4340, 0x01, 0x11)
    Motor2 = Motor(DM_Motor_Type.DM4340, 0x02, 0x12)
    Motor3 = Motor(DM_Motor_Type.DM4340, 0x03, 0x13)
    motors = [Motor1, Motor2, Motor3]

    # 2. 创建机器人模型
    robot = NeckMechanism(
        l_crank=13.5,
        l_rod=118,
        w_platform=22.41,
        h_platform=24.23,
        d_platform=24.05,
        w_motor=35.58,
        d_motor=24.05,
        z_motor_offset=-97.57
    )

    # 3. 初始化通信并启动电机
    ctrl, serial_dev = init_motors('/dev/ttyACM0', motors)
    enable_motors(ctrl, motors)

    # 4. 定义控制参数
    kp_list = [13, 13, 13]
    kd_list = [1, 1, 1]
    dt_val = 0.001
    print("完成初始化")

    #5. 移动到零位姿
    zero_targets = [0, 2.64, 2.85]
    print(f"移动到零位姿 ")
    move_to_target_linear(
        ctrl, motors, zero_targets, kp_list, kd_list,
        desired_velocity=0.5,
        dt=dt_val
       )
    
    return robot, ctrl, serial_dev, motors, kp_list, kd_list, dt_val
    
def run_main_task(robot, ctrl, motors, kp_list, kd_list, dt_val):
    """
    执行主要的机器人运动任务。
    """
    try:
        # 任务2: 循环执行姿态控制（在这里添加人脸追踪）
        print("进入循环姿态控制")
        while True:
            my_pose_command = [0, 7, 0] # [yaw,pitch,roll]  pitch-angL, roll-angR
            set_neck_pose(robot, ctrl, motors, kp_list, kd_list, dt_val, my_pose_command)
            time.sleep(0.1)
    except KeyboardInterrupt:
        # 按下Ctrl+C，退出循环
        print("捕获到 Ctrl+C,退出姿态控制")
        pass

def cleanup_and_exit(ctrl, motors, serial_dev):
    """
    安全地禁用电机和关闭串口。
    """
    print("\n失能电机并关闭串口")
    if ctrl and motors:
        disable_all(ctrl, motors)
    if serial_dev and serial_dev.is_open:
        serial_dev.close()

if __name__ == "__main__":
    # 初始化工作
    robot, ctrl, serial_dev, motors, kp, kd, dt = setup_robot_and_motors()
    # 执行主要任务
    run_main_task(robot, ctrl, motors, kp, kd, dt)
    # 调用清理函数
    cleanup_and_exit(ctrl, motors, serial_dev)
