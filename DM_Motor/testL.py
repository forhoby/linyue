import time
import numpy as np
from DM_CAN import *
from linear import *
from import_math import *
# ==============================================================================
# <<< 主程序 >>>
# ==============================================================================

if __name__ == "__main__":
    Motor1 = Motor(DM_Motor_Type.DM4340, 0x01, 0x11)
    Motor2 = Motor(DM_Motor_Type.DM4340, 0x02, 0x12)
    Motor3 = Motor(DM_Motor_Type.DM4340, 0x03, 0x13)
    motors = [Motor1, Motor2, Motor3]

    robot = NeckMechanism(
    l_crank=13.5,       # 舵机臂长 (假设)
    l_rod=118,         # 连杆长 (假设)
    w_platform=22.41,    # 平台上连接点离中心线的宽度
    h_platform=24.23,   # 平台初始高度 (万向节中心到上面的连杆点)
    d_platform=24.05,    # 平台连接点的前后偏移 (平台在万向节前面)
    w_motor=35.58,       # 两个电机轴之间的距离的一半
    d_motor=24.05,       # 电机轴平面的前后位置
    z_motor_offset=-97.57 # 【关键】电机轴在万向节中心下方30mm
    )
    target_pitch = 0 # 抬头 10 度
    target_roll = 0    # 向左歪头 5 度
    # 人类视角：roll > 0 表示“我看到的向左歪头”

    ang_L, ang_R = robot.compute_ik(target_roll, target_pitch)
    ang_H = 0  #下方电机
    #需要将角度转换为弧度[yaw, roll, pitch]
    q_targets = [
    ang_H,
    math.radians(ang_L),
    math.radians(ang_R),
    ]

    kp_list = [13, 13, 13]
    kd_list = [1, 1, 1]

    ctrl, serial_dev = init_motors('/dev/ttyACM0', motors)
    enable_motors(ctrl, motors)
    dt_val = 0.001

    #回零
    zero_targets = [0, 2.64, 2.85]
    print(f"\n 任务1: 移动到目标 {zero_targets} ")
    move_to_target_linear(
        ctrl, motors, zero_targets, kp_list, kd_list,
        desired_velocity=0.5,
        dt=dt_val
    )

    time.sleep(1)

    #目标位置

    print(f"\n 任务2: 移动到目标 {q_targets} ")
    move_to_target_linear(
         ctrl, motors, q_targets, kp_list, kd_list,
         desired_velocity=0.5,
         dt=dt_val
    )

    servo_L_command = ang_L 
    servo_R_command = ang_R 
    print(f"实际舵机控制参考值:")
    print(f"  左舵机: {servo_L_command:.2f}")
    print(f"  右舵机: {servo_R_command:.2f}")
 
    # 结束
    print("\n 所有任务完成，失能电机 ")
    disable_all(ctrl, motors)
    serial_dev.close()