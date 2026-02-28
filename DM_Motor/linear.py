import time
import numpy as np
from DM_CAN import *
import math
from import_math import *
# 最小加加速度规划器
class MinimumJerkPlanner:
    def __init__(self, dt):
        self.dt = dt
    def plan_trajectory(self, start_pos, end_pos, duration):
        trajectory = []
        T = duration
        delta_p = end_pos - start_pos
        if T <= self.dt or abs(delta_p) < 1e-6:
            trajectory.append((end_pos, 0.0, 0.0))
            return trajectory
        num_steps = int(T / self.dt)
        for i in range(num_steps + 1):
            t = i * self.dt
            tau = min(max(t / T, 0.0), 1.0)
            pos = start_pos + delta_p * (10 * tau**3 - 15 * tau**4 + 6 * tau**5)
            vel = (delta_p / T) * (30 * tau**2 - 60 * tau**3 + 30 * tau**4)
            acc = (delta_p / T**2) * (60 * tau - 180 * tau**2 + 120 * tau**3)
            trajectory.append((pos, vel, acc))
        if trajectory:
            trajectory[-1] = (end_pos, 0.0, 0.0)
        else:
            trajectory.append((end_pos, 0.0, 0.0))
        return trajectory

# 计算自适应运动时间的辅助函数 
def calculate_adaptive_duration(distance, desired_velocity, min_duration, max_duration):
    if abs(distance) < 1e-4:
        return min_duration
    base_duration = abs(distance) / desired_velocity
    adaptive_duration = max(min_duration, min(base_duration, max_duration))
    return adaptive_duration

# 使用自适应时间的运动函数 

def move_to_target_linear(ctrl, motors, q_targets, kp_list, kd_list,
                          desired_velocity, # 期望的“巡航”速度
                          dt=0.001,
                          min_duration=0.5, # 任何运动的最短时间
                          max_duration=5.0  # 任何运动的最长时间
                         ):
    """
    使用“线性插补”和“最小加加速度”进行多轴同步运动。
    保证所有电机运动完成的百分比在每一刻都相同，从而让末端走出直线。
    """
    #  阶段1: 找出“主导轴”和计算“同步时间” 
    refresh_all(ctrl, motors)
    initial_positions = [m.getPosition() for m in motors]
    
    # 计算每个轴需要运动的距离
    distances = [abs(q_targets[i] - initial_positions[i]) for i in range(len(motors))]
    
    # 找到运动距离最长的那个轴（主导轴）
    max_distance = max(distances) if distances else 0
    
    if max_distance < 1e-4:
        print("所有电机已在目标位置，无需移动。")
        return None

    #print("轨迹规划")
    #print(f"主导轴运动距离: {max_distance:.4f} rad")

    # 根据主导轴的距离和期望速度，计算出全局的同步时间
    sync_duration = calculate_adaptive_duration(
        max_distance, desired_velocity, min_duration, max_duration
    )
    
    #print(f"根据主导轴计算出的同步运动时间 (T_sync): {sync_duration:.2f}s")
    
    #  阶段2: 使用同一个同步时间为所有电机规划轨迹 
    trajectories = []
   
    max_traj_len = 0 
    planner = MinimumJerkPlanner(dt)

    for i in range(len(motors)):
        # 每一个电机，都使用相同的 sync_duration 进行规划
        trajectory = planner.plan_trajectory(initial_positions[i], q_targets[i], sync_duration)
        trajectories.append(trajectory)
    
    # 规划出的所有轨迹列表长度相同
    if trajectories:
        max_traj_len = len(trajectories[0])

    #print(f"轨迹规划完成，所有轴的轨迹步数均为: {max_traj_len}")
    
    #阶段3: 执行轨迹 
    #print("执行轨迹")
    start_time = time.time()
    for step_idx in range(max_traj_len):
        for i, m in enumerate(motors):
            # 由于所有轨迹一样长，不再需要判断 step_idx 是否越界
            qd, dqd, _ = trajectories[i][step_idx]
            ctrl.controlMIT(m, kp_list[i], kd_list[i], qd, dqd, 0.1) # MIT 控制
        time.sleep(dt)
    end_time = time.time()
    #print(f"轨迹执行完毕，耗时: {round(end_time - start_time, 3)}s")

    # 读取并打印最终位置——验证
    #print("验证最终电机位置")
    # time.sleep(0.1)
    # refresh_all(ctrl, motors)
    # for i, m in enumerate(motors):
    #     final_pos = m.getPosition()
    #     error = final_pos - q_targets[i]
    #print(f"  电机 {i}: 目标 = {q_targets[i]:.4f}, 实际 = {final_pos:.4f}, 误差 = {error:.4f} rad")
    
    return trajectories


def set_neck_pose(robot, ctrl, motors, kp_list, kd_list, dt_val, pose_array):
    """
    根据传入的包含三个姿态角的“数组”，计算并驱动电机运动。

    """
    # 解析姿态角[yaw, roll, pitch]
    target_yaw, target_roll, target_pitch = pose_array
    
    ang_L, ang_R = robot.compute_ik(target_pitch, target_roll)
    ang_H = -target_yaw
    q_targets = [
        math.radians(ang_H),
        math.radians(ang_L),
        math.radians(ang_R),
    ]
    move_to_target_linear(
        ctrl, motors, q_targets, kp_list, kd_list,
        desired_velocity=0.5,
        dt=dt_val
    )
