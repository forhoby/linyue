import math
import numpy as np
import time
import matplotlib.pyplot as plt
from import_math import *

# 主测试程序 (带交互功能)

if __name__ == "__main__":
    
    robot = NeckMechanism(
        l_crank=13.5, l_rod=118, w_platform=22.41, h_platform=24.23,
        d_platform=24.05, w_motor=35.58, d_motor=24.05, z_motor_offset=-97.57
    )
    
    PITCH_RANGE = (-90.0, 90.0)
    ROLL_RANGE = (-90.0, 90.0)
    STEP = 1.0  # 为了更精确地探索边界，建议使用较小的步长

    print("--- 正在扫描工作空间以生成可视化图表... ---")
    start_time = time.time()

    successful_points_x = []
    successful_points_y = []
    
    pitch_values = np.arange(PITCH_RANGE[0], PITCH_RANGE[1] + STEP, STEP)
    roll_values = np.arange(ROLL_RANGE[0], ROLL_RANGE[1] + STEP, STEP)
    
    for pitch in pitch_values:
        for roll in roll_values:
            result_L, result_R = robot.compute_ik(roll, pitch)
            if result_L is not None and result_R is not None:
                successful_points_x.append(roll)
                successful_points_y.append(pitch)
    
    end_time = time.time()
    print(f"--- 扫描完成，耗时: {end_time - start_time:.2f} 秒 ---")
    
    #  使用 Matplotlib 绘图 
    if successful_points_x:
        fig, ax = plt.subplots(figsize=(12, 10)) # 获取 fig 和 ax 对象
        
        # 绘制散点图，并保存 scatter 对象
        sc = plt.scatter(successful_points_x, successful_points_y, s=15, alpha=0.7)
        
        ax.set_title('Interactive Neck Mechanism Workspace', fontsize=16)
        ax.set_xlabel('Roll Angle (°)', fontsize=12)
        ax.set_ylabel('Pitch Angle (°)', fontsize=12)
        
        ax.grid(True)
        ax.axhline(0, color='black', linewidth=0.5)
        ax.axvline(0, color='black', linewidth=0.5)
        ax.axis('equal')

        # 创建并初始化一个用于显示坐标的注释框 
        annot = ax.annotate("", xy=(0,0), xytext=(20,20),
                            textcoords="offset points",
                            bbox=dict(boxstyle="round", fc="w"),
                            arrowprops=dict(arrowstyle="->"))
        annot.set_visible(False)
        
        # 将所有成功的点打包，方便后续查找
        all_points = np.column_stack((successful_points_x, successful_points_y))

        def update_annot(ind):
            """更新注释框的文本和位置"""
            pos = sc.get_offsets()[ind["ind"][0]]
            annot.xy = pos
            text = f"Roll: {pos[0]:.1f}\nPitch: {pos[1]:.1f}"
            annot.set_text(text)
            annot.get_bbox_patch().set_alpha(0.8)

        def on_hover(event):
            """当鼠标悬停时触发的事件"""
            vis = annot.get_visible()
            if event.inaxes == ax:
                cont, ind = sc.contains(event)
                if cont:
                    update_annot(ind)
                    annot.set_visible(True)
                    fig.canvas.draw_idle()
                else:
                    if vis:
                        annot.set_visible(False)
                        fig.canvas.draw_idle()

        # 将 on_hover 函数与“鼠标移动”事件绑定 
        fig.canvas.mpl_connect("motion_notify_event", on_hover)
        
        plt.show()
    else:
        print("【警告】未能找到任何可行的点来绘图！")
