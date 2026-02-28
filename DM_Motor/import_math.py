### Python 代码实现
import math

class NeckMechanism:
    def __init__(self, 
                 l_crank,      # 舵机摇臂长度 (mm)
                 l_rod,        # 连杆长度 (mm)
                 w_platform,   # 平台连杆点距离中心线的水平宽度 (mm, 单侧)
                 h_platform,   # 平台连杆点距离万向节中心的高度 (mm)
                 d_platform,   # 平台连杆点距离万向节中心的前后距离 (mm, 正值为前方)
                 w_motor,      # 电机轴中心距离中心线的水平宽度 (mm, 单侧)
                 d_motor,      # 电机轴中心距离万向节中心的前后距离 (mm, 正值为前方)
                 z_motor_offset # 【关键参数】电机轴相对于万向节中心的高度偏移 (mm, 负值代表在万向节下方)
                 ):
        """
        初始化机械结构参数
        所有坐标系以万向节中心为原点 (0,0,0)
        X: 左右 (右为正)
        Y: 前后 (前为正)
        Z: 上下 (上为正)
        """
        self.l_crank = l_crank
        self.l_rod = l_rod
        
        # 头部平台连接点初始位置 (相对于万向节)
        # 左侧点 (x负) 和 右侧点 (x正)
        self.p_top_left_init = [-w_platform, d_platform, h_platform]
        self.p_top_right_init = [w_platform, d_platform, h_platform]
        
        # 电机轴中心位置
        # 左电机在左边(x负)，右电机在右边(x正)
        self.p_motor_left = [-w_motor, d_motor, z_motor_offset]
        self.p_motor_right = [w_motor, d_motor, z_motor_offset]

    def _rotate_point(self, point, pitch_deg, roll_deg):
        """
        应用旋转矩阵计算新坐标
        Rotation Order: Pitch (X轴) -> Roll (Y轴)
        """
        x, y, z = point
        
        # 角度转弧度
        alpha = math.radians(pitch_deg) # 绕X轴 (点头)
        beta = math.radians(roll_deg)   # 绕Y轴 (侧偏)
        
        # 1. 绕 X 轴旋转 (Pitch)
        # y' = y*cos(a) - z*sin(a)
        # z' = y*sin(a) + z*cos(a)
        y_pitch = y * math.cos(alpha) - z * math.sin(alpha)
        z_pitch = y * math.sin(alpha) + z * math.cos(alpha)
        x_pitch = x # x不变
        
        # 2. 绕 Y 轴旋转 (Roll) - 基于变换后的点继续转
        # x'' = x*cos(b) + z'*sin(b)
        # z'' = -x*sin(b) + z'*cos(b)
        x_final = x_pitch * math.cos(beta) + z_pitch * math.sin(beta)
        z_final = -x_pitch * math.sin(beta) + z_pitch * math.cos(beta)
        y_final = y_pitch # y不变
        
        return [x_final, y_final, z_final]

    def _solve_motor_angle_geometry(self, target_point, motor_center, is_left_motor):
        """
        核心逆运动学算法：解算单个电机的角度
        """
        # 1. 提取坐标
        x_t, y_t, z_t = target_point
        x_m, y_m, z_m = motor_center
        
        # 2. 计算 Y 轴方向的距离差
        dy = y_t - y_m
        
        # 3. 计算连杆在 XZ 平面的有效投影长度
        # 如果 dy > rod_length，说明连杆够不着，物理上不可能
        if abs(dy) > self.l_rod:
            raise ValueError(f"错误：目标位置超出机械极限，Y轴差距({dy:.2f})大于连杆长度")
            
        l_rod_proj = math.sqrt(self.l_rod**2 - dy**2)
        
        # 4. 在 XZ 平面求解圆圆相交
        # 我们需要找到角度 theta，使得电机臂末端到目标点的距离等于 l_rod_proj
        
        # 目标点相对于电机轴中心的坐标 (XZ平面)
        dx = x_t - x_m
        dz = z_t - z_m
        
        # 电机轴到目标点的直线距离
        dist_centers = math.sqrt(dx**2 + dz**2)
        
        # 检查是否构成三角形 (两边之和大于第三边)
        if dist_centers > (self.l_crank + l_rod_proj) or dist_centers < abs(self.l_crank - l_rod_proj):
            raise ValueError("错误：目标位置不可达 (XZ平面几何构型限制)")
            
        # 5. 利用余弦定理求解角度
        # 三角形三边：a=l_crank, b=l_rod_proj, c=dist_centers
        # 求电机臂与连接线(dist_centers)之间的夹角 phi
        # cos(phi) = (a^2 + c^2 - b^2) / (2ac)
        cos_phi = (self.l_crank**2 + dist_centers**2 - l_rod_proj**2) / (2 * self.l_crank * dist_centers)
        
        # 防止浮点数误差导致 acos 越界
        cos_phi = max(-1.0, min(1.0, cos_phi))
        phi = math.acos(cos_phi)
        
        # 连接线(电机中心指向目标点) 与 X轴(水平) 的基础角度
        base_angle = math.atan2(dz, dx)
        
        # 6. 计算最终电机角度
        # 连杆结构有两种解 (肘部向上 or 肘部向下)。
        # 根据你的图片，连杆是在电机上方的，所以我们要取 z 值较大的那个解。
        # 通常意味着：BaseAngle + phi
        theta_rad = base_angle + phi
        
        theta_deg = math.degrees(theta_rad)
        return theta_deg

    def compute_ik(self, pitch, roll):
        """
        主接口：输入目标 Pitch 和 Roll，返回左右电机角度
        返回: (left_angle, right_angle) 单位：度
        注意：0度代表水平向右(X正方向)，90度代表垂直向上
        """
        try:
            # 1. 计算头部两个连接点的新位置
            p_left_new = self._rotate_point(self.p_top_left_init, pitch, roll)
            p_right_new = self._rotate_point(self.p_top_right_init, pitch, roll)
            
            # 2. 分别解算左右电机
            # 注意：返回的角度是数学上的平面角度 (0度=X轴正向)
            angle_L = self._solve_motor_angle_geometry(p_left_new, self.p_motor_left, is_left_motor=True)
            angle_R = self._solve_motor_angle_geometry(p_right_new, self.p_motor_right, is_left_motor=False)
            
            return angle_L, angle_R
            
        except ValueError as e:
            print(f"运算出错: {e}")
            return None, None

# # ==========================================
# # 使用示例
# # ==========================================

# # 1. 实例化：请在此处填入你的 SolidWorks 测量数据 (单位 mm)
# # 下面的数据是根据你图片目测估计的，请务必修改！
# robot = NeckMechanism(
#     l_crank=13.5,       # 舵机臂长 (假设)
#     l_rod=123.33,         # 连杆长 (假设)
#     w_platform=22.41,    # 平台上连接点离中心线的宽度
#     h_platform=24.23,   # 平台初始高度 (万向节中心到上面的连杆点)
#     d_platform=24.05,    # 平台连接点的前后偏移 (平台在万向节前面)
#     w_motor=35.58,       # 两个电机轴之间的距离的一半
#     d_motor=24.05,       # 电机轴平面的前后位置
#     z_motor_offset=-97.57 # 【关键】电机轴在万向节中心下方30mm
# )

# # 2. 测试计算
# target_pitch = 0  # 点头 10 度
# target_roll = 0    # 向左歪头 5 度
#  # 人类视角：roll > 0 表示“我看到的向左歪头”

# ang_L, ang_R = robot.compute_ik(target_pitch, target_roll)

# if ang_L is not  None:
#     print("-" * 30)
#     print(f"目标姿态: Pitch={target_pitch}°, Roll={target_roll}°")
#     print(f"数学计算角度 (0°=X轴正向):")
#     print(f"  左电机: {ang_L:.2f}°")
#     print(f"  右电机: {ang_R:.2f}°")
#     print("-" * 30)
    
#     # 3. 映射到你的具体舵机安装方式
#     # 你的描述：左电机初始指向X正(0度)，右电机初始指向X负(180度)
#     # 假设这是你的 PWM 1500us (中间值) 或者 0位
    
#     # 左边如果是标准的：输出就是需要的角度
#     servo_L_command = ang_L 
    
#     # 右边如果初始是向左(180度)，那么需要的舵机指令可能是：
#     # 如果舵机也是逆时针转增大，那就是 ang_R
#     # 但通常为了对称，你可能需要用 180 - ang_R 或者 ang_R - 180，取决于具体安装
#     servo_R_command = ang_R 
    
#     print(f"实际舵机控制参考值:")
#     print(f"  左舵机: {servo_L_command:.2f}")
#     print(f"  右舵机: {servo_R_command:.2f}")

