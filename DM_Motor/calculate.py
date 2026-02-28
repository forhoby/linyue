import math

# 摄像头对角视场角
DIAGONAL_FOV = 90

# 摄像头的分辨率
FRAME_WIDTH = 2592
FRAME_HEIGHT = 1944

def calculate_hv_fov(diagonal_fov_deg, aspect_ratio):
    """
    根据对角视场角和宽高比，计算水平和垂直视场角。
    """
    # 将对角视场角从度转换为弧度
    diagonal_fov_rad = math.radians(diagonal_fov_deg)
    
    # 核心公式
    try:
        fov_v_rad = 2 * math.atan(math.tan(diagonal_fov_rad / 2) / math.sqrt(aspect_ratio**2 + 1))
        fov_h_rad = 2 * math.atan(aspect_ratio * math.tan(fov_v_rad / 2))
    except (ValueError, ZeroDivisionError):
        return None, None
        
    # 将弧度转换回度并返回
    return math.degrees(fov_h_rad), math.degrees(fov_v_rad)


def main():


    # 步骤1：检查并计算基本参数 
    if FRAME_WIDTH <= 0 or FRAME_HEIGHT <= 0:
        print("[错误] 分辨率的宽度和高度必须是正数。")
        return
        
    aspect_ratio_val = FRAME_WIDTH / FRAME_HEIGHT
    
    # 步骤2：调用核心函数进行转换
    horizontal_fov, vertical_fov = calculate_hv_fov(DIAGONAL_FOV, aspect_ratio_val)

    # 步骤3：显示结果 
    if horizontal_fov is not None:
        print("输入的预设参数:")
        print(f"  - 对角视场角: {DIAGONAL_FOV}°")
        print(f"  - 分辨率:      {FRAME_WIDTH} x {FRAME_HEIGHT}")
        print(f"  - 宽高比:      {aspect_ratio_val:.3f}")
        print("-" * 35)
        print("计算出的视场角:")
        print(f"  >>> 水平视场角 (FOV-H): {horizontal_fov:.2f}° <<<")
        print(f"  >>> 垂直视场角 (FOV-V): {vertical_fov:.2f}° <<<")
        print("-" * 35)
    else:
        print("\n[错误] 计算失败，请检查预设的参数值是否有效。")

# 主程序
if __name__ == "__main__":
    main()
