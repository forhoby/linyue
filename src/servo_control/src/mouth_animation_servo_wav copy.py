import pyaudio
import numpy as np
import math
import cv2
import threading
import queue
from collections import deque
import time
import wave

# ================= 配置区域 =================
# 音频文件设置
AUDIO_FILE = "ZH_7_1.wav"  # 音频文件路径

# 音频设置
CHUNK = 1024            # 每次读取的音频帧大小
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000            # 16kHz 采样率（会根据文件自动调整）

# 舵机平滑系数 (0.0-1.0), 越小越平滑但延迟越高
SMOOTHING_JAW = 0.6
SMOOTHING_CORNER = 0.6

# 视频设置
VIDEO_WIDTH = 800
VIDEO_HEIGHT = 600
FPS = 30

# 嘴巴绘制参数
MOUTH_CENTER_X = VIDEO_WIDTH // 2
MOUTH_CENTER_Y = VIDEO_HEIGHT // 2
MOUTH_BASE_WIDTH = 200    # 基础宽度
MOUTH_BASE_HEIGHT = 40    # 基础高度
MOUTH_MAX_OPEN = 120      # 最大张开高度

# 自适应参数设置
ADAPTIVE_WINDOW_SIZE = 150   # 用于统计的历史帧数（约10秒）
ADAPTIVE_UPDATE_INTERVAL = 30  # 每N帧更新一次参数

# 舵机参数设置
SERVO_JAW_MIN = 110      # 嘴部张合最小角度（闭合）
SERVO_JAW_MAX = 155      # 嘴部张合最大角度（张开）
SERVO_LEFT_MIN = 80      # 左嘴角最小角度（内聚）
SERVO_LEFT_MAX = 160     # 左嘴角最大角度（外展）
SERVO_RIGHT_MIN = 80     # 右嘴角最小角度（外展）
SERVO_RIGHT_MAX = 160    # 右嘴角最大角度（内收）

# 舵机索引（根据实际硬件配置修改）
SERVO_JAW_INDEX = 10      # 嘴部张合舵机索引
SERVO_LEFT_INDEX = 9     # 左嘴角舵机索引
SERVO_RIGHT_INDEX = 8    # 右嘴角舵机索引

# 舵机更新频率限制（避免过于频繁的指令）
SERVO_UPDATE_INTERVAL_MS = 50  # 最小更新间隔（毫秒）

# ===========================================

class AdaptiveParameterManager:
    """运行时自适应参数管理器 - 动态调整参数"""
    def __init__(self):
        # 初始默认参数
        self.noise_gate = 20
        self.max_rms = 80
        self.min_freq = 400
        self.max_freq = 4000
        
        # 历史数据缓冲区（使用双端队列以提高性能）
        self.rms_history = deque(maxlen=ADAPTIVE_WINDOW_SIZE)
        self.freq_history = deque(maxlen=ADAPTIVE_WINDOW_SIZE)
        
        # 更新计数器
        self.frame_count = 0
        self.is_warming_up = True  # 前30帧为预热期
        
    def update(self, rms, fft_data, freqs):
        """
        更新参数（每帧调用）
        rms: 当前帧的RMS值
        fft_data: FFT频谱数据
        freqs: 频率轴
        """
        self.frame_count += 1
        
        # 收集数据到历史缓冲区
        self.rms_history.append(rms)
        
        # 频谱分析
        if np.sum(fft_data) > 0:
            # 计算频谱质心
            centroid = np.sum(freqs * fft_data) / np.sum(fft_data)
            
            # 找到主频率范围（能量集中区域）
            cumsum = np.cumsum(fft_data)
            total_energy = cumsum[-1]
            
            if total_energy > 0:
                # 找到包含80%能量的频率范围
                lower_idx = np.searchsorted(cumsum, total_energy * 0.10)
                upper_idx = np.searchsorted(cumsum, total_energy * 0.90)
                freq_range = (freqs[lower_idx], freqs[upper_idx])
                
                self.freq_history.append({
                    'centroid': centroid,
                    'range': freq_range
                })
        
        # 定期更新参数（每N帧）
        if self.frame_count % ADAPTIVE_UPDATE_INTERVAL == 0 and len(self.rms_history) > 20:
            self._update_parameters()
            
            # 预热期结束
            if self.frame_count >= 30:
                self.is_warming_up = False
    
    def _update_parameters(self):
        """根据历史数据更新参数"""
        
        # === 1. 更新 NOISE_GATE ===
        if len(self.rms_history) > 20:
            rms_array = np.array(self.rms_history)
            
            # 找出最低的20%作为"安静"样本
            sorted_rms = np.sort(rms_array)
            quiet_samples = sorted_rms[:len(sorted_rms)//5]
            
            if len(quiet_samples) > 0:
                # 噪音门限 = 安静样本均值 + 标准差
                quiet_mean = np.mean(quiet_samples)
                quiet_std = np.std(quiet_samples)
                new_noise_gate = quiet_mean + quiet_std
                
                # 平滑更新，避免突变
                self.noise_gate = self.noise_gate * 0.9 + new_noise_gate * 0.1
        
        # === 2. 更新 MAX_RMS ===
        if len(self.rms_history) > 20:
            rms_array = np.array(self.rms_history)
            
            # 过滤掉低于噪音门限的样本
            speech_rms = rms_array[rms_array > self.noise_gate]
            
            if len(speech_rms) > 10:
                # 使用90百分位数作为最大值参考
                new_max_rms = np.percentile(speech_rms, 90)
                
                # 确保最大值至少是噪音门限的3倍
                new_max_rms = max(new_max_rms, self.noise_gate * 3)
                
                # 平滑更新
                self.max_rms = self.max_rms * 0.9 + new_max_rms * 0.1
        
        # === 3. 更新频率范围 ===
        if len(self.freq_history) > 10:
            all_ranges = [f['range'] for f in self.freq_history]
            
            lower_bounds = [r[0] for r in all_ranges if r[0] > 0]
            upper_bounds = [r[1] for r in all_ranges if r[1] > 0]
            
            if lower_bounds and upper_bounds:
                # 使用10百分位和90百分位
                new_min_freq = max(400, np.percentile(lower_bounds, 10))
                new_max_freq = min(4000, np.percentile(upper_bounds, 90))
                
                # 平滑更新
                self.min_freq = self.min_freq * 0.95 + new_min_freq * 0.05
                self.max_freq = self.max_freq * 0.95 + new_max_freq * 0.05
    
    def get_params(self):
        """获取当前参数"""
        return {
            'noise_gate': self.noise_gate,
            'max_rms': self.max_rms,
            'min_freq': self.min_freq,
            'max_freq': self.max_freq,
            'is_warming_up': self.is_warming_up
        }


class ServoMapper:
    """舵机角度映射器 - 将口型参数映射到舵机角度"""
    def __init__(self):
        self.last_update_time = 0
        
    def map_to_servo_angles(self, jaw_val, width_val):
        """
        将口型参数映射到舵机角度
        
        参数:
        jaw_val: 0.0 (闭嘴) ~ 1.0 (最大张开)
        width_val: 0.0 (嘟嘴/圆唇) ~ 0.5 (自然) ~ 1.0 (最大咧嘴)
        
        返回:
        (jaw_angle, left_angle, right_angle)
        """
        
        # === 1. 嘴部张合舵机 ===
        # jaw_val: 0 -> SERVO_JAW_MIN (闭合)
        # jaw_val: 1 -> SERVO_JAW_MAX (张开)
        jaw_angle = SERVO_JAW_MIN + (SERVO_JAW_MAX - SERVO_JAW_MIN) * jaw_val
        
        # === 2. 左嘴角舵机 ===
        # width_val: 0.0 -> SERVO_LEFT_MIN (内聚/嘟嘴)
        # width_val: 0.5 -> 中间位置
        # width_val: 1.0 -> SERVO_LEFT_MAX (外展/咧嘴)
        left_angle = SERVO_LEFT_MIN + (SERVO_LEFT_MAX - SERVO_LEFT_MIN) * width_val
        
        # === 3. 右嘴角舵机 ===
        # 注意：右嘴角的运动方向与左嘴角相反
        # width_val: 0.0 -> SERVO_RIGHT_MAX (外展/嘟嘴)
        # width_val: 0.5 -> 中间位置
        # width_val: 1.0 -> SERVO_RIGHT_MIN (内收/咧嘴)
        right_angle = SERVO_RIGHT_MAX - (SERVO_RIGHT_MAX - SERVO_RIGHT_MIN) * width_val
        
        return jaw_angle, left_angle, right_angle
    
    def should_update(self):
        """判断是否应该更新舵机（基于时间间隔限制）"""
        current_time = time.time() * 1000  # 转换为毫秒
        if current_time - self.last_update_time >= SERVO_UPDATE_INTERVAL_MS:
            self.last_update_time = current_time
            return True
        return False


class RobotFaceController:
    def __init__(self, face_controller=None, audio_file=None):
        # 初始化当前舵机值 (0.0 - 1.0)
        self.current_jaw = 0.0
        self.current_width = 0.5  # 0.5 代表嘴角自然放松位置

        # 初始化 PyAudio
        self.p = pyaudio.PyAudio()
        
        # 音频文件相关
        self.audio_file = audio_file
        self.wf = None
        self.output_stream = None  # 用于播放音频
        self.is_playing = False
        self.audio_data_queue = queue.Queue()
        
        if self.audio_file:
            # 打开WAV文件
            self.wf = wave.open(self.audio_file, 'rb')
            
            # 获取音频文件参数
            global RATE, CHANNELS, FORMAT
            CHANNELS = self.wf.getnchannels()
            RATE = self.wf.getframerate()
            sample_width = self.wf.getsampwidth()
            
            # 根据采样宽度设置格式
            if sample_width == 1:
                FORMAT = pyaudio.paInt8
            elif sample_width == 2:
                FORMAT = pyaudio.paInt16
            elif sample_width == 4:
                FORMAT = pyaudio.paInt32
            
            print(f"\n📁 音频文件信息:")
            print(f"   - 采样率: {RATE} Hz")
            print(f"   - 声道数: {CHANNELS}")
            print(f"   - 采样宽度: {sample_width} bytes")
            print(f"   - 总帧数: {self.wf.getnframes()}")
            print(f"   - 时长: {self.wf.getnframes() / RATE:.2f} 秒")
            
            # 创建输出流用于播放
            self.output_stream = self.p.open(
                format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                output=True,
                frames_per_buffer=CHUNK
            )
        else:
            # 使用麦克风输入（原有逻辑）
            self.stream = self.p.open(format=FORMAT,
                                      channels=CHANNELS,
                                      rate=RATE,
                                      input=True,
                                      frames_per_buffer=CHUNK)
        
        # 预计算频率轴 (用于FFT)
        self.freqs = np.fft.rfftfreq(CHUNK, 1.0/RATE)
        
        # 创建自适应参数管理器
        self.param_manager = AdaptiveParameterManager()
        
        # 创建舵机映射器
        self.servo_mapper = ServoMapper()
        
        # 舵机控制器（可选）
        self.face_controller = face_controller
        
        # 初始化频率掩码
        self.update_freq_mask()
        
        # 播放控制
        self.playback_finished = False

    def update_freq_mask(self):
        """更新频率掩码"""
        params = self.param_manager.get_params()
        self.freq_mask = (self.freqs > params['min_freq']) & (self.freqs < params['max_freq'])

    def calculate_spectral_centroid(self, magnitudes):
        """
        计算频谱质心：用于判断声音是低沉的(O/U)还是尖锐的(E/I/Smile)
        返回: 归一化的频率重心 (0.0 - 1.0)
        """
        # 只取人声频段
        masked_freqs = self.freqs[self.freq_mask]
        masked_mags = magnitudes[self.freq_mask]

        if np.sum(masked_mags) == 0:
            return 0.5

        # 加权平均频率
        centroid = np.sum(masked_freqs * masked_mags) / np.sum(masked_mags)
        
        # 将重心频率映射到 0-1 (假设人声重心在 500Hz 到 2500Hz 之间变化)
        # 高频(>1500Hz)通常对应 'i', 'e' (嘴角拉开)
        # 低频(<800Hz) 通常对应 'u', 'o' (嘴角收缩)
        normalized = np.clip((centroid - 500) / 2000, 0.0, 1.0)
        return normalized

    def update(self):
        try:
            # 1. 读取音频数据
            if self.audio_file:
                # 从WAV文件读取
                data = self.wf.readframes(CHUNK)
                
                # 检查是否到达文件末尾
                if len(data) < CHUNK * self.wf.getsampwidth() * CHANNELS:
                    self.playback_finished = True
                    # 填充剩余部分为静音
                    if len(data) > 0:
                        padding_size = CHUNK * self.wf.getsampwidth() * CHANNELS - len(data)
                        data += b'\x00' * padding_size
                    else:
                        return 0, 0.5, 0
                
                # 同步播放音频
                if self.output_stream and len(data) > 0:
                    self.output_stream.write(data)
            else:
                # 从麦克风读取（原有逻辑）
                data = self.stream.read(CHUNK, exception_on_overflow=False)
            
            # 2. 转换为numpy数组
            if self.wf and CHANNELS == 2:
                # 如果是立体声，转换为单声道
                audio_data = np.frombuffer(data, dtype=np.int16)
                audio_data = audio_data.reshape(-1, 2).mean(axis=1).astype(np.int16)
            else:
                audio_data = np.frombuffer(data, dtype=np.int16)

            # 3. 计算能量 (RMS) - 控制嘴巴张合
            rms = np.sqrt(np.mean(audio_data**2))
            
            # 4. 快速傅里叶变换 (FFT) - 控制嘴形宽窄
            fft_data = np.abs(np.fft.rfft(audio_data))
            
            # 5. 更新自适应参数
            self.param_manager.update(rms, fft_data, self.freqs)
            params = self.param_manager.get_params()
            
            # 定期更新频率掩码
            if self.param_manager.frame_count % ADAPTIVE_UPDATE_INTERVAL == 0:
                self.update_freq_mask()

            # === 核心逻辑 ===
            target_jaw = 0.0
            target_width = 0.5 

            if rms > params['noise_gate']:
                # --- A. 纵向张合 (Jaw) ---
                # 使用对数映射让小声说话也能张嘴
                # 归一化并限制在 0-1
                target_jaw = np.clip((rms - params['noise_gate']) / (params['max_rms'] - params['noise_gate']), 0, 1)
                # 稍微放大一点小信号
                target_jaw = math.pow(target_jaw, 0.7) 

                # --- B. 横向拉伸 (Corners) ---
                # 计算频谱质心
                centroid_factor = self.calculate_spectral_centroid(fft_data)
                
                # 逻辑：
                # 质心高 (i/e) -> width > 0.5 (拉宽)
                # 质心低 (u/o) -> width < 0.5 (收缩/嘟嘴)
                target_width = centroid_factor 

            # === 平滑滤波 (Low Pass Filter) ===
            # 防止舵机抖动，模拟肌肉惯性
            self.current_jaw = (self.current_jaw * (1 - SMOOTHING_JAW)) + (target_jaw * SMOOTHING_JAW)
            self.current_width = (self.current_width * (1 - SMOOTHING_CORNER)) + (target_width * SMOOTHING_CORNER)

            # === 舵机控制 ===
            if self.face_controller and self.servo_mapper.should_update():
                jaw_angle, left_angle, right_angle = self.servo_mapper.map_to_servo_angles(
                    self.current_jaw, self.current_width
                )
                
                # 将角度映射到factor（根据FaceController的需求）
                list_factor_run = self.face_controller.list_factor.copy()
                list_factor_run[SERVO_JAW_INDEX] = int(jaw_angle)
                list_factor_run[SERVO_LEFT_INDEX] = int(left_angle)
                list_factor_run[SERVO_RIGHT_INDEX] = int(right_angle)
                
                # 发送舵机指令（使用较快的响应时间）
                self.face_controller.set_servo_angle_time_16(
                    list_factor_run, 
                    [SERVO_JAW_INDEX, SERVO_LEFT_INDEX, SERVO_RIGHT_INDEX], 
                    SERVO_UPDATE_INTERVAL_MS
                )

            return self.current_jaw, self.current_width, rms

        except IOError:
            return 0, 0.5, 0

    def close(self):
        if self.wf:
            self.wf.close()
        if self.output_stream:
            self.output_stream.stop_stream()
            self.output_stream.close()
        if hasattr(self, 'stream'):
            self.stream.stop_stream()
            self.stream.close()
        self.p.terminate()
    
    def reset_audio(self):
        """重置音频文件到开始位置"""
        if self.wf:
            self.wf.rewind()
            self.playback_finished = False


class MouthRenderer:
    """口型渲染器"""
    def __init__(self):
        self.frame = np.zeros((VIDEO_HEIGHT, VIDEO_WIDTH, 3), dtype=np.uint8)
        self.show_params = False  # 是否显示详细参数
        self.show_servo_angles = True  # 是否显示舵机角度
        
    def draw_mouth(self, jaw_val, width_val, rms, params=None, servo_angles=None, playback_info=None):
        """
        绘制嘴巴
        jaw_val: 0.0 (闭嘴) ~ 1.0 (最大张开)
        width_val: 0.0 (嘟嘴/圆唇) ~ 0.5 (自然) ~ 1.0 (最大咧嘴/E音)
        params: 自适应参数字典（可选）
        servo_angles: (jaw_angle, left_angle, right_angle) 舵机角度（可选）
        playback_info: {'current_time': float, 'total_time': float} 播放信息（可选）
        """
        # 清空画布
        self.frame.fill(30)  # 深灰色背景
        
        # 绘制标题和参数信息
        title = "Real-time Mouth Animation + Servo Control"
        if playback_info:
            title = "Audio Playback + Servo Control"
        cv2.putText(self.frame, title, 
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # 显示播放进度
        if playback_info:
            current = playback_info.get('current_time', 0)
            total = playback_info.get('total_time', 1)
            progress = (current / total) * 100 if total > 0 else 0
            
            cv2.putText(self.frame, f"Progress: {current:.1f}s / {total:.1f}s ({progress:.1f}%)", 
                        (VIDEO_WIDTH - 400, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 255), 1)
            
            # 绘制进度条
            bar_x = VIDEO_WIDTH - 420
            bar_y = 50
            bar_w = 400
            bar_h = 10
            cv2.rectangle(self.frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (100, 100, 100), 1)
            filled_w = int(bar_w * (current / total)) if total > 0 else 0
            cv2.rectangle(self.frame, (bar_x, bar_y), (bar_x + filled_w, bar_y + bar_h), (100, 200, 255), -1)
        
        # 显示预热状态
        elif params and params.get('is_warming_up', False):
            cv2.putText(self.frame, "[Adapting...]", 
                        (VIDEO_WIDTH - 150, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 1)
        
        # 显示当前参数
        cv2.putText(self.frame, f"Jaw Open: {jaw_val:.2f}", 
                    (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 100), 1)
        cv2.putText(self.frame, f"Width: {width_val:.2f}", 
                    (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 100), 1)
        cv2.putText(self.frame, f"RMS: {rms:.0f}", 
                    (20, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 100), 1)
        
        # 显示舵机角度
        if servo_angles and self.show_servo_angles:
            jaw_angle, left_angle, right_angle = servo_angles
            y_offset = 170
            cv2.putText(self.frame, "Servo Angles:", 
                        (20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 100), 1)
            cv2.putText(self.frame, f"  Jaw: {jaw_angle:.1f}deg", 
                        (20, y_offset + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 100), 1)
            cv2.putText(self.frame, f"  Left: {left_angle:.1f}deg", 
                        (20, y_offset + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 100), 1)
            cv2.putText(self.frame, f"  Right: {right_angle:.1f}deg", 
                        (20, y_offset + 75), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 100), 1)
        
        # 显示自适应参数（如果开启）
        if params and self.show_params:
            y_offset = 270 if self.show_servo_angles else 170
            cv2.putText(self.frame, "Adaptive Parameters:", 
                        (20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            cv2.putText(self.frame, f"  Noise Gate: {params['noise_gate']:.0f}", 
                        (20, y_offset + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)
            cv2.putText(self.frame, f"  Max RMS: {params['max_rms']:.0f}", 
                        (20, y_offset + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)
            cv2.putText(self.frame, f"  Freq Range: {params['min_freq']:.0f}-{params['max_freq']:.0f} Hz", 
                        (20, y_offset + 75), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)
        
        # 显示口型类型
        width_msg = "NORMAL"
        if width_val > 0.6: 
            width_msg = "WIDE (E/I)"
        elif width_val < 0.4: 
            width_msg = "ROUND (O/U)"
        
        y_pos = 170 if not self.show_servo_angles else 270
        if params and self.show_params:
            y_pos = 370
        cv2.putText(self.frame, f"Type: {width_msg}", 
                    (20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 100), 1)
        
        # === 绘制嘴巴 ===
        # 计算嘴巴实际尺寸
        width_factor = 0.5 + (width_val - 0.5) * 1.5  # 放大宽度变化效果
        mouth_width = int(MOUTH_BASE_WIDTH * width_factor)
        
        # 高度：根据 jaw_val 变化
        mouth_height = int(MOUTH_BASE_HEIGHT + (MOUTH_MAX_OPEN - MOUTH_BASE_HEIGHT) * jaw_val)
        
        # 绘制椭圆形嘴巴
        center = (MOUTH_CENTER_X, MOUTH_CENTER_Y)
        axes = (mouth_width // 2, mouth_height // 2)
        
        # 绘制外轮廓（嘴唇外边缘）
        cv2.ellipse(self.frame, center, axes, 0, 0, 360, (200, 100, 100), 3)
        
        # 绘制内部（口腔）
        if jaw_val > 0.1:  # 只有张嘴时才显示口腔
            inner_axes = (max(5, axes[0] - 15), max(5, axes[1] - 10))
            cv2.ellipse(self.frame, center, inner_axes, 0, 0, 360, (50, 20, 20), -1)
        
        # 绘制嘴唇内边缘
        if jaw_val > 0.05:
            inner_lip_axes = (max(5, axes[0] - 10), max(5, axes[1] - 5))
            cv2.ellipse(self.frame, center, inner_lip_axes, 0, 0, 360, (180, 80, 80), 2)
        
        # 绘制嘴角点（用于调试）
        left_corner = (MOUTH_CENTER_X - axes[0], MOUTH_CENTER_Y)
        right_corner = (MOUTH_CENTER_X + axes[0], MOUTH_CENTER_Y)
        cv2.circle(self.frame, left_corner, 5, (100, 255, 255), -1)
        cv2.circle(self.frame, right_corner, 5, (100, 255, 255), -1)
        
        # 上下嘴唇中心点
        top_point = (MOUTH_CENTER_X, MOUTH_CENTER_Y - axes[1])
        bottom_point = (MOUTH_CENTER_X, MOUTH_CENTER_Y + axes[1])
        cv2.circle(self.frame, top_point, 5, (255, 100, 255), -1)
        cv2.circle(self.frame, bottom_point, 5, (255, 100, 255), -1)
        
        # 绘制参数条形图
        self._draw_parameter_bars(jaw_val, width_val)
        
        return self.frame
    
    def _draw_parameter_bars(self, jaw_val, width_val):
        """绘制参数条形图"""
        bar_x = 20
        bar_y_jaw = VIDEO_HEIGHT - 100
        bar_y_width = VIDEO_HEIGHT - 50
        bar_width = 300
        bar_height = 20
        
        # Jaw 条形图
        cv2.rectangle(self.frame, (bar_x, bar_y_jaw), 
                     (bar_x + bar_width, bar_y_jaw + bar_height), 
                     (100, 100, 100), 2)
        filled_width = int(bar_width * jaw_val)
        cv2.rectangle(self.frame, (bar_x, bar_y_jaw), 
                     (bar_x + filled_width, bar_y_jaw + bar_height), 
                     (100, 255, 100), -1)
        cv2.putText(self.frame, "Jaw", 
                    (bar_x + bar_width + 10, bar_y_jaw + 15), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Width 条形图（中心为0.5）
        cv2.rectangle(self.frame, (bar_x, bar_y_width), 
                     (bar_x + bar_width, bar_y_width + bar_height), 
                     (100, 100, 100), 2)
        center_x = bar_x + bar_width // 2
        cv2.line(self.frame, (center_x, bar_y_width), 
                (center_x, bar_y_width + bar_height), (200, 200, 200), 1)
        
        if width_val >= 0.5:
            filled_start = center_x
            filled_end = int(bar_x + bar_width * width_val)
            cv2.rectangle(self.frame, (filled_start, bar_y_width), 
                         (filled_end, bar_y_width + bar_height), 
                         (255, 200, 100), -1)
        else:
            filled_start = int(bar_x + bar_width * width_val)
            filled_end = center_x
            cv2.rectangle(self.frame, (filled_start, bar_y_width), 
                         (filled_end, bar_y_width + bar_height), 
                         (100, 200, 255), -1)
        
        cv2.putText(self.frame, "Width", 
                    (bar_x + bar_width + 10, bar_y_width + 15), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)


def audio_processing_thread(controller, param_queue, stop_event):
    """音频处理线程"""
    while not stop_event.is_set():
        jaw, width, rms = controller.update()
        
        # 计算舵机角度
        jaw_angle, left_angle, right_angle = controller.servo_mapper.map_to_servo_angles(jaw, width)
        
        # 计算播放进度
        playback_info = None
        if controller.audio_file and controller.wf:
            current_frame = controller.wf.tell()
            total_frames = controller.wf.getnframes()
            sample_rate = controller.wf.getframerate()
            
            current_time = current_frame / sample_rate
            total_time = total_frames / sample_rate
            
            playback_info = {
                'current_time': current_time,
                'total_time': total_time
            }
        
        try:
            # 非阻塞方式放入队列，如果队列满了就丢弃旧数据
            if param_queue.full():
                param_queue.get_nowait()
            param_queue.put_nowait((jaw, width, rms, (jaw_angle, left_angle, right_angle), playback_info))
        except queue.Full:
            pass
        
        # 检查播放是否完成
        if controller.playback_finished:
            print("\n🎵 音频播放完成")
            stop_event.set()
            break


# ================= 主程序 =================
if __name__ == "__main__":
    print("\n" + "="*60)
    print("🎭 音频播放口型同步系统 + 舵机控制")
    print("="*60)
    print("\n✨ 特性:")
    print("  - 从WAV文件读取音频并同步播放")
    print("  - 自动适应不同人声音色")
    print("  - 运行时持续优化参数")
    print("  - 实时控制舵机口型匹配")
    print("\n" + "="*60 + "\n")
    
    # 检查音频文件是否存在
    import os
    if not os.path.exists(AUDIO_FILE):
        print(f"❌ 错误: 找不到音频文件 '{AUDIO_FILE}'")
        print(f"   请确保文件在当前目录: {os.getcwd()}")
        input("\n按 Enter 键退出...")
        exit(1)
    
    # 询问是否连接舵机
    print("是否连接舵机硬件？")
    print("  [y] 是 - 同时控制舵机和显示动画")
    print("  [n] 否 - 仅显示动画（测试模式）")
    
    choice = input("\n请选择 (y/n): ").strip().lower()
    
    face_controller = None
    
    if choice == 'y':
        # 导入舵机控制相关模块
        try:
            from motor import FaceController, UARTDevice
            
            print("\n⚙️  配置舵机参数:")
            serial_port = input(f"  串口号 (默认 COM15): ").strip() or "COM15"
            
            CONFIG_YAML = 'servo_config.yaml'
            SERVO_NUM = 29
            current_dir = os.path.dirname(os.path.abspath(__file__))
            
            face_controller = FaceController(
                UARTDevice(serial_port, 9600), 
                SERVO_NUM,
                current_dir + '/' + CONFIG_YAML
            )
            face_controller.open()
            
            print("✅ 舵机初始化成功")
            
        except Exception as e:
            print(f"\n❌ 舵机初始化失败: {e}")
            print("   将以仅动画模式运行")
            face_controller = None
    
    print(f"\n📂 加载音频文件: {AUDIO_FILE}")
    print("初始化音频控制器...")
    
    try:
        controller = RobotFaceController(face_controller, audio_file=AUDIO_FILE)
    except Exception as e:
        print(f"\n❌ 音频文件加载失败: {e}")
        input("\n按 Enter 键退出...")
        exit(1)
    
    renderer = MouthRenderer()
    
    # 创建参数队列和停止事件
    param_queue = queue.Queue(maxsize=2)
    stop_event = threading.Event()
    
    # 启动音频处理线程
    audio_thread = threading.Thread(
        target=audio_processing_thread, 
        args=(controller, param_queue, stop_event)
    )
    audio_thread.daemon = True
    audio_thread.start()
    
    print("\n" + "="*60)
    print("🎬 系统启动成功！")
    print("="*60)
    print("\n💡 提示:")
    print("  - 音频将自动播放并同步显示口型")
    print("  - 播放完成后自动退出")
    if face_controller:
        print("  - 舵机会实时跟随音频口型")
    print("\n控制键:")
    print("  [q] 或 [ESC] - 提前退出")
    print("  [r] - 重新播放")
    print("  [p] - 显示/隐藏自适应参数")
    print("  [s] - 显示/隐藏舵机角度")
    print("="*60 + "\n")
    
    print("▶️  开始播放...")
    
    # 默认参数
    current_jaw = 0.0
    current_width = 0.5
    current_rms = 0
    current_servo_angles = (SERVO_JAW_MIN, (SERVO_LEFT_MIN + SERVO_LEFT_MAX) / 2, (SERVO_RIGHT_MIN + SERVO_RIGHT_MAX) / 2)
    playback_info = None
    
    try:
        while True:
            # 从队列获取最新参数
            try:
                result = param_queue.get_nowait()
                if len(result) == 5:
                    current_jaw, current_width, current_rms, current_servo_angles, playback_info = result
                else:
                    current_jaw, current_width, current_rms, current_servo_angles = result
            except queue.Empty:
                pass
            
            # 获取当前自适应参数
            params = controller.param_manager.get_params()
            
            # 渲染当前帧
            frame = renderer.draw_mouth(current_jaw, current_width, current_rms, params, 
                                       current_servo_angles, playback_info)
            
            # 显示画面
            cv2.imshow('Audio Playback + Servo Control', frame)
            
            # 按键检测 (等待时间与FPS对应)
            key = cv2.waitKey(int(1000 / FPS))
            if key == ord('q') or key == 27:  # 'q' 或 ESC
                break
            elif key == ord('r'):  # 重新播放
                print("\n🔄 重新播放...")
                controller.reset_audio()
                controller.playback_finished = False
            elif key == ord('p'):  # 切换参数显示
                renderer.show_params = not renderer.show_params
            elif key == ord('s'):  # 切换舵机角度显示
                renderer.show_servo_angles = not renderer.show_servo_angles
            
            # 检查是否播放完成
            if stop_event.is_set():
                print("\n⏸️  播放已停止")
                # 等待3秒后自动退出，或按任意键立即退出
                print("3秒后自动退出，或按任意键立即退出...")
                for i in range(30):  # 3秒 = 30 * 100ms
                    if cv2.waitKey(100) != -1:
                        break
                break
                
    except KeyboardInterrupt:
        print("\n接收到中断信号")
    finally:
        print("\n正在关闭...")
        stop_event.set()
        audio_thread.join(timeout=1)
        controller.close()
        if face_controller:
            face_controller.close()
        cv2.destroyAllWindows()
        print("程序已退出")
