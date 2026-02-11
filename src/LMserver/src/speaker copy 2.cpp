#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <alsa/asoundlib.h>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>
#include <complex>
#include <valarray>
#include <deque>

// 宏定义：选择音频设备
// 可选值：
// - "hw:0,0"：第一个音频设备
// - "hw:1,0"：第二个音频设备
// - "default"：默认音频设备
#define AUDIO_DEVICE "hw:1,0"

// 音频处理参数
#define CHUNK_SIZE 1323             // 每次处理的音频帧大小（60ms @ 22050Hz）
#define SMOOTHING_JAW 0.6           // 舵机平滑系数
#define SMOOTHING_CORNER 0.6        // 舵机平滑系数

// 自适应参数设置
#define ADAPTIVE_WINDOW_SIZE 150    // 用于统计的历史帧数
#define ADAPTIVE_UPDATE_INTERVAL 30 // 每N帧更新一次参数

// 舵机参数设置
#define SERVO_JAW_MIN 1300           // 嘴部张合最小参数（闭合）
#define SERVO_JAW_MAX 1600           // 嘴部张合最大参数（张开）
#define SERVO_LEFT_MIN 1300           // 左嘴角最小参数（内聚）
#define SERVO_LEFT_MAX 1800          // 左嘴角最大参数（外展）
#define SERVO_RIGHT_MIN 1700          // 右嘴角最小参数（外展）
#define SERVO_RIGHT_MAX 1300        // 右嘴角最大参数（内收）

// 舵机索引
#define SERVO_JAW_INDEX 10          // 嘴部张合舵机索引
#define SERVO_LEFT_INDEX 9          // 左嘴角舵机索引
#define SERVO_RIGHT_INDEX 8         // 右嘴角舵机索引

// 舵机更新频率限制
#define SERVO_UPDATE_INTERVAL_MS 50 // 最小更新间隔（毫秒）

// 音频处理辅助函数
// 计算RMS（均方根）
double calculate_rms(const std::vector<int16_t>& audio_data) {
    double sum = 0.0;
    for (int16_t sample : audio_data) {
        sum += static_cast<double>(sample) * sample;
    }
    return std::sqrt(sum / audio_data.size());
}

// FFT实现
typedef std::complex<double> Complex;
typedef std::valarray<Complex> CArray;

void fft(CArray& x) {
    const size_t N = x.size();
    if (N <= 1) return;

    // 分离偶数和奇数索引
    CArray even = x[std::slice(0, N/2, 2)];
    CArray odd = x[std::slice(1, N/2, 2)];

    // 递归计算FFT
    fft(even);
    fft(odd);

    // 合并结果
    for (size_t k = 0; k < N/2; ++k) {
        Complex t = std::polar(1.0, -2 * M_PI * k / N) * odd[k];
        x[k] = even[k] + t;
        x[k + N/2] = even[k] - t;
    }
}

// 计算频谱质心
double calculate_spectral_centroid(const CArray& fft_result, double sample_rate) {
    double sum_freq = 0.0;
    double sum_mag = 0.0;
    size_t N = fft_result.size();

    for (size_t i = 0; i < N; ++i) {
        double freq = static_cast<double>(i) * sample_rate / (2 * N);
        double mag = std::abs(fft_result[i]);
        
        // 只考虑人声频段 (400Hz - 4000Hz)
        if (freq >= 400.0 && freq <= 4000.0) {
            sum_freq += freq * mag;
            sum_mag += mag;
        }
    }

    if (sum_mag == 0.0) {
        return 1500.0; // 默认值
    }

    return sum_freq / sum_mag;
}

class SpeakerNode : public rclcpp::Node {
private:
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr servo_publisher_;
    // 音频设备
    snd_pcm_t *pcm_handle_;
    snd_pcm_stream_t stream_;
    snd_pcm_format_t format_;
    unsigned int rate_;
    unsigned int channels_;
    unsigned int frames_;
    
    // 音频缓冲区
    std::vector<int16_t> audio_buffer_;
    // 记录状态
    bool is_recording_;
    // 时间跟踪
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point last_callback_time_;
    std::chrono::steady_clock::time_point last_servo_update_time_;
    // 定时器，用于检查200ms无数据的情况
    rclcpp::TimerBase::SharedPtr check_timer_;
    
    // 音频处理相关
    std::deque<double> rms_history_;
    std::deque<double> centroid_history_;
    int frame_count_;
    bool is_warming_up_;
    
    // 自适应参数
    double noise_gate_;
    double max_rms_;
    double min_freq_;
    double max_freq_;
    
    // 当前口型参数
    double current_jaw_;
    double current_width_;
    
    // 频率轴（用于FFT）
    std::vector<double> freqs_;

public:
    SpeakerNode() : Node("speaker_node"), 
                    is_recording_(false), 
                    frame_count_(0), 
                    is_warming_up_(true), 
                    noise_gate_(20.0), 
                    max_rms_(80.0), 
                    min_freq_(400.0), 
                    max_freq_(4000.0), 
                    current_jaw_(0.0), 
                    current_width_(0.5) {
        // 初始化 ALSA
        init_alsa();
        
        // 订阅音频回复主题
        subscription_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "/robot/audio_reply", 10, 
            std::bind(&SpeakerNode::audio_callback, this, std::placeholders::_1));
        
        // 创建舵机控制发布者
        servo_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/robot/servo_angles", 10);

        // 创建定时器，每50ms检查一次是否超过200ms无数据
        check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&SpeakerNode::check_recording_timeout, this));

        // 初始化频率轴
        init_freq_axis();

        RCLCPP_INFO(this->get_logger(), "Speaker node started, listening to /robot/audio_reply");
        RCLCPP_INFO(this->get_logger(), "Servo control publisher initialized, publishing to /robot/servo_angles");
    }

    ~SpeakerNode() {
        // 关闭音频设备
        if (pcm_handle_) {
            snd_pcm_drain(pcm_handle_);
            snd_pcm_close(pcm_handle_);
        }
    }

private:
    void init_alsa() {
        // 初始化音频设备（使用宏定义的设备）
        init_alsa_device(&pcm_handle_, AUDIO_DEVICE, "Audio Device");
    }
    
    void init_freq_axis() {
        // 初始化频率轴
        size_t n = CHUNK_SIZE;
        freqs_.resize(n / 2 + 1);
        for (size_t i = 0; i <= n / 2; ++i) {
            freqs_[i] = static_cast<double>(i) * rate_ / n;
        }
    }
    
    void update_adaptive_parameters() {
        // 更新噪声门限
        if (rms_history_.size() > 20) {
            // 找出最低的20%作为"安静"样本
            std::vector<double> sorted_rms(rms_history_.begin(), rms_history_.end());
            std::sort(sorted_rms.begin(), sorted_rms.end());
            size_t quiet_samples_count = sorted_rms.size() / 5;
            double quiet_mean = 0.0;
            for (size_t i = 0; i < quiet_samples_count; ++i) {
                quiet_mean += sorted_rms[i];
            }
            quiet_mean /= quiet_samples_count;
            
            double quiet_std = 0.0;
            for (size_t i = 0; i < quiet_samples_count; ++i) {
                quiet_std += std::pow(sorted_rms[i] - quiet_mean, 2);
            }
            quiet_std = std::sqrt(quiet_std / quiet_samples_count);
            
            double new_noise_gate = quiet_mean + quiet_std;
            noise_gate_ = noise_gate_ * 0.9 + new_noise_gate * 0.1;
        }
        
        // 更新最大RMS
        if (rms_history_.size() > 20) {
            std::vector<double> speech_rms;
            for (double rms : rms_history_) {
                if (rms > noise_gate_) {
                    speech_rms.push_back(rms);
                }
            }
            
            if (speech_rms.size() > 10) {
                std::sort(speech_rms.begin(), speech_rms.end());
                size_t idx = static_cast<size_t>(speech_rms.size() * 0.9);
                double new_max_rms = speech_rms[idx];
                new_max_rms = std::max(new_max_rms, noise_gate_ * 3);
                max_rms_ = max_rms_ * 0.9 + new_max_rms * 0.1;
            }
        }
    }
    
    void process_audio_chunk(const std::vector<int16_t>& audio_chunk) {
        // 计算RMS
        double rms = calculate_rms(audio_chunk);
        
        // 准备FFT输入
        size_t n = CHUNK_SIZE;
        CArray data(n);
        for (size_t i = 0; i < n && i < audio_chunk.size(); ++i) {
            data[i] = static_cast<double>(audio_chunk[i]);
        }
        for (size_t i = audio_chunk.size(); i < n; ++i) {
            data[i] = 0.0;
        }
        
        // 应用汉宁窗
        for (size_t i = 0; i < n; ++i) {
            double window = 0.5 * (1 - std::cos(2 * M_PI * i / (n - 1)));
            data[i] *= window;
        }
        
        // 执行FFT
        fft(data);
        
        // 计算频谱质心
        double centroid = calculate_spectral_centroid(data, rate_);
        
        // 更新历史数据
        rms_history_.push_back(rms);
        if (rms_history_.size() > ADAPTIVE_WINDOW_SIZE) {
            rms_history_.pop_front();
        }
        
        centroid_history_.push_back(centroid);
        if (centroid_history_.size() > ADAPTIVE_WINDOW_SIZE) {
            centroid_history_.pop_front();
        }
        
        frame_count_++;
        
        // 定期更新自适应参数
        if (frame_count_ % ADAPTIVE_UPDATE_INTERVAL == 0 && rms_history_.size() > 20) {
            update_adaptive_parameters();
            if (frame_count_ >= 30) {
                is_warming_up_ = false;
            }
        }
        
        // 计算口型参数
        double target_jaw = 0.0;
        double target_width = 0.5;
        
        if (rms > noise_gate_) {
            // 计算嘴巴张合程度
            target_jaw = std::clamp((rms - noise_gate_) / (max_rms_ - noise_gate_), 0.0, 1.0);
            target_jaw = std::pow(target_jaw, 0.7); // 稍微放大一点小信号
            
            // 计算嘴巴宽度
            // 高频(>1500Hz)通常对应 'i', 'e' (嘴角拉开)
            // 低频(<800Hz) 通常对应 'u', 'o' (嘴角收缩)
            target_width = std::clamp((centroid - 500.0) / 2000.0, 0.0, 1.0);
        }
        
        // 平滑滤波
        current_jaw_ = current_jaw_ * (1 - SMOOTHING_JAW) + target_jaw * SMOOTHING_JAW;
        current_width_ = current_width_ * (1 - SMOOTHING_CORNER) + target_width * SMOOTHING_CORNER;
        
        // 控制舵机
        update_servos();
    }
    
    void update_servos() {
        // 检查是否达到更新间隔
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_servo_update_time_).count();
        
        if (elapsed < SERVO_UPDATE_INTERVAL_MS) {
            return;
        }
        
        // 计算舵机参数
        int jaw_param = SERVO_JAW_MIN + static_cast<int>((SERVO_JAW_MAX - SERVO_JAW_MIN) * current_jaw_);
        int left_param = SERVO_LEFT_MIN + static_cast<int>((SERVO_LEFT_MAX - SERVO_LEFT_MIN) * current_width_);
        int right_param = SERVO_RIGHT_MAX - static_cast<int>((SERVO_RIGHT_MAX - SERVO_RIGHT_MIN) * current_width_);
        
        // 发布舵机参数
        auto msg = std::make_unique<std_msgs::msg::Int32MultiArray>();
        msg->data.resize(3);
        msg->data[0] = jaw_param;
        msg->data[1] = left_param;
        msg->data[2] = right_param;
        
        servo_publisher_->publish(std::move(msg));
        
        // 更新最后更新时间
        last_servo_update_time_ = now;
    }

    // 检查录制超时
    void check_recording_timeout() {
        if (is_recording_) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_callback_time_).count();
            
            if (elapsed >= 200) {
                // 超过200ms无数据，结束录制并播放
                finish_recording();
            }
        }
    }

    // 结束录制并播放音频
    void finish_recording() {
        if (is_recording_ && !audio_buffer_.empty()) {
            is_recording_ = false;
            
            RCLCPP_INFO(this->get_logger(), "Recording finished, playing buffered audio...");
            RCLCPP_INFO(this->get_logger(), "Buffer size: %zu samples", audio_buffer_.size());
            
            // 播放缓冲的音频
            if (pcm_handle_) {
                // 分块播放音频数据，每次播放合适的大小
                size_t total_samples = audio_buffer_.size();
                size_t offset = 0;
                size_t chunk_size = 1323; // 每次播放 1323 个样本（60ms @ 22050Hz）
                size_t total_played = 0;
                bool device_error = false;
                
                // 确保设备处于准备状态
                int err = snd_pcm_prepare(pcm_handle_);
                if (err < 0) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to prepare audio device: %s", snd_strerror(err));
                    device_error = true;
                }
                
                if (!device_error) {
                    while (offset < total_samples && !device_error) {
                        // 计算当前块的大小
                        size_t current_chunk = std::min(chunk_size, total_samples - offset);
                        
                        // 计算帧数（考虑声道数）
                        size_t frames = current_chunk;
                        if (channels_ == 2) {
                            // 立体声时，帧数是样本数的一半
                            frames = current_chunk / 2;
                        }
                        
                        // 提取当前块的音频数据用于处理
                        std::vector<int16_t> chunk_data(audio_buffer_.begin() + offset, audio_buffer_.begin() + offset + current_chunk);
                        
                        // 先处理音频数据，计算RMS和频谱质心，并控制舵机
                        process_audio_chunk(chunk_data);
                        
                        // 非阻塞式播放当前块
                        snd_pcm_sframes_t frames_written = snd_pcm_writei(
                            pcm_handle_, 
                            audio_buffer_.data() + offset, 
                            frames
                        );

                        if (frames_written < 0) {
                            // 处理错误
                            RCLCPP_WARN(this->get_logger(), "Error writing to audio device: %s, trying to recover", snd_strerror(frames_written));
                            frames_written = snd_pcm_recover(pcm_handle_, frames_written, 0);
                            if (frames_written < 0) {
                                RCLCPP_ERROR(this->get_logger(), "Failed to recover audio device: %s", snd_strerror(frames_written));
                                device_error = true;
                                break;
                            }
                        } else {
                            // 计算实际播放的样本数
                            size_t played_samples = frames_written;
                            if (channels_ == 2) {
                                played_samples = frames_written * 2;
                            }
                            
                            // 更新已播放的样本数和偏移量
                            total_played += played_samples;
                            offset += played_samples;
                            
                            RCLCPP_DEBUG(this->get_logger(), "Played chunk: %zu frames, %zu samples, total played: %zu samples", frames_written, played_samples, total_played);
                        }
                        
                        
                    }
                    
                    // 所有音频块播放完成后，阻塞等待剩余数据播放完成
                    int drain_err = snd_pcm_drain(pcm_handle_);
                    if (drain_err < 0) {
                        RCLCPP_WARN(this->get_logger(), "Error draining audio device: %s", snd_strerror(drain_err));
                    }
                }
                
                if (total_played > 0) {
                    RCLCPP_INFO(this->get_logger(), "Played %zu samples total", total_played);
                }
                
                if (device_error) {
                    RCLCPP_ERROR(this->get_logger(), "Audio device error occurred during playback");
                }
            }
            
            // 清空缓冲区
            audio_buffer_.clear();
        }
    }

    void init_alsa_device(snd_pcm_t **pcm_handle, const char *device, const char *device_name) {
        int err;

        // 打开 PCM 设备
        stream_ = SND_PCM_STREAM_PLAYBACK;
        err = snd_pcm_open(pcm_handle, device, stream_, 0);
        if (err < 0) {
            RCLCPP_WARN(this->get_logger(), "Warning: Could not open %s PCM device: %s", device_name, snd_strerror(err));
            *pcm_handle = nullptr;
            return;
        }

        // 配置 PCM 参数
        format_ = SND_PCM_FORMAT_S16_LE; // 16位 PCM
        rate_ = 24000; // 16kHz
        unsigned int channels = 1; // 尝试立体声
        frames_ = 320; // 帧大小

        snd_pcm_hw_params_t *hw_params;
        snd_pcm_hw_params_alloca(&hw_params);

        err = snd_pcm_hw_params_any(*pcm_handle, hw_params);
        if (err < 0) {
            RCLCPP_WARN(this->get_logger(), "Warning: Could not get hw params for %s: %s", device_name, snd_strerror(err));
            snd_pcm_close(*pcm_handle);
            *pcm_handle = nullptr;
            return;
        }

        err = snd_pcm_hw_params_set_access(*pcm_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
        if (err < 0) {
            RCLCPP_WARN(this->get_logger(), "Warning: Could not set access for %s: %s", device_name, snd_strerror(err));
            snd_pcm_close(*pcm_handle);
            *pcm_handle = nullptr;
            return;
        }

        err = snd_pcm_hw_params_set_format(*pcm_handle, hw_params, format_);
        if (err < 0) {
            RCLCPP_WARN(this->get_logger(), "Warning: Could not set format for %s: %s", device_name, snd_strerror(err));
            snd_pcm_close(*pcm_handle);
            *pcm_handle = nullptr;
            return;
        }

        err = snd_pcm_hw_params_set_rate_near(*pcm_handle, hw_params, &rate_, 0);
        if (err < 0) {
            RCLCPP_WARN(this->get_logger(), "Warning: Could not set rate for %s: %s", device_name, snd_strerror(err));
            snd_pcm_close(*pcm_handle);
            *pcm_handle = nullptr;
            return;
        }

        err = snd_pcm_hw_params_set_channels(*pcm_handle, hw_params, channels);
        if (err < 0) {
            RCLCPP_WARN(this->get_logger(), "Warning: Could not set channels for %s: %s", device_name, snd_strerror(err));
            // 尝试单声道
            channels = 1;
            err = snd_pcm_hw_params_set_channels(*pcm_handle, hw_params, channels);
            if (err < 0) {
                RCLCPP_WARN(this->get_logger(), "Warning: Could not set mono channels for %s: %s", device_name, snd_strerror(err));
                snd_pcm_close(*pcm_handle);
                *pcm_handle = nullptr;
                return;
            }
        }

        // 更新类成员变量
        channels_ = channels;

        err = snd_pcm_hw_params(*pcm_handle, hw_params);
        if (err < 0) {
            RCLCPP_WARN(this->get_logger(), "Warning: Could not set hw params for %s: %s", device_name, snd_strerror(err));
            snd_pcm_close(*pcm_handle);
            *pcm_handle = nullptr;
            return;
        }

        RCLCPP_INFO(this->get_logger(), "%s initialized: %s, %dHz, %d channels", device_name, device, rate_, channels);
    }

    void audio_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        try {
            const std::vector<int16_t> &audio_data = msg->data;
            size_t data_size = audio_data.size();

            if (data_size == 0) {
                return;
            }

            // 记录时间
            auto now = std::chrono::steady_clock::now();
            
            // 检查是否是 200ms 间隔后的新音频
            if (is_recording_) {
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - last_callback_time_).count();
                
                if (elapsed >= 1000) {
                    // 超过200ms，打断当前播放，清空缓冲
                    RCLCPP_INFO(this->get_logger(), "200ms gap detected, interrupting playback and clearing buffer");
                    
                    // 打断当前播放（通过 drain 操作）
                    if (pcm_handle_) {
                        snd_pcm_drain(pcm_handle_);
                    }
                    
                    // 清空缓冲
                    audio_buffer_.clear();
                    is_recording_ = false;
                }
            }
            
            if (!is_recording_) {
                // 开始新的录制
                is_recording_ = true;
                start_time_ = now;
                last_callback_time_ = now;
                audio_buffer_.clear();
                RCLCPP_INFO(this->get_logger(), "Started recording new audio segment");
            } else {
                // 更新最后回调时间
                last_callback_time_ = now;
            }

            // 将音频数据添加到缓冲区
            audio_buffer_.insert(audio_buffer_.end(), audio_data.begin(), audio_data.end());
            RCLCPP_DEBUG(this->get_logger(), "Added %zu samples to buffer, total buffer size: %zu", data_size, audio_buffer_.size());

        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error in audio callback: %s", e.what());
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpeakerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
