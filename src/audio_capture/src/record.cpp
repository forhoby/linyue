#include <rclcpp/rclcpp.hpp>
#include <audio_capture/msg/audio_data.hpp>
#include <std_msgs/msg/header.hpp>
#include <alsa/asoundlib.h>
#include <cstring>
#include <memory>
#include <vector>
#include <thread>
#include <chrono>
#define HighPassFilterEnabled 0
#define TargetSampleRate 16000
#define macro "hw:2,0"
// 简单的高通滤波器类
class HighPassFilter {
private:
    float cutoff_freq_;    // 截止频率
    float sample_rate_;     // 采样率
    float a1_, b0_, b1_;    // 滤波器系数
    float x1_, y1_;         // 延迟样本

public:
    HighPassFilter(float cutoff_freq, float sample_rate) {
        cutoff_freq_ = cutoff_freq;
        sample_rate_ = sample_rate;
        x1_ = 0.0f;
        y1_ = 0.0f;
        calculate_coefficients();
    }

    void calculate_coefficients() {
        float wc = 2.0f * M_PI * cutoff_freq_ / sample_rate_;
        float alpha = sin(wc) / (2.0f * 0.707f); // 0.707是Q值，用于巴特沃斯滤波器

        b0_ = 1.0f / (1.0f + alpha);
        b1_ = -b0_;
        a1_ = (1.0f - alpha) * b0_;
    }

    float process(float x) {
        float y = b0_ * x + b1_ * x1_ - a1_ * y1_;
        x1_ = x;
        y1_ = y;
        return y;
    }

    void reset() {
        x1_ = 0.0f;
        y1_ = 0.0f;
    }
};

class AudioRecorderNode : public rclcpp::Node {
public:
    AudioRecorderNode() : Node("audio_recorder_node") {
        this->declare_parameter<std::string>("device", macro);
        this->declare_parameter<int>("sample_rate", 16000);
        this->declare_parameter<int>("channels", 1);
        this->declare_parameter<int>("format", SND_PCM_FORMAT_S16_LE);
        this->declare_parameter<int>("frames", 512);

        device_ = this->get_parameter("device").as_string();
        sample_rate_ = static_cast<unsigned int>(this->get_parameter("sample_rate").as_int());
        channels_ = static_cast<unsigned int>(this->get_parameter("channels").as_int());
        format_ = this->get_parameter("format").as_int();
        frames_ = this->get_parameter("frames").as_int();

        // 根据宏定义决定是否初始化高通滤波器
        #if HighPassFilterEnabled
        hp_filter_ = std::make_unique<HighPassFilter>(150.0f, static_cast<float>(sample_rate_));
        #endif

        audio_publisher_ = this->create_publisher<audio_capture::msg::AudioData>("audio_data", 10);

        RCLCPP_INFO(this->get_logger(), "采集节点启动");
        RCLCPP_INFO(this->get_logger(), "设备: %s", device_.c_str());
        RCLCPP_INFO(this->get_logger(), "采样率: %d Hz", sample_rate_);
        RCLCPP_INFO(this->get_logger(), "目标采样率: %d Hz", target_sample_rate);

        RCLCPP_INFO(this->get_logger(), "声道数: %d", channels_);
        #if HighPassFilterEnabled
        RCLCPP_INFO(this->get_logger(), "已启用高通滤波器，截止频率: 150Hz");
        #else
        RCLCPP_INFO(this->get_logger(), "高通滤波器已禁用");
        #endif

        if (!init_alsa()) {
            RCLCPP_ERROR(this->get_logger(), "ALSA 初始化失败");
            return;
        }

        recording_thread_ = std::thread(&AudioRecorderNode::record_audio, this);
    }

    ~AudioRecorderNode() {
        stop_recording_ = true;
        if (recording_thread_.joinable()) {
            recording_thread_.join();
        }
        if (capture_handle_) {
            snd_pcm_drain(capture_handle_);
            snd_pcm_close(capture_handle_);
        }
    }

private:
    bool init_alsa() {
        int err;
        snd_pcm_hw_params_t *hw_params;

        err = snd_pcm_open(&capture_handle_, device_.c_str(), SND_PCM_STREAM_CAPTURE, 0);
        if (err < 0) {
            RCLCPP_ERROR(this->get_logger(), "无法打开音频设备 %s: %s", device_.c_str(), snd_strerror(err));
            return false;
        }

        snd_pcm_hw_params_alloca(&hw_params);
        snd_pcm_hw_params_any(capture_handle_, hw_params);
        snd_pcm_hw_params_set_access(capture_handle_, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
        snd_pcm_hw_params_set_format(capture_handle_, hw_params, static_cast<snd_pcm_format_t>(format_));
        snd_pcm_hw_params_set_channels(capture_handle_, hw_params, static_cast<unsigned int>(channels_));
        snd_pcm_hw_params_set_rate_near(capture_handle_, hw_params, &sample_rate_, 0);
        snd_pcm_hw_params_set_period_size_near(capture_handle_, hw_params, &frames_, 0);

        err = snd_pcm_hw_params(capture_handle_, hw_params);
        if (err < 0) {
            RCLCPP_ERROR(this->get_logger(), "无法设置硬件参数: %s", snd_strerror(err));
            return false;
        }

        snd_pcm_hw_params_get_period_size(hw_params, &frames_, 0);
        snd_pcm_hw_params_get_period_time(hw_params, &period_time_, 0);

        int bytes_per_sample = 0;
        switch (format_) {
            case SND_PCM_FORMAT_U8:
                bytes_per_sample = 1;
                break;
            case SND_PCM_FORMAT_S16_LE:
            case SND_PCM_FORMAT_S16_BE:
                bytes_per_sample = 2;
                break;
            case SND_PCM_FORMAT_S24_LE:
            case SND_PCM_FORMAT_S24_BE:
                bytes_per_sample = 3;
                break;
            case SND_PCM_FORMAT_S32_LE:
            case SND_PCM_FORMAT_S32_BE:
            case SND_PCM_FORMAT_FLOAT_LE:
            case SND_PCM_FORMAT_FLOAT_BE:
                bytes_per_sample = 4;
                break;
            default:
                bytes_per_sample = 2;
        }
        
        buffer_size_ = frames_ * channels_ * bytes_per_sample;
        buffer_.resize(buffer_size_);

        RCLCPP_INFO(this->get_logger(), "ALSA 初始化成功，周期大小: %ld 帧", frames_);
        return true;
    }

    bool recover_alsa() {
        /* 尝试恢复ALSA设备连接 */
        RCLCPP_INFO(this->get_logger(), "尝试恢复 ALSA 设备连接...");
        
        // 关闭当前的捕获句柄
        if (capture_handle_) {
            snd_pcm_close(capture_handle_);
            capture_handle_ = nullptr;
        }
        
        // 等待一段时间后重新初始化
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        // 重新初始化ALSA
        return init_alsa();
    }

    void record_audio() {
        RCLCPP_INFO(this->get_logger(), "开始采集音频...");
        
        int bytes_per_sample = 0;
        switch (format_) {
            case SND_PCM_FORMAT_U8:
                bytes_per_sample = 1;
                break;
            case SND_PCM_FORMAT_S16_LE:
            case SND_PCM_FORMAT_S16_BE:
                bytes_per_sample = 2;
                break;
            case SND_PCM_FORMAT_S24_LE:
            case SND_PCM_FORMAT_S24_BE:
                bytes_per_sample = 3;
                break;
            case SND_PCM_FORMAT_S32_LE:
            case SND_PCM_FORMAT_S32_BE:
            case SND_PCM_FORMAT_FLOAT_LE:
            case SND_PCM_FORMAT_FLOAT_BE:
                bytes_per_sample = 4;
                break;
            default:
                bytes_per_sample = 2;
        }
        
        // 目标采样率（16k）
        
        // 目标声道数（单声道）
        const uint16_t target_channels = 1;
        
        while (rclcpp::ok() && !stop_recording_) {
            auto start_time = std::chrono::steady_clock::now();
            
            int err = snd_pcm_readi(capture_handle_, buffer_.data(), frames_);
            if (err == -EPIPE) {
                RCLCPP_WARN(this->get_logger(), "缓冲区溢出");
                snd_pcm_prepare(capture_handle_);
            } else if (err < 0) {
                RCLCPP_ERROR(this->get_logger(), "读取错误: %s", snd_strerror(err));
                // 尝试恢复连接
                if (recover_alsa()) {
                    RCLCPP_INFO(this->get_logger(), "ALSA 设备已成功恢复，继续采集");
                    continue;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "无法恢复 ALSA 设备，停止采集");
                    break;
                }
            } else if (err != (int)frames_) {
                RCLCPP_WARN(this->get_logger(), "短读取，读取了 %d 帧", err);
            }
            
            auto read_time = std::chrono::steady_clock::now();

            auto message = audio_capture::msg::AudioData();
            message.header.stamp = this->now();
            message.header.frame_id = "audio_frame";
            message.sample_rate = target_sample_rate; // 使用目标采样率（16k）
            message.channels = target_channels; // 转换为
            message.format = static_cast<uint8_t>(format_); // 直接使用ALSA格式值
            message.bytes_per_sample = bytes_per_sample;
            
            // 处理音频数据
            size_t data_size = err * channels_ * bytes_per_sample;
            
            // 检查是否已经是单通道16k的数据
            if (sample_rate_ == target_sample_rate && channels_ == target_channels) {
                // 已经是单通道16k的数据，直接使用原始数据
                RCLCPP_DEBUG(this->get_logger(), "已经是单通道16k数据，直接使用原始数据");
                message.data.assign(reinterpret_cast<uint8_t*>(buffer_.data()), 
                                   reinterpret_cast<uint8_t*>(buffer_.data()) + data_size);
                audio_publisher_->publish(message);
                continue;
            } else {
                if (bytes_per_sample == 2) { // 处理16位有符号整数格式
                    // 1. 首先将8声道降为单通道（减少计算量）
                    size_t num_frames = data_size / (channels_ * bytes_per_sample);
                    std::vector<float> mono_float(num_frames);
                    
                    for (size_t frame = 0; frame < num_frames; frame++) {
                        // 计算所有声道的平均值作为单通道
                        float sum = 0.0f;
                        for (uint16_t ch = 0; ch < channels_; ch++) {
                            size_t index = frame * channels_ + ch;
                            int16_t sample = *reinterpret_cast<int16_t*>(buffer_.data() + index * bytes_per_sample);
                            sum += sample / 32768.0f;
                        }
                        // 先放大30倍
                        mono_float[frame] = (sum / channels_) * 30.0f;
                    }
                    
                    // 直接使用单通道数据，不需要重采样
                    std::vector<int16_t> mono_data(mono_float.size());
                    for (size_t i = 0; i < mono_float.size(); i++) {
                        float sample = mono_float[i];
                        // 根据宏定义决定是否应用高通滤波
                        #if HighPassFilterEnabled
                        sample = hp_filter_->process(sample);
                        #endif
                        // 转换为int16_t（使用更快的整数转换方式）
                        int32_t output_sample = static_cast<int32_t>(sample * 32768.0f);
                        // 溢出保护
                        if (output_sample > 32767) {
                            output_sample = 32767;
                        } else if (output_sample < -32768) {
                            output_sample = -32768;
                        }
                        mono_data[i] = static_cast<int16_t>(output_sample);
                    }
                    message.data.assign(reinterpret_cast<uint8_t*>(mono_data.data()), 
                                       reinterpret_cast<uint8_t*>(mono_data.data()) + mono_data.size() * bytes_per_sample);
                } else {
                    // 对于其他格式，直接使用原始数据
                    message.data.assign(reinterpret_cast<uint8_t*>(buffer_.data()), 
                                       reinterpret_cast<uint8_t*>(buffer_.data()) + data_size);
                    audio_publisher_->publish(message);
                    continue;
                }
            }
            
            auto process_time = std::chrono::steady_clock::now();
            
            audio_publisher_->publish(message);
            
            auto publish_time = std::chrono::steady_clock::now();
            
            // 计算各阶段耗时（毫秒）
            auto read_duration = std::chrono::duration_cast<std::chrono::milliseconds>(read_time - start_time).count();
            auto process_duration = std::chrono::duration_cast<std::chrono::milliseconds>(process_time - read_time).count();
            auto publish_duration = std::chrono::duration_cast<std::chrono::milliseconds>(publish_time - process_time).count();
            auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(publish_time - start_time).count();
            
            // 每100帧打印一次性能统计
            static int frame_count = 0;
            if (++frame_count % 100 == 0) {
                RCLCPP_INFO(this->get_logger(), "性能统计: 读取=%ldms, 处理=%ldms, 发布=%ldms, 总计=%ldms", 
                            read_duration, process_duration, publish_duration, total_duration);
            }
        }
    }

    std::string device_;
    unsigned int sample_rate_;
    unsigned int channels_;
    int format_;
    snd_pcm_uframes_t frames_;
    unsigned int period_time_;
    std::vector<uint8_t> buffer_;
    size_t buffer_size_;
    const uint32_t target_sample_rate = TargetSampleRate;
    // 高通滤波器，用于减少低频噪声
    std::unique_ptr<HighPassFilter> hp_filter_;

    snd_pcm_t *capture_handle_ = nullptr;
    rclcpp::Publisher<audio_capture::msg::AudioData>::SharedPtr audio_publisher_;
    std::thread recording_thread_;
    bool stop_recording_ = false;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AudioRecorderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
