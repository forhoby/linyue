#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include <alsa/asoundlib.h>
#include <vector>
#include <chrono>
#include <thread>

// 宏定义：选择音频设备
// 可选值：
// - "hw:0,0"：第一个音频设备
// - "hw:1,0"：第二个音频设备
// - "default"：默认音频设备
#define AUDIO_DEVICE "hw:1,0"

class SpeakerNode : public rclcpp::Node {
private:
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr subscription_;
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
    // 定时器，用于检查200ms无数据的情况
    rclcpp::TimerBase::SharedPtr check_timer_;

public:
    SpeakerNode() : Node("speaker_node"), is_recording_(false) {
        // 初始化 ALSA
        init_alsa();

        // 订阅音频回复主题
        subscription_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "/robot/audio_reply", 10, 
            std::bind(&SpeakerNode::audio_callback, this, std::placeholders::_1));

        // 创建定时器，每50ms检查一次是否超过200ms无数据
        check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&SpeakerNode::check_recording_timeout, this));

        RCLCPP_INFO(this->get_logger(), "Speaker node started, listening to /robot/audio_reply");
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
                size_t chunk_size = 320; // 每次播放 320 个样本（20ms @ 16kHz）
                size_t total_played = 0;
                bool device_error = false;
                
                while (offset < total_samples && !device_error) {
                    // 计算当前块的大小
                    size_t current_chunk = std::min(chunk_size, total_samples - offset);
                    
                    // 计算帧数（考虑声道数）
                    size_t frames = current_chunk;
                    if (channels_ == 2) {
                        // 立体声时，帧数是样本数的一半
                        frames = current_chunk / 2;
                    }
                    
                    // 检查设备状态
                    snd_pcm_state_t state = static_cast<snd_pcm_state_t>(snd_pcm_state(pcm_handle_));
                    if (state != SND_PCM_STATE_RUNNING && state != SND_PCM_STATE_PREPARED) {
                        // 设备状态不正确，尝试恢复
                        RCLCPP_WARN(this->get_logger(), "Audio device in bad state: %s, trying to recover", snd_pcm_state_name(state));
                        
                        int err = snd_pcm_prepare(pcm_handle_);
                        if (err < 0) {
                            RCLCPP_ERROR(this->get_logger(), "Failed to prepare audio device: %s", snd_strerror(err));
                            device_error = true;
                            break;
                        }
                    }
                    
                    // 播放当前块
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
                    
                    // 短暂延迟，确保音频流畅播放
                    
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
        rate_ = 16000; // 16kHz
        unsigned int channels = 2; // 尝试立体声
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
