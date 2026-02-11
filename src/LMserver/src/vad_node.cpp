#include <rclcpp/rclcpp.hpp>
#include <audio_capture/msg/audio_data.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <vector>
#include <cmath>
#include <webrtc_vad.h>
#include <chrono>
#include <memory>

class VadServerNode : public rclcpp::Node {
public:
    VadServerNode() : Node("vad_node") {
        // ==================== 参数声明 ====================
        this->declare_parameter<int>("vad_aggressiveness", 0);  // 0-3 (0: 最不敏感, 3: 最敏感)
        this->declare_parameter<int>("vad_frame_duration_ms", 30);  // 10, 20, 或 30
        this->declare_parameter<double>("silence_threshold", 0.4);  // 静音阈值（语音比例）
        this->declare_parameter<double>("min_speech_ratio", 0.4);  // 最小人声占比（判断本次录音是否有人说话）
        this->declare_parameter<double>("continuous_timeout", 3.0);  // 连续模式超时时间（秒）
        this->declare_parameter<double>("check_interval", 0.2);  // 检查间隔（秒）
        this->declare_parameter<int>("max_consecutive_silence", 5);  // 连续静音次数（停止当前录音）
        this->declare_parameter<double>("max_recording_duration", 10.0);  // 单次最大录音时长
        
        // ==================== 获取参数值 ====================
        vad_aggressiveness_ = this->get_parameter("vad_aggressiveness").as_int();
        vad_frame_duration_ms_ = this->get_parameter("vad_frame_duration_ms").as_int();
        silence_threshold_ = this->get_parameter("silence_threshold").as_double();
        min_speech_ratio_ = this->get_parameter("min_speech_ratio").as_double();
        continuous_timeout_ = this->get_parameter("continuous_timeout").as_double();
        check_interval_ = this->get_parameter("check_interval").as_double();
        max_consecutive_silence_ = this->get_parameter("max_consecutive_silence").as_int();
        max_recording_duration_ = this->get_parameter("max_recording_duration").as_double();
        
        // ==================== 初始化 VAD ====================
        vad_handle_ = WebRtcVad_Create();
        if (!vad_handle_) {
            RCLCPP_ERROR(this->get_logger(), "VAD创建失败");
            return;
        }
        
        if (WebRtcVad_Init(vad_handle_) != 0) {
            RCLCPP_ERROR(this->get_logger(), "VAD初始化失败");
            WebRtcVad_Free(vad_handle_);
            vad_handle_ = nullptr;
            return;
        }
        
        if (WebRtcVad_set_mode(vad_handle_, vad_aggressiveness_) != 0) {
            RCLCPP_ERROR(this->get_logger(), "VAD模式设置失败");
            WebRtcVad_Free(vad_handle_);
            vad_handle_ = nullptr;
            return;
        }
        
        // ==================== 创建订阅和发布者 ====================
        audio_subscriber_ = this->create_subscription<audio_capture::msg::AudioData>(
            "audio_data", 10, std::bind(&VadServerNode::audio_callback, this, std::placeholders::_1));
        
        voice_publisher_ = this->create_publisher<audio_capture::msg::AudioData>(
            "voice_data", 10);
        
        // ==================== 状态变量 ====================
        is_running_ = true;
        wake_up_ = true;  // 默认为true，直接开始检测
        continuous_mode_ = true;  // 默认为true，持续检测
        last_speech_time_ = nullptr;
        recording_start_time_ = nullptr;
        last_print_time_ = nullptr;
        total_speech_ratio_ = 0.0;
        check_count_ = 0;
        consecutive_silence_ = 0;
        current_audio_data_ = std::make_shared<std::vector<uint8_t>>();
        vad_buffer_ = std::make_shared<std::vector<uint8_t>>();  // VAD检测缓冲区
        last_period_speech_ratio_ = 0.0;
        last_period_check_count_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "VAD服务器节点启动");
        RCLCPP_INFO(this->get_logger(), "VAD灵敏度: %d", vad_aggressiveness_);
        RCLCPP_INFO(this->get_logger(), "VAD帧时长: %d ms", vad_frame_duration_ms_);
        RCLCPP_INFO(this->get_logger(), "静音阈值: %.2f", silence_threshold_);
        RCLCPP_INFO(this->get_logger(), "最小语音占比: %.2f", min_speech_ratio_);
        RCLCPP_INFO(this->get_logger(), "连续模式超时: %.1f s", continuous_timeout_);
        RCLCPP_INFO(this->get_logger(), "检查间隔: %.2f s", check_interval_);
        RCLCPP_INFO(this->get_logger(), "最大连续静音次数: %d", max_consecutive_silence_);
        RCLCPP_INFO(this->get_logger(), "最大录音时长: %.1f s", max_recording_duration_);
    }
    
    ~VadServerNode() {
        if (vad_handle_) {
            WebRtcVad_Free(vad_handle_);
        }
        if (last_speech_time_) {
            delete last_speech_time_;
        }
        if (recording_start_time_) {
            delete recording_start_time_;
        }
        if (last_print_time_) {
            delete last_print_time_;
        }
    }

private:
    void audio_callback(const audio_capture::msg::AudioData::SharedPtr msg) {
        // 判断是否可以开始录音
        bool can_record = wake_up_ || continuous_mode_;
        
        if (can_record) {
            // 开始录音时记录开始时间
            if (!recording_start_time_) {
                recording_start_time_ = new rclcpp::Time(this->now());
                consecutive_silence_ = 0;
                RCLCPP_INFO(this->get_logger(), "🎙️  开始录音");
            }
            
            // 收集音频数据
            current_audio_data_->insert(current_audio_data_->end(), msg->data.begin(), msg->data.end());
            
            // 累积音频数据用于VAD检测
            vad_buffer_->insert(vad_buffer_->end(), msg->data.begin(), msg->data.end());
            
            // 计算VAD检测所需的最小数据大小
            size_t min_data_size = static_cast<size_t>(msg->sample_rate * vad_frame_duration_ms_ / 1000) * 
                                   msg->bytes_per_sample * msg->channels;
            
            // 只有当累积的数据达到足够大小时，才进行VAD检测
            if (vad_buffer_->size() >= min_data_size) {
                // 创建临时音频消息用于VAD检测
                auto temp_msg = std::make_shared<audio_capture::msg::AudioData>();
                temp_msg->header = msg->header;
                temp_msg->sample_rate = msg->sample_rate;
                temp_msg->channels = msg->channels;
                temp_msg->format = msg->format;
                temp_msg->bytes_per_sample = msg->bytes_per_sample;
                temp_msg->data = *vad_buffer_;
                
                // 实时VAD检测
                double speech_ratio = vad_check_audio(temp_msg);
                
                // 清空VAD缓冲区，准备下一次检测
                vad_buffer_->clear();
                
                // 更新语音占比
                total_speech_ratio_ += speech_ratio;
                check_count_++;
                
                // 打印实时状态
                auto now = this->now();
                double elapsed = 0.0;
                if (recording_start_time_) {
                    elapsed = now.seconds() - recording_start_time_->seconds();
                }
                
                RCLCPP_INFO(this->get_logger(),
                    "%s %.1fs | %.1fKB | 语音: %.0f%%",
                    speech_ratio > silence_threshold_ ? "🗣️" : "🔇",
                    elapsed, current_audio_data_->size() / 1024.0, speech_ratio * 100
                );
                
                // 判断是否为静音
                if (speech_ratio < silence_threshold_) {
                    // 未检测到人声，增加连续静音计数
                    consecutive_silence_++;
                    RCLCPP_INFO(this->get_logger(), "连续静音次数: %d / %d", consecutive_silence_, max_consecutive_silence_);
                    
                    // 检查连续静音是否达到阈值
                    if (consecutive_silence_ >= max_consecutive_silence_) {
                        RCLCPP_INFO(this->get_logger(), "检测到静音，停止本次录音");
                        // 发布已录制的音频包
                        publish_audio_if_has_speech(msg);
                        
                        // 检查是否需要进入/保持连续模式
                        if (continuous_mode_) {
                            // 在连续模式下，检查是否超时
                            if (last_speech_time_) {
                                double elapsed_since_speech = now.seconds() - last_speech_time_->seconds();
                                RCLCPP_INFO(this->get_logger(),
                                    "⏱️  距上次人声: %.1f秒 / %.1f秒",
                                    elapsed_since_speech, continuous_timeout_
                                );
                                
                                if (elapsed_since_speech >= continuous_timeout_) {
                                    RCLCPP_INFO(this->get_logger(),
                                        "⏸️  %.1f秒无人声，退出连续模式",
                                        continuous_timeout_
                                    );
                                    reset_recording_state();
                                } else {
                                    RCLCPP_INFO(this->get_logger(), "🔄 继续等待人声...");
                                    // 清空缓存，准备下一次录音
                                    clear_audio_cache();
                                    consecutive_silence_ = 0;
                                    if (recording_start_time_) {
                                        delete recording_start_time_;
                                        recording_start_time_ = new rclcpp::Time(now);
                                    }
                                }
                            } else {
                                RCLCPP_INFO(this->get_logger(), "⚠️  连续模式异常，退出");
                                reset_recording_state();
                            }
                        } else {
                            // 唤醒模式下未检测到人声
                            RCLCPP_INFO(this->get_logger(), "⏸️  唤醒后无人声，等待下次唤醒");
                            reset_recording_state();
                        }
                    }
                } else {
                    // 检测到人声，重置连续静音计数
                    consecutive_silence_ = 0;
                    
                    // 更新时间戳并进入/保持连续模式
                    if (last_speech_time_) {
                        delete last_speech_time_;
                    }
                    last_speech_time_ = new rclcpp::Time(now);
                    continuous_mode_ = true;
                    wake_up_ = false;
                    RCLCPP_INFO(this->get_logger(), "🔄 检测到人声，进入连续录音模式...");
                }
            }
            
            // 检查是否达到最大录音时长
            if (recording_start_time_) {
                auto now = this->now();
                double recording_duration = now.seconds() - recording_start_time_->seconds();
                if (recording_duration >= max_recording_duration_) {
                    RCLCPP_INFO(this->get_logger(), "⏭️  达到单次最大时长，停止本次录音");
                    // 发布已录制的音频包
                    publish_audio_if_has_speech(msg);
                    // 清空缓存，准备下一次录音
                    clear_audio_cache();
                    consecutive_silence_ = 0;
                    // 重置录音开始时间，继续检测
                    delete recording_start_time_;
                    recording_start_time_ = new rclcpp::Time(now);
                }
            }
        }
    }
    
    double vad_check_audio(const audio_capture::msg::AudioData::SharedPtr msg) {
        if (!vad_handle_) {
            RCLCPP_ERROR(this->get_logger(), "VAD未初始化");
            return 0.0;
        }
        
        // 检查采样率是否支持
        int sample_rate = msg->sample_rate;
        if (sample_rate != 8000 && sample_rate != 16000 && sample_rate != 32000 && sample_rate != 48000) {
            RCLCPP_WARN(this->get_logger(), "不支持的采样率: %d", sample_rate);
            return 0.0;
        }
        
        // 获取通道数
        int channels = 1;
        if (msg->channels > 0) {
            channels = msg->channels;
        }
        
        // 计算每帧的采样数
        size_t frame_size = static_cast<size_t>(sample_rate * vad_frame_duration_ms_ / 1000);
        
        // 检查音频数据大小
        size_t data_size = msg->data.size();
        if (data_size < frame_size * msg->bytes_per_sample * channels) {
            RCLCPP_WARN(this->get_logger(), "音频数据太小，无法进行VAD检测");
            return 0.0;
        }
        
        // 转换音频数据为int16_t格式并进行通道混合
        std::vector<int16_t> samples;
        switch (msg->format) {
            case 2: // S16_LE - SND_PCM_FORMAT_S16_LE
                {
                    size_t samples_per_channel = data_size / (msg->bytes_per_sample * channels);
                    samples.reserve(samples_per_channel);
                    
                    for (size_t i = 0; i < samples_per_channel; ++i) {
                        int32_t sum = 0;
                        for (int c = 0; c < channels; ++c) {
                            size_t index = i * channels * msg->bytes_per_sample + c * msg->bytes_per_sample;
                            int16_t sample = *reinterpret_cast<const int16_t*>(&msg->data[index]);
                            sum += sample;
                        }
                        int16_t mixed = static_cast<int16_t>(sum / channels);
                        samples.push_back(mixed);
                    }
                }
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "未知音频格式: %d", msg->format);
                return 0.0;
        }
        
        // 进行VAD检测
        int speech_frames = 0;
        int total_frames = 0;
        
        for (size_t i = 0; i + frame_size <= samples.size(); i += frame_size) {
            int result = WebRtcVad_Process(
                vad_handle_,
                sample_rate,
                &samples[i],
                frame_size
            );
            
            if (result == 1) {
                speech_frames++;
            }
            total_frames++;
        }
        
        // 计算语音占比
        double ratio = total_frames > 0 ? static_cast<double>(speech_frames) / total_frames : 0.0;
        return ratio;
    }
    
    void publish_audio_if_has_speech(const audio_capture::msg::AudioData::SharedPtr msg) {
        // 计算平均语音占比
        double avg_speech_ratio = check_count_ > 0 ? total_speech_ratio_ / check_count_ : 0.0;
        // 判断是否检测到人声
        bool has_speech = avg_speech_ratio >= min_speech_ratio_;
        
        // 统计信息
        double duration = 0.0;
        if (recording_start_time_) {
            auto now = this->now();
            duration = now.seconds() - recording_start_time_->seconds();
        }
        
        RCLCPP_INFO(this->get_logger(),
            "录音完成 | 时长: %.2f秒 | 大小: %.1fKB | 平均语音占比: %.1f%%",
            duration, current_audio_data_->size() / 1024.0, avg_speech_ratio * 100
        );
        
        if (has_speech && !current_audio_data_->empty()) {
            // ✅ 检测到人声，发布音频
            auto voice_msg = std::make_shared<audio_capture::msg::AudioData>();
            voice_msg->header = msg->header;
            voice_msg->sample_rate = msg->sample_rate;
            voice_msg->channels = msg->channels;
            voice_msg->format = msg->format;
            voice_msg->bytes_per_sample = msg->bytes_per_sample;
            voice_msg->data = *current_audio_data_;
            
            voice_publisher_->publish(*voice_msg);
            
            RCLCPP_INFO(this->get_logger(),
                "✅ 音频已发布 | %.2f秒 | %.1fKB",
                duration, current_audio_data_->size() / 1024.0
            );
        } else {
            // ❌ 未检测到人声，不发布音频
            RCLCPP_INFO(this->get_logger(), "⏭️  未检测到人声，跳过音频发布");
        }
    }
    
    void reset_recording_state() {
        // 重置状态变量，可被再次唤醒
        continuous_mode_ = true;  // 保持连续模式
        wake_up_ = true;  // 保持唤醒状态
        if (last_speech_time_) {
            delete last_speech_time_;
            last_speech_time_ = nullptr;
        }
        if (recording_start_time_) {
            delete recording_start_time_;
            recording_start_time_ = nullptr;
        }
        clear_audio_cache();
        RCLCPP_INFO(this->get_logger(), "🔄 重置录音状态，继续检测");
    }
    
    void clear_audio_cache() {
        // 清空本地的音频缓存
        current_audio_data_->clear();
        vad_buffer_->clear();  // 同时清空VAD缓冲区
        total_speech_ratio_ = 0.0;
        check_count_ = 0;
        consecutive_silence_ = 0;
        last_period_speech_ratio_ = 0.0;
        last_period_check_count_ = 0;
        RCLCPP_INFO(this->get_logger(), "🗑️  清空音频缓存");
    }
    
    rclcpp::Subscription<audio_capture::msg::AudioData>::SharedPtr audio_subscriber_;
    rclcpp::Publisher<audio_capture::msg::AudioData>::SharedPtr voice_publisher_;
    
    // WebRTC VAD 相关
    VadInst* vad_handle_;
    int vad_aggressiveness_;
    int vad_frame_duration_ms_;
    
    // 检测相关参数
    double silence_threshold_;
    double min_speech_ratio_;
    double continuous_timeout_;
    double check_interval_;
    int max_consecutive_silence_;
    double max_recording_duration_;
    
    // 状态变量
    bool is_running_;
    bool wake_up_;
    bool continuous_mode_;
    rclcpp::Time* last_speech_time_;
    rclcpp::Time* recording_start_time_;
    rclcpp::Time* last_print_time_;
    double total_speech_ratio_;
    int check_count_;
    int consecutive_silence_;
    double last_period_speech_ratio_;
    int last_period_check_count_;
    std::shared_ptr<std::vector<uint8_t>> current_audio_data_;
    std::shared_ptr<std::vector<uint8_t>> vad_buffer_;  // VAD检测缓冲区
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VadServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}