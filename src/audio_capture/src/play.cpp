#include <rclcpp/rclcpp.hpp>
#include <audio_capture/msg/audio_data.hpp>
#include <std_msgs/msg/header.hpp>
#include <fstream>
#include <vector>
#include <chrono>
#include <atomic>

class AudioPlayerNode : public rclcpp::Node {
public:
    AudioPlayerNode() : Node("audio_player_node") {
        audio_subscriber_ = this->create_subscription<audio_capture::msg::AudioData>(
            "audio_data", 10, 
            std::bind(&AudioPlayerNode::audio_callback, this, std::placeholders::_1)
        );

        start_time_ = std::chrono::steady_clock::now();
        is_recording_ = true;
        audio_config_received_ = true;
        RCLCPP_INFO(this->get_logger(), "音频播放节点启动，开始订阅音频数据");
        RCLCPP_INFO(this->get_logger(), "将采集10秒音频数据并保存为本地文件");
    }

    ~AudioPlayerNode() {
        if (is_recording_) {
            save_audio_file();
        }
    }

private:
    void audio_callback(const audio_capture::msg::AudioData::SharedPtr msg) {
        if (!is_recording_) {
            return;
        }

        // 保存第一次接收到的消息的配置信息
        if (audio_config_received_) {
            audio_config_ = *msg;
            audio_config_.data.clear(); // 清空数据，只保留配置
            audio_config_received_ = false;
            RCLCPP_INFO(this->get_logger(), "接收到音频配置: 采样率=%d Hz, 声道数=%d, 位深=%d", 
                        audio_config_.sample_rate, audio_config_.channels, audio_config_.bytes_per_sample * 8);
        }

        // 累加音频数据
        audio_data_.insert(audio_data_.end(), msg->data.begin(), msg->data.end());

        // 检查是否已经采集了10秒
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_).count();

        if (elapsed_time >= 10) {
            RCLCPP_INFO(this->get_logger(), "已采集10秒音频数据，准备保存文件");
            is_recording_ = false;
            save_audio_file();
            rclcpp::shutdown();
        }
    }

    void save_audio_file() {
        if (audio_data_.empty()) {
            RCLCPP_WARN(this->get_logger(), "没有采集到音频数据，无法保存文件");
            return;
        }

        std::string filename = "recorded_audio.wav";
        std::ofstream outfile(filename, std::ios::binary);

        if (!outfile) {
            RCLCPP_ERROR(this->get_logger(), "无法打开文件 %s 进行写入", filename.c_str());
            return;
        }

        // 计算音频数据的参数
        uint32_t sample_rate = audio_config_.sample_rate;
        uint16_t original_channels = audio_config_.channels;
        uint16_t bits_per_sample = audio_config_.bytes_per_sample * 8;
        uint16_t bytes_per_sample = audio_config_.bytes_per_sample;

        // 使用单通道直接保存
        uint16_t mono_channels = 1;
        std::vector<uint8_t> mono_data;
        
        if (original_channels != 1) {
            RCLCPP_INFO(this->get_logger(), "将 %d 声道音频转换为单声道", original_channels);
            
            // 计算采样点数量
            size_t num_samples = audio_data_.size() / (original_channels * bytes_per_sample);
            mono_data.reserve(num_samples * mono_channels * bytes_per_sample);

            // 处理16位有符号整数格式
            if (bytes_per_sample == 2) {
                for (size_t i = 0; i < num_samples; i++) {
                    int32_t sum = 0;
                    // 对每个声道的值求和
                    for (uint16_t ch = 0; ch < original_channels; ch++) {
                        size_t offset = (i * original_channels + ch) * bytes_per_sample;
                        int16_t sample = *reinterpret_cast<const int16_t*>(&audio_data_[offset]);
                        sum += sample;
                    }
                    // 计算平均值作为单声道的采样值
                    int16_t mono_sample = static_cast<int16_t>(sum / original_channels);
                    
                    // 添加单声道数据
                    mono_data.insert(mono_data.end(), 
                                    reinterpret_cast<const uint8_t*>(&mono_sample), 
                                    reinterpret_cast<const uint8_t*>(&mono_sample) + bytes_per_sample);
                }
            } else {
                // 对于其他格式，简单使用第一个声道
                RCLCPP_WARN(this->get_logger(), "仅支持16位音频格式的声道转换，将使用第一个声道");
                size_t num_samples = audio_data_.size() / (original_channels * bytes_per_sample);
                mono_data.reserve(num_samples * mono_channels * bytes_per_sample);
                
                for (size_t i = 0; i < num_samples; i++) {
                    size_t offset = i * original_channels * bytes_per_sample;
                    // 复制第一个声道数据
                    mono_data.insert(mono_data.end(), 
                                    audio_data_.begin() + offset, 
                                    audio_data_.begin() + offset + bytes_per_sample);
                }
            }
        } else {
            // 已经是单声道，直接使用原始数据
            mono_data = audio_data_;
        }

        // 计算单声道的参数
        uint32_t byte_rate = sample_rate * mono_channels * (bits_per_sample / 8);
        uint16_t block_align = mono_channels * (bits_per_sample / 8);
        uint32_t data_size = mono_data.size();
        uint32_t file_size = 36 + data_size;

        // 写入WAV文件头
        // RIFF标记
        outfile.write("RIFF", 4);
        outfile.write(reinterpret_cast<const char*>(&file_size), 4);
        outfile.write("WAVE", 4);
        
        // fmt子块
        outfile.write("fmt ", 4);
        uint32_t fmt_chunk_size = 16;
        outfile.write(reinterpret_cast<const char*>(&fmt_chunk_size), 4);
        uint16_t format_tag = 1; // PCM格式
        outfile.write(reinterpret_cast<const char*>(&format_tag), 2);
        outfile.write(reinterpret_cast<const char*>(&mono_channels), 2);
        outfile.write(reinterpret_cast<const char*>(&sample_rate), 4);
        outfile.write(reinterpret_cast<const char*>(&byte_rate), 4);
        outfile.write(reinterpret_cast<const char*>(&block_align), 2);
        outfile.write(reinterpret_cast<const char*>(&bits_per_sample), 2);
        
        // data子块
        outfile.write("data", 4);
        outfile.write(reinterpret_cast<const char*>(&data_size), 4);
        
        // 写入音频数据
        outfile.write(reinterpret_cast<const char*>(mono_data.data()), data_size);
        outfile.close();

        RCLCPP_INFO(this->get_logger(), "音频数据已保存为文件: %s", filename.c_str());
        RCLCPP_INFO(this->get_logger(), "文件大小: %u 字节", data_size + 44);
        RCLCPP_INFO(this->get_logger(), "音频参数: %d Hz, %d 声道, %d 位/采样", 
                    sample_rate, mono_channels, bits_per_sample);
        if (original_channels != 1) {
            RCLCPP_INFO(this->get_logger(), "已将原始 %d 声道音频转换为单声道", original_channels);
        }
        
        // 使用默认声卡播放保存的音频
        RCLCPP_INFO(this->get_logger(), "正在播放保存的音频...");
        std::string play_command = "aplay " + filename;
        int result = system(play_command.c_str());
        if (result == 0) {
            RCLCPP_INFO(this->get_logger(), "音频播放成功！");
        } else {
            RCLCPP_WARN(this->get_logger(), "音频播放失败，错误码: %d", result);
        }
    }

    rclcpp::Subscription<audio_capture::msg::AudioData>::SharedPtr audio_subscriber_;
    std::chrono::steady_clock::time_point start_time_;
    std::atomic<bool> is_recording_;
    bool audio_config_received_;
    audio_capture::msg::AudioData audio_config_;
    std::vector<uint8_t> audio_data_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AudioPlayerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
