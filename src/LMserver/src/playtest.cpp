#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include <vector>
#include <fstream>
#include <chrono>
#include <atomic>

// WAV 文件头结构
struct WavHeader {
    // RIFF 标记
    char riff[4] = { 'R', 'I', 'F', 'F' };
    uint32_t file_size; // 文件大小 - 8
    char wave[4] = { 'W', 'A', 'V', 'E' };
    
    // fmt 标记
    char fmt[4] = { 'f', 'm', 't', ' ' };
    uint32_t fmt_size = 16; // PCM 格式的 fmt 块大小
    uint16_t format = 1; // PCM 格式
    uint16_t channels = 1; // 单声道
    uint32_t sample_rate = 16000; // 16kHz 采样率
    uint32_t byte_rate = 16000 * 2; // 字节率 = 采样率 * 声道数 * 位深 / 8
    uint16_t block_align = 2; // 块对齐 = 声道数 * 位深 / 8
    uint16_t bits_per_sample = 16; // 16 位深
    
    // data 标记
    char data[4] = { 'd', 'a', 't', 'a' };
    uint32_t data_size; // 数据大小
};

class PlaytestNode : public rclcpp::Node {
private:
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr subscription_;
    
    // 音频数据缓冲区
    std::vector<int16_t> audio_data_;
    
    // 录制状态
    std::atomic<bool> recording_;
    std::atomic<bool> recording_completed_;
    
    // 开始录制时间
    std::chrono::steady_clock::time_point start_time_;
    
    // 录制时长（秒）
    const int recording_duration_ = 20;

public:
    PlaytestNode() : Node("playtest_node"), recording_(false), recording_completed_(false) {
        // 订阅音频回复主题
        subscription_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "/robot/audio_reply", 10, 
            std::bind(&PlaytestNode::audio_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Playtest node started, listening to /robot/audio_reply");
        RCLCPP_INFO(this->get_logger(), "Will record 20 seconds of audio and save to current directory");
    }

    ~PlaytestNode() {
        // 如果录制已完成，保存音频
        if (recording_completed_) {
            save_audio();
        }
    }

    // 开始录制
    void start_recording() {
        recording_ = true;
        recording_completed_ = false;
        audio_data_.clear();
        start_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Started recording audio for 20 seconds");
    }

    // 检查录制是否完成
    bool is_recording_completed() {
        return recording_completed_;
    }

private:
    void audio_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        if (!recording_) {
            // 如果还未开始录制，自动开始
            start_recording();
        }

        try {
            const std::vector<int16_t> &audio_chunk = msg->data;
            size_t chunk_size = audio_chunk.size();

            if (chunk_size > 0) {
                // 将音频数据添加到缓冲区
                audio_data_.insert(audio_data_.end(), audio_chunk.begin(), audio_chunk.end());
                
                // 检查录制时长
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::steady_clock::now() - start_time_).count();
                
                if (elapsed >= recording_duration_) {
                    // 录制完成
                    recording_ = false;
                    recording_completed_ = true;
                    RCLCPP_INFO(this->get_logger(), "Recording completed after %d seconds", recording_duration_);
                    RCLCPP_INFO(this->get_logger(), "Total audio samples recorded: %zu", audio_data_.size());
                    
                    // 保存音频
                    save_audio();
                    
                    // 退出节点
                    rclcpp::shutdown();
                }
            }

        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error in audio callback: %s", e.what());
        }
    }

    // 保存音频为 WAV 文件
    void save_audio() {
        try {
            // 生成文件名
            std::string filename = "recorded_audio.wav";
            
            // 打开文件
            std::ofstream file(filename, std::ios::binary);
            if (!file) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing: %s", filename.c_str());
                return;
            }
            
            // 准备 WAV 文件头
            WavHeader header;
            header.data_size = audio_data_.size() * sizeof(int16_t);
            header.file_size = header.data_size + 36; // 36 是文件头大小减去 8
            
            // 写入文件头
            file.write(reinterpret_cast<const char*>(&header), sizeof(header));
            
            // 写入音频数据
            file.write(reinterpret_cast<const char*>(audio_data_.data()), header.data_size);
            
            // 关闭文件
            file.close();
            
            RCLCPP_INFO(this->get_logger(), "Audio saved to: %s", filename.c_str());
            RCLCPP_INFO(this->get_logger(), "File size: %u bytes", header.file_size + 8);
            RCLCPP_INFO(this->get_logger(), "Audio duration: %.2f seconds", static_cast<float>(audio_data_.size()) / 16000.0f);
            
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error saving audio: %s", e.what());
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlaytestNode>();
    
    // 开始录制
    node->start_recording();
    
    // 运行节点
    rclcpp::spin(node);
    
    // 关闭节点
    node.reset();
    rclcpp::shutdown();
    
    return 0;
}