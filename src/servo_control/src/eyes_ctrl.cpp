#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <iostream>
#include <thread>
#include <chrono>
#include <errno.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

// ROS2 头文件
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#define FRAME_HEADER 0x55
#define CMD_SERVO_MOVE 0x03
#define CMD_ACTION_GROUP_RUN 0x06
#define CMD_ACTION_GROUP_STOP 0x07
#define CMD_ACTION_GROUP_SPEED 0x0B
#define CMD_GET_BATTERY_VOLTAGE 0x0F
#define GET_LOW_BYTE(A) ((uint8_t)(A))
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))
#define baud_rate B9600

typedef struct _lobot_servo_ {
    uint8_t ID;
    uint16_t Position;
} LobotServo;

// 串口参数
#define AUDIO_SERIAL_PORT "/dev/ttyS7"
#define AUDIO_BAUDRATE B9600

// 协议相关定义
#define MSG_START_BYTE0 0xEE
#define MSG_START_BYTE1 0xEF
#define MSG_END_BYTE 0xFF

// 状态机状态
typedef enum {
    STATE_WAIT_START0,
    STATE_WAIT_START1,
    STATE_WAIT_TYPE,
    STATE_WAIT_LENGTH,
    STATE_WAIT_DATA,
    STATE_WAIT_CHECKSUM,
    STATE_WAIT_END
} ParserState;

// 解析器结构体
typedef struct {
    ParserState state;
    unsigned char buffer[256];
    int buffer_pos;
    int data_length;
    unsigned char message_type;
} ProtocolParser;

// 舵机参数
#define SERVO_RIGHT_EYE_YAW 3     // 右眼左右转动舵机ID
#define SERVO_LEFT_EYE_YAW 4      // 左眼左右转动舵机ID
#define SERVO_BOTH_EYES_PITCH 5   // 双眼上下转动舵机ID
#define SERVO_MIN_POS 500         // 舵机最小位置
#define SERVO_MAX_POS 2500        // 舵机最大位置
#define SERVO_YAW_CENTER 1500     // 左右转动中心位置
#define SERVO_PITCH_CENTER 1700   // 上下转动中心位置
#define SERVO_YAW_LIMIT 300       // 左右转动限位（中心位置±300）
#define SERVO_PITCH_LIMIT 300     // 上下转动限位（中心位置±300）
#define ANGLE_RANGE 60.0          // 角度范围（度）
#define MOVE_TIME 400             // 舵机移动时间（毫秒）

// 小端序字节转单精度浮点数
float bytes_to_float_little_endian(unsigned char byte0, unsigned char byte1, unsigned char byte2, unsigned char byte3) {
    // 构建小端序浮点数
    union {
        unsigned char bytes[4];
        float value;
    } conversion;
    
    conversion.bytes[0] = byte0;
    conversion.bytes[1] = byte1;
    conversion.bytes[2] = byte2;
    conversion.bytes[3] = byte3;
    
    return conversion.value;
}

// 初始化解析器
void init_parser(ProtocolParser *parser) {
    parser->state = STATE_WAIT_START0;
    parser->buffer_pos = 0;
    parser->data_length = 0;
    parser->message_type = 0;
}

// 处理单个字节
int process_byte(ProtocolParser *parser, unsigned char byte) {
    switch (parser->state) {
        case STATE_WAIT_START0:
            if (byte == MSG_START_BYTE0) {
                parser->buffer[parser->buffer_pos++] = byte;
                parser->state = STATE_WAIT_START1;
            }
            break;
            
        case STATE_WAIT_START1:
            if (byte == MSG_START_BYTE1) {
                parser->buffer[parser->buffer_pos++] = byte;
                parser->state = STATE_WAIT_TYPE;
            } else {
                init_parser(parser); // 重置
            }
            break;
            
        case STATE_WAIT_TYPE:
            parser->message_type = byte;
            parser->buffer[parser->buffer_pos++] = byte;
            parser->state = STATE_WAIT_LENGTH;
            break;
            
        case STATE_WAIT_LENGTH:
            parser->data_length = byte;
            parser->buffer[parser->buffer_pos++] = byte;
            if (parser->data_length > 0) {
                parser->state = STATE_WAIT_DATA;
            } else {
                parser->state = STATE_WAIT_CHECKSUM;
            }
            break;
            
        case STATE_WAIT_DATA:
            parser->buffer[parser->buffer_pos++] = byte;
            if (parser->buffer_pos - 4 >= parser->data_length) { // 4是前面的起始字节、类型和长度
                parser->state = STATE_WAIT_CHECKSUM;
            }
            break;
            
        case STATE_WAIT_CHECKSUM:
            parser->buffer[parser->buffer_pos++] = byte;
            parser->state = STATE_WAIT_END;
            break;
            
        case STATE_WAIT_END:
            if (byte == MSG_END_BYTE) {
                parser->buffer[parser->buffer_pos++] = byte;
                // 解析完成
                return 1;
            } else {
                init_parser(parser); // 重置
            }
            break;
    }
    return 0;
}

class ServoController {
private:
    int uart_fd_;
    std::string uart_device_;
    
    uint8_t tx_buf_[128];
    
    // 串口发送互斥锁
    std::mutex uart_mutex_;
    
    // 跟踪所有舵机的当前实际角度
    std::map<uint8_t, float> current_servo_angles;

public:
    ServoController(const std::string& device) : uart_fd_(-1), uart_device_(device) {
        if (!init_uart()) {
            std::cerr << "Failed to initialize UART" << std::endl;
        }
    }
    
    ~ServoController() {
        if (uart_fd_ >= 0) {
            close(uart_fd_);
        }
    }
    
    bool init_uart() {
        uart_fd_ = open(uart_device_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (uart_fd_ < 0) {
            std::cerr << "Cannot open UART device: " << uart_device_ << ", error: " << strerror(errno) << std::endl;
            return false;
        }
        
        struct termios options;
        tcgetattr(uart_fd_, &options);
        
        cfsetispeed(&options, baud_rate);
        cfsetospeed(&options, baud_rate);
        
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        
        options.c_cflag |= (CLOCAL | CREAD);
        
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        
        options.c_oflag &= ~OPOST;
        
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 1;
        
        tcsetattr(uart_fd_, TCSANOW, &options);
        
        tcflush(uart_fd_, TCIOFLUSH);
        
        std::cout << "UART initialized: " << uart_device_ << " @ 9600 baud" << std::endl;
        return true;
    }
    
    // 设置波特率
    bool set_baud_rate(speed_t baud) {
        if (uart_fd_ < 0) {
            std::cerr << "UART not initialized" << std::endl;
            return false;
        }
        
        struct termios options;
        if (tcgetattr(uart_fd_, &options) != 0) {
            std::cerr << "Failed to get UART attributes" << std::endl;
            return false;
        }
        
        if (cfsetispeed(&options, baud) != 0 || cfsetospeed(&options, baud) != 0) {
            std::cerr << "Failed to set UART baud rate" << std::endl;
            return false;
        }
        
        if (tcsetattr(uart_fd_, TCSANOW, &options) != 0) {
            std::cerr << "Failed to apply UART settings" << std::endl;
            return false;
        }
        
        return true;
    }
    
    bool send_command(uint8_t* data, size_t length) {
        if (uart_fd_ < 0) {
            std::cerr << "UART not initialized" << std::endl;
            return false;
        }
        
        // 使用互斥锁保护串口发送
        std::lock_guard<std::mutex> lock(uart_mutex_);
        
        ssize_t bytes_written = write(uart_fd_, data, length);
        if (bytes_written < 0) {
            std::cerr << "Failed to write to UART" << std::endl;
            return false;
        }
        
        if ((size_t)bytes_written != length) {
            std::cerr << "Partial write: " << bytes_written << "/" << length << " bytes" << std::endl;
            return false;
        }
        
        return true;
    }
    
    // 获取串口文件描述符
    int get_uart_fd() const {
        return uart_fd_;
    }
    

    
    void move_servo(uint8_t servo_id, uint16_t position, uint16_t time) {
        if (servo_id > 31 || time == 0) {
            std::cerr << "Invalid servo parameters: ID=" << (unsigned int)servo_id << ", Time=" << time << std::endl;
            return;
        }
        
        // 清空接收缓冲区
        tcflush(uart_fd_, TCIFLUSH);
        
        tx_buf_[0] = tx_buf_[1] = FRAME_HEADER;
        tx_buf_[2] = 8;
        tx_buf_[3] = CMD_SERVO_MOVE;
        tx_buf_[4] = 1;
        tx_buf_[5] = GET_LOW_BYTE(time);
        tx_buf_[6] = GET_HIGH_BYTE(time);
        tx_buf_[7] = servo_id;
        tx_buf_[8] = GET_LOW_BYTE(position);
        tx_buf_[9] = GET_HIGH_BYTE(position);
        
        std::cout << "Sending servo command: ID=" << (unsigned int)servo_id << ", Position=" << position << ", Time=" << time << std::endl;
        
        bool result = send_command(tx_buf_, 10);
        
        // if (result) {
        //     std::cout << "Servo " << (unsigned int)servo_id << " moved to position " << position << " in " << time << " ms" << std::endl;
        // }
    }
    
    void move_servos_by_array(LobotServo servos[], uint8_t num, uint16_t time) {
        if (num < 1 || num > 32 || time == 0) {
            std::cerr << "Invalid parameters: Num=" << (unsigned int)num << ", Time=" << time << std::endl;
            return;
        }
        
        uint8_t index = 7;
        tx_buf_[0] = tx_buf_[1] = FRAME_HEADER;
        tx_buf_[2] = num * 3 + 5;
        tx_buf_[3] = CMD_SERVO_MOVE;
        tx_buf_[4] = num;
        tx_buf_[5] = GET_LOW_BYTE(time);
        tx_buf_[6] = GET_HIGH_BYTE(time);
        
        for (uint8_t i = 0; i < num; i++) {
            tx_buf_[index++] = servos[i].ID;
            tx_buf_[index++] = GET_LOW_BYTE(servos[i].Position);
            tx_buf_[index++] = GET_HIGH_BYTE(servos[i].Position);
        }
        
        bool result = send_command(tx_buf_, tx_buf_[2] + 2);
        
        // if (result) {
        //     std::cout << "Moved " << (unsigned int)num << " servos in " << time << " ms" << std::endl;
        // }
    }
    
    void run_action_group(uint8_t num_of_action, uint16_t times) {
        tx_buf_[0] = tx_buf_[1] = FRAME_HEADER;
        tx_buf_[2] = 5;
        tx_buf_[3] = CMD_ACTION_GROUP_RUN;
        tx_buf_[4] = num_of_action;
        tx_buf_[5] = GET_LOW_BYTE(times);
        tx_buf_[6] = GET_HIGH_BYTE(times);
        
        bool result = send_command(tx_buf_, 7);
        
        if (result) {
            std::cout << "Action group " << (unsigned int)num_of_action << " started, " << times << " times" << std::endl;
        }
    }
    
    void stop_action_group() {
        tx_buf_[0] = FRAME_HEADER;
        tx_buf_[1] = FRAME_HEADER;
        tx_buf_[2] = 2;
        tx_buf_[3] = CMD_ACTION_GROUP_STOP;
        
        bool result = send_command(tx_buf_, 4);
        
        if (result) {
            std::cout << "Action group stopped" << std::endl;
        }
    }
    
    void set_action_group_speed(uint8_t num_of_action, uint16_t speed) {
        tx_buf_[0] = tx_buf_[1] = FRAME_HEADER;
        tx_buf_[2] = 5;
        tx_buf_[3] = CMD_ACTION_GROUP_SPEED;
        tx_buf_[4] = num_of_action;
        tx_buf_[5] = GET_LOW_BYTE(speed);
        tx_buf_[6] = GET_HIGH_BYTE(speed);
        
        bool result = send_command(tx_buf_, 7);
        
        if (result) {
            if (num_of_action == 0xFF) {
                std::cout << "All action groups speed set to " << speed << std::endl;
            } else {
                std::cout << "Action group " << (unsigned int)num_of_action << " speed set to " << speed << std::endl;
            }
        }
    }
    
    void set_all_action_group_speed(uint16_t speed) {
        set_action_group_speed(0xFF, speed);
    }
    
    void get_battery_voltage() {
        tx_buf_[0] = FRAME_HEADER;
        tx_buf_[1] = FRAME_HEADER;
        tx_buf_[2] = 2;
        tx_buf_[3] = CMD_GET_BATTERY_VOLTAGE;
        
        bool result = send_command(tx_buf_, 4);
        
        if (result) {
            std::cout << "Battery voltage request sent" << std::endl;
        }
    }
    
    // 读取CSV文件并返回舵机角度数据
    std::vector<std::vector<float>> read_csv(const std::string& file_path) {
        std::vector<std::vector<float>> data;
        std::ifstream file(file_path);
        
        if (!file.is_open()) {
            std::cerr << "Failed to open CSV file: " << file_path << std::endl;
            return data;
        }
        
        std::string line;
        while (std::getline(file, line)) {
            std::vector<float> row;
            std::stringstream ss(line);
            std::string value;
            
            while (std::getline(ss, value, ',')) {
                try {
                    row.push_back(std::stof(value));
                } catch (...) {
                    row.push_back(0.0f);
                }
            }
            
            data.push_back(row);
        }
        
        file.close();
        return data;
    }
    
    // 控制指定舵机以40ms间隔依照CSV数据切换
    void control_servos_from_csv(const std::string& csv_file, bool* interrupt_flag = nullptr, int jaw_diff = 0, int left_diff = 0, int right_diff = 0) {
        // 读取CSV数据
        auto csv_data = read_csv(csv_file);
        if (csv_data.empty()) {
            std::cerr << "No data read from CSV file" << std::endl;
            return;
        }
        
        // 指定要控制的舵机ID
        std::vector<uint8_t> servo_ids = {0, 1, 2, 6, 7, 8, 9, 10};
        
        // 遍历CSV的每一行
        for (size_t row_idx = 0; row_idx < csv_data.size(); row_idx++) {
            // 检查是否需要中断
            if (interrupt_flag && *interrupt_flag) {
                std::cout << "Emotion control interrupted by new emotion" << std::endl;
                return;
            }
            
            const auto& row = csv_data[row_idx];
            
            // 检查行数据是否足够
            if (row.size() < 14) {
                std::cerr << "Row " << row_idx << " has insufficient data" << std::endl;
                continue;
            }
            
            // 准备要控制的舵机数据
            std::vector<LobotServo> servos;
            for (uint8_t servo_id : servo_ids) {
                if (servo_id < row.size()) {
                    float target_angle = row[servo_id];
                    float limited_angle = target_angle;
                    
                    // 只有在前25帧（表情切换后刚开始）对指定舵机应用角度限制
                    if (row_idx < 40 && (servo_id == 0 || (servo_id >= 6 && servo_id <= 10))) {
                        if (current_servo_angles.find(servo_id) != current_servo_angles.end()) {
                            float current_angle = current_servo_angles[servo_id];
                            float angle_diff = target_angle - current_angle;
                            
                            // 取消角度变化限制，直接使用目标角度
                            limited_angle = target_angle;
                        } else {
                            // 如果是第一次控制该舵机，使用当前目标角度作为初始角度
                            current_servo_angles[servo_id] = target_angle;
                        }
                    }
                    
                    LobotServo servo;
                    servo.ID = servo_id;
                    // 将角度转换为舵机位置（角度范围0-180对应位置500-2500）
                    servo.Position = static_cast<uint16_t>((limited_angle / 180.0f) * 2000.0f + 500.0f);
                    
                    // 对8、9、10号舵机加上对应的差值
                    if (servo_id == 8) { // 右嘴角舵机
                        servo.Position += right_diff;
                    } else if (servo_id == 9) { // 左嘴角舵机
                        servo.Position += left_diff;
                    } else if (servo_id == 10) { // 嘴部张合舵机
                        servo.Position += jaw_diff;
                    }
                    
                    servos.push_back(servo);
                    
                    // 更新当前角度
                    current_servo_angles[servo_id] = limited_angle;
                }
            }
            
            // 计算动态移动时间，表情切换时使用更长的执行时间
            uint16_t move_time = 200; // 默认移动时间
            if (row_idx < 40) {
                // 在前40帧，随着帧序号增加，逐渐减少移动时间
                // 开始时使用较长时间（800ms），然后逐渐减少到200ms
                move_time = 2000 - (1000 - 200) * row_idx / 40;
            }
            
            // 控制舵机
            if (!servos.empty()) {
                LobotServo* servo_array = servos.data();
                move_servos_by_array(servo_array, static_cast<uint8_t>(servos.size()), move_time);
                
                // 等待40ms
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
    }
};


class EyesControlNode : public rclcpp::Node {
public:
    EyesControlNode() : Node("eyes_control_node") {
        // 初始化舵机控制器
        servo_ctrl_ = std::make_unique<ServoController>("/dev/ttyS7");
        
        // 初始化舵机到中心位置
        servo_ctrl_->move_servo(SERVO_RIGHT_EYE_YAW, SERVO_YAW_CENTER, MOVE_TIME);
        servo_ctrl_->move_servo(SERVO_LEFT_EYE_YAW, SERVO_YAW_CENTER, MOVE_TIME);
        servo_ctrl_->move_servo(SERVO_BOTH_EYES_PITCH, SERVO_PITCH_CENTER, MOVE_TIME);
        
        // 订阅人脸角度话题
        angle_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "face/angle",
            10,
            std::bind(&EyesControlNode::angle_callback, this, std::placeholders::_1)
        );
        
        // 订阅情绪话题
        emotion_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/robot/emotion",
            10,
            std::bind(&EyesControlNode::emotion_callback, this, std::placeholders::_1)
        );
        
        // 订阅嘴部舵机参数话题
        mouth_params_subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/robot/servo_angles",
            10,
            [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
                this->mouth_params_callback(msg);
            }
        );
        
        RCLCPP_INFO(this->get_logger(), "Eyes Control Node started successfully");
        RCLCPP_INFO(this->get_logger(), "Subscribed to face/angle topic");
        RCLCPP_INFO(this->get_logger(), "Subscribed to /robot/emotion topic");
        RCLCPP_INFO(this->get_logger(), "Subscribed to /robot/servo_angles topic");
        RCLCPP_INFO(this->get_logger(), "Servos initialized to center position");
        RCLCPP_INFO(this->get_logger(), "Right eye yaw servo: ID=%d, Center=%d", SERVO_RIGHT_EYE_YAW, SERVO_YAW_CENTER);
        RCLCPP_INFO(this->get_logger(), "Left eye yaw servo: ID=%d, Center=%d", SERVO_LEFT_EYE_YAW, SERVO_YAW_CENTER);
        RCLCPP_INFO(this->get_logger(), "Both eyes pitch servo: ID=%d, Center=%d", SERVO_BOTH_EYES_PITCH, SERVO_PITCH_CENTER);
        
        // 初始化解析器
        init_parser(&parser_);
        
        // 启动串口读取线程
        serial_thread_ = std::thread(&EyesControlNode::audio_serial_read_thread, this);
        
        // 启动自然动作控制
        start_nature_control();
        
        RCLCPP_INFO(this->get_logger(), "音频串口读取线程启动成功");
    }
    
    ~EyesControlNode() {
        // 停止自然动作控制
        nature_control_running_ = false;
        
        // 等待自然动作控制线程结束
        if (nature_control_thread_.joinable()) {
            nature_control_thread_.join();
        }
        
        // 等待串口读取线程结束
        if (serial_thread_.joinable()) {
            serial_thread_.join();
        }
    }
    
private:
    // 自然动作控制线程函数
    void nature_control_thread_func() {
        while (nature_control_running_) {
            // 根据当前情绪选择对应的CSV文件
            std::string emotion_folder = current_emotion_;
            
            // 检查情绪文件夹是否存在，如果不存在则使用默认的nature
            std::string folder_path = "/home/orangepi/ros/LinYue/src/servo_control/angle_date/" + emotion_folder;
            std::string csv_file = folder_path + "/1.csv";
            
            // 检查文件是否存在
            std::ifstream file(csv_file);
            if (!file.good()) {
                // 如果文件不存在，使用默认的nature
                RCLCPP_WARN(this->get_logger(), "情绪文件夹 %s 不存在或文件不存在，使用默认的nature", emotion_folder.c_str());
                emotion_folder = "nature";
                csv_file = "/home/orangepi/ros/LinYue/src/servo_control/angle_date/nature/1.csv";
            }
            file.close();
            
            // 重置新情绪标志
            new_emotion_received_ = false;
            
            RCLCPP_INFO(this->get_logger(), "开始执行 %s 情绪动作", emotion_folder.c_str());
            
            // 根据当前情绪设置表情目标位置
            if (emotion_folder == "happy") {
                // 开心表情：嘴巴张开，嘴角上扬
                expression_jaw_target_ = 1400;
                expression_left_target_ = 1400;
                expression_right_target_ = 1400;
            } else if (emotion_folder == "sad") {
                // 悲伤表情：嘴巴微张，嘴角下垂
                expression_jaw_target_ = 1350;
                expression_left_target_ = 1600;
                expression_right_target_ = 1600;
            } else if (emotion_folder == "angry") {
                // 愤怒表情：嘴巴紧闭，嘴角下拉
                expression_jaw_target_ = 1250;
                expression_left_target_ = 1650;
                expression_right_target_ = 1650;
            } else if (emotion_folder == "surprised") {
                // 惊讶表情：嘴巴大张，嘴角自然
                expression_jaw_target_ = 1500;
                expression_left_target_ = 1500;
                expression_right_target_ = 1500;
            } else {
                // 自然表情：嘴巴闭合，嘴角自然
                expression_jaw_target_ = 1300;
                expression_left_target_ = 1500;
                expression_right_target_ = 1500;
            }
            
            // 传递中断标志和嘴部舵机参数差值给控制函数
            servo_ctrl_->control_servos_from_csv(csv_file, &new_emotion_received_, jaw_diff_, left_diff_, right_diff_);
            RCLCPP_INFO(this->get_logger(), "%s 情绪动作执行完成，等待2秒后再次执行", emotion_folder.c_str());
            
            // 除了nature之外的情绪执行完后切换回nature
            if (emotion_folder != "nature") {
                RCLCPP_INFO(this->get_logger(), "非nature情绪执行完成，切换回nature情绪");
                current_emotion_ = "nature";
            }
            
            // 等待2秒后再次执行
            for (int i = 0; i < 20 && nature_control_running_; i++) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
    
    // 开始自然动作控制
    void start_nature_control() {
        nature_control_running_ = true;
        nature_control_thread_ = std::thread(&EyesControlNode::nature_control_thread_func, this);
        RCLCPP_INFO(this->get_logger(), "自然动作控制线程启动成功");
    }
    
    // 情绪回调函数
    void emotion_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string emotion = msg->data;
        RCLCPP_INFO(this->get_logger(), "接收到情绪: %s", emotion.c_str());
        
        // 更新当前情绪
        current_emotion_ = emotion;
        
        // 设置新情绪标志，用于打断当前执行的表情
        new_emotion_received_ = true;
    }
    
    // 嘴部舵机参数回调函数
    void mouth_params_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 3) {
            jaw_diff_ = msg->data[0];
            left_diff_ = msg->data[1];
            right_diff_ = msg->data[2];
            RCLCPP_INFO(this->get_logger(), "接收到嘴部舵机参数差值: jaw=%d, left=%d, right=%d", jaw_diff_, left_diff_, right_diff_);
            
            // 在回调函数中直接控制嘴部舵机
            control_mouth_servos();
        }
    }
    
    // 控制嘴部舵机
    void control_mouth_servos() {
        // 嘴部舵机的基点位置
        const int BASE_JAW = 1300;
        const int BASE_LEFT = 1500;
        const int BASE_RIGHT = 1500;
        
        // 计算语音同步的目标位置
        int speech_jaw = BASE_JAW + jaw_diff_*1.5;
        int speech_left = BASE_LEFT + left_diff_*4;
        int speech_right = BASE_RIGHT + right_diff_*4;
        
        // 计算表情的目标位置
        int expression_jaw = expression_jaw_target_;
        int expression_left = expression_left_target_;
        int expression_right = expression_right_target_;
        
        // 使用权重融合两个目标位置
        int jaw_position = static_cast<int>(speech_jaw * speech_weight_ + expression_jaw * expression_weight_);
        int left_position = static_cast<int>(speech_left * speech_weight_ + expression_left * expression_weight_);
        int right_position = static_cast<int>(speech_right * speech_weight_ + expression_right * expression_weight_);
        
        // 限制舵机位置范围
        jaw_position = std::max(500, std::min(2500, jaw_position));
        left_position = std::max(500, std::min(2500, left_position));
        right_position = std::max(500, std::min(2500, right_position));
        
        RCLCPP_INFO(this->get_logger(), "控制嘴部舵机: jaw=%d, left=%d, right=%d (语音: %d,%d,%d, 表情: %d,%d,%d)", 
                    jaw_position, left_position, right_position, 
                    speech_jaw, speech_left, speech_right, 
                    expression_jaw, expression_left, expression_right);
        
        // 单独控制嘴部张合舵机（每次都控制）
        servo_ctrl_->move_servo(10, static_cast<uint16_t>(jaw_position), 50); // 嘴部张合舵机
        
        // 检查嘴角舵机控制的冷却时间（0.8秒）
        auto now = std::chrono::steady_clock::now();
        auto corner_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_corner_control_time_).count();
        
        if (corner_elapsed >= 200 || last_corner_control_time_ == std::chrono::steady_clock::time_point()) {
            // 同时控制左右嘴角舵机（达到冷却时间才控制）
            LobotServo corner_servos[2];
            corner_servos[0].ID = 9; // 左嘴角舵机
            corner_servos[0].Position = static_cast<uint16_t>(left_position);
            corner_servos[1].ID = 8; // 右嘴角舵机
            corner_servos[1].Position = static_cast<uint16_t>(right_position);
            
            servo_ctrl_->move_servos_by_array(corner_servos, 2, 200);
            
            // 更新嘴角舵机控制时间
            last_corner_control_time_ = now;
            RCLCPP_INFO(this->get_logger(), "控制嘴角舵机，冷却时间已过 %ld 毫秒", corner_elapsed);
        } else {
            RCLCPP_INFO(this->get_logger(), "嘴角舵机冷却时间未到，跳过控制，已过 %ld 毫秒", corner_elapsed);
        }
    }
    
    
    // 串口读取线程
    void audio_serial_read_thread() {
        unsigned char buffer[256];
        
        while (rclcpp::ok()) {
            // 获取舵机控制器的串口文件描述符
            int uart_fd = servo_ctrl_->get_uart_fd();
            if (uart_fd < 0) {
                RCLCPP_ERROR(this->get_logger(), "无法获取串口文件描述符");
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            
            // 设置读取超时（使用select实现）
            struct timeval timeout;
            timeout.tv_sec = 1;  // 1秒超时
            timeout.tv_usec = 0;
            
            fd_set read_fds;
            FD_ZERO(&read_fds);
            FD_SET(uart_fd, &read_fds);
            
            int select_result = select(uart_fd + 1, &read_fds, NULL, NULL, &timeout);
            
            if (select_result > 0) {
                if (FD_ISSET(uart_fd, &read_fds)) {
                    // 有数据可读，尝试读取
                    int n = read(uart_fd, buffer, sizeof(buffer));
                    if (n > 0) {
                        // 处理每个字节
                        for (int i = 0; i < n; i++) {
                            if (process_byte(&parser_, buffer[i])) {
                                // 解析成功，处理数据
                                RCLCPP_INFO(this->get_logger(), "解析成功: %d 字节", parser_.buffer_pos);
                                RCLCPP_INFO(this->get_logger(), "消息类型: 0x%02X", parser_.message_type);
                                RCLCPP_INFO(this->get_logger(), "数据长度: %d", parser_.data_length);
                                
                                // 解析声源角度数据（从第4字节开始，跳过起始字节、类型和长度）
                                if (parser_.data_length >= 4) {
                                    process_audio_angle(&parser_.buffer[4], parser_.data_length);
                                }
                                
                                // 重置解析器
                                init_parser(&parser_);
                            }
                        }
                    } else if (n < 0) {
                        if (!(errno == EAGAIN || errno == EWOULDBLOCK)) {
                            RCLCPP_ERROR(this->get_logger(), "读取数据失败: %s", strerror(errno));
                        }
                    }
                }
            } else if (select_result < 0) {
                RCLCPP_ERROR(this->get_logger(), "select失败: %s", strerror(errno));
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
    
    // 处理声源角度数据
    void process_audio_angle(unsigned char *data, int length) {
        if (length >= 4) {
            float azimuth = bytes_to_float_little_endian(data[0], data[1], data[2], data[3]);
            
            RCLCPP_INFO(this->get_logger(), "接收到声源角度: %.6f", azimuth);
        }
    }
    
    // 角度回调函数
    void angle_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
            double raw_angle_yaw = msg->data[0];   // 原始偏航角 (水平)
            double angle_pitch = msg->data[1];      // 俯仰角 (垂直)
            double angle_yaw = -raw_angle_yaw;      // 反转偏航角以纠正镜像问题
            
            RCLCPP_INFO(this->get_logger(), "Received angles: Raw Yaw=%.2f, Corrected Yaw=%.2f, Pitch=%.2f", raw_angle_yaw, angle_yaw, angle_pitch);
            
            // 计算舵机位置
            uint16_t yaw_position = calculate_servo_position(angle_yaw, SERVO_YAW_CENTER);
            uint16_t pitch_position = calculate_servo_position(angle_pitch, SERVO_PITCH_CENTER);
            
            // 控制舵机：使用同时控制多个舵机的指令
            LobotServo servos[3];
            
            // 右眼偏航舵机
            servos[0].ID = SERVO_RIGHT_EYE_YAW;
            servos[0].Position = yaw_position;
            
            // 左眼偏航舵机
            servos[1].ID = SERVO_LEFT_EYE_YAW;
            servos[1].Position = yaw_position;
            
            // 双眼俯仰舵机
            servos[2].ID = SERVO_BOTH_EYES_PITCH;
            servos[2].Position = pitch_position;
            
            // 同时控制多个舵机
            servo_ctrl_->move_servos_by_array(servos, 3, MOVE_TIME);
            
            RCLCPP_INFO(this->get_logger(), "Controlled servos: Yaw=%d, Pitch=%d", yaw_position, pitch_position);
        }
    }
    
    // 计算舵机位置
    uint16_t calculate_servo_position(double angle, uint16_t center_pos) {
        // 将角度映射到舵机位置范围
        // 角度范围: -ANGLE_RANGE/2 到 ANGLE_RANGE/2
        // 位置范围: SERVO_MIN_POS 到 SERVO_MAX_POS
        
        // 归一化角度到 0-1 范围
        double normalized_angle = (angle + ANGLE_RANGE/2) / ANGLE_RANGE;
        
        // 限制范围
        if (normalized_angle < 0.0) normalized_angle = 0.0;
        if (normalized_angle > 1.0) normalized_angle = 1.0;
        
        // 映射到舵机位置
        uint16_t position = SERVO_MIN_POS + (SERVO_MAX_POS - SERVO_MIN_POS) * normalized_angle;
        
        // 应用左右转动限位（仅适用于偏航角舵机）
        if (center_pos == SERVO_YAW_CENTER) {
            uint16_t min_limit = SERVO_YAW_CENTER - SERVO_YAW_LIMIT;
            uint16_t max_limit = SERVO_YAW_CENTER + SERVO_YAW_LIMIT;
            
            if (position < min_limit) position = min_limit;
            if (position > max_limit) position = max_limit;
        }
        
        // 应用上下转动限位（仅适用于俯仰角舵机）
        if (center_pos == SERVO_PITCH_CENTER) {
            uint16_t min_limit = SERVO_PITCH_CENTER - SERVO_PITCH_LIMIT;
            uint16_t max_limit = SERVO_PITCH_CENTER + SERVO_PITCH_LIMIT;
            
            if (position < min_limit) position = min_limit;
            if (position > max_limit) position = max_limit;
        }
        
        return position;
    }
    
    // 成员变量
    std::unique_ptr<ServoController> servo_ctrl_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr angle_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr emotion_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr mouth_params_subscription_;
    
    // 音频串口相关变量
    ProtocolParser parser_;
    std::thread serial_thread_;
    
    // 自然动作控制线程
    std::thread nature_control_thread_;
    bool nature_control_running_ = false;
    
    // 当前情绪
    std::string current_emotion_ = "nature";
    
    // 新情绪标志
    bool new_emotion_received_ = false;
    
    // 嘴部舵机参数差值（来自speaker.cpp）
    int jaw_diff_ = 0;
    int left_diff_ = 0;
    int right_diff_ = 0;
    
    // 表情对嘴部舵机的目标位置（来自CSV文件）
    int expression_jaw_target_ = 1300;
    int expression_left_target_ = 1500;
    int expression_right_target_ = 1500;
    
    // 权重参数
    float speech_weight_ = 0.7;  // 语音同步权重
    float expression_weight_ = 0.3;  // 表情权重
    
    // 上次执行嘴部舵机控制的时间
    std::chrono::steady_clock::time_point last_mouth_control_time_;
    // 上次执行嘴角舵机控制的时间
    std::chrono::steady_clock::time_point last_corner_control_time_;
};

int main(int argc, char** argv) {
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    // 创建节点
    auto node = std::make_shared<EyesControlNode>();
    
    // 运行节点
    rclcpp::spin(node);
    
    // 关闭ROS2
    rclcpp::shutdown();
    
    return 0;
}
