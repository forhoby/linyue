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

// 嘴部舵机参数
#define SERVO_JAW_INDEX 10          // 嘴部张合舵机索引
#define SERVO_LEFT_INDEX 9          // 左嘴角舵机索引
#define SERVO_RIGHT_INDEX 8         // 右嘴角舵机索引
#define SERVO_MIN_POS 500         // 舵机最小位置
#define SERVO_MAX_POS 2500        // 舵机最大位置
#define MOVE_TIME 50              // 舵机移动时间（毫秒）

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
    

};

// 主节点类
class MouthControlNode : public rclcpp::Node {
public:
    MouthControlNode() : Node("mouth_control_node") {
        // 初始化舵机控制器
        servo_ctrl_ = std::make_unique<ServoController>("/dev/ttyS7");
        
        // 初始化舵机到默认位置
        servo_ctrl_->move_servo(SERVO_JAW_INDEX, 1300, MOVE_TIME);  // 嘴部闭合
        servo_ctrl_->move_servo(SERVO_LEFT_INDEX, 1500, MOVE_TIME);  // 左嘴角居中
        servo_ctrl_->move_servo(SERVO_RIGHT_INDEX, 1500, MOVE_TIME); // 右嘴角居中
        
        // 订阅嘴部舵机参数话题
        mouth_params_subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/robot/servo_angles",
            10,
            [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) { this->mouth_params_callback(msg); }
        );
        
        RCLCPP_INFO(this->get_logger(), "Mouth Control Node started successfully");
        RCLCPP_INFO(this->get_logger(), "Subscribed to /robot/servo_angles topic");
        RCLCPP_INFO(this->get_logger(), "Servos initialized to default position");
        RCLCPP_INFO(this->get_logger(), "Jaw servo: ID=%d, Default=1300", SERVO_JAW_INDEX);
        RCLCPP_INFO(this->get_logger(), "Left corner servo: ID=%d, Default=1500", SERVO_LEFT_INDEX);
        RCLCPP_INFO(this->get_logger(), "Right corner servo: ID=%d, Default=1500", SERVO_RIGHT_INDEX);
    }
    
    ~MouthControlNode() {
    }
    
private:
    // 嘴部舵机参数回调函数
    void mouth_params_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 3) {
            // 接收到的是相对于基点的差值
            int jaw_diff = msg->data[0];   // 嘴部张合差值
            int left_diff = msg->data[1];  // 左嘴角差值
            int right_diff = msg->data[2]; // 右嘴角差值
            
            // 嘴部舵机的基点位置
            const int BASE_JAW = 1300;
            const int BASE_LEFT = 1500;
            const int BASE_RIGHT = 1500;
            
            // 计算最终舵机位置
            int jaw_position = BASE_JAW + jaw_diff;
            int left_position = BASE_LEFT + left_diff;
            int right_position = BASE_RIGHT + right_diff;
            
            // 限制舵机位置范围
            jaw_position = std::max(SERVO_MIN_POS, std::min(SERVO_MAX_POS, jaw_position));
            left_position = std::max(SERVO_MIN_POS, std::min(SERVO_MAX_POS, left_position));
            right_position = std::max(SERVO_MIN_POS, std::min(SERVO_MAX_POS, right_position));
            
            RCLCPP_INFO(this->get_logger(), "Received mouth params diff: Jaw=%d, Left=%d, Right=%d", jaw_diff, left_diff, right_diff);
            RCLCPP_INFO(this->get_logger(), "Calculated final position: Jaw=%d, Left=%d, Right=%d", jaw_position, left_position, right_position);
            
            // 控制嘴部舵机
            LobotServo servos[3];
            
            // 嘴部张合舵机
            servos[0].ID = SERVO_JAW_INDEX;
            servos[0].Position = static_cast<uint16_t>(jaw_position);
            
            // 左嘴角舵机
            servos[1].ID = SERVO_LEFT_INDEX;
            servos[1].Position = static_cast<uint16_t>(left_position);
            
            // 右嘴角舵机
            servos[2].ID = SERVO_RIGHT_INDEX;
            servos[2].Position = static_cast<uint16_t>(right_position);
            
            // 同时控制多个舵机
            servo_ctrl_->move_servos_by_array(servos, 3, MOVE_TIME);
            
            RCLCPP_INFO(this->get_logger(), "Controlled mouth servos: Jaw=%d, Left=%d, Right=%d", jaw_position, left_position, right_position);
        }
    }
    
    // 成员变量
    std::unique_ptr<ServoController> servo_ctrl_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr mouth_params_subscription_;
};

int main(int argc, char** argv) {
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    // 创建节点
    auto node = std::make_shared<MouthControlNode>();
    
    // 运行节点
    rclcpp::spin(node);
    
    // 关闭ROS2
    rclcpp::shutdown();
    
    return 0;
}
