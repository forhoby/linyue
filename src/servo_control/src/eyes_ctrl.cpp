#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <iostream>
#include <thread>
#include <chrono>
#include <errno.h>

// ROS2 头文件
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#define FRAME_HEADER 0x55
#define CMD_SERVO_MOVE 0x03
#define GET_LOW_BYTE(A) ((uint8_t)(A))
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))
#define baud_rate B9600

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
            std::cerr << "Cannot open UART device: " << uart_device_ << std::endl;
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
        
        std::cout << "UART initialized: " << uart_device_ << " @ 115200 baud" << std::endl;
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
    
    void move_servo(uint8_t servo_id, uint16_t position, uint16_t time) {
        if (servo_id > 31 || time == 0) {
            std::cerr << "Invalid servo parameters: ID=" << (unsigned int)servo_id << ", Time=" << time << std::endl;
            return;
        }
        
        // 临时设置波特率为9600（舵机控制需要）
        if (!set_baud_rate(B9600)) {
            std::cerr << "Failed to set baud rate to 9600" << std::endl;
            return;
        }
        
        tx_buf_[0] = tx_buf_[1] = FRAME_HEADER;
        tx_buf_[2] = 8;
        tx_buf_[3] = CMD_SERVO_MOVE;
        tx_buf_[4] = 1;
        tx_buf_[5] = GET_LOW_BYTE(time);
        tx_buf_[6] = GET_HIGH_BYTE(time);
        tx_buf_[7] = servo_id;
        tx_buf_[8] = GET_LOW_BYTE(position);
        tx_buf_[9] = GET_HIGH_BYTE(position);
        
        bool result = send_command(tx_buf_, 10);
        
        // 恢复波特率为115200（角度读取需要）
        if (!set_baud_rate(B115200)) {
            std::cerr << "Failed to set baud rate back to 115200" << std::endl;
        }
        
        if (result) {
            std::cout << "Servo " << (unsigned int)servo_id << " moved to position " << position << " in " << time << " ms" << std::endl;
        }
    }
};

// 主节点类
class EyesControlNode : public rclcpp::Node {
public:
    EyesControlNode() : Node("eyes_control_node") {
        // 创建发布者，发布方位角数据
        publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("azimuth_angle", 10);
        
        // 初始化舵机控制器
        servo_ctrl_ = std::make_unique<ServoController>("/dev/ttyS7");
        
        // 初始化解析器
        init_parser(&parser_);
        
        // 启动舵机控制线程
        servo_thread_ = std::thread(&EyesControlNode::servo_control_thread, this);
        
        // 启动角度读取线程
        angle_thread_ = std::thread(&EyesControlNode::angle_read_thread, this);
        
        RCLCPP_INFO(this->get_logger(), "Eyes Control Node started successfully");
    }
    
    ~EyesControlNode() {
        if (servo_thread_.joinable()) {
            servo_thread_.join();
        }
        if (angle_thread_.joinable()) {
            angle_thread_.join();
        }
    }
    
private:
    // 舵机控制线程
    void servo_control_thread() {
        uint16_t position1 = 500;
        uint16_t position2 = 2500;
        uint16_t time = 1000;
        bool direction = true; // true表示向position2移动，false表示向position1移动
        
        while (rclcpp::ok()) {
            if (direction) {
                servo_ctrl_->move_servo(0, position2, time);
            } else {
                servo_ctrl_->move_servo(0, position1, time);
            }
            
            direction = !direction;
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        }
    }
    
    // 角度读取线程
    void angle_read_thread() {
        int fd = open("/dev/ttyS7", O_RDWR | O_NOCTTY);
        if (fd < 0) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open UART device for angle reading: %s", strerror(errno));
            return;
        }
        
        // 配置串口
        struct termios options;
        tcgetattr(fd, &options);
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CRTSCTS;
        options.c_cflag |= CREAD | CLOCAL;
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 50;
        tcsetattr(fd, TCSANOW, &options);
        
        unsigned char buffer[256];
        
        while (rclcpp::ok()) {
            // 设置读取超时
            struct timeval timeout;
            timeout.tv_sec = 1;
            timeout.tv_usec = 0;
            
            fd_set read_fds;
            FD_ZERO(&read_fds);
            FD_SET(fd, &read_fds);
            
            int select_result = select(fd + 1, &read_fds, NULL, NULL, &timeout);
            
            if (select_result > 0) {
                if (FD_ISSET(fd, &read_fds)) {
                    int n = read(fd, buffer, sizeof(buffer));
                    if (n > 0) {
                        for (int i = 0; i < n; i++) {
                            if (process_byte(&parser_, buffer[i])) {
                                // 解析成功，处理数据
                                RCLCPP_INFO(this->get_logger(), "解析成功: %d 字节", parser_.buffer_pos);
                                RCLCPP_INFO(this->get_logger(), "消息类型: 0x%02X", parser_.message_type);
                                RCLCPP_INFO(this->get_logger(), "数据长度: %d", parser_.data_length);
                                
                                // 解析方位角数据
                                if (parser_.data_length >= 4) {
                                    publish_azimuth(&parser_.buffer[4], parser_.data_length);
                                }
                                
                                // 重置解析器
                                init_parser(&parser_);
                            }
                        }
                    }
                }
            }
        }
        
        close(fd);
    }
    
    // 发布方位角数据
    void publish_azimuth(unsigned char *data, int length) {
        if (length >= 4) {
            float azimuth = bytes_to_float_little_endian(data[0], data[1], data[2], data[3]);
            
            // 创建消息
            auto message = geometry_msgs::msg::Vector3();
            message.x = azimuth;  // 方位角
            message.y = 0.0;      // 预留
            message.z = 0.0;      // 预留
            
            // 发布消息
            publisher_->publish(message);
            
            RCLCPP_INFO(this->get_logger(), "发布方位角: %.6f", azimuth);
        }
    }
    
    // 成员变量
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
    std::unique_ptr<ServoController> servo_ctrl_;
    ProtocolParser parser_;
    std::thread servo_thread_;
    std::thread angle_thread_;
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
