#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <thread>
#include <chrono>
#include <sys/time.h>
#include <wiringPi.h>

// ROS2 头文件
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#define GPIO_PIN 27 // wiringPi pin number
#define BAUDRATE 9600
#define BIT_DELAY_US (1000000 / BAUDRATE)

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

// GPIO初始化
int init_gpio() {
    // 初始化wiringPi
    if (wiringPiSetup() == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("gpio"), "无法初始化wiringPi");
        return -1;
    }
    
    // 设置GPIO为输入模式
    pinMode(GPIO_PIN, INPUT);
    
    RCLCPP_INFO(rclcpp::get_logger("gpio"), "GPIO %d 初始化成功", GPIO_PIN);
    return 0;
}

// 读取GPIO电平
int read_gpio(int) {
    return digitalRead(GPIO_PIN);
}

// 精确延时（纳秒）
void delay_ns(unsigned long long ns) {
    struct timespec start, end;
    unsigned long long elapsed_ns;
    clock_gettime(CLOCK_MONOTONIC, &start);
    
    do {
        clock_gettime(CLOCK_MONOTONIC, &end);
        elapsed_ns = (end.tv_sec - start.tv_sec) * 1000000000LL + (end.tv_nsec - start.tv_nsec);
    } while (elapsed_ns < ns);
}

// 精确延时（微秒）
void delay_us(unsigned int us) {
    delay_ns(us * 1000LL);
}

// 模拟UART接收一个字节
int uart_read_byte() {
    unsigned char byte = 0;
    unsigned long long bit_delay_ns = BIT_DELAY_US * 1000LL;
    
    // 等待起始位（下降沿）
    while (read_gpio(0) == 1);
    delay_ns(bit_delay_ns / 2);
    
    // 读取8位数据位（LSB first）
    for (int i = 0; i < 8; i++) {
        delay_ns(bit_delay_ns);
        byte |= (read_gpio(0) << i);
    }
    
    // 等待停止位
    delay_ns(bit_delay_ns);
    
    return byte;
}

// 方位角节点类
class AzimuthNode : public rclcpp::Node {
public:
    AzimuthNode() : Node("azimuth_node") {
        // 创建发布者，发布方位角数据
        publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("azimuth_angle", 10);
        
        // 初始化GPIO
        if (init_gpio() < 0) {
            RCLCPP_ERROR(this->get_logger(), "无法初始化GPIO");
            return;
        }
        
        // 初始化解析器
        init_parser(&parser_);
        
        // 启动GPIO读取线程
        gpio_thread_ = std::thread(&AzimuthNode::gpio_read_thread, this);
        
        RCLCPP_INFO(this->get_logger(), "方位角节点启动成功，使用GPIO %d模拟UART RX", GPIO_PIN);
    }
    
    ~AzimuthNode() {
        if (gpio_thread_.joinable()) {
            gpio_thread_.join();
        }
    }
    
private:
    // GPIO读取线程
    void gpio_read_thread() {
        unsigned char byte;
        
        while (rclcpp::ok()) {
            // 模拟UART读取一个字节
            byte = uart_read_byte();
            
            // 处理字节
            if (process_byte(&parser_, byte)) {
                // 解析成功，处理数据
                RCLCPP_INFO(this->get_logger(), "解析成功: %d 字节", parser_.buffer_pos);
                RCLCPP_INFO(this->get_logger(), "消息类型: 0x%02X", parser_.message_type);
                RCLCPP_INFO(this->get_logger(), "数据长度: %d", parser_.data_length);
                
                // 解析方位角数据（从第4字节开始，跳过起始字节、类型和长度）
                if (parser_.data_length >= 4) {
                    publish_azimuth(&parser_.buffer[4], parser_.data_length);
                }
                
                // 重置解析器
                init_parser(&parser_);
            }
        }
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
    ProtocolParser parser_;
    std::thread gpio_thread_;
};

int main(int argc, char *argv[]) {
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    // 创建节点
    auto node = std::make_shared<AzimuthNode>();
    
    // 运行节点
    rclcpp::spin(node);
    
    // 关闭ROS2
    rclcpp::shutdown();
    
    return 0;
}
