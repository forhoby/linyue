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

#define FRAME_HEADER 0x55
#define CMD_SERVO_MOVE 0x03
#define CMD_ACTION_GROUP_RUN 0x06
#define CMD_ACTION_GROUP_STOP 0x07
#define CMD_ACTION_GROUP_SPEED 0x0B
#define CMD_GET_BATTERY_VOLTAGE 0x0F
#define GET_LOW_BYTE(A) ((uint8_t)(A))
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))
#define BAUD B9600

typedef struct _lobot_servo_ {
    uint8_t ID;
    uint16_t Position;
} LobotServo;

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
            std::cerr << "Cannot open UART device: " << uart_device_ << ", error: " << strerror(errno) << std::endl;
            return false;
        }
        
        struct termios options;
        tcgetattr(uart_fd_, &options);
        
        cfsetispeed(&options, BAUD);
        cfsetospeed(&options, BAUD);
        
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
    
    void move_servo(uint8_t servo_id, uint16_t position, uint16_t time) {
        if (servo_id > 31 || time == 0) {
            std::cerr << "Invalid servo parameters: ID=" << (unsigned int)servo_id << ", Time=" << time << std::endl;
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
        
        if (result) {
            std::cout << "Servo " << (unsigned int)servo_id << " moved to position " << position << " in " << time << " ms" << std::endl;
        }
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
        
        if (result) {
            std::cout << "Moved " << (unsigned int)num << " servos in " << time << " ms" << std::endl;
        }
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
    void control_servos_from_csv(const std::string& csv_file) {
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
                    LobotServo servo;
                    servo.ID = servo_id;
                    // 将角度转换为舵机位置（角度范围0-180对应位置500-2500）
                    servo.Position = static_cast<uint16_t>((row[servo_id] / 180.0f) * 2000.0f + 500.0f);
                    servos.push_back(servo);
                }
            }
            
            // 控制舵机
            if (!servos.empty()) {
                LobotServo* servo_array = servos.data();
                move_servos_by_array(servo_array, static_cast<uint8_t>(servos.size()), 200);
                
                // 等待40ms
                std::this_thread::sleep_for(std::chrono::milliseconds(40));
            }
        }
    }
};

int main(int argc, char** argv) {
    std::cout << "Servo Control Program started" << std::endl;
    std::cout << "Using UART7 device: /dev/ttyS7" << std::endl;
    
    ServoController servo_ctrl("/dev/ttyS7");
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    std::cout << "Starting servo control from CSV..." << std::endl;
    
    // 控制指定舵机以40ms间隔依照CSV数据切换
    std::string csv_file = "/home/orangepi/ros/LinYue/src/servo_control/angle_date/nature/1.csv";
    servo_ctrl.control_servos_from_csv(csv_file);
    
    std::cout << "Servo control from CSV completed" << std::endl;
    
    return 0;
}
