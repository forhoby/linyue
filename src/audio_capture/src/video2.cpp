#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <atomic>
#include <std_msgs/msg/header.hpp>
#include <mutex>

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define video_size_width 1280
#define video_size_height 720
struct Buffer {
    void *start;
    size_t length;
};

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : rclcpp::Node("camera_node"), running_(true)
    {
        // 创建图像发布者
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
        
        // 初始化摄像头
        if (!init_camera()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera");
            running_ = false;
            return;
        }
        
        // 创建窗口显示图像
        cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);
        
        // 创建图像捕获和处理线程
        camera_thread_ = std::thread(&CameraNode::camera_process, this);
        
        // 创建定时器，每500ms调用一次timer_callback
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CameraNode::timer_callback, this));
    }
    
    ~CameraNode()
    {
        // 停止线程
        running_ = false;
        if (camera_thread_.joinable()) {
            camera_thread_.join();
        }
        
        // 释放资源
        cleanup_camera();
        cv::destroyAllWindows();
    }
    
private:
    bool init_camera()
    {
        // 1. 打开摄像头设备
        fd_ = open("/dev/video0", O_RDWR);
        if (fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "无法打开摄像头设备");
            return false;
        }

        // 2. 设置摄像头格式（分辨率和像素格式）
        struct v4l2_format fmt;
        CLEAR(fmt);
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = video_size_width;
        fmt.fmt.pix.height = video_size_height;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;

        if (ioctl(fd_, VIDIOC_S_FMT, &fmt) == -1) {
            RCLCPP_ERROR(this->get_logger(), "设置摄像头格式失败");
            close(fd_);
            return false;
        }

        // 3. 申请内存映射缓冲区
        struct v4l2_requestbuffers req;
        CLEAR(req);
        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd_, VIDIOC_REQBUFS, &req) == -1) {
            RCLCPP_ERROR(this->get_logger(), "申请内存映射缓冲区失败");
            close(fd_);
            return false;
        }

        // 4. 映射缓冲区
        for (unsigned int i = 0; i < req.count; ++i) {
            struct v4l2_buffer buf;
            CLEAR(buf);
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;

            if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) == -1) {
                RCLCPP_ERROR(this->get_logger(), "查询缓冲区失败");
                close(fd_);
                return false;
            }

            buffers_[i].length = buf.length;
            buffers_[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
            if (buffers_[i].start == MAP_FAILED) {
                RCLCPP_ERROR(this->get_logger(), "内存映射失败");
                close(fd_);
                return false;
            }
        }

        // 5. 将缓冲区放入队列
        for (unsigned int i = 0; i < req.count; ++i) {
            struct v4l2_buffer buf;
            CLEAR(buf);
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;

            if (ioctl(fd_, VIDIOC_QBUF, &buf) == -1) {
                RCLCPP_ERROR(this->get_logger(), "将缓冲区放入队列失败");
                close(fd_);
                for (unsigned int j = 0; j < i; ++j) {
                    munmap(buffers_[j].start, buffers_[j].length);
                }
                return false;
            }
        }

        // 6. 启动视频流
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(fd_, VIDIOC_STREAMON, &type) == -1) {
            RCLCPP_ERROR(this->get_logger(), "启动视频流失败");
            close(fd_);
            for (unsigned int i = 0; i < req.count; ++i) {
                munmap(buffers_[i].start, buffers_[i].length);
            }
            return false;
        }
        
        return true;
    }
    
    void cleanup_camera()
    {
        // 停止视频流
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd_, VIDIOC_STREAMOFF, &type);

        // 释放资源
        for (int i = 0; i < 4; ++i) {
            if (buffers_[i].start != MAP_FAILED) {
                munmap(buffers_[i].start, buffers_[i].length);
                buffers_[i].start = MAP_FAILED;
            }
        }
        
        if (fd_ != -1) {
            close(fd_);
            fd_ = -1;
        }
    }
    
    void camera_process()
    {
        while (running_ && rclcpp::ok()) {
            // 取出已填数据的缓冲区
            struct v4l2_buffer buf;
            CLEAR(buf);
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;

            if (ioctl(fd_, VIDIOC_DQBUF, &buf) == -1) {
                RCLCPP_ERROR(this->get_logger(), "取出缓冲区失败");
                break;
            }

            // 转换图像格式
            cv::Mat yuyv_img(cv::Size(video_size_width, video_size_height), CV_8UC2, buffers_[buf.index].start);
            cv::Mat bgr_img;
            cv::cvtColor(yuyv_img, bgr_img, cv::COLOR_YUV2BGR_YUYV);
            
            // 更新当前图像，使用互斥锁保护
            { 
                std::lock_guard<std::mutex> lock(image_mutex_);
                current_image_ = bgr_img.clone();
            }
            
            // 显示图像
            cv::imshow("Camera", bgr_img);

            // 按下 'q' 键退出
            if (cv::waitKey(1) == 'q') {
                running_ = false;
                rclcpp::shutdown();
                break;
            }

            // 将缓冲区重新放入队列
            if (ioctl(fd_, VIDIOC_QBUF, &buf) == -1) {
                RCLCPP_ERROR(this->get_logger(), "重新放入缓冲区失败");
                break;
            }
        }
    }
    
    void timer_callback()
    {
        // 发布当前显示的图像
        std::lock_guard<std::mutex> lock(image_mutex_);
        if (!current_image_.empty()) {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", current_image_).toImageMsg();
            image_publisher_->publish(*msg);
        }
    }
    
    // 摄像头相关变量
    int fd_ = -1;
    Buffer buffers_[4];
    
    // ROS2相关变量
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 线程相关变量
    std::thread camera_thread_;
    std::atomic<bool> running_;
    
    // 图像相关变量
    cv::Mat current_image_;
    std::mutex image_mutex_;
};

int main(int argc, char **argv)
{
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    // 创建节点
    auto node = std::make_shared<CameraNode>();
    
    // 运行节点
    rclcpp::spin(node);
    
    // 关闭ROS2
    rclcpp::shutdown();
    
    return 0;
}
