#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <atomic>
#include <std_msgs/msg/header.hpp>

using namespace cv;
using namespace std;

class CameraNode : public rclcpp::Node
{
public:
        CameraNode() : rclcpp::Node("camera_node"), running_(true)
        {
                // 创建图像发布者
                image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
                
                // 打开摄像头
                cap_.open(0);
                if(!cap_.isOpened())
                {
                        RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
                        running_ = false;
                        return;
                }
                
                // 设置摄像头参数（提高性能）
                cap_.set(CAP_PROP_BUFFERSIZE, 5); // 减少缓冲区大小
                
                // 创建窗口显示图像
                namedWindow("camera", WINDOW_AUTOSIZE);
                
                // 创建图像捕获和处理线程
                camera_thread_ = std::thread(&CameraNode::camera_process, this);
        }
        
        ~CameraNode()
        {
                // 停止线程
                running_ = false;
                if(camera_thread_.joinable())
                {
                        camera_thread_.join();
                }
                
                cap_.release();
                destroyAllWindows();
        }
        
private:
        void camera_process()
        {
                while(running_ && rclcpp::ok())
                {
                        Mat frame;
                        
                        // 捕获图像
                        cap_ >> frame;
                        
                        if(frame.empty())
                        {
                                RCLCPP_WARN(this->get_logger(), "Empty frame");
                                continue;
                        }
                        
                        // 显示图像
                        imshow("camera", frame);
                        
                        // 发布图像消息
                        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
                        image_publisher_->publish(*msg);
                        
                        // 检查是否按下'q'键退出（使用1ms延迟，减少阻塞）
                        if(waitKey(1) == 'q')
                        {
                                running_ = false;
                                rclcpp::shutdown();
                        }
                }
        }
        
        VideoCapture cap_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
        std::thread camera_thread_;
        std::atomic<bool> running_;
};

int main(int argc, char **argv)
{
        // 初始化ROS2
        rclcpp::init(argc, argv);
        
        // 创建节点
        auto node = std::make_shared<CameraNode>();
        
        // 运行节点（这里只需要保持节点存活，实际工作在独立线程中）
        rclcpp::spin(node);
        
        // 关闭ROS2
        rclcpp::shutdown();
        
        return 0;
}