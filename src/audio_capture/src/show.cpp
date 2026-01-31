#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ShowNode : public rclcpp::Node
{
public:
    ShowNode() : rclcpp::Node("show_node")
    {
        // 创建图像订阅者，订阅"camera/image"话题
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image", 
            10, 
            std::bind(&ShowNode::image_callback, this, std::placeholders::_1)
        );
        
        // 创建窗口显示图像
        cv::namedWindow("Received Image", cv::WINDOW_AUTOSIZE);
        
        RCLCPP_INFO(this->get_logger(), "ShowNode initialized");
    }
    
    ~ShowNode()
    {
        // 清理窗口
        cv::destroyAllWindows();
        RCLCPP_INFO(this->get_logger(), "ShowNode destroyed");
    }
    
private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // 将ROS图像转换为OpenCV格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            
            // 显示图像
            cv::imshow("Received Image", cv_ptr->image);
            
            // 处理键盘事件，按下'q'键退出
            if (cv::waitKey(1) == 'q') {
                rclcpp::shutdown();
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    
    // ROS2图像订阅者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
};

int main(int argc, char **argv)
{
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    // 创建节点
    auto node = std::make_shared<ShowNode>();
    
    // 运行节点
    rclcpp::spin(node);
    
    // 关闭ROS2
    rclcpp::shutdown();
    
    return 0;
}