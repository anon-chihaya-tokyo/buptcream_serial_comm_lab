
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp" 
#include <memory>
#include <string>


#include "serial_comm/serial_receiver.hpp" 
#include "serial_comm/message.hpp"         

/**
 * @brief ImuPublisherNode 类
 
 */
class ImuPublisherNode : public rclcpp::Node {
public:
  ImuPublisherNode()
      : Node("serial_relay_node") // 1. 初始化 ROS 节点
  {
    
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    std::string port = this->get_parameter("serial_port").as_string();

    RCLCPP_INFO(this->get_logger(), "正在初始化串口接收器，端口: %s", port.c_str());

    try {
      
      serial_receiver_ = std::make_unique<SerialReceiver>(port);
      
    } catch (const std::exception& e) {
      RCLCPP_FATAL(this->get_logger(), "创建 SerialReceiver 失败: %s", e.what());
      // 如果串口打不开, 停止 rclcpp 并退出
      rclcpp::shutdown();
      return;
    }

    
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);

    
    serial_receiver_->registerCallback(
        std::bind(&ImuPublisherNode::onSerialDataReceived, this, std::placeholders::_1)
    );

    
    serial_receiver_->start();

    RCLCPP_INFO(this->get_logger(), "serial_relay_node 节点已启动。");
  }

  

private:
  /**
   * @brief (关键) 串口数据回调函数
   * 当 SerialReceiver 接收到一条完整的 SerialMessage 时, 它会调用此函数
   * @param serial_msg 从串口接收到的完整消息
   */
  void onSerialDataReceived(const serial_comm::SerialMessage& serial_msg) {
    
    
    auto ros_msg = std::make_unique<sensor_msgs::msg::Imu>();

    
    ros_msg->header.stamp = this->get_clock()->now();
    ros_msg->header.frame_id = "imu_link"; 

    
    const auto& imu_data = serial_msg.data;

    // 拷贝四元数
    ros_msg->orientation.w = imu_data.quaternion.w;
    ros_msg->orientation.x = imu_data.quaternion.x;
    ros_msg->orientation.y = imu_data.quaternion.y;
    ros_msg->orientation.z = imu_data.quaternion.z;

    // 拷贝角速度
    ros_msg->angular_velocity.x = imu_data.angular_velocity.x;
    ros_msg->angular_velocity.y = imu_data.angular_velocity.y;
    ros_msg->angular_velocity.z = imu_data.angular_velocity.z;

    // 拷贝线加速度
    ros_msg->linear_acceleration.x = imu_data.linear_acceleration.x;
    ros_msg->linear_acceleration.y = imu_data.linear_acceleration.y;
    ros_msg->linear_acceleration.z = imu_data.linear_acceleration.z;

    
    imu_publisher_->publish(std::move(ros_msg));
  }

  // --- 成员变量 ---
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  std::unique_ptr<SerialReceiver> serial_receiver_;
  
  
};

// --- main 函数 ---
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ImuPublisherNode>();
  
  
  if (rclcpp::ok()) {
    rclcpp::spin(node);
  }
  
  rclcpp::shutdown();
  return 0;
}
