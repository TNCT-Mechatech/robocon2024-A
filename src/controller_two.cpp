#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class JoyToFloat32MultiArrayNode : public rclcpp::Node {
public:
  JoyToFloat32MultiArrayNode() : Node("joy_to_float32_array") {
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("controller_two", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy_two", 10, std::bind(&JoyToFloat32MultiArrayNode::joy_callback, this, std::placeholders::_1));
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Robot 2 Sending data... ");
    auto array_msg = std_msgs::msg::Float32MultiArray();
    float buttons = static_cast<float>(msg->buttons[0]) * 1000 + static_cast<float>(msg->buttons[1]) * 100 + static_cast<float>(msg->buttons[2]) * 10 + static_cast<float>(msg->buttons[3]);
    array_msg.data = {msg->axes[0], msg->axes[1], msg->axes[3], msg->axes[4], msg->axes[2], msg->axes[5], buttons};
    publisher_->publish(array_msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToFloat32MultiArrayNode>());
  rclcpp::shutdown();
  return 0;
}

