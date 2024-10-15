#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"  // Joyメッセージをインクルード

#include "LinuxHardwareSerial.hpp"
#include "SerialBridge.hpp"
#include "controller.hpp"
#include "feedback.hpp"
#include <std_msgs/msg/float32_multi_array.hpp> 

#include "feetech_handler.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

#define SERIAL_PORT "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.2"

// sudo chmod 666 /dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.2

using namespace std::chrono_literals;
using std::placeholders::_1;




class UnifiedNode : public rclcpp::Node {
public:
  UnifiedNode(const char port[], speed_t baud_rate = B115200)
          : Node("unified_node"), serial_dev(new LinuxHardwareSerial(port, baud_rate)),
            serial(new SerialBridge(serial_dev, 1024)) {

    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "sp1/joy", 10, std::bind(&UnifiedNode::joy_callback, this, _1));

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("feedback_data", 1);  // パブリッシャを追加

    serial->add_frame(0, &msg);
    serial->add_frame(1, &fb);

    timer_ = this->create_wall_timer(
            10ms, std::bind(&UnifiedNode::serial_callback, this));
  }

private:

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    lx = msg->axes[0] * -1;
    ly = msg->axes[1];
    rx = msg->axes[2] * -1;
    ry = msg->axes[3];
    l2 = msg->axes[4];
    r2 = msg->axes[5];
//    l2 = (msg->axes[4] - 1) * -0.5f;
//    r2 = (msg->axes[5] - 1) * -0.5f;


    // buttons
    cross = msg->buttons[0];
    circle = msg->buttons[1];
    square = msg->buttons[2];
    triangle = msg->buttons[3];
    r1 = msg->buttons[10];

    RCLCPP_INFO(this->get_logger(), "Received: [lx: %.2f, ly: %.2f, rx: %.2f, ry: %.2f, l2: %.2f, r2: %.2f", lx, ly, rx, ry, l2, r2);
  }

  void serial_callback() {
    int result = serial->update();
    RCLCPP_INFO(this->get_logger(), "Serial data updated: %d", result);

    msg.data.lx = lx;
    msg.data.ly = ly * -1;
    msg.data.rx = rx;
    msg.data.ry = ry;
    msg.data.l2 = l2;
    msg.data.r2 = r2;
    msg.data.cross = cross;
    msg.data.circle = circle;
    msg.data.square = square;
    msg.data.triangle = triangle;
    msg.data.r1 = r1;

    RCLCPP_INFO(this->get_logger(), "Buttons: [cross: %d, circle: %d, square: %d, triangle: %d, r1: %d]", cross, circle, square, triangle, r1);

    serial->write(0);

    if (fb.was_updated()) {
      rps = fb.data.rps;
      timing = fb.data.timing;
      timing_2 = fb.data.timing_2;
      count = fb.data.count;

      // Float32MultiArrayのメッセージを作成してデータを追加
      std_msgs::msg::Float32MultiArray array_msg;
      array_msg.data.push_back(rps);
      array_msg.data.push_back(static_cast<float>(timing));  // timingをfloatにキャスト
      array_msg.data.push_back(static_cast<float>(timing_2));  // timing_2をfloatにキャスト
      array_msg.data.push_back(static_cast<float>(count));   // countをfloatにキャスト

      publisher_->publish(array_msg);  // パブリッシュ

      RCLCPP_INFO(this->get_logger(), "Publishing data: [rps: %.3f, timing: %d, timing_2: %d, count: %d]", rps, timing, timing_2, count);
    }

    RCLCPP_INFO(this->get_logger(), "Sending message");
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;  // パブリッシャの宣言
  rclcpp::TimerBase::SharedPtr timer_;
  SerialDev *serial_dev;
  SerialBridge *serial;
  Controller msg;
  Feedback fb;
  float lx, ly, rx, ry, l2, r2;
  bool cross, circle, triangle, square, r1;
  float rps;
  bool timing, timing_2;
  int count;
};

//////////////////////////////////////////////////////

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto unified_node = std::make_shared<UnifiedNode>(SERIAL_PORT);

  rclcpp::spin(unified_node);

  rclcpp::shutdown();
  return 0;
}

