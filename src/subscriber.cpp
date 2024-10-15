#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"

#include "LinuxHardwareSerial.hpp"
#include "SerialBridge.hpp"
#include "controller.hpp"

#define SERIAL_PORT "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.2"

// sudo chmod 666 /dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.2

using namespace std::chrono_literals;
using std::placeholders::_1;

class UnifiedNode : public rclcpp::Node {
public:
  UnifiedNode(const char port[], speed_t baud_rate = B115200)
          : Node("unified_node"), serial_dev(new LinuxHardwareSerial(port, baud_rate)),
            serial(new SerialBridge(serial_dev, 1024)) {

    // Float32MultiArrayのサブスクライバ
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "controller_one", 10, std::bind(&UnifiedNode::array_callback, this, _1));

    serial->add_frame(0, &msg);

    // シリアル通信のチェックを行うタイマーの設定
    timer_ = this->create_wall_timer(
            100ms, std::bind(&UnifiedNode::serial_callback, this));
  }

private:

  // サブスクライブ時のコールバック
  void array_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {

    lx = msg->data[0];
    ly = msg->data[1];
    rx = msg->data[2];
    ry = msg->data[3];
    l2 = (msg->data[4] - 1) * -0.5f;
    r2 = (msg->data[5] - 1) * -0.5f;
    buttons = msg->data[6];

    RCLCPP_INFO(this->get_logger(), "Received: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", lx, ly, rx, ry, l2, r2, buttons);


  }

  // シリアル通信時のコールバック
  void serial_callback() {
    // シリアルデバイスからデータを読み取る
    int result = serial->update();
    RCLCPP_INFO(this->get_logger(), "Serial data updated: %d", result);

    msg.data.lx = lx;
    msg.data.ly = ly;
    msg.data.rx = rx;
    msg.data.ry = ry;
    msg.data.l2 = l2;
    msg.data.r2 = r2;
    msg.data.buttons = buttons;

    serial->write(0);

    RCLCPP_INFO(this->get_logger(), "Sending message");
  }

  // サブスクライバ
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;

  // タイマー
  rclcpp::TimerBase::SharedPtr timer_;

  // シリアルデバイス
  SerialDev *serial_dev;
  SerialBridge *serial;

  // メッセージ
  Controller msg;

  // サブスクライブしたデータを保存する変数
  float lx, ly, rx, ry, l2, r2, buttons;
  bool cross, circle, triangle, square;
};

//////////////////////////////////////////////////////

// メイン関数
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto unified_node = std::make_shared<UnifiedNode>(SERIAL_PORT);

  rclcpp::spin(unified_node);

  rclcpp::shutdown();
  return 0;
}

