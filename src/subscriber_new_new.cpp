#include <chrono>
#include <functional>
#include <memory>
#include <map>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"  // Joyメッセージをインクルード
#include "sensor_msgs/msg/joint_state.hpp"
// #include "std_msgs/msg/float32_multi_array.hpp"

#include "LinuxHardwareSerial.hpp"
#include "SerialBridge.hpp"
#include "controller.hpp"
#include "feedback.hpp"
#include "feetech_handler.hpp"

#define SERIAL_PORT "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.2"

// sudo chmod 666 /dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.2

using namespace std::chrono_literals;
using std::placeholders::_1;

class UnifiedNode : public rclcpp::Node {
public:
  UnifiedNode(const char port[], speed_t baud_rate = B115200)
      : Node("unified_node"), 
        serial_dev(new LinuxHardwareSerial(port, baud_rate)),
        serial(new SerialBridge(serial_dev, 1024)) {

    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "sp1/joy", 10, std::bind(&UnifiedNode::joy_callback, this, _1));

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/one/joint_states", 1);

    // publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("feedback_data", 1);  // パブリッシャを追加

    serial->add_frame(0, &msg);
    serial->add_frame(1, &fb);

    // タイマーの設定（シリアル通信とジョイント状態）
    timer_ = this->create_wall_timer(
            10ms, std::bind(&UnifiedNode::serial_callback, this));
    joint_timer_ = this->create_wall_timer(
            50ms, std::bind(&UnifiedNode::onJointTimer, this));

    // Feetechハンドラの初期化
    printf("start feetech\n");
    std::map<int, ServoConfig> config_list;
    config_list[1] = {-32237, 32236};
    config_list[2] = {-32237, 32236};
    config_list[7] = {-32237, 32236};
    config_list[20] = {-32237, 32236};
    config_list[21] = {-32237, 32236};
    config_list[30] = {-32237, 32236};
    config_list[31] = {-32237, 32236};
    if (!feetech_handler_.Initialize(config_list)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize FeetechHandler.");
      throw;
    }
  }

private:

  void onJointTimer() {
    feetech_handler_.RequestStatus();

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = now();

    auto s1_opt = feetech_handler_.GetStatus(7);
    if (s1_opt) {
      auto& status = s1_opt.value();
      joint_state.name.push_back("turret_pan_joint");
      joint_state.position.push_back(static_cast<float>(status.position));
      joint_state.velocity.push_back(static_cast<float>(status.velocity));
    }

    joint_state_pub_->publish(joint_state);
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    lx = msg->axes[0] * -1;
    ly = msg->axes[1];
    rx = msg->axes[2] * -1;
    ry = msg->axes[3];
    l2 = msg->axes[4];
    r2 = msg->axes[5];

    cross = msg->buttons[0];
    circle = msg->buttons[1];
    square = msg->buttons[2];
    triangle = msg->buttons[3];
    r1 = msg->buttons[10];

    down = msg->buttons[11];
    up = msg->buttons[12];
    // 11 12 13 14 juuji

    // RCLCPP_INFO(this->get_logger(), "Received: [lx: %.2f, ly: %.2f, rx: %.2f, ry: %.2f, l2: %.2f, r2: %.2f]", lx, ly, rx, ry, l2, r2);

    if( cross == 1 ){
      setCommandCustomPos(7, 3317);  // Panのコマンド
      RCLCPP_INFO(this->get_logger(), "-----------------unlock-----------------\n");
    }

    if( circle == 1 ){
      setCommandCustomPos(7, 2724);  // Panのコマンド
      RCLCPP_INFO(this->get_logger(), "-----------------unlock-----------------\n");
    }

    if( up == 1){
      setTorqueEnable(7, 0);
      RCLCPP_INFO(this->get_logger(), "-----------------torque-unlock-----------------\n");
    }

    if( down == 1){
      setTorqueEnable(7, 1);
      RCLCPP_INFO(this->get_logger(), "-----------------torque-lock-----------------\n");
    }
    // RCLCPP_INFO(this->get_logger(), "setCommand [%.2f]", msg->axes[1]);
  }

  void setCommand(const int id, const float value) {
    feetech_handler_.SetCommand(id, 0, value * 3150);
    usleep(1000);
  }

  void setCommandCustomPos(const int id, const int value)
  {
    feetech_handler_.SetCommand(id, value, 0);
    usleep(100000); // 100ms
  }

  void setTorqueEnable(const int id, const bool state)
  {
    feetech_handler_.SetTorqueEnable(id, state);
    usleep(100000);
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

    // RCLCPP_INFO(this->get_logger(), "Buttons: [cross: %d, circle: %d, square: %d, triangle: %d, r1: %d]", cross, circle, square, triangle, r1);

    serial->write(0);

    if (fb.was_updated()) {
      rps = fb.data.rps;
      timing = fb.data.timing;
      timing_2 = fb.data.timing_2;
      count = fb.data.count;

      /*
      std_msgs::msg::Float32MultiArray array_msg;
      array_msg.data.push_back(rps);
      array_msg.data.push_back(static_cast<float>(timing));
      array_msg.data.push_back(static_cast<float>(timing_2));
      array_msg.data.push_back(static_cast<float>(count));
      */

      // publisher_->publish(array_msg);

      // RCLCPP_INFO(this->get_logger(), "Publishing data: [rps: %.3f, timing: %d, timing_2: %d, count: %d]", rps, timing, timing_2, count);
    }
  }

  

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  // rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr joint_timer_;

  SerialDev *serial_dev;
  SerialBridge *serial;
  Controller msg;
  Feedback fb;
  float lx, ly, rx, ry, l2, r2;
  bool cross, circle, triangle, square, r1;
  bool up, down;
  float rps;
  bool timing, timing_2;
  int count;

  FeetechHandler feetech_handler_;
  static constexpr int center_tick_ = 2048;
  static constexpr int tick_per_rad_ = 651.9f;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto unified_node = std::make_shared<UnifiedNode>(SERIAL_PORT);
  rclcpp::spin(unified_node);
  rclcpp::shutdown();
  return 0;
}
