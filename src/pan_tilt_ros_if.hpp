#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>

#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;  // NOLINT

struct controllerStatus
{
  int stop_position_state_;
  bool lastButtonState_ = false;
  bool air_state_{false};
};

class PanTiltRosIf : public rclcpp::Node
{
 public:
  PanTiltRosIf() : Node("pan_tilt_node")
  {
    position_state_.resize(4);
    velocity_state_.resize(4);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
    //  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("cmd_rate", 10, std::bind(&PanTiltRosIf::onTwistReceived, this, std::placeholders::_1));
    //    twist_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("custom_topic", 10, std::bind(&PanTiltRosIf::onTwistReceived, this, std::placeholders::_1));
    twist_sub_.push_back(this->create_subscription<sensor_msgs::msg::Joy>("foot/joy", 10, std::bind(&PanTiltRosIf::onTwistReceivedFoot, this, std::placeholders::_1)));
    twist_sub_.push_back(this->create_subscription<sensor_msgs::msg::Joy>("armL/joy", 10, std::bind(&PanTiltRosIf::onTwistReceivedLeftArm, this, std::placeholders::_1)));
    twist_sub_.push_back(this->create_subscription<sensor_msgs::msg::Joy>("armR/joy", 10, std::bind(&PanTiltRosIf::onTwistReceivedRightArm, this, std::placeholders::_1)));
    // f32_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("feedback_data", 1000, std::bind(&PanTiltRosIf::onFloat32MultiArrayReceived, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(10ms, std::bind(&PanTiltRosIf::onTimer, this));
  }

 protected:
  virtual void onTimer() = 0;
  //  virtual void onTwistReceived(const geometry_msgs::msg::TwistStamped::SharedPtr msg) = 0;
  virtual void onTwistReceivedFoot(const sensor_msgs::msg::Joy::SharedPtr msg) = 0;
  virtual void onTwistReceivedLeftArm(const sensor_msgs::msg::Joy::SharedPtr msg) = 0;
  virtual void onTwistReceivedRightArm(const sensor_msgs::msg::Joy::SharedPtr msg) = 0;

  // virtual void onFloat32MultiArrayReceived(const std_msgs::msg::Float32MultiArray::SharedPtr msg) = 0;

  void publishJointState(const sensor_msgs::msg::JointState &msg)
  {
    joint_state_pub_->publish(msg);
  }

  controllerStatus leftArm;
  controllerStatus rightArm;

  std::vector<int> position_state_;
  std::vector<int> velocity_state_;

  float joy_data;

 private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_{nullptr};
  // rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_{nullptr};
  // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr twist_sub_{nullptr};
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr> twist_sub_;
  // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr f32_sub_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
};