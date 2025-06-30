#include "feetech_handler.hpp"
#include "pan_tilt_ros_if.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "PCA9685_RasPi/PCA9685_Raspi.hpp
#include "robocon2025_a/slider.hpp"

#include <cmath>

#include <pigpio.h>

class PanTiltNode : public PanTiltRosIf, public PCA9685_RasPi
{
public:
  PanTiltNode(void) : PanTiltRosIf{} , PCA9685_RasPi(2000)
  {
    printf("start feetech\n");
    std::map<int, ServoConfig> config_list;
    config_list[23] = {-32237, 32236};
    config_list[25] = {-32237, 32236};

    bool open_success = feetech_handler_.Initialize(config_list);
    if (!open_success)
    {
      printf("fail to open serial\n");
      throw;
    }
    else
      printf("success to open serial\n");

    // Float32MultiArrayのサブスクライバを作成
    float32_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "feedback_data", 10, std::bind(&PanTiltNode::onFloat32MultiArrayReceived, this, _1));
  }

private:
  void PwmGpio(int pinName, float unitInterval)
  {
    gpioPWM(pinName, static_cast<int>(255.0 * unitInterval));
  }

  void DirGpio(int pinName, float unitStatus)
  {
    if (unitStatus >= 0)
    {
      gpioWrite(pinName, 0);
    }
    else
    {
      gpioWrite(pinName, 1);
    }
  }

  /////////////////////////////////////////
  // As req
  void onTimer() override
  {
    feetech_handler_.RequestStatus();

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = now();

    /*

    auto s1_opt = feetech_handler_.GetStatus(10); // 左車輪
    if (s1_opt)
    {
      auto &status = s1_opt.value();
      joint_state.name.push_back("left_wheel_joint");
      joint_state.position.push_back(static_cast<float>(status.position));
      joint_state.velocity.push_back(static_cast<float>(status.velocity));
    }




    auto s2_opt = feetech_handler_.GetStatus(11); // 右車輪
    if (s2_opt)
    {
      auto &status = s2_opt.value();
      joint_state.name.push_back("right_wheel_joint");
      joint_state.position.push_back(static_cast<float>(status.position));
      joint_state.velocity.push_back(static_cast<float>(status.velocity));
    }

    */

    auto s2_opt = feetech_handler_.GetStatus(23);
    if (s2_opt)
    {
      // 取得成功
      auto &status = s2_opt.value();
      joint_state.name.push_back("arm_");
      joint_state.position.push_back(static_cast<float>(status.position));
      joint_state.velocity.push_back(static_cast<float>(status.velocity));

      position_state_ = status.position;
      velocity_state_ = status.velocity;

      RCLCPP_INFO(this->get_logger(), "id: %d, position: %d, velocity: %d\n", 23, status.position, status.velocity);
    }
    else
    {
      // 取得失敗
      std::cout << "サーボの状態取得に失敗" << std::endl;
    }

    publishJointState(joint_state);
  }

  /////////////////////////////////////////

  void
  onTwistReceived_servo(int id, bool input_x, bool input_y)
  {
    constexpr int stop_tick = 250;
    int position_state;

    if (velocity_state_ < 0)
    {
      position_state = position_state_ - 1 * stop_tick;
    }
    else
      position_state = position_state_ + stop_tick;

    if (position_state > 4094*4 -1 )
    {
      position_state = 4094*4 - 1;
    }
    else if (position_state < 1)
    {
      position_state = 1;
    }

    if (input_x != input_y)
    {

      float joy_input = static_cast<float>(input_x ? input_x : -1 * input_y) * 0.25;

      // joy_input: -1.0 ～ +1.0 を native speed (0～4096)に変換
      // フィジカル速度の上限を指定（例：max_speed = 4096）
      constexpr int max_speed = 3150;
      int native_speed = static_cast<int>(std::abs(joy_input) * max_speed);

      // 回転方向によって方向ビットを制御（feetech側で負値処理不要の場合）
      if (joy_input >= 0)
      {
        // 時計回り（bit15 = 0）
        feetech_handler_.SetCommand(id, 4094*4 - 1, native_speed);
        // RCLCPP_INFO(this->get_logger(), "CW speed = %d", native_speed);
      }
      else
      {
        // 反時計回り（bit15 = 1）
        feetech_handler_.SetCommand(id, 1, native_speed /*| 0x8000*/);
        // RCLCPP_INFO(this->get_logger(), "CCW speed = %d", native_speed);
      }

      // stop_position_state_ = position_state_;
      stop_position_state_ = position_state;
    }
    else
    {

      if (abs(velocity_state_) != 0)
      {

        feetech_handler_.SetCommand(id, stop_position_state_, 1);

        if (abs(velocity_state_) <= 100)
        {
          feetech_handler_.SetCommand(id, position_state_, 1);
          // RCLCPP_INFO(this->get_logger(), "CW speed = %d", 0);
        }
      }
    }
  }

  ////////////////////////////////////////////////

  void onTwistReceived(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // float position;
    // float velocity;

    float joy_ly = msg->axes[1] * -1;
    float joy_ry = msg->axes[3] * -1;

    bool x_button = msg->buttons[2]; // アーム1
    bool y_button = msg->buttons[3]; // アーム2
    bool a_button = msg->buttons[0]; // エア
    bool r_button = msg->buttons[5]; // リセット
    bool l_button = msg->buttons[4]; // ロック

    // auto s2_opt = feetech_handler_.GetStatus(23);
    // if (s2_opt)
    // {
    //   auto &status = s2_opt.value();
    //   position = status.position;
    //   velocity = status.velocity;
    // }

    onTwistReceived_servo(23, l_button, r_button);

    if(x_button && !y_button)
    {
      setCommand(25, 0.1);
    }
    else if(y_button && !x_button)
    {
      setCommand(25, -0.1);
    }
    else
    {
      setCommand(25, 0.0);
      setCommand(25, 0.0);
    }

    PwmGpio(12, fabs(joy_ry) * 0.8);
    PwmGpio(18, fabs(joy_ly) * 0.4);



    // ボタンが押された瞬間を検出（立ち下がりエッジ）
    if (lastButtonState_ == true && a_button == false)
    {
      air_state_ = !air_state_; // 状態を反転（ON⇔OFF）
      PwmGpio(13, air_state_ ? 0.97 : 0.0);
    }
    lastButtonState_ = a_button; // 前回値を更新

    DirGpio(7, joy_ry);
    DirGpio(8, joy_ly);

    RCLCPP_INFO(this->get_logger(), "%f, %f\n", joy_ly, joy_ry);
  }

  /////////////////////////////////////////
  // Float32MultiArrayを受け取るコールバック
  void onFloat32MultiArrayReceived(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {

    if (msg->data.size() >= 4)
    {
      float rps = msg->data[0];
      float timing = msg->data[1];
      float timing_2 = msg->data[2];
      float count = msg->data[3];

      RCLCPP_INFO(this->get_logger(), "Received Float32MultiArray: [rps: %.3f, timing: %.3f, timing_2: %.3f, count: %.3f]", rps, timing, timing_2, count);
      // 必要に応じてここで処理を追加する

      if (timing_2 == 1)
      {
        setCommandCustomPos(23, 250); // unlock
        RCLCPP_INFO(this->get_logger(), "------------------------unlocked------------------------");
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Invalid Float32MultiArray size");
    }
  }

  void setCommand(const int id, const float value)
  {
    feetech_handler_.SetCommand(id, 0, static_cast<int>(value * 3150)); // id, position, speed
    // RCLCPP_INFO(this->get_logger(), "%d\n", static_cast<int>(value * 3150));

    usleep(1000); // 1msのガード時間
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

private:
  FeetechHandler feetech_handler_;
  static constexpr int center_tick_ = 2048;
  static constexpr float tick_per_rad_ = 651.9f;

  // Float32MultiArrayのサブスクライバ
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr float32_subscriber_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // pigpioライブラリの初期化
  if (gpioInitialise() < 0)
  {
    std::cerr << "pigpioの初期化に失敗しました。" << std::endl;
    return 1;
  }

  // GPIOピンを出力モードに設定
  gpioSetMode(7, PI_OUTPUT);
  gpioSetMode(8, PI_OUTPUT);

  // PWM信号を開始
  gpioSetPWMfrequency(12, 20 * 1000); // 周波数を設定
  gpioSetPWMfrequency(18, 20 * 1000);
  gpioSetPWMfrequency(13, 20 * 1000); // エア

  auto pan_tilt_node = std::make_shared<PanTiltNode>();
  rclcpp::spin(pan_tilt_node);
  rclcpp::shutdown();
  return 0;
}
