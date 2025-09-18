#include <pigpio.h>

#include <cmath>
#include <iomanip>
#include <memory>

#include "Mecanum_on_Ros2/src/MecanumPCA9685_GPIO.hpp"
#include "SignalControl.hpp"
#include "feetech_handler.hpp"
#include "pan_tilt_ros_if.hpp"
// #include "robocon2025_a/slider.hpp"
// #include "PinName.hpp"
#include "PinName_v2.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <nlohmann/json.hpp>
#include <fstream>

using json = nlohmann::json;

class PanTiltNode : public PanTiltRosIf
{
 private:
  std::shared_ptr<PCA9685_RasPi> pcacontrol_0;
  std::shared_ptr<PCA9685_RasPi> pcacontrol_1;
  std::unique_ptr<MecanumPCA9685_GPIO> controller;

  json j_;

 public:
  PanTiltNode(void) : PanTiltRosIf{}
  {
    pcacontrol_0 = std::make_shared<PCA9685_RasPi>(OpenById{0, 1000});
    pcacontrol_1 = std::make_shared<PCA9685_RasPi>(OpenById{1, 1000});

    gpioSetMode(FRONT_OUTERLIMITTER, PI_INPUT);
    gpioSetMode(FRONT_INNERLIMITTER, PI_INPUT);
    gpioSetMode(BACK_OUTERLIMITTER, PI_INPUT);
    gpioSetMode(BACK_INNERLIMITTER, PI_INPUT);

    gpioSetPullUpDown(FRONT_OUTERLIMITTER, PI_PUD_DOWN);
    gpioSetPullUpDown(FRONT_INNERLIMITTER, PI_PUD_DOWN);
    gpioSetPullUpDown(BACK_OUTERLIMITTER, PI_PUD_DOWN);
    gpioSetPullUpDown(BACK_INNERLIMITTER, PI_PUD_DOWN);

    gpioGlitchFilter(FRONT_OUTERLIMITTER, 5000);
    gpioGlitchFilter(FRONT_INNERLIMITTER, 5000);
    gpioGlitchFilter(BACK_OUTERLIMITTER, 5000);
    gpioGlitchFilter(BACK_INNERLIMITTER, 5000);

    // パッケージのshareディレクトリを取得
    std::string pkg_path = ament_index_cpp::get_package_share_directory("robocon2025_a");
    std::string config_path = pkg_path + "/config/config.json";

    std::ifstream ifs(config_path);
    if (!ifs) {
        std::cerr << "config.json has not loaded." << std::endl;
        return;
    }
    
    try {
        ifs >> j_;
    } catch (const json::parse_error& e) {
        std::cerr << "json parse error " << e.what() << std::endl;
        return;
    }
    std::cout << "config.json has loaded." << std::endl;

    paramater.slider_vel = j_["slider"];
    paramater.foot_vel = j_["foot"]["velocity"];

    for(auto& elem : j_["LeftArm"]["theta_vel"])
      paramater.Left_theta_vel.push_back(elem);

    for(auto& elem : j_["RightArm"]["theta_vel"])
      paramater.Right_theta_vel.push_back(elem);

    for (int i = 0; i < 4; i++)
    {
      pwm_controls[i].handle = pcacontrol_0->handle();
      dir_controls[i].handle = pcacontrol_0->handle();
    }
    controller = std::make_unique<MecanumPCA9685_GPIO>(pwm_controls, dir_controls, pcacontrol_0, pcacontrol_0);

    controller->printConfigInfo();

    printf("start feetech\n");
    std::map<int, ServoConfig> config_list;
    config_list[2] = {-32237, 32236};   // 右アーム先端
    config_list[20] = {-32237, 32236};  // 右アーム肩
    config_list[23] = {-32237, 32236};  // 左アーム肩
    config_list[25] = {-32237, 32236};  // 左アーム先端

    // // // config_list[21] = {-32237, 32236};

    bool open_success = feetech_handler_.Initialize(config_list);
    if (!open_success)
    {
      printf("fail to open serial\n");
      throw;
    }
    else
      printf("success to open serial\n");

    // // デフォルトで無限回転モード
    feetech_handler_.SetOperatingMode(2, 1);
    feetech_handler_.SetOperatingMode(20, 1);
    feetech_handler_.SetOperatingMode(23, 1);
    feetech_handler_.SetOperatingMode(25, 1);

    // feetech_handler_.SetOperatingMode(21, 1);

    // Float32MultiArrayのサブスクライバを作成
    // float32_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("feedback_data", 10, std::bind(&PanTiltNode::onFloat32MultiArrayReceived, this, _1));
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

    auto s1_opt = feetech_handler_.GetStatus(2);
    if (s1_opt)
    {
      // 取得成功
      auto &status = s1_opt.value();
      joint_state.name.push_back("right_arm_hand");
      joint_state.position.push_back(static_cast<float>(status.position));
      joint_state.velocity.push_back(static_cast<float>(status.velocity));

      position_state_[0] = status.position;
      velocity_state_[0] = status.velocity;

      // std::cout << "|  " << "id: 2, " << "position: " << status.position << ", velocity: " << status.velocity << "|  joyData: " << joy_data << "\r" << std::flush;
    }
    else
    {
      // std::cout << 2 << ": サーボの状態取得に失敗" << std::endl;
    }

    auto s2_opt = feetech_handler_.GetStatus(20);
    if (s2_opt)
    {
      // 取得成功
      auto &status = s2_opt.value();
      joint_state.name.push_back("right_arm_shoulder");
      joint_state.position.push_back(static_cast<float>(status.position));
      joint_state.velocity.push_back(static_cast<float>(status.velocity));

      position_state_[1] = status.position;
      velocity_state_[1] = status.velocity;
    }
    else
    {
      // std::cout << 20 << ": サーボの状態取得に失敗" << std::endl;
    }

    auto s3_opt = feetech_handler_.GetStatus(23);
    if (s3_opt)
    {
      // 取得成功
      auto &status = s3_opt.value();
      joint_state.name.push_back("left_arm_shoulder");
      joint_state.position.push_back(static_cast<float>(status.position));
      joint_state.velocity.push_back(static_cast<float>(status.velocity));

      position_state_[2] = status.position;
      velocity_state_[2] = status.velocity;
    }
    else
    {
      // std::cout << 23 << ": サーボの状態取得に失敗" << std::endl;
    }

    auto s4_opt = feetech_handler_.GetStatus(25);
    if (s4_opt)
    {
      // 取得成功
      auto &status = s4_opt.value();
      joint_state.name.push_back("left_arm_hand");
      joint_state.position.push_back(static_cast<float>(status.position));
      joint_state.velocity.push_back(static_cast<float>(status.velocity));

      position_state_[3] = status.position;
      velocity_state_[3] = status.velocity;
    }
    else
    {
      // std::cout << 25 << ": サーボの状態取得に失敗" << std::endl;
    }

    publishJointState(joint_state);
  }

  ///////////////////////////////////////

  // id: 23, home position: 2000
  // id: 20, home position: 1600

  // void setCommandLimitted(int id, float value) {
  // }

  ////////////////////////////////////////////////

  void onTwistReceivedFoot(const sensor_msgs::msg::Joy::SharedPtr msg) override
  {
    // std::cout << "a" << std::endl;
    float joy_lx = msg->axes[0];
    float joy_ly = msg->axes[1] * -1;
    float joy_rx = msg->axes[2];
    float joy_ry = msg->axes[3] * -1;

    bool button_up = msg->buttons[11];
    bool button_down = msg->buttons[12];

    bool FOLimSwitch = gpioRead(FRONT_OUTERLIMITTER);
    bool FCLimSwitch = gpioRead(FRONT_INNERLIMITTER);
    bool BOLimSwitch = gpioRead(BACK_OUTERLIMITTER);
    bool BCLimSwitch = gpioRead(BACK_INNERLIMITTER);

    joy_data = joy_ly;

    // bool r_button = msg->buttons[2];  // アーム1
    // bool l_button = msg->buttons[3];  // アーム2
    // bool a_button = msg->buttons[0];  // エア
    // bool r2_button = msg->buttons[5];  // リセット
    // bool l2_button = msg->buttons[4];  // ロック

    // // ボタンが押された瞬間を検出（立ち下がりエッジ）
    if (Foot.lastButtonState_ == true && button_up == false)
    {
      Foot.air_state_ = !Foot.air_state_;  // 状態を反転（ON⇔OFF）
      pcacontrol_1->setPwm(FRONT_SLIDER_PWM, Foot.air_state_ ? 0.97 : 0.0);
    }
    Foot.lastButtonState_ = button_up;  // 前回値を更新

    if (Foot2.lastButtonState_ == true && button_down == false)
    {
      Foot2.air_state_ = !Foot2.air_state_;  // 状態を反転（ON⇔OFF）
      pcacontrol_1->setPwm(BACK_SLIDER_PWM, Foot2.air_state_ ? 0.97 : 0.0);
    }
    Foot2.lastButtonState_ = button_down;  // 前回値を更新

    if(FCLimSwitch && !FOLimSwitch){
      pcacontrol_1->setPwm(FRONT_SLIDER_DIR, 0);
    }else if(!FCLimSwitch && FOLimSwitch){
      pcacontrol_1->setPwm(FRONT_SLIDER_DIR, 1);
    }

    if(BCLimSwitch && !BOLimSwitch){
      pcacontrol_1->setPwm(BACK_SLIDER_DIR, 0);
    }else if(!BCLimSwitch && BOLimSwitch){
      pcacontrol_1->setPwm(BACK_SLIDER_DIR, 1);
    }

    // テストコード
    // pcacontrol_1->setPwm(FRONT_SLIDER_PWM, button_up ? 0.5 : 0.0);
    // pcacontrol_1->setPwm(BACK_SLIDER_PWM, button_down ? 0.5 : 0.0);

    controller->pinWrite(joy_lx * paramater.foot_vel, joy_ly * paramater.foot_vel, joy_rx * paramater.foot_vel);

    controller->printControlInfo();

            std::cout << "\n" << std::setw(11) << std::left << joy_lx << " | " << std::setw(11) << std::left << joy_ly << " | " << std::setw(11) << std::left << joy_rx << " | " << std::setw(11) << std::left << joy_ry << " | " << std::left << (FOLimSwitch ? 1 : 0) << std::left << (FCLimSwitch ? 1 : 0) << std::left << (BOLimSwitch ? 1 : 0) << std::left << (BCLimSwitch ? 1 : 0) << std::flush;

    for (int i = 0; i < 6; i++)
    {
      move_up(1);
      clear_line();
    }
  }

  void onTwistReceivedLeftArm(const sensor_msgs::msg::Joy::SharedPtr msg) override
  {
    float position;
    float velocity;
    float joy_ly = msg->axes[1] * -1;
    float joy_ry = msg->axes[3] * -1;

    bool a_button = msg->buttons[0];  // エア
    bool r2_button = msg->axes[5];  // 右旋回
    bool l2_button = msg->axes[4];  // 左旋回
    bool r_button = msg->buttons[5];  // 先端右旋回
    bool l_button = msg->buttons[4];  // 先端左旋回
    // auto s1_opt = feetech_handler_.GetStatus(20);
    // if (s1_opt)
    // {
    //   auto &status = s1_opt.value();
    //   position = status.position;
    //   velocity = status.velocity;
    // }

    // setCommand(20, joy_ly);

    pcacontrol_1->setPwm(LEFT_SHOULDER_PWM, std::fabs(joy_ly) * paramater.Left_theta_vel[1]);
    pcacontrol_1->setPwm(LEFT_ELBOW_PWM, std::fabs(joy_ry) * paramater.Left_theta_vel[2]);

    pcacontrol_0->setDigital(LEFT_SHOULDER_DIR, (joy_ly >= 0.0 ? false : true));
    pcacontrol_0->setDigital(LEFT_ELBOW_DIR, (joy_ry >= 0.0 ? false : true));

    if (l2_button && !r2_button)
    {
      setCommand(LEFT_SHOULDER, paramater.Left_theta_vel[0] * l2_button);
    }
    else if (!l2_button && r2_button)
    {
      setCommand(LEFT_SHOULDER, -paramater.Left_theta_vel[0] * r2_button);
    }
    else
    {
      setCommand(LEFT_SHOULDER, 0.0);
      setCommand(LEFT_SHOULDER, 0.0);
    }

    if (r_button && !l_button)
    {
      setCommand(LEFT_HAND, paramater.Left_theta_vel[3]);
    }
    else if (!r_button && l_button)
    {
      setCommand(LEFT_HAND, -paramater.Left_theta_vel[3]);
    }
    else
    {
      setCommand(LEFT_HAND, 0.0);
      setCommand(LEFT_HAND, 0.0);
    }

    // ボタンが押された瞬間を検出（立ち下がりエッジ）
    if (leftArm.lastButtonState_ == true && a_button == false)
    {
      leftArm.air_state_ = !leftArm.air_state_;  // 状態を反転（ON⇔OFF）
      pcacontrol_1->setPwm(LEFT_ARM_AIR_PWM, leftArm.air_state_ ? 0.97 : 0.0);
      // PwmGpio(13, leftArm.air_state_ ? 0.97 : 0.0);
    }
    leftArm.lastButtonState_ = a_button;  // 前回値を更新

   // test-code
  //  pcacontrol_1->setPwm(LEFT_ARM_AIR_PWM, a_button ? 0.97 : 0.0);

    // std::cout << std::left << std::setw(11) << (leftArm.air_state_ ? "true" : "false") << "\r" << std::flush;
  }

  void onTwistReceivedRightArm(const sensor_msgs::msg::Joy::SharedPtr msg) override
  {
    float position;
    float velocity;
    float joy_ly = msg->axes[1] * -1;
    float joy_ry = msg->axes[3] * -1;

    bool a_button = msg->buttons[0];  // エア
    bool r2_button = msg->axes[5];  // 右旋回
    bool l2_button = msg->axes[4];  // 左旋回
    bool r_button = msg->buttons[5];  // 先端右旋回
    bool l_button = msg->buttons[4];  // 先端左旋回
    // auto s1_opt = feetech_handler_.GetStatus(20);
    // if (s1_opt)
    // {
    //   auto &status = s1_opt.value();
    //   position = status.position;
    //   velocity = status.velocity;
    // }

    pcacontrol_1->setPwm(RIGHT_SHOULDER_PWM, std::fabs(joy_ly) * paramater.Right_theta_vel[1]);
    pcacontrol_1->setPwm(RIGHT_ELBOW_PWM, std::fabs(joy_ry) * paramater.Right_theta_vel[2]);

    pcacontrol_0->setDigital(RIGHT_SHOULDER_DIR, (joy_ly >= 0.0 ? false : true));
    pcacontrol_0->setDigital(RIGHT_ELBOW_DIR, (joy_ry >= 0.0 ? false : true));

    if (l2_button && !r2_button)
    {
      setCommand(RIGHT_SHOULDER, paramater.Right_theta_vel[0] * r2_button);
    }
    else if (r2_button && !l2_button)
    {
      setCommand(RIGHT_SHOULDER, -paramater.Right_theta_vel[0] * l2_button);
    }
    else
    {
      setCommand(RIGHT_SHOULDER, 0.0);
      setCommand(RIGHT_SHOULDER, 0.0);
    }

    if (r_button && !l_button)
    {
      setCommand(RIGHT_HAND, paramater.Right_theta_vel[3]);
    }
    else if (!r_button && l_button)
    {
      setCommand(RIGHT_HAND, -paramater.Right_theta_vel[3]);
    }
    else
    {
      setCommand(RIGHT_HAND, 0.0);
      setCommand(RIGHT_HAND, 0.0);
    }

    // // ボタンが押された瞬間を検出（立ち下がりエッジ）
    if (rightArm.lastButtonState_ == true && a_button == false)
    {
      rightArm.air_state_ = !rightArm.air_state_;  // 状態を反転（ON⇔OFF）
      pcacontrol_1->setPwm(RIGHT_ARM_AIR_PWM, rightArm.air_state_ ? 0.97 : 0.0);
      // PwmGpio(13, rightArm.air_state_ ? 0.97 : 0.0);
    }
    rightArm.lastButtonState_ = a_button;  // 前回値を更新

   // test-code
  //  pcacontrol_1->setPwm(LEFT_ARM_AIR_PWM, a_button ? 0.97 : 0.0);

    // std::cout << std::left << std::setw(11) << (rightArm.air_state_ ? "true" : "false") << "\r" << std::flush;
  }

  /////////////////////////////////////////////////////////////////

  /////////////////////////////////////////////////////////////////

  void clear_line()
  {
    std::cout << "\33[2K\r";  // 行をクリアしてカーソルを行頭へ
  }

  void move_up(int n)
  {
    std::cout << "\33[" << n << "A";  // 上にn行
  }

  void move_down(int n)
  {
    std::cout << "\33[" << n << "B";  // 下にn行
  }

  /////////////////////////////////////////

  void setCommand(const int id, const float value)
  {
    feetech_handler_.SetCommand(id, 0, static_cast<int>(value * 3150));  // id, position, speed
    // RCLCPP_INFO(this->get_logger(), "%d\n", static_cast<int>(value * 3150));

    usleep(1000);  // 1msのガード時間
  }

  void setCommandCustomPos(const int id, const int value)
  {
    feetech_handler_.SetOperatingMode(id, 0);
    feetech_handler_.SetCommand(id, value, 0);
    feetech_handler_.SetOperatingMode(id, 1);
    usleep(100000);  // 100ms
  }

  void setTorqueEnable(const int id, const bool state)
  {
    feetech_handler_.SetTorqueEnable(id, state);
    usleep(100000);
  }

 private:
  FeetechHandler feetech_handler_;
  // static constexpr int center_tick_ = 2048;
  // static constexpr float tick_per_rad_ = 651.9f;

  // Float32MultiArrayのサブスクライバ
  // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr float32_subscriber_;
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

  auto pan_tilt_node = std::make_shared<PanTiltNode>();
  rclcpp::spin(pan_tilt_node);
  rclcpp::shutdown();
  return 0;
}
