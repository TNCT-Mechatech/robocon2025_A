#ifndef _KILLER_SLIDE_
#define _KILLER_SLIDE_

#include <pigpio.h>

#include <cmath>
#include <iostream>
#include <string>

class Morter_front
{
 private:
  int pwm_front;
  int dir_front;
  int switch_front;

 public:
  Morter_front(int pwm, int dir, int sw) : pwm_front(pwm), dir_front(dir), switch_front(sw) {}

  bool init()
  {
    if (gpioInitialise() < 0)
    {
      std::cerr << "pigpioの初期化に失敗しました" << std::endl;
      return false;
    }
    gpioSetMode(pwm_front, PI_OUTPUT);
    gpioSetMode(dir_front, PI_OUTPUT);
    gpioSetMode(switch_front, PI_INPUT);

    gpioWrite(dir_front, 1);
    return true;
  }
  void update()
  {
    int sw = gpioRead(switch_front);
    if (sw == 1)
    {
      gpioPWM(pwm_front, 128);
    }
    else
    {
      gpioPWM(pwm_front, 0);
    }
  }

  void shutdown()
  {
    gpioPWM(pwm_front, 0);
    gpioTerminate();
  }
};

class Slider_PCA9685
{
 private:
 PCA9685_RasPi pca_controller_F;
 PCA9685_RasPi pca_controller_B;

 public:
  Slider_PCA9685(int id_F, int id_B, float freq_F, float freq_B) : pca_controller_F(id_F, freq_F), pca_controller_B(id_B, freq_B){}

  void pinWrite(int channel_F, int channel_B, float duty, bool input_F, bool )
  {
    if()
    pca_controller.setPwm(channel_F, duty);
    pca_controller.setPwm(channel_B, duty);
  }
}

#endif