#include "SignalControl.hpp"
#include "PinName.hpp"

PWMTYPE pwm_controls[4] = 
{
    {ControlType::PCA9685, 0, WHEEL_FL_PWM, 1000},
    {ControlType::PCA9685, 0, WHEEL_FR_PWM, 1000},
    {ControlType::PCA9685, 0, WHEEL_BL_PWM, 1000},
    {ControlType::PCA9685, 0, WHEEL_BR_PWM, 1000}
};

DIRTYPE dir_controls[4] = 
{
    {ControlType::RASPIGPIO, -1, WHEEL_FL_DIR, 500, false},
    {ControlType::RASPIGPIO, -1, WHEEL_FR_DIR, 500, true},
    {ControlType::RASPIGPIO, -1, WHEEL_BL_DIR, 500, true},
    {ControlType::RASPIGPIO, -1, WHEEL_BR_DIR, 500, true}
};
