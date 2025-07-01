#pragma once

// Delete "_EMPTY" and enter DC motor pin name

// PCA9685
// ex. 7, etc...

// RaspBerryPi 4b GPIO
// ex. 17, etc...

// mecanum wheel pwm pin name
// set 0 to 15 or GPIO pin name
#define WHEEL_FR_PWM 0
#define WHEEL_FL_PWM 1
#define WHEEL_BR_PWM 2
#define WHEEL_BL_PWM 3

// mecanum wheel dir pin name
// set 0 to 15 or GPIO
#define WHEEL_FR_DIR 21
#define WHEEL_FL_DIR 20
#define WHEEL_BR_DIR 16
#define WHEEL_BL_DIR 12

// left arm motor pin name
// set 0 to 15 or GPIO
#define LEFT_SHOULDER_PWM 4
#define LEFT_ELBOW_PWM 5

#define LEFT_SHOULDER_DIR 8
#define LEFT_ELBOW_DIR 9

// right arm motor pin name
// set 0 to 15 or GPIO
#define RIGHT_SHOULDER_PWM 6
#define RIGHT_ELBOW_PWM 7

#define RIGHT_SHOULDER_DIR 10
#define RIGHT_ELBOW_DIR 11

// slider motor pin name
// set 0 to 15 or GPIO
#define FRONT_SLIDER_PWM 12
#define BACK_SLIDER_PWM 13

#define FRONT_SLIDER_DIR 1
#define BACK_SLIDER_DIR 7

// air motor pin name
//set 0 to 15 or GPIO
#define LEFT_ARM_AIR_PWM 14
#define RIGHT_ARM_AIR_PWM 15
