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
#define LEFT_SHOULDER_PWM 5
#define LEFT_ELBOW_PWM 6

#define LEFT_SHOULDER_DIR 12
#define LEFT_ELBOW_DIR 13

// right arm motor pin name
// set 0 to 15 or GPIO
#define RIGHT_SHOULDER_PWM 7
#define RIGHT_ELBOW_PWM 8

#define RIGHT_SHOULDER_DIR 14
#define RIGHT_ELBOW_DIR 15

// slider motor pin name
// set 0 to 15 or GPIO
#define FRONT_SLIDER_PWM 10
#define BACK_SLIDER_PWM 11

#define FRONT_SLIDER_DIR 9
#define BACK_SLIDER_DIR 8

// air motor pin name
//set 0 to 15 or GPIO
#define LEFT_ARM_AIR_PWM EMPTY
#define RIGHT_ARM_AIR_PWM EMPTY
