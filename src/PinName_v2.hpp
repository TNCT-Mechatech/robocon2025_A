#pragma once

// Delete "_EMPTY" and enter DC motor pin name

// PCA9685
// ex. 7, etc...

// RaspBerryPi 4b GPIO
// ex. 17, etc...

// mecanum wheel pwm pin name
// set 0 to 15 or GPIO pin name
#define WHEEL_FR_PWM 0  // 0
#define WHEEL_FL_PWM 2  // 0
#define WHEEL_BR_PWM 4  // 0
#define WHEEL_BL_PWM 6  // 0

// mecanum wheel dir pin name
// set 0 to 15 or GPIO
#define WHEEL_FR_DIR 1  // 0
#define WHEEL_FL_DIR 3  // 0
#define WHEEL_BR_DIR 5  // 0
#define WHEEL_BL_DIR 7  // 0

// left arm motor pin name
// set 0 to 15 or GPIO
#define LEFT_SHOULDER_PWM 8  // 1
#define LEFT_ELBOW_PWM 9     // 1

#define LEFT_SHOULDER_DIR 8  // 0
#define LEFT_ELBOW_DIR 9     // 0

// right arm motor pin name
// set 0 to 15 or GPIO
#define RIGHT_SHOULDER_PWM 10  // 1
#define RIGHT_ELBOW_PWM 11     // 1

#define RIGHT_SHOULDER_DIR 10  // 0
#define RIGHT_ELBOW_DIR 11     // 0

// slider motor pin name
// set 0 to 15 or GPIO
#define FRONT_SLIDER_PWM 1  // 1
#define BACK_SLIDER_PWM 3   // 1

#define FRONT_SLIDER_DIR 0  // 1
#define BACK_SLIDER_DIR 2   // 1

// air motor pin name
// set 0 to 15 or GPIO
#define LEFT_ARM_AIR_PWM 5   // 1
#define RIGHT_ARM_AIR_PWM 7  // 1

// servo id
#define RIGHT_HAND 2
#define RIGHT_SHOULDER 20
#define LEFT_HAND 25
#define LEFT_SHOULDER 23