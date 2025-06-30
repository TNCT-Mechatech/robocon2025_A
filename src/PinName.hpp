#pragma once

// Delete "_EMPTY" and enter DC motor pin name

// PCA9685
// ex. 7, etc...

// RaspBerryPi 4b GPIO
// ex. 17, etc...

// mecanum wheel pwm pin name
// set 0 to 15 or GPIO pin name
#define WHEEL_FR_PWM _EMPTY
#define WHEEL_FL_PWM _EMPTY
#define WHEEL_BR_PWM _EMPTY
#define WHEEL_BL_PWM _EMPTY

// mecanum wheel dir pin name
// set 0 to 15 or GPIO
#define WHEEL_FR_DIR _EMPTY
#define WHEEL_FL_DIR _EMPTY
#define WHEEL_BR_DIR _EMPTY
#define WHEEL_BL_DIR _EMPTY

// left arm motor pin name
// set 0 to 15 or GPIO
#define LEFT_SHOULDER_PWM _EMPTY
#define LEFT_ELBOW_PWM _EMPTY

#define LEFT_SHOULDER_DIR _EMPTY
#define LEFT_ELBOW_DIR _EMPTY

// right arm motor pin name
// set 0 to 15 or GPIO
#define RIGHT_SHOULDER_PWM _EMPTY
#define RIGHT_ELBOW_PWM _EMPTY

#define RIGHT_SHOULDER_DIR _EMPTY
#define RIGHT_ELBOW_DIR _EMPTY

// slider motor pin name
// set 0 to 15 or GPIO
#define FRONT_SLIDER_PWM _EMPTY
#define BACK_SLIDER_PWM _EMPTY

#define FRONT_SLIDER_DIR _EMPTY
#define BACK_SLIDER_DIR _EMPTY

// air motor pin name
//set 0 to 15 or GPIO
#define LEFT_ARM_AIR_PWM _EMPTY
#define RIGHT_ARM_AIR_PWM _EMPTY