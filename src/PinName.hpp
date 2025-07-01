#pragma once

// Delete "_EMPTY" and enter DC motor pin name

// PCA9685
// ex. 7, etc...

// RaspBerryPi 4b GPIO
// ex. 17, etc...

// mecanum wheel pwm pin name
// set 0 to 15 or GPIO pin name
#define WHEEL_FR_PWM _0
#define WHEEL_FL_PWM _1
#define WHEEL_BR_PWM _2
#define WHEEL_BL_PWM _3

// mecanum wheel dir pin name
// set 0 to 15 or GPIO
#define WHEEL_FR_DIR _GPIO21
#define WHEEL_FL_DIR _GPIO20
#define WHEEL_BR_DIR _GPIO16
#define WHEEL_BL_DIR _GPIO12

// left arm motor pin name
// set 0 to 15 or GPIO
#define LEFT_SHOULDER_PWM _5
#define LEFT_ELBOW_PWM _6

#define LEFT_SHOULDER_DIR _12
#define LEFT_ELBOW_DIR _13

// right arm motor pin name
// set 0 to 15 or GPIO
#define RIGHT_SHOULDER_PWM _7
#define RIGHT_ELBOW_PWM _8

#define RIGHT_SHOULDER_DIR _14
#define RIGHT_ELBOW_DIR _15

// slider motor pin name
// set 0 to 15 or GPIO
#define FRONT_SLIDER_PWM _10
#define BACK_SLIDER_PWM _11

#define FRONT_SLIDER_DIR _9
#define BACK_SLIDER_DIR _8

// air motor pin name
//set 0 to 15 or GPIO
#define LEFT_ARM_AIR_PWM _EMPTY
#define RIGHT_ARM_AIR_PWM _EMPTY
