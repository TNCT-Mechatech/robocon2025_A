#include "trapezoidal_acc.hpp"
#include <cmath>

// コンストラクタ: 最大加速度、最大減速度、現在速度を初期化
TrapezoidalAcc::TrapezoidalAcc(float max_acc, float max_dec)
    : max_acc_(max_acc), max_dec_(max_dec), current_vel_(0.0f) {}

// 台形加速の強さを設定する関数
void TrapezoidalAcc::setAccStrength(float max_acc, float max_dec) {
  max_acc_ = max_acc;
  max_dec_ = max_dec;
}

// 台形加速に基づいて速度を計算する関数
float TrapezoidalAcc::updateVel(float target_vel, float dt) {
  float vel_diff = target_vel - current_vel_;  // 目標速度との差

  // 加速または減速を計算
  if (vel_diff > 0) {
    // 加速フェーズ
    current_vel_ += max_acc_ * dt;
    if (current_vel_ > target_vel) {
      current_vel_ = target_vel;  // 目標速度を超えないように補正
    }
  } else if (vel_diff < 0) {
    // 減速フェーズ
    current_vel_ -= max_dec_ * dt;
    if (current_vel_ < target_vel) {
      current_vel_ = target_vel;  // 目標速度を下回らないように補正
    }
  }

  // 現在の速度を返す
  return current_vel_;
}