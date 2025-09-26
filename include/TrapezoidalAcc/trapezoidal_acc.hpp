#ifndef TRAPEZOIDAL_ACC_HPP
#define TRAPEZOIDAL_ACC_HPP

#include <vector>

// 台形加速を行うクラス
class TrapezoidalAcc {
public:
  // コンストラクタ: 最大加速度、最大減速度を設定
  TrapezoidalAcc(float max_acc, float max_dec);

  // 台形加速で速度を更新
  // target_vel: 目標速度
  // dt: 時間ステップ
  float updateVel(float target_vel, float dt);

  // 台形加速の強さを設定
  void setAccStrength(float max_acc, float max_dec);

private:
  float max_acc_;   // 最大加速度
  float max_dec_;   // 最大減速度
  float current_vel_; // 現在速度（内部管理用）
};

#endif // TRAPEZOIDAL_ACC_HPP