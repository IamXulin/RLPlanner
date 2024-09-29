/**
 * @file action.h
 * @brief This file defines the Action class as well as the ActionNoise struct.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <memory>
#include <vector>

#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/config/scenarioOptions.h"
#include "proseco_planning/util/alias.h"

namespace proseco_planning {

class ActionSpace;
class Vehicle;

/**
 * @brief 该结构定义了将噪声添加到动作中时车辆行为的估计
 */
struct ActionNoise {
  /// 当噪声添加到动作中时 Y 的可能性
  float m_likelihoodY{0.0f};

  /// 当噪声添加到动作中时 Vx 的可能性
  float m_likelihoodVx{0.0f};

  /// Y 的期望值
  float m_muY{0.0f};

  /// Vx的期望值
  float m_muVx{0.0f};

  /// 标准。 Y 的偏差
  float m_sigmaY{0.0f};

  /// 标准。 Vx的偏差
  float m_sigmaVx{0.0f};
};

/**
 * @brief Action 类是动作的通用类
 */
class Action {
 public:
  explicit Action(ActionClass actionClass);

  Action(float velocityChange, float lateralChange);

  Action(ActionClass actionClass, float accelerationX, float accelerationY);

  void updateActionClass(const ActionSpace& actionSpace, const Vehicle& vehicle);

  static float getSimilarity(const ActionPtr& x, const ActionPtr& y, const float gamma);

  static float getSimilarity(const ActionPtr& x, const ActionPtr& y);

  float getSquaredDistance(const ActionPtr& action = nullptr) const;

  float getDistance(const ActionPtr& action = nullptr) const;

  /// 动作的类别
  ActionClass m_actionClass;

  /// 纵向速度变化
  const float m_velocityChange;

  /// 横向位置变化
  const float m_lateralChange;

  /// 纵向加速度
  const float m_accelerationX;

  /// 横向加速度
  const float m_accelerationY;

  /// 给定车辆当前状态的操作的有效性
  bool m_invalidAction{false};

  /// 添加到动作中的噪音.
  ActionNoise noise;

  /// 选择该动作的概率。 目前仅用于exp_q采样最终选择
  float m_selectionLikelihood{0.0f};

  /// 选择该动作的权重
  std::vector<float> m_selectionWeights;
};

void to_json(json& j, const Action& action);
}  // namespace proseco_planning
