/**
 * @file trajectory.h
 * @brief This file defines the Trajectory class.
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

#include <cstddef>
#include <vector>

#include "nlohmann/json.hpp"
using json = nlohmann::json;

namespace proseco_planning {
class Vehicle;

/**
 * @brief Trajectory 类定义车辆的轨迹
 *
 */
class Trajectory {
 public:
  Trajectory(const float t0, const float initialHeading);

  size_t getFractionIndex() const;

  static float getCurrentFraction();

  void determineLane();

  void determineLaneChange();

  void calculateAverageSpeed();

  void calculateAverageAbsoluteAcceleration();

  bool isValidAction(const Vehicle& vehicle) const;

  bool isValidState(const Vehicle& vehicle) const;

  ///@{
  /// 轨迹的开始和结束时间
  float m_t0;
  float m_t1;
  ///@}
  ///@{
  /// 轨迹的开始和结束时间的 2 次方
  float m_t0_2;
  float m_t1_2;
  ///@}

  /// 步数
  size_t m_nSteps;

  ///@{
  /// 包含每个时间步的轨迹值的向量
  std::vector<float> m_time;
  std::vector<float> m_sPosition;
  std::vector<float> m_dPosition;
  std::vector<float> m_sVelocity;
  std::vector<float> m_dVelocity;
  std::vector<float> m_sAcceleration;
  std::vector<float> m_dAcceleration;
  std::vector<float> m_curvature;
  std::vector<int> m_lane;
  std::vector<float> m_heading;
  std::vector<float> m_steeringAngle;
  std::vector<float> m_totalVelocity;
  std::vector<float> m_totalAcceleration;
  ///@}

  /// 更新车辆状态的最终状态
  std::vector<float> m_finalState{std::vector<float>(8)};

  /// 当前场景的平均速度--[m/s]
  float m_averageVelocity{0.0f};

  /// 当前场景下的平均绝对加速度 -- [m/s^2]
  float m_averageAbsoluteAcceleration{0.0f};

  /// 轨迹纵向累积平方加速度
  float m_cumSquaredAccelerationLon{0.0f};

  /// 横向轨迹累积平方加速度
  float m_cumSquaredAccelerationLat{0.0f};

  /// 确定是否执行完整轨迹（在 MCTS 搜索期间）或是否仅执行一小部分（在“环境”= ProSeCoPlanner 中选择的操作）的标志
  static bool useActionFraction;

  /// 车道变更量
  int m_laneChange{0};

  /// 该标志指示轨迹是否导致无效动作
  bool m_invalidAction{false};

  /// 该标志表明轨迹是否导致无效状态
  bool m_invalidState{false};
};
void to_json(json& j, const Trajectory& trajectory);
}  // namespace proseco_planning
