/**
 * @file trajectorygenerator.h
 * @brief This file defines the TrajectoryGenerator class, it is the base class for all trajectory
 * generators.
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "proseco_planning/util/alias.h"

namespace proseco_planning {
class Trajectory;
class Vehicle;

/**
 * @brief 边界条件定义了轨迹生成的约束
 * 
 */
struct BoundaryCondition {
  /// 限制轨迹的位置。
  float position{0.0f};

  /// 限制轨迹的速度
  float velocity{0.0f};

  /// 限制轨迹的加速度
  float acceleration{0.0f};
};

/**
 * @brief TrajectoryGenerator类定义所有轨迹生成器的基类
 */

class TrajectoryGenerator {
 public:
  explicit TrajectoryGenerator(const std::string& name);

  /// 虚拟析构函数
  virtual ~TrajectoryGenerator() = default;

  static std::unique_ptr<TrajectoryGenerator> createTrajectoryGenerator(const std::string& name);

  Trajectory createTrajectory(float t0, ActionPtr action, const Vehicle& vehicle) const;

  /// 轨迹生成方法的名称
  std::string m_name;

 protected:
  virtual std::tuple<BoundaryCondition, BoundaryCondition, BoundaryCondition, BoundaryCondition>
  createBoundaryConditions(ActionPtr action, const Vehicle& vehicle) const = 0;

  virtual Trajectory calculateTrajectory(const Vehicle& vehicle, const float t0,
                                         const BoundaryCondition& startS,
                                         const BoundaryCondition& startD,
                                         const BoundaryCondition& endS,
                                         const BoundaryCondition& endD) const = 0;

  virtual void updateFinalState(Trajectory& trajectory) const = 0;

  virtual void evaluateTrajectory(Trajectory& trajectory, const BoundaryCondition& startS,
                                  const BoundaryCondition& startD, const BoundaryCondition& endS,
                                  const BoundaryCondition& endD) const;

  virtual void calculateCumulativeAcceleration(Trajectory& trajectory,
                                               const BoundaryCondition& startS,
                                               const BoundaryCondition& startD,
                                               const BoundaryCondition& endS,
                                               const BoundaryCondition& endD) const = 0;

  void detectLaneChange(Trajectory& trajectory) const;
};
}  // namespace proseco_planning