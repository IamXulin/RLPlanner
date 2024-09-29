#include "proseco_planning/trajectory/trajectorygenerator.h"

#include <cstddef>
#include <iostream>
#include <memory>
#include <tuple>

#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/scenarioOptions.h"
#include "proseco_planning/trajectory/constantacceleration.h"
#include "proseco_planning/trajectory/polynomialgenerator.h"
#include "proseco_planning/trajectory/trajectory.h"

namespace proseco_planning {
class Vehicle;

/**
 * @brief 构造一个新的轨迹生成器对象
 *
 * @param name 轨迹生成方法
 */
TrajectoryGenerator::TrajectoryGenerator(const std::string& name) : m_name(name) {}

/**
 * @brief 创建轨迹生成器（工厂方法）
 *
 * @param name 轨迹生成方法
 * @return std::unique_ptr<TrajectoryGenerator> 轨迹生成器
 */
std::unique_ptr<TrajectoryGenerator> TrajectoryGenerator::createTrajectoryGenerator(
    const std::string& name) {
  if (name == "jerkOptimal") {
    return std::make_unique<PolynomialGenerator>(name);
  } else if (name == "constantAcceleration") {
    return std::make_unique<ConstantAcceleration>(name);
  } else {
    throw std::invalid_argument("Unknown trajectory generation type: " + name);
  }
}

/**
 * @brief 根据所选动作更新轨迹的边界条件和最终状态
 * @details 该方法还计算并评估轨迹
 *
 * @param t0 轨迹的开始时间
 * @param trajectory 应更新最终状态的轨迹
 * @param action 所选择的动作
 * @param vehicle 车辆的当前状态
 */

Trajectory TrajectoryGenerator::createTrajectory(float t0, ActionPtr action,
                                                 const Vehicle& vehicle) const {
  // 使用 action = [dLat, dV] + currentState 计算边界条件
  const auto [startS, startD, endS, endD] = createBoundaryConditions(action, vehicle);

  // 计算轨迹的离散表示
  auto trajectory = calculateTrajectory(vehicle, t0, startS, startD, endS, endD);
  // 计算与操作相关的成本
  evaluateTrajectory(trajectory, startS, startD, endS, endD);

  // 用于更新车辆状态的紧凑表示，无需集成运动学模型
  updateFinalState(trajectory);
  return trajectory;
}

/**
 * @brief 根据边界条件计算加速成本
 *
 * @param trajectory 待评估的轨迹
 * @param startS 与起点纵向方向相关的边界条件
 * @param startD 起始时与横向方向相关的边界条件
 * @param endS 与末端纵向方向相关的约束条件
 * @param endD 与末端横向方向相关的边界条件
 */
void TrajectoryGenerator::evaluateTrajectory(Trajectory& trajectory,
                                             const BoundaryCondition& startS,
                                             const BoundaryCondition& startD,
                                             const BoundaryCondition& endS,
                                             const BoundaryCondition& endD) const {
  calculateCumulativeAcceleration(trajectory, startS, startD, endS, endD);

  TrajectoryGenerator::detectLaneChange(trajectory);
  trajectory.calculateAverageSpeed();
  trajectory.calculateAverageAbsoluteAcceleration();
}

/**
 * @brief 检查轨迹中的车道变化并相应地更新轨迹中的信息
 *
 * @param trajectory 检查车道变换的轨迹
 */
void TrajectoryGenerator::detectLaneChange(Trajectory& trajectory) const {
  trajectory.determineLane();
  trajectory.determineLaneChange();
}

}  // namespace proseco_planning
