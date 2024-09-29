#include "proseco_planning/trajectory/constantacceleration.h"

#include <cstddef>
#include <memory>

#include "proseco_planning/action/action.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/trajectory/trajectory.h"

namespace proseco_planning {

/**
 * @brief 创建轨迹生成的边界条件
 *
 * @param action 要执行的动作
 * @param trajectory 应为其创建边界条件的轨迹
 * @param vehicle 车辆当前状态
 * @return std::tuple<边界条件，边界条件，边界条件，边界条件>
 * 边界条件，从纵向和横向的起始条件开始，然后是纵向和横向的终止条件
 */
std::tuple<BoundaryCondition, BoundaryCondition, BoundaryCondition, BoundaryCondition>
ConstantAcceleration::createBoundaryConditions(ActionPtr action, const Vehicle& vehicle) const {
  // 转换动作：delta Vs、delta d 至加速度
  BoundaryCondition startS;
  BoundaryCondition startD;
  BoundaryCondition endS;
  BoundaryCondition endD;

  // Initial
  // 初始位置
  startS.position = vehicle.m_positionX;
  startD.position = vehicle.m_positionY;
  // 初始速度
  startS.velocity = vehicle.m_velocityX;
  startD.velocity = vehicle.m_velocityY;
  // 恒定加速度
  // 避免倒车
  float veloChangeX = action->m_velocityChange;
  if ((vehicle.m_velocityX + veloChangeX) * vehicle.m_velocityX < 0.0f) {
    if (vehicle.m_velocityX > 0) {
      veloChangeX = -(vehicle.m_velocityX - 0.01f);
    } else {
      veloChangeX = -(vehicle.m_velocityX + 0.01f);
    }
  }
  startS.acceleration = veloChangeX / cOpt().action_duration;
  // 给定值：delta d、d0、vd0 + 假设恒定加速度 => 纵向加速度
  startD.acceleration = 2 / cOpt().action_duration / cOpt().action_duration *
                        (action->m_lateralChange - vehicle.m_velocityY * cOpt().action_duration);

  return {startS, startD, endS, endD};
}

/**
 * @brief 使用给定时间内的恒定加速度计算位置
 *
 * @param time 计算仓位的时间
 * @param position 时间间隔开始时的位置
 * @param velocity 时间间隔开始时的速度
 * @param acceleration 时间间隔内的加速度
 * @return float 给定时间的位置
 */
float ConstantAcceleration::position(float time, float position, float velocity,
                                     float acceleration) const {
  return 0.5 * time * time * acceleration + time * velocity + position;
};

/**
 * @brief 使用给定时间内的恒定加速度计算速度
 *
 * @param time 应计算速度的时间
 * @param velocity 时间间隔开始时的速度
 * @param acceleration 时间间隔内的加速度
 * @return float 给定时间的速度
 */
float ConstantAcceleration::velocity(float time, float velocity, float acceleration) const {
  return time * acceleration + velocity;
};

/**
 * @brief 根据恒定加速度的约束计算轨迹

 * @param trajectory 生成的轨迹
 * @param startS 与起点纵向方向相关的边界条件
 * @param startD 起始时与横向方向相关的边界条件
 * @param endS 与末端纵向方向相关的约束条件
 * @param endD 与末端横向方向相关的边界条件
 *
 */
Trajectory ConstantAcceleration::calculateTrajectory(const Vehicle& vehicle, const float t0,
                                                     const BoundaryCondition& startS,
                                                     const BoundaryCondition& startD,
                                                     const BoundaryCondition& endS,
                                                     const BoundaryCondition& endD) const {
  Trajectory trajectory(t0, vehicle.m_heading);
  trajectory.m_t0 = t0;
  trajectory.m_t1 = t0 + cOpt().action_duration;

  for (size_t i = 0; i < trajectory.m_nSteps; ++i) {
    const auto t = t0 + i * cOpt().delta_t;

    trajectory.m_time[i]      = t;
    trajectory.m_sPosition[i] = position(t, startS.position, startS.velocity, startS.acceleration);
    trajectory.m_dPosition[i] = position(t, startD.position, startD.velocity, startD.acceleration);
    trajectory.m_sVelocity[i] = velocity(t, startS.velocity, startS.acceleration);
    trajectory.m_dVelocity[i] = velocity(t, startD.velocity, startD.acceleration);
    trajectory.m_sAcceleration[i] = startS.acceleration;
    trajectory.m_dAcceleration[i] = startD.acceleration;
    // 航向角不受影响，因为已经在初始化中设置
  }
  return trajectory;
}

/**
 * @brief 更新轨迹的最终状态以更新车辆的状态
 *
 * @param trajectory 更新轨迹
 */
void ConstantAcceleration::updateFinalState(Trajectory& trajectory) const {
  const auto i               = trajectory.getFractionIndex();
  trajectory.m_finalState[0] = trajectory.m_sPosition[i];
  trajectory.m_finalState[1] = trajectory.m_dPosition[i];
  trajectory.m_finalState[2] = trajectory.m_sVelocity[i];
  trajectory.m_finalState[3] = 0.0f;
  trajectory.m_finalState[4] = 0.0f;
  trajectory.m_finalState[5] = 0.0f;
  trajectory.m_finalState[6] = trajectory.m_lane[i];
  trajectory.m_finalState[7] = 0.0f;
}

/**
 * @brief 根据边界条件计算加速成本
 *
 * @param trajectory 应计算成本的轨迹
 * @param startS 与起点纵向方向相关的边界条件
 * @param startD 起始时与横向方向相关的边界条件
 * @param endS 与末端纵向方向相关的约束条件
 * @param endD 与末端横向方向相关的边界条件
 */
void ConstantAcceleration::calculateCumulativeAcceleration(Trajectory& trajectory,
                                                           const BoundaryCondition& startS,
                                                           const BoundaryCondition& startD,
                                                           const BoundaryCondition& endS,
                                                           const BoundaryCondition& endD) const {
  const auto duration = Trajectory::getCurrentFraction() * trajectory.m_t1;
  // 计算出的恒定加速度的平方值
  trajectory.m_cumSquaredAccelerationLon = startS.acceleration * startS.acceleration * duration;
  trajectory.m_cumSquaredAccelerationLat = startD.acceleration * startD.acceleration * duration;
}
}  // namespace proseco_planning
