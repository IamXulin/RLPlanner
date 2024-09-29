#include "proseco_planning/action/actionSpaceRectangle.h"

#include <memory>
#include <random>
#include <stdexcept>

#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/math/mathlib.h"

namespace proseco_planning {

/**
 * @brief 构造一个新的 ActionSpaceRectangle 对象
 *
 * @param config ActionSpaceRectangle 配置
 */
ActionSpaceRectangle::ActionSpaceRectangle(const config::ActionSpaceRectangle& config)
    : ActionSpace(config::ActionSpaceRectangle::TYPE), m_config(config) {
  math::MinMaxPair<float> velocityChange{-m_config.max_velocity_change,
                                         m_config.max_velocity_change};
  math::MinMaxPair<float> lateralchange{-m_config.max_lateral_change, m_config.max_lateral_change};
  m_boundary = ActionBoundary(velocityChange, lateralchange);

  m_collisionChecker    = CollisionChecker::createCollisionChecker(cOpt().collision_checker);
  m_trajectoryGenerator = TrajectoryGenerator::createTrajectoryGenerator(cOpt().trajectory_type);
}

/**
 * @brief 获取预定义代理的操作.
 * @details 预定义意味着代理不会改变其速度或横向位置，因此操作是 DO_NOTHING.
 * @note 一般先探索动作集中的第一个动作.
 *
 * @return ActionSet 预定义的操作.
 */
ActionSet ActionSpaceRectangle::getPredefinedActions() const {
  return {std::make_shared<Action>(ActionClass::DO_NOTHING)};
}

/**
 * @brief 获取详细的动作集，即总共九个动作，分布在动作空间上.
 *
 * @note 一般先探索动作集中的第一个动作.
 *
 * @param vehicle The vehicle.
 * @return ActionSet 详细动作.
 */
ActionSet ActionSpaceRectangle::getDetailedActions(const Vehicle& vehicle) const {
  const auto distanceToLeftLaneCenter{vehicle.getDistanceToLeftLaneCenter()};
  const auto distanceToRightLaneCenter{vehicle.getDistanceToRightLaneCenter()};

  ActionSet availableActions;
  availableActions.reserve(9);

  // 首先设置稀疏动作空间表示然后扩展
  availableActions = getModerateActions(vehicle);
  // 右转并减速
  availableActions.push_back(
      std::make_shared<Action>(-m_config.max_velocity_change / 2.0f, distanceToRightLaneCenter));
  // 右转并加速
  availableActions.push_back(
      std::make_shared<Action>(m_config.max_velocity_change / 2.0f, distanceToRightLaneCenter));
  // 左转并减速
  availableActions.push_back(
      std::make_shared<Action>(m_config.max_velocity_change / 2.0f, distanceToLeftLaneCenter));
  // 左转并加速
  availableActions.push_back(
      std::make_shared<Action>(-m_config.max_velocity_change / 2.0f, distanceToLeftLaneCenter));
  return availableActions;
}

/**
 * @brief 获取中等动作集，即总共五个动作，在动作空间上间隔开，用于搜索低于最大深度的渐进加宽.
 *
 * @note 一般先探索返回向量的第一个动作.
 *
 * @param vehicle The vehicle.
 * @return ActionSet 适度的动作.
 */
ActionSet ActionSpaceRectangle::getModerateActions(const Vehicle& vehicle) const {
  ActionSet availableActions;
  availableActions.reserve(5);

  // 初始化中等动作 - 仅使用基本动作类
  // do nothing
  availableActions.push_back(std::make_shared<Action>(0.0f, 0.0f));
  // 加速
  availableActions.push_back(std::make_shared<Action>(m_config.max_velocity_change / 2.0f, 0.0f));
  // 减速
  availableActions.push_back(std::make_shared<Action>(-m_config.max_velocity_change / 2.0f, 0.0f));
  // 向左变道
  availableActions.push_back(std::make_shared<Action>(0.0f, vehicle.getDistanceToLeftLaneCenter()));
  // 向右变道
  availableActions.push_back(
      std::make_shared<Action>(0.0f, vehicle.getDistanceToRightLaneCenter()));
  return availableActions;
}

/**
 * @brief 获取给定车辆及其动作空间的动作的动作类.
 * @details 动作空间分为五个区域，对应五个基本动作类别.
 * + : accelerate
 * - : decelerate
 * 0 : Keep constant velocity and lateral position
 * R : Change lateral position to the right
 * L : Change lateral position to the left
 * @note 动作空间的划分没有调整，仅使用轨迹点而不是车辆的整个矩形
 *
 * @param action 分类动作.
 * @param vehicle The vehicle.
 * @return ActionClass 动作的动作类.
 */
ActionClass ActionSpaceRectangle::getActionClass(const Action& action,
                                                 const Vehicle& vehicle) const {
  const auto distanceToLeftLane  = vehicle.getDistanceToLeftLane();
  const auto distanceToRightLane = vehicle.getDistanceToRightLane();

  if (doNothingActionClass(action, distanceToLeftLane, distanceToRightLane))
    return ActionClass::DO_NOTHING;
  else if (changeLeftActionClass(action, distanceToLeftLane)) {
    if (fastActionClass(action))
      return ActionClass::CHANGE_LEFT_FAST;
    else if (slowActionClass(action))
      return ActionClass::CHANGE_LEFT_SLOW;
    else
      return ActionClass::CHANGE_LEFT;
  } else if (changeRightActionClass(action, distanceToRightLane)) {
    if (fastActionClass(action))
      return ActionClass::CHANGE_RIGHT_FAST;
    else if (slowActionClass(action))
      return ActionClass::CHANGE_RIGHT_SLOW;
    else
      return ActionClass::CHANGE_RIGHT;
  } else if (accelerateActionClass(action, distanceToLeftLane, distanceToRightLane)) {
    return ActionClass::ACCELERATE;
  } else if (decelerateActionClass(action, distanceToLeftLane, distanceToRightLane)) {
    return ActionClass::DECELERATE;
  } else {
    throw std::runtime_error("无法确定操作类别.");
  }
}

/**
 * @brief 获取动作类的边界。
 *
 * @param actionClass 获取边界的动作类
 * @param vehicle The vehicle.
 * @return ActionBoundary 动作类的边界
 */
ActionBoundary ActionSpaceRectangle::getActionClassBoundary(const ActionClass actionClass,
                                                            const Vehicle& vehicle) const {
  const auto distanceToLeftLane  = vehicle.getDistanceToLeftLane();
  const auto distanceToRightLane = vehicle.getDistanceToRightLane();

  ActionBoundary boundary;

  switch (actionClass) {
    case ActionClass::DO_NOTHING:
      boundary.velocityChange.min = -m_config.delta_velocity;
      boundary.velocityChange.max = m_config.delta_velocity;
      boundary.lateralChange.min  = distanceToRightLane;
      boundary.lateralChange.max  = distanceToLeftLane;
      break;

    case ActionClass::CHANGE_LEFT:
      boundary.velocityChange.min = -m_config.delta_velocity;
      boundary.velocityChange.max = m_config.delta_velocity;
      boundary.lateralChange.min  = distanceToLeftLane;
      boundary.lateralChange.max  = m_config.max_lateral_change;
      break;

    case ActionClass::CHANGE_RIGHT:
      boundary.velocityChange.min = -m_config.delta_velocity;
      boundary.velocityChange.max = m_config.delta_velocity;
      boundary.lateralChange.min  = -m_config.max_lateral_change;
      boundary.lateralChange.max  = distanceToRightLane;
      break;

    case ActionClass::ACCELERATE:
      boundary.velocityChange.min = m_config.delta_velocity;
      boundary.velocityChange.max = m_config.max_velocity_change;
      boundary.lateralChange.min  = distanceToRightLane;
      boundary.lateralChange.max  = distanceToLeftLane;
      break;

    case ActionClass::DECELERATE:
      boundary.velocityChange.min = -m_config.max_velocity_change;
      boundary.velocityChange.max = -m_config.delta_velocity;
      boundary.lateralChange.min  = distanceToRightLane;
      boundary.lateralChange.max  = distanceToLeftLane;
      break;

    case ActionClass::CHANGE_LEFT_FAST:
      boundary.velocityChange.min = m_config.delta_velocity;
      boundary.velocityChange.max = m_config.max_velocity_change;
      boundary.lateralChange.min  = distanceToLeftLane;
      boundary.lateralChange.max  = m_config.max_lateral_change;
      break;

    case ActionClass::CHANGE_LEFT_SLOW:
      boundary.velocityChange.min = -m_config.max_velocity_change;
      boundary.velocityChange.max = -m_config.delta_velocity;
      boundary.lateralChange.min  = distanceToLeftLane;
      boundary.lateralChange.max  = m_config.max_lateral_change;
      break;

    case ActionClass::CHANGE_RIGHT_FAST:
      boundary.velocityChange.min = m_config.delta_velocity;
      boundary.velocityChange.max = m_config.max_velocity_change;
      boundary.lateralChange.min  = -m_config.max_lateral_change;
      boundary.lateralChange.max  = distanceToRightLane;
      break;

    case ActionClass::CHANGE_RIGHT_SLOW:
      boundary.velocityChange.min = -m_config.max_velocity_change;
      boundary.velocityChange.max = -m_config.delta_velocity;
      boundary.lateralChange.min  = -m_config.max_lateral_change;
      boundary.lateralChange.max  = distanceToRightLane;
      break;

    case ActionClass::NONE:
      throw std::invalid_argument("Action class must be initialized.");
  }

  return boundary;
}

/**
 * @brief Gets the boundaries of the entire action space.
 *
 * @return ActionBoundary The boundaries of the action space.
 */

ActionBoundary ActionSpaceRectangle::getActionSpaceBoundary() const {
  return {{-m_config.max_velocity_change, m_config.max_velocity_change},
          {-m_config.max_lateral_change, m_config.max_lateral_change}};
}

/**
 * @brief 在动作边界内对随机动作进行采样
 *
 * @param boundary 采样的操作边界
 * @return ActionPtr 采样动作
 */
ActionPtr ActionSpaceRectangle::sampleRandomActionInBoundary(const ActionBoundary& boundary) {
  auto velocityChange =
      math::getRandomNumberInInterval(boundary.velocityChange.min, boundary.velocityChange.max);
  auto lateralChange =
      math::getRandomNumberInInterval(boundary.lateralChange.min, boundary.lateralChange.max);
  return std::make_shared<Action>(velocityChange, lateralChange);
}

/**
 * @brief 在动作空间的动作边界内采样一个随机动作
 *
 * @param vehicle The vehicle.
 * @return ActionPtr The sampled action.
 */
ActionPtr ActionSpaceRectangle::sampleRandomAction(const Vehicle& vehicle) const {
  return sampleValidAction(
      vehicle, std::bind(&ActionSpaceRectangle::sampleRandomActionInBoundary, m_boundary),
      *m_collisionChecker, *m_trajectoryGenerator);
}

/**
 * @brief 在指定的动作类别中抽取随机动作
 *
 * @param actionClass The action class.
 * @param vehicle The vehicle.
 * @return ActionPtr The sampled action.
 */
ActionPtr ActionSpaceRectangle::sampleRandomActionInActionClass(const ActionClass& actionClass,
                                                                const Vehicle& vehicle) const {
  auto classBoundary = getActionClassBoundary(actionClass, vehicle);
  return sampleValidAction(
      vehicle, std::bind(&ActionSpaceRectangle::sampleRandomActionInBoundary, classBoundary),
      *m_collisionChecker, *m_trajectoryGenerator);
};

}  // namespace proseco_planning
