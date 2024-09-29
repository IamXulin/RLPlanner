#include "proseco_planning/action/action.h"

#include <map>
#include <string>

#include "nlohmann/json.hpp"
#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/action/actionSpace.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"

namespace proseco_planning {

/**
 * @brief 使用操作类构造一个新的操作对象
 * @details 委托给具有默认值的标准构造函数（默认构造函数）
 * @param actionClass 动作的动作类别
 */
Action::Action(ActionClass actionClass)
    : m_actionClass(actionClass),
      m_velocityChange(0),
      m_lateralChange(0),
      m_accelerationX(0),
      m_accelerationY(0) {}

/**
 * @brief 使用加速构造一个新的 Action 对象
 * @details 假设横向加速度恒定且初始速度为零，则将输入转换为相应的纵向速度和横向位置变化。
 * @param actionClass 动作的动作类别
 * @param accelerationX 纵向加速度
 * @param accelerationY 横向加速度
 */
Action::Action(ActionClass actionClass, float accelerationX, float accelerationY)
    : m_actionClass(actionClass),
      m_velocityChange(accelerationX * cOpt().action_duration),
      m_lateralChange(accelerationY * 0.5f * cOpt().action_duration * cOpt().action_duration),
      m_accelerationX(accelerationX),
      m_accelerationY(accelerationY) {}

/**
 * @brief 使用纵向速度变化和横向位置变化构造一个新的 Action 对象
 * @details 假设横向加速度恒定且初始速度为零，将输入转换为纵向和横向的相应加速度
 * @param velocityChange 速度变化
 * @param lateralChange 改变横向位置
 */
Action::Action(float velocityChange, float lateralChange)
    : m_actionClass(ActionClass::NONE),
      m_velocityChange(velocityChange),
      m_lateralChange(lateralChange),
      m_accelerationX(m_velocityChange / cOpt().action_duration),
      m_accelerationY((2 / (cOpt().action_duration * cOpt().action_duration)) * m_lateralChange) {}

/**
 * @brief 根据动作空间和车辆当前状态更新`m_actionClass`
 *
 * @param actionSpace 动作空间.
 * @param vehicle 车辆当前状态.
 */
void Action::updateActionClass(const ActionSpace& actionSpace, const Vehicle& vehicle) {
  m_actionClass = actionSpace.getActionClass(*this, vehicle);
}

/**
 * @brief 使用径向基函数核计算动作 x 和动作 y 的相似度
 * @param x 第一个动作
 * @param y 第二个动作
 * @param gamma 相似度函数的 gamma
 * @return float 两个动作的相似度
 */
float Action::getSimilarity(const ActionPtr& x, const ActionPtr& y, const float gamma) {
  return std::exp(-gamma * (x->getSquaredDistance(y)));
}

/**
 * @brief 使用径向基函数内核计算动作 x 和动作 y 的相似度并从配置中检索 gamma
 * @note 超载
 * @param x 第一个动作
 * @param y 第二个动作
 * @return float 两个动作的相似度
 */
float Action::getSimilarity(const ActionPtr& x, const ActionPtr& y) {
  return getSimilarity(x, y, cOpt().parallelization_options.similarity_gamma);
}

/**
 * @brief 计算到动作的欧氏距离平方，如果没有给出，则默认为动作空间的原点
 * @details 假设原点位于 [0,0]
 *
 * @param action 计算距离的动作
 * @return float 两个动作的欧氏距离平方
 */
float Action::getSquaredDistance(const ActionPtr& action) const {
  if (action == nullptr) {
    return m_lateralChange * m_lateralChange + m_velocityChange * m_velocityChange;
  } else {
    auto distLateral  = m_lateralChange - action->m_lateralChange;
    auto distVelocity = m_velocityChange - action->m_velocityChange;
    return distLateral * distLateral + distVelocity * distVelocity;
  }
}

/**
 * @brief 计算到动作的欧氏距离，如果没有给出，则默认为动作空间的原点
 * @details 假设原点位于 [0,0]
 * @param action 计算距离的动作
 * @return float 两个动作之间的欧氏距离
 */
float Action::getDistance(const ActionPtr& action) const {
  return std::sqrt(getSquaredDistance(action));
}

/**
 * @brief 允许将 Action 转换为 JSON 对象的函数
 * @details 由 nlohmann json 库的 json 构造函数调用
 *
 * @param j 需要填充的JSON对象
 * @param action 要转换的操作
 */
void to_json(json& j, const Action& action) {
  j["class"]           = ActionSpace::ACTION_CLASS_NAME_MAP.at(action.m_actionClass);
  j["acceleration_x"]  = action.m_accelerationX;
  j["acceleration_y"]  = action.m_accelerationY;
  j["velocity_change"] = action.m_velocityChange;
  j["lateral_change"]  = action.m_lateralChange;
}
}  // namespace proseco_planning
