#include "proseco_planning/search_guide/searchGuideRandom.h"

#include "proseco_planning/action/actionSpace.h"

namespace proseco_planning {
class Vehicle;

/**
 * @brief 返回动作空间内的随机动作，以进行逐步扩大
 *
 * @param actionSpace 代理的行动空间
 * @param vehicle The current state of the vehicle.
 * @param actionUCT The UCT values for all available actions.
 * @return ActionPtr 采取最佳行动逐步拓宽
 */
ActionPtr SearchGuideRandom::getBestActionForPW(const ActionSpace& actionSpace,
                                                const Vehicle& vehicle,
                                                const std::map<ActionPtr, float>& actionUCT) const {
  // 在整个动作空间内进行采样
  return actionSpace.sampleRandomAction(vehicle);
}

/**
 * @brief 返回指定动作类中的随机动作，用于逐步扩展
 *
 * @param actionClass 采样内的动作类
 * @param actionSpace 代理的动作空间
 * @param vehicle 车辆当前状态
 * @param actionUCT 所有可用操作的 UCT 值
 * @return ActionPtr 向最佳动作进行渐进加宽
 */
ActionPtr SearchGuideRandom::getBestActionInActionClassForPW(
    const ActionClass& actionClass, const ActionSpace& actionSpace, const Vehicle& vehicle,
    const std::map<ActionPtr, float>& actionUCT) const {
  // 在指定动作类中采样
  return actionSpace.sampleRandomActionInActionClass(actionClass, vehicle);
}

}  // namespace proseco_planning
