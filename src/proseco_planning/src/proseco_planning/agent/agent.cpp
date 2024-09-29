#include "proseco_planning/agent/agent.h"

#include <cassert>
#include <numeric>
#include <string>
#include <type_traits>
#include <utility>

#include <iostream>
#include <proseco_planning/matplotlibcpp.h>
#include "nlohmann/json.hpp"
#include "proseco_planning/action/action.h"
#include "proseco_planning/action/actionSpace.h"
#include "proseco_planning/agent/cost_model/costModel.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/scenarioOptions.h"
#include "proseco_planning/math/mathlib.h"
#include "proseco_planning/search_guide/searchGuide.h"
#include "proseco_planning/trajectory/trajectorygenerator.h"
#include "proseco_planning/util/json.h"

namespace proseco_planning {
/**
 * @brief Constructs a new Agent object using an agent config.
 *
 * @param agent The agent config.
 */
Agent::Agent(const config::Agent& agent)
    : m_desire(agent.desire),
      m_vehicle(agent.vehicle),
      m_cooperationFactor(agent.cooperation_factor),
      m_egoReward(0.0f),
      m_coopReward(0.0f),
      m_actionValue(0.0f),
      m_trajectory(Trajectory(0.0f, agent.vehicle.heading)),
      m_id(agent.id),
      m_currentPotential(0) {
  m_actionSpace = ActionSpace::createActionSpace(agent.action_space);
  m_costModel   = CostModel::createCostModel(agent.cost_model);

  // 创建搜索指南
  m_searchGuide =
      SearchGuide::createSearchGuide(cOpt().policy_options.policy_enhancements.search_guide.type);

  // 初始化最终的潜在值，这个值以后不会改变，因为这个构造函数只在主函数中调用
  m_costModel->initializeMaximumStatePotential(m_desire, m_vehicle);
}

/**
 * @brief 清除所有操作映射（即操作和操作类映射）
 */
void Agent::clearActionMaps() {
  // 重置行动地图
  m_actionVisits.clear();
  m_actionValues.clear();
  m_actionUCT.clear();
  // 重置动作类映射
  m_actionClassUCT.clear();
  m_actionClassValues.clear();
  m_actionClassVisits.clear();
  m_actionClassCount.clear();
}

/**
 * @brief 设置当前情况下所有可用的操作
 * @note 该函数在action_execution之后执行
 * @note 目前采取所有行动
 *
 * @param depth 节点深度
 */
void Agent::setAvailableActions(unsigned int depth) {
  // 清除所有行动地图
  clearActionMaps();

  // 预定义代理仅具有非常有限的一组操作
  // 例如，如果使用 ActionSpaceRectangle，则仅“do_nothing”
  if (m_isPredefined) {
    m_availableActions = m_actionSpace->getPredefinedActions();
  }
  // 如果达到一定的树深度，则代理仅剩下基本的机动操作（中等动作），
  //例如，+，-，0，L（到车道中心），R（到车道中心），如果 ActionSpaceRectangle 用来
  else if (depth < cOpt().policy_options.policy_enhancements.progressive_widening.max_depth_pw) {
    // 代理可以执行所有详细操作
    m_availableActions = m_actionSpace->getDetailedActions(m_vehicle);
  } else {
    // 代理只能采取适度的行动，以减少行动空间探索
    m_availableActions = m_actionSpace->getModerateActions(m_vehicle);
  }

  updateActionClasses();

  addActionsToMaps(m_availableActions);
}

/**
 * @brief 将操作添加到可用操作集中
 * @note 这用于渐进加宽
 *
 * @param action 要添加的操作
 */
void Agent::addAvailableAction(ActionPtr action) {
  action->updateActionClass(*m_actionSpace, m_vehicle);

  m_availableActions.push_back(action);

  // 更新地图
  addActionToMaps(action);
}

/**
 * @brief 将操作集添加到所有操作映射中
 *
 * @param actions 要添加的操作
 */
void Agent::addActionsToMaps(const ActionSet& actions) {
  for (const auto& action : actions) {
    addActionToMaps(action);
  }
}

/**
 * @brief 将动作添加到所有动作图中
 *
 * @param action 要添加的操作
 */
void Agent::addActionToMaps(const ActionPtr& action) {
  // 使用默认值将新操作添加到地图
  m_actionVisits.insert(std::make_pair(action, 0.0f));
  m_actionValues.insert(std::make_pair(action, 0.0f));
  m_actionUCT.insert(std::make_pair(action, config::ComputeOptions::initial_uct));

  m_actionClassVisits.insert(std::make_pair(action->m_actionClass, 0.0f));
  m_actionClassValues.insert(std::make_pair(action->m_actionClass, 0.0f));
  m_actionClassUCT.insert(
      std::make_pair(action->m_actionClass, config::ComputeOptions::initial_uct));
  auto [_, inserted] = m_actionClassCount.insert(std::make_pair(action->m_actionClass, 1));
  if (!inserted) {
    // !inserted, 因为动作类已经存在于映射中，因此，将动作类计数加一
    ++m_actionClassCount.at(action->m_actionClass);
  }
}

/**
 * @brief 根据操作的状态变化更新操作的操作类
 */
void Agent::updateActionClasses() {
  for (const auto& action : m_availableActions) {
    action->updateActionClass(*m_actionSpace, m_vehicle);
  }
}

/**
 * @brief 设置代理动作空间的动作
 * @param action 用于使用代理成本模型进行轨迹计算和评估的动作
 * @param trajectoryGenerator 轨迹生成器用于创建轨迹
 */
void Agent::setAction(ActionPtr action, const TrajectoryGenerator& trajectoryGenerator) {
  /// 0. 重置奖励值
  m_actionCost    = 0.0f;
  m_stateReward   = 0.0f;
  m_egoReward     = 0.0f;
  m_coopReward    = 0.0f;
  m_safeRangeCost = 0.0f;
  // 1. 根据选择的动作计算轨迹
  // t0: 对于模拟很重要或仅对于导出
  std::vector<double> ss;
  std::vector<double> dd;
  m_trajectory = trajectoryGenerator.createTrajectory(0.0f, action, m_vehicle);
  for (int i=0; i<m_trajectory.m_dPosition.size();i++){
    //std::cout<<"s="<<m_trajectory.m_sPosition[i]<<"  d="<<m_trajectory.m_dPosition[i]<<std::endl;
    ss.push_back(m_trajectory.m_sPosition[i]);
    dd.push_back(m_trajectory.m_dPosition[i]);
  }

  // 2. 使用代理的成本模型评估所选操作
  m_actionCost = m_costModel->calculateActionCost(m_trajectory);
}

void Agent::calculateCosts(const Vehicle& vehiclePreviousStep, const float beforePotential) {
  if ("costExponential" == m_costModel->m_type) {
    m_stateReward = m_costModel->updateStatePotential(m_desire, m_vehicle);
    m_egoReward   = m_stateReward + m_actionCost;

    // 碰撞/无效成本更新
    if (m_collision) {
      costCollision();
    }
    if (m_invalid) {
      costInvalidState();
    }
  } else if ("costLinear" == m_costModel->m_type) {
    m_stateReward = m_costModel->calculateStateCost(m_desire, m_vehicle, m_collision, m_invalid);
    m_egoReward   = m_stateReward + m_actionCost;
  } else if ("costNonLinear" == m_costModel->m_type ||
             "costLinearCooperative" == m_costModel->m_type) {
    m_egoReward = m_costModel->calculateCost(m_desire, m_vehicle, vehiclePreviousStep, m_collision,
                                             m_invalid, m_trajectory);
  } else {
    // 更新当前潜力
    m_currentPotential = m_costModel->updateStatePotential(m_desire, m_vehicle);

    // 更新奖励（=状态的改善）
    m_stateReward = m_costModel->updateStateReward(m_currentPotential, beforePotential);

    m_egoReward = m_stateReward + m_actionCost + m_safeRangeCost;
    // 碰撞/无效成本更新
    if (m_collision) {
      costCollision();
    }
    if (m_invalid) {
      costInvalidState();
    }
  }
}

/**
 * @brief 更新碰撞成本
 */
void Agent::costCollision() { m_egoReward += m_costModel->m_costCollision; }

/**
 * @brief 更新终端操作的奖励
 */
void Agent::rewardTerminal() { m_egoReward += m_costModel->m_rewardTerminal; }

/**
 * @brief 更新无效操作的成本
 */
void Agent::costInvalidState() { m_egoReward += m_costModel->m_costInvalidState; }

/**
 * @brief 使用轨迹生成器在一个步骤的持续时间内模拟代理
 * @details TrajectoryGenerator 构造函数中指定的方法
 */
void Agent::simulate() {
  // 保存旧状态的潜力以更新当前状态奖励
  float beforePotential{m_currentPotential};

  Vehicle vehiclePreviousStep(m_vehicle);

  // 使用动作和车辆模型模拟车辆
  // 使用轨迹生成器的方法
  m_vehicle.updateState(m_trajectory.m_finalState);

  calculateCosts(vehiclePreviousStep, beforePotential);
}

/**
 * @brief 指示代理是否已达到其终止状态
 *
 * @return True, 如果所有愿望都满足，否则为false
 */
bool Agent::desiresFulfilled() const { return m_desire.desiresFulfilled(m_vehicle); }

/**
 * @brief 计算所有操作的总操作访问次数
 *
 * @return float 所有操作的总操作访问次数
 */
float Agent::cumulativeActionVisits() const {
  return std::accumulate(
      m_actionVisits.begin(), m_actionVisits.end(), 0.0f,
      [](float sum, const std::pair<ActionPtr, float>& action) { return (sum + action.second); });
}

/**
 * @brief 计算所有操作类别的总操作访问次数
 *
 * @return float 所有操作类别的操作访问总数
 */
float Agent::cumulativeActionClassVisits() const {
  return std::accumulate(m_actionClassVisits.begin(), m_actionClassVisits.end(), 0.0f,
                         [](float sum, const std::pair<ActionClass, float>& actionClass) {
                           return (sum + actionClass.second);
                         });
}

/**
 * @brief 返回任何操作中访问次数最多的操作
 *
 * @return ActionPtr 最大访问次数的动作
 */
ActionPtr Agent::maxActionVisitsAction() const { return math::max_map_element(m_actionVisits); }

/**
 * @brief 返回具有任何操作类的最大访问次数的操作类
 *
 * @return ActionClass 访问次数最多的动作类
 */
ActionClass Agent::maxActionVisitsActionClass() const {
  return math::max_map_element(m_actionClassVisits);
}

/**
 * @brief 返回任何操作的最大操作值
 *
 * @return float 最大动作值
 */
float Agent::maxActionValue() const { return math::max_map_value(m_actionValues); }

/**
 * @brief 返回任何操作的最小操作值
 *
 * @return float 最小动作值
 */
float Agent::minActionValue() const { return math::min_map_value(m_actionValues); }

/**
 * @brief 返回任意动作中动作值最大的动作
 *
 * @return ActionPtr 最大动作值action
 */
ActionPtr Agent::maxActionValueAction() const { return math::max_map_element(m_actionValues); }

/**
 * @brief 返回任何动作类的最大动作值
 *
 * @return float 最大动作值
 */
float Agent::maxActionClassActionValue() const { return math::max_map_value(m_actionClassValues); }

/**
 * @brief 返回任何动作类的最小动作值
 *
 * @return float 最小动作值.
 */
float Agent::minActionClassActionValue() const { return math::min_map_value(m_actionClassValues); }

/**
 * @brief 返回具有任何操作类中最大操作值的操作类
 *
 * @return ActionClass 最大动作值动作类别
 */
ActionClass Agent::maxActionValueActionClass() const {
  return math::max_map_element(m_actionClassValues);
}

/**
 * @brief 返回任何操作的最大 UCT 值
 *
 * @return float 最大 UCT 值
 */
float Agent::maxActionUCT() const { return math::max_map_value(m_actionUCT); }

/**
 * @brief 返回任何操作的最小 UCT 值
 *
 * @return float 最小 UCT 值
 */
float Agent::minActionUCT() const { return math::min_map_value(m_actionUCT); }

/**
 * @brief 返回具有任何操作的最大 UCT 值的操作
 *
 * @return ActionPtr 最大 UCT 值操作
 */
ActionPtr Agent::maxActionUCTAction() const { return math::max_map_element(m_actionUCT); }

/**
 * @brief 返回具有任何操作类的最大 UCT 值的操作类
 *
 * @return ActionClass 最大 UCT 值操作类
 */
ActionClass Agent::maxActionUCTActionClass() const {
  assert(m_actionClassUCT.count(ActionClass::NONE) == 0 &&
         "Action classes must be initialized before usage.");
  return math::max_map_element(m_actionClassUCT);
}

/**
 * @brief 将特定轨迹步骤的信息导出为 JSON
 * @return json 特定轨迹步骤的信息
 */
json Agent::trajectoryStepToJSON(const size_t index) const {
  json jStep;
  jStep["ego_reward"]         = m_egoReward;
  jStep["coop_reward"]        = m_coopReward;
  jStep["position_x"]         = m_trajectory.m_sPosition[index];
  jStep["position_y"]         = m_trajectory.m_dPosition[index];
  jStep["velocity_x"]         = m_trajectory.m_sVelocity[index];
  jStep["velocity_y"]         = m_trajectory.m_dVelocity[index];
  jStep["acceleration_x"]     = m_trajectory.m_sAcceleration[index];
  jStep["acceleration_y"]     = m_trajectory.m_dAcceleration[index];
  jStep["total_velocity"]     = m_trajectory.m_totalVelocity[index];
  jStep["total_acceleration"] = m_trajectory.m_totalAcceleration[index];
  jStep["lane"]               = m_trajectory.m_lane[index];
  jStep["heading"]            = m_trajectory.m_heading[index];
  return jStep;
}

/**
 * @brief 允许将 Agent 转换为 JSON 对象的函数
 * @details 由 nlohmann json 库的 json 构造函数调用
 *
 * @param j 需要填充的JSON对象
 * @param agent 待转换的代理
 */
void to_json(json& j, const Agent& agent) {
  j["m_actionSpace"]       = config::ActionSpace::toJSON(sOpt().agents[agent.m_id].action_space);
  j["m_searchGuide"]       = cOpt().policy_options.policy_enhancements.search_guide.toJSON();
  j["m_costModel"]         = sOpt().agents[agent.m_id].cost_model.toJSON();
  j["vehicle"]             = agent.m_vehicle;
  j["m_actionVisits"]      = agent.m_actionVisits;
  j["m_actionValues"]      = agent.m_actionValues;
  j["m_actionUCT"]         = agent.m_actionUCT;
  j["m_actionClassVisits"] = agent.m_actionClassVisits;
  j["m_actionClassValues"] = agent.m_actionClassValues;
  j["m_actionClassUCT"]    = agent.m_actionClassUCT;
  j["m_actionClassCount"]  = agent.m_actionClassCount;
  j["m_desire"]            = agent.m_desire;
  j["m_cooperationFactor"] = agent.m_cooperationFactor;
  j["m_egoReward"]         = agent.m_egoReward;
  j["m_actionCost"]        = agent.m_actionCost;
  j["m_safeRangeCost"]     = agent.m_safeRangeCost;
  j["m_actionValue"]       = agent.m_actionValue;
  j["m_trajectory"]        = agent.m_trajectory;
  j["m_id"]                = agent.m_id;
  j["is_ego"]              = agent.is_ego;
  j["m_availableActions"]  = agent.m_availableActions;
  j["m_collision"]         = agent.m_collision;
  j["m_invalid"]           = agent.m_invalid;
  j["m_isPredefined"]      = agent.m_isPredefined;
  j["m_currentPotential"]  = agent.m_currentPotential;
  j["m_stateReward"]       = agent.m_stateReward;
  j["m_currentPotential"]  = agent.m_currentPotential;
}

}  // namespace proseco_planning
