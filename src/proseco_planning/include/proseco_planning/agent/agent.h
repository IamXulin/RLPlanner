/**
 * @file agent.h
 * @brief This file defines the Agent class.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <map>
#include <memory>
#include <vector>

#include "nlohmann/json.hpp"
using json = nlohmann::json;
#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/agent/desire.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/trajectory/trajectory.h"
#include "proseco_planning/util/alias.h"

namespace proseco_planning {
class ActionSpace;
class CostModel;
class SearchGuide;
class TrajectoryGenerator;
namespace config {
struct Agent;
}  // namespace config

/**
 * @brief Agent class
 */
class Agent {
 public:
  explicit Agent(const config::Agent& agent);

  void addAvailableAction(ActionPtr action);

  void setAvailableActions(unsigned int depth);

  void clearActionMaps();

  void addActionToMaps(const ActionPtr& action);

  void addActionsToMaps(const ActionSet& actions);

  void updateActionClasses();

  void setAction(ActionPtr action, const TrajectoryGenerator& trajectoryGenerator);

  void simulate();

  void calculateCosts(const Vehicle& vehiclePreviousStep, const float beforePotential);

  void costCollision();

  void costInvalidState();

  void rewardTerminal();

  bool desiresFulfilled() const;

  float cumulativeActionVisits() const;

  float cumulativeActionClassVisits() const;

  json trajectoryStepToJSON(const size_t index) const;

  ActionPtr maxActionVisitsAction() const;

  ActionClass maxActionVisitsActionClass() const;

  float maxActionValue() const;

  float minActionValue() const;

  float maxActionClassActionValue() const;

  float minActionClassActionValue() const;

  ActionPtr maxActionValueAction() const;

  ActionClass maxActionValueActionClass() const;

  float maxActionUCT() const;

  float minActionUCT() const;

  ActionPtr maxActionUCTAction() const;

  ActionClass maxActionUCTActionClass() const;

  /// 行动空间
  std::shared_ptr<ActionSpace> m_actionSpace;

  /// 每个动作的访问次数
  std::map<ActionPtr, float> m_actionVisits;

  /// 每个动作的价值
  std::map<ActionPtr, float> m_actionValues;

  /// 每个动作的UCT值
  std::map<ActionPtr, float> m_actionUCT;

  /// 每个动作类的总访问量
  std::map<ActionClass, float> m_actionClassVisits;

  /// 每个动作类别内的平均动作值
  std::map<ActionClass, float> m_actionClassValues;

  /// 每个动作类别内的平均 UCT 分数
  std::map<ActionClass, float> m_actionClassUCT;

  /// 每个动作类别中的动作数量
  std::map<ActionClass, int> m_actionClassCount;

  /// 期望的代理状态
  Desire m_desire;

  /// 代理控制的车辆
  Vehicle m_vehicle;

  /// 代理的合作因素
  const float m_cooperationFactor;

  /// 到目前为止 ego 车辆的成本
  float m_egoReward{0.0f};

  /// 代理的合作成本
  float m_coopReward{0.0f};

  /// 动作成本：acc，变道，累计，用于计算egoReward
  float m_actionCost{0.0f};

  /// 进入安全范围的成本
  float m_safeRangeCost{0.0f};

  /// action值：该action的期望返回值，实际上是actionSet的值
  float m_actionValue{0.0f};

  /// 将 Action 对象转换为轨迹的对象
  Trajectory m_trajectory;

  /// 指导搜索的模型
  std::shared_ptr<SearchGuide> m_searchGuide{nullptr};

  /// 代理id
  unsigned int m_id;

  /// 用于确定该代理的数据是否应包含在导出文件中
  bool is_ego{true};

  /// 可用的操作-执行
  ActionSet m_availableActions;

  /// 代表执行导致碰撞的动作的代理
  bool m_collision{false};

  /// 代表代理执行无效操作
  bool m_invalid{false};

  /// 如果标记为不会发生渐进加宽
  bool m_isPredefined{false};

  /// 代理成本模型
  std::shared_ptr<CostModel> m_costModel;

  /// 最终状态的潜力，用于状态奖励的计算
  float m_finalPotential{0.0f};

  /// 当前状态的潜力
  float m_currentPotential{0.0f};

  /// 状态成本：基于潜力的奖励塑造
  float m_stateReward{0.0f};
};
void to_json(json& j, const Agent& agent);
}  // namespace proseco_planning
