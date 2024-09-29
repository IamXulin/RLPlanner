/**
 * @file node.h
 * @brief This file defines the Node class.
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

#include <algorithm>
#include <map>
#include <memory>
#include <tuple>
#include <vector>

#include "nlohmann/json.hpp"
using json = nlohmann::json;
#include "proseco_planning/agent/agent.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/util/alias.h"

namespace proseco_planning {
class CollisionChecker;
class TrajectoryGenerator;

namespace config {
struct Agent;
}  // namespace config
/**
 * @brief Node 类定义搜索树中的节点
 *
 */
class Node {
 public:
  // default constructor
  Node();

  explicit Node(std::vector<Agent> agents);

  explicit Node(const std::vector<config::Agent>& agents);

  Node(const ActionSet& actionSet, Node* const parent);

  explicit Node(const Node* node);

  bool hasChildren() const;

  Node* addChild(const ActionSet& actionSet);

  Node* getChild(const ActionSet& actionSet) const;

  void checkCollision(CollisionChecker& collisionChecker);

  std::tuple<bool, bool> validateInitialization();

  bool checkInitForCollisions();

  void checkValidity();

  /**
   * @brief 检查所有代理是否已使用有效位置进行初始化.
   *
   * @return 如果所有代理起始位置均有效，则为 true.
   */
  inline bool checkValidInit() const {
    return std::all_of(m_agents.begin(), m_agents.end(),
                       [](const Agent& agent) { return agent.m_vehicle.isValid(); });
  }

  void checkTerminality();

  void checkSafeRangeCost();

  void executeActions(const ActionSet& actionSet, CollisionChecker& collisionChecker,
                      const TrajectoryGenerator& trajectoryGenerator, const bool executeFraction);

  // json exports
  json childMapToJSON(const ActionSet& bestActionSet) const;
  json permutationMapToJSON(const ActionSet& bestActionSet) const;
  json moveGroupsToJSON() const;
  static void treeToJSON(const Node* const node, json& jTree);

  json treeNodeToJSON() const;

  // 将数据保存到磁盘
  void exportChildMap(const int step, const ActionSet& bestActionSet) const;
  void exportPermutationMap(const int step, const ActionSet& bestActionSet) const;
  void exportMoveGroups(const int step) const;
  void exportTree(const int step) const;

  // 节点的操作集。
  ActionSet m_actionSet;
  // parent
  Node* m_parent;
  // agents
  std::vector<Agent> m_agents;
  // 访问次数
  unsigned int m_visits;
  // 节点深度
  unsigned int m_depth;
  // 子地图，动作集作为索引
  std::map<ActionSet, std::unique_ptr<Node>> m_childMap;

  // 表示由代理执行导致至少两个代理之间发生碰撞的动作而产生的碰撞状态
  bool m_collision{false};
  // 表示代理执行无效操作导致的无效状态
  bool m_invalid{false};
  // 表示所有智能体达到其期望状态所产生的最终状态
  bool m_terminal{false};

  // 计算无效/冲突节点的概率
  static std::tuple<float, float, float> calculateActionStatistics(
      const std::map<ActionSet, std::unique_ptr<Node>>& childMap, const ActionPtr& action,
      const int agentIdx);
};

void to_json(json& j, const Node& node);
}  // namespace proseco_planning