#include "proseco_planning/policies/policy.h"

#include <cstddef>
#include <map>
#include <memory>
#include <random>

#include "proseco_planning/agent/agent.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/math/mathlib.h"
#include "proseco_planning/node.h"

namespace proseco_planning {

Policy::Policy(const std::string& name) : m_name(name) {}

/**
 * @brief 从“节点”中提取每个代理的合作奖励，并将值存储在“agentRewards”中
 * @param node 指向要从中提取奖励的节点的指针
 * @param agentsRewards 包含沿着树路径的每一步的向量，以及存储每个代理的奖励的另一个向量
 */
void Policy::extractReward(const Node* const node,
                           std::vector<std::vector<float> >& agentsRewards) {
  for (size_t i = 0; i < node->m_agents.size(); ++i) {
    agentsRewards[node->m_depth - 1][i] = node->m_agents[i].m_coopReward;
  }
}

/**
 * @brief 返回随机动作的共享动作指针
 *
 * @param agent 需要返回动作的代理
 * @return ActionPtr 可用操作的操作指针
 */
// 从所有可用操作中获取操作
ActionPtr Policy::getRandomAction(const Agent& agent) {
  /*
   * 检查是否已访问过不执行任何操作（始终是 available_actions 中的第一个操作）
   * 如果还没有被访问过，则先扩展它
   */
  if (agent.m_actionVisits.at(agent.m_availableActions[0]) <
      config::ComputeOptions::error_tolerance) {
    return agent.m_availableActions[0];
  }
  return math::getRandomElementFromVector(agent.m_availableActions);
}

/**
 * @brief 检查节点是否为终端节点，例如，表示不能有任何子节点的叶节点
 * @param node 需要检查的节点
 * @param maxDepth 树的最大允许深度
 * @return true 如果节点是终端
 * @return false 否则
 */
bool Policy::isNodeTerminal(const Node* const node, unsigned int maxDepth) {
  return node->m_invalid || node->m_collision || node->m_depth >= maxDepth;
}

/**
 * @brief 根据节点代理的可用操作设置随机操作集
 *
 * @param node 指向动作集的指针
 */
void Policy::getRandomActionSet(Node* const node) {
  node->m_actionSet.clear();
  for (const auto& agent : node->m_agents) {
    // random selection
    node->m_actionSet.push_back(getRandomAction(agent));
  }
}

}  // namespace proseco_planning
