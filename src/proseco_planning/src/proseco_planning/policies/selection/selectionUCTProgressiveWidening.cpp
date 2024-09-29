#include "proseco_planning/policies/selection/selectionUCTProgressiveWidening.h"

#include <limits>
#include <map>
#include <memory>
#include <utility>

#include "proseco_planning/action/action.h"
#include "proseco_planning/agent/agent.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/node.h"
#include "proseco_planning/search_guide/searchGuide.h"

namespace proseco_planning {

/**
 * @brief Constructs a new Selection UCT Progressive Widening object.
 *
 * @param name The name of the selection policy.
 */
SelectionUCTProgressiveWidening::SelectionUCTProgressiveWidening(const std::string& name)
    : SelectionPolicy(name) {
  // specification of progressive widening
  m_progressiveWideningCoefficient =
      cOpt().policy_options.policy_enhancements.progressive_widening.coefficient;
  m_progressiveWideningExponent =
      cOpt().policy_options.policy_enhancements.progressive_widening.exponent;
  // initialize PW parameter
  if (cOpt().policy_options.policy_enhancements.move_grouping.active &&
      cOpt().policy_options.policy_enhancements.move_grouping.move_grouping_criteria_pw.active) {
    m_progressiveWideningExponent =
        cOpt()
            .policy_options.policy_enhancements.move_grouping.move_grouping_criteria_pw.exponent_pw;
    m_progressiveWideningCoefficient = cOpt()
                                           .policy_options.policy_enhancements.move_grouping
                                           .move_grouping_criteria_pw.coefficient_pw;
  }
}
/**
 * @brief 测试是否满足逐步扩大的标准（即，操作是访问次数的次线性函数）
 *
 * @param actions 代理当前可以选择的操作数
 * @param coefficient 次线性函数系数
 * @param visits 该状态当前的访问次数
 * @param exponent 次线性函数的指数
 * @return true 如果需要添加新动作, 否则为false .
 */
bool SelectionUCTProgressiveWidening::progressiveWidening(const unsigned int actions,
                                                          const float coefficient,
                                                          const float visits,
                                                          const float exponent) {
  return actions < coefficient * std::pow(visits, exponent);
}

/**
 * @brief 测试移动分组逐步扩展的先决条件是否满足
 *
 * @param node 当前节点
 * @param agent 应检查代理逐步扩大
 * @param agentIdx m_actionClassSet 中的代理的索引
 * @return true 如果需要添加新操作，否则为 false
 */
bool SelectionUCTProgressiveWidening::meetsMoveGroupingPWCriteria(const Node* const node,
                                                                  const Agent& agent,
                                                                  const size_t agentIdx) const {
  return !agent.m_isPredefined &&
         node->m_depth <
             cOpt().policy_options.policy_enhancements.progressive_widening.max_depth_pw &&
         progressiveWidening(agent.m_actionClassCount.at(m_actionClassSet.at(agentIdx)),
                             m_progressiveWideningCoefficient,
                             agent.m_actionClassVisits.at(m_actionClassSet.at(agentIdx)),
                             m_progressiveWideningExponent);
}

/**
 * @brief 测试是否满足逐步加宽的先决条件
 * @todo 这个需要检查，为什么moveGrouping版本是基于agent的，而这个版本是基于node的?
 * @param node The current node.
 * @return true 如果需要添加新动作, false otherwise.
 */
bool SelectionUCTProgressiveWidening::meetsPWCriteria(const Node* const node) const {
  return node->m_depth <
             cOpt().policy_options.policy_enhancements.progressive_widening.max_depth_pw &&
         progressiveWidening(node->m_childMap.size(), m_progressiveWideningCoefficient,
                             node->m_visits, m_progressiveWideningExponent);
}

/**
 * @brief 选择接下来要展开的节点
 *
 * @param node 指向开始选择过程的节点的指针
 * @param actionSet 用于扩展的操作集（通过引用更新）
 * @param agentsRewards 该向量包含沿着树路径的每一步的另一个向量，该向量存储每个代理的奖励； （通过参考更新）
 * @return Node* 指向所选节点的指针
 */
Node* SelectionUCTProgressiveWidening::selectNodeForExpansion(
    Node* node, ActionSet& actionSet, std::vector<std::vector<float> >& agentsRewards) {
  while (!node->m_collision &&  // 发生碰撞 -> 路径无效
         !node->m_invalid &&    // 发生无效状态 -> 无效路径
         getBestNode(node) !=
             nullptr)  // 如果尚未采取最佳操作，则 getBestNode 返回 nullPtr
  {
    // 如果采用渐进式扩大，则继续扩大政策
    if (checkForProgressiveWidening(node)) {
      break;
    }

    // 检查已选择的最佳行动，
    // 尚未选择的会导致“无限”UCT 分数
    // 如果 UCT 分数无限：
    //  - 随机选择其中之一
    //  - 将它们全部展开

    // 采取最佳行动并从树上下来
    node = node->getChild(m_actionSet);
    // 从所采取的行动中收集奖励
    extractReward(node, agentsRewards);
  }
  // 通过参考指定返回的最佳操作
  actionSet = m_actionSet;
  return node;
}

/**
 * @brief 根据 UCT 返回“最佳”子节点。如果此节点不存在，则返回 nullptr。它会相应地设置成员变量“m_actionSet”。
 *        移动分组集成在这里
 *
 * @param node 指向当前下降节点的指针
 * @return 指向现有子节点的指针，如果子节点不存在则为 nullptr
 */
Node* SelectionUCTProgressiveWidening::getBestNode(const Node* const node) {
  if (cOpt().policy_options.policy_enhancements.move_grouping.active)
  // 确定最佳行动组中的最佳行动（基于行动类别）
  {
    // 设置最佳的 actionClass 集
    getBestActionClassUCT(node);
    // 计算每个代理的最佳动作类别中的最佳动作
    getBestActionUCT(node, m_actionClassSet);
  } else {
    // 根据所有行动确定最佳行动
    getBestActionUCT(node);
  }
  // 如果所有代理都根据自己的观点确定 UCT 值，则得分最高的动作不必已经执行 => 首先探索（所有/大量）排列

  // 已经探索行动
  if (node->m_childMap.count(m_actionSet)) {
    return node->m_childMap.at(m_actionSet).get();
  }
  // 未尝试可用动作的排列
  else {
    return nullptr;
  }
}

/**
 * @brief 根据UCT值设置最佳动作集
 *
 * @param node 确定最佳动作集的节点
 */
void SelectionUCTProgressiveWidening::getBestActionUCT(const Node* const node) {
  // 重置先前的操作集
  m_actionSet.clear();
  // 确定每个智能体的最佳行动
  for (const auto& agent : node->m_agents) {
    m_actionSet.push_back(agent.maxActionUCTAction());
  }
}

/**
 * @brief 根据 UCT 值设置最佳动作类集
 *
 * @param node 确定最佳动作类别集的节点
 */
void SelectionUCTProgressiveWidening::getBestActionClassUCT(const Node* const node) {
  // 重置先前的操作集
  m_actionClassSet.clear();
  // 确定每个智能体的最佳动作类别
  for (const auto& agent : node->m_agents) {
    m_actionClassSet.push_back(agent.maxActionUCTActionClass());
  }
}

/**
 * @brief 根据最佳动作类集的UCT值设置最佳动作集
 *
 * @param node 确定最佳动作集的节点
 * @param actionClassSet 确定最佳动作集的动作类别集
 */
void SelectionUCTProgressiveWidening::getBestActionUCT(
    const Node* const node, const std::vector<ActionClass>& actionClassSet) {
  // 重置先前的操作集
  m_actionSet.clear();

  // 确定最佳动作类别中的最佳动作
  ActionPtr bestAction;

  for (size_t i = 0; i < node->m_agents.size(); ++i) {
    // 重置最佳动作和最佳 UCT 分数
    bestAction    = nullptr;
    float bestUCT = std::numeric_limits<float>::lowest();

    for (const auto& [actionPtr, value] : node->m_agents[i].m_actionUCT) {
      if (actionPtr->m_actionClass == m_actionClassSet[i]) {
        if (value > bestUCT) {
          bestUCT    = value;
          bestAction = actionPtr;
        }
      }
    }

    m_actionSet.push_back(bestAction);

    ///@todo 检查一下
    // assert(bestAction == nullptr && "best action is a nullptr");
    // assert(bestAction->m_actionClass != m_actionClassSet[i] &&
    //        "best action is not part of the best action class");
  }
}

// 应用渐进式拓宽标准
// 如果满足渐进加宽的标准，则返回布尔值
// Input:
// 最佳动作集
// Output:
// 指示是否执行 PW
// 修改后的最佳动作集
bool SelectionUCTProgressiveWidening::checkForProgressiveWidening(Node* const node) {
  // 对于移动分组，所选组与检测渐进加宽的必要性相关
  if (cOpt().policy_options.policy_enhancements.move_grouping.active &&
      cOpt().policy_options.policy_enhancements.move_grouping.move_grouping_criteria_pw.active) {
    //是否渐进式加宽
    bool progressiveWidening = false;
    // 代理级别决策 每个代理决定是否应该进行渐进加宽
    // yes：对新动作进行采样并添加到自己的动作空间
    // no: 从 bestActionSet 中获取已确定的 bestAction
    for (size_t agentIdx{}; agentIdx < node->m_agents.size(); ++agentIdx) {
      // 检查是否有必要在组内逐步扩大
      auto& agent = node->m_agents[agentIdx];
      //符合移动分组 PW 标准
      if (meetsMoveGroupingPWCriteria(node, agent, agentIdx)) {
        // 从最佳动作类中获取动作并操作动作集
        m_actionSet[agentIdx] = getGuidedActionForProgressiveWidening(agent, agentIdx);
        progressiveWidening   = true;
      }
    }

    if (progressiveWidening) {
      ///@todo check this
      // 断言(std::any_of(m_actionSet.begin(), m_actionSet.end(),
      //                    [](ActionPtr a) { return a == nullptr; }) &&
      //        "best action is a nullptr");
      //为是否渐进式加宽设置ActionSet
      setActionSetForProgressiveWidening(node);
      return true;
    } else {
      return false;
    }
  }
  // 基于节点统计的渐进加宽
  else {
    // 检查当前节点是否逐渐加宽,
    // 如果是，则选择要采取的操作并打破仅应用于深度 2 的渐进加宽 节点级别决策
    if (meetsPWCriteria(node)) {
      getActionSetForProgressiveWidening(node);
      setActionSetForProgressiveWidening(node);
      return true;
    } else {
      return false;
    }
  }
}

/**
 * @brief 获取每个代理逐步扩大的操作集
 *
 * @param node 获取操作集的节点
 */
void SelectionUCTProgressiveWidening::getActionSetForProgressiveWidening(const Node* const node) {
  for (size_t agentIdx{}; agentIdx < node->m_agents.size(); ++agentIdx) {
    auto& agent = node->m_agents[agentIdx];
    if (agent.m_isPredefined) {
      // 从可用动作中选择随机动作，而无需向动作空间添加新动作
      m_actionSet[agentIdx] = getRandomAction(agent);
    } else {
      m_actionSet[agentIdx] = getGuidedActionForProgressiveWidening(agent, agentIdx);
    }
  }
}

/**
 * @brief 获取逐步扩展的引导动作，即使用配置的搜索指南从代理的动作空间中执行的动作
 *
 * @param agent 代理获取引导操作
 * @param agentIdx 代理人索引
 * @return ActionPtr 引导动作
 */
ActionPtr SelectionUCTProgressiveWidening::getGuidedActionForProgressiveWidening(
    const Agent& agent, const size_t agentIdx) {
  if (cOpt().policy_options.policy_enhancements.move_grouping.active &&
      cOpt().policy_options.policy_enhancements.move_grouping.move_grouping_bias_pw) {
    // 最佳行动类别中的样本
    auto& actionClass = m_actionSet.at(agentIdx)->m_actionClass;
    return agent.m_searchGuide->getBestActionInActionClassForPW(actionClass, *agent.m_actionSpace,
                                                                agent.m_vehicle, agent.m_actionUCT);
  } else {
    // 在动作空间内进行采样
    return agent.m_searchGuide->getBestActionForPW(*agent.m_actionSpace, agent.m_vehicle,
                                                   agent.m_actionUCT);
  }
}

/**
 * @brief 设置逐步扩展的动作集。将新动作添加到代理的动作空间
 *
 * @param node 应应用操作集的节点
 */
void SelectionUCTProgressiveWidening::setActionSetForProgressiveWidening(Node* const node) {
  for (size_t i = 0; i < node->m_agents.size(); ++i) {
    // 如果一个代理这次没有应用渐进式加宽，则无法向地图添加任何其他操作，因为它已经在那里了
    if (!node->m_agents[i].m_actionVisits.contains(m_actionSet.at(i))) {
      // 将生成的动作添加到智能体的可用动作中
      node->m_agents[i].addAvailableAction(m_actionSet.at(i));
    }
  }
}
}  // namespace proseco_planning
