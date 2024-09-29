#include "proseco_planning/policies/final_selection/finalSelectionSampleExpQ.h"

#include <map>
#include <memory>
#include <random>

#include "proseco_planning/action/action.h"
#include "proseco_planning/agent/agent.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/math/mathlib.h"
#include "proseco_planning/node.h"

namespace proseco_planning {

/**
 * @brief Calculates the exponential action value of an action factored by a constant q_scale.
 * @note The action weight can be infinity due to "floating point overflow". Since the resulting
 * distribution would return NaN as a probability, the action weight is set to the maximum limit of
 * float. Furthermore, the action weight can be zero due to "floating point underflow". Since a
 * distribution cannot be zero at any given point, the action weight is set to the minimum limit of
 * float. If the weight is NaN, this functions throws an exception.
 *
 * @param actionValue The action value.
 * @return float The exponential action value.
 */
float FinalSelectionSampleExpQ::calculateActionWeight(const float actionValue) {
  float weight = std::exp(actionValue * cOpt().policy_options.policy_enhancements.q_scale);
  weight       = std::isinf(weight) ? std::numeric_limits<float>::max() : weight;
  weight       = weight == 0 ? std::numeric_limits<float>::min() : weight;
  if (std::isnan(weight)) {
    throw std::runtime_error("The action weight is NaN.");
  } else {
    return weight;
  }
}

/**
 * @brief 实例化动作权重上的 softmax Q 分布，并从中采样动作
 * @note std::discrete_distribution 自动标准化 actionWeights 以获得有效的分布
 *
 * @param weights 动作的权重
 * @return std::tuple<unsigned int, float> 采样动作的索引及其概率
 */
std::tuple<unsigned int, float> FinalSelectionSampleExpQ::sampleActionFromWeights(
    std::vector<float> weights) {
  std::discrete_distribution<unsigned int> distribution(weights.begin(), weights.end());
  unsigned int index = distribution(math::Random::engine());
  float probability  = static_cast<float>(distribution.probabilities()[index]);
  return {index, probability};
}

/**
 * @brief 根据指数操作值的结果策略获取最佳操作集
 *
 * @param node 从中获取最佳操作集的节点
 * @return ActionSet 最佳动作集
 */
ActionSet FinalSelectionSampleExpQ::getBestActionSet(const Node* const node) {
  m_bestActionSet.clear();

  for (const auto& agent : node->m_agents) {
    // 初始化占位符
    std::vector<float> actionWeights;
    ActionSet actions;
    /*
     * 计算用于定义代理操作的分类分布的权重。 选择每个动作的权重是通过对代理的 Q 值应用 softmax 函数来定义的。
     */
    for (const auto& [action, value] : agent.m_actionValues) {
      actionWeights.push_back(calculateActionWeight(value));
      actions.push_back(action);
    }

    auto [index, probability] = sampleActionFromWeights(actionWeights);
    auto bestAction           = actions[index];
    // 将采样概率添加到所选操作中
    bestAction->m_selectionLikelihood = probability;
    // 将非标准化动作选择权重添加到所选动作
    bestAction->m_selectionWeights = actionWeights;
    // 将代理 i 的操作附加到最终操作集中
    m_bestActionSet.push_back(bestAction);
  }

  return m_bestActionSet;
}
}  // namespace proseco_planning
