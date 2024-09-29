/**
 * @file selectionPolicy.h
 * @brief The SelectionPolicy class is the base class for all selection policies. It is used to
 * select the best action from the action set.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "policy.h"
#include "proseco_planning/util/alias.h"

namespace proseco_planning {
class Node;

/**
 * @brief The SelectionPolicy class is the base class for all selection policies. It is used to
 * select the best action from the action set.
 */
class SelectionPolicy : public Policy {
 public:
  /**
   * @brief Construct a new Selection Policy object.
   *
   * @param name The name of the policy.
   */
  explicit SelectionPolicy(const std::string& name) : Policy(name) {}

  /// The virtual destructor
  virtual ~SelectionPolicy() = default;

  static std::unique_ptr<SelectionPolicy> createPolicy(const std::string& name);

  /**
   * @brief 选择接下来要扩展的节点
   *
   * @param node 指向开始选择过程的节点的指针
   * @param actionSet 用于扩展的操作集（通过引用更新）
   * @param agentsRewards 包含沿着树路径的每一步的向量还有另一个存储每个代理的奖励的向量；（通过引用更新）
   * @return Node* 指向选定节点的指针
   */
  virtual Node* selectNodeForExpansion(Node* node, ActionSet& actionSet,
                                       std::vector<std::vector<float> >& agentsRewards) = 0;

 protected:
  /**
   * @brief 确定一个“最佳”动作集，相应地更新成员“m_actionSet”，如果此节点已存在于搜索树中，则下降到相应的“最佳”子节点
   *
   * @param node 指向当前下降节点的指针
   * @return 指向现有子节点的指针，如果子节点不存在则为 nullptr
   */
  virtual Node* getBestNode(const Node* const node) = 0;

  /// 计算动作集
  ActionSet m_actionSet;
};
}  // namespace proseco_planning