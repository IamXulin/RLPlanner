/**
 * @file selectionUCTProgressiveWidening.h
 * @brief This file defines the SelectionUCTProgressiveWidening class.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <sys/types.h>
#include <cstddef>
#include <string>
#include <vector>

#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/policies/selectionPolicy.h"
#include "proseco_planning/util/alias.h"

namespace proseco_planning {
class Agent;
class Node;

/**
 * @brief The SelectionUCTProgressiveWidening class uses UCT with progressive widening to create a
 * selection policy.
 *
 */
class SelectionUCTProgressiveWidening : public SelectionPolicy {
 public:
  // inherit the constructor from the parent class
  using SelectionPolicy::SelectionPolicy;

  SelectionUCTProgressiveWidening(const std::string& name);

  Node* selectNodeForExpansion(Node* node, ActionSet& actionSet,
                               std::vector<std::vector<float> >& agentsRewards) override;

 private:
  Node* getBestNode(const Node* const node) override;

  void getBestActionUCT(const Node* const node);

  void getBestActionUCT(const Node* const node, const std::vector<ActionClass>& actionClassSet);

  void getBestActionClassUCT(const Node* const node);

  static bool progressiveWidening(const unsigned int actions, const float coefficient,
                                  const float visits, const float exponent);

  bool meetsPWCriteria(const Node* const node) const;

  bool meetsMoveGroupingPWCriteria(const Node* const node, const Agent& agent,
                                   const size_t agentIdx) const;

  bool checkForProgressiveWidening(Node* const node);

  ActionPtr getGuidedActionForProgressiveWidening(const Agent& agent, const size_t agentIdx);

  void getActionSetForProgressiveWidening(const Node* const node);

  void setActionSetForProgressiveWidening(Node* const node);

  /// 最佳动作类别集，存储每个代理的最佳动作类别
  std::vector<ActionClass> m_actionClassSet;

  /// 逐渐加宽的指数。
  float m_progressiveWideningExponent;

  /// 逐渐加宽的系数。
  float m_progressiveWideningCoefficient;
};
}  // namespace proseco_planning
