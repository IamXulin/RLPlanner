#include "proseco_planning/policies/expansion/expansionUCT.h"

#include <memory>

#include "proseco_planning/node.h"
#include "proseco_planning/policies/policy.h"

namespace proseco_planning {

/**
 * @brief 通过添加子节点来扩展搜索树
 *
 * @param node 指向父节点的指针
 * @param actionSet 从父节点到子节点的动作集
 * @param agentsRewards 包含沿树路径的每个步骤的向量，另一个存储每个代理的奖励的向量；通过引用更新此参数
 * @param maxDepth 搜索树的最大深度
 * @return 指向子节点的指针
 */
Node* ExpansionUCT::expandTree(Node* node, ActionSet& actionSet,
                               std::vector<std::vector<float> >& agentsRewards,
                               const unsigned int maxDepth) {
  // 检查节点是否可扩展
  if (!Policy::isNodeTerminal(node, maxDepth)) {
    // 创建子节点
    node = node->addChild(actionSet);
    // 执行用于到达子结点的动作
    node->executeActions(actionSet, *m_collisionChecker, *m_trajectoryGenerator, false);
    // 提取奖励
    extractReward(node, agentsRewards);
  }
  return node;
}
}  // namespace proseco_planning
