#include "proseco_planning/policies/finalSelectionPolicy.h"

#include <iostream>
#include <map>

#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/node.h"
#include "proseco_planning/policies/final_selection/finalSelectionKernelRegressionLCB.h"
#include "proseco_planning/policies/final_selection/finalSelectionMaxActionValue.h"
#include "proseco_planning/policies/final_selection/finalSelectionMaxVisitCount.h"
#include "proseco_planning/policies/final_selection/finalSelectionMostTrusted.h"
#include "proseco_planning/policies/final_selection/finalSelectionSampleExpQ.h"

namespace proseco_planning {

/**
 * @brief Creates a policy object based upon the input parameter.
 *
 * @param name String that describes the kind of policy to create.
 * @return std::shared_ptr<FinalSelectionPolicy> Instance of the specified selection policy.
 */
std::shared_ptr<FinalSelectionPolicy> FinalSelectionPolicy::createPolicy(const std::string& name) {
  if (name == "maxActionValue") {
    return std::make_shared<FinalSelectionMaxActionValue>(name);
  } else if (name == "maxVisitCount") {
    return std::make_shared<FinalSelectionMaxVisitCount>(name);
  } else if (name == "mostTrusted") {
    return std::make_shared<FinalSelectionMostTrusted>(name);
  } else if (name == "sampleExpQ") {
    return std::make_shared<FinalSelectionSampleExpQ>(name);
  } else if (name == "kernelRegressionLCB") {
    return std::make_shared<FinalSelectionKernelRegressionLCB>(name);
  } else {
    throw std::invalid_argument("Unknown final selection policy type: " + name);
  }
}

/**
 * @brief 返回最佳计划序列
 *
 * @param nodeFinalSelection 指向最终选择节点（例如根节点）的指针
 * @return 最佳计划序列：包含连续最佳动作集的向量
 */
ActionSetSequence FinalSelectionPolicy::getBestPlan(const Node* nodeFinalSelection) {
  ActionSetSequence bestPlan;
  ActionSet bestActionSet;

  // 提取整个规划
  bool rootParallelizationActive{1 < cOpt().parallelization_options.n_threads};
  while (nodeFinalSelection != nullptr &&
         (!nodeFinalSelection->m_childMap.empty() || rootParallelizationActive)) {
    bestActionSet.clear();
    bestActionSet = getBestActionSet(nodeFinalSelection);

    bestPlan.push_back(bestActionSet);

    if (nodeFinalSelection->m_childMap.count(m_bestActionSet)) {
      nodeFinalSelection = nodeFinalSelection->m_childMap.at(m_bestActionSet).get();
    } else {
      // 如果找不到，nodeFinalSelection 将被赋值为 nullptr，这将终止 while 循环
      nodeFinalSelection = nullptr;
      // std::cout << "node was not explored" << std::endl;
    }
  }
  return bestPlan;
}
}  // namespace proseco_planning
