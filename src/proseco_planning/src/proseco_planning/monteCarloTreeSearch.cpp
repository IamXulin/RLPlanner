#include "proseco_planning/monteCarloTreeSearch.h"

#include <sys/types.h>
#include <chrono>
#include <climits>
#include <cstddef>
#include <fstream>
#include <future>
#include <map>
#include <string>
#include <utility>

#include "proseco_planning/action/action.h"
#include "proseco_planning/agent/agent.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/outputOptions.h"
#include "proseco_planning/math/mathlib.h"
#include "proseco_planning/node.h"
#include "proseco_planning/policies/expansionPolicy.h"
#include "proseco_planning/policies/finalSelectionPolicy.h"
#include "proseco_planning/policies/selectionPolicy.h"
#include "proseco_planning/policies/simulationPolicy.h"
#include "proseco_planning/policies/updatePolicy.h"

namespace proseco_planning {

//#########################################################
//### BUILD THE SEARCH TREE
//#########################################################

/**
 * @brief 根据MCTS方法构建搜索树
 *
 * @param root 指向根节点的指针
 * @return std::unique_ptr<Node> 指向最终搜索树的根节点的指针
 */
std::unique_ptr<Node> computeTree(std::unique_ptr<Node> root) {
  //### 根据计算选项创建策略
  auto selectionPolicy  = SelectionPolicy::createPolicy(cOpt().policy_options.selection_policy);
  auto simulationPolicy = SimulationPolicy::createPolicy(cOpt().policy_options.simulation_Policy,
                                                         root->m_agents.size());
  auto expansionPolicy  = ExpansionPolicy::createPolicy(cOpt().policy_options.expansion_policy);
  auto updatePolicy     = UpdatePolicy::createPolicy(cOpt().policy_options.update_policy);

  // 初始化 rootNode 的可用操作
  for (auto& agent : root->m_agents) {
    agent.setAvailableActions(root->m_depth);
  }

  // 一个计划步骤的最长持续时间
  // 注意：由于整数溢出，cOpt().max_step_duration 不得超过 4294 秒（~= 72 分钟）
  unsigned int maxStepDuration{cOpt().max_step_duration == 0
                                   ? UINT_MAX
                                   : static_cast<unsigned int>(cOpt().max_step_duration * 1000000)};
  // 测量此计划步骤所用的时间
  unsigned int elapsedTime{0};

  for (unsigned int iteration = 0;
       (iteration < cOpt().n_iterations && elapsedTime < maxStepDuration); ++iteration) {
    // 启动计时器来测量本次迭代的持续时间
    auto startTime = std::chrono::steady_clock::now();

    // 用全长初始化奖励向量（保留存储）
    // “stepReward”向量包含每个代理在树路径的特定步骤处的奖励。 树路径步骤与规划步骤无关
    std::vector<float> stepReward(root->m_agents.size(), 0);
    // 嵌套的“agentsRewards”向量包含从根节点开始沿着探索的树路径的每个步骤的“stepReward”
    std::vector<std::vector<float>> agentsRewards(cOpt().max_search_depth, stepReward);

    //#########################################################
    //### 第一阶段选择
    //#########################################################
    // 从根节点开始，递归地应用子选择策略来向下遍历树，直到到达最紧急的可扩展节点。 
    //如果节点表示非终结状态并且具有未访问（即未展开）的子节点，则该节点是可展开的
    ActionSet actionSet;

    auto node = root.get();
    //选择扩展节点
    node      = selectionPolicy->selectNodeForExpansion(node, actionSet, agentsRewards);
    // “node”现在是应展开的选定节点

    // 向代理的位置添加噪声
    if (cOpt().noise.active) {
      for (auto& agent : node->m_agents) {
        agent.m_vehicle.m_positionX =
            agent.m_vehicle.m_positionX + math::getNoise(cOpt().noise.mean, cOpt().noise.sigma);
        agent.m_vehicle.m_positionY =
            agent.m_vehicle.m_positionY + math::getNoise(cOpt().noise.mean, cOpt().noise.sigma);
      }
    }

    //#########################################################
    //### 第二阶段扩建
    //#########################################################
    // 根据actionSet添加一个子节点来展开树
    // 可以设置某些限制，例如最小访问次数...

    node = expansionPolicy->expandTree(node, actionSet, agentsRewards, cOpt().max_search_depth);
    // `node` 现在是已附加到搜索树的新节点

    //#########################################################
    //### 第三阶段模拟
    //#########################################################
    // 根据模拟策略从新节点运行模拟以产生结果

    auto simDepth = simulationPolicy->runSimulation(node, agentsRewards, cOpt().max_search_depth);
    // `simDepth` 指定执行模拟的深度

    //#########################################################
    //### 第 4 阶段反向传播
    //#########################################################
    // 所添加节点的评估通过所选节点逐步（即反向传播）备份到其父节点以更新其统计信息

    updatePolicy->updateTree(node, agentsRewards, simDepth);

    // 测量时间以确定本次迭代的持续时间
    auto endTime = std::chrono::steady_clock::now();
    // 更新计划步骤的已用时间
    elapsedTime +=
        std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
  }
  // 返回整个搜索树
  return root;
}

//##########################################################################
//### 主入口点：应用 MCTS 并返回最佳的 actionSetSequence
//##########################################################################

/**
 * @brief 应用 MCTS 并返回最佳操作集序列，即描述规划范围内所有代理（轨迹）的操作的操作集序列
 * @note 该函数是该库的主要入口点
 *
 * @param rootNode 指向根节点的指针
 * @param step 规划 当前规划步骤
 * @return actionSetSequence 包含最终选择的动作的动作集序列
 */
ActionSetSequence computeActionSetSequence(std::unique_ptr<Node> rootNode, int step) {
  // 该函数返回的actionSetSequence。 它包含应连续执行的最佳操作集
  ActionSetSequence actionSetSequence;
  // 最终搜索树的根节点
  std::unique_ptr<Node> rootFinal;

  // 设置线程安全随机引擎的种子
  // 将种子逐步相乘以在规划步骤之间实现伪随机
  /// @todo 考虑删除，这目前不会增加任何好处
  math::Random::g_seed = cOpt().random_seed + step * 1151;

  unsigned int nThreads{cOpt().parallelization_options.n_threads};
  if (nThreads > 1) {
    //### 根并行化的未来
    // 创造未来
    std::vector<std::future<std::unique_ptr<Node>>> rootFutures;
    std::vector<std::unique_ptr<Node>> roots(nThreads);
    for (unsigned int t = 0; t < nThreads; ++t) {
      roots[t] = std::make_unique<Node>(rootNode.get());
    }
    for (unsigned int t = 1; t <= nThreads; ++t) {
      // 为根并行化创建 lambda 函数
      auto func = [t, rootInLambda = &roots[t - 1], step]() -> std::unique_ptr<Node> {
        // 将线程 id 与随机数相乘并移位该步骤以创建随机盐
        math::Random::setSalt((t * 11779) + (step << 13));
        return computeTree(std::move(*rootInLambda));
      };
      // 推回作业并开始处理
      rootFutures.push_back(std::async(std::launch::async, func));
    }

    // 收集rootFutures结果
    for (unsigned int t = 0; t < nThreads; ++t) {
      roots[t] = rootFutures[t].get();
    }
    if (cOpt().parallelization_options.similarity_voting) {
      //// 如果启用了相似性投票，则使用相似性投票方法处理 roots
      actionSetSequence = similarityVoting(roots);
    } else {
      //// 否则，使用相似性合并方法处理 roots
      actionSetSequence = similarityMerge(roots);
    }
    //### 为了评估目的，将 ROOTFINAL 设置为其中一个根
    rootFinal = std::move(roots[0]);
  } else {
    rootFinal = computeTree(std::move(rootNode));

    // 用于最终选择的节点，对应于最终搜索树的根节点
    auto nodeFinalSelection = rootFinal.get();
    // 创建最终的选择策略
    auto finalSelectionPolicy =
        FinalSelectionPolicy::createPolicy(cOpt().policy_options.final_selection_policy);

    actionSetSequence = finalSelectionPolicy->getBestPlan(nodeFinalSelection);
  }

  if (oOpt().hasExportType("tree")) {
    rootFinal->exportTree(step);
  }

  //### 出口分布
  if (!actionSetSequence.empty()) {
    if (oOpt().hasExportType("childMap")) {
      rootFinal->exportChildMap(step, actionSetSequence[0]);
    }
    if (oOpt().hasExportType("permutationMap")) {
      rootFinal->exportPermutationMap(step, actionSetSequence[0]);
    }
    if (oOpt().hasExportType("moveGroups")) {
      rootFinal->exportMoveGroups(step);
    }
  }
  return actionSetSequence;
}

/**
 * @brief 使用“node”的操作更新“master”的操作
 *
 * @param master The master node.
 * @param node 比较节点
 */
void similarityUpdate(Node* const master, const Node* const node) {
  // update each agent
  for (unsigned int agentIndex = 0; agentIndex < master->m_agents.size(); ++agentIndex) {
    float similarity{0.0f};
    float n_new{0.0f};
    float n_old{0.0f};
    float q_old{0.0f};
    float n_other{0.0f};
    float q_other{0.0f};
    // masterActionPtr, masterActionValue
    for (auto& [masterActionPtr, masterActionValue] :
         master->m_agents[agentIndex].m_actionValues) {  // nodeActionPtr, nodeActionValue
      for (const auto& [nodeActionPtr, nodeActionValue] :
           node->m_agents[agentIndex].m_actionValues) {
        similarity = Action::getSimilarity(masterActionPtr, nodeActionPtr);
        n_old      = master->m_agents[agentIndex].m_actionVisits.at(masterActionPtr);
        q_old      = masterActionValue;
        n_other    = node->m_agents[agentIndex].m_actionVisits.at(nodeActionPtr);
        q_other    = nodeActionValue;
        // 跳过低相似度的计算
        if (similarity > 0.1) {
          n_new = n_old + similarity * n_other;
          // calculation of q_new
          masterActionValue = 1 / n_new * (q_old * n_old + q_other * similarity * n_other);
          // set n_new
          master->m_agents[agentIndex].m_actionVisits.at(masterActionPtr) = n_new;
        }
      }
    }
  }
}

/**
 * @brief Inserts the actions of `node` into the master node.
 *
 * @param master The master node
 * @param node The node to be merged.
 */
void mergeTrees(Node* const master, const Node* const node) {
  master->m_visits += node->m_visits;
  for (unsigned int agentIndex = 0; agentIndex < master->m_agents.size(); ++agentIndex) {
    master->m_agents[agentIndex].m_actionValues.insert(
        node->m_agents[agentIndex].m_actionValues.begin(),
        node->m_agents[agentIndex].m_actionValues.end());
    master->m_agents[agentIndex].m_actionVisits.insert(
        node->m_agents[agentIndex].m_actionVisits.begin(),
        node->m_agents[agentIndex].m_actionVisits.end());
  }
}

/**
 * @brief 将“resultRoots”中的所有子操作合并到“rootFinal”节点中，更新此新根节点的所有操作并返回最佳的最终操作
 *
 * @param resultRoots 生成的搜索树的根节点
 * @return actionSetSequence 包含最终选定的操作
 */
ActionSetSequence similarityMerge(const std::vector<std::unique_ptr<Node>>& resultRoots) {
  ActionSetSequence actionSetSequence;

  // 创建最终的选择策略
  auto finalSelectionPolicy =
      FinalSelectionPolicy::createPolicy(cOpt().policy_options.final_selection_policy);

  // 合并所有线程树
  std::unique_ptr<Node> rootFinal = std::make_unique<Node>(resultRoots[0].get());
  for (size_t t = 1; t < resultRoots.size(); ++t) {
    mergeTrees(rootFinal.get(), resultRoots[t].get());
  }

  // 根节点最终的更新操作
  for (auto& root : resultRoots) {
    similarityUpdate(rootFinal.get(), root.get());
  }

  // 提取最终动作
  Node* nodeFinalSelection = rootFinal.get();
  actionSetSequence        = finalSelectionPolicy->getBestPlan(nodeFinalSelection);

  return actionSetSequence;
}

/**
 * @brief 生成投票以从所有“resultRoots”中选择最佳的最终操作.
 *
 * @param resultRoots 生成的搜索树的根节点.
 * @return ActionSetSequence 包含最终选择的操作
 */
ActionSetSequence similarityVoting(const std::vector<std::unique_ptr<Node>>& resultRoots) {
  ActionSetSequence actionSetSequence;

  // 创建最终的选择策略
  auto finalSelectionPolicy =
      FinalSelectionPolicy::createPolicy(cOpt().policy_options.final_selection_policy);

  std::size_t size{resultRoots.size()};
  // 假设所有节点都有相同数量的代理！
  std::size_t agentsSize{resultRoots[0]->m_agents.size()};

  ActionSetSequence bestActions;
  for (size_t t = 0; t < size; ++t) {
    // 确定最佳行动
    Node* nodeFinalSelection = resultRoots[t].get();
    bestActions.push_back(finalSelectionPolicy->getBestActionSet(nodeFinalSelection));
  }

  // 创建一个矩阵，其中包含所有最佳操作之间的相似性乘以操作值
  std::vector<std::vector<std::vector<float>>> similarities;
  for (size_t t = 0; t < size; ++t) {
    std::vector<std::vector<float>> inner;
    for (size_t s = 0; s < size; ++s) {
      std::vector<float> innerInner(agentsSize, 0);
      inner.push_back(innerInner);
    }
    similarities.push_back(inner);
  }
  for (size_t t = 0; t < size; ++t) {
    for (size_t s = 0; s < size; ++s) {
      for (size_t a = 0; a < agentsSize; ++a) {
        similarities[t][s][a] = Action::getSimilarity(bestActions[t][a], bestActions[s][a]) *
                                resultRoots[s]->m_agents[a].m_actionValues.at(bestActions[s][a]);
      }
    }
  }

  // 对每一行求和以获得每个线程的投票
  std::vector<std::vector<float>> sumSimilarities;
  for (size_t s = 0; s < size; ++s) {
    std::vector<float> innerInner(agentsSize, 0);
    sumSimilarities.push_back(innerInner);
  }
  for (size_t t = 0; t < size; ++t) {
    for (size_t s = 0; s < size; ++s) {
      for (size_t a = 0; a < agentsSize; ++a) {
        sumSimilarities[t][a] += similarities[t][s][a];
      }
    }
  }

  // 确定每个代理的最高投票索引
  std::vector<int> max_index(agentsSize, 0);
  for (size_t a = 0; a < agentsSize; ++a) {
    max_index[a]  = 0;
    float current = sumSimilarities[0][a];
    for (size_t t = 0; t < size; ++t) {
      if (sumSimilarities[t][a] > current) {
        current      = sumSimilarities[t][a];
        max_index[a] = t;
      }
    }
  }

  // 提取最终动作
  ActionSet finalActionSet;
  for (size_t a = 0; a < agentsSize; ++a) {
    finalActionSet.push_back(bestActions[max_index[a]][a]);
  }

  actionSetSequence.push_back(finalActionSet);

  return actionSetSequence;
}

}  // namespace proseco_planning
