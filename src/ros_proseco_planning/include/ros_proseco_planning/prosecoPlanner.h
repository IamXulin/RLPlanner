/**
 * @file prosecoPlanner.h
 * @brief The definition of the ProSeCoPlanner.
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

#include <ros/console_backend.h>
#include <ros/duration.h>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"
using json = nlohmann::json;
#include "proseco_planning/action/noiseGenerator.h"
#include "proseco_planning/config/outputOptions.h"
#include "proseco_planning/exporters/exporter.h"
#include "proseco_planning/node.h"
#include "proseco_planning/util/alias.h"

namespace proseco_planning {
class Agent;
class Config;

/*!
 * @brief ProSeCoPlanner 类调用所有必要的初始化程序并执行基于蒙特卡罗树搜索 (MCTS) 的规划算法
 */

using config::exportFormat;

class ProSeCoPlanner {
 private:
  /// 导出器将各种数据保存到磁盘
  std::unique_ptr<Exporter> m_exporter;

  /// ROS包的路径
  const std::string packagePath;

  /// options.json 文件的文件名
  const std::string optionsArg;

  /// scene.json 文件的文件名
  const std::string scenarioArg;

  /// 指向蒙特卡罗搜索树根节点的指针。 它是一个向量，因为搜索可以在根节点级别并行化
  std::vector<std::unique_ptr<Node>> m_rootNodes;

  /// 指向模拟环境状态的指针
  std::unique_ptr<Node> m_environment;

  /// 中央配置对象。 它是使用单例实现的，因此可以在整个程序中使用
  const Config* m_cfg{nullptr};

  /// 场景当前步骤的计数器
  unsigned int m_step{0};

  /// 所有代理场景中的标准化利己奖励值
  float m_normalizedEgoRewardSum{0};

  /// 场景中所有智能体的标准化合作奖励值
  float m_normalizedCoopRewardSum{0};

  /// 规划器的持续时间
  ros::Duration m_duration{0};

  /// 动作噪声发生器。 它可用于向操作添加噪音，从而使操作的执行具有不确定性
  std::unique_ptr<NoiseGenerator> m_noiseGenerator;

 public:
  ProSeCoPlanner(const std::string& packagePath, const std::string& optionsArg,
                 const std::string& scenarioArg, const bool absolute_path);

  static json load_config(const std::string& argument, const std::string& packagePath,
                          const std::string& argPath, const bool absolute_path);

  std::vector<Agent> agents_of_interest(const Agent& ego_agent);

  bool isTerminal() const;

  bool max_steps_reached() const;

  bool max_duration_reached() const;

  void save() const;

  void save_config() const;

  void save_result() const;

  void setLogLevel(ros::console::Level level) const;

  static size_t getMinimumSequenceSize(
      const std::vector<ActionSetSequence>& decentralizedActionSetSequences);

  ActionSetSequence mergeDecentralizedActionSetSequences(
      const std::vector<ActionSetSequence>& decentralizedActionSetSequences,
      std::vector<std::vector<Agent>>& agentsROI) const;

  void accumulate_reward();

  void overrideActionsPredefined(ActionSet& actionSet) const;

  void plan();
};
}  // namespace proseco_planning
