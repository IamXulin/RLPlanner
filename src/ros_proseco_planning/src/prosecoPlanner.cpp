
#include <ros/console.h>
#include <ros/time.h>
#include <cassert>
#include <fstream>
#include <map>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nlohmann/json.hpp"
#include "proseco_planning/action/action.h"
#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/action/noiseGenerator.h"
#include "proseco_planning/agent/agent.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/collision_checker/collisionChecker.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/outputOptions.h"
#include "proseco_planning/config/scenarioOptions.h"
#include "proseco_planning/exporters/exporter.h"
#include "proseco_planning/math/mathlib.h"
#include "proseco_planning/monteCarloTreeSearch.h"
#include "proseco_planning/node.h"
#include "proseco_planning/scenarioEvaluation.h"
#include "proseco_planning/trajectory/trajectorygenerator.h"
#include "proseco_planning/util/alias.h"
#include "proseco_planning/util/utilities.h"
#include "ros_proseco_planning/prosecoPlanner.h"

namespace proseco_planning {

/**
 * @brief 构造一个新的 ProSeCo Planner 对象
 *
 * @param packagePath ros_proseco_planning 包的路径
 * @param optionsArg 选项文件相对于 ros_proseco_planning/config/options 的路径
 * @param scenarioArg 场景文件相对路径ros_proseco_planning/config/scenarios
 * @param absolute_path 指示是否使用绝对路径而不是相对于 ros_proseco_planning 包
 */
ProSeCoPlanner::ProSeCoPlanner(const std::string& packagePath, const std::string& optionsArg,
                               const std::string& scenarioArg, const bool absolute_path)
    : packagePath(packagePath), optionsArg(optionsArg), scenarioArg(scenarioArg) {
  std::string optionsPath{"/config/options/"};
  std::string scenarioPath{"/config/scenarios/"};

  // 加载选项
  ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
  const auto& options =
      config::Options::fromJSON(load_config(optionsArg, optionsPath, packagePath, absolute_path));
  ROS_INFO("选项已初始化");
  // 设置线程安全随机引擎的种子以生成随机性（需要在场景之前加载，因为场景使用引擎）
  math::Random::setRandomSeed(options.compute_options.random_seed);

  // 加载场景
  const auto& scenario = config::Scenario::fromJSON(
      load_config(scenarioArg, scenarioPath, packagePath, absolute_path));
  ROS_INFO("场景已初始化");

  // 将配置分配给规划器
  m_cfg = Config::create(scenario, options);

  // 根据数据格式初始化正确的导出器类
  if (oOpt().export_format != config::exportFormat::NONE)
    m_exporter = Exporter::createExporter(oOpt().output_path, oOpt().export_format);

  // 初始化噪声发生器
  m_noiseGenerator = std::make_unique<NoiseGenerator>();

  // 初始化环境
  m_environment = std::make_unique<Node>(sOpt().agents);
  // 检查起始位置不会产生无效状态或冲突
  const auto& [notInvalid, notColliding] = m_environment->validateInitialization();
  if (!notInvalid) {
    throw std::runtime_error("初始代理位置无效。正在中止");
  } else if (!notColliding) {
    throw std::runtime_error("初始代理位置会产生碰撞。中止");
  }

  ROS_INFO("已成功初始化 proseco 规划器");
}

/**
 * @brief 从 JSON 字符串加载配置
 * @details 它可以直接从命令行、绝对路径或相对于 ros_proseco 规划包的路径加载
 *
 * @param config 要加载的配置，可以是 .json 文件或 JSON 字符串
 * @param configPath 配置文件的路径
 * @param packagePath ros_proseco_planning 包路径
 * @param absolute_path 指示是否使用绝对路径而不是相对于 ros_proseco_planning 包
 * @return json 加载的 JSON 配置
 */
json ProSeCoPlanner::load_config(const std::string& config, const std::string& configPath,
                                 const std::string& packagePath, const bool absolute_path) {
  if (!util::hasEnding(config, ".json")) {
    ROS_INFO_STREAM("CONFIG IS NOT A FILE");
    return json::parse(config);
  } else if (absolute_path) {
    ROS_INFO_STREAM("Config: " + config);
    return util::loadJSON(config);
  } else {
    ROS_INFO_STREAM("Config: " + packagePath + configPath);
    return util::loadJSON(packagePath + configPath + config);
  }
}

/**
 * @brief 确定特定代理的感兴趣代理（即传感器范围内的代理）
 *
 * @param ego_agent 为其确定感兴趣的代理的代理
 * @return std::vector<Agent> 感兴趣区域内的代理
 */
std::vector<Agent> ProSeCoPlanner::agents_of_interest(const Agent& ego_agent) {
  std::vector<Agent> agents;
  agents.reserve(m_environment->m_agents.size());
  for (auto& agent : m_environment->m_agents) {
    // 如果不是智能体本身
    if (ego_agent.m_id != agent.m_id) {
      agent.is_ego = false;
      if (std::abs(agent.m_vehicle.m_positionX - ego_agent.m_vehicle.m_positionX) <
          cOpt().region_of_interest) {
        agents.emplace_back(agent);
      }
    }
    // 自我智能体始终处于 ROI 中
    else {
      agents.emplace_back(ego_agent);
    }
  }
  return agents;
}

/**
 * @brief 评估规划器是否已达到终止状态
 *
 * @return true 如果环境达到碰撞或无效状态，以及达到最大步数或最大持续时间。此外，当愿望得到满足或场景的终止条件得到满足时
 * @return false 否则.
 */
bool ProSeCoPlanner::isTerminal() const {
  // 如果发生碰撞或者达到无效状态则终止
  if (m_environment->m_collision || m_environment->m_invalid || max_steps_reached() ||
      max_duration_reached()) {
    return true;
  }
  // 如果达到期望则终止
  else if (cOpt().end_condition == "desire") {
    return m_environment->m_terminal && isScenarioTerminal(m_environment.get());
    // 如果满足场景中指定的终止条件则终止
  } else if (cOpt().end_condition == "scenario") {
    return isScenarioTerminal(m_environment.get());
  } else {
    return false;
  }
}

/**
 * @brief 检查是否已达到场景的最大规划步骤数
 *
 * @return true 如果已达到最大值
 * @return false 否则
 */
bool ProSeCoPlanner::max_steps_reached() const {
  return cOpt().max_scenario_steps != 0 && m_step >= cOpt().max_scenario_steps;
}

/**
 * @brief 检查是否已达到场景的最大规划持续时间
 *
 * @return true 如果已达到最大值
 * @return false 否则
 */
bool ProSeCoPlanner::max_duration_reached() const {
  return cOpt().max_scenario_duration != 0 && m_duration.toSec() > cOpt().max_scenario_duration;
}

/**
 * @brief 将规划器的结果和配置保存到磁盘
 *
 */
void ProSeCoPlanner::save() const {
  save_config();
  ROS_INFO("................");
  if (oOpt().hasExportType("result")){
    save_result();
    ROS_INFO("SAVE................");
  }else{
    ROS_INFO("NOT SAVE................");
  }
}

/**
 * @brief 将选项和场景配置保存到磁盘
 * @details 这对于分析和重现特定运行是必需的
 *
 */
void ProSeCoPlanner::save_config() const {
  ROS_INFO_STREAM("Writing output to: " + oOpt().output_path);

  util::saveJSON(oOpt().output_path + "/options_output", m_cfg->options.toJSON());
  util::saveJSON(oOpt().output_path + "/scenario_output", sOpt().toJSON());
}

/**
 * @brief 将结果保存到磁盘
 *
 */
void ProSeCoPlanner::save_result() const {
  json jResult;
  std::cout<<"======"<<sOpt().name<<std::endl;
  jResult["scenario"]          = sOpt().name;
  jResult["carsCollided"]      = m_environment->m_collision;
  jResult["carsInvalid"]       = m_environment->m_invalid;
  jResult["desiresFulfilled"]  = m_environment->m_terminal;
  jResult["maxSimTimeReached"] = max_duration_reached();
  jResult["maxStepsReached"]   = max_steps_reached();
  // final step is the last step that has been executed
  jResult["finalstep"] = m_step - 1;
  // normalize over steps to get a fully comparable reward value for one run of the MCTS
  // m_step is incremented at the end of the plan method
  jResult["normalizedEgoRewardSum"]  = m_normalizedEgoRewardSum / m_step;
  jResult["normalizedCoopRewardSum"] = m_normalizedCoopRewardSum / m_step;
  jResult["normalizedStepDuration"]  = m_duration.toSec() / m_step;
  std::cout<<oOpt().output_path<<"=========="<<std::endl;
  //util::saveJSON(oOpt().output_path + "/result", jResult);
  std::string filePath=oOpt().output_path + "/result";
  filePath="/home/xulin/proseco_workspace/src/ros_proseco_planning/config/xulin/result";
  std::ofstream file(filePath + ".json");
  std::cout<<"----"<<filePath<<std::endl;
  file << jResult;
  file.close();
}

/**
 * @brief Determines the minimum size of the action set sequence of the decentralized plans.
 *
 * @param decentralizedActionSetSequences The different action set sequences for each agent.
 * @return size_t
 * @todo refactor/doc
 */
size_t ProSeCoPlanner::getMinimumSequenceSize(
    const std::vector<ActionSetSequence>& decentralizedActionSetSequences) {
  size_t minSequenceSize = decentralizedActionSetSequences[0].size();
  for (const auto& sequences : decentralizedActionSetSequences) {
    if (sequences.size() < minSequenceSize) {
      minSequenceSize = sequences.size();
    }
  }
  return minSequenceSize;
}

/**
 * @brief 从分散的规划结果中选择每个代理的动作集序列
 *
 * @param decentralizedActionSetSequences
 * @param agentsROI
 * @return 动作集序列
 * @todo refactor/doc
 */
ActionSetSequence ProSeCoPlanner::mergeDecentralizedActionSetSequences(
    const std::vector<ActionSetSequence>& decentralizedActionSetSequences,
    std::vector<std::vector<Agent>>& agentsROI) const {
  // 创建动作集序列向量，其中每个条目代表所有代理在此节点采取的动作
  ActionSetSequence actionSetSequence;
  // 分散的动作集序列可能具有不同的深度，因此我们确定所有返回的动作集序列的最小值
  const auto minSequenceSize = getMinimumSequenceSize(decentralizedActionSetSequences);
  for (size_t sequenceIdx{}; sequenceIdx < minSequenceSize; ++sequenceIdx) {
    ActionSet actionSet;
    // 对于每个去中心化节点，只获取该节点“自我代理”的返回动作
    for (size_t nodeIdx{}; nodeIdx < m_environment->m_agents.size(); ++nodeIdx) {
      // rootAgents是这个去中心化节点中的所有代理
      auto& rootAgents            = agentsROI[nodeIdx];
      auto decentralizedActionSet = decentralizedActionSetSequences[nodeIdx][sequenceIdx];

      // 找到去中心化节点的“ego”代理的动作集
      for (size_t agentIdx{}; agentIdx < rootAgents.size(); ++agentIdx) {
        if (m_environment->m_agents[nodeIdx].m_id == rootAgents[agentIdx].m_id) {
          actionSet.emplace_back(decentralizedActionSet[agentIdx]);
          break;
        }
      }
    }
    actionSetSequence.emplace_back(actionSet);
  }
  return actionSetSequence;
}

/**
 * @brief 覆盖预定义代理选择的操作
 * @details 如果预定义了代理，仍将根据最终选择策略选择操作，因此为了确保它实际执行预定义操作，此处将其覆盖
 *
 * @param actionSet 要执行的动作集
 */
void ProSeCoPlanner::overrideActionsPredefined(ActionSet& actionSet) const {
  for (size_t agentIdx{}; agentIdx < sOpt().agents.size(); ++agentIdx) {
    // modify output
    if (sOpt().agents[agentIdx].is_predefined) {
      actionSet.at(agentIdx) =
          std::make_shared<Action>(Action(ActionClass::DO_NOTHING, 0.0f, 0.0f));
    }
  }
}

/**
 * @brief 在场景中积累自我以及合作奖励
 * @details 使用场景中代理的数量对总和进行归一化，以使不同场景之间具有可比性
 *
 */
void ProSeCoPlanner::accumulate_reward() {
  float egoRewardSum =
      std::accumulate(m_environment->m_agents.begin(), m_environment->m_agents.end(), 0.0f,
                      [](float sum, const Agent& agent) { return (sum + agent.m_egoReward); });

  float coopRewardSum =
      std::accumulate(m_environment->m_agents.begin(), m_environment->m_agents.end(), 0.0f,
                      [](float sum, const Agent& agent) { return (sum + agent.m_coopReward); });
  // 由于合作奖励已经是所有智能体的加权总和，因此我们用智能体的数量对总和进行标准化以保持合作因子
  // coopRewardSum /= m_environment->m_agents.size();

  // 与代理标准化，因此奖励可用于比较具有不同数量代理的场景
  egoRewardSum /= m_environment->m_agents.size();
  coopRewardSum /= m_environment->m_agents.size();

  // 总结所有步骤的奖励； 最后，我们对最终的步骤量进行规范，以便能够将相同的场景与不同的步骤量进行比较
  m_normalizedEgoRewardSum += egoRewardSum;
  m_normalizedCoopRewardSum += coopRewardSum;
}

/**
 * @brief Starts the actual search of the Monte Carlo Tree Search.
 *
 */
void ProSeCoPlanner::plan() {
  // 每个 mct 中每个代理的动作向量
  ActionSetSequence actionSetSequence;
  std::vector<ActionSetSequence> decentralizedActionSetSequences;
  std::vector<std::vector<Agent>> agentsROI;

  // 追踪表现的计时器
  ros::Time startTime;
  ros::Time endTime;
  ros::Duration stepDuration;

  //### 开始搜索
  while (!ProSeCoPlanner::isTerminal()) {
    // 使用MCTS计算最佳动作集
    // 测量场景开始以来的时间
    startTime     = ros::Time::now();
    auto rootNode = std::make_unique<Node>(m_environment.get());
    if (cOpt().region_of_interest > 0.0f) {
      decentralizedActionSetSequences.clear();
      agentsROI.clear();
      // 使用各自的 ROI 初始化每个代理的每个根节点
      for (const auto& agent : m_environment->m_agents) {
        auto _rootNode      = std::make_unique<Node>(m_environment.get());
        _rootNode->m_agents = agents_of_interest(agent);
        agentsROI.emplace_back(_rootNode->m_agents);
        decentralizedActionSetSequences.emplace_back(
            computeActionSetSequence(std::move(_rootNode), m_step));
      }
      actionSetSequence =
          mergeDecentralizedActionSetSequences(decentralizedActionSetSequences, agentsROI);
    } else {
      actionSetSequence = computeActionSetSequence(std::move(rootNode), m_step);
    }

    endTime      = ros::Time::now();
    stepDuration = endTime - startTime;
    m_duration += stepDuration;
    ROS_INFO_STREAM("step " << m_step << " took\t" << stepDuration.toSec() << " seconds");

    ActionSet actionSet;
    if (actionSetSequence.empty()) {
      ROS_FATAL_STREAM("迭代次数太少 正在中止.");
      throw std::runtime_error("规划者未制定有效的计划.");
    } else {
      actionSet.clear();
      actionSet = actionSetSequence[0];
    }

    // 为操作添加噪音（如果启用）
    if (cOpt().action_noise.active) {
      actionSet = m_noiseGenerator->createNoisyActions(actionSet);
    }
    // 如果预定义了其他车辆，则修改输出（保持速度）
    overrideActionsPredefined(actionSet);

    // 保存rootNode的状态以导出单次计划
    auto singleShotNode = std::make_unique<Node>(m_environment.get());

    // 创建碰撞检查器
    auto collisionChecker =
        CollisionChecker::createCollisionChecker(cOpt().collision_checker, 0.0f);

    // 创建轨迹生成器
    auto trajectoryGenerator =
        TrajectoryGenerator::createTrajectoryGenerator(cOpt().trajectory_type);

    for (auto action : actionSet){
      std::cout<<action.get()->getDistance()<<std::endl;
    }
    // 执行动作集
    m_environment->executeActions(actionSet, *collisionChecker, *trajectoryGenerator, true);

    // 输出
    if (oOpt().export_format != config::exportFormat::NONE) {
      // 将当前步骤保存到轨迹中
      m_exporter->exportTrajectory(m_environment.get(), actionSet, m_step);
      // 保存数据用于逆强化学习
      m_exporter->exportIrlTrajectory(m_environment.get(), actionSet, m_step);

      // 导出单次计划
      if (oOpt().hasExportType("singleShot"))
        m_exporter->exportSingleShot(singleShotNode.get(), actionSetSequence, m_step);
    }
    
    // 累积奖励指标
    accumulate_reward();

    // 增加步长
    ++m_step;

  }  //### 搜索结束
  if (m_step == 0) {
    throw std::runtime_error("Planning finished before any steps have been executed.");
  }
  // 将轨迹写入磁盘
  if (oOpt().export_format != config::exportFormat::NONE) {
    if (oOpt().hasExportType("trajectory"))
      m_exporter->writeData(m_step, ExportType::EXPORT_TRAJECTORY);
    if (oOpt().hasExportType("irlTrajectory"))
      m_exporter->writeData(m_step, ExportType::EXPORT_IRL_TRAJECTORY);
  }
};
}  // namespace proseco_planning