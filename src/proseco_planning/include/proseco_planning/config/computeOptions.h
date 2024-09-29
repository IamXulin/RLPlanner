/**
 * @file computeOptions.h
 * @brief This file defines the configuration related to the compute options.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <sys/types.h>
#include <string>

#include "nlohmann/json.hpp"
using json = nlohmann::json;

namespace proseco_planning::config {

/**
 * @brief The struct that contains the parameters for the similarity update, used for evaluating
 * similar actions of different action spaces.
 *
 */
struct SimilarityUpdate {
  /// The flag that indicates whether similarity update is enabled.
  const bool active;
  /// The value to control the size of the RBF kernel (larger values incorporate less actions,
  /// smaller values incorporate more actions).
  const float gamma;

  /**
   * @brief Constructs a new Similarity Update object.
   *
   * @param active The flag that indicates whether similarity update is enabled.
   * @param gamma The gamma to control the size of the RBF kernel.
   */
  SimilarityUpdate(bool active, float gamma) : active(active), gamma(gamma) {}

  json toJSON() const;

  static SimilarityUpdate fromJSON(const json& jSimilarityUpdate);
};

/**
 * @brief The struct that contains the parameters for the search guide, used for exploration.
 *
 */
struct SearchGuide {
  /// The number of samples to be drawn for the search guide calculation.
  const unsigned int n_samples;
  /// The type of the search guide.
  const std::string type;

  /**
   * @brief Constructs a new Search Guide object.
   *
   * @param n_samples The number of samples to be drawn for the search guide calculation.
   * @param type The type of the search guide.
   */
  SearchGuide(unsigned int n_samples, std::string type) : n_samples(n_samples), type(type) {}

  json toJSON() const;

  static SearchGuide fromJSON(const json& jSearchGuide);
};

/**
 * @brief The struct that contains the parameters responsible for move grouping in progressive
 * widening.
 */
struct MoveGroupingCriteriaPW {
  /// The flag that indicates whether progressive widening is based on move groups (i.e. true:
  /// action class specific values are used for criteria, false: node specific values are used for
  /// criteria).
  const bool active;
  /// The progressive widening coefficient.
  const float coefficient_pw;
  /// The progressive widening exponent.
  const float exponent_pw;
  /**
   * @brief Constructs a new Move Grouping Criteria PW object.
   *
   * @param active The flag that indicates whether progressive widening is based on move groups.
   * @param coefficient_pw The progressive widening coefficient.
   * @param exponent_pw The progressive widening exponent.
   */
  MoveGroupingCriteriaPW(bool active, float coefficient_pw, float exponent_pw)
      : active(active), coefficient_pw(coefficient_pw), exponent_pw(exponent_pw) {}

  json toJSON() const;

  static MoveGroupingCriteriaPW fromJSON(const json& jMoveGroupingCriteriaPW);
};

/**
 * @brief The struct that contains the the parameters for move grouping.
 *
 */
struct MoveGrouping {
  /// 指示是否启用移动分组的标志
  const bool active;
  /// 移动分组中 UCT 探索项的系数
  const float cp;
  /// 渐进式加宽中使用的移动分组标准
  const MoveGroupingCriteriaPW move_grouping_criteria_pw;
  /// 指示是否使用移动分组来偏置采样的标志
  const bool move_grouping_bias_pw;
  /// 指示移动组是否用于最终选择策略的标志
  const bool final_decision;

  /**
   * @brief Constructs a new Move Grouping object.
   *
   * @param active The flag that indicates whether move grouping is enabled.
   * @param cp The coefficient for the exploration term of UCT within move grouping.
   * @param move_grouping_criteria_pw The move grouping criteria that determines the progressive
   * widening process.
   * @param move_grouping_bias_pw The flag that indicates whether move grouping is used to bias
   * sampling.
   * @param final_decision The flag that indicates whether move groups are used for the final
   * selection policy.
   */
  MoveGrouping(bool active, float cp, MoveGroupingCriteriaPW move_grouping_criteria_pw,
               bool move_grouping_bias_pw, bool final_decision)
      : active(active),
        cp(cp),
        move_grouping_criteria_pw(move_grouping_criteria_pw),
        move_grouping_bias_pw(move_grouping_bias_pw),
        final_decision(final_decision) {}

  json toJSON() const;

  static MoveGrouping fromJSON(const json& jMoveGrouping);
};

/**
 * @brief The struct that contains the the parameters for progressive widening in the MCTS.
 *
 */
struct ProgressiveWidening {
  /// PW 所应用的搜索深度.
  const unsigned int max_depth_pw;
  /// The progressive widening exponent.
  const float exponent;
  /// The progressive widening coefficient.
  const float coefficient;

  /**
   * @brief Constructs a new Progressive Widening object.
   *
   * @param max_depth_pw The search depth up to which PW is being applied.
   * @param exponent The progressive widening exponent.
   * @param coefficient The progressive widening coefficient.
   */
  ProgressiveWidening(unsigned int max_depth_pw, float exponent, float coefficient)
      : max_depth_pw(max_depth_pw), exponent(exponent), coefficient(coefficient) {}

  json toJSON() const;

  static ProgressiveWidening fromJSON(const json& jProgressiveWidening);
};

/**
 * @brief The struct that contains the parameters for the parallelization of the MCTS.
 *
 */
struct ParallelizationOptions {
  /// 根并行化运行的线程数
  const unsigned int n_threads;
  /// 叶并行化运行的线程数
  const unsigned int n_simulationThreads;
  /// 指示相似性投票是否有效的标志
  const bool similarity_voting;
  /// 根并行化中相似函数的gamma 
  const float similarity_gamma;
  /// 聚合多个模拟线程的方法
  const std::string simulation_aggregation;

  ParallelizationOptions(unsigned int n_threads, unsigned int n_simulationThreads,
                         bool similarity_voting, float similarity_gamma,
                         std::string simulation_aggregation)
      : n_threads(n_threads),
        n_simulationThreads(n_simulationThreads),
        similarity_voting(similarity_voting),
        similarity_gamma(similarity_gamma),
        simulation_aggregation(simulation_aggregation) {}

  json toJSON() const;

  static ParallelizationOptions fromJSON(const json& ParallelizationOptions);
};

/**
 * @brief 该结构包含有关策略增强所需的连续操作空间中的搜索的捆绑信息
 *
 */
struct PolicyEnhancements {
  /// 存储相似性更新参数的结构体
  const SimilarityUpdate similarity_update;
  /// 存储搜索指南参数的结构体
  const SearchGuide search_guide;
  /// 存储移动分组参数的结构体
  const MoveGrouping move_grouping;
  /// 存储渐进加宽参数的结构体
  const ProgressiveWidening progressive_widening;
  /// 控制执行阶段轨迹长度的参数
  const float action_execution_fraction;
  /// 决定sample-exp-q策略锐度的参数
  const float q_scale;

  /**
   * @brief 构造一个新的策略增强对象
   *
   * @param similarity_update
   * @param search_guide
   * @param move_grouping
   * @param progressive_widening
   * @param action_execution_fraction
   * @param q_scale
   */
  PolicyEnhancements(SimilarityUpdate similarity_update, SearchGuide search_guide,
                     MoveGrouping move_grouping, ProgressiveWidening progressive_widening,
                     float action_execution_fraction, float q_scale)
      : similarity_update(similarity_update),
        search_guide(search_guide),
        move_grouping(move_grouping),
        progressive_widening(progressive_widening),
        action_execution_fraction(action_execution_fraction),
        q_scale(q_scale) {}

  json toJSON() const;

  static PolicyEnhancements fromJSON(const json& jPolicyEnhancements);
};

/**
 * @brief 包含核回归 LCB 参数的结构
 *
 */
struct KernelRegressionLCB {
  /// 指示是否启用移动分组的标志
  bool move_grouping;

  /// 动作回归的参数
  struct Action {
    /// 内核变量
    std::string kernel_variant;
    /// 相似度/核函数的gamma
    float gamma;
    /// 探索期的系数
    float cp;
  } action;

  /// 动作类别回归的参数
  struct ActionClass {
    /// 内核变量
    std::string kernel_variant;
    /// 相似度/核函数的gamma
    float gamma;
    /// 勘探期的系数
    float cp;
  } action_class;

  json toJSON() const;

  static KernelRegressionLCB fromJSON(const json& jKernelRegressionLCB);
};

/**
 * @brief The struct that contains the parameters for the PolicyOptions.
 *
 */
struct PolicyOptions {
  /// The selection policy used by the MCTS.
  const std::string selection_policy;
  /// The expansion policy used by the MCTS.
  const std::string expansion_policy;
  /// The simulation policy used by the MCTS.
  const std::string simulation_Policy;
  /// The update policy used by the MCTS.
  const std::string update_policy;
  /// The final selection policy used by the MCTS (determines how the actual action that gets
  /// executed is chosen).
  const std::string final_selection_policy;
  /// The policy enhancement object.
  const PolicyEnhancements policy_enhancements;
  /// The parameters for FinalSelectionKernelRegressionLCB
  KernelRegressionLCB kernel_regression_lcb;

  /**
   * @brief Constructs a new Policy Options object.
   *
   * @param selection_policy
   * @param expansion_policy
   * @param simulation_Policy
   * @param update_policy
   * @param final_selection_policy
   * @param policy_enhancements
   */
  PolicyOptions(const std::string& selection_policy, const std::string& expansion_policy,
                const std::string& simulation_Policy, const std::string& update_policy,
                const std::string& final_selection_policy, PolicyEnhancements policy_enhancements)
      : selection_policy(selection_policy),
        expansion_policy(expansion_policy),
        simulation_Policy(simulation_Policy),
        update_policy(update_policy),
        final_selection_policy(final_selection_policy),
        policy_enhancements(policy_enhancements) {}

  json toJSON() const;

  static PolicyOptions fromJSON(const json& jPolicyOptions);
};

/**
 * @brief The struct that contains the parameters for generating noise in general.
 *
 */
struct Noise {
  /// The flag that indicates whether noise is enabled.
  const bool active;
  /// The mean of the noise.
  const float mean;
  /// The standard deviation of the noise.
  const float sigma;
  /**
   * @brief Constructs a new Noise object.
   *
   * @param active
   * @param mean
   * @param sigma
   */
  Noise(bool active, float mean, float sigma) : active(active), mean(mean), sigma(sigma) {}

  json toJSON() const;

  static Noise fromJSON(const json& jNoise);
};
/**
 * @brief The struct that contains the parameters for generating noise particularly for actions.
 *
 */
struct ActionNoise {
  /// The flag that indicates whether noise is enabled.
  const bool active;
  /// The mean of the noise for the y-coordinate.
  const float meanY;
  /// The standard deviation of the noise for the y-coordinate.
  const float sigmaY;
  /// The mean of the noise for the x-velocity.
  const float meanVx;
  /// The standard deviation of the noise for the x-velocity.
  const float sigmaVx;
  /**
   * @brief Constructs a new Action Noise object.
   *
   * @param active
   * @param meanY
   * @param sigmaY
   * @param meanVx
   * @param sigmaVx
   */
  ActionNoise(bool active, float meanY, float sigmaY, float meanVx, float sigmaVx)
      : active(active), meanY(meanY), sigmaY(sigmaY), meanVx(meanVx), sigmaVx(sigmaVx) {}

  json toJSON() const;

  static ActionNoise fromJSON(const json& jNoise);
};
/**
 * @brief The struct that contains the hyperparameters necessary for the computation of the MCTS.
 *
 */
struct ComputeOptions {
  /// The random seed used to generate identical results.
  const unsigned int random_seed;
  /// The number of iterations the MCTS is run for.
  const unsigned int n_iterations;
  /// The [s] maximum duration a scenario is allowed to take (i.e. ensuring that a scenario does not
  /// run forever independent of whether a desired state is reached); 0 means no maximum duration
  /// limit.
  const float max_scenario_duration;
  /// The maximum number of steps a scenario is allowed to take (aka episode length; 0 means no
  /// maximum step limit).
  const unsigned int max_scenario_steps;
  /// The [s] maximum duration a step is allowed to take (i.e. ensuring a certain planning frequency
  /// 0.1s => 10Hz); 0 means no maximum duration limit
  const float max_step_duration;
  /// The maximum depth the MCTS looks into the future (i.e. the planning horizon).
  const unsigned int max_search_depth;
  /// The maximum number of invalid actions (of a single agent) to be sampled when
  /// expanding/simulating a node.
  const unsigned int max_invalid_action_samples;
  /// The influence of future rewards on the current action/state-value.
  const float discount_factor;
  /// The time delta between collision checks.
  const float delta_t;
  /// The gravity of earth.
  static constexpr float gravity{9.807f};
  /// 浮点数比较的阈值
  static constexpr float error_tolerance{1.0e-04};
  /** The initial UCT value for unexplored actions.
   * @note: This cannot be set to infinity or numeric_max() since the values are used for
   * calculations in the blindValue searchGuide. Setting max values will lead to overflow and
   * undefined behavior when summing UCT values there.
   */
  static constexpr float initial_uct{1.0e05};
  /// The duration of an action.
  const float action_duration;
  /// The type of collision checker.
  const std::string collision_checker;
  /// The safety distance used for collision checking.
  const float safety_distance;
  /// The type of end condition.
  const std::string end_condition;
  /// The policy options.
  const PolicyOptions policy_options;
  /// The parallel MCTS options.
  const ParallelizationOptions parallelization_options;
  /// The trajectory type.
  const std::string trajectory_type;
  /// The UCT cp.
  const float uct_cp;
  /// The noise added to agent position.
  const Noise noise;
  /// The noise applied to selected actions.
  const ActionNoise action_noise;
  /// 如果 > 0, 1) 该值充当最大观看距离 & 2) 分别为每个代理执行 mcts，而不是在单个（集中式）节点树中执行
  const float region_of_interest;
  /**
   * @brief 构造一个新的计算选项对象
   *
   * @param random_seed
   * @param n_iterations
   * @param max_scenario_duration
   * @param max_scenario_steps
   * @param max_step_duration
   * @param max_search_depth
   * @param max_invalid_action_samples
   * @param discount_factor
   * @param delta_t
   * @param action_duration
   * @param collision_checker
   * @param safety_distance
   * @param end_condition
   * @param policy_options
   * @param parallelization_options
   * @param trajectory_generation
   * @param uct_cp
   * @param noise
   * @param action_noise
   * @param region_of_interest
   */
  ComputeOptions(unsigned int random_seed, unsigned int n_iterations, float max_scenario_duration,
                 unsigned int max_scenario_steps, float max_step_duration,
                 unsigned int max_search_depth, unsigned int max_invalid_action_samples,
                 float discount_factor, float delta_t, float action_duration,
                 std::string collision_checker, float safety_distance, std::string end_condition,
                 PolicyOptions policy_options, ParallelizationOptions parallelization_options,
                 std::string trajectory_type, float uct_cp, Noise noise, ActionNoise action_noise,
                 const float region_of_interest)
      : random_seed(random_seed),
        n_iterations(n_iterations),
        max_scenario_duration(max_scenario_duration),
        max_scenario_steps(max_scenario_steps),
        max_step_duration(max_step_duration),
        max_search_depth(max_search_depth),
        max_invalid_action_samples(max_invalid_action_samples),
        discount_factor(discount_factor),
        delta_t(delta_t),
        action_duration(action_duration),
        collision_checker(collision_checker),
        safety_distance(safety_distance),
        end_condition(end_condition),
        policy_options(policy_options),
        parallelization_options(parallelization_options),
        trajectory_type(trajectory_type),
        uct_cp(uct_cp),
        noise(noise),
        action_noise(action_noise),
        region_of_interest(region_of_interest) {}

  json toJSON() const;

  static ComputeOptions fromJSON(const json& jComputeOptions);
};
}  // namespace proseco_planning::config
