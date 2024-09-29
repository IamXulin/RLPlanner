/**
 * @file scenarioOptions.h
 * @brief This file defines the scenario configuration.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <sys/types.h>
#include <eigen3/Eigen/Core>
#include <string>
#include <variant>
#include <vector>

#include "nlohmann/json.hpp"
using json = nlohmann::json;

namespace proseco_planning::config {
/**
 * @brief The struct that contains the specifications of the obstacles involved in a scenario.
 *
 */
struct Obstacle {
  /// The unique identifier.
  const unsigned int id;
  /// The flag that indicates randomness in the objects attributes.
  const bool random;
  /// The x position [m].
  const float position_x;
  /// The y position [m].
  const float position_y;
  /// The heading [rad].
  const float heading;
  /// The length [m].
  const float length;
  /// The width [m].
  const float width;
  /// The standard deviation in x position [m].
  const float sigma_position_x;
  /// The standard deviation in y position [m].
  const float sigma_position_y;
  /// The standard deviation in heading [rad].
  const float sigma_heading;
  /// The standard deviation in length [m].
  const float sigma_length;
  /// The standard deviation in width [m].
  const float sigma_width;

  /**
   * @brief Constructs a new Obstacle object.
   *
   * @param id
   * @param random
   * @param position_x
   * @param position_y
   * @param heading
   * @param length
   * @param width
   * @param sigma_position_x
   * @param sigma_position_y
   * @param sigma_heading
   * @param sigma_length
   * @param sigma_width
   */
  Obstacle(unsigned int id, bool random, float position_x, float position_y, float heading,
           float length, float width, float sigma_position_x, float sigma_position_y,
           float sigma_heading, float sigma_length, float sigma_width)
      : id(id),
        random(random),
        position_x(position_x),
        position_y(position_y),
        heading(heading),
        length(length),
        width(width),
        sigma_position_x(sigma_position_x),
        sigma_position_y(sigma_position_y),
        sigma_heading(sigma_heading),
        sigma_length(sigma_length),
        sigma_width(sigma_width) {}

  json toJSON() const;

  static Obstacle fromJSON(const json& jObstacle);
};

/**
 * @brief The specification of the road.
 *
 */
struct Road {
  /// The flag that indicates randomness in the objects attributes.
  const bool random{};
  /// The number of lanes the road consists of.
  const unsigned int number_lanes{};
  /// The lane width of each lane [m].
  const float lane_width{};
  /// The standard deviation of the lane width [m].
  const float sigma_lane_width{};
  /**
   * @brief Constructs a new Road object.
   *
   * @param random
   * @param number_lanes
   * @param lane_width
   * @param sigma_lane_width
   */
  Road(bool random, unsigned int number_lanes, float lane_width, float sigma_lane_width)
      : random(random),
        number_lanes(number_lanes),
        lane_width(lane_width),
        sigma_lane_width(sigma_lane_width) {}

  json toJSON() const;

  static Road fromJSON(const json& jRoad);
};

/**
 * @brief The specification of the vehicle.
 *
 */
struct Vehicle {
  /// The flag that indicates randomness in the objects attributes.
  const bool random;
  /// The x position [m].
  const float position_x;
  /// The y position [m].
  const float position_y;
  /// The x velocity [m/s].
  const float velocity_x;
  /// The y velocity [m/s].
  const float velocity_y;
  /// The heading [rad].
  const float heading;
  /// The width [m].
  const float width;
  /// The length [m].
  const float length;
  /// The standard deviation in x position [m].
  const float sigma_position_x;
  /// The standard deviation in y position [m].
  const float sigma_position_y;
  /// The standard deviation in x velocity [m/s].
  const float sigma_velocity_x;
  /// The standard deviation in y velocity [m/s].
  const float sigma_velocity_y;
  /// The standard deviation in heading [rad].
  const float sigma_heading;
  /// The standard deviation in width [m].
  const float sigma_width;
  /// The standard deviation in length [m].
  const float sigma_length;
  /// The wheel base [m].
  const float wheel_base;
  /// The maximum steering angle [rad].
  const float max_steering_angle;
  /// The maximum speed [m/s].
  const float max_speed;
  /// The maximum acceleration [m/s^2].
  const float max_acceleration;
  /**
   * @brief Constructs a new Vehicle object.
   *
   * @param random
   * @param position_x
   * @param position_y
   * @param velocity_x
   * @param velocity_y
   * @param heading
   * @param length
   * @param width
   * @param sigma_position_x
   * @param sigma_position_y
   * @param sigma_velocity_x
   * @param sigma_velocity_y
   * @param sigma_heading
   * @param sigma_length
   * @param sigma_width
   * @param wheel_base
   * @param max_steering_angle
   * @param max_speed
   * @param max_acceleration
   */
  Vehicle(bool random, float position_x, float position_y, float velocity_x, float velocity_y,
          float heading, float length, float width, float sigma_position_x, float sigma_position_y,
          float sigma_velocity_x, float sigma_velocity_y, float sigma_heading, float sigma_length,
          float sigma_width, float wheel_base, float max_steering_angle, float max_speed,
          float max_acceleration)
      : random(random),
        position_x(position_x),
        position_y(position_y),
        velocity_x(velocity_x),
        velocity_y(velocity_y),
        heading(heading),
        width(width),
        length(length),
        sigma_position_x(sigma_position_x),
        sigma_position_y(sigma_position_y),
        sigma_velocity_x(sigma_velocity_x),
        sigma_velocity_y(sigma_velocity_y),
        sigma_heading(sigma_heading),
        sigma_width(sigma_width),
        sigma_length(sigma_length),
        wheel_base(wheel_base),
        max_steering_angle(max_steering_angle),
        max_speed(max_speed),
        max_acceleration(max_acceleration) {}

  json toJSON() const;

  static Vehicle fromJSON(const json& jVehicle);
};

/**
 * @brief The struct that contains the desire of the agent.
 *
 */
struct Desire {
  /// The desired velocity [m/s].
  const float velocity;
  /// The tolerance for velocity deviation [m/s], so that the desire remains fulfilled.
  const float velocity_tolerance;
  /// The desired lane.
  const unsigned int lane;
  /// The tolerance for lane center deviation [m], so that the desire remains fulfilled.
  const float lane_center_tolerance;

  /**
   * @brief Constructs a new Desire object.
   *
   * @param velocity
   * @param velocity_tolerance
   * @param lane
   * @param lane_center_tolerance
   */
  Desire(float velocity, float velocity_tolerance, unsigned int lane, float lane_center_tolerance)
      : velocity(velocity),
        velocity_tolerance(velocity_tolerance),
        lane(lane),
        lane_center_tolerance(lane_center_tolerance) {}

  json toJSON() const;

  static Desire fromJSON(const json& jDesire);
};

/**
 * @brief 包含终止条件的结构体
 *
 */
struct TerminalCondition {
  /// x 位置 [m]
  const float position_x;
  /// y 位置 [m]
  const float position_y;
  /// 定义要比较的关系的比较器
  const std::string comparator_position_x;
  /// 定义要比较的关系的比较器
  const std::string comparator_position_y;

  /**
   * @brief 构造一个新的终端条件对象
   *
   * @param position_x
   * @param position_y
   * @param comparator_position_x
   * @param comparator_position_y
   */
  TerminalCondition(float position_x, float position_y, std::string comparator_position_x,
                    std::string comparator_position_y)
      : position_x(position_x),
        position_y(position_y),
        comparator_position_x(comparator_position_x),
        comparator_position_y(comparator_position_y) {}

  json toJSON() const;

  static TerminalCondition fromJSON(const json& jTerminalCondition);
};

/**
 * @brief 包含有关矩形形式的动作空间信息的结构体
 *
 */
struct ActionSpaceRectangle {
  /// 动作空间类型
  inline static const std::string TYPE{"rectangle"};
  /// 最大纵向速度变化
  const float max_velocity_change;
  /// 最大横向位置变化
  const float max_lateral_change;
  /// 指定动作类“do_nothing”和“accelerate”之间界限的速度变化。
  const float delta_velocity;

  /**
   * @brief 构造一个新的动作空间矩形对象
   *
   * @param max_velocity_change
   * @param max_lateral_change
   * @param delta_velocity
   */
  ActionSpaceRectangle(float max_velocity_change, float max_lateral_change, float delta_velocity)
      : max_velocity_change(max_velocity_change),
        max_lateral_change(max_lateral_change),
        delta_velocity(delta_velocity){};

  json toJSON() const;

  static ActionSpaceRectangle fromJSON(const json& jActionSpaceRectangle);
};

/**
 * @brief The struct that contains the parameters of the ActionSpace.
 *
 */
struct ActionSpace {
  /// The enum for setting the type of the action space
  enum class Type {
    /// The invalid action space type (default for variant type).
    INVALID,
    /// The rectangular action space type.
    Rectangle
  };

  using variant_t = std::variant<std::monostate, ActionSpaceRectangle>;

  static json toJSON(const ActionSpace::variant_t& variant);

  static ActionSpace::variant_t fromJSON(const json& jActionSpace);
};

/// Maps ActionSpace::Type values to JSON as strings.*/
NLOHMANN_JSON_SERIALIZE_ENUM(ActionSpace::Type,
                             {
                                 {ActionSpace::Type::INVALID, nullptr},  // Invalid is default
                                 {ActionSpace::Type::Rectangle, ActionSpaceRectangle::TYPE},
                             })

struct CostModel {
  /// 成本模型名称
  const std::string name;
  /// 变道权重
  const float w_lane_change;
  /// 车道偏离的权重（与所需车道相比）
  const float w_lane_deviation;
  /// 偏离车道中心的权重
  const float w_lane_center_deviation;
  /// 速度偏差的权重（与所需速度相比）
  const float w_velocity_deviation;
  /// x轴 加速度的权重
  const float w_acceleration_x;
  /// y轴 加速度的权重
  const float w_acceleration_y;
  /// 碰撞权重
  const float cost_collision;
  /// 无效状态的权重（即越野行驶）
  const float cost_invalid_state;
  /// 无效动作的权重（即违反最大转向角度或加速度）
  const float cost_invalid_action;
  /// 达到安全范围的重量@todo remove。
  const float cost_enter_safe_range;
  /// 最终奖励的权重（即当场景被认为已解决时）
  const float reward_terminal;

  ///@{
  /// 线性协作参数。 具体定义参见线性参数
  const float w_acceleration_y_cooperative;
  const float w_lane_deviation_cooperative;
  const float w_lane_center_deviation_cooperative;
  const float w_velocity_deviation_cooperative;
  const float cost_collision_cooperative;
  const float cost_invalid_state_cooperative;
  const float cost_invalid_action_cooperative;
  ///@}

  ///@{
  /// 非线性参数
  const Eigen::MatrixXd w1;
  const Eigen::MatrixXd w2;
  ///@}

  /**
   * @brief 构造一个新的成本模型对象
   *
   * @param name
   * @param w_lane_change
   * @param w_lane_deviation
   * @param w_lane_center_deviation
   * @param w_velocity_deviation
   * @param w_acceleration_x
   * @param w_acceleration_y
   * @param cost_collision
   * @param cost_invalid_state
   * @param cost_invalid_action
   * @param cost_enter_safe_range
   * @param reward_terminal
   * @param w_acceleration_y_cooperative
   * @param w_lane_deviation_cooperative
   * @param w_lane_center_deviation_cooperative
   * @param w_velocity_deviation_cooperative
   * @param cost_collision_cooperative
   * @param cost_invalid_state_cooperative
   * @param cost_invalid_action_cooperative
   * @param w1
   * @param w2
   */
  CostModel(const std::string& name, float w_lane_change, float w_lane_deviation,
            float w_lane_center_deviation, float w_velocity_deviation, float w_acceleration_x,
            float w_acceleration_y, float cost_collision, float cost_invalid_state,
            float cost_invalid_action, float cost_enter_safe_range, float reward_terminal,
            float w_acceleration_y_cooperative, float w_lane_deviation_cooperative,
            float w_lane_center_deviation_cooperative, float w_velocity_deviation_cooperative,
            float cost_collision_cooperative, float cost_invalid_state_cooperative,
            float cost_invalid_action_cooperative, Eigen::MatrixXd w1, Eigen::MatrixXd w2)
      : name(name),
        w_lane_change(w_lane_change),
        w_lane_deviation(w_lane_deviation),
        w_lane_center_deviation(w_lane_center_deviation),
        w_velocity_deviation(w_velocity_deviation),
        w_acceleration_x(w_acceleration_x),
        w_acceleration_y(w_acceleration_y),
        cost_collision(cost_collision),
        cost_invalid_state(cost_invalid_state),
        cost_invalid_action(cost_invalid_action),
        cost_enter_safe_range(cost_enter_safe_range),
        reward_terminal(reward_terminal),
        w_acceleration_y_cooperative(w_acceleration_y_cooperative),
        w_lane_deviation_cooperative(w_lane_deviation_cooperative),
        w_lane_center_deviation_cooperative(w_lane_center_deviation_cooperative),
        w_velocity_deviation_cooperative(w_velocity_deviation_cooperative),
        cost_collision_cooperative(cost_collision_cooperative),
        cost_invalid_state_cooperative(cost_invalid_state_cooperative),
        cost_invalid_action_cooperative(cost_invalid_action_cooperative),
        w1(w1),
        w2(w2) {}

  json toJSON() const;

  static CostModel fromJSON(const json& jCostModel);

  static Eigen::MatrixXd convertVectorToEigenMatrix(const std::vector<float>& values, int nRows,
                                                    int nCols);

  static std::tuple<std::vector<float>, unsigned int, unsigned int> convertEigenMatrixToVector(
      const Eigen::MatrixXd& matrix);
};

/**
 * @brief The agent.
 *
 */
struct Agent {
  /// 唯一标识符
  const unsigned int id;
  /// 指示恒定行为（即恒定速度）的标志
  const bool is_predefined;
  /// 加权纳入其他代理人奖励的系数
  const float cooperation_factor;
  /// 代理人的期望.
  const Desire desire;
  /// 代理人的车辆
  const Vehicle vehicle;
  /// 最终条件（例如，当场景可以被认为已解决时）
  const TerminalCondition terminal_condition;
  /// 行动空间
  const ActionSpace::variant_t action_space;
  /// 用于计算成本的成本模型
  const CostModel cost_model;

  /**
   * @brief 构造一个新的 Agent 对象
   *
   * @param id
   * @param is_predefined
   * @param cooperation_factor
   * @param desire
   * @param vehicle
   * @param terminal_condition
   * @param action_space
   * @param cost_model
   */
  Agent(unsigned int id, bool is_predefined, float cooperation_factor, Desire desire,
        Vehicle vehicle, TerminalCondition terminal_condition, ActionSpace::variant_t action_space,
        CostModel cost_model)
      : id(id),
        is_predefined(is_predefined),
        cooperation_factor(cooperation_factor),
        desire(desire),
        vehicle(vehicle),
        terminal_condition(terminal_condition),
        action_space(action_space),
        cost_model(cost_model) {}

  json toJSON() const;

  static Agent fromJSON(const json& jAgent);
};

/**
 * @brief The simulated scenario.
 */
struct Scenario {
  /// 场景名称（例如 SC01）
  const std::string name;
  /// 场景道路
  config::Road road;
  /// 场景智能体
  const std::vector<config::Agent> agents;
  /// 场景障碍物
  const std::vector<config::Obstacle> obstacles;
  // 构造函数
  Scenario(const std::string& name, config::Road road, std::vector<config::Agent> agents,
           std::vector<config::Obstacle> obstacles)
      : name(name), road(road), agents(agents), obstacles(obstacles) {}

  json toJSON() const;

  static Scenario fromJSON(const json& jScenario);
};
}  // namespace proseco_planning::config