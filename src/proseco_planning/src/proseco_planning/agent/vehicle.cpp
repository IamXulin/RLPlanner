#include "proseco_planning/agent/vehicle.h"

#include <sys/types.h>
#include <map>

#include "nlohmann/json.hpp"

namespace proseco_planning {

/**
 * @brief 使用车辆配置构造新的车辆对象
 *
 * @param vehicle 车辆配置
 */
Vehicle::Vehicle(const config::Vehicle& vehicle)
    : m_positionX(vehicle.position_x),
      m_positionY(vehicle.position_y),
      m_velocityX(vehicle.velocity_x),
      m_velocityY(vehicle.velocity_y),
      m_accelerationX(0),
      m_accelerationY(0),
      m_heading(vehicle.heading),
      m_yawRate(0),
      m_lane(getLane(vehicle.position_y)),
      m_length(vehicle.length),
      m_width(vehicle.width),
      m_wheelBase(vehicle.wheel_base),
      m_maxSteeringAngle(vehicle.max_steering_angle),
      m_maxSpeed(vehicle.max_speed),
      m_maxAcceleration(vehicle.max_acceleration){};
/**
 * @brief 设置车辆的车道以及 y 坐标
 * @details 此方法仅应在场景初始化期间调用，因为它还会将车辆的 y 坐标更新为车道的正确 y 坐标
 *
 * @param lane 车辆应设置的车道
 */
void Vehicle::setLane(const int lane) {
  m_lane      = lane;
  m_positionY = (lane + 0.5) * sOpt().road.lane_width;
}

/**
 * @brief 计算车辆后轴中心到车道中心的距离
 *
 * @return float 到车辆车道中心的距离
 */
float Vehicle::getDistanceToLaneCenter() const {
  return (m_lane + 0.5) * sOpt().road.lane_width - m_positionY;
}

/**
 * @brief 计算从车辆后轴中心到车辆左侧下一车道的距离
 *
 * @return float 车辆与左侧相邻车道之间的距离
 */
float Vehicle::getDistanceToLeftLane() const {
  return (m_lane + 1) * sOpt().road.lane_width - m_positionY;
}

/**
 * @brief 计算车辆后轴中心到车辆右侧下一车道的距离.
 *
 * @return float 到车辆右侧下一车道的距离.
 */
float Vehicle::getDistanceToRightLane() const {
  return -(m_positionY - m_lane * sOpt().road.lane_width);
}

/**
 * @brief 计算车辆后轴中心到车辆左侧下一车道中心的距离
 *
 * @return float 到车辆左侧下一车道中心的距离
 */
float Vehicle::getDistanceToLeftLaneCenter() const {
  return 0.5 * sOpt().road.lane_width + getDistanceToLeftLane();
}

/**
 * @brief 计算车辆后轴中心到车辆右侧下一车道中心的距离.
 *
 * @return float 到车辆右侧下一车道中心的距离.
 */
float Vehicle::getDistanceToRightLaneCenter() const {
  return -0.5 * sOpt().road.lane_width + getDistanceToRightLane();
}

/**
 * @brief 根据 y 坐标计算车辆所在的车道
 *
 * @param position_Y 车辆的 y 坐标
 * @return int 车辆所在车道
 */
int Vehicle::getLane(const float position_Y) {
  return static_cast<int>(position_Y / sOpt().road.lane_width);
}

/**
 * @brief 根据轨迹更新车辆状态
 *
 * @param finalState
 */
void Vehicle::updateState(const std::vector<float>& finalState) {
  m_positionX     = finalState[0];
  m_positionY     = finalState[1];
  m_velocityX     = finalState[2];
  m_velocityY     = finalState[3];
  m_accelerationX = finalState[4];
  m_accelerationY = finalState[5];
  m_lane          = finalState[6];
  m_heading       = finalState[7];
}

/**
 * @brief 检查当前车辆状态是否有效
 *
 * @return true
 * @return false
 */
bool Vehicle::isValid() const { return isValid(m_positionY, m_heading); }

/**
 * @brief 检查车辆在给定的 y 位置和航向下是否处于有效状态，即整个车辆是否在道路边界内
 *
 * @param positionY 车辆的 y 位置
 * @param heading 车辆航向
 * @return true
 * @return false
 */
bool Vehicle::isValid(float positionY, float heading) const {
  /// 检查四个角点中是否有一个离开道路（仅横向分量相关）
  /// Front right
  if (!isOnRoad(positionY + m_length * std::sin(heading) - m_width / 2.0f * std::cos(heading)))
    return false;
  /// Back right
  if (!isOnRoad(positionY - m_width / 2.0f * std::cos(heading))) return false;
  /// Front left
  if (!isOnRoad(positionY + m_length * std::sin(heading) + m_width / 2.0f * std::cos(heading)))
    return false;
  /// Back left
  if (!isOnRoad(positionY + m_width / 2.0f * std::cos(heading))) return false;

  return true;
}

/**
 * @brief 确定动作（轨迹）是否有效。即它不违反最大加速度、速度或转向角等物理限制
 *
 * @param totalVelocity 车辆速度的大小
 * @param totalacceleration 车辆加速度的大小
 * @param steeringAngle 车辆的转向角度
 * @return bool 如果操作有效为true，否则为 false
 */
bool Vehicle::isValid(const float totalVelocity, const float totalacceleration,
                      const float steeringAngle) const {
  return std::abs(totalacceleration) < m_maxAcceleration && std::abs(totalVelocity) < m_maxSpeed &&
         std::abs(steeringAngle) < m_maxSteeringAngle;
}

/**
 * @brief 确定给定坐标 y 是否在道路上
 *
 * @param y 点的 y 坐标
 * @return true
 * @return false
 */
bool Vehicle::isOnRoad(const float y) {
  return y >= 0.0f && y <= sOpt().road.number_lanes * sOpt().road.lane_width;
}

/**
 * @brief 允许将 Vehicle 转换为 JSON 对象的函数
 * @details 由 nlohmann json 库的 json 构造函数调用
 *
 * @param j 需要填充的 JSON 对象
 * @param vehicle 待转换车辆
 */
void to_json(json& j, const Vehicle& vehicle) {
  j["position_x"]         = vehicle.m_positionX;
  j["position_y"]         = vehicle.m_positionY;
  j["velocity_x"]         = vehicle.m_velocityX;
  j["velocity_y"]         = vehicle.m_velocityY;
  j["acceleration_x"]     = vehicle.m_accelerationX;
  j["acceleration_y"]     = vehicle.m_accelerationY;
  j["lane"]               = vehicle.m_lane;
  j["heading"]            = vehicle.m_heading;
  j["yaw_rate"]           = vehicle.m_yawRate;
  j["width"]              = vehicle.m_width;
  j["length"]             = vehicle.m_length;
  j["wheel_base"]         = vehicle.m_wheelBase;
  j["max_steering_angle"] = vehicle.m_maxSteeringAngle;
  j["max_speed"]          = vehicle.m_maxSpeed;
  j["max_acceleration"]   = vehicle.m_maxAcceleration;
}
}  // namespace proseco_planning
