#include "proseco_planning/trajectory/trajectory.h"

#include <cmath>
#include <numeric>

#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/math/mathlib.h"

namespace proseco_planning {

/** 指示是否执行完整轨迹（false）或仅执行一部分轨迹（true）的标志*/
bool Trajectory::useActionFraction{false};

/**
 * @brief 构造一个新的轨迹对象
 *
 * @param t0 轨迹的开始时间
 * @param initialHeading 车辆的初始航向
 */
Trajectory::Trajectory(const float t0, const float initialHeading) {
  // initialize characteristic values
  m_t0     = t0;
  m_t1     = t0 + cOpt().action_duration;
  m_t0_2   = m_t0 * m_t0;
  m_t1_2   = m_t1 * m_t1;
  m_nSteps = (m_t1 - m_t0) / cOpt().delta_t + 1;

  // 初始化 vectors
  m_time.reserve(m_nSteps);

  m_sPosition.resize(m_nSteps, 0);
  m_dPosition.resize(m_nSteps, 0);
  m_sVelocity.resize(m_nSteps, 0);
  m_dVelocity.resize(m_nSteps, 0);
  m_sAcceleration.resize(m_nSteps, 0);
  m_dAcceleration.resize(m_nSteps, 0);
  m_lane.resize(m_nSteps, 0);
  m_heading.resize(m_nSteps, 0);
  m_steeringAngle.resize(m_nSteps, 0);
  m_curvature.resize(m_nSteps, 0);
  m_totalVelocity.resize(m_nSteps, 0);
  m_totalAcceleration.resize(m_nSteps, 0);
}

/**
 * @brief 确定轨迹每一步的车道
 * @note 如果车辆在道路上（即后轴中心是否在道路上），则分配车道，否则分配车道 -1
 *
 */
void Trajectory::determineLane() {
  const auto roadWidth = sOpt().road.number_lanes * sOpt().road.lane_width;
  for (size_t i = 0; i <= getFractionIndex(); ++i) {
    if (m_dPosition[i] < 0 || m_dPosition[i] > roadWidth) {
      m_lane[i] = -1;
    } else {
      m_lane[i] = Vehicle::getLane(m_dPosition[i]);
    }
  }
}

/**
 * @brief 检查轨迹是否遵循所有物理约束
 *
 * @param vehicle 跟踪轨迹的车辆
 * @return true 如果轨迹遵循所有物理约束
 * @return false 否则
 */
bool Trajectory::isValidAction(const Vehicle& vehicle) const {
  for (size_t i = 0; i <= getFractionIndex(); ++i) {
    if (!vehicle.isValid(m_totalVelocity[i], m_totalAcceleration[i], m_steeringAngle[i]))
      return false;
  }
  return true;
}

/**
 * @brief 检查轨迹的所有状态是否有效
 *
 * @param vehicle 跟踪轨迹的车辆
 * @return true 如果所有状态均有效
 * @return false 否则
 */
bool Trajectory::isValidState(const Vehicle& vehicle) const {
  for (size_t i = 0; i <= getFractionIndex(); ++i) {
    if (!vehicle.isValid(m_dPosition[i], m_heading[i])) return false;
  }
  return true;
}

/**
 * @brief 确定轨迹起点和终点之间的车道变换量
 */
void Trajectory::determineLaneChange() { m_laneChange = m_lane.back() - m_lane.front(); }

/**
 * @brief 计算轨迹的平均速度
 */
void Trajectory::calculateAverageSpeed() {
  const auto fractionVector = math::getSubvectorFromVector(m_totalVelocity, 0, getFractionIndex());
  m_averageVelocity =
      std::accumulate(fractionVector.begin(), fractionVector.end(), 0.0f) / fractionVector.size();
}

/**
 * @brief 计算轨迹的平均绝对加速度
 */
void Trajectory::calculateAverageAbsoluteAcceleration() {
  const auto fractionVector =
      math::getSubvectorFromVector(m_totalAcceleration, 0, getFractionIndex());
  m_averageAbsoluteAcceleration = math::absSum(fractionVector) / fractionVector.size();
}

/**
 * @brief 获取执行轨迹的索引。基于动作执行分数参数和动作分数标志
 *
 * @return size_t
 */
size_t Trajectory::getFractionIndex() const {
  // index: 例如 0-20 -> m_nSteps=21
  // 50%: index = 10
  // 100%: index = 20
  return useActionFraction//是否使用动作分数                //动作执行分数
             ? cOpt().policy_options.policy_enhancements.action_execution_fraction * (m_nSteps - 1)
             : m_nSteps - 1;
}

/**
 * @brief 根据动作分数标志返回当前应用于此轨迹对象的动作执行分数参数
 *
 * @return float
 */
float Trajectory::getCurrentFraction() {
  // 当useFraction为false时，执行完整轨迹->返回1.0
  return useActionFraction ? cOpt().policy_options.policy_enhancements.action_execution_fraction
                           : 1.0f;
}

/**
 * @brief 允许将轨迹转换为 JSON 对象的函数
 * @details 由 nlohmann json 库的 json 构造函数调用
 *
 * @param j 需要填充的JSON对象
 * @param trajectory 需要转换的轨迹
 */
void to_json(json& j, const Trajectory& trajectory) {
  j["sPosition"]                   = trajectory.m_sPosition;
  j["dPosition"]                   = trajectory.m_dPosition;
  j["sVelocity"]                   = trajectory.m_sVelocity;
  j["dVelocity"]                   = trajectory.m_dVelocity;
  j["sAcceleration"]               = trajectory.m_sAcceleration;
  j["dAcceleration"]               = trajectory.m_dAcceleration;
  j["curvature"]                   = trajectory.m_curvature;
  j["lane"]                        = trajectory.m_lane;
  j["heading"]                     = trajectory.m_heading;
  j["steeringAngle"]               = trajectory.m_steeringAngle;
  j["laneChange"]                  = trajectory.m_laneChange;
  j["totalVelocity"]               = trajectory.m_totalVelocity;
  j["totalAcceleration"]           = trajectory.m_totalAcceleration;
  j["averageVelocity"]             = trajectory.m_averageVelocity;
  j["averageAbsoluteAcceleration"] = trajectory.m_averageAbsoluteAcceleration;
  j["cumSquaredAccelerationLon"]   = trajectory.m_cumSquaredAccelerationLon;
  j["cumSquaredAccelerationLat"]   = trajectory.m_cumSquaredAccelerationLat;
  j["finalState"]                  = trajectory.m_finalState;
  j["invalidAction"]               = trajectory.m_invalidAction;
  j["invalidState"]                = trajectory.m_invalidState;
  j["useActionFraction"]           = trajectory.useActionFraction;
}
}  // namespace proseco_planning
