/**
 * @file actionClass.h
 * @brief This file defines the ActionClass class.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

namespace proseco_planning {
/**
 * @brief 动作类是一个枚举，用于描述 ActionSpace 中的不同动作类
 */
enum class ActionClass : int {
  /// 动作类别未知
  NONE = -1,
  /// 代理动作类别没有显着变化
  DO_NOTHING = 0,
  /// 代理动作类别正在加速
  ACCELERATE = 1,
  /// 代理减速时的动作类别
  DECELERATE = 2,
  /// 代理动作类为左转加速
  CHANGE_LEFT_FAST = 3,
  /// 代理动作类为左转减速
  CHANGE_LEFT_SLOW = 4,
  /// 代理动作类为右转加速
  CHANGE_RIGHT_FAST = 5,
  /// 代理动作类为右转减速
  CHANGE_RIGHT_SLOW = 6,
  /// 代理动作类为左转
  CHANGE_LEFT = 11,
  /// 代理动作类为右转
  CHANGE_RIGHT = 12,
};
}  // namespace proseco_planning
