/**
 * @file alias.h
 * @brief A collection of aliases for common types.
 * @copyright Copyright (c) 2021
 *
 */

#pragma once
#include <memory>
#include <vector>

namespace proseco_planning {

class Action;
/// ActionPtr 是指向动作的共享指针
using ActionPtr = std::shared_ptr<Action>;

/// ActionSet 是 ActionPtr 的向量
using ActionSet = std::vector<ActionPtr>;

/// ActionSetSequence 是 ActionSet 的向量
using ActionSetSequence = std::vector<ActionSet>;

}  // namespace proseco_planning
