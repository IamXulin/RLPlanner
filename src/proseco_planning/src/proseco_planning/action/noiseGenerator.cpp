
#include "proseco_planning/action/noiseGenerator.h"

#include <boost/math/policies/policy.hpp>
#include <boost/math/special_functions/fpclassify.hpp>
#include <memory>

#include "proseco_planning/action/action.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/math/mathlib.h"

namespace proseco_planning {

/**
 * @brief Constructs a new Noise Generator object
 *
 */
NoiseGenerator::NoiseGenerator() {
  m_distributionY =
      std::normal_distribution<float>(cOpt().action_noise.meanY, cOpt().action_noise.sigmaY);
  m_distributionVx =
      std::normal_distribution<float>(cOpt().action_noise.meanVx, cOpt().action_noise.sigmaVx);
  m_normalDensityY  = boost::math::normal_distribution<float>(cOpt().action_noise.meanY,
                                                             cOpt().action_noise.sigmaY);
  m_normalDensityVx = boost::math::normal_distribution<float>(cOpt().action_noise.meanVx,
                                                              cOpt().action_noise.sigmaVx);
}

/**
 * @brief 使用配置中高斯分布的噪声参数创建噪声动作
 *
 * @param action 变得有噪声的动作
 * @return ActionPtr 有噪音的动作
 */
ActionPtr NoiseGenerator::createNoisyAction(const ActionPtr& action) {
  float epsilonY{m_distributionY(math::Random::engine())};
  float epsilonVx{m_distributionVx(math::Random::engine())};
  float likelihoodY{boost::math::pdf(m_normalDensityY, epsilonY)};
  float likelihoodVx{boost::math::pdf(m_normalDensityVx, epsilonVx)};

  auto noisyAction = std::make_shared<Action>(action->m_velocityChange + epsilonVx,
                                              action->m_lateralChange + epsilonY);

  noisyAction->noise.m_likelihoodY   = likelihoodY;
  noisyAction->noise.m_likelihoodVx  = likelihoodVx;
  noisyAction->noise.m_muY           = action->m_lateralChange;
  noisyAction->noise.m_muVx          = action->m_velocityChange;
  noisyAction->noise.m_sigmaY        = cOpt().action_noise.sigmaY;
  noisyAction->noise.m_sigmaVx       = cOpt().action_noise.sigmaVx;
  noisyAction->m_selectionLikelihood = action->m_selectionLikelihood;
  return noisyAction;
}

/**
 * @brief 使用噪声生成器创建操作集的所有操作的噪声版本.
 *
 * @param actionSet 动作集
 * @return ActionSet 带有噪音的动作集
 */
ActionSet NoiseGenerator::createNoisyActions(const ActionSet& actionSet) {
  ActionSet noisyActionSet;
  for (const auto& action : actionSet) {
    noisyActionSet.push_back(createNoisyAction(action));
  }
  return noisyActionSet;
}
}  // namespace proseco_planning