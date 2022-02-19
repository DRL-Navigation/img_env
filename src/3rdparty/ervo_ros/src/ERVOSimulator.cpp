
/*
 * Simplified ERVO wrapper
 *
 * Copyright 2020 OMRON SINIC X
 * Mai Nishimura <mai.nishimura@sinicx.com> <denkivvakame@gmail.com>
 */

#include "ERVOSimulator.h"

#include "Agent.h"
#include "KdTree.h"
#include<vector>

namespace RVO {
// https://github.com/denkiwakame/Python-ERVO, try to add multiple beeps.
void ERVOSimulator::doStep(const std::vector<Vector2> &points, std::vector<float> rs) {
  kdTree_->buildAgentTree();

#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
    agents_[i]->computeNeighbors();
    agents_[i]->computeNewVelocityForERVO(points, rs);
  }

#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
    agents_[i]->update();
  }

  globalTime_ += timeStep_;
}

}  // namespace RVO

