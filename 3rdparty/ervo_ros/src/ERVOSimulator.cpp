/*
 * Simplified ERVO wrapper
 *
 * Copyright 2020 OMRON SINIC X
 * Mai Nishimura <mai.nishimura@sinicx.com> <denkivvakame@gmail.com>
 */

#include "ERVOSimulator.h"

#include "Agent.h"
#include "KdTree.h"

namespace RVO {

void ERVOSimulator::doStep(const Vector2 &point, float r) {
  kdTree_->buildAgentTree();

#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
    agents_[i]->computeNeighbors();
    agents_[i]->computeNewVelocityForERVO(point, r);
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
