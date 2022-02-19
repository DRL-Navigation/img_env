/*
 * Simplified ERVO wrapper
 *
 * Copyright 2020 OMRON SINIC X
 * Mai Nishimura <mai.nishimura@sinicx.com> <denkivvakame@gmail.com>
 */

#include "RVOSimulator.h"

namespace RVO {

class ERVOSimulator : public RVOSimulator {
 public:
  using RVOSimulator::RVOSimulator;  // inherit constructor
  void doStep(const Vector2 &point, float r);
};

}  // namespace RVO

