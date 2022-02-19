#ifndef _ervosimulator_
#define _ervosimulator_
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
  void doStep(const std::vector<Vector2> &points, std::vector<float> rs);
};

}  // namespace RVO

#endif