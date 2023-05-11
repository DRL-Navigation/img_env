/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Enrique Fernández
 */

#include <algorithm>

#include "speed_limit.h"

template<typename T>
T clamp(T x, T min, T max)
{
  return std::min(std::max(min, x), max);
}

template<typename T>
int sign(T x)
{
  return x == 0 ? 0 : x / fabs(x);
}


  SpeedLimiter::SpeedLimiter(comn_pkg::SpeedLimiter speed_limiter){
    has_velocity_limits = speed_limiter.has_velocity_limits;
    has_acceleration_limits = speed_limiter.has_acceleration_limits;
    has_jerk_limits = speed_limiter.has_jerk_limits;
    max_velocity = speed_limiter.max_velocity;
    min_velocity = speed_limiter.min_velocity;
    max_acceleration = speed_limiter.max_acceleration;
    min_acceleration = speed_limiter.min_acceleration;
    max_jerk = speed_limiter.min_jerk;
  }

  SpeedLimiter::SpeedLimiter(
    bool has_velocity_limits,
    bool has_acceleration_limits,
    bool has_jerk_limits,
    double min_velocity,
    double max_velocity,
    double min_acceleration,
    double max_acceleration,
    double min_jerk,
    double max_jerk
  )
  : has_velocity_limits(has_velocity_limits)
  , has_acceleration_limits(has_acceleration_limits)
  , has_jerk_limits(has_jerk_limits)
  , min_velocity(min_velocity)
  , max_velocity(max_velocity)
  , min_acceleration(min_acceleration)
  , max_acceleration(max_acceleration)
  , min_jerk(min_jerk)
  , max_jerk(max_jerk)
  {
  }



  double SpeedLimiter::limit(double& v, double v0, double v1, double dt)
  {
    const double tmp = v;

    limit_jerk(v, v0, v1, dt);
    limit_acceleration(v, v0, dt);
    limit_velocity(v);

    return tmp != 0.0 ? v / tmp : 1.0;
  }

  double SpeedLimiter::limit_velocity(double& v)
  {
    const double tmp = v;

    if (has_velocity_limits)
    {
      v = clamp(v, min_velocity, max_velocity);
    }

    return tmp != 0.0 ? v / tmp : 1.0;
  }

  double SpeedLimiter::limit_acceleration(double& v, double v0, double dt)
  {
    const double tmp = v;

    if (has_acceleration_limits)
    {
      const int v_sign = sign(v);
      const int v0_sign = sign(v0);
      if (v_sign + v0_sign != 0)
      {
        const double dv_min = min_acceleration * dt;
        const double dv_max = max_acceleration * dt;
        double dv = v - v0;
        const int dv_sign = sign(dv);
        if (dv_sign == v0_sign || dv_sign == v_sign)
          dv = dv_sign * clamp(fabs(dv), dv_min, dv_max);
        else
          dv = dv_sign * fabs(clamp(-fabs(dv), dv_min, dv_max));
        v = v0 + dv;
      }
      else
      {
        const double zero_dt = fabs(v0 / min_acceleration);
        if (zero_dt >= dt)
          v = v0_sign * (fabs(v0) - fabs(min_acceleration) * dt);
        else
        {
          const double v_dt = fabs(v / max_acceleration);
          if (zero_dt + v_dt >= dt)
            v = v_sign * fabs(max_acceleration * (dt - zero_dt));
          else
            v = tmp;
        }
      }
    }
    return tmp != 0.0 ? v / tmp : 1.0;
  }

  double SpeedLimiter::limit_jerk(double& v, double v0, double v1, double dt)
  {
    const double tmp = v;

    if (has_jerk_limits)
    {
      const double dv  = v  - v0;
      const double dv0 = v0 - v1;

      const double dt2 = 2. * dt * dt;

      const double da_min = min_jerk * dt2;
      const double da_max = max_jerk * dt2;

      const double da = clamp(dv - dv0, da_min, da_max);

      v = v0 + dv0 + da;
    }

    return tmp != 0.0 ? v / tmp : 1.0;
  }

