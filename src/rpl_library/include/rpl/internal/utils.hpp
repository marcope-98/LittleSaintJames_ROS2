#ifndef RPL_INTERNAL_UTILS_HPP
#define RPL_INTERNAL_UTILS_HPP

#ifndef INCLUDE_CMATH
#include <cmath>
#define INCLUDE_CMATH
#endif

#ifndef RPL_COMMON_HPP
#include "rpl/common.hpp"
#endif

namespace rpl
{
  struct utils
  {
    static float mod2pi(const float &angle)
    {
      float temp = angle / settings::M_2PI();
      return (angle - settings::M_2PI() * floorf(temp));
    }

    static float sinc(const float &value)
    {
      const float one_over_120 = 1.F / 120.F;
      const float abs_value    = fabsf(value);
      const float value_2      = value * value;
      if (abs_value < 0.002F)
        return 1.F - one_over_120 * value_2 * (20.F - value_2);
      return sinf(value) / value;
    }
  };
} // namespace rpl
#endif