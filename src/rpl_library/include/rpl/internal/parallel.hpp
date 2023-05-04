#ifndef RPL_INTERNAL_PARALLEL_HPP
#define RPL_INTERNAL_PARALLEL_HPP

#ifndef INCLUDE_CSTDINT
#include <cstdint>
#define INCLUDE_CSTDINT
#endif

#ifndef INCLUDE_CMATH
#include <cmath>
#define INCLUDE_CMATH
#endif

#ifndef INCLUDE_IMMINTRIN_H
#include <immintrin.h>
#define INCLUDE_IMMINTRIN_H
#endif

#ifndef RPL_COMMON_HPP
#include "rpl/common.hpp"
#endif

namespace rpl
{
  struct parallel
  {
  private:
    union f32_ps
    {
      float  f32[4];
      __m128 ps;
      explicit f32_ps(const __m128 &ps_) : ps(ps_) {}
    };

  public:
    static __m128 mod2pi(const __m128 &angle)
    {
      __m128 twopi_ps = _mm_set1_ps(settings::M_2PI());
      __m128 temp     = _mm_div_ps(angle, twopi_ps);
      temp            = _mm_sub_ps(angle, _mm_mul_ps(twopi_ps, _mm_floor_ps(temp)));
      __m128 mask     = _mm_cmpeq_ps(temp, twopi_ps);
      return _mm_andnot_ps(mask, temp);
    }

    static __m128 cos(const __m128 &angle)
    {
      f32_ps res(angle);
      for (std::size_t i = 0; i < 4; ++i)
        res.f32[i] = cosf(res.f32[i]);
      return res.ps;
    }

    static __m128 sin(const __m128 &angle)
    {
      f32_ps res(angle);
      for (std::size_t i = 0; i < 4; ++i)
        res.f32[i] = sinf(res.f32[i]);
      return res.ps;
    }

    static __m128 acos(const __m128 &angle)
    {
      f32_ps res(angle);
      for (std::size_t i = 0; i < 4; ++i)
        res.f32[i] = acosf(res.f32[i]);
      return res.ps;
    }

    static __m128 atan2(const __m128 &y, const __m128 &x)
    {
      f32_ps res(y);
      f32_ps deltax(x);
      for (std::size_t i = 0; i < 4; ++i)
        res.f32[i] = atan2f(res.f32[i], deltax.f32[i]);
      return res.ps;
    }
  };
} // namespace rpl

#endif