#ifndef RPL_RPLINTIN_HPP
#define RPL_RPLINTIN_HPP

#ifndef INCLUDE_CSTDINT
#include <cstdint>
#define INCLUDE_CSTDINT
#endif

#ifndef INCLUDE_LIMITS
#include <limits>
#define INCLUDE_LIMITS
#endif

namespace rpl
{
  struct ops
  {
  private:
    union cvt
    {
      float         f32;
      std::uint32_t u32;

      explicit cvt(const float &a) : f32(a) {}
    };

  public:
    static inline float cvt_hex_to_f32(const std::uint32_t &hex)
    {
      cvt temp(0.F);
      temp.u32 = hex;
      return temp.f32;
    }

    static inline float abs_f32(const float &value)
    {
      cvt a(value);
      a.u32 &= (0x7FFFFFFFU);
      return a.f32;
    }

    static inline bool cmpgt_f32(const float &a, const float &b)
    {
      const float epsilon = std::numeric_limits<float>::epsilon();
      const float fabs_a  = abs_f32(a);
      const float fabs_b  = abs_f32(b);
      return (a - b) > ((fabs_a < fabs_b ? fabs_b : fabs_a) * epsilon);
    }
    static inline bool cmplt_f32(const float &a, const float &b)
    {
      const float epsilon = std::numeric_limits<float>::epsilon();
      const float fabs_a  = abs_f32(a);
      const float fabs_b  = abs_f32(b);
      return (b - a) > ((fabs_a < fabs_b ? fabs_b : fabs_a) * epsilon);
    }

    static inline bool cmpge_f32(const float &a, const float &b) { return !cmplt_f32(a, b); }
    static inline bool cmple_f32(const float &a, const float &b) { return !cmpgt_f32(a, b); }
    static inline bool cmpeq_f32(const float &a, const float &b) { return !(cmplt_f32(a, b) || cmpgt_f32(a, b)); }
  };
} // namespace rpl

#endif