#ifndef RPL_PLANNING_DUBINS_HPP
#define RPL_PLANNING_DUBINS_HPP

#ifndef INCLUDE_CSTDINT
#include <cstdint>
#define INCLUDE_CSTDINT
#endif

#ifndef RPL_TYPES_HPP
#include "rpl/types.hpp"
#endif

namespace rpl
{
  struct Dubins
  {
    struct DubinsScaled;
    struct DubinsStandard;
    struct DubinsInternal;

    // Member variables
    float  d_lambda                  = 0.F;
    float *p_segmentSum              = nullptr;
    float *p_segmentLengthsMatrix[3] = {nullptr, nullptr, nullptr};

    // Constructors
    Dubins();
    Dubins(const Dubins &) = delete;
    Dubins(Dubins &&)      = delete;
    ~Dubins();

    // Manipulators
    Dubins &operator=(const Dubins &) = delete;
    Dubins &operator=(Dubins &&) = delete;

  private:
    // Methods
    void deallocate_all();

    void cvt_standard_to_scaled(const DubinsStandard &standard, DubinsScaled &scaled);
    void cvt_scaled_to_internal(const DubinsScaled &scaled, DubinsInternal &internal) const;

    void rsr(const DubinsScaled &scaled, const DubinsInternal &internal); // NOTE: 0 0
    void rsl(const DubinsScaled &scaled, const DubinsInternal &internal); // NOTE: 0 1
    void lsr(const DubinsScaled &scaled, const DubinsInternal &internal); // NOTE: 1 0
    void lsl(const DubinsScaled &scaled, const DubinsInternal &internal); // NOTE: 1 1

    void        compute_segment_sum();
    std::size_t find_minimum_segment() const;
    void        interpolate(const std::size_t &index, std::vector<Point> &out) const;
  };
} // namespace rpl
#endif