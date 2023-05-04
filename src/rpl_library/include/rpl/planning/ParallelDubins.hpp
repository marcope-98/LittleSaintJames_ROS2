#ifndef RPL_PLANNING_PARALLELDUBINS_HPP
#define RPL_PLANNING_PARALLELDUBINS_HPP

#ifndef INCLUDE_CSTDINT
#include <cstdint>
#define INCLUDE_CSTDINT
#endif

#ifndef RPL_TYPES_HPP
#include "rpl/types.hpp"
#endif

namespace rpl
{
  struct ParallelDubins
  {
    struct DubinsScaled;
    struct DubinsStandard;
    struct DubinsInternal;

    float d_lambda                    = 0.F;
    alignas(16) float *d_thetaf       = nullptr;
    float *              d_segmentSum = nullptr;
    std::vector<Polygon> d_obstacles;
    float *              d_segmentLengthsMatrix[3] = {nullptr, nullptr, nullptr};

    ParallelDubins() = default;
    explicit ParallelDubins(std::vector<Polygon> obstacles);
    ParallelDubins(const ParallelDubins &) = delete;
    ParallelDubins(ParallelDubins &&)      = delete;
    ~ParallelDubins();

    ParallelDubins &operator=(const ParallelDubins &) = delete;
    ParallelDubins &operator=(ParallelDubins &&) = delete;

    void execute(const Point &start, const Point &end, const float &theta0);

  private:
    void deallocate_all();

    void cvt_standard_to_scaled(const DubinsStandard &standard, DubinsScaled &scaled);
    void cvt_scaled_to_internal(const DubinsScaled &scaled, DubinsInternal &internal) const;

    void lsl(const DubinsScaled &scaled, const DubinsInternal &internal);
    void rsr(const DubinsScaled &scaled, const DubinsInternal &internal);
    void lsr(const DubinsScaled &scaled, const DubinsInternal &internal);
    void rsl(const DubinsScaled &scaled, const DubinsInternal &internal);
    void rlr(const DubinsScaled &scaled, const DubinsInternal &internal); // [unused]
    void lrl(const DubinsScaled &scaled, const DubinsInternal &internal); // [unused]

    void        compute_segment_sum();
    std::size_t find_minimum_segment() const;
    bool        collision_detection(const std::size_t &index, const Pose &start) const;
    Pose        interpolate_single(const Pose &start, const float &length, const float &kappa) const;
  };
} // namespace rpl

#endif