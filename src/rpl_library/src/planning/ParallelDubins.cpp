#include "rpl/planning/ParallelDubins.hpp"

#ifndef INCLUDE_ALGORITHM
#include <algorithm>
#define INCLUDE_ALGORITHM
#endif

#ifndef INCLUDE_CMATH
#include <cmath>
#define INCLUDE_CMATH
#endif

#ifndef INCLUDE_IMMINTRIN_H
#include <immintrin.h>
#define INCLUDE_IMMINTRIN_H
#endif

#include <iostream>

#ifndef RPL_COMMON_HPP
#include "rpl/common.hpp"
#endif

#ifndef RPL_INTERNAL_UTILS_HPP
#include "rpl/internal/utils.hpp"
#endif

#ifndef RPL_INTERNAL_GEOMETRY_HPP
#include "rpl/internal/geometry.hpp"
#endif

#ifndef RPL_INTERNAL_RPLINTRIN_HPP
#include "rpl/internal/rplintrin.hpp"
#endif

#ifndef RPL_INTERNAL_PARALLEL_HPP
#include "rpl/internal/parallel.hpp"
#endif

struct rpl::ParallelDubins::DubinsScaled
{
  float phi    = 0.F;
  float kappa  = 0.F;
  float theta0 = 0.F;
};

struct rpl::ParallelDubins::DubinsStandard
{
  Point start;
  Point end;
  float theta0 = 0.F;
};

struct rpl::ParallelDubins::DubinsInternal
{
  float cos_theta0                           = 0.F;
  float sin_theta0                           = 0.F;
  alignas(16) float *cos_thetaf              = nullptr;
  alignas(16) float *sin_thetaf              = nullptr;
  alignas(16) float *cos_theta0_minus_thetaf = nullptr;

  DubinsInternal()
  {
    this->cos_thetaf              = new float[settings::granularity()]();
    this->sin_thetaf              = new float[settings::granularity()]();
    this->cos_theta0_minus_thetaf = new float[settings::granularity()]();
  }
  DubinsInternal(const DubinsInternal &) = delete;
  DubinsInternal(DubinsInternal &&)      = delete;
  ~DubinsInternal()
  {
    delete[] this->cos_thetaf;
    delete[] this->sin_thetaf;
    delete[] this->cos_theta0_minus_thetaf;
  }
  DubinsInternal &operator=(const DubinsInternal &) = delete;
  DubinsInternal &operator=(DubinsInternal &&) = delete;
};

rpl::ParallelDubins::ParallelDubins(std::vector<Polygon> obstacles)
{
  const std::size_t granularity = settings::granularity();
  this->d_thetaf                = new float[granularity]();
  this->d_segmentSum            = new float[granularity * 6 + 1]();
  this->d_segmentSum[0]         = ops::cvt_hex_to_f32(0x7F800000);
  for (auto &subarray : this->d_segmentLengthsMatrix)
    subarray = new float[granularity * 6]();
  this->d_obstacles = std::move(obstacles);
}

rpl::ParallelDubins::~ParallelDubins()
{
  this->deallocate_all();
}

void rpl::ParallelDubins::deallocate_all()
{
  delete[] this->d_thetaf;
  delete[] this->d_segmentSum;
  for (auto &subarray : this->d_segmentLengthsMatrix)
    delete[] subarray;
}

void rpl::ParallelDubins::execute(const Point &start, const Point &end, const float &theta0)
{
  const float    inf = ops::cvt_hex_to_f32(0x7F800000);
  DubinsStandard standard;
  DubinsScaled   scaled;
  DubinsInternal internal;
  // TODO: for loop over all waypoints
  standard = {start, end, theta0};
  // preprocess data
  this->cvt_standard_to_scaled(standard, scaled);
  this->cvt_scaled_to_internal(scaled, internal);
  // fill segment matrix
  this->lsl(scaled, internal);
  this->rsr(scaled, internal);
  this->lsr(scaled, internal);
  this->rsl(scaled, internal);
  this->rlr(scaled, internal);
  this->lrl(scaled, internal);
  // sum
  this->compute_segment_sum();
  std::size_t res = this->find_minimum_segment();
  // while

  // TODO: while res is valid and collision happens
  while (res != 0 && this->collision_detection(res, {start, 0.f}))
  {
    // if collision happens
    this->d_segmentSum[res] = inf;
    // set the index in segment sum to infinity and repeat find_minimum_segment
    res = this->find_minimum_segment();
  }
}

void rpl::ParallelDubins::cvt_standard_to_scaled(const DubinsStandard &standard, DubinsScaled &scaled)
{
  float dx       = standard.end.x - standard.start.x;
  float dy       = standard.end.y - standard.start.y;
  scaled.phi     = atan2f(dy, dx);
  this->d_lambda = 0.5F * sqrtf(dx * dx + dy * dy);
  scaled.theta0  = utils::mod2pi(standard.theta0 - scaled.phi);
  scaled.kappa   = settings::kappa() * this->d_lambda;

  const std::size_t granularity = settings::granularity();
  const float       step        = settings::interp_step();
  float             theta       = 0.F;

  for (std::size_t i = 0; i < granularity; ++i)
  {
    this->d_thetaf[i] = utils::mod2pi(theta - scaled.phi);
    theta += step;
  }
}

void rpl::ParallelDubins::cvt_scaled_to_internal(const DubinsScaled &scaled, DubinsInternal &internal) const
{
  internal.cos_theta0 = cosf(scaled.theta0);
  internal.sin_theta0 = sinf(scaled.theta0);

  const std::size_t granularity = settings::granularity();
  const __m128      theta0      = _mm_set1_ps(scaled.theta0);
  __m128            thetaf;

  for (std::size_t i = 0; i < granularity; i += 4)
  {
    thetaf = _mm_load_ps(this->d_thetaf + i);
    _mm_store_ps(internal.cos_thetaf + i, parallel::cos(thetaf));
    _mm_store_ps(internal.sin_thetaf + i, parallel::sin(thetaf));
    _mm_store_ps(internal.cos_theta0_minus_thetaf + i, parallel::cos(_mm_sub_ps(theta0, thetaf)));
  }
}

void rpl::ParallelDubins::lsl(const DubinsScaled &scaled, const DubinsInternal &internal)
{
  const std::size_t granularity = settings::granularity();
  const __m128      kappa       = _mm_set1_ps(scaled.kappa);
  const __m128      inv_kappa   = _mm_set1_ps(this->d_lambda / scaled.kappa);
  const __m128      cos_theta0  = _mm_set1_ps(internal.cos_theta0);
  const __m128      sin_theta0  = _mm_set1_ps(internal.sin_theta0);
  const __m128      theta0      = _mm_set1_ps(scaled.theta0);
  const __m128      two         = _mm_set1_ps(2.F);
  const __m128      four        = _mm_set1_ps(4.F);

  __m128 C, S, s1, s2, s3;
  __m128 cos_thetaf, sin_thetaf, thetaf, atan2_CS, cos_theta0_minus_thetaf;

  alignas(16) float *s1_vec = this->d_segmentLengthsMatrix[0];
  alignas(16) float *s2_vec = this->d_segmentLengthsMatrix[1];
  alignas(16) float *s3_vec = this->d_segmentLengthsMatrix[2];

  for (std::size_t i = 0; i < granularity; i += 4)
  {
    thetaf                  = _mm_load_ps(this->d_thetaf + i);
    cos_thetaf              = _mm_load_ps(internal.cos_thetaf + i);
    sin_thetaf              = _mm_load_ps(internal.sin_thetaf + i);
    cos_theta0_minus_thetaf = _mm_load_ps(internal.cos_theta0_minus_thetaf + i);

    C        = _mm_sub_ps(cos_thetaf, cos_theta0);
    S        = _mm_add_ps(_mm_mul_ps(two, kappa), _mm_sub_ps(sin_theta0, sin_thetaf));
    atan2_CS = parallel::atan2(C, S);

    s1 = parallel::mod2pi(_mm_sub_ps(atan2_CS, theta0));
    s2 = _mm_sqrt_ps(_mm_add_ps(two,
                                _mm_add_ps(_mm_mul_ps(four, _mm_mul_ps(kappa, kappa)),
                                           _mm_sub_ps(_mm_mul_ps(four, _mm_mul_ps(kappa, _mm_sub_ps(sin_theta0, sin_thetaf))),
                                                      _mm_mul_ps(two, cos_theta0_minus_thetaf)))));
    s3 = parallel::mod2pi(_mm_sub_ps(thetaf, atan2_CS));

    _mm_store_ps(s1_vec + i, _mm_mul_ps(s1, inv_kappa));
    _mm_store_ps(s2_vec + i, _mm_mul_ps(s2, inv_kappa));
    _mm_store_ps(s3_vec + i, _mm_mul_ps(s3, inv_kappa));
  }
}

void rpl::ParallelDubins::rsr(const DubinsScaled &scaled, const DubinsInternal &internal)
{
  const std::size_t granularity = settings::granularity();
  const __m128      kappa       = _mm_set1_ps(scaled.kappa);
  const __m128      inv_kappa   = _mm_set1_ps(this->d_lambda / scaled.kappa);
  const __m128      cos_theta0  = _mm_set1_ps(internal.cos_theta0);
  const __m128      sin_theta0  = _mm_set1_ps(internal.sin_theta0);
  const __m128      theta0      = _mm_set1_ps(scaled.theta0);
  const __m128      two         = _mm_set1_ps(2.F);
  const __m128      four        = _mm_set1_ps(4.F);

  __m128 C, S, s1, s2, s3;
  __m128 cos_thetaf, sin_thetaf, thetaf, atan2_CS, cos_theta0_minus_thetaf;

  alignas(16) float *s1_vec = this->d_segmentLengthsMatrix[0];
  alignas(16) float *s2_vec = this->d_segmentLengthsMatrix[1];
  alignas(16) float *s3_vec = this->d_segmentLengthsMatrix[2];

  for (std::size_t i = 0; i < granularity; i += 4)
  {
    thetaf                  = _mm_load_ps(this->d_thetaf + i);
    cos_thetaf              = _mm_load_ps(internal.cos_thetaf + i);
    sin_thetaf              = _mm_load_ps(internal.sin_thetaf + i);
    cos_theta0_minus_thetaf = _mm_load_ps(internal.cos_theta0_minus_thetaf + i);

    C        = _mm_sub_ps(cos_theta0, cos_thetaf);
    S        = _mm_add_ps(_mm_mul_ps(two, kappa), _mm_sub_ps(sin_thetaf, sin_theta0));
    atan2_CS = parallel::atan2(C, S);

    s1 = parallel::mod2pi(_mm_sub_ps(theta0, atan2_CS));
    s2 = _mm_sqrt_ps(_mm_add_ps(two,
                                _mm_sub_ps(_mm_mul_ps(four, _mm_mul_ps(kappa, kappa)),
                                           _mm_add_ps(
                                               _mm_mul_ps(two, cos_theta0_minus_thetaf), _mm_mul_ps(four, _mm_mul_ps(kappa, _mm_sub_ps(sin_theta0, sin_thetaf)))))));
    s3 = parallel::mod2pi(_mm_sub_ps(atan2_CS, thetaf));
    _mm_store_ps(s1_vec + 360 + i, _mm_mul_ps(s1, inv_kappa));
    _mm_store_ps(s2_vec + 360 + i, _mm_mul_ps(s2, inv_kappa));
    _mm_store_ps(s3_vec + 360 + i, _mm_mul_ps(s3, inv_kappa));
  }
}

void rpl::ParallelDubins::lsr(const DubinsScaled &scaled, const DubinsInternal &internal)
{
  const std::size_t granularity = settings::granularity();
  const __m128      kappa       = _mm_set1_ps(scaled.kappa);
  const __m128      inv_kappa   = _mm_set1_ps(this->d_lambda / scaled.kappa);
  const __m128      cos_theta0  = _mm_set1_ps(internal.cos_theta0);
  const __m128      sin_theta0  = _mm_set1_ps(internal.sin_theta0);
  const __m128      theta0      = _mm_set1_ps(scaled.theta0);
  const __m128      two         = _mm_set1_ps(2.F);
  const __m128      four        = _mm_set1_ps(4.F);
  const __m128      zero        = _mm_setzero_ps();

  __m128 C, S, s1, s2, s3;
  __m128 cos_thetaf, sin_thetaf, thetaf, atan2_CS, atan2_2s2, cos_theta0_minus_thetaf;

  alignas(16) float *s1_vec = this->d_segmentLengthsMatrix[0];
  alignas(16) float *s2_vec = this->d_segmentLengthsMatrix[1];
  alignas(16) float *s3_vec = this->d_segmentLengthsMatrix[2];
  for (std::size_t i = 0; i < granularity; i += 4)
  {
    thetaf                  = _mm_load_ps(this->d_thetaf + i);
    cos_thetaf              = _mm_load_ps(internal.cos_thetaf + i);
    sin_thetaf              = _mm_load_ps(internal.sin_thetaf + i);
    cos_theta0_minus_thetaf = _mm_load_ps(internal.cos_theta0_minus_thetaf + i);

    C        = _mm_add_ps(cos_theta0, cos_thetaf);
    S        = _mm_add_ps(_mm_mul_ps(two, kappa), _mm_add_ps(sin_theta0, sin_thetaf));
    atan2_CS = parallel::atan2(_mm_sub_ps(zero, C), S);

    s2        = _mm_sqrt_ps(_mm_sub_ps(
        _mm_add_ps(_mm_mul_ps(four, _mm_mul_ps(kappa, kappa)),
                   _mm_add_ps(_mm_mul_ps(two, cos_theta0_minus_thetaf),
                              _mm_mul_ps(four,
                                         _mm_mul_ps(kappa,
                                                    _mm_add_ps(sin_theta0, sin_thetaf))))),
        two));
    atan2_2s2 = parallel::atan2(_mm_sub_ps(zero, two), s2);
    s1        = parallel::mod2pi(_mm_sub_ps(atan2_CS, _mm_add_ps(atan2_2s2, theta0)));
    s3        = parallel::mod2pi(_mm_sub_ps(atan2_CS, _mm_add_ps(atan2_2s2, thetaf)));
    _mm_store_ps(s1_vec + 360 * 2 + i, _mm_mul_ps(s1, inv_kappa));
    _mm_store_ps(s2_vec + 360 * 2 + i, _mm_mul_ps(s2, inv_kappa));
    _mm_store_ps(s3_vec + 360 * 2 + i, _mm_mul_ps(s3, inv_kappa));
  }
}

void rpl::ParallelDubins::rsl(const DubinsScaled &scaled, const DubinsInternal &internal)
{
  const std::size_t granularity = settings::granularity();
  const __m128      kappa       = _mm_set1_ps(scaled.kappa);
  const __m128      inv_kappa   = _mm_set1_ps(this->d_lambda / scaled.kappa);
  const __m128      cos_theta0  = _mm_set1_ps(internal.cos_theta0);
  const __m128      sin_theta0  = _mm_set1_ps(internal.sin_theta0);
  const __m128      theta0      = _mm_set1_ps(scaled.theta0);
  const __m128      two         = _mm_set1_ps(2.F);
  const __m128      four        = _mm_set1_ps(4.F);

  __m128             C, S, s1, s2, s3;
  __m128             cos_thetaf, sin_thetaf, thetaf, atan2_CS, atan2_2s2, cos_theta0_minus_thetaf;
  alignas(16) float *s1_vec = this->d_segmentLengthsMatrix[0];
  alignas(16) float *s2_vec = this->d_segmentLengthsMatrix[1];
  alignas(16) float *s3_vec = this->d_segmentLengthsMatrix[2];

  for (std::size_t i = 0; i < granularity; i += 4)
  {
    thetaf                  = _mm_load_ps(this->d_thetaf + i);
    cos_thetaf              = _mm_load_ps(internal.cos_thetaf + i);
    sin_thetaf              = _mm_load_ps(internal.sin_thetaf + i);
    cos_theta0_minus_thetaf = _mm_load_ps(internal.cos_theta0_minus_thetaf + i);

    C        = _mm_add_ps(cos_theta0, cos_thetaf);
    S        = _mm_sub_ps(_mm_mul_ps(two, kappa), _mm_add_ps(sin_theta0, sin_thetaf));
    atan2_CS = parallel::atan2(C, S);
    s2       = _mm_sqrt_ps(_mm_sub_ps(_mm_add_ps(_mm_mul_ps(four, _mm_mul_ps(kappa, kappa)), _mm_mul_ps(two, cos_theta0_minus_thetaf)),
                                _mm_add_ps(two, _mm_mul_ps(four, _mm_mul_ps(kappa, _mm_add_ps(sin_theta0, sin_thetaf))))));

    atan2_2s2 = parallel::atan2(two, s2);
    s1        = parallel::mod2pi(_mm_add_ps(theta0, _mm_sub_ps(atan2_2s2, atan2_CS)));
    s3        = parallel::mod2pi(_mm_add_ps(thetaf, _mm_sub_ps(atan2_2s2, atan2_CS)));
    _mm_store_ps(s1_vec + 360 * 3 + i, _mm_mul_ps(s1, inv_kappa));
    _mm_store_ps(s2_vec + 360 * 3 + i, _mm_mul_ps(s2, inv_kappa));
    _mm_store_ps(s3_vec + 360 * 3 + i, _mm_mul_ps(s3, inv_kappa));
  }
}

void rpl::ParallelDubins::rlr(const DubinsScaled &scaled, const DubinsInternal &internal)
{
  const std::size_t granularity = settings::granularity();
  const __m128      kappa       = _mm_set1_ps(scaled.kappa);
  const __m128      inv_kappa   = _mm_set1_ps(this->d_lambda / scaled.kappa);
  const __m128      cos_theta0  = _mm_set1_ps(internal.cos_theta0);
  const __m128      sin_theta0  = _mm_set1_ps(internal.sin_theta0);
  const __m128      theta0      = _mm_set1_ps(scaled.theta0);
  const __m128      two         = _mm_set1_ps(2.F);
  const __m128      four        = _mm_set1_ps(4.F);
  const __m128      six         = _mm_set1_ps(6.F);
  const __m128      half        = _mm_set1_ps(0.5F);
  const __m128      one_over_8  = _mm_set1_ps(0.125F);
  const __m128      two_pi      = _mm_set1_ps(settings::M_2PI());

  __m128             C, S, s1, s2, s3;
  __m128             cos_thetaf, sin_thetaf, thetaf, atan2_CS, arccos, cos_theta0_minus_thetaf;
  alignas(16) float *s1_vec = this->d_segmentLengthsMatrix[0];
  alignas(16) float *s2_vec = this->d_segmentLengthsMatrix[1];
  alignas(16) float *s3_vec = this->d_segmentLengthsMatrix[2];

  for (std::size_t i = 0; i < granularity; i += 4)
  {
    thetaf                  = _mm_load_ps(this->d_thetaf + i);
    cos_thetaf              = _mm_load_ps(internal.cos_thetaf + i);
    sin_thetaf              = _mm_load_ps(internal.sin_thetaf + i);
    cos_theta0_minus_thetaf = _mm_load_ps(internal.cos_theta0_minus_thetaf + i);

    C        = _mm_sub_ps(cos_theta0, cos_thetaf);
    S        = _mm_add_ps(_mm_mul_ps(two, kappa), _mm_sub_ps(sin_thetaf, sin_theta0));
    atan2_CS = parallel::atan2(C, S);
    arccos   = _mm_mul_ps(one_over_8, _mm_sub_ps(_mm_add_ps(six, _mm_add_ps(
                                                                   _mm_mul_ps(two, cos_theta0_minus_thetaf),
                                                                   _mm_mul_ps(four, _mm_mul_ps(kappa, _mm_sub_ps(sin_theta0, sin_thetaf))))),
                                               _mm_mul_ps(four, _mm_mul_ps(kappa, kappa))));
    arccos   = parallel::acos(arccos);
    s2       = parallel::mod2pi(_mm_sub_ps(two_pi, arccos));
    s1       = parallel::mod2pi(_mm_sub_ps(_mm_add_ps(theta0, _mm_mul_ps(half, s2)), atan2_CS));
    s3       = parallel::mod2pi(_mm_add_ps(theta0, _mm_sub_ps(_mm_sub_ps(s2, s1), thetaf)));
    _mm_store_ps(s1_vec + 360 * 4 + i, _mm_mul_ps(s1, inv_kappa));
    _mm_store_ps(s2_vec + 360 * 4 + i, _mm_mul_ps(s2, inv_kappa));
    _mm_store_ps(s3_vec + 360 * 4 + i, _mm_mul_ps(s3, inv_kappa));
  }
}

void rpl::ParallelDubins::lrl(const DubinsScaled &scaled, const DubinsInternal &internal)
{
  const std::size_t granularity = settings::granularity();
  const __m128      kappa       = _mm_set1_ps(scaled.kappa);
  const __m128      inv_kappa   = _mm_set1_ps(this->d_lambda / scaled.kappa);
  const __m128      cos_theta0  = _mm_set1_ps(internal.cos_theta0);
  const __m128      sin_theta0  = _mm_set1_ps(internal.sin_theta0);
  const __m128      theta0      = _mm_set1_ps(scaled.theta0);
  const __m128      two         = _mm_set1_ps(2.F);
  const __m128      four        = _mm_set1_ps(4.F);
  const __m128      six         = _mm_set1_ps(6.F);
  const __m128      half        = _mm_set1_ps(0.5F);
  const __m128      one_over_8  = _mm_set1_ps(0.125F);
  const __m128      two_pi      = _mm_set1_ps(settings::M_2PI());

  __m128             C, S, s1, s2, s3;
  __m128             cos_thetaf, sin_thetaf, thetaf, atan2_CS, arccos, cos_theta0_minus_thetaf;
  alignas(16) float *s1_vec = this->d_segmentLengthsMatrix[0];
  alignas(16) float *s2_vec = this->d_segmentLengthsMatrix[1];
  alignas(16) float *s3_vec = this->d_segmentLengthsMatrix[2];

  for (std::size_t i = 0; i < granularity; i += 4)
  {
    thetaf                  = _mm_load_ps(this->d_thetaf + i);
    cos_thetaf              = _mm_load_ps(internal.cos_thetaf + i);
    sin_thetaf              = _mm_load_ps(internal.sin_thetaf + i);
    cos_theta0_minus_thetaf = _mm_load_ps(internal.cos_theta0_minus_thetaf + i);

    C        = _mm_sub_ps(cos_thetaf, cos_theta0);
    S        = _mm_sub_ps(_mm_add_ps(_mm_mul_ps(two, kappa), sin_theta0), sin_thetaf);
    atan2_CS = parallel::atan2(C, S);

    arccos = _mm_mul_ps(one_over_8, _mm_sub_ps(_mm_add_ps(six, _mm_mul_ps(two, cos_theta0_minus_thetaf)),
                                               _mm_add_ps(_mm_mul_ps(four, _mm_mul_ps(kappa, kappa)),
                                                          _mm_mul_ps(four, _mm_mul_ps(kappa, _mm_sub_ps(sin_theta0, sin_thetaf))))));
    arccos = parallel::acos(arccos);
    s2     = parallel::mod2pi(_mm_sub_ps(two_pi, arccos));
    s1     = parallel::mod2pi(_mm_sub_ps(_mm_add_ps(atan2_CS, _mm_mul_ps(half, s2)), theta0));
    s3     = parallel::mod2pi(_mm_sub_ps(_mm_add_ps(thetaf, _mm_sub_ps(s2, s1)), theta0));
    _mm_store_ps(s1_vec + 360 * 5 + i, _mm_mul_ps(s1, inv_kappa));
    _mm_store_ps(s2_vec + 360 * 5 + i, _mm_mul_ps(s2, inv_kappa));
    _mm_store_ps(s3_vec + 360 * 5 + i, _mm_mul_ps(s3, inv_kappa));
  }
}

void rpl::ParallelDubins::compute_segment_sum()
{
  alignas(16) float *dst    = this->d_segmentSum + 1;
  alignas(16) float *s1_vec = this->d_segmentLengthsMatrix[0];
  alignas(16) float *s2_vec = this->d_segmentLengthsMatrix[1];
  alignas(16) float *s3_vec = this->d_segmentLengthsMatrix[2];

  const std::size_t granularity_times_6 = 6 * settings::granularity();
  __m128            s1, s2, s3;
  // NOTE: I needed to use storeu cause the first value is infinity... always. This will grant that whenever the std::min_element is called, it does not get stuck on a nan
  for (std::size_t i = 0; i < granularity_times_6; i += 4)
  {
    s1 = _mm_load_ps(s1_vec + i);
    s2 = _mm_load_ps(s2_vec + i);
    s3 = _mm_load_ps(s3_vec + i);
    _mm_storeu_ps(dst + i, _mm_add_ps(s1, _mm_add_ps(s2, s3)));
  }
}

std::size_t rpl::ParallelDubins::find_minimum_segment() const
{
  const std::size_t size        = settings::granularity() * 6 + 1;
  auto *            min_element = std::min_element(this->d_segmentSum, this->d_segmentSum + size);
  std::size_t       index       = std::distance(this->d_segmentSum, min_element);
  return index;
}

bool rpl::ParallelDubins::collision_detection(const std::size_t &index, const Pose &start) const
{
  const std::size_t true_index = index - 1;
  const std::size_t manouever  = true_index / 360;
  // const std::size_t angle      = true_index - manouever * 360;
  const float s1 = this->d_segmentLengthsMatrix[0][index];
  const float s2 = this->d_segmentLengthsMatrix[1][index];
  const float s3 = this->d_segmentLengthsMatrix[2][index];
  bool        cond1, cond2, cond3;
  Pose        pose1, pose2, pose3, pose4;

  switch (manouever)
  {
    case 0:
    {
      // lsl
      std::cerr << "lsl\n";
      pose1 = start;
      pose2 = this->interpolate_single(pose1, s1, -1.F);
      pose3 = this->interpolate_single(pose2, s2, +0.F);
      pose4 = this->interpolate_single(pose3, s3, -1.F);
      // find circle center of first arc
      Circle circle1 = geometry::compute_circle_from_point(pose1, true);
      // find segment of second arc
      Segment segment2 = {pose2.point, pose3.point};
      // find circle center of third arc
      Circle                  circle3 = geometry::compute_circle_from_point(pose3, true);
      std::pair<float, float> range1, range3;
      range1.first  = geometry::intersection_angle(circle1, pose1.point);
      range1.second = geometry::intersection_angle(circle1, pose2.point);
      range3.first  = geometry::intersection_angle(circle3, pose3.point);
      range3.second = geometry::intersection_angle(circle3, pose4.point);

      Segment temp;
      for (const auto &polygon : this->d_obstacles)
      {
        for (std::size_t i = 0; i < polygon.size() - 1; ++i)
        {
          temp.start = polygon[i];
          temp.end   = polygon[i + 1];
          cond1      = geometry::arc_segment_intersection(circle1, temp, range1);
          cond2      = geometry::intersects(temp, segment2);
          cond3      = geometry::arc_segment_intersection(circle3, temp, range3);
          if (cond1 || cond2 || cond3) return true;
        }
        temp.start = polygon.front();
        temp.end   = polygon.back();
        cond1      = geometry::arc_segment_intersection(circle1, temp, range1);
        cond2      = geometry::intersects(temp, segment2);
        cond3      = geometry::arc_segment_intersection(circle3, temp, range3);
        if (cond1 || cond2 || cond3) return true;
      }
      // get intersection of circle
      // get intersection of line
      // get intersection of circle
    }
    break;
    case 1:
    {
      // rsr
      std::cerr << "rsr\n";
      pose1 = start;
      pose2 = this->interpolate_single(pose1, s1, +1.F);
      pose3 = this->interpolate_single(pose2, s2, +0.F);
      pose4 = this->interpolate_single(pose3, s3, +1.F);

      // find circle center of first arc
      Circle circle1 = geometry::compute_circle_from_point(pose1, false);
      // find segment of second arc
      Segment segment2 = {pose2.point, pose3.point};
      // find circle center of third arc
      Circle circle3 = geometry::compute_circle_from_point(pose3, false);

      std::pair<float, float> range1, range3;
      range1.first  = geometry::intersection_angle(circle1, pose1.point);
      range1.second = geometry::intersection_angle(circle1, pose2.point);
      range3.first  = geometry::intersection_angle(circle3, pose3.point);
      range3.second = geometry::intersection_angle(circle3, pose4.point);

      Segment temp;
      for (const auto &polygon : this->d_obstacles)
      {
        for (std::size_t i = 0; i < polygon.size() - 1; ++i)
        {
          temp.start = polygon[i];
          temp.end   = polygon[i + 1];
          cond1      = geometry::arc_segment_intersection(circle1, temp, range1);
          cond2      = geometry::intersects(temp, segment2);
          cond3      = geometry::arc_segment_intersection(circle3, temp, range3);
          if (cond1 || cond2 || cond3) return true;
        }
        temp.start = polygon.front();
        temp.end   = polygon.back();
        cond1      = geometry::arc_segment_intersection(circle1, temp, range1);
        cond2      = geometry::intersects(temp, segment2);
        cond3      = geometry::arc_segment_intersection(circle3, temp, range3);
        if (cond1 || cond2 || cond3) return true;
      }
      // get intersection of circle
      // get intersection of line
      // get intersection of circle
    }
    break;
    case 2:
    {
      // lsr
      std::cerr << "lsr\n";
      pose1 = start;
      pose2 = this->interpolate_single(pose1, s1, -1.F);
      pose3 = this->interpolate_single(pose2, s2, +0.F);
      pose4 = this->interpolate_single(pose3, s3, +1.F);

      // find circle center of first arc
      Circle circle1 = geometry::compute_circle_from_point(pose1, true);
      // find segment of second arc
      Segment segment2 = {pose2.point, pose3.point};
      // find circle center of third arc
      Circle circle3 = geometry::compute_circle_from_point(pose3, false);

      std::pair<float, float> range1, range3;
      range1.first  = geometry::intersection_angle(circle1, pose1.point);
      range1.second = geometry::intersection_angle(circle1, pose2.point);
      range3.first  = geometry::intersection_angle(circle3, pose3.point);
      range3.second = geometry::intersection_angle(circle3, pose4.point);

      Segment temp;
      for (const auto &polygon : this->d_obstacles)
      {
        for (std::size_t i = 0; i < polygon.size() - 1; ++i)
        {
          temp.start = polygon[i];
          temp.end   = polygon[i + 1];
          cond1      = geometry::arc_segment_intersection(circle1, temp, range1);
          cond2      = geometry::intersects(temp, segment2);
          cond3      = geometry::arc_segment_intersection(circle3, temp, range3);
          if (cond1 || cond2 || cond3) return true;
        }
        temp.start = polygon.front();
        temp.end   = polygon.back();
        cond1      = geometry::arc_segment_intersection(circle1, temp, range1);
        cond2      = geometry::intersects(temp, segment2);
        cond3      = geometry::arc_segment_intersection(circle3, temp, range3);
        if (cond1 || cond2 || cond3) return true;
      }
      // get intersection of circle
      // get intersection of line
      // get intersection of circle
    }
    break;
    case 3:
    {
      // rsl
      std::cerr << "rsl\n";
      pose1 = start;
      pose2 = this->interpolate_single(pose1, s1, +1.F);
      pose3 = this->interpolate_single(pose2, s2, +0.F);
      pose4 = this->interpolate_single(pose3, s3, -1.F);

      // find circle center of first arc
      Circle circle1 = geometry::compute_circle_from_point(pose1, false);
      // find segment of second arc
      Segment segment2 = {pose2.point, pose3.point};
      // find circle center of third arc
      Circle circle3 = geometry::compute_circle_from_point(pose3, true);

      std::pair<float, float> range1, range3;
      range1.first  = geometry::intersection_angle(circle1, pose1.point);
      range1.second = geometry::intersection_angle(circle1, pose2.point);
      range3.first  = geometry::intersection_angle(circle3, pose3.point);
      range3.second = geometry::intersection_angle(circle3, pose4.point);

      Segment temp;
      for (const auto &polygon : this->d_obstacles)
      {
        for (std::size_t i = 0; i < polygon.size() - 1; ++i)
        {
          temp.start = polygon[i];
          temp.end   = polygon[i + 1];
          cond1      = geometry::arc_segment_intersection(circle1, temp, range1);
          cond2      = geometry::intersects(temp, segment2);
          cond3      = geometry::arc_segment_intersection(circle3, temp, range3);
          if (cond1 || cond2 || cond3) return true;
        }
        temp.start = polygon.front();
        temp.end   = polygon.back();
        cond1      = geometry::arc_segment_intersection(circle1, temp, range1);
        cond2      = geometry::intersects(temp, segment2);
        cond3      = geometry::arc_segment_intersection(circle3, temp, range3);
        if (cond1 || cond2 || cond3) return true;
      }
      // get intersection of circle
      // get intersection of line
      // get intersection of circle
    }
    break;
    case 4:
    {
      // rlr
      std::cerr << "rlr\n";
      pose1 = start;
      pose2 = this->interpolate_single(pose1, s1, +1.F);
      pose3 = this->interpolate_single(pose2, s2, -1.F);
      pose4 = this->interpolate_single(pose3, s3, +1.F);

      // find circle center of first arc
      Circle circle1 = geometry::compute_circle_from_point(pose1, false);
      // find circle center of second arc
      Circle circle2 = geometry::compute_circle_from_point(pose2, true);
      // find circle center of third arc
      Circle circle3 = geometry::compute_circle_from_point(pose3, false);

      std::pair<float, float> range1, range2, range3;
      range1.first  = geometry::intersection_angle(circle1, pose1.point);
      range1.second = geometry::intersection_angle(circle1, pose2.point);
      range2.first  = geometry::intersection_angle(circle2, pose2.point);
      range2.second = geometry::intersection_angle(circle2, pose3.point);
      range3.first  = geometry::intersection_angle(circle3, pose3.point);
      range3.second = geometry::intersection_angle(circle3, pose4.point);

      Segment temp;
      for (const auto &polygon : this->d_obstacles)
      {
        for (std::size_t i = 0; i < polygon.size() - 1; ++i)
        {
          temp.start = polygon[i];
          temp.end   = polygon[i + 1];
          cond1      = geometry::arc_segment_intersection(circle1, temp, range1);
          cond2      = geometry::arc_segment_intersection(circle2, temp, range2);
          cond3      = geometry::arc_segment_intersection(circle3, temp, range3);
          if (cond1 || cond2 || cond3) return true;
        }
        temp.start = polygon.front();
        temp.end   = polygon.back();
        cond1      = geometry::arc_segment_intersection(circle1, temp, range1);
        cond2      = geometry::arc_segment_intersection(circle2, temp, range2);
        cond3      = geometry::arc_segment_intersection(circle3, temp, range3);
        if (cond1 || cond2 || cond3) return true;
      }
      // get intersection of circle
      // get intersection of circle
      // get intersection of circle
    }
    break;
    case 5:
    {
      // lrl
      std::cerr << "lrl\n";
      pose1 = start;
      pose2 = this->interpolate_single(pose1, s1, -1.F);
      pose3 = this->interpolate_single(pose2, s2, +1.F);
      pose4 = this->interpolate_single(pose3, s3, -1.F);

      // find circle center of first arc
      Circle circle1 = geometry::compute_circle_from_point(pose1, true);
      // find circle center of second arc
      Circle circle2 = geometry::compute_circle_from_point(pose2, false);
      // find circle center of third arc
      Circle circle3 = geometry::compute_circle_from_point(pose3, true);

      std::pair<float, float> range1, range2, range3;
      range1.first  = geometry::intersection_angle(circle1, pose1.point);
      range1.second = geometry::intersection_angle(circle1, pose2.point);
      range2.first  = geometry::intersection_angle(circle2, pose2.point);
      range2.second = geometry::intersection_angle(circle2, pose3.point);
      range3.first  = geometry::intersection_angle(circle3, pose3.point);
      range3.second = geometry::intersection_angle(circle3, pose4.point);

      Segment temp;
      for (const auto &polygon : this->d_obstacles)
      {
        for (std::size_t i = 0; i < polygon.size() - 1; ++i)
        {
          temp.start = polygon[i];
          temp.end   = polygon[i + 1];
          cond1      = geometry::arc_segment_intersection(circle1, temp, range1);
          cond2      = geometry::arc_segment_intersection(circle2, temp, range2);
          cond3      = geometry::arc_segment_intersection(circle3, temp, range3);
          if (cond1 || cond2 || cond3) return true;
        }
        temp.start = polygon.front();
        temp.end   = polygon.back();
        cond1      = geometry::arc_segment_intersection(circle1, temp, range1);
        cond2      = geometry::arc_segment_intersection(circle2, temp, range2);
        cond3      = geometry::arc_segment_intersection(circle3, temp, range3);
        if (cond1 || cond2 || cond3) return true;
      }
      // get intersection of circle
      // get intersection of circle
      // get intersection of circle
    }
    break;
    default:
    {
      std::cerr << "[ERROR] index refers to a unseen row in the matrix\n";
      exit(1);
    }
  }

  return false;
}

rpl::Pose rpl::ParallelDubins::interpolate_single(const Pose &start, const float &length, const float &kappa) const
{
  Pose  res;
  float arg   = 0.5F * settings::kappa() * kappa * length;
  res.point.x = start.point.x + length * utils::sinc(arg) * cosf(start.theta + arg);
  res.point.y = start.point.y + length * utils::sinc(arg) * sinf(start.theta + arg);
  res.theta   = utils::mod2pi(start.theta + 2.F * arg);
  return res;
}