#include "rpl/planning/Dubins.hpp"

#ifndef INCLUDE_ALGORITHM
#include <algorithm>
#define INCLUDE_ALGORITHM
#endif

#ifndef INCLUDE_IMMINTRIN_H
#include <immintrin.h>
#define INCLUDE_IMMINTRIN_H
#endif

#ifndef RPL_COMMON_HPP
#include "rpl/common.hpp"
#endif

#ifndef RPL_INTERNAL_PARALLEL_HPP
#include "rpl/internal/parallel.hpp"
#endif

#ifndef RPL_INTERNAL_RPLINTRIN_HPP
#include "rpl/internal/rplintrin.hpp"
#endif

#ifndef RPL_INTERNAL_UTILS_HPP
#include "rpl/internal/utils.hpp"
#endif

// TODO: change primitive functions to extract constants
// TODO: ideally i would like to eliminate ccc primitives
// TODO: find bit representation to discriminate start and end c primitives

struct rpl::Dubins::DubinsScaled
{
  float kappa               = 0.F;
  float theta0              = 0.F;
  alignas(16) float *thetaf = nullptr;

  DubinsScaled() { this->thetaf = new float[settings::granularity()](); }
  DubinsScaled(const DubinsScaled &) = delete;
  DubinsScaled(DubinsScaled &&)      = delete;
  ~DubinsScaled() { delete[] this->thetaf; }

  DubinsScaled &operator=(const DubinsScaled &) = delete;
  DubinsScaled &operator=(DubinsScaled &&) = delete;
};

struct rpl::Dubins::DubinsStandard
{
  Point start;
  Point end;
  float theta0 = 0.F;
};

struct rpl::Dubins::DubinsInternal
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

rpl::Dubins::Dubins()
{
  const std::size_t granularity = settings::granularity();
  this->p_segmentSum            = new float[granularity * 4 + 1]();
  this->p_segmentSum[0]         = ops::cvt_hex_to_f32(0x7F800000);
  for (std::size_t i = 0; i < 3; ++i)
    this->p_segmentLengthsMatrix[i] = new float[granularity * 4]();
}

rpl::Dubins::~Dubins() { this->deallocate_all(); }

void rpl::Dubins::deallocate_all()
{
  delete[] this->p_segmentSum;
  for (std::size_t i = 0; i < 3; ++i)
    delete[] this->p_segmentLengthsMatrix[i];
}

void rpl::Dubins::cvt_standard_to_scaled(const DubinsStandard &standard, DubinsScaled &scaled)
{
  float dx       = standard.end.x - standard.start.x;
  float dy       = standard.end.y - standard.start.y;
  float phi      = atan2f(dy, dx);
  this->d_lambda = 0.5F * sqrtf(dx * dx + dy * dy);
  scaled.theta0  = utils::mod2pi(standard.theta0 - phi);
  scaled.kappa   = settings::kappa() * this->d_lambda;

  // TODO: this can be done in parallel
  const std::size_t granularity = settings::granularity();
  const float       step        = settings::angle_step();
  float             theta       = -phi;

  for (std::size_t i = 0; i < granularity; ++i)
  {
    scaled.thetaf[i] = utils::mod2pi(theta);
    theta += step;
  }
}

void rpl::Dubins::cvt_scaled_to_internal(const DubinsScaled &scaled, DubinsInternal &internal) const
{
  internal.cos_theta0           = cosf(scaled.theta0);
  internal.sin_theta0           = sinf(scaled.theta0);
  const std::size_t granularity = settings::granularity();
  const __m128      theta0      = _mm_set1_ps(scaled.theta0);
  __m128            thetaf;

  for (std::size_t i = 0; i < granularity; i += 4)
  {
    thetaf = _mm_load_ps(scaled.thetaf + i);
    _mm_store_ps(internal.cos_thetaf + i, parallel::cos(thetaf));
    _mm_store_ps(internal.sin_thetaf + i, parallel::sin(thetaf));
    _mm_store_ps(internal.cos_theta0_minus_thetaf + i, parallel::cos(_mm_sub_ps(theta0, thetaf)));
  }
}

void rpl::Dubins::rsr(const DubinsScaled &scaled, const DubinsInternal &internal)
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

  alignas(16) float *s1_vec = this->p_segmentLengthsMatrix[0];
  alignas(16) float *s2_vec = this->p_segmentLengthsMatrix[1];
  alignas(16) float *s3_vec = this->p_segmentLengthsMatrix[2];

  for (std::size_t i = 0; i < granularity; i += 4)
  {
    thetaf                  = _mm_load_ps(scaled.thetaf + i);
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
    _mm_store_ps(s1_vec + i, _mm_mul_ps(s1, inv_kappa));
    _mm_store_ps(s2_vec + i, _mm_mul_ps(s2, inv_kappa));
    _mm_store_ps(s3_vec + i, _mm_mul_ps(s3, inv_kappa));
  }
}

void rpl::Dubins::rsl(const DubinsScaled &scaled, const DubinsInternal &internal)
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
  alignas(16) float *s1_vec = this->p_segmentLengthsMatrix[0];
  alignas(16) float *s2_vec = this->p_segmentLengthsMatrix[1];
  alignas(16) float *s3_vec = this->p_segmentLengthsMatrix[2];

  for (std::size_t i = 0; i < granularity; i += 4)
  {
    thetaf                  = _mm_load_ps(scaled.thetaf + i);
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
    _mm_store_ps(s1_vec + 360 + i, _mm_mul_ps(s1, inv_kappa));
    _mm_store_ps(s2_vec + 360 + i, _mm_mul_ps(s2, inv_kappa));
    _mm_store_ps(s3_vec + 360 + i, _mm_mul_ps(s3, inv_kappa));
  }
}

void rpl::Dubins::lsr(const DubinsScaled &scaled, const DubinsInternal &internal)
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

  alignas(16) float *s1_vec = this->p_segmentLengthsMatrix[0];
  alignas(16) float *s2_vec = this->p_segmentLengthsMatrix[1];
  alignas(16) float *s3_vec = this->p_segmentLengthsMatrix[2];
  for (std::size_t i = 0; i < granularity; i += 4)
  {
    thetaf                  = _mm_load_ps(scaled.thetaf + i);
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

void rpl::Dubins::lsl(const DubinsScaled &scaled, const DubinsInternal &internal)
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

  alignas(16) float *s1_vec = this->p_segmentLengthsMatrix[0];
  alignas(16) float *s2_vec = this->p_segmentLengthsMatrix[1];
  alignas(16) float *s3_vec = this->p_segmentLengthsMatrix[2];

  for (std::size_t i = 0; i < granularity; i += 4)
  {
    thetaf                  = _mm_load_ps(scaled.thetaf + i);
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

    _mm_store_ps(s1_vec + 360 * 3 + i, _mm_mul_ps(s1, inv_kappa));
    _mm_store_ps(s2_vec + 360 * 3 + i, _mm_mul_ps(s2, inv_kappa));
    _mm_store_ps(s3_vec + 360 * 3 + i, _mm_mul_ps(s3, inv_kappa));
  }
}

void rpl::Dubins::compute_segment_sum()
{
  alignas(16) float *dst    = this->p_segmentSum + 1;
  alignas(16) float *s1_vec = this->p_segmentLengthsMatrix[0];
  alignas(16) float *s2_vec = this->p_segmentLengthsMatrix[1];
  alignas(16) float *s3_vec = this->p_segmentLengthsMatrix[2];

  const std::size_t granx4 = 4 * settings::granularity();
  __m128            s1, s2, s3;
  for (std::size_t i = 0; i < granx4; i += 4)
  {
    s1 = _mm_load_ps(s1_vec + i);
    s2 = _mm_load_ps(s2_vec + i);
    s3 = _mm_load_ps(s3_vec + i);
    _mm_storeu_ps(dst + i, _mm_add_ps(s1, _mm_add_ps(s2, s3)));
  }
}

std::size_t rpl::Dubins::find_minimum_segment() const
{
  const std::size_t size        = settings::granularity() * 4 + 1;
  auto *            min_element = std::min_element(this->p_segmentSum, this->p_segmentSum + size);
  std::size_t       index       = std::distance(this->p_segmentSum, min_element);
  return index;
}

void rpl::Dubins::interpolate(const std::size_t &index, std::vector<Point> &out) const
{
  const float       step     = settings::interp_step();
  const std::size_t maneuver = (index - 1) / 360;

  const float s1 = this->p_segmentLengthsMatrix[0][index - 1];
  const float s2 = this->p_segmentLengthsMatrix[1][index - 1];
  const float s3 = this->p_segmentLengthsMatrix[2][index - 1];

  const auto        n_points_s1 = std::size_t(ceilf(s1 / step));
  const auto        n_points_s2 = std::size_t(ceilf(s2 / step));
  const auto        n_points_s3 = std::size_t(ceilf(s3 / step));
  const std::size_t n_points    = n_points_s1 + n_points_s2 + n_points_s3;
  out.reserve(n_points);

  for (std::size_t i = 0; i < n_points; ++i)
  {
    out.emplace_back(Point{0.F, 0.F}); // TODO: change this
  }
}