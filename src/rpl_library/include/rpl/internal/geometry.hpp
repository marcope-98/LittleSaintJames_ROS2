#ifndef RPL_INTERNAL_GEOMETRY_HPP
#define RPL_INTERNAL_GEOMETRY_HPP

#ifndef USE_MATH_DEFINES
#define USE_MATH_DEFINES
#endif

#ifndef INCLUDE_CMATH
#include <cmath>
#define INCLUDE_CMATH
#endif

#ifndef INCLUDE_CSTDINT
#include <cstdint>
#define INCLUDE_CSTDINT
#endif

#ifndef INCLUDE_UTILITY
#include <utility>
#define INCLUDE_UTILITY
#endif

#ifndef RPL_COMMON_HPP
#include "rpl/common.hpp"
#endif

#ifndef RPL_TYPES_HPP
#include "rpl/types.hpp"
#endif

#ifndef RPL_INTERNAL_RPLINTRIN_HPP
#include "rpl/internal/rplintrin.hpp"
#endif

#ifndef RPL_INTERNAL_UTILS_HPP
#include "rpl/internal/utils.hpp"
#endif

namespace rpl
{
  struct geometry
  {
    static std::int32_t orientation(const Point &p0, const Point &p1, const Point &p2)
    {
      float res = ((p1.y - p0.y) * (p2.x - p1.x)) - ((p1.x - p0.x) * (p2.y - p1.y));
      return std::int32_t(ops::cmpgt_f32(res, 0.F)) - std::int32_t(ops::cmplt_f32(res, 0.F));
    }

    static bool intersects(const Segment &p, const Segment &q)
    {
      return (orientation(p.start, q.start, q.end) != orientation(p.end, q.start, q.end)) &&
             (orientation(p.start, p.end, q.start) != orientation(p.start, p.end, q.end));
    }

    static Point intersection_point(const Segment &segment, const float &t)
    {
      if (t > 1.5F)
      {
        const float inf = ops::cvt_hex_to_f32(0x7F800000);
        return {inf, inf};
      }
      Point res;
      res.x = segment.start.x + t * (segment.end.x - segment.start.x);
      res.y = segment.start.y + t * (segment.end.y - segment.start.y);
      return res;
    }

    static float intersection_angle(const Circle &circle, const Point &q)
    {
      return utils::mod2pi(atan2f(q.y - circle.center.y, q.x - circle.center.x));
    }

    static std::pair<float, float> circle_segment_intersection(const Circle &circle, const Segment &v)
    {
      const float a     = squared_norm(v);
      const float b     = (v.end.x - v.start.x) * (v.start.x - circle.center.x) + (v.end.y - v.start.y) * (v.start.y - circle.center.y);
      const float c     = squared_norm({v.start, circle.center}) - (circle.radius * circle.radius);
      float       Delta = sqrtf(b * b - a * c);
      float       t0    = (Delta - b) / a;
      float       t1    = (-Delta - b) / a;

      if (ops::cmpgt_f32(t0, 0.F) && ops::cmplt_f32(t0, 1.F) &&
          ops::cmpgt_f32(t1, 0.F) && ops::cmplt_f32(t1, 1.F))
        return {t0, t1};
      return {2.F, 2.F};
    }

    static bool arc_segment_intersection(const Circle &circle, const Segment &v, const std::pair<float, float> &range)
    {
      std::pair<float, float> res = circle_segment_intersection(circle, v);
      if (res.first > 1.F && res.second > 1.F) return false; // TODO: only need to check one

      Point ipoint1 = intersection_point(v, res.first);
      Point ipoint2 = intersection_point(v, res.second);
      float iangle1 = intersection_angle(circle, ipoint1);
      float iangle2 = intersection_angle(circle, ipoint2);

      if (range.first < range.second)
        return (range.first < iangle1 && iangle1 < range.second) ||
               (range.first < iangle2 && iangle2 < range.second);

      return (range.first < iangle1 && iangle1 < settings::M_2PI()) ||
             (0.F < iangle1 && iangle1 < range.second) ||
             (range.first < iangle2 && iangle2 < settings::M_2PI()) ||
             (0.F < iangle2 && iangle2 < range.second);
    }

    static Circle compute_circle_from_point(const Pose &pose, const bool &left_turn)
    {
      float       sign    = left_turn ? +1.F : -1.F;
      const float radius  = settings::min_turning_radius();
      const float normal  = pose.theta + sign * float(M_PI_2);
      float       point_x = pose.point.x + radius * cosf(normal);
      float       point_y = pose.point.y + radius * sinf(normal);

      return {{point_x, point_y}, radius};
    }

    static float segment_segment_intersection(const Segment &v1, const Segment &v2)
    {
      float det = (v2.end.x - v2.start.x) * (v1.start.y - v1.end.y) -
                  (v1.start.x - v1.end.x) * (v2.end.y - v2.start.y);
      if (ops::cmpeq_f32(det, 0.F)) return 2.F;
      float det_inv = 1.F / det;
      float t       = det_inv * ((v2.start.y - v2.end.y) * (v1.start.x - v2.start.x) + (v2.end.x - v2.start.x) * (v1.start.y - v2.start.y));
      float u       = det_inv * ((v1.start.y - v1.end.y) * (v1.start.x - v2.start.x) + (v1.end.x - v1.start.x) * (v1.start.y - v2.start.y));
      if (ops::cmpgt_f32(t, 0.F) && ops::cmplt_f32(t, 1.F) &&
          ops::cmpgt_f32(u, 0.F) && ops::cmplt_f32(u, 1.F))
        return t;
      return 2.F;
    }

    static float halfplane_intersection(const Point &origin, const Segment &vec)
    {
      float det = vec.start.y - vec.end.y;
      float u   = (vec.start.y - origin.y) / det;

      if (!ops::cmpeq_f32(det, 0.F) && ops::cmpgt_f32(u, 0.F) && ops::cmplt_f32(u, 1.F))
        return 0.5F;
      return 2.F;
    }

    static Point centroid(const Polygon &polygon)
    {
      Point      res          = {0.F, 0.F};
      const auto n_points_inv = 1.F / float(polygon.size());
      for (const auto &point : polygon)
      {
        res.x += point.x;
        res.y += point.y;
      }
      return {res.x * n_points_inv, res.y * n_points_inv};
    }

    static bool point_in_polygon(const Point &point, const Polygon &polygon)
    {
      std::size_t n    = polygon.size();
      std::size_t low  = 0;
      std::size_t high = n;
      std::size_t mid;

      do
      {
        mid = (low + high) >> 1;
        if (orientation(polygon[0], polygon[mid], point) == -1)
          low = mid;
        else
          high = mid;

      } while (low + 1 < high);

      if (low == 0 || high == n) return false;
      return orientation(polygon[low], polygon[high], point) == -1;
    }

    static float squared_norm(const Segment &v)
    {
      float diffx = v.end.x - v.start.x;
      float diffy = v.end.y - v.start.y;
      return (diffx * diffx) + (diffy * diffy);
    }

    static float norm(const Segment &v)
    {
      return sqrtf(squared_norm(v));
    }
  };

} // namespace rpl

#endif