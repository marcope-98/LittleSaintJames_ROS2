#ifndef RPL_TYPES_HPP
#define RPL_TYPES_HPP

#ifndef INCLUDE_CMATH
#include <cmath>
#define INCLUDE_CMATH
#endif

#ifndef INCLUDE_CSTDDEF
#include <cstddef>
#define INCLUDE_CSTDDEF
#endif

#ifndef INCLUDE_VECTOR
#include <vector>
#define INCLUDE_VECTOR
#endif

namespace rpl
{
#if 1

  struct Point
  {
    float x = 0.F;
    float y = 0.F;
  };

  struct Pose
  {
    Point point;
    float theta = 0.F;
  };

  // TODO: define this later
  struct Path
  {
  };

  struct Circle
  {
    Point center;
    float radius = 0.F;
  };

  struct Segment
  {
    Point start;
    Point end;
  };

  using Polygon = std::vector<Point>;

#else
  // these are in utils.hpp
  struct Pose
  {
    float s, x, y, theta, kappa;

    Pose() : Pose(0, 0, 0, 0, 0) {}
    Pose(const float &s,
         const float &x,
         const float &y,
         const float &theta,
         const float &kappa) : s(s),
                               x(x),
                               y(y),
                               theta(theta),
                               kappa(kappa) {}

    float distance(const float &_x, const float &_y) const { return std::hypot(x - _x, y - _y); }
  };

  struct Path
  {
    std::vector<Pose> points;

    Path() = default;
    Path(std::vector<Pose> const &points) : points(points) {}

    bool        empty() const { return points.empty(); }
    std::size_t size() const { return points.size(); }
    void        setPoints(const std::vector<Pose> &points) { this->points = points; }
  };

  struct Point
  {
    float x, y;
    Point() : Point(0, 0) {}
    Point(const float &x, const float &y) : x(x), y(y) {}
  };

  struct PoseInternal
  {
    Point point;
    float theta;
  };

  struct Circle
  {
    Point center;
    float radius = 0.F;
  };

  using Polygon = std::vector<Point>;
  using Vector  = Point[2];
#endif
} // namespace rpl

#endif