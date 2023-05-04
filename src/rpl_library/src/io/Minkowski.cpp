#include "rpl/io/Minkowski.hpp"

// rpl
#ifndef RPL_COMMON_HPP
#include "rpl/common.hpp"
#endif

void rpl::Minkowski::execute(const std::vector<Polygon> &polygons, const float &offset,
                             std::vector<Polygon> &out)
{
  this->reset();
  this->d_paths.reserve(polygons.size());

  const auto int_offset = std::int32_t(offset * settings::exp_factor());

  for (const auto &polygon : polygons)
    this->cvt_polygon_to_paths(polygon);
  this->inflate(int_offset);
  this->cvt_paths_to_polygon(out);
}

void rpl::Minkowski::cvt_polygon_to_paths(const Polygon &polygon)
{
  ClipperLib::Path path;
  path.reserve(settings::max_polygon_size());
  for (const auto &point : polygon)
    path << ClipperLib::IntPoint(std::int32_t(point.x * settings::exp_factor()),
                                 std::int32_t(point.y * settings::exp_factor()));
  this->d_paths.emplace_back(path);
}

void rpl::Minkowski::inflate(const std::int32_t &offset)
{
  this->d_clipperOffset.AddPaths(this->d_paths, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
  this->d_clipperOffset.Execute(this->d_paths, offset);
}

void rpl::Minkowski::reset()
{
  this->d_clipperOffset.Clear();
  this->d_paths.clear();
}

void rpl::Minkowski::cvt_paths_to_polygon(std::vector<Polygon> &out)
{
  rpl::Polygon polygon;
  polygon.reserve(settings::max_polygon_size());
  out.reserve(this->d_paths.size());
  for (auto &path : this->d_paths)
  {
    if (!ClipperLib::Orientation(path)) ClipperLib::ReversePath(path);
    for (const auto &intpoint : path)
      polygon.emplace_back(Point{float(intpoint.X) * settings::iexp_factor(),
                                 float(intpoint.Y) * settings::iexp_factor()});
    out.emplace_back(polygon);
    polygon.clear();
  }
}
