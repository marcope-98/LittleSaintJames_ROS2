#ifndef RPL_IO_MINKOWSKI_HPP
#define RPL_IO_MINKOWSKI_HPP

// stl
#ifndef INCLUDE_VECTOR
#include <vector>
#define INCLUDE_VECTOR
#endif

// clipper
#ifndef RPL_CLIPPER_CLIPPER_HPP
#include "rpl/clipper/clipper.hpp"
#define RPL_CLIPPER_CLIPPER_HPP
#endif

// rpl
#ifndef RPL_TYPES_HPP
#include "rpl/types.hpp"
#endif

namespace rpl
{
  struct Minkowski
  {
  private:
    ClipperLib::ClipperOffset d_clipperOffset;
    ClipperLib::Paths         d_paths;

  public:
    void execute(const std::vector<Polygon> &polygons, const float &offset,
                 std::vector<Polygon> &out);

  private:
    void cvt_polygon_to_paths(const Polygon &polygon);
    void cvt_paths_to_polygon(std::vector<Polygon> &out);
    void inflate(const std::int32_t &offset);
    void reset();
  };
} // namespace rpl

#endif