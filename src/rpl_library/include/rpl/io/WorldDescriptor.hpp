#ifndef RPL_IO_WORLDDESCRIPTOR_HPP
#define RPL_IO_WORLDDESCRIPTOR_HPP

#ifndef INCLUDE_VECTOR
#include <vector>
#define INCLUDE_VECTOR
#endif

#ifndef RPL_TYPES_HPP
#include "rpl/types.hpp"
#endif

#ifndef RPL_IO_MINKOWSKI_HPP
#include "rpl/io/Minkowski.hpp"
#endif

namespace rpl
{
  struct WorldDescriptor
  {
  private:
    Minkowski d_minkowski;

    std::vector<Polygon> d_obstacles;
    std::vector<Point>   d_gates;
    std::vector<Point>   d_robots;
    Polygon              d_worldLimits;

  public:
    // Getters
    const std::vector<Polygon> &obstacles() const { return this->d_obstacles; };
    const std::vector<Point> &  gates() const { return this->d_gates; };
    const std::vector<Point> &  robots() const { return this->d_robots; };
    const Polygon &             border() const { return this->d_worldLimits; };

    void process_obstacles(const std::vector<Polygon> &obstacles);
    void process_gates(const std::vector<Polygon> &gates);
    void process_robots(const std::vector<float> &x, const std::vector<float> &y);
    void process_border(const Polygon &border);
  };
} // namespace rpl

#endif