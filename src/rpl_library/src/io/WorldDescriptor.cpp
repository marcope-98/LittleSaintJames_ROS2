#include "rpl/io/WorldDescriptor.hpp"

#ifndef INCLUDE_CSTDINT
#include <cstdint>
#define INCLUDE_CSTDINT
#endif

#ifndef RPL_COMMON_HPP
#include "rpl/common.hpp"
#endif

#ifndef RPL_INTERNAL_GEOMETRY_HPP
#include "rpl/internal/geometry.hpp"
#endif
#include <utility>

void rpl::WorldDescriptor::process_obstacles(const std::vector<Polygon> &obstacles)
{
  this->d_minkowski.execute(obstacles, settings::working_envelope(), this->d_obstacles);
}

void rpl::WorldDescriptor::process_border(const Polygon &border)
{
  std::vector<Polygon> temp1, temp2;
  temp1.emplace_back(border);
  this->d_minkowski.execute(temp1, -settings::working_envelope(), temp2);
  this->d_worldLimits = std::move(temp2.front());
}

void rpl::WorldDescriptor::process_gates(const std::vector<Polygon> &gates)
{
  this->d_gates.clear();
  this->d_gates.reserve(gates.size());

  for (const auto &gate : gates)
    this->d_gates.emplace_back(geometry::centroid(gate));
}

void rpl::WorldDescriptor::process_robots(const std::vector<float> &x, const std::vector<float> &y)
{
  const std::size_t size = x.size();
  this->d_robots.clear();
  this->d_robots.reserve(size);

  for (std::size_t i = 0; i < size; ++i)
    this->d_robots.emplace_back(Point{x[i], y[i]});
}
