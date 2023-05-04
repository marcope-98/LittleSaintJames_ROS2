#ifndef RPL_INTERFACE_HPP
#define RPL_INTERFACE_HPP

// stl
#ifndef INCLUDE_VECTOR
#include <vector>
#define INCLUDE_VECTOR
#endif

// rpl
#ifndef RPL_TYPES_HPP
#include "rpl/types.hpp"
#endif

// rpl/io
#ifndef RPL_IO_WORLDDESCRIPTOR_HPP
#include "rpl/io/WorldDescriptor.hpp"
#endif

// rpl/map
#ifndef RPL_MAP_ROADMAP_HPP
#include "rpl/map/RoadMap.hpp"
#endif

namespace rpl
{
  struct interface
  {
#if 1
    static bool planPath()
    {
      std::cerr << "planPath function\n";
      return true;
    }
#else
    static bool planPath(const rpl::Polygon &             border,
                         const std::vector<rpl::Polygon> &obstacle_list,
                         const std::vector<rpl::Point> &  gate_list,
                         const std::vector<rpl::Pose> &   robot_list,
                         std::vector<rpl::Path> &         path)
    {
      // World Descriptor is just a container that takes care of inflating obstacles
      rpl::WorldDescriptor wd;
      wd.process_border(border);
      wd.process_robots(robot_list);
      wd.process_gates(gate_list);
      wd.process_obstacles(obstacle_list);

      // RoadMap construction
      std::vector<std::vector<rpl::Point>> answer;
      rpl::RoadMap                         roadmap(wd.obstacles(), wd.gates(), wd.robots());
      roadmap.execute(wd.border());
      roadmap.dijkstra(answer);
      roadmap.draw(answer);
      return true;
    }
#endif
  };
} // namespace rpl

#endif