#ifndef RPL_MAP_ROADMAP_HPP
#define RPL_MAP_ROADMAP_HPP
// stl
#ifndef INCLUDE_CSTDINT
#include <cstdint>
#define INCLUDE_CSTDINT
#endif

#ifndef INCLUDE_VECTOR
#include <vector>
#define INCLUDE_VECTOR
#endif

// rpl
#ifndef RPL_TYPES_HPP
#include "rpl/types.hpp"
#endif

// rpl/map
#ifndef RPL_MAP_GRAPH_HPP
#include "rpl/map/Graph.hpp"
#endif

#ifndef RPL_DATA_STRUCTURES_PSEUDOAVLTREE_HPP
#include "rpl/data_structures/PseudoAVLTree.hpp"
#endif

namespace rpl
{
  struct RoadMap
  {
  private:
    struct Metadata
    {
      std::size_t vertices = 0;
      std::size_t robots   = 0;
      std::size_t gates    = 0;
    };

    struct SegmentLookUp
    {
      std::size_t prev = 0;
      std::size_t next = 0;
    };

  public:
    Metadata       d_metadata;
    PseudoAVLTree *d_avltree  = nullptr;
    Graph *        d_graph    = nullptr;
    Point *        d_points   = nullptr;
    SegmentLookUp *d_segments = nullptr;

    // Constructors
    RoadMap() = default;
    RoadMap(const std::vector<Polygon> &polylist,
            const std::vector<Point> &  gates_list,
            const std::vector<Point> &  robots_list);
    RoadMap(const RoadMap &) = delete;
    RoadMap(RoadMap &&)      = delete;
    ~RoadMap();

    // Manipulators
    RoadMap &operator=(const RoadMap &) = delete;
    RoadMap &operator=(RoadMap &&) = delete;

  public:
    void draw(const std::vector<std::vector<Point>> &answer);
    void execute(const Polygon &border);
    void dijkstra(std::vector<std::vector<Point>> &out) const;

  private:
    // call stack of visibility graph construction
    void compute_visibility_graph();
    void visible_vertices(const std::size_t &v, std::vector<std::size_t> &W);
    void angular_sort(const std::size_t &v, std::vector<std::size_t> &sorted_vertices) const;
    bool visible(const std::size_t &v, const std::size_t &wi) const;
    void rm_out_of_bounds_nodes(const Polygon &border);

    // rpl::geometry wrappers
    float        get_intersection(const std::size_t &p0, const std::size_t &p1, const std::size_t &p2, const std::size_t &p3) const;
    float        get_halfplane_intersection(const std::size_t &v, const std::size_t &edge1, const std::size_t &edge2);
    std::int32_t orientation(const std::size_t &p0, const std::size_t &p1, const std::size_t &p2) const;
    bool         intersects(const std::size_t &p0, const std::size_t &p1, const std::size_t &p2, const std::size_t &p3) const;

    // memory management
    void deallocate_all();

    // AVLTree management
    void AVLTree_initialize(const std::size_t &v);
    void AVLTree_update_keys(const std::size_t &v, const std::size_t &wi);
    void AVLTree_update_tree(const std::size_t &v, const std::size_t &wi);

    // utils for polygon processing
    std::size_t count_polygon_vertices(const std::vector<Polygon> &polylist) const;
    bool        in_polygon(const std::size_t &v, const std::size_t &wi) const;
    bool        polygon_search(const std::size_t &v, const std::size_t &wi) const;
    void        check_connectivity();
  };
} // namespace rpl

#endif