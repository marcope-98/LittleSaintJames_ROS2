#include "rpl/map/RoadMap.hpp"

#ifndef INCLUDE_ALGORITHM
#include <algorithm>
#define INCLUDE_ALGORITHM
#endif

#ifndef INCLUDE_CMATH
#include <cmath>
#define INCLUDE_CMATH
#endif

#ifndef INCLUDE_NUMERIC
#include <numeric>
#define INCLUDE_NUMERIC
#endif

#ifndef INCLUDE_QUEUE
#include <queue>
#define INCLUDE_QUEUE
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

#ifndef INCLUDE_IMMINTRIN_H
#include <immintrin.h>
#define INCLUDE_IMMINTRIN_H
#endif

#include <iostream>

rpl::RoadMap::RoadMap(const std::vector<Polygon> &polylist,
                      const std::vector<Point> &  gates_list,
                      const std::vector<Point> &  robots_list) : RoadMap()
{
  // count number of nodes
  std::size_t n_vertices = this->count_polygon_vertices(polylist);
  std::size_t n_robots   = robots_list.size();
  std::size_t n_gates    = gates_list.size();
  std::size_t n_nodes    = n_vertices + n_gates + n_robots;

  // initialize member variables
  this->d_metadata = {n_vertices, n_gates, n_robots};
  this->d_avltree  = new PseudoAVLTree(n_nodes);
  this->d_graph    = new Graph(n_nodes);
  this->d_points   = new Point[n_nodes];
  this->d_segments = new SegmentLookUp[n_nodes];

  // process obstacles
  std::size_t i = 0;
  std::size_t first, second;
  for (const auto &polygon : polylist)
  {
    first  = i;
    second = i + polygon.size() - 1;
    for (const auto &point : polygon)
    {
      this->d_points[i]   = point;
      this->d_segments[i] = {i - 1, i + 1};
      ++i;
    }
    this->d_segments[first].prev  = second;
    this->d_segments[second].next = first;
  }

  // process robots
  for (const auto &point : robots_list)
  {
    this->d_points[i]   = point;
    this->d_segments[i] = {i, i};
    ++i;
  }

  // process gates
  for (const auto &point : gates_list)
  {
    this->d_points[i]   = point;
    this->d_segments[i] = {i, i};
    ++i;
  }
}

rpl::RoadMap::~RoadMap() { this->deallocate_all(); }

void rpl::RoadMap::deallocate_all()
{
  delete this->d_graph;
  delete this->d_avltree;
  delete[] this->d_points;
  delete[] this->d_segments;
}

void rpl::RoadMap::execute(const Polygon &border)
{
  this->compute_visibility_graph();
  // this->rm_out_of_bounds_nodes(border);
  this->check_connectivity();
}

std::size_t rpl::RoadMap::count_polygon_vertices(const std::vector<Polygon> &polylist) const
{
  std::size_t res = 0;
  for (const auto &polygon : polylist)
    res += polygon.size();
  return res;
}

void rpl::RoadMap::compute_visibility_graph()
{
  const std::size_t        vertices = this->d_graph->height();
  std::vector<std::size_t> W;
  W.reserve(vertices);
  // for all vertices v \in V
  for (std::size_t v = 0; v < vertices; ++v)
  {
    // do W <- VisibleVertices(v, S)
    this->visible_vertices(v, W);
    //    For every vertex w \in W, add the arc (v, w) to E
    for (const auto &w : W)
      this->d_graph->add_halfedge(v, w);
  }
}

void rpl::RoadMap::visible_vertices(const std::size_t &v, std::vector<std::size_t> &W)
{
  const std::size_t        vertices = this->d_graph->height();
  std::vector<std::size_t> out;
  /*
   sort the obstacle vertices according to the counterclockwise angle that the half-line
    from p to each vertex makes with the positive x-axis. In case of 
    ties, vertices closer to p should come before vertices farther from p.
    Let w1,...wn be the sorted list.
  */
  this->angular_sort(v, out);

  /*
  Let rho be the half-line parallel to the positive x-axis starting at p. Find the obstacle edges
   that are properly intersected by rho, and store them in a balanced search tree T in the order 
   in which they are intersected by rho.
  */
  this->AVLTree_initialize(v);
  W.clear();
  std::size_t wi;
  // skip last vertices (which is v)
  for (std::size_t i = 0; i < vertices - 1; ++i)
  {
    wi = out[i];
    this->AVLTree_update_keys(v, wi);
    if (this->visible(v, wi))
      W.emplace_back(wi);
    this->AVLTree_update_tree(v, wi);
  }
}

void rpl::RoadMap::angular_sort(const std::size_t &v, std::vector<std::size_t> &sorted_vertices) const
{
  const std::size_t  vertices = this->d_graph->height();
  std::vector<float> theta;
  std::vector<float> rho;
  theta.reserve(vertices);
  rho.reserve(vertices);

  float x_ref = this->d_points[v].x;
  float y_ref = this->d_points[v].y;
  float dx, dy;
  for (std::size_t i = 0; i < vertices; ++i)
  {
    dx = this->d_points[i].x - x_ref;
    dy = this->d_points[i].y - y_ref;
    rho.emplace_back(dx * dx + dy * dy);
    theta.emplace_back(utils::mod2pi(atan2(dy, dx)));
  }
  theta[v] = 8.F;

  // actual angular sort
  sorted_vertices.resize(vertices);
  std::iota(sorted_vertices.begin(), sorted_vertices.end(), 0);
  std::sort(sorted_vertices.begin(), sorted_vertices.end(), [&](const std::size_t &a, const std::size_t &b)
            { return (theta[a] < theta[b]) ||
                     ((theta[a] == theta[b]) && (rho[a] < rho[b])); });
}

bool rpl::RoadMap::visible(const std::size_t &v, const std::size_t &wi) const
{
  if (v == wi) return false;
  // if l does not lie between the two edges incident on v (the sweep line does not intersect the interior of the obstacle at v)
  if (this->in_polygon(v, wi)) return false;
  // if the line segment vwi does not intersect the closest edge in S
  std::size_t minseg = this->d_avltree->minseg();
  return !(ops::cmplt_f32(this->d_avltree->d_keys[minseg], 1.F));
}

void rpl::RoadMap::AVLTree_initialize(const std::size_t &v)
{
  this->d_avltree->reset();
  const std::size_t size = this->d_graph->height();
  float             intersection_point;
  std::size_t       j;
  for (std::size_t i = 0; i < size; ++i)
  {
    j                  = this->d_segments[i].next;
    intersection_point = this->get_halfplane_intersection(v, i, j);
    if (ops::cmplt_f32(intersection_point, 1.f))
      this->d_avltree->insert(i, intersection_point);
  }
}

void rpl::RoadMap::AVLTree_update_keys(const std::size_t &v, const std::size_t &wi)
{
  SegmentLookUp segment;
  float         intersection;
  std::size_t   s = this->d_avltree->size();

  for (std::size_t node = 0; node < s; ++node)
  {
    if (this->d_avltree->is_set(node))
    {
      segment      = this->d_segments[node];
      intersection = this->get_intersection(v, wi, node, segment.next);
      this->d_avltree->update(node, intersection);
    }
  }
  this->d_avltree->update_minseg();
}

void rpl::RoadMap::AVLTree_update_tree(const std::size_t &v, const std::size_t &wi)
{
  SegmentLookUp ab     = this->d_segments[wi];
  float         length = 1.f;

  if (this->orientation(v, wi, ab.prev) == -1)
    this->d_avltree->insert(ab.prev, length);
  else
    this->d_avltree->remove(ab.prev);

  if (this->orientation(v, wi, ab.next) == -1)
    this->d_avltree->insert(wi, length);
  else
    this->d_avltree->remove(wi);
}

bool rpl::RoadMap::in_polygon(const std::size_t &v, const std::size_t &wi) const
{
  SegmentLookUp seg = this->d_segments[wi];
  // 2 cases: v and wi belong to the same polygon, v and wi do not belong to the same polygon

  // TODO: is there a better way of doing this?
  if (this->polygon_search(v, wi))
  {
    if (seg.prev == v || seg.next == v) return false;
    bool one_left = false, one_right = false;
    for (std::size_t edge = seg.next; edge != v; edge = this->d_segments[edge].next)
    {
      if (this->orientation(wi, v, edge) == 1)
      {
        one_right = true;
        break;
      }
    }

    for (std::size_t edge = seg.prev; edge != v; edge = this->d_segments[edge].prev)
    {
      if (this->orientation(wi, v, edge) == -1)
      {
        one_left = true;
        break;
      }
    }
    return one_left && one_right;
  }

  return false;
}

bool rpl::RoadMap::polygon_search(const std::size_t &v, const std::size_t &wi) const
{
  std::size_t vertex = this->d_segments[wi].next;
  while (vertex != wi)
  {
    if (vertex == v) return true;
    vertex = this->d_segments[vertex].next;
  }

  return false;
}

void rpl::RoadMap::check_connectivity()
{
  std::size_t n_vertices = this->d_graph->height();
  for (std::size_t i = 0; i < n_vertices; ++i)
    for (std::size_t j = 0; j < n_vertices; ++j)
    {
      if (this->d_graph->operator()(i, j) != this->d_graph->operator()(j, i))
        std::cerr << i << " " << j << " are incongruent\n";
    }
}

void rpl::RoadMap::rm_out_of_bounds_nodes(const Polygon &border)
{
  const std::size_t n_points = this->d_graph->height();
  Point             point;

  for (std::size_t i = 0; i < n_points; ++i)
  {
    point = this->d_points[i];
    if (!geometry::point_in_polygon(point, border))
      this->d_graph->rm_node(i);
  }
}

float rpl::RoadMap::get_intersection(const std::size_t &p0, const std::size_t &p1, const std::size_t &p2, const std::size_t &p3) const
{
  Point x0 = this->d_points[p0];
  Point x1 = this->d_points[p1];
  Point y0 = this->d_points[p2];
  Point y1 = this->d_points[p3];
  return geometry::segment_segment_intersection({x0, x1}, {y0, y1});
}

float rpl::RoadMap::get_halfplane_intersection(const std::size_t &v, const std::size_t &edge1, const std::size_t &edge2)
{
  Point x0 = this->d_points[v];
  Point y0 = this->d_points[edge1];
  Point y1 = this->d_points[edge2];
  return geometry::halfplane_intersection(x0, {y0, y1});
}

std::int32_t rpl::RoadMap::orientation(const std::size_t &p0, const std::size_t &p1, const std::size_t &p2) const
{
  Point x0 = this->d_points[p0];
  Point x1 = this->d_points[p1];
  Point x2 = this->d_points[p2];
  return geometry::orientation(x0, x1, x2);
}

bool rpl::RoadMap::intersects(const std::size_t &p0, const std::size_t &p1, const std::size_t &p2, const std::size_t &p3) const
{
  Point x0 = this->d_points[p0];
  Point x1 = this->d_points[p1];
  Point y0 = this->d_points[p2];
  Point y1 = this->d_points[p3];
  return geometry::intersects({x0, x1}, {y0, y1});
}

void rpl::RoadMap::dijkstra(std::vector<std::vector<rpl::Point>> &out) const
{
  const std::size_t gate_index = this->d_metadata.vertices + this->d_metadata.robots;
  const std::size_t n_nodes    = this->d_graph->height();
  float *           dist       = new float[n_nodes]();
  std::size_t *     prev       = new std::size_t[n_nodes]();
  std::vector<bool> visited(n_nodes, false);

  std::priority_queue<std::pair<float, std::size_t>> queue;

  for (std::size_t i = 0; i < gate_index; ++i)
  {
    prev[i] = n_nodes;
    dist[i] = 1e9;
  }

  dist[gate_index] = 0.f;
  queue.push({0.f, gate_index});
  std::pair<float, std::size_t> a;
  std::vector<std::size_t>      neighbors;
  Point                         point1 = this->d_points[gate_index];
  Point                         point2;
  float                         norm;

  while (!queue.empty())
  {
    a = queue.top();
    queue.pop();
    if (visited[a.second]) continue;
    visited[a.second] = true;
    this->d_graph->neighborhood(a.second, neighbors);
    float update;
    for (const auto &neighbor : neighbors)
    {
      point2 = this->d_points[neighbor];
      update = dist[a.second] + geometry::norm({point1, point2});
      if (update < dist[neighbor])
      {
        dist[neighbor] = update;
        prev[neighbor] = a.second;
        queue.push({-update, neighbor});
      }
    }
  }

  out.resize(this->d_metadata.robots);
  std::size_t current;
  for (std::size_t robot = 0; robot < this->d_metadata.robots; ++robot)
  {
    current = this->d_metadata.vertices + robot;
    while (current < gate_index)
    {
      if (current == n_nodes)
      {
        out[robot].clear();
        break;
      }
      current = prev[current];
      out[robot].emplace_back(this->d_points[current]);
    }
  }

  delete[] dist;
  delete[] prev;
}