#ifndef RPL_MAP_GRAPH_HPP
#define RPL_MAP_GRAPH_HPP

#ifndef INCLUDE_CSTDINT
#include <cstdint>
#define INCLUDE_CSTDINT
#endif

#ifndef INCLUDE_VECTOR
#include <vector>
#define INCLUDE_VECTOR
#endif

namespace rpl
{
  struct Graph
  {
    // Member variables
    std::size_t   d_nodes           = 0;
    std::uint8_t *d_AdjacencyMatrix = nullptr;

    // Constructors
    Graph() = default;
    explicit Graph(const std::size_t &nodes);
    Graph(const Graph &other);
    Graph(Graph &&other) noexcept;
    ~Graph();

    // Manipulators
    Graph &operator=(const Graph &other);
    Graph &operator=(Graph &&other) noexcept;
    bool   operator()(const std::size_t &src, const std::size_t &dst) const;

    // Getters
    std::size_t width() const { return ((this->d_nodes + 7u) & ~(7u)) >> 3u; }
    std::size_t height() const { return this->d_nodes; }
    std::size_t size() const { return this->width() * this->height(); }
    std::size_t capacity() const { return (this->size() + 15u) & ~(15u); }
    // Methods
  public:
    // add operations
    void add_edge(const std::size_t &node1, const std::size_t &node2);
    void add_halfedge(const std::size_t &node1, const std::size_t &node2);
    // rm operation
    void rm_node(const std::size_t &node);
    void rm_edge(const std::size_t &node1, const std::size_t &node2);
    void rm_halfedge(const std::size_t &node1, const std::size_t &node2);

    void neighborhood(const std::size_t &node, std::vector<std::size_t> &out) const;
    void print() const;

  private:
    void deallocate_all();
    void reallocate(const std::size_t &capacity);
    void zero_all();
    void fast_copy(const Graph &other);
  };
} // namespace rpl
#endif