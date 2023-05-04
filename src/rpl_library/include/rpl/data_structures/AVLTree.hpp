#ifndef RPL_DATA_STRUCTURES_AVLTREE_HPP
#define RPL_DATA_STRUCTURES_AVLTREE_HPP

#ifndef INCLUDE_CSTDINT
#include <cstdint>
#define INCLUDE_CSTDINT
#endif

namespace rpl
{
  struct AVLNode
  {
    std::size_t left;
    std::size_t right;
    std::size_t parent;
  };

  struct AVLTree
  {
    std::size_t   d_root      = 0;
    std::size_t   d_tombstone = 0;
    AVLNode *     d_tree      = nullptr;
    float *       d_keys      = nullptr;
    std::int32_t *d_heights   = nullptr;

    // Constructors
    AVLTree() = default;
    explicit AVLTree(const std::size_t &size);
    AVLTree(const AVLTree &) = delete;
    AVLTree(AVLTree &&)      = delete;
    ~AVLTree();

    // Manipulators
    AVLTree &operator=(const AVLTree &) = delete;
    AVLTree &operator=(AVLTree &&) = delete;

    // Getters
    std::int32_t height(const std::size_t &node) const { return this->empty(node) ? 0 : this->d_heights[node]; }
    std::int32_t balance_factor(const std::size_t &node) const
    {
      AVLNode           parent               = this->d_tree[node];
      const std::size_t height_left_subtree  = this->height(parent.left);
      const std::size_t height_right_subtree = this->height(parent.right);
      return height_left_subtree - height_right_subtree;
    }

    bool is_root(const std::size_t &node) const { return node == this->d_root; }
    bool empty(const std::size_t &node) const { return this->d_tombstone == node; }
    bool is_invalid(const std::size_t &node) const { return !this->is_root(node) && this->empty(this->d_tree[node].parent); }

    // Methods
  public:
    void        insert(const float &key, const std::size_t &value);
    void        remove(const std::size_t &value);
    std::size_t search(const float &key);
    std::size_t minimum(const std::size_t &node) const;
    void        reset();
    void        debug_info() const;

  private:
    std::int32_t max_height(const std::size_t &node) const
    {
      AVLNode           parent               = this->d_tree[node];
      const std::size_t height_left_subtree  = this->height(parent.left);
      const std::size_t height_right_subtree = this->height(parent.right);
      return height_left_subtree >= height_right_subtree ? height_left_subtree : height_right_subtree;
    }
    void update_height(const std::size_t &node) { this->d_heights[node] = 1 + this->max_height(node); }
    void deallocate_all();
    void rotate_left(const std::size_t &x);
    void rotate_right(const std::size_t &y);
    void balance(const std::size_t &start_node);
    void delete_node(const std::size_t &node);
    void update_all_heights(const std::size_t &start);
  };
} // namespace rpl
#endif