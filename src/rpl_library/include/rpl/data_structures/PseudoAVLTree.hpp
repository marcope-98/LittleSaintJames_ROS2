#ifndef RPL_DATA_STRUCTURES_PSEUDOAVLTREE_HPP
#define RPL_DATA_STRUCTURES_PSEUDOAVLTREE_HPP

#ifndef INCLUDE_CSTDINT
#include <cstdint>
#define INCLUDE_CSTDINT
#endif

namespace rpl
{
  struct PseudoAVLTree
  {
    // Member variables
    std::size_t   d_minseg      = 0;
    std::size_t   d_size        = 0;
    float *       d_keys        = nullptr;
    std::uint8_t *d_ActiveEdges = nullptr;

    // Constructors
    PseudoAVLTree() = default;
    explicit PseudoAVLTree(const std::size_t &N);
    PseudoAVLTree(const PseudoAVLTree &) = delete;
    PseudoAVLTree(PseudoAVLTree &&)      = delete;
    ~PseudoAVLTree();

    // Manipulators
    PseudoAVLTree &operator=(const PseudoAVLTree &) = delete;
    PseudoAVLTree &operator=(PseudoAVLTree &&) = delete;

  public:
    // Getters
    bool        empty() const { return this->d_minseg == this->d_size; }
    bool        is_set(const std::size_t &value) const;
    std::size_t minseg() const { return this->d_minseg; }
    std::size_t size() const { return this->d_size; }

    // Methods
    void insert(const std::size_t &value, const float &key);
    void remove(const std::size_t &value);
    void update(const std::size_t &value, const float &key);
    void reset();
    void debug_info() const;
    void update_minseg();

  private:
    std::size_t minimum() const;
    std::size_t align_up(const std::size_t &bytes) const { return (this->size() + (bytes - 1)) & ~(bytes - 1); }
    void        deallocate_all();
  };
} // namespace rpl
#endif
