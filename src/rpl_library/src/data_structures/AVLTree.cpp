#include "rpl/data_structures/AVLTree.hpp"
#include <iostream>

rpl::AVLTree::AVLTree(const std::size_t &size) : AVLTree()
{
  this->d_root      = size;
  this->d_tombstone = size;
  this->d_tree      = new AVLNode[size + 1];
  this->d_keys      = new float[size + 1]();
  this->d_heights   = new std::int32_t[size + 1]();

  for (std::size_t i = 0; i <= size; ++i)
  {
    this->d_tree[i] = {size, size, size};
    ++this->d_heights[i];
  }
}

rpl::AVLTree::~AVLTree()
{
  this->deallocate_all();
}

void rpl::AVLTree::deallocate_all()
{
  delete[] this->d_tree;
  delete[] this->d_keys;
  delete[] this->d_heights;
}

void rpl::AVLTree::insert(const float &key, const std::size_t &value)
{
  this->d_keys[value] = key;
  if (this->empty(this->d_root))
  {
    this->d_root = value;
    this->update_height(value); // TODO: not needed... height should be 1 regardless
    return;
  }
  std::size_t src = this->search(key);
  // change child connection of src
  this->d_tree[value].parent = src;
  // change parent connection of dst
  if (key <= this->d_keys[src])
    this->d_tree[src].left = value;
  else
    this->d_tree[src].right = value;

  // balance tree
  this->balance(src);
}

void rpl::AVLTree::remove(const std::size_t &value)
{
  if (!this->is_root(value) && this->empty(this->d_tree[value].parent)) return;
  AVLNode     node   = this->d_tree[value];
  std::size_t parent = node.parent;

  std::size_t children_number = std::size_t(!this->empty(node.left)) + std::size_t(!this->empty(node.right));
  std::size_t start;

  switch (children_number)
  {
    case 0: // leaf node with no children -> just delete the node
    {
      if (this->is_root(value))
        this->d_root = this->d_tombstone;
      // remove parent connection
      if (this->d_tree[parent].left == value)
        this->d_tree[parent].left = this->d_tombstone;
      else
        this->d_tree[parent].right = this->d_tombstone;

      start = parent;
    }
    break;
    case 1:
    {
      std::size_t child = !this->empty(node.left) ? node.left : node.right;

      if (this->is_root(value)) this->d_root = child;
      if (this->d_tree[parent].left == value)
        this->d_tree[parent].left = child;
      else
        this->d_tree[parent].right = child;

      this->d_tree[child].parent = parent;
      start                      = child;
    }
    break;
    case 2:
    {
      std::size_t child = this->minimum(node.right);
      start             = this->d_tree[child].parent;

      if (this->is_root(value)) this->d_root = child;

      // detatch child
      if (start == value)
      {
        start      = child;
        node.right = this->d_tombstone;
      }
      else
        this->d_tree[start].left = this->d_tombstone;

      // substitute child to value
      this->d_tree[child].parent = node.parent;
      this->d_tree[child].left   = node.left;
      this->d_tree[child].right  = node.right;

      // fix connections
      this->d_tree[node.left].parent  = child;
      this->d_tree[node.right].parent = child;
      if (this->d_tree[parent].left == value)
        this->d_tree[parent].left = child;
      else
        this->d_tree[parent].right = child;
    }
    break;
  }
  this->delete_node(this->d_tombstone);
  this->delete_node(value);

  this->update_all_heights(start);
  // if (value == 7)
  // {
  // this->debug_info();
  // std::cerr << start << "\n";
  // }
  this->balance(start);
}

std::size_t rpl::AVLTree::search(const float &key)
{
  std::size_t current   = this->d_root;
  std::size_t candidate = current;
  for (;;)
  {
    if (this->empty(current)) break;
    candidate = current;
    if (key <= this->d_keys[current])
      current = this->d_tree[current].left;
    else
      current = this->d_tree[current].right;
  }
  return candidate;
}

std::size_t rpl::AVLTree::minimum(const std::size_t &node) const
{
  std::size_t current = node;
  for (;;)
  {
    if (this->empty(this->d_tree[current].left)) break;
    current = this->d_tree[current].left;
  }
  return current;
}

void rpl::AVLTree::reset()
{
  this->d_root = this->d_tombstone;
  for (std::size_t node = 0; node < this->d_tombstone; ++node)
    this->delete_node(node);
}

void rpl::AVLTree::rotate_left(const std::size_t &x)
{
  std::size_t p = this->d_tree[x].parent;
  std::size_t y = this->d_tree[x].right;
  std::size_t b = this->d_tree[y].left;

  AVLNode p_new = this->d_tree[p]; // parent node
  AVLNode x_new = this->d_tree[x]; // x node
  AVLNode y_new = this->d_tree[y]; // y node
  AVLNode b_new = this->d_tree[b]; // beta node

  // if y has a left subtree, assign x as the parent of the left subtree of y
  if (!empty(b)) b_new.parent = x;
  // if the parent of x is null, make y as the root of the tree
  if (this->is_root(x)) this->d_root = y;
  // else if x is the left child of p, make y as the left child of p
  // else assign y as the right child of p
  if (p_new.left == x)
    p_new.left = y;
  else
    p_new.right = y;
  y_new.parent = p;
  // make y as the parent of x
  x_new.parent = y;
  y_new.left   = x;

  //
  x_new.right = this->d_tombstone;

  // assign
  this->d_tree[p] = p_new;
  this->d_tree[x] = x_new;
  this->d_tree[y] = y_new;
  this->d_tree[b] = b_new;
  // update heights
  this->delete_node(this->d_tombstone);
  this->update_height(x);
  this->update_height(y);
}

void rpl::AVLTree::rotate_right(const std::size_t &y)
{
  std::size_t p = this->d_tree[y].parent;
  std::size_t x = this->d_tree[y].left;
  std::size_t b = this->d_tree[x].right;

  AVLNode p_new = this->d_tree[p]; // parent node
  AVLNode x_new = this->d_tree[x]; // x node
  AVLNode y_new = this->d_tree[y]; // y node
  AVLNode b_new = this->d_tree[b]; // beta node
  // if x has a right subtree, assign y as the parent of the right subtree of x
  if (!empty(b)) b_new.parent = y;
  // if the parent of y is null, make x as the root of the tree
  if (this->is_root(y))
    this->d_root = x;
  // if y is the right child of its p[arent p, make x as the right child of p
  // else assign x as the left child of p
  if (p_new.left == y)
    p_new.left = x;
  else
    p_new.right = x;
  x_new.parent = p;

  y_new.parent = x;
  x_new.right  = y;

  y_new.left = this->d_tombstone;

  // assign
  this->d_tree[p] = p_new;
  this->d_tree[x] = x_new;
  this->d_tree[y] = y_new;
  this->d_tree[b] = b_new;
  // update heights
  this->delete_node(this->d_tombstone);
  this->update_height(x);
  this->update_height(y);
}

void rpl::AVLTree::balance(const std::size_t &start_node)
{
  std::int32_t bFactor;
  std::size_t  temp;
  std::size_t  counter = 0;
  for (std::size_t node = start_node; !this->empty(node);)
  {
    ++counter;

    this->update_height(node);
    bFactor = this->balance_factor(node);
    if (bFactor > 1) // left heavy
    {
      std::size_t left_child = this->d_tree[node].left;
      if (this->balance_factor(left_child) < 0)
      {
        this->rotate_left(left_child);
      }
      this->rotate_right(node);
    }
    if (bFactor < -1) // right heavy
    {
      std::size_t right_child = this->d_tree[node].right;
      if (this->balance_factor(right_child) > 0)
      {
        this->rotate_right(right_child);
      }
      this->rotate_left(node);
    }
    node = this->d_tree[node].parent;
  }
}

void rpl::AVLTree::delete_node(const std::size_t &node)
{
  this->d_tree[node]    = {this->d_tombstone, this->d_tombstone, this->d_tombstone};
  this->d_keys[node]    = 0.f;
  this->d_heights[node] = 1;
}

void rpl::AVLTree::debug_info() const
{
  std::cerr << "root: " << this->d_root << "\n";
  std::cerr << "tombstone: " << this->d_tombstone << "\n";
  AVLNode current = {0, 0, 0};
  for (std::size_t node = 0; node <= this->d_tombstone; ++node)
  {
    current = this->d_tree[node];
    std::cerr << "Node " << node
              << ": {left: " << current.left
              << " , right: " << current.right
              << " , parent: " << current.parent
              << " , key: " << this->d_keys[node]
              << " , bfact: " << this->balance_factor(node)
              << " , height " << this->height(node) << "}\n";
  }
}

void rpl::AVLTree::update_all_heights(const std::size_t &start)
{
  for (std::size_t node = start; !this->empty(node);)
  {
    this->update_height(node);
    node = this->d_tree[node].parent;
  }
}
