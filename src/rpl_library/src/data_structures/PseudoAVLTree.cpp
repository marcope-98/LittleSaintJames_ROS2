#include "rpl/data_structures/PseudoAVLTree.hpp"

#ifndef RPL_INTERNAL_RPLINTRIN_HPP
#include "rpl/internal/rplintrin.hpp"
#endif

#ifndef INCLUDE_IMMINTRIN_H
#include <immintrin.h>
#define INCLUDE_IMMINTRIN_H
#endif

#include <iostream>

rpl::PseudoAVLTree::PseudoAVLTree(const std::size_t &N) : PseudoAVLTree()
{
  this->d_minseg = N;
  this->d_size   = N;

  std::size_t active_edges_size = this->align_up(16); // round up to multiple of 16
  std::size_t keys_size         = this->align_up(4);  // round up to multiple of 4

  this->d_keys        = new float[keys_size];
  this->d_ActiveEdges = new std::uint8_t[active_edges_size]();

  __m128 two = _mm_set1_ps(2.f);
  for (std::size_t i = 0; i < keys_size; i += 4)
    _mm_storeu_ps(this->d_keys + i, two);
}

rpl::PseudoAVLTree::~PseudoAVLTree()
{
  this->deallocate_all();
}

void rpl::PseudoAVLTree::deallocate_all()
{
  delete[] this->d_keys;
  delete[] this->d_ActiveEdges;
}

bool rpl::PseudoAVLTree::is_set(const std::size_t &value) const
{
  std::size_t quo = value >> 3u;
  std::size_t rem = value - (quo << 3u);
  return bool(this->d_ActiveEdges[quo] & (0x80u >> rem));
}

void rpl::PseudoAVLTree::insert(const std::size_t &value, const float &key)
{
  std::size_t quo = value >> 3u;
  std::size_t rem = value - (quo << 3u);
  this->d_ActiveEdges[quo] |= (0x80u >> rem); // set bit
  this->d_keys[value] = key;
}

void rpl::PseudoAVLTree::update(const std::size_t &value, const float &key)
{
  this->d_keys[value] = key;
}

void rpl::PseudoAVLTree::remove(const std::size_t &value)
{
  std::size_t quo = value >> 3u;
  std::size_t rem = value - (quo << 3u);
  this->d_ActiveEdges[quo] &= ~(0x80u >> rem);
  this->d_keys[value] = 2.f;
}

void rpl::PseudoAVLTree::reset()
{
  this->d_minseg             = this->size();
  std::size_t N_active_edges = this->align_up(16);
  std::size_t N_keys         = this->align_up(4);

  __m128i zero = _mm_setzero_si128();
  __m128  two  = _mm_set1_ps(2.f);

  for (std::size_t i = 0; i < N_active_edges; i += 16)
    _mm_storeu_si128((__m128i *)(this->d_ActiveEdges + i), zero);

  for (std::size_t i = 0; i < N_keys; i += 4)
    _mm_storeu_ps(this->d_keys + i, two);
}

void rpl::PseudoAVLTree::debug_info() const
{
  for (std::size_t i = 0; i < this->size(); ++i)
    std::cerr << i << (this->is_set(i) ? " true  " : " false ") << this->d_keys[i] << "\n";
  std::cerr << "\n";
}

std::size_t rpl::PseudoAVLTree::minimum() const
{
  const __m128i increment  = _mm_set1_epi32(4);
  __m128i       indices    = _mm_set_epi32(3, 2, 1, 0);
  __m128i       minindices = indices;
  __m128        minvalues  = _mm_loadu_ps(this->d_keys);

  __m128  values;
  __m128i mask;
  for (std::size_t i = 4; i < this->size(); i += 4)
  {
    indices    = _mm_add_epi32(indices, increment);
    values     = _mm_loadu_ps(this->d_keys + i);
    mask       = _mm_castps_si128(_mm_cmplt_ps(values, minvalues));
    minindices = _mm_blendv_epi8(minindices, indices, mask);
    minvalues  = _mm_min_ps(values, minvalues);
  }

  float         values_array[4];
  std::uint32_t indices_array[4];
  _mm_storeu_ps(values_array, minvalues);
  _mm_storeu_si128((__m128i *)indices_array, minindices);
  std::size_t minindex = indices_array[0];
  float       minvalue = values_array[0];
  for (std::size_t i = 1; i < 4; ++i)
  {
    if (ops::cmplt_f32(values_array[i], minvalue))
    {
      minindex = indices_array[i];
      minvalue = values_array[i];
    }
  }
  return minindex > this->size() ? this->size() : minindex;
}

void rpl::PseudoAVLTree::update_minseg()
{
  this->d_minseg = this->minimum();
}