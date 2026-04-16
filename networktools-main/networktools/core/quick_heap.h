/*
 * Software Name : networktools
 * SPDX-FileCopyrightText: Copyright (c) Orange SA
 * SPDX-License-Identifier: MIT
 *
 * This software is distributed under the MIT licence,
 * see the "LICENSE" file for more details or https://opensource.org/license/MIT
 *
 * Authors: see CONTRIBUTORS.md
 * Software description: An efficient C++ library for modeling and solving network optimization problems
 */

/**
 * @file
 * @brief Priority queue implementation using a d-ary heap data structure.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_QUICK_HEAP_H_
#define _NT_QUICK_HEAP_H_

#include "common.h"
#include "arrays.h"
#include "maps.h"
#include "tuple.h"
#include "bin_heap.h"

namespace nt {

  /**
   * @class QuickHeap
   * @headerfile dary_heap.h
   * @brief Implementation of a d-ary heap data structure that uses SIMD instructions if available.
   *
   * Example of usage :
   *
   * @code
   * int main() {
   *
   *  // Create a QuickHeap containing integers with priority type int
   *  using ItemIntMap = nt::ArrayMap< int >;
   *  nt::QuickHeap< int, ItemIntMap > heap;
   *
   *  // Initialize the heap to accomodate 10 elements
   *  heap.init(10);
   *
   *  // Push some items with priorities into the heap
   *  heap.push(1000, 5);
   *  heap.push(2000, 3);
   *  heap.push(3000, 7);
   *  heap.push(4000, 2);
   *
   *  // Print the top item and its priority
   *  std::cout << "Top item: " << heap.top() << ", Priority: " << heap.prio() << std::endl;
   *
   *  // Modify the priority of an item
   *  heap.decrease(3000, 1);
   *  std::cout << "Top item: " << heap.top() << ", Priority: " << heap.prio() << std::endl;
   *
   *  // Pop the top item from the heap
   *  heap.pop();
   *  std::cout << "Top item: " << heap.top() << ", Priority: " << heap.prio() << std::endl;
   *
   *  // Check if the heap is empty
   *  if (heap.empty()) {
   *    std::cout << "Heap is empty." << std::endl;
   *  } else {
   *    std::cout << "Heap is not empty." << std::endl;
   *  }
   *
   *  return 0;
   * }
   * @endcode
   *
   * @tparam PR Type of the priorities of the items. If PR is of type 'uint16_t' then the heap may
   * use SIMD instructions to improve the running time of the restoreDown() operation.
   * @tparam IIM Type representing the item-to-integer map used internally. This map stores the position of each item in
   * the heap.
   */
  template < class PR, class IIM >
  struct QuickHeap {
    static_assert(std::is_arithmetic< PR >::value, "QuickHeap : Priority must be arithmetic.");
    using ItemIntMap = IIM;
    using Priority = PR;
    using Item = typename ItemIntMap::Key;
    using Pair = nt::Pair< Item, Priority >;

    enum State { IN_HEAP = 0, PRE_HEAP = -1, POST_HEAP = -2 };

    TrivialDynamicArray< Item >     _items;
    TrivialDynamicArray< Priority > _priority;
    ItemIntMap                      _item_int_map;

    constexpr static int POWER =
       std::is_same< Priority, std::uint16_t >::value ? 3 : 1;   //< Power of the degree (D = 2^Power)
    constexpr static int D = 1 << POWER;                         //< Degree of the heap

    QuickHeap() = default;

    /**
     * @brief Returns the parent of the given node.
     *
     * @param i The index of the node.
     * @return The index of the parent node.
     */
    constexpr int parentOf(int i) const noexcept { return (i - 1) / D; }

    /**
     * @brief Returns the index of the first child of the given node.
     *
     * @param i The index of the node.
     * @return The index of the first child node.
     */
    constexpr int firstChildOf(int i) const noexcept { return D * i + 1; }

    /**
     * @brief Initializes the heap with the given size.
     *
     * @param i_size The initial size of the heap.
     */
    constexpr void init(int i_size) noexcept {
      clear();
      const int capacity = D * ((i_size >> POWER) + 1) + 1;
      _items.ensure(capacity);
      _priority.ensure(capacity);
      _item_int_map.build(i_size);
      _item_int_map.setBitsToOne();

      // Initialize priority array with max values to serve as sentinels for SIMD operations
      // For unsigned types, use memset(0xFF)
      // For signed types, we must use the loop since max() != all bits set
      if constexpr (std::is_unsigned_v< Priority >) {
        std::memset(_priority.data(), 0xFF, _priority.capacity() * sizeof(Priority));
      } else {
        for (int k = 0; k < _priority.capacity(); ++k)
          _priority.data()[k] = std::numeric_limits< Priority >::max();
      }
    }

    /**
     * @brief Clears the heap, removing all elements.
     */
    constexpr void clear() noexcept {
      _items.removeAll();
      _priority.removeAll();
      _item_int_map.setBitsToOne();
    }

    /**
     * @brief Checks if the heap is empty.
     *
     * @return True if the heap is empty, false otherwise.
     */
    constexpr bool empty() const noexcept { return _items.empty(); }

    /**
     * @brief Returns the item at the top of the heap.
     *
     * @return The item at the top of the heap.
     */
    constexpr Item top() const noexcept { return _items[0]; }

    /**
     * @brief Returns the priority of the top item in the heap.
     *
     * @return The priority of the top item.
     */
    constexpr Priority prio() const noexcept { return _priority[0]; }

    /**
     * @brief Returns the priority associated with the given item.
     *
     * @param i The item.
     * @return The priority associated with the item.
     */
    constexpr Priority operator[](const Item& i) const noexcept {
      const int id = _item_int_map[i];
      return _priority[id];
    }

    /**
     * @brief Returns the state of the given item.
     *
     * @param i The item.
     * @return The state of the item.
     */
    constexpr State state(const Item& i) const noexcept {
      int s = _item_int_map[i];
      if (s >= 0) s = 0;
      return static_cast< State >(s);
    }

    /**
     * @brief Pushes a new item with the given priority into the heap.
     *
     * @param Item The item to be inserted.
     * @param p The priority of the item.
     */
    constexpr void push(const Item& item, const Priority& p) noexcept {
      _items.add(item);
      _priority.add(p);
      return _restoreUp(_items.size() - 1);
    }

    /**
     * @brief Sets the priority of an item in the heap.
     *
     * If the item does not exist in the heap, it will be added with the given priority.
     * If the item already exists in the heap, its priority will be updated.
     *
     * @param i The item to set or update.
     * @param p The priority to set or update for the item.
     */
    void set(const Item& i, const Priority& p) noexcept {
      const int id = _item_int_map[i];
      if (id < 0) {
        push(i, p);
      } else if (p < _priority[id]) {
        _priority[id] = p;
        _restoreUp(id);
      } else {
        _priority[id] = p;
        _restoreDown(id);
      }
    }

    /**
     * @brief Removes the top item from the heap.
     */
    constexpr void pop() noexcept {
      _item_int_map.set(_items[0], POST_HEAP);
      _items.remove(0);
      _priority.remove(0);
      assert(_priority.size() < _priority.capacity());
      _priority._p_data[_priority.size()] = std::numeric_limits< Priority >::max();   // Put back the sentinel
      if (!empty()) _restoreDown(0);
    }

    /**
     * @brief Decreases the priority of the given item in the heap.
     *
     * @param i The item.
     * @param p The new priority.
     */
    constexpr void decrease(const Item& i, const Priority& p) noexcept {
      const int id = _item_int_map[i];
      _priority[id] = p;
      _restoreUp(id);
    }

    /**
     * @brief Increases the priority of the given item in the heap.
     *
     * @param i The item.
     * @param p The new priority.
     */
    constexpr void increase(const Item& i, const Priority& p) noexcept {
      const int id = _item_int_map[i];
      _priority[id] = p;
      _restoreDown(id);
    }

    // Private methods

    /**
     * @brief Moves the given pair to the specified index in the heap.
     *
     * @param p The pair to be moved.
     * @param i The index where the pair should be moved.
     */
    constexpr void _move(Pair&& p, int i) noexcept {
      _items[i] = p.first;
      _priority[i] = p.second;
      _item_int_map.set(p.first, i);
    }

    /**
     * @brief Restores the heap property by moving a node up in the heap until its parent is
     * smaller.
     *
     * @param i_child The index of the child node to restore.
     */
    constexpr void _restoreUp(int i_child) noexcept {
      Pair pair = {_items[i_child], _priority[i_child]};
      int  i_parent = parentOf(i_child);
      while (i_parent != i_child && pair.second < _priority[i_parent]) {
        _move({_items[i_parent], _priority[i_parent]}, i_child);
        i_child = i_parent;
        i_parent = parentOf(i_child);
      }
      _move(std::move(pair), i_child);
    }

    /**
     * @brief Restores the heap property by moving a node down in the heap until its children are
     * larger.
     *
     * @param i_parent The index of the parent node to restore.
     */
    void _restoreDown(int i_parent) noexcept {
      ZoneScoped;
      Pair pair = {_items[i_parent], _priority[i_parent]};
      int  i_min_child = _findMinChild(i_parent);
      while (i_min_child >= 0 && _priority[i_min_child] < pair.second) {
        _move({_items[i_min_child], _priority[i_min_child]}, i_parent);
        i_parent = i_min_child;
        i_min_child = _findMinChild(i_min_child);
      }
      _move(std::move(pair), i_parent);
    }

    /**
     * @brief Finds the index of the minimum child node of a given parent node.
     *
     * @param i_node The index of the parent node.
     * @return The index of the minimum child node, or -1 if the parent has no children.
     */
    int _findMinChild(int i_node) const noexcept {
      ZoneScoped;
      if constexpr (std::is_same< Priority, std::int32_t >::value) {
        return _findMinChildDefault(i_node);   // use _findMinChild32bits once implemented
      } else if constexpr (std::is_same< Priority, std::uint16_t >::value) {
        return _findMinChild16bits(i_node);
      } else {
        return _findMinChildDefault(i_node);
      }
    }

#if defined(NT_USE_SIMD)
    constexpr bool simdEnabled() const noexcept { return true; }
    constexpr bool useMinChild16bits() const noexcept { return std::is_same< Priority, std::uint16_t >::value; }
    constexpr bool useMinChild32bits() const noexcept { return std::is_same< Priority, std::int32_t >::value; }
    constexpr bool useMinChildDefault() const noexcept { return !useMinChild16bits() && !useMinChild32bits(); }

    /**
     * @brief Finds the index of the minimum child node of a given parent node (SIMD
     * implementation)
     *
     * @param i_node The index of the parent node.
     * @return The index of the minimum child node, or -1 if the parent has no children.
     */
    int _findMinChild32bits(int i_node) const noexcept {
      ZoneScoped;
      // WARNING : This code only works if Priority is a signed 32bits integer
      const int from = i_node * 4 + 1;
      if (from >= _items.size()) return -1;
      const __m128i v = _mm_loadu_si128((__m128i*)(&_priority[from]));
      __m128i       vmin = v;
      vmin = _mm_min_epi32(vmin, _mm_alignr_epi8(vmin, vmin, 4));
      vmin = _mm_min_epi32(vmin, _mm_alignr_epi8(vmin, vmin, 8));
#  if 1
      const __m128i vmask = _mm_cmpeq_epi32(v, vmin);
      const int16_t mask = _mm_movemask_epi8(vmask);
      return (__builtin_ctz(mask) >> 2) + from;
#  else
      __m128i vmask = _mm_cmpeq_epi32(v, vmin);
      vmask = _mm_xor_si128(vmask, _mm_set1_epi32(-1));
      const __m128i vpos = _mm_minpos_epu16(vmask);
      return (_mm_extract_epi16(vpos, 1) >> 1) + from;
#  endif
    }

    /**
     * @brief Finds the index of the minimum child node of a given parent node.
     * Implementation optimized for 16bits integer weights .
     *
     * @param i_node The index of the parent node.
     * @return The index of the minimum child node, or -1 if the parent has no children.
     */
    int _findMinChild16bits(int i_node) const noexcept {
      const int from = firstChildOf(i_node);
      if (from >= _items.size()) return -1;
      const __m128i in = _mm_loadu_si128((__m128i*)(&_priority[from]));
      const __m128i res = _mm_minpos_epu16(in);
      return from + _mm_extract_epi16(res, 1);
    }
#else
    constexpr bool simdEnabled() const noexcept { return false; }
    constexpr bool useMinChild16bits() const noexcept { return false; }
    constexpr bool useMinChild32bits() const noexcept { return false; }
    constexpr bool useMinChildDefault() const noexcept { return true; }

    int _findMinChild32bits(int i_node) const noexcept { return _findMinChildDefault(i_node); }
    int _findMinChild16bits(int i_node) const noexcept { return _findMinChildDefault(i_node); }
#endif

    /**
     * @brief Finds the index of the minimum child node of a given parent node (default
     * implementation)
     *
     * @param iNode The index of the parent node.
     * @return The index of the minimum child node, or -1 if the parent has no children.
     */
    constexpr int _findMinChildDefault(int i_node) const noexcept {
      const int from = i_node * D + 1;
      if (from >= _items.size()) return -1;
      const int to = (i_node + 1) * D;
      int       i_min_child = from;
      for (int i = from + 1; i <= to && i < _items.size(); ++i)
        if (_priority[i] < _priority[i_min_child]) i_min_child = i;
      return i_min_child;
    }
  };

  /**
   * @brief Automatically selects the most efficient heap implementation based on compilation flags and priority type.
   *
   * This type alias provides an intelligent selection mechanism that chooses the best heap implementation:
   * - When SIMD is enabled (NT_USE_SIMD defined) and priority type is uint16_t: uses QuickHeap (8-ary heap with SIMD)
   * - Otherwise: uses BinHeapLocal (binary heap optimized for non-SIMD scenarios)
   *
   * Use this as the default heap type in your code to automatically benefit from the best available implementation
   * based on your compilation settings and priority type.
   *
   * Example:
   * @code
   * using ItemIntMap = nt::ArrayMap< int >;
   * nt::OptimalHeap< std::uint16_t, ItemIntMap > heap;  // Will use QuickHeap if SIMD enabled
   * nt::OptimalHeap< int, ItemIntMap > heap2;           // Will use BinHeapLocal
   * @endcode
   *
   * @tparam PR Type of the priorities of the items.
   * @tparam IIM Type representing the item-to-integer map used internally.
   */
#if defined(NT_USE_SIMD)
  template < class PR, class IIM >
  using OptimalHeap =
     std::conditional_t< std::is_same< PR, std::uint16_t >::value, QuickHeap< PR, IIM >, BinHeapLocal< PR, IIM > >;
#else
  template < class PR, class IIM >
  using OptimalHeap = BinHeapLocal< PR, IIM >;
#endif

}   // namespace nt

#endif
