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
 * @brief STL compatible implementation of a queue data structure.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_QUEUE_H_
#define _NT_QUEUE_H_

#include "arrays.h"

namespace nt {

  /**
   * @brief STL compatible implementation of a queue data structure.
   *
   * Example of usage:
   *
   * @code
   * int main() {
   *   // Create a queue of integers
   *   Queue<int> queue;
   *
   *   // Push elements into the queue
   *   queue.push(10);
   *   queue.push(20);
   *   queue.push(30);
   *
   *   // Access the front element
   *   std::cout << "Front element: " << queue.front() << std::endl;
   *
   *   // Check if the queue is empty
   *   std::cout << "Is queue empty? " << (queue.empty() ? "Yes" : "No") << std::endl;
   *
   *   // Get the size of the queue
   *   std::cout << "Queue size: " << queue.size() << std::endl;
   *
   *   // Pop an element from the queue
   *   queue.pop();
   *
   *   // Iterate over the elements in the queue
   *   std::cout << "Queue elements: ";
   *   for (auto it = queue.begin(); it != queue.end(); ++it) {
   *     std::cout << *it << " ";
   *   }
   *   std::cout << std::endl;
   *
   *   // Clear the queue
   *   queue.clear();
   *
   *   return 0;
   * }
   * @endcode
   *
   * @tparam T The type of elements stored in the queue.
   * @tparam ARRAY The underlying container type used to store the elements. Default is DynamicArray<T>.
   */
  template < class T, class ARRAY = DynamicArray< T > >
  struct Queue {
    using container_type = ARRAY;
    using size_type = int;
    using value_type = T;
    using reference = value_type&;
    using const_reference = const value_type&;

    using iterator = typename container_type::iterator;
    using const_iterator = typename container_type::const_iterator;

    container_type _array;
    int            _i_head;

    /**
     * @brief Default constructor. Initializes the queue with a size of zero.
     */
    constexpr Queue() : _i_head(0){};

    /**
     * @brief Constructor that initializes the queue with the given starting size.
     *
     * @param i_starting_size The initial size of the queue.
     */
    constexpr Queue(int i_starting_size) : _array(i_starting_size), _i_head(0) {}

    Queue(const Queue& other) = delete;   // < Use copyFrom()

    /**
     * @brief Returns reference to the first element in the queue. This element will be the first
     * element to be removed on a call to pop().
     *
     * @return Reference to the first element in the queue.
     */
    constexpr reference front() noexcept { return _array[_i_head]; }

    /**
     * @brief Returns constant reference to the first element in the queue. This element will be
     * the first element to be removed on a call to pop().
     *
     * @return Constant reference to the first element in the queue.
     */
    constexpr const_reference front() const noexcept { return _array[_i_head]; }

    /**
     * @brief Checks if queue has no elements.
     *
     * @return true if the queue is empty, false otherwise
     */
    constexpr bool empty() const noexcept { return size() == 0; }


    /**
     * @brief Returns the number of elements in the queue.
     *
     * @return constexpr int The number of elements in the queue.
     */
    constexpr int size() const noexcept { return _array._i_count - _i_head; }

    /**
     * @brief Pushes the given element value to the end of the queue.
     *
     * @param value The value of the element to push.
     */
    constexpr void push(const value_type& value) noexcept { _array.add(value); }

    /**
     * @brief Pushes the given rvalue reference to the end of the queue.
     *
     * @param value The rvalue reference of the element to push.
     */
    constexpr void push(value_type&& value) noexcept { _array.add(std::move(value)); }

    /**
     * @brief Removes an element from the front of the queue.
     *
     */
    constexpr void pop() noexcept { ++_i_head; }

    /**
     * @brief Allocates enough memory to store the specified number of elements.
     *
     * @param i_size The number of elements to allocate memory for.
     */
    constexpr void reserve(size_type i_size) noexcept { _array.ensure(i_size); }

    /**
     * @brief Copies the contents of another queue to this queue.
     *
     * @param from The queue to copy from.
     */
    constexpr void copyFrom(const Queue& from) noexcept {
      _array.copyFrom(from._array);
      _i_head = from._i_head;
    }

    /**
     * @brief Removes all elements from the queue.
     */
    constexpr void clear() noexcept {
      _array.removeAll();
      _i_head = 0;
    }

    /**
     * @brief Accesses the element at the specified index in the queue.
     *
     * @param i The index of the element to access.
     * @return Reference to the element at the specified index.
     */
    constexpr reference operator[](int i) noexcept { return _array[i]; }

    /**
     * @brief Accesses the element at the specified index in the queue.
     *
     * @param i The index of the element to access.
     * @return Constant reference to the element at the specified index.
     */
    constexpr const_reference operator[](int i) const noexcept { return _array[i]; }

    /**
     * @brief Returns an iterator pointing to the beginning of the queue.
     *
     * @return An iterator pointing to the beginning of the queue.
     */
    constexpr iterator begin() noexcept { return _array.begin(); }

    /**
     * @brief Returns a constant iterator pointing to the beginning of the queue.
     *
     * @return A constant iterator pointing to the beginning of the queue.
     */
    constexpr const_iterator begin() const noexcept { return _array.begin(); }

    /**
     * @brief Returns an iterator pointing to the end of the queue.
     *
     * @return An iterator pointing to the end of the queue.
     */
    constexpr iterator end() noexcept { return _array.end(); }

    /**
     * @brief Returns a constant iterator pointing to the end of the queue.
     *
     * @return A constant iterator pointing to the end of the queue.
     */
    constexpr const_iterator end() const noexcept { return _array.end(); }
  };

}   // namespace nt

#endif
