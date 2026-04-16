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
 * @brief STL compatible implementation of a stack data structure.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_STACK_H_
#define _NT_STACK_H_

#include "arrays.h"

namespace nt {

  /**
   * @brief STL compatible implementation of a stack data structure.
   *
   * Example of usage :
   *
   * @code
   *   int main() {
   *     // Create a Stack of integers
   *     Stack<int> stack;
   *
   *     // Push elements onto the stack
   *     stack.push(10);
   *     stack.push(20);
   *     stack.push(30);
   *
   *     // Access the top element of the stack
   *     int topElement = stack.top();
   *     std::cout << "Top element: " << topElement << std::endl;
   *
   *     // Pop elements from the stack
   *     stack.pop();
   *
   *     // Check if the stack is empty
   *     bool isEmpty = stack.empty();
   *     std::cout << "Is stack empty? " << (isEmpty ? "Yes" : "No") << std::endl;
   *
   *     // Iterate over the elements in the stack
   *     for (int i = 0; i < stack.size(); i++) {
   *       std::cout << "Element " << i << ": " << stack[i] << std::endl;
   *     }
   *
   *     // Clear the stack
   *     stack.clear();
   *
   *     return 0;
   *   }
   * @endcode
   *
   * @tparam T The type of elements stored in the stack.
   * @tparam ARRAY The type of container used for internal storage. Default is
   * `nt::DynamicArray<T>`.
   */
  template < class T, class ARRAY = nt::DynamicArray< T > >
  struct Stack {
    using container_type = ARRAY;
    using size_type = int;
    using value_type = T;
    using reference = value_type&;
    using const_reference = const value_type&;

    container_type _array;

    /**
     * @brief Default constructor. Constructs an empty stack.
     */
    constexpr Stack() = default;

    /**
     * @brief Constructs a stack with the specified starting size.
     *
     * @param i_starting_size The starting size of the stack.
     */
    constexpr Stack(int i_starting_size) noexcept : _array(i_starting_size) {}


    Stack(const Stack& other) = delete;   // < Use copyFrom()

    /**
     * @brief Returns a reference to the top element in the stack.
     *
     * @return A reference to the top element in the stack.
     */
    constexpr reference top() noexcept { return _array[head()]; }

    /**
     * @brief Returns a const reference to the top element in the stack.
     *
     * @return A const reference to the top element in the stack.
     */
    constexpr const_reference top() const noexcept { return _array[head()]; }

    /**
     * @brief Checks if the stack has no elements.
     *
     * @return true if the stack is empty, false otherwise.
     */
    constexpr bool empty() const noexcept { return size() == 0; }

    /**
     * @brief Returns the index of the top element in the stack.
     *
     * @return The index of the top element in the stack.
     */
    constexpr int head() const noexcept {
      assert(!empty());
      return _array._i_count - 1;
    }

    /**
     * @brief Returns the number of elements in the stack.
     *
     * @return The number of elements in the stack.
     */
    constexpr int size() const noexcept { return _array._i_count; }

    /**
     * @brief Inserts a new element at the top of the stack, above its current top element.
     *
     * @param value The value to be inserted.
     */
    constexpr void push(const value_type& value) noexcept { _array.add(value); }
    constexpr void push(value_type&& value) noexcept { _array.add(std::move(value)); }

    /**
     * @brief Removes the element on top of the stack, effectively reducing its size by one.
     *
     * The element removed is the latest element inserted into the stack, whose value can be
     * retrieved by calling member Stack::top.
     *
     */
    constexpr void pop() noexcept { _array.removeLast(); }

    /**
     * @brief Removes the element on top of the stack and returns it.
     *
     * @return A copy of the top element in the stack.
     */
    constexpr value_type topAndPop() noexcept {
      const value_type e = top();
      pop();
      return e;
    }

    /**
     * @brief Accesses the element at the specified index in the stack.
     *
     * @param i The index of the element.
     * @return A const reference to the element at the specified index.
     */
    constexpr const value_type& operator[](int i) const noexcept { return _array[i]; }

    /**
     * @brief Allocates enough memory to store the specified number of elements.
     *
     * @param i_size The number of elements to allocate memory for.
     */
    void reserve(size_type i_size) noexcept { _array.ensure(i_size); }

    /**
     * @brief Copies the contents of another stack into this stack.
     *
     * @param from The stack to copy from.
     */
    constexpr void copyFrom(const Stack& from) noexcept { _array.copyFrom(from._array); }

    /**
     * @brief Removes all elements from the stack, leaving it empty.
     */
    constexpr void clear() noexcept { _array.removeAll(); }
  };
}   // namespace nt

#endif
