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
 * @brief STL-compatible doubly-linked list implementation.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_LIST_H_
#define _NT_LIST_H_

#include "arrays.h"

namespace nt {

  /**
   * @brief STL-compatible doubly-linked list with efficient memory management.
   *
   * Example of usage:
   *
   * @code
   * #include "networktools/core/list.h"
   *
   * int main() {
   *   nt::List<int> list;
   *   list.push_back(1);
   *   list.push_back(2);
   *   list.push_front(0);
   *
   *   for(int val : list) {
   *     std::cout << val << " ";  // Outputs: 0 1 2
   *   }
   *
   *   auto it = list.begin();
   *   ++it;
   *   list.erase(it);  // Remove element with value 1
   *
   *   return 0;
   * }
   * @endcode
   *
   * @tparam T The type of elements stored in the list.
   * @tparam K Size of the buffer
   */
  template < class T, int K = 6 >
  struct List {
    using value_type = T;
    using reference = value_type&;
    using const_reference = const value_type&;
    using size_type = int;
    using difference_type = int;

    // Node structure for doubly-linked list
    struct Node {
      value_type value;
      Node*      prev;   // Pointer to previous node (nullptr for none)
      Node*      next;   // Pointer to next node (nullptr for none)

      template < typename... Args >
      Node(Node* p, Node* n, Args&&... args) : value(std::forward< Args >(args)...), prev(p), next(n) {}
    };

    using NodeArray = FixedMemoryArray< Node, K >;

    // Bidirectional iterator
    template < bool IsConst >
    struct Iterator {
      using iterator_category = std::bidirectional_iterator_tag;
      using value_type = T;
      using difference_type = int;
      using pointer = typename std::conditional< IsConst, const T*, T* >::type;
      using reference = typename std::conditional< IsConst, const T&, T& >::type;

      using ListPtr = typename std::conditional< IsConst, const List*, List* >::type;
      using NodePtr = typename std::conditional< IsConst, const Node*, Node* >::type;

      ListPtr _list;
      NodePtr _node;

      Iterator() : _list(nullptr), _node(nullptr) {}
      Iterator(ListPtr list, NodePtr node) : _list(list), _node(node) {}

      reference operator*() const noexcept { return _node->value; }
      pointer   operator->() const noexcept { return &_node->value; }

      Iterator& operator++() noexcept {
        _node = _node->next;
        return *this;
      }

      Iterator operator++(int) noexcept {
        Iterator tmp = *this;
        ++(*this);
        return tmp;
      }

      Iterator& operator--() noexcept {
        if (_node == nullptr) {
          _node = _list->_tail;
        } else {
          _node = _node->prev;
        }
        return *this;
      }

      Iterator operator--(int) noexcept {
        Iterator tmp = *this;
        --(*this);
        return tmp;
      }

      bool operator==(const Iterator& other) const noexcept { return _node == other._node; }

      bool operator!=(const Iterator& other) const noexcept { return !(*this == other); }

      // Allow conversion from non-const to const iterator
      operator Iterator< true >() const noexcept { return Iterator< true >(_list, _node); }
    };

    using iterator = Iterator< false >;
    using const_iterator = Iterator< true >;
    using reverse_iterator = std::reverse_iterator< iterator >;
    using const_reverse_iterator = std::reverse_iterator< const_iterator >;

    NodeArray _nodes;        // Fixed-address storage for all nodes
    Node*     _head;         // Pointer to first element (nullptr if empty)
    Node*     _tail;         // Pointer to last element (nullptr if empty)
    Node*     _first_free;   // Pointer to first free node (nullptr if none)
    int       _size;         // Number of elements in the list

    /**
     * @brief Default constructor. Initializes an empty list.
     */
    constexpr List() noexcept : _head(nullptr), _tail(nullptr), _first_free(nullptr), _size(0) {}

    /**
     * @brief Constructor that pre-allocates space for a given number of elements.
     *
     * @param capacity The number of elements to pre-allocate space for.
     */
    explicit List(int capacity) : _head(nullptr), _tail(nullptr), _first_free(nullptr), _size(0) {
      _nodes.reserve(capacity);
    }

    /**
     * @brief Destructor.
     */
    ~List() = default;

    // Copy constructor
    List(const List& other) : _head(nullptr), _tail(nullptr), _first_free(nullptr), _size(0) { copyFrom(other); }

    // Copy assignment operator
    List& operator=(const List& other) {
      if (this != &other) {
        clear();
        copyFrom(other);
      }
      return *this;
    }

    // Move constructor
    List(List&& other) noexcept :
        _nodes(std::move(other._nodes)), _head(other._head), _tail(other._tail), _first_free(other._first_free),
        _size(other._size) {
      other._head = nullptr;
      other._tail = nullptr;
      other._first_free = nullptr;
      other._size = 0;
    }

    // Move assignment operator
    List& operator=(List&& other) noexcept {
      if (this != &other) {
        clear();
        _nodes = std::move(other._nodes);
        _head = other._head;
        _tail = other._tail;
        _first_free = other._first_free;
        _size = other._size;

        other._head = nullptr;
        other._tail = nullptr;
        other._first_free = nullptr;
        other._size = 0;
      }
      return *this;
    }

    /**
     * @brief Returns reference to the first element in the list.
     * @return Reference to the first element.
     */
    constexpr reference front() noexcept {
      assert(_head != nullptr);
      return _head->value;
    }

    constexpr const_reference front() const noexcept {
      assert(_head != nullptr);
      return _head->value;
    }

    /**
     * @brief Returns reference to the last element in the list.
     * @return Reference to the last element.
     */
    constexpr reference back() noexcept {
      assert(_tail != nullptr);
      return _tail->value;
    }

    constexpr const_reference back() const noexcept {
      assert(_tail != nullptr);
      return _tail->value;
    }

    /**
     * @brief Checks if list has no elements.
     * @return true if the list is empty, false otherwise.
     */
    constexpr bool empty() const noexcept { return _size == 0; }

    /**
     * @brief Returns the number of elements in the list.
     * @return The number of elements in the list.
     */
    constexpr int size() const noexcept { return _size; }

    /**
     * @brief Returns the capacity of the underlying node storage.
     * @return The capacity of the node storage array.
     */
    constexpr int capacity() const noexcept { return _nodes.capacity(); }

    /**
     * @brief Pushes the given element value to the front of the list.
     * @param value The value of the element to push.
     */
    void push_front(const value_type& value) {
      Node* new_node = _allocateNode(nullptr, _head, value);
      if (_head != nullptr) {
        _head->prev = new_node;
      } else {
        _tail = new_node;
      }
      _head = new_node;
      ++_size;
    }

    void push_front(value_type&& value) {
      Node* new_node = _allocateNode(nullptr, _head, std::move(value));
      if (_head != nullptr) {
        _head->prev = new_node;
      } else {
        _tail = new_node;
      }
      _head = new_node;
      ++_size;
    }

    /**
     * @brief Pushes the given element value to the back of the list.
     * @param value The value of the element to push.
     */
    void push_back(const value_type& value) {
      Node* new_node = _allocateNode(_tail, nullptr, value);
      if (_tail != nullptr) {
        _tail->next = new_node;
      } else {
        _head = new_node;
      }
      _tail = new_node;
      ++_size;
    }

    void push_back(value_type&& value) {
      Node* new_node = _allocateNode(_tail, nullptr, std::move(value));
      if (_tail != nullptr) {
        _tail->next = new_node;
      } else {
        _head = new_node;
      }
      _tail = new_node;
      ++_size;
    }

    /**
     * @brief Constructs element in-place at the front of the list.
     * @param args Arguments to forward to the constructor of the element.
     */
    template < typename... Args >
    reference emplace_front(Args&&... args) {
      Node* new_node = _allocateNode(nullptr, _head, std::forward< Args >(args)...);
      if (_head != nullptr) {
        _head->prev = new_node;
      } else {
        _tail = new_node;
      }
      _head = new_node;
      ++_size;
      return new_node->value;
    }

    /**
     * @brief Constructs element in-place at the back of the list.
     * @param args Arguments to forward to the constructor of the element.
     */
    template < typename... Args >
    reference emplace_back(Args&&... args) {
      Node* new_node = _allocateNode(_tail, nullptr, std::forward< Args >(args)...);
      if (_tail != nullptr) {
        _tail->next = new_node;
      } else {
        _head = new_node;
      }
      _tail = new_node;
      ++_size;
      return new_node->value;
    }

    /**
     * @brief Removes the first element from the list.
     */
    void pop_front() noexcept {
      assert(_head != nullptr);
      Node* old_head = _head;
      _head = _head->next;
      if (_head != nullptr) {
        _head->prev = nullptr;
      } else {
        _tail = nullptr;
      }
      _freeNode(old_head);
      --_size;
    }

    /**
     * @brief Removes the last element from the list.
     */
    void pop_back() noexcept {
      assert(_tail != nullptr);
      Node* old_tail = _tail;
      _tail = _tail->prev;
      if (_tail != nullptr) {
        _tail->next = nullptr;
      } else {
        _head = nullptr;
      }
      _freeNode(old_tail);
      --_size;
    }

    /**
     * @brief Inserts element before the specified position.
     * @param pos Iterator before which the element will be inserted.
     * @param value The value of the element to insert.
     * @return Iterator pointing to the inserted element.
     */
    iterator insert(iterator pos, const value_type& value) {
      if (pos._node == nullptr) {
        // Insert at end
        push_back(value);
        return iterator(this, _tail);
      } else if (pos._node == _head) {
        // Insert at front
        push_front(value);
        return iterator(this, _head);
      } else {
        // Insert in middle
        Node* prev_node = pos._node->prev;
        Node* new_node = _allocateNode(prev_node, pos._node, value);
        prev_node->next = new_node;
        pos._node->prev = new_node;
        ++_size;
        return iterator(this, new_node);
      }
    }

    iterator insert(iterator pos, value_type&& value) {
      if (pos._node == nullptr) {
        push_back(std::move(value));
        return iterator(this, _tail);
      } else if (pos._node == _head) {
        push_front(std::move(value));
        return iterator(this, _head);
      } else {
        Node* prev_node = pos._node->prev;
        Node* new_node = _allocateNode(prev_node, pos._node, std::move(value));
        prev_node->next = new_node;
        pos._node->prev = new_node;
        ++_size;
        return iterator(this, new_node);
      }
    }

    /**
     * @brief Constructs element in-place before the specified position.
     * @param pos Iterator before which the element will be inserted.
     * @param args Arguments to forward to the constructor of the element.
     * @return Iterator pointing to the inserted element.
     */
    template < typename... Args >
    iterator emplace(iterator pos, Args&&... args) {
      if (pos._node == nullptr) {
        emplace_back(std::forward< Args >(args)...);
        return iterator(this, _tail);
      } else if (pos._node == _head) {
        emplace_front(std::forward< Args >(args)...);
        return iterator(this, _head);
      } else {
        Node* prev_node = pos._node->prev;
        Node* new_node = _allocateNode(prev_node, pos._node, std::forward< Args >(args)...);
        prev_node->next = new_node;
        pos._node->prev = new_node;
        ++_size;
        return iterator(this, new_node);
      }
    }

    /**
     * @brief Erases element at the specified position.
     * @param pos Iterator to the element to remove.
     * @return Iterator following the last removed element.
     */
    iterator erase(iterator pos) noexcept {
      assert(pos._node != nullptr);
      Node* node = pos._node;
      Node* prev_node = node->prev;
      Node* next_node = node->next;

      if (prev_node != nullptr) {
        prev_node->next = next_node;
      } else {
        _head = next_node;
      }

      if (next_node != nullptr) {
        next_node->prev = prev_node;
      } else {
        _tail = prev_node;
      }

      _freeNode(node);
      --_size;
      return iterator(this, next_node);
    }

    iterator erase(const_iterator pos) noexcept { return erase(iterator(this, pos._node)); }

    /**
     * @brief Erases a range of elements.
     * @param first Iterator to the first element to remove.
     * @param last Iterator to the element following the last element to remove.
     * @return Iterator following the last removed element.
     */
    iterator erase(iterator first, iterator last) noexcept {
      while (first != last) {
        first = erase(first);
      }
      return first;
    }

    /**
     * @brief Transfers elements from another list.
     *
     * Transfers all elements from other into *this. The elements are inserted before the element
     * pointed to by pos. The container other becomes empty after the operation.
     *
     * @param pos Iterator before which the content will be inserted.
     * @param other Another list to transfer the content from.
     */
    void splice(iterator pos, List& other) noexcept {
      if (other.empty() || this == &other) return;
      splice(pos, other, other.begin(), other.end());
    }

    /**
     * @brief Transfers one element from another list.
     *
     * Transfers the element pointed to by it from other into *this. The element is inserted
     * before the element pointed to by pos.
     *
     * @param pos Iterator before which the content will be inserted.
     * @param other Another list to transfer the content from.
     * @param it Iterator to the element to transfer from other to *this.
     */
    void splice(iterator pos, List& other, iterator it) noexcept {
      if (it == other.end()) return;

      iterator next = it;
      ++next;
      splice(pos, other, it, next);
    }

    /**
     * @brief Transfers a range of elements from another list.
     *
     * Transfers the elements in the range [first, last) from other into *this. The elements
     * are inserted before the element pointed to by pos. This is a true O(1) operation per
     * element that just relinks pointers without copying or moving data.
     *
     * @param pos Iterator before which the content will be inserted.
     * @param other Another list to transfer the content from.
     * @param first Iterator to the first element in the range to transfer.
     * @param last Iterator to the element following the last element to transfer.
     */
    void splice(iterator pos, List& other, iterator first, iterator last) noexcept {
      if (first == last) return;

      // Find the last node in the range
      Node* first_node = first._node;
      Node* last_node = last._node;
      Node* range_last = first_node;
      int   count = 1;

      while (range_last->next != last_node) {
        range_last = range_last->next;
        ++count;
      }

      // Unlink the range from other list
      Node* before_first = first_node->prev;
      Node* after_last = range_last->next;

      if (before_first != nullptr) {
        before_first->next = after_last;
      } else {
        other._head = after_last;
      }

      if (after_last != nullptr) {
        after_last->prev = before_first;
      } else {
        other._tail = before_first;
      }

      other._size -= count;

      // Insert the range into this list
      Node* pos_node = pos._node;

      if (pos_node == nullptr) {
        // Insert at end
        if (_tail != nullptr) {
          _tail->next = first_node;
          first_node->prev = _tail;
        } else {
          _head = first_node;
          first_node->prev = nullptr;
        }
        range_last->next = nullptr;
        _tail = range_last;
      } else if (pos_node == _head) {
        // Insert at beginning
        range_last->next = _head;
        _head->prev = range_last;
        first_node->prev = nullptr;
        _head = first_node;
      } else {
        // Insert in middle
        Node* before_pos = pos_node->prev;
        before_pos->next = first_node;
        first_node->prev = before_pos;
        range_last->next = pos_node;
        pos_node->prev = range_last;
      }

      _size += count;
    }

    /**
     * @brief Pre-allocates memory for the specified number of elements.

     * @param capacity The number of elements to allocate memory for (unused).
     */
    void reserve(size_type capacity) { _nodes.reserve(capacity); }

    /**
     * @brief Copies the contents of another list to this list.
     * @param from The list to copy from.
     */
    void copyFrom(const List& from) {
      clear();
      for (const auto& val: from) {
        push_back(val);
      }
    }

    /**
     * @brief Removes all elements from the list.
     */
    void clear() noexcept {
      _head = nullptr;
      _tail = nullptr;
      _first_free = nullptr;
      _size = 0;
      _nodes.removeAll();
    }

    /**
     * @brief Returns an iterator pointing to the first element.
     * @return Iterator to the first element.
     */
    constexpr iterator begin() noexcept { return iterator(this, _head); }

    constexpr const_iterator begin() const noexcept { return const_iterator(this, _head); }

    constexpr const_iterator cbegin() const noexcept { return const_iterator(this, _head); }

    /**
     * @brief Returns an iterator pointing to the past-the-end element.
     * @return Iterator to the past-the-end element.
     */
    constexpr iterator end() noexcept { return iterator(this, nullptr); }

    constexpr const_iterator end() const noexcept { return const_iterator(this, nullptr); }

    constexpr const_iterator cend() const noexcept { return const_iterator(this, nullptr); }

    /**
     * @brief Returns a reverse iterator pointing to the last element.
     * @return Reverse iterator to the last element.
     */
    reverse_iterator rbegin() noexcept { return reverse_iterator(end()); }

    const_reverse_iterator rbegin() const noexcept { return const_reverse_iterator(end()); }

    const_reverse_iterator crbegin() const noexcept { return const_reverse_iterator(cend()); }

    /**
     * @brief Returns a reverse iterator pointing to the theoretical element preceding the first element.
     * @return Reverse iterator to the theoretical element preceding the first element.
     */
    reverse_iterator rend() noexcept { return reverse_iterator(begin()); }

    const_reverse_iterator rend() const noexcept { return const_reverse_iterator(begin()); }

    const_reverse_iterator crend() const noexcept { return const_reverse_iterator(cbegin()); }

    // Private methods

    // Allocate a new node (from free-list or by growing the array)
    template < typename... Args >
    Node* _allocateNode(Node* prev, Node* next, Args&&... args) {
      if (_first_free != nullptr) {
        // Reuse node from free-list
        Node* node = _first_free;
        _first_free = node->next;
        // Reconstruct node in-place
        node->~Node();
        new (node) Node(prev, next, std::forward< Args >(args)...);
        return node;
      } else {
        // Allocate new node - address is guaranteed to be stable
        int idx = _nodes.add(prev, next, std::forward< Args >(args)...);
        return &_nodes[idx];
      }
    }

    // Free a node (add to free-list)
    void _freeNode(Node* node) noexcept {
      node->next = _first_free;
      _first_free = node;
    }
  };

}   // namespace nt

#endif
