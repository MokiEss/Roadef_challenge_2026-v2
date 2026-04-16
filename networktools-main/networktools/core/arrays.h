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
 * @file arrays.h
 * @brief Self-contained header that offers various and efficient array implementations
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 *
 * @section array_guide ARRAY SELECTION GUIDE
 *
 * @subsection intro_sec Introduction
 *
 * This self-contained header offers various and efficient array implementations
 * that can be used independently of networktools. When choosing the appropriate
 * array type, consider your specific requirements and constraints. This guide will
 * help you navigate through the selection process.
 *
 * @subsection static_dynamic_sec I. Static or Dynamic Array?
 *
 * Decide whether you need an array with a dynamic size or a fixed size.
 *
 * @par A. Static Array
 * - Use StaticArray when you know the size of the array at compile time.
 * - Use FixedSizeArray when you know the size of the array only at runtime.
 *
 * @par B. Dynamic Array
 * Choose from the following options based on the stored value type:
 *
 * @par 1. LazyDynamicArray
 * - Mandatory for storing objects that require their constructor/destructor to be called.
 * - Optionally implements Small Size Optimization for efficiency.
 *
 * @par 2. TrivialDynamicArray
 * - Suitable for PODs (Plain Old Data) or native types (e.g., int, float).
 * - Optionally implements Small Size Optimization for efficiency.
 *
 * @par 3. DynamicArray
 * - Use when it's uncertain whether TrivialDynamicArray or LazyDynamicArray is suitable.
 * - Optionally implements Small Size Optimization for efficiency.
 *
 * @par 4. FixedMemoryArray
 * - Opt for this when you need to store elements that will be later accessed by their address.
 *
 * @par 5. SymmetricArray
 * - A symmetric array is a special kind array in which each element in position i
 * has a "symmetric" element in position ~i.
 *
 * @subsection sso_sec II. Small Size Optimization (SSO)
 *
 * Some arrays have a special template parameter 'SSO'. This parameter specifies the size of
 * pre-allocated memory on the stack. It is used for Small Size Optimization (SSO), which
 * can improve performance for small arrays. You need to set this parameter to a strictly
 * positive value to activate SSO.
 *
 * @subsection multidim_sec III. Multi-dimensional Array
 *
 * If you require a multi-dimensional array, consider using MultiDimArray, TrivialMultiDimArray
 * or LazyMultiDimArray.
 *
 * @subsection arrayview_sec IV. Array View
 *
 * ArrayView is useful when only a read-access to an array is needed whatever its type (StaticArray,
 * LazyDynamicArray, etc). It provides a unified API for traversing non-modifiable arrays.
 */

#ifndef _NT_ARRAYS_H_
#define _NT_ARRAYS_H_
#include <type_traits>
#include <cstdlib>    // malloc(), realloc(), free(), size_t
#include <cstring>    // memset(), memcpy(), memmove(), memcmp()
#include <cassert>    // assert()
#include <cstddef>    // ptrdiff_t
#include <cstdint>    // std::int8_t, std::int16_t, ...
#include <utility>    // move()
#include <iterator>   // std::reverse_iterator
#include <bit>        // std::popcount
#include <limits>     // std::numeric_limits

#if TRACY_ENABLE
#  include <tracy/Tracy.hpp>
#endif

// Macros to allow custom memory allocators
#ifndef NT_MALLOC
#  define NT_MALLOC(sz) std::malloc(sz)
#endif
#ifndef NT_CALLOC
#  define NT_CALLOC(n, sz) std::calloc(n, sz)
#endif
#ifndef NT_REALLOC
#  define NT_REALLOC(ptr, sz) std::realloc(ptr, sz)
#endif
#ifndef NT_MEMMOVE
#  define NT_MEMMOVE(dest, src, sz) std::memmove(dest, src, sz)
#endif
#ifndef NT_FREE
#  define NT_FREE(ptr) std::free(ptr)
#endif

// These macros are part of the common.h file put redefined here to make arrays.h dependency-free
#define NT_SIZEOF_BITS_(X) (sizeof(X) * 8)
#define NT_UINT64_MAX_ ((std::numeric_limits< std::uint64_t >::max)())
#define NT_MODULO_64_(x) ((x)&63)
#define NT_DIVIDE_BY_64_(x) ((x) >> 6)

#if defined(NT_USE_SIMD)
#  if (_MSC_VER >= 1800)
#    include <intrin.h>
#  else
#    include <x86intrin.h>
#  endif
#endif

#if defined(NT_USE_SIMD)
#  define NT_SIMD_CONSTEXPR_
#else
#  define NT_SIMD_CONSTEXPR_ constexpr
#endif

namespace nt {

  /**
   * @class ArrayView
   * @headerfile arrays.h
   * @brief This class provides a read-only access to any existing sequence container without
   * making a copy. A sequence container is a container that stores objects of the same type in a
   * linear arrangement like DynamicArray, std::vector, std::array, ...
   *
   * This class is especially useful when it comes to define a function that only needs read
   * access to an array whatever its type is.
   *
   * Example of usage:
   *
   * @code
   * #include <iostream>
   * #include <vector>
   * #include "networktools/core/arrays.h"
   *
   *  void printValues(ArrayView<int> a) {
   *    for(int i = 0; i < a.size(); ++i)
   *      std::cout << a[i] << std::endl;
   *  }
   *
   * int main() {
   *  nt::IntDynamicArray arr1   = { 1, 2, 3};
   *  std::vector<int>    arr2   = { 1, 2, 3};
   *  int                 arr3[] = { 1, 2, 3};
   *
   *  printValues(arr1);
   *  printValues(arr2);
   *  printValues(arr3);
   * }
   * @endcode
   *
   * @warning ArrayView is a non-owning view and does not manage the lifetime of the data it references.
   * You must ensure that the underlying data remains valid for the entire lifetime of the ArrayView.
   * Creating an ArrayView from a temporary object results in a dangling reference and leads to
   * undefined behavior.
   *
   * Example of INCORRECT usage (dangling reference):
   * @code
   * // BAD: ArrayView references temporary array returned by getPath()
   * nt::ArrayView<int> view = getPath();  // Temporary destroyed at end of statement
   * int value = view[0];                  // Undefined behavior: accessing freed memory
   * @endcode
   *
   * Example of CORRECT usage:
   * @code
   * // GOOD: Store the array first, then create a view
   * auto path = getPath();
   * nt::ArrayView<int> view = path;
   * int value = view[0];  // OK: path is still alive
   *
   * // GOOD: Pass temporary directly to function (view lifetime matches temporary lifetime)
   * my_array.copyFrom(getPath());  // OK: temporary lives until copyFrom() completes
   * @endcode
   *
   * @tparam TYPE The type of the stored values
   *
   */
  template < class TYPE >
  struct ArrayView {
    using value_type = TYPE;
    using reference = value_type&;
    using const_reference = const value_type&;
    using pointer = value_type*;
    using const_pointer = const value_type*;
    using size_type = int;
    using difference_type = ptrdiff_t;
    using iterator = pointer;
    using const_iterator = const_pointer;
    using reverse_iterator = std::reverse_iterator< const_iterator >;
    using const_reverse_iterator = std::reverse_iterator< const_iterator >;

    const int     _i_count;
    const_pointer _p_datas;

    constexpr ArrayView() : _i_count(0), _p_datas(nullptr) {}
    constexpr explicit ArrayView(const_pointer p_datas, int i_count) noexcept : _i_count(i_count), _p_datas(p_datas) {}
    template < class ARRAY >
    constexpr ArrayView(const ARRAY& array) noexcept : ArrayView(array.data(), array.size()) {}
    template < size_t N >
    constexpr ArrayView(const value_type (&p_datas)[N]) noexcept : ArrayView(p_datas, N) {}

    /**
     * @brief Returns a reference to the element at specified location index.
     *
     * @param index Position of the element to return
     * @return Reference to the requested element.
     */
    constexpr const_reference operator[](int index) const noexcept {
      assert(index < _i_count && index >= 0);
      return _p_datas[index];   // MSVC 2015 : *(_p_datas+index)
    }

    /**
     * @brief Returns a reference to the element at specified location index.
     *        Same as the operator[]
     *
     * @param index Position of the element to return
     * @return Reference to the requested element
     */
    constexpr const_reference at(int index) const noexcept { return operator[](index); }

    /**
     * @brief Returns the position to the first element equals to value.
     *
     * @param value Value to compare the elements to.
     * @return int Position of the first element equals to value or _i_count if no such element is
     * found.
     */
    constexpr int find(const value_type& value) const noexcept {
      int i = 0;
      for (; i < size(); ++i)
        if (_p_datas[i] == value) break;
      return i;
    }

    //---------------------------------------------------------------------------
    // STL methods
    //---------------------------------------------------------------------------

    // Iterators
    constexpr const_iterator begin() const noexcept { return _p_datas; }
    constexpr const_iterator end() const noexcept { return _p_datas + _i_count; }

    constexpr const_iterator cbegin() const noexcept { return _p_datas; }
    constexpr const_iterator cend() const noexcept { return _p_datas + _i_count; }

    constexpr const_reverse_iterator rbegin() const noexcept { return const_reverse_iterator(end()); }
    constexpr const_reverse_iterator rend() const noexcept { return const_reverse_iterator(begin()); }

    constexpr const_reverse_iterator crbegin() const noexcept { return const_reverse_iterator(cend()); }
    constexpr const_reverse_iterator crend() const noexcept { return const_reverse_iterator(cbegin()); }

    // Capacity
    constexpr size_type size() const noexcept { return _i_count; }
    constexpr size_type capacity() const noexcept { return _i_count; }
    constexpr bool      empty() const noexcept { return _i_count == 0; }
    constexpr size_type max_size() const noexcept { return _i_count; }

    // Element access
    constexpr const_pointer data() const noexcept { return _p_datas; }

    constexpr const_reference back() const noexcept { return operator[](_i_count - 1); }
    constexpr const_reference front() const noexcept { return operator[](0); }
  };

  /**
   * @class FixedSizeArray
   * @headerfile arrays.h
   * @brief This class implements an immutable array whose size is known only at run time.
   *
   * Example of usage:
   *
   * @code
   * #include "networktools/core/arrays.h"
   *
   * int main(int argc, char** argv) {
   *  nt::FixedSizeArray<const char*> arr(argc);
   *
   *  for(int i = 0; i < arr.size(); ++i)
   *    arr[i] = argv[i];
   * }
   * @endcode
   *
   * @tparam TYPE The type of the stored values
   */
  template < class TYPE >
  struct FixedSizeArray {
    using value_type = TYPE;
    using reference = value_type&;
    using const_reference = const value_type&;
    using pointer = value_type*;
    using const_pointer = const value_type*;
    using size_type = int;
    using difference_type = ptrdiff_t;
    using iterator = pointer;
    using const_iterator = const_pointer;

    const int _i_size;
    pointer   _p_datas;

    template < typename... Args >
    FixedSizeArray(int i_size, Args&&... args) noexcept : _i_size(i_size) {
      assert(i_size > 0);
      _p_datas = static_cast< pointer >(NT_MALLOC(i_size * sizeof(value_type)));
      assert(_p_datas);
      if (_p_datas)
        for (int i = 0; i < _i_size; ++i)
          new (_p_datas + i) value_type(std::forward< Args >(args)...);
    }
    FixedSizeArray(FixedSizeArray&& other) noexcept : _i_size(other._i_size), _p_datas(other._p_datas) {
      other._p_datas = nullptr;
    }
    FixedSizeArray& operator=(FixedSizeArray&& other) noexcept {
      if (this == &other) return *this;

      for (int i = _i_size - 1; i >= 0; --i)
        _p_datas[i].~value_type();
      NT_FREE(_p_datas);

      const_cast< int& >(_i_size) = other._i_size;
      _p_datas = other._p_datas;

      other._p_datas = nullptr;

      return *this;
    }

    ~FixedSizeArray() noexcept {
      for (int i = _i_size - 1; i >= 0; --i)
        _p_datas[i].~value_type();
      NT_FREE(_p_datas);
      _p_datas = nullptr;
    }

    // Copy constructor/assignment are deleted to prevent expensive copies
    FixedSizeArray(const FixedSizeArray&) = delete;
    FixedSizeArray& operator=(const FixedSizeArray&) = delete;

    /**
     * @brief Returns a reference to the element at specified location index.
     *
     * @param index Position of the element to return
     * @return Reference to the requested element.
     */
    constexpr reference operator[](int index) noexcept {
      assert(index < _i_size && index >= 0);
      return _p_datas[index];
    }

    constexpr const_reference operator[](int index) const noexcept {
      assert(index < _i_size && index >= 0);
      return _p_datas[index];
    }

    /**
     * @brief Returns a reference to the element at specified location index.
     *        Same as the operator[]
     *
     * @param index Position of the element to return
     * @return Reference to the requested element
     */
    constexpr reference       at(int index) noexcept { return operator[](index); }
    constexpr const_reference at(int index) const noexcept { return operator[](index); }

    /**
     * @brief Set the given value to the element at specified location index.
     *
     */
    constexpr void set(int index, const value_type& v) noexcept { operator[](index) = v; }
    constexpr void set(int index, value_type&& v) noexcept { operator[](index) = std::move(v); }

    //---------------------------------------------------------------------------
    // STL methods
    //---------------------------------------------------------------------------

    // Iterators
    constexpr iterator       begin() noexcept { return _p_datas; }
    constexpr const_iterator begin() const noexcept { return _p_datas; }
    constexpr const_iterator cbegin() const noexcept { return _p_datas; }

    constexpr iterator       end() noexcept { return _p_datas + _i_size; }
    constexpr const_iterator end() const noexcept { return _p_datas + _i_size; }
    constexpr const_iterator cend() const noexcept { return _p_datas + _i_size; }

    // Capacity
    constexpr size_type size() const noexcept { return _i_size; }
    constexpr size_type capacity() const noexcept { return _i_size; }
    constexpr bool      empty() const noexcept { return _i_size == 0; }
    constexpr size_type max_size() const noexcept { return _i_size; }

    // Element access
    constexpr pointer       data() noexcept { return _p_datas; }
    constexpr const_pointer data() const noexcept { return _p_datas; }

    constexpr reference       back() noexcept { return operator[](_i_size - 1); }
    constexpr const_reference back() const noexcept { return operator[](_i_size - 1); }

    constexpr reference       front() noexcept { return operator[](0); }
    constexpr const_reference front() const noexcept { return operator[](0); }
  };

  /**
   * @class StaticArray
   * @headerfile arrays.h
   * @brief An array which is not mutable and whose size can be determined at compile time.
   *
   * Example of usage:
   *
   * @code
   * #include "networktools/core/arrays.h"
   *
   * int main(int argc, char** argv) {
   *  nt::StaticArray<int, 3> arr; // Create an array of ints of size 3
   *  arr[0] = 1000;
   *  arr[2] = 1001;
   *  arr[3] = 1002;
   * }
   * @endcode
   *
   * @tparam TYPE The type of the stored values
   * @tparam N The maximum number of elements the array can accomodate
   */
  template < class TYPE, int N >
  struct StaticArray {
    using value_type = TYPE;
    using size_type = int;
    using reference = TYPE&;
    using const_reference = const TYPE&;
    using pointer = value_type*;
    using const_pointer = const value_type*;
    using difference_type = ptrdiff_t;
    using iterator = pointer;
    using const_iterator = const_pointer;
    using reverse_iterator = std::reverse_iterator< iterator >;
    using const_reverse_iterator = std::reverse_iterator< const_iterator >;

    int        _i_count;
    value_type _p_data[N];

    constexpr StaticArray() noexcept : _i_count(0) {}
    constexpr StaticArray(bool b_fill) noexcept : _i_count(b_fill ? N : 0) {}
    constexpr StaticArray(StaticArray&& other) noexcept : _i_count(other._i_count) {
      for (int i = 0; i < _i_count; ++i)
        _p_data[i] = std::move(other._p_data[i]);
      other._i_count = 0;
    }
    constexpr StaticArray& operator=(StaticArray&& other) noexcept {
      if (this == &other) return *this;
      _i_count = other._i_count;
      for (int i = 0; i < _i_count; ++i)
        _p_data[i] = std::move(other._p_data[i]);
      other._i_count = 0;
      return *this;
    }

    /**
     * @brief Returns a reference to the element at specified location index.
     *
     * @param index  	Position of the element to return
     * @return Reference to the requested element.
     */
    constexpr reference operator[](int index) noexcept {
      assert(index < _i_count && index >= 0);
      return _p_data[index];
    }

    constexpr const_reference operator[](int index) const noexcept {
      assert(index < _i_count && index >= 0);
      return _p_data[index];
    }

    /**
     * @brief Returns a reference to the element at specified location index.
     *        Same as the operator[]
     *
     * @param index Position of the element to return
     * @return Reference to the requested element
     */
    constexpr reference       at(int index) noexcept { return operator[](index); }
    constexpr const_reference at(int index) const noexcept { return operator[](index); }

    /**
     * @brief Set the given value to the element at specified location index.
     *
     */
    constexpr void set(int index, const value_type& v) noexcept { operator[](index) = v; }
    constexpr void set(int index, value_type&& v) noexcept { operator[](index) = std::move(v); }

    /**
     * @brief Inserts a default created element into the array.
     *
     * @return Returns the position of the inserted element.
     */
    constexpr int add() noexcept {
      assert(_i_count < N);
      return _i_count++;
    }

    /**
     * @brief Inserts the provided value into the array.
     *
     * @return Returns the position of the inserted element.
     */
    constexpr int add(const value_type& value) noexcept {
      const int k = add();
      set(k, value);
      return k;
    }

    constexpr int add(value_type&& value) noexcept {
      const int k = add();
      set(k, std::move(value));
      return k;
    }

    /**
     * @brief Removes the element at position index from the array.
     *
     * @param index Position to the element to remove.
     */
    constexpr void remove(int index) noexcept {
      assert(index < _i_count && index >= 0);
      --_i_count;
      if (index != _i_count) _p_data[index] = std::move(_p_data[_i_count]);
    }

    /**
     * @brief Check whether the array is full
     *
     */
    constexpr bool full() const noexcept { return _i_count >= N; }

    /**
     * @brief Fills the array with elements
     *
     */
    constexpr void fill() noexcept { _i_count = N; }

    /**
     * @brief Removes all elements from the array.
     *
     */
    constexpr void removeAll() noexcept { _i_count = 0; }

    /**
     * @brief Removes the last element from the array.
     *
     */
    constexpr void removeLast() noexcept { remove(_i_count - 1); }

    /**
     * @brief Returns the position to the first element equals to value.
     *
     * @param value Value to compare the elements to.
     * @return Position of the first element equals to value or Count if no such element is found.
     */
    constexpr int find(const value_type& value) const noexcept {
      int i = 0;
      for (; i < _i_count; ++i)
        if (_p_data[i] == value) break;
      return i;
    }

    //---------------------------------------------------------------------------
    // STL methods
    //---------------------------------------------------------------------------

    // Iterators
    constexpr iterator       begin() noexcept { return _p_data; }
    constexpr const_iterator begin() const noexcept { return _p_data; }
    constexpr const_iterator cbegin() const noexcept { return _p_data; }

    constexpr iterator       end() noexcept { return _p_data + _i_count; }
    constexpr const_iterator end() const noexcept { return _p_data + _i_count; }
    constexpr const_iterator cend() const noexcept { return _p_data + _i_count; }

    constexpr reverse_iterator       rbegin() noexcept { return reverse_iterator(end()); }
    constexpr const_reverse_iterator rbegin() const noexcept { return const_reverse_iterator(end()); }
    constexpr const_reverse_iterator crbegin() const noexcept { return const_reverse_iterator(cend()); }

    constexpr reverse_iterator       rend() noexcept { return reverse_iterator(begin()); }
    constexpr const_reverse_iterator rend() const noexcept { return const_reverse_iterator(begin()); }
    constexpr const_reverse_iterator crend() const noexcept { return const_reverse_iterator(cbegin()); }

    // Capacity
    constexpr size_type size() const noexcept { return _i_count; }
    constexpr size_type capacity() const noexcept { return N; }
    constexpr bool      empty() const noexcept { return _i_count == 0; }
    constexpr size_type max_size() const noexcept { return N; }

    // Modifiers
    constexpr void push_back(const value_type& val) noexcept { add(val); }
    constexpr void push_back(value_type&& val) noexcept { add(std::move(val)); }
    constexpr void pop_back() noexcept { removeLast(); }

    // Element access
    constexpr pointer       data() noexcept { return _p_data; }
    constexpr const_pointer data() const noexcept { return _p_data; }

    constexpr reference       back() noexcept { return operator[](_i_count - 1); }
    constexpr const_reference back() const noexcept { return operator[](_i_count - 1); }

    constexpr reference       front() noexcept { return operator[](0); }
    constexpr const_reference front() const noexcept { return operator[](0); }

    void clear() noexcept { removeAll(); }
  };

  namespace details {
    template < class TYPE, int N >
    struct SmallSizeOptim {
      TYPE _static_datas[N];
    };

    template < class TYPE >
    struct SmallSizeOptim< TYPE, 0 > {
      constexpr static TYPE* _static_datas = nullptr;
    };

    /**
     * @class DynamicArrayBase
     * @headerfile arrays.h
     * @brief Base class for the dynamic array types that should not be used directly. Consider using DynamicArray
     * instead.
     *
     * @tparam TYPE The type of the stored values
     * @tparam MEMOP Low-level memory operations
     * @tparam GF_NUM Numerator value of the grow factor
     * @tparam GF_DEN Denominator value of the grow factor
     * @tparam SSO The size of the pre-allocated memory on the stack
     *
     */
    template < class TYPE, class MEMOP, const int GF_NUM, const int GF_DEN, int SSO >
    struct DynamicArrayBase: details::SmallSizeOptim< TYPE, SSO > {
      static_assert(SSO >= 0, "DynamicArrayBase : SSO must be >= 0");
      static_assert(SSO == 0 || std::is_default_constructible< TYPE >::value,
                    "DynamicArrayBase : SSO activated but the type is not default constructible.");
      static_assert(std::is_move_constructible< TYPE >::value,
                    "DynamicArrayBase : The type must be move constructible.");

      using Parent = details::SmallSizeOptim< TYPE, SSO >;
      using value_type = TYPE;
      using reference = value_type&;
      using const_reference = const value_type&;
      using pointer = value_type*;
      using const_pointer = const value_type*;
      using size_type = int;
      using difference_type = ptrdiff_t;
      using iterator = pointer;
      using const_iterator = const_pointer;
      using reverse_iterator = std::reverse_iterator< iterator >;
      using const_reverse_iterator = std::reverse_iterator< const_iterator >;

      int     _i_count;      //< Number of stored elements
      int     _i_capacity;   //< Maximum number of elements the array can accomodate before resize
      pointer _p_data;

      /**
       * @brief Construct an empty array
       *
       */
      constexpr DynamicArrayBase() noexcept : _i_count(0), _i_capacity(SSO), _p_data(Parent::_static_datas) {}

      /**
       * @brief Constructor that initializes the array with i_size unitialized elements.
       *
       * @param i_size The initial size of the array.
       */
      explicit DynamicArrayBase(int i_size) noexcept : _i_count(0), _i_capacity(SSO), _p_data(Parent::_static_datas) {
        ensureAndFill(i_size);
      }

      /**
       * @brief Constructs an array with i_size copies of elements with value v
       *
       * @param i_size The initial size of the array.
       * @param v The value to be copied
       */
      explicit DynamicArrayBase(int i_size, const value_type& v) noexcept :
          _i_count(0), _i_capacity(SSO), _p_data(Parent::_static_datas) {
        ensureAndFill(i_size, v);
      }

      /**
       * @brief Destroy the Dynamic Array Base object
       *
       */
      ~DynamicArrayBase() noexcept { reset(); }

      /**
       * @brief Move constructor
       *
       * This function is less trivial than one would expect since we need to handle the cases where
       * other._p_data is either allocated on the heap or on the stack.
       *
       * @param other The array to move-construct from
       */
      DynamicArrayBase(DynamicArrayBase&& other) noexcept :
          _i_count(other._i_count), _i_capacity(other._i_capacity), _p_data(Parent::_static_datas) {
        // If other has allocated memory, then just steal other's buffer.
        if (other.isAllocated()) {
          _p_data = other._p_data;
        } else {
          // _p_data already points to Parent::_static_datas
          *(static_cast< Parent* >(this)) = std::move(*static_cast< Parent* >(&other));
        }

        // Leave other in a valid state
        other._i_count = 0;
        other._i_capacity = SSO;
        other._p_data = other._static_datas;
      }

      /**
       * @brief Move assignment
       *
       * @param other The array to move-assign from
       *
       * @return DynamicArrayBase& A reference to this array
       */
      DynamicArrayBase& operator=(DynamicArrayBase&& other) noexcept {
        if (this == &other) return *this;
        reset();

        _i_count = other._i_count;
        _i_capacity = other._i_capacity;

        // If 'other' has allocated memory, then just steal other's buffer.
        if (other.isAllocated()) {
          _p_data = other._p_data;
        } else {
          // 'this' was already reset above, so it's empty and '_p_data' points back to
          // 'Parent::_static_datas'.
          *(static_cast< Parent* >(this)) = std::move(*static_cast< Parent* >(&other));
        }

        // Leave 'other' in a valid state
        other._i_count = 0;
        other._i_capacity = SSO;
        other._p_data = other._static_datas;

        return *this;
      }

      // Copy constructor/operator are deleted to enforce the use of copyFrom() (or
      // shallowCopyFrom()). This design choice prevents accidental (and possibly costly) copies
      // with the = operator.
      DynamicArrayBase(const DynamicArrayBase&) = delete;
      DynamicArrayBase& operator=(const DynamicArrayBase&) = delete;

      /**
       * @brief Inserts a default created element to the end of the array
       *
       * @return Returns the position of the inserted element.
       */
      constexpr int add() noexcept { return addEmplace(); }

      /**
       * @brief Inserts the provided value to the end of the array
       *
       * @return Returns the position of the inserted element.
       */
      constexpr int add(const value_type& v) noexcept {
        assert((&v < data())
               || (&v > data() + capacity())
                     && "Attempting to insert an element whose memory location is in the array memory range");
        return addEmplace(v);
      }
      constexpr int add(value_type&& v) noexcept { return addEmplace(std::move(v)); }

      /**
       * @brief Create and inserts a constructible element to the end of the array
       *
       * @return Returns the position of the inserted element.
       */
      template < typename... Args >
      constexpr int addEmplace(Args&&... args) noexcept {
        _growIfNecessary();
        MEMOP::constructAt(_p_data, _i_count, std::forward< Args >(args)...);
        return _i_count++;
      }

      /**
       * @brief Construct a DynamicArrayBase object with its value being the
       * concatenation of the values in `this` followed by those of `other`.
       *
       * @return The newly constructed array.
       */
      DynamicArrayBase operator+(const ArrayView< value_type >& other) const noexcept {
        DynamicArrayBase result;

        if (other.empty()) {
          result.copyFrom(*this);
          return result;
        }

        const int i_size = size() + other.size();

        result.ensure(i_size);
        MEMOP::copy(result.data(), data(), size());
        MEMOP::copy(result.data() + size(), other.data(), other.size());
        result._i_count = i_size;

        return result;
      }

      /**
       * @brief Append the values contained in other to the end of the array.
       *
       */
      void append(const ArrayView< value_type >& other) noexcept {
        if (other.empty()) return;
        const int i_size = size() + other.size();
        ensure(i_size);
        MEMOP::copy(data() + size(), other.data(), other.size());
        _i_count = i_size;
      }

      /**
       * @brief Replaces the contents with a copy of the contents of other.
       *
       */
      constexpr void copyFrom(const ArrayView< value_type >& other) noexcept {
        if (other.empty()) {
          removeAll();
        } else {
          ensure(other.size());
          MEMOP::copy(_p_data, other.data(), other.size());
          _i_count = other.size();
        }
      }

      /**
       * @brief Replaces the contents with a copy of the elements in the range defined by [first, last).
       *
       */
      constexpr void copyFrom(const_iterator first, const_iterator last) noexcept {
        if (first == last) {
          removeAll();
        } else {
          const int i_size = last - first;
          ensure(i_size);
          MEMOP::copy(_p_data, first, i_size);
          _i_count = i_size;
        }
      }

      /**
       * @brief Makes the datas points to the contents of other.
       *
       * WARNING: Do not use this function unless you know what you are doing
       *
       */
      constexpr void shallowCopyFrom(const DynamicArrayBase& other) noexcept {
        _i_count = other._i_count;
        _i_capacity = other._i_capacity;
        _p_data = other._p_data;
      }

      /**
       * @brief Removes the element at position index from the array.
       *
       * @param index Position to the element to remove.
       */
      constexpr void remove(int index) noexcept {
        assert(index < _i_count && index >= 0);
        --_i_count;
        if (index != _i_count) {
          _p_data[index] = std::move(_p_data[_i_count]);
          MEMOP::destroyAt(_p_data, _i_count);
        } else
          MEMOP::destroyAt(_p_data, index);
      }

      /**
       * @brief Removes the last element from the array.
       *
       */
      constexpr void removeLast() noexcept {
        assert(_i_count > 0);
        --_i_count;
        MEMOP::destroyAt(_p_data, _i_count);
      }

      /**
       * @brief Fill the array with default-created elements
       *
       */
      constexpr void fill() noexcept { return fillEmplace(); }

      /**
       * @brief Fill the array with copies of the given value
       *
       * @param v The value to initialize the elements with.
       *
       */
      constexpr void fill(const value_type& v) noexcept { return fillEmplace(v); }

      /**
       * @brief Fill the array with a constructible element
       *
       */
      template < typename... Args >
      constexpr void fillEmplace(Args&&... args) noexcept {
        MEMOP::destroyRange(_p_data, 0, _i_count);
        MEMOP::constructRange(_p_data, 0, _i_capacity, std::forward< Args >(args)...);
        _i_count = _i_capacity;
      }

      /**
       * @brief Removes all elements from the array.
       *
       */
      constexpr void removeAll() noexcept {
        MEMOP::destroyRange(_p_data, 0, _i_count);
        _i_count = 0;
      }

      /**
       * @brief Ensures that the array has sufficient allocated memory to hold at least i_new_size elements.
       *        If the current capacity is less than i_new_size, the function reallocates memory to accommodate
       *        the new size. The newly allocated memory (if any) remains uninitialized.
       *
       * @param i_new_size The desired minimum capacity for the array.
       */
      constexpr void ensure(int i_new_size) noexcept {
        if (i_new_size <= _i_capacity || i_new_size <= 0) return;
        assert(i_new_size > 0 && "Cannot allocate non-positive size");
        _p_data = MEMOP::grow(_p_data, _i_count, i_new_size, isAllocated());
        _i_capacity = i_new_size;
      }

      /**
       * @brief Ensures that the array has enough capacity to store at least i_new_count elements,
       *        and fills any newly allocated memory with default-initialized elements until the
       *        total count reaches i_new_count.
       *
       * @param i_new_count The total number of elements the array should hold after the operation.
       */
      constexpr void ensureAndFill(int i_new_count) noexcept { ensureAndFillEmplace(i_new_count); }

      /**
       * @brief Ensures that the array has enough capacity to store at least i_new_count elements,
       *        and fills any newly allocated memory with copies of the given value v until the total count reaches
       * i_new_count.
       *
       * @param i_new_count The total number of elements the array should hold after the operation.
       * @param v The value to initialize any newly allocated elements with.
       */
      constexpr void ensureAndFill(int i_new_count, const value_type& v) noexcept {
        ensureAndFillEmplace(i_new_count, v);
      }

      /**
       * @brief Ensures that the array has enough capacity to store at least i_new_count elements,
       *        and fills any newly allocated memory with in-place constructed elements until the
       *        total count reaches i_new_count. The arguments args... are forwarded to the constructor
       *        as std::forward<Args>(args)...
       *
       * @param i_new_count The number of elements to add
       * @param args The arguments used to construct each added element
       */
      template < typename... Args >
      constexpr void ensureAndFillEmplace(int i_new_count, Args&&... args) noexcept {
        if (_i_count >= i_new_count) return;
        ensure(i_new_count);
        MEMOP::constructRange(_p_data, _i_count, i_new_count, std::forward< Args >(args)...);
        _i_count = i_new_count;
      }

      /**
       * @brief Resizes the array to contain exactly i_new_count elements.
       *
       * @param i_new_count New size of the container
       */
      constexpr void resize(int i_new_count) noexcept { resizeEmplace(i_new_count); }

      /**
       * @brief Resizes the array to contain exactly i_new_count elements initialized with the given
       * value.
       *
       * @param i_new_count New size of the container
       */
      constexpr void resize(int i_new_count, const value_type& v) noexcept { resizeEmplace(i_new_count, v); }

      /**
       * @brief Resizes the array to contain i_count elements.
       *
       * @param i_new_count New size of the container
       * @param args Arguments to pass to the constructor.
       */
      template < typename... Args >
      constexpr void resizeEmplace(int i_new_count, Args&&... args) noexcept {
        ensure(i_new_count);
        if (_i_count < i_new_count)
          MEMOP::constructRange(_p_data, _i_count, i_new_count, std::forward< Args >(args)...);
        else if (_i_count > i_new_count)
          MEMOP::destroyRange(_p_data, i_new_count, _i_count);
        _i_count = i_new_count;
      }

      /**
       * @brief Returns the position to the first element equals to value.
       *
       * @param value Value to compare the elements to.
       * @return Position of the first element equals to value or Count if no such element is found.
       */
      constexpr int find(const value_type& value) const noexcept {
        int i = 0;
        for (; i < _i_count; ++i)
          if (_p_data[i] == value) break;
        return i;
      }

      /**
       * @brief Returns the position to an element equals to value.
       *
       * @param value Value to compare the elements to.
       * @param compare comparison function
       *
       * @return Position of an element equals to value or Count if no such element is found.
       */
      template < typename COMPARE >
      constexpr int findBinary(const value_type& value, const COMPARE& compare) const noexcept {
        return findBinary(0, _i_count, value, compare);
      }

      /**
       * @brief Returns the position to an element equals to value in the range [i_start, i_end).
       *
       * @param i_start the start of the range of elements to examine
       * @param i_end the end of the range of elements to examine
       * @param value value to compare the elements to
       * @param compare comparison function
       *
       * @return Position of an element equals to value or Count if no such element is found.
       */
      template < typename COMPARE >
      constexpr int findBinary(int i_start, int i_end, const value_type& value, const COMPARE& compare) const noexcept {
        // Start with a binary search
        constexpr int i_cache_line_size = 64;   // in bytes
        const int     i_type_size = sizeof(value_type);
        const int     i_thr = static_cast< int >(i_cache_line_size / i_type_size);

        while (i_end - i_start > i_thr) {
          const int c = (i_end - i_start) >> 1;
          const int i_cmp = compare(at(c), value);
          if (!i_cmp) return c;
          if (i_cmp < 0)
            --i_end;
          else
            ++i_start;
        }

        // Fallback to linear search if the interval gets small enough
        for (int c = i_start; c < i_end; ++c)
          if (!compare(at(c), value)) return c;

        return size();
      }

      /**
       * @brief Returns a reference to the element at specified location index.
       *
       * @param index Position of the element to return
       * @return Reference to the requested element.
       */
      constexpr reference operator[](int index) noexcept {
        assert(index < _i_count && index >= 0);
        return _p_data[index];
      }
      constexpr const_reference operator[](int index) const noexcept {
        assert(index < _i_count && index >= 0);
        return _p_data[index];
      }

      /**
       * @brief Returns a reference to the element at specified location index.
       *        Same as the operator[]
       *
       * @param index Position of the element to return
       * @return Reference to the requested element
       */
      constexpr reference       at(int index) noexcept { return operator[](index); }
      constexpr const_reference at(int index) const noexcept { return operator[](index); }

      /**
       * @brief Set the given value to the element at specified location index.
       *
       */
      constexpr void set(int index, const value_type& v) noexcept { operator[](index) = v; }
      constexpr void set(int index, value_type&& v) noexcept { operator[](index) = std::move(v); }

      /**
       * @brief Shifts all elements from position i_pos onwards by i_shift to the right.
       *
       * This operation makes space for inserting new elements at i_pos by moving
       * elements [i_pos, size()-i_shift) to [i_pos+i_shift, size()).
       * The array size is NOT changed.
       *
       * Example:
       * @code
       * arr = [0, 1, 2, 3, 4];
       * arr.shiftRight(2, 1);
       * // arr = [0, 1, ?, 2, 3];
       *
       * arr = [0, 1, 2, 3, 4];
       * arr.shiftRight(1, 2);
       * // arr = [0, ?, ?, 1, 2];
       * @endcode
       *
       * @param i_pos Starting position for the shift (0 <= i_pos < size())
       * @param i_shift Number of positions to shift right (default: 1, must be > 0)
       *
       * @note For TrivialDynamicArray, uses std::memmove
       * @warning Last i_shift elements are lost unless array is extended first
       * @see shiftRightAndExtend() to preserve all elements
       */
      constexpr void shiftRight(int i_pos, int i_shift = 1) noexcept {
        MEMOP::shiftRight(_p_data, i_pos, _i_count, i_shift);
      }

      /**
       * @brief Extends array by i_shift elements and shifts elements right at position i_pos.
       *
       * Equivalent to adding i_shift elements followed by shiftRight(i_pos, i_shift),
       * but potentially more efficient. Creates space at i_pos for inserting new elements
       * without losing data.
       *
       * Example:
       * @code
       * arr = [0, 1, 2, 3, 4];
       * arr.shiftRightAndExtend(2, 2);
       * arr[2] = 99;
       * arr[3] = 88;
       * // arr = [0, 1, 99, 88, 2, 3, 4];
       * @endcode
       *
       * @param i_pos Starting position for the shift (0 <= i_pos <= size())
       * @param i_shift Number of positions to shift and extend (default: 1, must be > 0)
       *
       * @note Increases array size by i_shift
       * @warning Invalidates iterators and references to elements >= i_pos
       * @warning May trigger reallocation if capacity is exceeded
       */
      constexpr void shiftRightAndExtend(int i_pos, int i_shift = 1) noexcept {
        const int i_new_count = _i_count + i_shift;
        ensure(i_new_count);
        MEMOP::constructRange(_p_data, _i_count, i_new_count);
        _i_count = i_new_count;
        shiftRight(i_pos, i_shift);
      }

      /**
       * @brief Shifts all elements from position i_pos onwards by i_shift to the left.
       *
       * Example:
       * @code
       * arr = [0, 1, 2, 3, 4];
       * arr.shiftLeft(1, 1);
       * // arr = [1, 2, 3, 4, ?];
       *
       * arr = [0, 1, 2, 3, 4];
       * arr.shiftLeft(1, 2);
       * // arr = [2, 3, 4, ?, ?];
       * @endcode
       *
       * @param i_pos Starting position for the shift (0 <= i_pos < size())
       * @param i_shift Number of positions to shift left (default: 1, must be > 0)
       *
       * @note For TrivialDynamicArray, uses std::memmove
       * @see shiftLeftAndShrink() to remove the duplicated elements
       */
      constexpr void shiftLeft(int i_pos, int i_shift = 1) noexcept {
        MEMOP::shiftLeft(_p_data, i_pos, _i_count, i_shift);
      }

      /**
       * @brief Shifts elements left at position i_pos by i_shift and removes the last i_shift elements.
       *
       * Example:
       * @code
       * arr = [0, 1, 2, 3, 4];
       * arr.shiftLeftAndShrink(1, 1);
       * // arr = [1, 2, 3, 4];
       *
       * arr = [0, 1, 2, 3, 4];
       * arr.shiftLeftAndShrink(1, 2);
       * // arr = [2, 3, 4];
       * @endcode
       *
       * @param i_pos Starting position for the shift (0 <= i_pos < size())
       * @param i_shift Number of positions to shift and shrink (default: 1, must be > 0)
       *
       * @note Reduces array size by i_shift
       * @warning Invalidates iterators and references to elements >= i_pos
       */
      constexpr void shiftLeftAndShrink(int i_pos, int i_shift = 1) noexcept {
        MEMOP::shiftLeft(_p_data, i_pos, _i_count, i_shift);
        MEMOP::destroyRange(_p_data, _i_count - i_shift, _i_count);
        _i_count -= i_shift;
      }

      /**
       * @brief Check whether the array is allocated on the heap
       *
       * @return true if the array is allocated on the heap, false if it uses static storage
       */
      constexpr bool isAllocated() const noexcept { return _p_data != Parent::_static_datas; }

      /**
       * @brief Remove and destroy all elements from the array and free any allocated memory.
       * Prefer to call removeAll() unless you specifically need to release the array memory.
       *
       */
      constexpr void reset() noexcept {
        if (isAllocated()) {
          removeAll();
          MEMOP::free(_p_data);
          _p_data = Parent::_static_datas;
        }
        _i_count = 0;
        _i_capacity = SSO;
      }

      /**
       * @brief Steal the internal raw pointer and pass it to p_data along with count and capacity values.
       * This function applies only if the array is allocated on the heap. If not, it does nothing and
       * return false.
       *
       * WARNING: this function should be used with extreme care as the allocated memory will not
       * be deleted once the array is destroyed. It is the responsability of the caller to free the memory.
       *
       * @param p_data Pointer that will receive the memory address
       * @param i_count Store the number of elements in the array
       * @param i_capacity Store the maximum number of elements the array can accomodate before growing
       *
       * @return true is the
       *
       */
      constexpr bool release(pointer& p_data, int& i_count, int& i_capacity) noexcept {
        if (!isAllocated()) return false;
        p_data = _p_data;
        i_count = _i_count;
        i_capacity = _i_capacity;
        _p_data = Parent::_static_datas;
        _i_count = 0;
        _i_capacity = SSO;
        return true;
      }

      //---------------------------------------------------------------------------
      // STL methods
      //---------------------------------------------------------------------------

      // Iterators
      constexpr iterator       begin() noexcept { return _p_data; }
      constexpr const_iterator begin() const noexcept { return _p_data; }
      constexpr const_iterator cbegin() const noexcept { return _p_data; }

      constexpr iterator       end() noexcept { return _p_data + _i_count; }
      constexpr const_iterator end() const noexcept { return _p_data + _i_count; }
      constexpr const_iterator cend() const noexcept { return _p_data + _i_count; }

      constexpr reverse_iterator       rbegin() noexcept { return reverse_iterator(end()); }
      constexpr const_reverse_iterator rbegin() const noexcept { return const_reverse_iterator(end()); }
      constexpr const_reverse_iterator crbegin() const noexcept { return const_reverse_iterator(cend()); }

      constexpr reverse_iterator       rend() noexcept { return reverse_iterator(begin()); }
      constexpr const_reverse_iterator rend() const noexcept { return const_reverse_iterator(begin()); }
      constexpr const_reverse_iterator crend() const noexcept { return const_reverse_iterator(cbegin()); }

      // Capacity
      constexpr size_type size() const noexcept { return _i_count; }
      constexpr size_type capacity() const noexcept { return _i_capacity; }
      constexpr bool      empty() const noexcept { return _i_count == 0; }
      void                reserve(size_type n) { ensure(n); }

      // Modifiers
      constexpr void push_back(const value_type& val) noexcept { add(val); }
      constexpr void push_back(value_type&& val) noexcept { add(std::move(val)); }
      constexpr void pop_back() noexcept { removeLast(); }
      template < class... Args >
      constexpr void emplace_back(Args&&... args) noexcept {
        addEmplace(std::forward< Args >(args)...);
      }

      /**
       * @brief Inserts element at the specified location in the container.
       *
       * @param pos Iterator before which the content will be inserted (pos may be the end() iterator)
       * @param value Element value to insert
       * @return Iterator pointing to the inserted value
       */
      constexpr iterator insert(const_iterator pos, const value_type& value) noexcept {
        const int index = pos - begin();
        assert(index >= 0 && index <= _i_count);
        shiftRightAndExtend(index);
        _p_data[index] = value;
        return begin() + index;
      }

      /**
       * @brief Inserts element at the specified location in the container (move version).
       *
       * @param pos Iterator before which the content will be inserted (pos may be the end() iterator)
       * @param value Element value to insert
       * @return Iterator pointing to the inserted value
       */
      constexpr iterator insert(const_iterator pos, value_type&& value) noexcept {
        const int index = pos - begin();
        assert(index >= 0 && index <= _i_count);
        shiftRightAndExtend(index);
        _p_data[index] = std::move(value);
        return begin() + index;
      }

      /**
       * @brief Inserts count copies of the value before pos.
       *
       * @param pos Iterator before which the content will be inserted
       * @param count Number of elements to insert
       * @param value Element value to insert
       * @return Iterator pointing to the first element inserted, or pos if count==0
       */
      constexpr iterator insert(const_iterator pos, size_type count, const value_type& value) noexcept {
        if (count == 0) return begin() + (pos - begin());
        const int index = pos - begin();
        assert(index >= 0 && index <= _i_count);

        shiftRightAndExtend(index, count);

        // Insert new elements
        for (int i = 0; i < count; ++i) {
          MEMOP::constructAt(_p_data, index + i, value);
        }

        return begin() + index;
      }

      /**
       * @brief Inserts elements from range [first, last) before pos.
       *
       * @param pos Iterator before which the content will be inserted
       * @param first The beginning of the range of elements to insert
       * @param last The end of the range of elements to insert
       * @return Iterator pointing to the first element inserted, or pos if first==last
       */
      template < class InputIt >
      constexpr iterator insert(const_iterator pos, InputIt first, InputIt last) noexcept {
        if (first == last) return begin() + (pos - begin());
        const int index = pos - begin();
        assert(index >= 0 && index <= _i_count);

        const int count = last - first;

        shiftRightAndExtend(index, count);

        // Insert new elements
        int i = 0;
        for (InputIt it = first; it != last; ++it, ++i) {
          MEMOP::constructAt(_p_data, index + i, *it);
        }

        return begin() + index;
      }

      /**
       * @brief Inserts elements from initializer list before pos.
       *
       * @param pos Iterator before which the content will be inserted
       * @param ilist Initializer list to insert the values from
       * @return Iterator pointing to the first element inserted, or pos if ilist is empty
       */
      constexpr iterator insert(const_iterator pos, std::initializer_list< value_type > ilist) noexcept {
        return insert(pos, ilist.begin(), ilist.end());
      }

      /**
       * @brief Constructs element in-place at the specified location.
       *
       * @param pos Iterator before which the new element will be constructed
       * @param args Arguments to forward to the constructor of the element
       * @return Iterator pointing to the emplaced element
       */
      template < class... Args >
      constexpr iterator emplace(const_iterator pos, Args&&... args) noexcept {
        const int index = pos - begin();
        assert(index >= 0 && index <= _i_count);
        shiftRightAndExtend(index);
        MEMOP::destroyAt(_p_data, index);
        MEMOP::constructAt(_p_data, index, std::forward< Args >(args)...);
        return begin() + index;
      }

      // Element access
      constexpr pointer       data() noexcept { return _p_data; }
      constexpr const_pointer data() const noexcept { return _p_data; }

      constexpr reference       back() noexcept { return operator[](_i_count - 1); }
      constexpr const_reference back() const noexcept { return operator[](_i_count - 1); }

      constexpr reference       front() noexcept { return operator[](0); }
      constexpr const_reference front() const noexcept { return operator[](0); }

      constexpr void clear() noexcept { removeAll(); }

      //---------------------------------------------------------------------------
      // Private methods
      //---------------------------------------------------------------------------

      constexpr void _growIfNecessary() noexcept {
        assert(size() <= capacity());
        if (size() < capacity()) return;
        const int i_new_size = (capacity() + 1) * (1 + (GF_NUM / static_cast< double >(GF_DEN)));
        assert(i_new_size > size());   // If false, probably overflow the array cannot grow anymore
        ensure(i_new_size);
      }
    };

    /**
     * @class LazyOperations
     * @headerfile arrays.h
     * @brief Low-level memory operations for LazyDynamicArrays
     *
     * @tparam TYPE The type of the stored values
     *
     */
    template < class TYPE >
    struct LazyOperations {
      template < typename... Args >
      constexpr static void constructAt(TYPE* p_datas, int i, Args&&... args) noexcept {
        assert(p_datas);
        new (p_datas + i) TYPE(std::forward< Args >(args)...);
      }

      constexpr static void destroyAt(TYPE* p_datas, int i) noexcept {
        assert(p_datas);
        p_datas[i].~TYPE();
      }

      template < typename... Args >
      constexpr static void constructRange(TYPE* p_datas, int i_start, int i_end, Args&&... args) noexcept {
        for (int i = i_start; i < i_end; ++i)
          constructAt(p_datas, i, std::forward< Args >(args)...);
      }

      constexpr static void destroyRange(TYPE* p_datas, int i_start, int i_end) noexcept {
        for (int i = i_end - 1; i >= i_start; --i)
          destroyAt(p_datas, i);
      }

      constexpr static void copy(TYPE* __restrict p_dst, const TYPE* __restrict p_src, int n) noexcept {
        assert(p_dst && p_src && n >= 0);
        for (int i = 0; i < n; ++i)
          p_dst[i].copyFrom(p_src[i]);
      }

      constexpr static void free(TYPE* p_datas) noexcept {
#if TRACY_ENABLE
        TracyFreeN(p_datas, "lazy");
#endif
        NT_FREE(p_datas);
      }

      constexpr static TYPE* grow(TYPE* p_datas, int i_count, int i_new_size, bool b_allocated) noexcept {
        assert(i_new_size > 0 && "Invalid allocation size");

        //-- Increase the size
        TYPE* p_raw_mem = static_cast< TYPE* >(NT_MALLOC(i_new_size * sizeof(TYPE)));
        assert(p_raw_mem);
#if TRACY_ENABLE
        TracyAllocN(p_raw_mem, i_new_size * sizeof(TYPE), "lazy");
#endif
        //-- Move elements if any
        if (p_datas) {
          for (int i = 0; i < i_count; ++i)
            new (p_raw_mem + i) TYPE(std::move(p_datas[i]));
          if (b_allocated) {
            destroyRange(p_datas, 0, i_count);
            free(p_datas);
          }
        }
        return p_raw_mem;
      }

      constexpr static void shiftLeft(TYPE* p_datas, int i_pos, int i_size, int i_shift = 1) noexcept {
        assert(p_datas && i_pos >= 0 && i_size > 0 && i_pos - i_shift >= 0);
        for (int k = i_pos; k < i_size; ++k)
          p_datas[k - i_shift] = std::move(p_datas[k]);
      }

      constexpr static void shiftRight(TYPE* p_datas, int i_pos, int i_size, int i_shift = 1) noexcept {
        assert(p_datas && i_pos >= 0 && i_size > 0 && i_pos + i_shift <= i_size);
        for (int k = i_size - 1; k >= i_pos + i_shift; --k)
          p_datas[k] = std::move(p_datas[k - i_shift]);
      }
    };

    /**
     * @class TrivialOperations
     * @headerfile arrays.h
     * @brief Low-level memory operations for TrivialDynamicArrays
     *
     * @tparam TYPE The type of the stored values
     *
     */
    template < class TYPE >
    struct TrivialOperations {
      template < typename... Args >
      constexpr static void constructAt(TYPE* p_datas, int i, Args&&... args) noexcept {
        assert(p_datas);
        if constexpr (sizeof...(args) > 0) new (p_datas + i) TYPE(std::forward< Args >(args)...);
      }

      constexpr static void destroyAt(TYPE*, int) noexcept {}

      constexpr static void constructRange(TYPE* p_datas, int i_start, int i_end) noexcept {
        // For types with default member initializers, we must construct them
        if constexpr (!std::is_trivially_default_constructible< TYPE >::value) {
          for (int i = i_start; i < i_end; ++i)
            new (p_datas + i) TYPE();
        }
      }

      template < typename Arg >
      constexpr static void constructRange(TYPE* p_datas, int i_start, int i_end, Arg&& arg) noexcept {
        if constexpr (std::is_arithmetic< TYPE >::value || std::is_pointer< TYPE >::value) {
          if (arg == static_cast< TYPE >(0)) {
            std::memset(static_cast< void* >(p_datas + i_start), 0, sizeof(TYPE) * (i_end - i_start));
            return;
          }
        }
        for (int i = i_start; i < i_end; ++i)
          constructAt(p_datas, i, std::forward< Arg >(arg));
      }

      template < typename... Args >
      constexpr static void constructRange(TYPE* p_datas, int i_start, int i_end, Args&&... args) noexcept {
        for (int i = i_start; i < i_end; ++i)
          constructAt(p_datas, i, std::forward< Args >(args)...);
      }

      constexpr static void destroyRange(TYPE*, int, int) noexcept {}

      constexpr static void copy(TYPE* __restrict p_dst, const TYPE* __restrict p_src, int n) noexcept {
        assert(p_dst && p_src && n >= 0);
        std::memcpy(p_dst, p_src, n * sizeof(TYPE));
      }

      constexpr static void free(TYPE* p_datas) noexcept {
#if TRACY_ENABLE
        TracyFreeN(p_datas, "trivial");
#endif
        NT_FREE(p_datas);
      }

      constexpr static TYPE* grow(TYPE* p_datas, int i_count, int i_new_size, bool b_allocated) noexcept {
        assert(i_new_size > 0 && "Invalid allocation size");

        if (b_allocated) {
          const size_t byte_size = static_cast< size_t >(i_new_size) * sizeof(TYPE);
          TYPE*        p_raw_mem = static_cast< TYPE* >(NT_REALLOC(p_datas, byte_size));
          assert(p_raw_mem);
#if TRACY_ENABLE
          if (p_raw_mem != p_datas) {
            TracyFreeN(p_datas, "trivial");
            TracyAllocN(p_raw_mem, i_new_size * sizeof(TYPE), "trivial");
          }
#endif
          return p_raw_mem;
        }
        const size_t byte_size = static_cast< size_t >(i_new_size) * sizeof(TYPE);
        TYPE*        p_raw_mem = static_cast< TYPE* >(NT_MALLOC(byte_size));
#if TRACY_ENABLE
        TracyAllocN(p_raw_mem, byte_size, "trivial");
#endif
        assert(p_raw_mem);
        if (p_datas) std::memcpy(p_raw_mem, p_datas, i_count * sizeof(TYPE));
        return p_raw_mem;
      }

      constexpr static void shiftLeft(TYPE* p_datas, int i_pos, int i_size, int i_shift = 1) noexcept {
        assert(p_datas && i_pos >= 0 && i_size >= 0);
        assert(i_shift > 0 && i_pos - i_shift >= 0);
        const int i_dst = i_pos - i_shift;
        NT_MEMMOVE(p_datas + i_dst, p_datas + i_pos, sizeof(TYPE) * (i_size - i_pos));
      }

      constexpr static void shiftRight(TYPE* p_datas, int i_pos, int i_size, int i_shift = 1) noexcept {
        assert(p_datas && i_pos >= 0 && i_size >= 0);
        assert(i_shift > 0 && i_pos + i_shift <= i_size);
        const int i_dst = i_pos + i_shift;
        NT_MEMMOVE(p_datas + i_dst, p_datas + i_pos, sizeof(TYPE) * (i_size - i_dst));
      }
    };

  }   // namespace details


  /**
   * @class LazyDynamicArray
   * @headerfile arrays.h
   * @brief Array that constructs/destructs objects only when necessary.
   *
   * @tparam TYPE The type of the stored values
   * @tparam SSO The size of the pre-allocated memory on the stack (default is 0, no pre-allocation)
   * @tparam GF_NUM Numerator value of the grow factor
   * @tparam GF_DEN Denominator value of the grow factor
   *
   */
  template < class TYPE, const int SSO = 0, const int GF_NUM = 1, const int GF_DEN = 2 >
  struct LazyDynamicArray: details::DynamicArrayBase< TYPE, details::LazyOperations< TYPE >, GF_NUM, GF_DEN, SSO > {
    using Parent = details::DynamicArrayBase< TYPE, details::LazyOperations< TYPE >, GF_NUM, GF_DEN, SSO >;
    using Parent::Parent;
  };

  /**
   * @class TrivialDynamicArray
   * @headerfile arrays.h
   * @brief Array optimized for storing trivially copyable types that do not need to be constructed.
   *        (see https://en.cppreference.com/w/cpp/named_req/TriviallyCopyable).
   *
   * Example of usage:
   *
   * @code
   * int main() {
   *  nt::TrivialDynamicArray< int > arr;
   *
   *  arr.add(1); // or arr.push_back(1);
   *  arr.add(2); // or arr.push_back(2);
   *
   *  for(int v : arr)
   *    std::cout << v << std::endl;
   *
   *  arr = {1, 3, 4}; // Initializer list can also be used.
   * }
   * @endcode
   *
   * @tparam TYPE The type of elements stored in the array
   * @tparam SSO The size of the pre-allocated memory on the stack (default is 0, no pre-allocation)
   * @tparam GF_NUM Numerator value of the grow factor
   * @tparam GF_DEN Denominator value of the grow factor
   *
   */
  template < class TYPE, const int SSO = 0, const int GF_NUM = 1, const int GF_DEN = 2 >
  struct TrivialDynamicArray:
      details::DynamicArrayBase< TYPE, details::TrivialOperations< TYPE >, GF_NUM, GF_DEN, SSO > {
    static_assert(std::is_trivially_copyable< TYPE >::value, "TrivialDynamicArray : Non-trivially copyable type");
    using Parent = details::DynamicArrayBase< TYPE, details::TrivialOperations< TYPE >, GF_NUM, GF_DEN, SSO >;
    using Parent::Parent;
    using value_type = typename Parent::value_type;

    /**
     * @brief Constructs the container with the contents of the provided initializer list
     *
     */
    constexpr TrivialDynamicArray(const std::initializer_list< value_type > l) noexcept {
      Parent::ensure(l.size());
      std::memcpy(Parent::data(), l.begin(), l.size() * sizeof(value_type));
      Parent::_i_count = l.size();
    }

    /**
     * @brief Fills the raw memory pointed by _p_data with the byte pattern (static_cast<unsigned char>(c)).
     *
     * This function sets each byte in the memory to the byte value `c`. It does not assign
     * `c` to each element of the array, but rather fills the raw memory byte-by-byte.
     *
     * @param c fill byte (interpreted as a byte pattern)
     */
    constexpr void fillBytes(int c) noexcept {
      std::memset(static_cast< void* >(Parent::data()), c, sizeof(value_type) * Parent::size());
    }

    /**
     * @brief Fills a range of the raw memory pointed by _p_data with the byte pattern (static_cast<unsigned char>(c)).
     *
     * This function sets each byte in the specified range [begin, end) to the byte value `c`.
     * It does not assign `c` to each element of the array, but rather fills the raw memory byte-by-byte.
     *
     * @param c fill byte (interpreted as a byte pattern)
     * @param begin Starting index of the range (inclusive, 0 <= begin < size())
     * @param end Ending index of the range (exclusive, begin <= end <= size())
     */
    constexpr void fillBytes(int c, int begin, int end) noexcept {
      assert(begin >= 0 && begin < Parent::size());
      assert(end >= begin && end <= Parent::size());
      if (begin >= end) return;
      std::memset(static_cast< void* >(Parent::data() + begin), c, sizeof(value_type) * (end - begin));
    }

    /**
     * @brief Fills a range of the raw memory pointed by _p_data with the byte pattern (static_cast<unsigned char>(c)).
     *
     * This function sets each byte in memory to the byte value `c` starting from the specified position .
     * It does not assign `c` to each element of the array, but rather fills the raw memory byte-by-byte.
     *
     * @param c fill byte (interpreted as a byte pattern)
     * @param pos Position index (inclusive, 0 <= pos < size())
     */
    constexpr void fillBytes(int c, int pos) noexcept { fillBytes(c, pos, Parent::size()); }

    /**
     * @brief Sets all bits in the underlying memory to 0 (bitwise).
     *
     * This function clears every bit in the raw memory pointed by Datas,
     * setting all bits to zero. Note that this affects the raw memory,
     * not necessarily the logical values of individual elements if they
     * are larger than a byte.
     */
    constexpr void setBitsToZero() noexcept { fillBytes(0); }

    /**
     * @brief Sets all bits in the underlying memory to 0 (bitwise) starting from the given position.
     *
     */
    constexpr void setBitsToZero(int pos) noexcept { fillBytes(0, pos); }

    /**
     * @brief Sets all bits in the underlying memory to 1 (bitwise).
     *
     * This function sets every bit in the raw memory pointed by Datas
     * to one. It fills the memory with all bits set.
     *
     */
    constexpr void setBitsToOne() noexcept { fillBytes(~static_cast< int >(0)); }

    /**
     * @brief Sets all bits in the underlying memory to 1 (bitwise) starting from the given position.
     *
     */
    constexpr void setBitsToOne(int pos) noexcept { fillBytes(~static_cast< int >(0), pos); }

    /**
     * @brief Ensures the array has enough capacity to contain at least `i_new_size` elements.
     * If the array is resized, fills the new memory with the byte pattern `c`.
     *
     * Note: This fills the raw memory with the byte `c`, which may not correspond
     * to the element value `c` if `value_type` is not `unsigned char`.
     *
     * @param i_new_size New size to ensure
     * @param c Fill byte (interpreted as a byte pattern)
     */
    constexpr void ensureAndFillBytes(int i_new_size, int c) noexcept {
      if (Parent::size() >= i_new_size) return;
      Parent::ensure(i_new_size);
      std::memset(
         static_cast< void* >(Parent::data() + Parent::size()), c, sizeof(value_type) * (i_new_size - Parent::size()));
      Parent::_i_count = i_new_size;
    }
  };

  /**
   * @class BitArray
   * @brief Class representing an array of bits.
   *
   * Example of usage :
   *
   * @code
   *   int main() {
   *     nt::BitArray bitarray;
   *
   *     // Initialize the array with 1024 bits
   *     bitarray.extendByBits(1024);
   *
   *     std::cout << "All bits are set to 0 ? " << (bitarray.none() ? "yes" : "no") << std::endl;
   *
   *     bitarray.setOneAt(56);
   *
   *     std::cout << "At least one bit set to 1 ? "
   *               << (bitarray.any() ? "yes" : "no") << std::endl;
   *
   *     std::cout << "Is bit 56 set to 1 ? "
   *               << (bitarray.isOne(56) ? "yes" : "no") << std::endl;
   *
   *     return 0;
   *   }
   * @endcode
   *
   */
  struct BitArray {
    enum { WORD_SIZE = NT_SIZEOF_BITS_(std::uint64_t), INVALID = NT_UINT64_MAX_ };
    TrivialDynamicArray< std::uint64_t > _array;

    constexpr BitArray() = default;
    explicit BitArray(int i_num_bits, int v = 0) { extendByBits(i_num_bits, v); }
    BitArray(const BitArray&) = delete;
    BitArray& operator=(const BitArray&) = delete;
    BitArray(BitArray&& other) noexcept : _array(std::move(other._array)) {}
    BitArray& operator=(BitArray&& other) noexcept {
      if (this == &other) return *this;
      _array = std::move(other._array);
      return *this;
    }

    /**
     * @brief Iterator over the indices of a bitarray for which the corresponding bit is equal to
     * one.
     *
     * Example of usage :
     *
     * BitArray bits;
     * ...
     * for (BitArray::OneIter it(bits); *it != BitArray::INVALID; ++it) {
     *  int index = *it;
     *  ...
     * }
     *
     */
    struct OneIter {
      const BitArray& _bitArray;

      uint64_t _index;

#if defined(NT_USE_SIMD)
      uint64_t __WordIter;
      int      __TabIter;
      constexpr explicit OneIter(const BitArray& BitArray) noexcept :
          _bitArray(BitArray), _index(INVALID), __WordIter(0), __TabIter(0) {
        _findNextOne();
      }
#else
      constexpr explicit OneIter(const BitArray& BitArray) noexcept : _bitArray(BitArray), _index(INVALID) {
        _findNextOne();
      }
#endif

      NT_SIMD_CONSTEXPR_ void reset() noexcept {
#if defined(NT_USE_SIMD)
        __WordIter = 0;
        __TabIter = 0;
#endif
        _index = INVALID;
        _findNextOne();
      }

      NT_SIMD_CONSTEXPR_ OneIter& operator++() noexcept {
        _findNextOne();
        return *this;
      }

      constexpr uint64_t operator*() const noexcept { return _index; }

      NT_SIMD_CONSTEXPR_ void _findNextOne() noexcept {
#if defined(NT_USE_SIMD)
        _index = INVALID;
        while (!__WordIter && __TabIter < _bitArray.countWords())
          __WordIter = _bitArray._array[__TabIter++];
        const uint64_t iNumTrailingZeros =
           _tzcnt_u64(__WordIter);   // _tzcnt_u64 : Count the number of trailing zero bits in __WordIter.
        if (iNumTrailingZeros < 64) {
          __WordIter = _blsr_u64(__WordIter);
          _index = iNumTrailingZeros + WORD_SIZE * (__TabIter - 1);
        }
#else
        ++_index;
        const uint64_t i_word_pos = NT_MODULO_64_(_index);
        //--
        const uint64_t i_mask = ~((static_cast< uint64_t >(1) << i_word_pos) - static_cast< uint64_t >(1));
        uint64_t       i_tab_index = NT_DIVIDE_BY_64_(_index);
        if (i_tab_index >= static_cast< uint64_t >(_bitArray.countWords())) {
          _index = INVALID;
          return;
        }
        uint64_t i_word = _bitArray._array[i_tab_index];
        i_word &= i_mask;
        while (i_tab_index < static_cast< uint64_t >(_bitArray.countWords() - 1) && !i_word)
          i_word = _bitArray._array[++i_tab_index];
        if (!i_word) {
          _index = INVALID;
          return;
        }
        const uint64_t i_word_minus_one = i_word - static_cast< uint64_t >(1);
        _index = BitArray::_countOnesFast(i_word ^ i_word_minus_one) + WORD_SIZE * i_tab_index - 1;
#endif
      }
    };

    /**
     * @brief Bitwise AND operator : [0 1 1 0] & [1 1 0 0] = [0 1 0 0]
     *
     * @param b
     * @return
     */
    constexpr BitArray& operator&=(const BitArray& b) noexcept {
      assert(countWords() == b.countWords());
      for (int i_field = 0; i_field < countWords(); ++i_field)
        _array[i_field] = _array[i_field] & b._array[i_field];
      return *this;
    }

    BitArray operator&(const BitArray& b) const noexcept {
      assert(countWords() == b.countWords());
      BitArray result;
      result.extendByWords(countWords());
      for (int i_field = 0; i_field < countWords(); ++i_field)
        result._array[i_field] = _array[i_field] & b._array[i_field];
      return result;
    }

    /**
     * @brief Bitwise OR operator : [0 1 1 0] | [1 1 0 0] = [1 1 1 0]
     *
     * @param b
     * @return
     */
    constexpr BitArray& operator|=(const BitArray& b) noexcept {
      assert(countWords() == b.countWords());
      for (int i_field = 0; i_field < countWords(); ++i_field)
        _array[i_field] = _array[i_field] | b._array[i_field];
      //--
      return *this;
    }
    BitArray operator|(const BitArray& b) const noexcept {
      assert(countWords() == b.countWords());
      BitArray result;
      result.extendByWords(countWords());
      for (int i_field = 0; i_field < countWords(); ++i_field)
        result._array[i_field] = _array[i_field] | b._array[i_field];
      return result;
    }

    /**
     * @brief Bitwise XOR operator : [0 1 1 0] ^ [1 1 0 0] = [1 0 1 0]
     *
     * @param b
     * @return
     */
    constexpr BitArray& operator^=(const BitArray& b) noexcept {
      assert(countWords() == b.countWords());
      for (int i_field = 0; i_field < countWords(); ++i_field)
        _array[i_field] = _array[i_field] ^ b._array[i_field];
      return *this;
    }
    BitArray operator^(const BitArray& b) const noexcept {
      assert(countWords() == b.countWords());
      BitArray result;
      result.extendByWords(countWords());
      for (int i_field = 0; i_field < countWords(); ++i_field)
        result._array[i_field] = _array[i_field] ^ b._array[i_field];
      return result;
    }

    /**
     * @brief Compute the Hamming distance between this bitarray and another.
     *
     * The Hamming distance is the number of bit positions at which the two bit arrays differ.
     * It is obtained by XOR-ing corresponding words and summing the population counts.
     *
     * @param b The bit array to compare against. Must have the same number of words/bits as this.
     * @return The number of differing bit positions.
     */
    int hamming(const BitArray& b) const noexcept {
      assert(countWords() == b.countWords());
      uint64_t i_count = 0;
      for (int i_field = 0; i_field < countWords(); ++i_field)
        i_count += _countOnesFast(_array[i_field] ^ b._array[i_field]);
      return i_count;
    }

    /**
     * @brief Bitwise equality testing
     *
     * @param b
     * @return
     */
    constexpr bool operator==(const BitArray& b) const noexcept {
      assert(countWords() == b.countWords());
      for (int i_field = 0; i_field < countWords(); ++i_field)
        if (_array[i_field] != b._array[i_field]) return false;
      return true;
    }

    /**
     * @brief Sets the i-th bit of the bitarray to one.
     *
     * @param i
     */
    constexpr void setOneAt(uint64_t i) noexcept {
      uint64_t& i_word = _getWordFromIndex(i);
      i_word |= (static_cast< uint64_t >(1) << NT_MODULO_64_(i));
    }

    /**
     * @brief Sets the i-th bit of the bitarray to zero.
     *
     * @param i
     */
    constexpr void setZeroAt(uint64_t i) noexcept {
      uint64_t& i_word = _array[NT_DIVIDE_BY_64_(i)];
      i_word &= ~(static_cast< uint64_t >(1) << NT_MODULO_64_(i));
    }

    /**
     * @brief Sets all bits to zero.
     *
     */
    constexpr void setBitsToZero() noexcept { _array.setBitsToZero(); }

    /**
     * @brief Sets all bits to zero starting from the given position.
     *
     */
    constexpr void setBitsToZero(uint64_t pos) noexcept {
      uint64_t& i_word = _array[NT_DIVIDE_BY_64_(pos)];
      i_word &= ~(static_cast< uint64_t >(~0) << NT_MODULO_64_(pos));
    }

    /**
     * @brief Sets all bits to one.
     *
     */
    constexpr void setBitsToOne() noexcept { _array.setBitsToOne(); }

    /**
     * @brief Sets all bits to one starting from the given position.
     *
     */
    constexpr void setBitsToOne(uint64_t pos) noexcept {
      uint64_t& i_word = _array[NT_DIVIDE_BY_64_(pos)];
      i_word |= (static_cast< uint64_t >(~0) << NT_MODULO_64_(pos));
    }

    /**
     * @brief Check if the i-th bit is equal to one.
     *
     * @param i
     * @return
     */
    constexpr bool isOne(uint64_t i) const noexcept {
      return _array[NT_DIVIDE_BY_64_(i)] & (static_cast< uint64_t >(1) << NT_MODULO_64_(i));
    }

    /**
     * @brief Flips all bits in the bitarray.
     *
     */
    constexpr void flip() noexcept {
      for (int i_field = 0; i_field < countWords(); ++i_field)
        _array[i_field] = ~_array[i_field];
    }

    /**
     * @brief  Returns bit value associated to the key
     *
     * @param i
     * @return
     */
    constexpr bool operator[](uint64_t i) const noexcept { return isOne(i); }

    /**
     * @brief Allocate enough memory to represent i_num_words words.
     *
     * @param i_num_words
     * @param v
     */
    constexpr void extendByWords(int i_num_words, int v = 0) noexcept { extendByBits(i_num_words * WORD_SIZE, v); }

    /**
     * @brief Allocate enough memory to represent i_num_bits bits.
     *
     * @param i_num_bits
     * @param v
     */
    constexpr void extendByBits(int i_num_bits, int v = 0) noexcept {
      if (i_num_bits < 1) return;
      _array.ensureAndFillBytes((i_num_bits + WORD_SIZE - 1) / WORD_SIZE, v);
    }

    /**
     * @brief Counts and returns the number of bits set to ones.
     *
     * @return uint64_t
     */
    constexpr uint64_t countOnes() const noexcept {
      uint64_t i_count = 0;
      for (int i_field = 0; i_field < countWords(); ++i_field)
        i_count += _countOnesFast(_array[i_field]);
      return i_count;
    }

    /**
     * @brief Counts and returns the number of words.
     *
     * @return constexpr uint64_t
     */
    constexpr uint64_t countWords() const noexcept {
      assert(_array._i_capacity == _array._i_count);
      return _array._i_capacity;
    }

    /**
     * @brief Replaces the contents with a copy of the contents of fro
     *
     */
    constexpr void copyFrom(const BitArray& from) noexcept { _array.copyFrom(from._array); }

    /**
     * @brief Removes all elements from the bitarray.
     *
     */
    constexpr void removeAll() noexcept { _array.removeAll(); }

    /**
     * @brief Return a string representation of the set
     *
     */
    // nt::String toString(uint64_t i_bound) const noexcept {
    //   nt::MemoryBuffer buffer;
    //   for (uint64_t i = 0; i < i_bound; ++i)
    //     nt::format_to(std::back_inserter(buffer), "{} ", isOne(i) ? "1" : "0");
    //   return nt::to_string(buffer);
    // }

    //---------------------------------------------------------------------------
    // STL methods
    //---------------------------------------------------------------------------

    // Element access
    constexpr bool test(std::size_t pos) const noexcept { return isOne(pos); }

    // Checks if all bits are set to 1
    constexpr bool all() const noexcept {
      for (int i_field = 0; i_field < countWords(); ++i_field)
        if (_array[i_field] != ~static_cast< uint64_t >(0)) return false;
      return true;
    }

    // Checks if any bits are set to 1.
    constexpr bool any() const noexcept {
      for (int i_field = 0; i_field < countWords(); ++i_field)
        if (_array[i_field] != 0) return true;
      return false;
    }

    // Checks if none of the bits are set to 1.
    constexpr bool none() const noexcept { return !any(); }

    // Returns the number of bits that are set to 1.
    constexpr std::size_t count() const noexcept { return countOnes(); }

    // Returns the number of bits that the BitArray holds.
    constexpr std::size_t size() const noexcept { return _array.size() * WORD_SIZE; }

    // Modifiers
    constexpr BitArray& set() noexcept {
      setBitsToOne();
      return *this;
    }

    constexpr BitArray& set(std::size_t pos, bool value = true) noexcept {
      if (value)
        setOneAt(pos);
      else
        setZeroAt(pos);
      return *this;
    }

    constexpr BitArray& reset() noexcept {
      setBitsToZero();
      return *this;
    }
    constexpr BitArray& reset(std::size_t pos) noexcept {
      setZeroAt(pos);
      return *this;
    }

    //---------------------------------------------------------------------------
    // Auxiliary methods
    //---------------------------------------------------------------------------

    NT_SIMD_CONSTEXPR_ static uint64_t _countOnesFast(uint64_t i_word) noexcept {
#if defined(NT_USE_SIMD)
      return _mm_popcnt_u64(i_word);
      // int      result;
      //__asm__ volatile("popcnt %1, %0" : "=r"(result) : "r"(i_word));
      // return result;
#else
      return std::popcount(i_word);
#endif
    }

    constexpr uint64_t& _getWordFromIndex(uint64_t index) noexcept {
      assert(index < static_cast< unsigned >(countWords()) * WORD_SIZE);
      return _array[NT_DIVIDE_BY_64_(index)];
    }
  };

  /**
   * @class BoolDynamicArray
   * @brief Memory-efficient dynamic array for boolean values.
   *
   * This class provides the same API as TrivialDynamicArray<bool> but uses
   * a compact bit-level representation (BitArray) internally.
   *
   * **API differences from TrivialDynamicArray<bool>:**
   * - `operator[]` returns a proxy object, not `bool&`
   * - Cannot take address of elements (`&arr[i]` won't compile)
   * - Iterators return proxy references
   *
   * Example of usage:
   *
   * @code
   * nt::BoolDynamicArray flags;
   * flags.add(true);
   * flags.add(false);
   * flags.add(true);
   *
   * if (flags[0]) {
   *   std::cout << "First flag is set\n";
   * }
   *
   * flags[1] = true;  // Set using proxy
   *
   * for (bool val : flags) {
   *   std::cout << val << " ";
   * }
   * @endcode
   */
  struct BoolDynamicArray {
    using value_type = bool;
    using size_type = int;
    using difference_type = ptrdiff_t;

    /**
     * @brief Proxy class for bool reference returned by operator[]
     *
     * This allows `arr[i] = true` to work even though we can't return
     * a true reference to a bit.
     */
    struct Reference {
      BitArray& _bitarray;
      int       _index;

      constexpr Reference(BitArray& bitarray, int index) : _bitarray(bitarray), _index(index) {}

      // Implicit conversion to bool for reading
      constexpr operator bool() const noexcept { return _bitarray.isOne(_index); }

      // Assignment for writing
      constexpr Reference& operator=(bool value) noexcept {
        if (value)
          _bitarray.setOneAt(_index);
        else
          _bitarray.setZeroAt(_index);
        return *this;
      }

      constexpr Reference& operator=(const Reference& other) noexcept { return *this = bool(other); }

      // For convenience
      constexpr bool operator~() const noexcept { return !bool(*this); }
    };

    /**
     * @brief Iterator template for BoolDynamicArray
     *
     * Template parameter ARRAY can be either BoolDynamicArray (for mutable iterator)
     * or const BoolDynamicArray (for const iterator).
     *
     * @tparam ARRAY The array type (BoolDynamicArray or const BoolDynamicArray)
     */
    template < typename ARRAY >
    struct BaseIterator {
      ARRAY* _array;
      int    _index;

      constexpr BaseIterator(ARRAY* array, int index) : _array(array), _index(index) {}

      using iterator_category = std::random_access_iterator_tag;
      using value_type = bool;
      using difference_type = ptrdiff_t;
      using pointer = void;   // Can't provide pointer to bit
      using reference = bool;

      constexpr bool operator*() const noexcept { return _array->_bitarray.isOne(_index); }

      constexpr BaseIterator& operator++() noexcept {
        ++_index;
        return *this;
      }

      constexpr BaseIterator operator++(int) noexcept {
        BaseIterator tmp = *this;
        ++_index;
        return tmp;
      }

      constexpr BaseIterator& operator--() noexcept {
        --_index;
        return *this;
      }

      constexpr BaseIterator operator--(int) noexcept {
        BaseIterator tmp = *this;
        --_index;
        return tmp;
      }

      constexpr BaseIterator& operator+=(difference_type n) noexcept {
        _index += n;
        return *this;
      }

      constexpr BaseIterator& operator-=(difference_type n) noexcept {
        _index -= n;
        return *this;
      }

      constexpr BaseIterator operator+(difference_type n) const noexcept { return BaseIterator(_array, _index + n); }

      constexpr BaseIterator operator-(difference_type n) const noexcept { return BaseIterator(_array, _index - n); }

      constexpr difference_type operator-(const BaseIterator& other) const noexcept { return _index - other._index; }

      constexpr bool operator==(const BaseIterator& other) const noexcept { return _index == other._index; }
      constexpr bool operator!=(const BaseIterator& other) const noexcept { return _index != other._index; }
      constexpr bool operator<(const BaseIterator& other) const noexcept { return _index < other._index; }
      constexpr bool operator>(const BaseIterator& other) const noexcept { return _index > other._index; }
      constexpr bool operator<=(const BaseIterator& other) const noexcept { return _index <= other._index; }
      constexpr bool operator>=(const BaseIterator& other) const noexcept { return _index >= other._index; }
    };

    using iterator = BaseIterator< BoolDynamicArray >;
    using const_iterator = BaseIterator< const BoolDynamicArray >;
    using reverse_iterator = std::reverse_iterator< iterator >;
    using const_reverse_iterator = std::reverse_iterator< const_iterator >;

    using reference = Reference;
    using const_reference = bool;

    BitArray _bitarray;   ///< Underlying bit storage (1 bit per boolean)
    int      _size;       ///< Number of boolean elements in the array

    //---------------------------------------------------------------------------
    // Constructors
    //---------------------------------------------------------------------------

    /**
     * @brief Default constructor - creates an empty array
     */
    BoolDynamicArray() noexcept : _size(0) {}

    /**
     * @brief Constructs the array with count default-initialized (false) elements
     *
     * @param count The number of elements to initialize
     */
    explicit BoolDynamicArray(int count) noexcept : _size(count) { _bitarray.extendByBits(count, 0); }

    /**
     * @brief Constructs the array with count elements initialized to value
     *
     * @param count The number of elements to initialize
     * @param value The value to initialize all elements with
     */
    BoolDynamicArray(int count, bool value) noexcept : _size(count) {
      _bitarray.extendByBits(count, value ? ~static_cast< int >(0) : 0);
    }

    /**
     * @brief Constructs the array from an initializer list
     *
     * @param init Initializer list of boolean values
     *
     * Example:
     * @code
     * BoolDynamicArray arr = {true, false, true, true};
     * @endcode
     */
    BoolDynamicArray(std::initializer_list< bool > init) noexcept : _size(init.size()) {
      _bitarray.extendByBits(_size, 0);
      int idx = 0;
      for (bool val: init) {
        if (val) _bitarray.setOneAt(idx);
        ++idx;
      }
    }

    BoolDynamicArray(const BoolDynamicArray&) = delete;
    BoolDynamicArray& operator=(const BoolDynamicArray&) = delete;

    /**
     * @brief Move constructor
     *
     */
    BoolDynamicArray(BoolDynamicArray&& other) noexcept : _bitarray(std::move(other._bitarray)), _size(other._size) {
      other._size = 0;
    }

    /**
     * @brief Move assignment operator
     *
     */
    BoolDynamicArray& operator=(BoolDynamicArray&& other) noexcept {
      if (this != &other) {
        _bitarray = std::move(other._bitarray);
        _size = other._size;
        other._size = 0;
      }
      return *this;
    }

    /**
     * @brief Assigns values from an initializer list
     *
     * @param init Initializer list of boolean values
     * @return Reference to this array
     */
    BoolDynamicArray& operator=(std::initializer_list< bool > init) noexcept {
      _size = init.size();
      _bitarray.removeAll();
      _bitarray.extendByBits(_size, 0);
      int idx = 0;
      for (bool val: init) {
        if (val) _bitarray.setOneAt(idx);
        ++idx;
      }
      return *this;
    }

    //---------------------------------------------------------------------------
    // Element access
    //---------------------------------------------------------------------------

    /**
     * @brief Returns whether the element at specified location index is true
     *
     * @param index Position of the element to be checked (0 <= index < size())
     * @return True if the element at the specified index is true, false otherwise
     *
     * @note No bounds checking is performed in release mode.
     */
    constexpr bool isTrue(int index) const noexcept {
      assert(index >= 0 && index < _size);
      return _bitarray.isOne(index);
    }

    /**
     * @brief Returns a reference proxy to the element at specified location index
     *
     * @param index Position of the element to return (0 <= index < size())
     * @return Proxy reference that allows reading and writing the boolean value
     *
     * @note No bounds checking is performed in release mode. Accessing out-of-bounds
     *       elements results in undefined behavior.
     */
    constexpr Reference operator[](int index) noexcept {
      assert(index >= 0 && index < _size);
      return Reference(_bitarray, index);
    }

    /**
     * @brief Returns the value of the element at specified location index (const version)
     *
     * @param index Position of the element to return (0 <= index < size())
     * @return The boolean value at the specified index
     *
     * @note No bounds checking is performed in release mode.
     */
    constexpr bool operator[](int index) const noexcept {
      assert(index >= 0 && index < _size);
      return _bitarray.isOne(index);
    }

    /**
     * @brief Returns a reference proxy to the element at specified location index
     *        Same as operator[]
     *
     * @param index Position of the element to return
     * @return Proxy reference to the element
     */
    constexpr Reference at(int index) noexcept { return operator[](index); }

    /**
     * @brief Returns the value of the element at specified location index (const version)
     *        Same as operator[]
     *
     * @param index Position of the element to return
     * @return The boolean value at the specified index
     */
    constexpr bool at(int index) const noexcept { return operator[](index); }

    /**
     * @brief Returns the value of the first element in the array
     *
     * @return The first boolean value
     *
     * @note Calling front() on an empty array results in undefined behavior
     */
    constexpr bool front() const noexcept {
      assert(_size > 0);
      return _bitarray.isOne(0);
    }

    /**
     * @brief Returns the value of the last element in the array
     *
     * @return The last boolean value
     *
     * @note Calling back() on an empty array results in undefined behavior
     */
    constexpr bool back() const noexcept {
      assert(_size > 0);
      return _bitarray.isOne(_size - 1);
    }

    //---------------------------------------------------------------------------
    // Capacity
    //---------------------------------------------------------------------------

    /**
     * @brief Checks whether the array is empty
     *
     * @return true if the array is empty, false otherwise
     */
    constexpr bool empty() const noexcept { return _size == 0; }

    /**
     * @brief Returns the number of elements in the array
     *
     * @return The number of boolean elements
     */
    constexpr int size() const noexcept { return _size; }

    /**
     * @brief Returns the number of elements that can be stored without reallocation
     *
     * @return The capacity in number of boolean elements
     */
    constexpr int capacity() const noexcept { return _bitarray._array.size() * BitArray::WORD_SIZE; }

    /**
     * @brief Increases the capacity of the array to a value that's at least new_capacity
     *
     * If new_capacity is greater than the current capacity(), new storage is allocated,
     * otherwise the method does nothing.
     *
     * @param new_capacity New capacity of the array in number of boolean elements
     */
    constexpr void reserve(int new_capacity) noexcept {
      if (new_capacity > capacity()) {
        const int i_words_needed = (new_capacity + BitArray::WORD_SIZE - 1) / BitArray::WORD_SIZE;
        _bitarray._array.ensure(i_words_needed);
      }
    }

    //---------------------------------------------------------------------------
    // Modifiers
    //---------------------------------------------------------------------------

    /**
     * @brief Removes all elements from the array
     *
     * Leaves the capacity unchanged. After this call, size() returns 0.
     */
    constexpr void clear() noexcept {
      _size = 0;
      _bitarray.removeAll();
    }

    /**
     * @brief Removes all elements from the array (same as clear())
     */
    constexpr void removeAll() noexcept { clear(); }

    /**
     * @brief Adds a new default-initialized (false) element to the end of the array
     *
     * @return The index of the newly added element
     */
    constexpr int add() noexcept {
      _bitarray.extendByBits(1, 0);
      return _size++;
    }

    /**
     * @brief Adds a new element with the specified value to the end of the array
     *
     * @param value The boolean value to add
     * @return The index of the newly added element
     */
    constexpr int add(bool value) noexcept {
      _bitarray.extendByBits(1, 0);
      const int idx = _size;
      if (value) _bitarray.setOneAt(idx);
      ++_size;
      return idx;
    }

    /**
     * @brief Adds a new element to the end of the array (STL-compatible)
     *
     * @param value The boolean value to add
     */
    constexpr void push_back(bool value) noexcept { add(value); }

    /**
     * @brief Removes the last element from the array
     *
     * @note Calling pop_back() on an empty array results in undefined behavior
     */
    constexpr void pop_back() noexcept {
      assert(_size > 0);
      --_size;
    }

    /**
     * @brief Resizes the array to contain i_new_size elements
     *
     * If i_new_size is greater than the current size, additional elements are
     * initialized to false.
     *
     * @param i_new_size New size of the array
     */
    constexpr void resize(int i_new_size) noexcept { resize(i_new_size, false); }

    /**
     * @brief Resizes the array to contain i_new_size elements
     *
     * If i_new_size is greater than the current size, additional elements are
     * initialized to value.
     *
     * @param i_new_size New size of the array
     * @param value The value to initialize new elements with
     */
    constexpr void resize(int i_new_size, bool value) noexcept {
      if (i_new_size <= 0) return;
      if (i_new_size > _size) {
        if (value) {
          _bitarray.extendByBits(i_new_size - _size, ~static_cast< int >(0));
          _bitarray.setBitsToOne(_size);
        } else {
          _bitarray.extendByBits(i_new_size - _size, 0);
          _bitarray.setBitsToZero(_size);
        }
      } else   // Reset the bits beyond the new size to make sure countOnes() works correctly
        _bitarray.setBitsToZero(i_new_size);

      _size = i_new_size;
    }

    /**
     * @brief Ensures that the array has sufficient allocated memory to hold at least
     *        i_new_capacity elements
     *
     * If the current capacity is less than i_new_capacity, the function reallocates
     * memory to accommodate the new capacity.
     *
     * @param i_new_capacity The desired minimum capacity for the array
     */
    constexpr void ensure(int i_new_capacity) noexcept { reserve(i_new_capacity); }

    /**
     * @brief Ensures that the array has enough capacity to store at least i_new_size elements,
     *        and fills any newly allocated memory with the specified value
     *
     * @param i_new_size The total number of elements the array should hold after the operation
     * @param value The value to initialize any newly added elements with (default: false)
     */
    constexpr void ensureAndFill(int i_new_size, bool value = false) noexcept {
      if (i_new_size > _size) _bitarray.extendByBits(i_new_size - _size, value ? ~static_cast< int >(0) : 0);
      _size = i_new_size;
    }

    /**
     * @brief Fills all elements in the array with the specified value
     *
     * @param value The boolean value to set for all elements
     */
    constexpr void fill(bool value) noexcept {
      if (value)
        _bitarray.setBitsToOne();
      else
        _bitarray.setBitsToZero();
    }

    /**
     * @brief Finds the first element equal to the specified value
     *
     * @param value The value to search for
     * @return The index of the first element equal to value, or size() if not found
     */
    constexpr int find(bool value) const noexcept {
      for (int i = 0; i < _size; ++i)
        if (_bitarray.isOne(i) == value) return i;
      return _size;
    }

    /**
     * @brief Replaces the contents with a copy of the contents of other
     *
     * @param other The array to copy from
     */
    constexpr void copyFrom(const BoolDynamicArray& other) noexcept {
      _size = other._size;
      _bitarray.copyFrom(other._bitarray);
    }

    /**
     * @brief Replaces the contents with a copy of the elements in the range [first, last)
     *
     * @param first The beginning of the range of elements to copy
     * @param last The end of the range of elements to copy
     */
    template < typename InputIt >
    constexpr void copyFrom(InputIt first, InputIt last) noexcept {
      clear();
      for (; first != last; ++first)
        add(*first);
    }

    //---------------------------------------------------------------------------
    // Iterators
    //---------------------------------------------------------------------------

    /**
     * @brief Returns an iterator to the first element of the array
     *
     * @return Iterator to the first element
     */
    constexpr iterator begin() noexcept { return iterator(this, 0); }

    /**
     * @brief Returns a const iterator to the first element of the array
     *
     * @return Const iterator to the first element
     */
    constexpr const_iterator begin() const noexcept { return const_iterator(this, 0); }

    /**
     * @brief Returns a const iterator to the first element of the array
     *
     * @return Const iterator to the first element
     */
    constexpr const_iterator cbegin() const noexcept { return const_iterator(this, 0); }

    /**
     * @brief Returns an iterator to the element following the last element of the array
     *
     * @return Iterator to the element following the last element
     */
    constexpr iterator end() noexcept { return iterator(this, _size); }

    /**
     * @brief Returns a const iterator to the element following the last element of the array
     *
     * @return Const iterator to the element following the last element
     */
    constexpr const_iterator end() const noexcept { return const_iterator(this, _size); }

    /**
     * @brief Returns a const iterator to the element following the last element of the array
     *
     * @return Const iterator to the element following the last element
     */
    constexpr const_iterator cend() const noexcept { return const_iterator(this, _size); }

    /**
     * @brief Returns a reverse iterator to the first element of the reversed array
     *
     * @return Reverse iterator to the first element of the reversed array
     */
    constexpr reverse_iterator rbegin() noexcept { return reverse_iterator(end()); }

    /**
     * @brief Returns a const reverse iterator to the first element of the reversed array
     *
     * @return Const reverse iterator to the first element of the reversed array
     */
    constexpr const_reverse_iterator rbegin() const noexcept { return const_reverse_iterator(end()); }

    /**
     * @brief Returns a const reverse iterator to the first element of the reversed array
     *
     * @return Const reverse iterator to the first element of the reversed array
     */
    constexpr const_reverse_iterator crbegin() const noexcept { return const_reverse_iterator(cend()); }

    /**
     * @brief Returns a reverse iterator to the element following the last element of the reversed array
     *
     * @return Reverse iterator to the element following the last element of the reversed array
     */
    constexpr reverse_iterator rend() noexcept { return reverse_iterator(begin()); }

    /**
     * @brief Returns a const reverse iterator to the element following the last element of the reversed array
     *
     * @return Const reverse iterator to the element following the last element of the reversed array
     */
    constexpr const_reverse_iterator rend() const noexcept { return const_reverse_iterator(begin()); }

    /**
     * @brief Returns a const reverse iterator to the element following the last element of the reversed array
     *
     * @return Const reverse iterator to the element following the last element of the reversed array
     */
    constexpr const_reverse_iterator crend() const noexcept { return const_reverse_iterator(cbegin()); }

    //---------------------------------------------------------------------------
    // BitArray-specific operations
    //---------------------------------------------------------------------------

    /**
     * @brief Set the bit at the specified index to the given value
     *
     * @param idx Index of the bit to set (0 <= idx < size())
     * @param value The boolean value to set the bit to
     */
    constexpr void set(int idx, bool value) noexcept {
      if (value)
        _bitarray.setOneAt(idx);
      else
        _bitarray.setZeroAt(idx);
    }

    /**
     * @brief Set the bit at the specified index to true
     *
     * @param idx Index of the bit to set to true
     */
    constexpr void setTrue(int idx) noexcept { _bitarray.setOneAt(idx); }

    /**
     * @brief Set all bits in the array to true
     */
    constexpr void setTrue() noexcept { _bitarray.setBitsToOne(); }

    /**
     * @brief Set the bit at the specified index to false
     *
     * @param idx Index of the bit to set to false
     */
    constexpr void setFalse(int idx) noexcept { _bitarray.setZeroAt(idx); }

    /**
     * @brief Set all bits in the array to false
     */
    constexpr void setFalse() noexcept { _bitarray.setBitsToZero(); }

    /**
     * @brief Check if all bits are set to true
     *
     * @return true if all elements are true, false otherwise
     */
    constexpr bool all() const noexcept { return _bitarray.all(); }

    /**
     * @brief Check if any bit is set to true
     *
     * @return true if at least one element is true, false otherwise
     */
    constexpr bool any() const noexcept { return _bitarray.any(); }

    /**
     * @brief Check if no bits are set to true
     *
     * @return true if all elements are false, false otherwise
     */
    constexpr bool none() const noexcept { return _bitarray.none(); }

    /**
     * @brief Count the number of bits set to true
     *
     * @return The number of elements with value true
     */
    constexpr int count() const noexcept { return _bitarray.countOnes(); }

    /**
     * @brief Flip all bits (toggle all boolean values)
     *
     * Converts all true values to false and all false values to true.
     */
    constexpr void flip() noexcept { _bitarray.flip(); }

    /**
     * @brief Flip a specific bit (toggle a single boolean value)
     *
     * Converts the value at index from true to false or false to true.
     *
     * @param index Position of the element to flip (0 <= index < size())
     */
    constexpr void flip(int index) noexcept {
      assert(index >= 0 && index < _size);
      if (_bitarray.isOne(index))
        _bitarray.setZeroAt(index);
      else
        _bitarray.setOneAt(index);
    }
  };

  /**
   * @class DynamicArray
   * @headerfile arrays.h
   * @brief This class provides an efficent dynamic array implementation.
   *
   * @tparam TYPE The type of elements stored in the array
   * @tparam SSO The size of the pre-allocated memory on the stack (default is 0, no pre-allocation)
   *
   */
  template < class TYPE, const int SSO = 0, const int GF_NUM = 1, const int GF_DEN = 2 >
  struct DynamicArray:
      std::conditional< std::is_trivially_copyable< TYPE >::value,
                        TrivialDynamicArray< TYPE, SSO, GF_NUM, GF_DEN >,
                        LazyDynamicArray< TYPE, SSO, GF_NUM, GF_DEN > >::type {
    using Parent = typename std::conditional< std::is_trivially_copyable< TYPE >::value,
                                              TrivialDynamicArray< TYPE, SSO, GF_NUM, GF_DEN >,
                                              LazyDynamicArray< TYPE, SSO, GF_NUM, GF_DEN > >::type;
    using Parent::Parent;
  };


  /**
   * @class FixedMemoryArray
   * @headerfile arrays.h
   * @brief This class implements an array in which each element retains a fixed memory address
   *        throughout the array's lifetime, regardless of any subsequent additions of elements.
   *
   * Example of usage:
   *
   * @code
   * #include "networktools/core/arrays.h"
   *
   * struct Dummy {
   *     (...)
   * };
   *
   * int main() {
   *  nt::FixedMemoryArray< Dummy, 1 > arr;
   *
   *  Dummy* p1 = &arr[arr.add()];
   *  assert(p1 == &arr[0]);
   *
   *  // After adding two more elements...
   *  arr.add();
   *  arr.add();
   *
   *  // ...the address value of the first element remains unchanged.
   *  assert(p1 == &arr[0]);
   * }
   * @endcode
   *
   * @tparam TYPE The type of the stored values.
   * @tparam K Power value that determines the size (BUFFER_SIZE) of each buffer i.e BUFFER_SIZE=2^K.
   * Check the following table for a quick reference:
   *
   *        K   |   BUFFER_SIZE
   *        -------------------
   *        1   |      2
   *        2   |      4
   *        3   |      8
   *        6   |     64
   *        12  |   4096
   *        16  |  65536
   *        19  | 524288
   *
   */
  template < class TYPE, int K >
  struct FixedMemoryArray {
    // static_assert that K is not "too big".
    static_assert(K > 0 && K <= 19, "K must be less or equal to 19 and greater or equal to 1.");
    constexpr static int BUFFER_SIZE = 1 << K;

    using value_type = TYPE;
    using size_type = int;
    using reference = TYPE&;
    using const_reference = const TYPE&;
    using pointer = TYPE*;
    using const_pointer = const TYPE*;

    using Buffer = DynamicArray< TYPE >;

    DynamicArray< Buffer > _buffers;
    int                    _i_current_buffer;

    FixedMemoryArray() : _i_current_buffer(-1) {}

    /**
     * @brief Inserts a new element to the end of the array.
     *
     * @return Returns the position of the inserted element.
     */
    template < typename... Args >
    constexpr int add(Args&&... args) noexcept {
      if (_buffers.empty() || _buffers[_i_current_buffer].size() >= BUFFER_SIZE) {
        _i_current_buffer = _buffers.add();
        _buffers[_i_current_buffer].ensure(BUFFER_SIZE);
      }
      Buffer&   buffer = _buffers[_i_current_buffer];
      const int i_index = buffer.addEmplace(std::forward< Args >(args)...);
      return (_i_current_buffer * BUFFER_SIZE) + i_index;
    }

    /**
     * @brief Returns a reference to the element at specified location index.
     *
     * @param index Position of the element to return
     * @return Reference to the requested element.
     */
    constexpr reference operator[](int index) noexcept {
      assert(index < size() && index >= 0);
      Buffer& buffer = _buffers[index >> K];
      return buffer[index & (BUFFER_SIZE - 1)];
    }

    constexpr const_reference operator[](int index) const noexcept {
      assert(index < size() && index >= 0);
      const Buffer& buffer = _buffers[index >> K];
      return buffer[index & (BUFFER_SIZE - 1)];
    }

    /**
     * @brief Returns a reference to the element at specified location index.
     *        Same as the operator[]
     *
     * @param index Position of the element to return
     * @return Reference to the requested element
     */
    constexpr reference       at(int index) noexcept { return operator[](index); }
    constexpr const_reference at(int index) const noexcept { return operator[](index); }

    /**
     * @brief Set the given value to the element at specified location index.
     *
     */
    constexpr void set(int index, const value_type& v) noexcept { operator[](index) = v; }
    constexpr void set(int index, value_type&& v) noexcept { operator[](index) = std::move(v); }

    /**
     * @brief Removes all elements from the array.
     *
     */
    constexpr void removeAll() noexcept {
      for (int i_buffer = 0; i_buffer < _buffers.size(); ++i_buffer)
        _buffers[i_buffer].removeAll();
      _buffers.removeAll();
      _i_current_buffer = -1;
    }

    /**
     * @brief Allocates enough memory to store i_new_size elements.
     *
     * @param i_new_size The new size to ensure capacity for.
     */
    constexpr void ensure(int i_new_size) noexcept {
      assert(i_new_size >= 0);
      const int i_delta = i_new_size - _buffers.capacity() * BUFFER_SIZE;
      if (i_delta <= 0) return;
      const int i_last_buffer = _buffers.capacity();
      _buffers.ensureAndFill(_buffers.capacity() + (i_delta >> K) + 1);
      for (int i_buffer = i_last_buffer; i_buffer < _buffers.capacity(); ++i_buffer) {
        assert(i_buffer < _buffers.capacity() && i_buffer >= 0);
        _buffers[i_buffer].ensure(BUFFER_SIZE);
      }
      if (_i_current_buffer < 0) _i_current_buffer = 0;
    }

    //---------------------------------------------------------------------------
    // STL methods
    //---------------------------------------------------------------------------

    // Capacity
    size_type size() const noexcept {
      return _buffers.size() ? (_buffers.size() - 1) * BUFFER_SIZE + _buffers[_buffers.size() - 1].size() : 0;
    }
    size_type capacity() const noexcept { return _buffers.capacity() * BUFFER_SIZE; }
    bool      empty() const noexcept { return _buffers.empty(); }
    void      reserve(size_type n) { ensure(n); }

    constexpr reference       back() noexcept { return operator[](_buffers.size() - 1); }
    constexpr const_reference back() const noexcept { return operator[](_buffers.size() - 1); }

    constexpr reference       front() noexcept { return operator[](0); }
    constexpr const_reference front() const noexcept { return operator[](0); }

    constexpr void clear() noexcept { removeAll(); }
  };

  /**
   * @class SymmetricArray
   * @headerfile arrays.h
   * @brief A symmetric array is a dynamic array in which each element in position i has a
   * corresponding element in position ~i (~ is the bitwise complement). A SymmetricArray
   * has size 2*n where n is the number of positive indices.
   *
   * Example of usage:
   *
   * @code
   * #include "networktools/core/arrays.h"
   *
   * int main() {
   *
   *  // Let's encode the path graph `0 -> 1 -> 2` with a SymmetricArray.
   *  // This representation allows to uniquely and quickly identify an arc i and its reverse
   *  // using the bitwise complement (i.e ~i) SymmetricArray<int> endpoints(2);
   *
   *  // Setting enpoints of arc 0 and its reverse ~0
   *  endpoints[ 0] = 1; // Target
   *  endpoints[~0] = 0; // Source
   *
   *  // Setting enpoints of arc 1  and its reverse ~1
   *  endpoints[ 1] = 2; // Target
   *  endpoints[~1] = 1; // Source
   *
   *  // Display each arc and its reverse
   *  for(int i_arc = 0; i_arc < endpoints.half_size(); ++i_arc) {
   *    nt::printf("Arc {} : {} -> {}, reverse : {} -> {}\n",
   *      i_arc,
   *      endpoints[~i_arc], endpoints[i_arc],
   *      endpoints[i_arc],  endpoints[~i_arc]
   *    );
   * }
   *@endcode
   *
   * @tparam TYPE The type of the stored values.
   *
   */
  template < class TYPE >
  struct SymmetricArray {
    using value_type = TYPE;
    using size_type = int;
    using reference = TYPE&;
    using const_reference = const TYPE&;
    using pointer = value_type*;
    using const_pointer = const value_type*;

    nt::TrivialDynamicArray< value_type > _zarray;
    pointer                               _p_middle;   //< Points to the element in the middle of _zarray.

    constexpr SymmetricArray() noexcept : _p_middle(nullptr) {}
    explicit SymmetricArray(int i_size) noexcept {
      _zarray.ensureAndFill(i_size << 1);
      _p_middle = _zarray.data() + i_size;
    }
    explicit SymmetricArray(int i_size, const value_type& v) noexcept {
      _zarray.ensureAndFill(i_size << 1, v);
      _p_middle = _zarray.data() + i_size;
    }

    /**
     * @brief Inserts a default created element into the array.
     *
     * @return Returns the position of the inserted element.
     */
    constexpr int add() noexcept {
      ensureAndFill(_zarray.size() + 1);
      return _zarray.size() - 1;
    }

    /**
     * @brief
     *
     * @param i_size
     */
    constexpr void ensureAndFill(size_type i_size) noexcept {
      _zarray.ensureAndFill(i_size << 1);
      _p_middle = _zarray.data() + i_size;
    }

    /**
     * @brief
     *
     */
    constexpr void removeAll() noexcept { _zarray.removeAll(); }

    /**
     * @brief Copies the value static_cast<unsigned char>(c) to every byte of the memory pointed
     * by Datas.
     *
     * @param c fill byte
     */
    constexpr void fillBytes(int c) noexcept { _zarray.fillBytes(c); }

    /**
     * @brief Set every bit of the memory pointed by Datas to 0.
     *
     */
    constexpr void setBitsToZero() noexcept { _zarray.setBitsToZero(); }

    /**
     * @brief Set every bit of the memory pointed by Datas to 1.
     *
     */
    constexpr void setBitsToOne() noexcept { _zarray.setBitsToOne(); }

    /**
     * @brief
     *
     * @param i_zindex
     * @return
     */
    constexpr bool isValidZIndex(int i_zindex) const noexcept {
      const int i_half_count = _zarray.size() >> 1;
      return (-i_half_count <= i_zindex) && (i_zindex < i_half_count);
    }

    /**
     * @brief Access operator relatively to _p_middle
     *
     * @param i
     * @return reference
     */
    constexpr reference operator[](int index) noexcept {
      assert(_p_middle);
      assert(-(_zarray.size() >> 1) <= index);
      assert((_zarray.size() >> 1) - 1 >= index);
      return _p_middle[index];
    }

    constexpr const_reference operator[](int index) const noexcept {
      assert(_p_middle);
      assert(-(_zarray.size() >> 1) <= index);
      assert((_zarray.size() >> 1) - 1 >= index);
      return _p_middle[index];
    }

    /**
     * @brief Returns a reference to the element at specified location index.
     *        Same as the operator[]
     *
     * @param index Position of the element to return
     * @return Reference to the requested element
     */
    constexpr reference       at(int index) noexcept { return operator[](index); }
    constexpr const_reference at(int index) const noexcept { return operator[](index); }

    /**
     * @brief Set the given value to the element at specified location index.
     *
     */
    constexpr void set(int index, const value_type& v) noexcept { operator[](index) = v; }
    constexpr void set(int index, value_type&& v) noexcept { operator[](index) = std::move(v); }

    constexpr size_type size() const noexcept { return _zarray.size(); }
    constexpr size_type half_size() const noexcept { return _zarray.size() >> 1; }

    constexpr bool empty() const noexcept { return _zarray.empty(); }

    constexpr pointer       middle() noexcept { return _p_middle; }
    constexpr const_pointer middle() const noexcept { return _p_middle; }
  };

  /**
   * @class MultiDimArrayBase
   * @headerfile arrays.h
   * @brief Represents a multi-dimensional array. Serves as a base class for the multi-dim array
   * based types and should not be used directly (see MultiDimArray).
   *
   * Values are stored in a single array in row-major order.
   *
   * @tparam ARRAY The underlying one-dimensional array type.
   * @tparam DIM The number of dimensions of the multi-dimensional array. Must be greater or equal
   * to 2.
   */
  template < int DIM, class ARRAY >
  struct MultiDimArrayBase {
    static_assert(DIM > 1, "Number of dimensions must be at least 2.");
    using value_type = typename ARRAY::value_type;
    using reference = value_type&;
    using const_reference = const value_type&;
    using pointer = value_type*;
    using const_pointer = const value_type*;
    using size_type = int;
    using difference_type = ptrdiff_t;
    using iterator = pointer;
    using const_iterator = const_pointer;

    ARRAY _values;            //< array that stores the actual values
    int   _psizes[DIM - 1];   //< array that stores the products of the last DIM-1 dimensions i.e
                              //_psizes[i] = d(i+1) x ... x d(DIM-1)

    MultiDimArrayBase() = default;
    template < class... DIMS >
    MultiDimArrayBase(int d, DIMS... dims) noexcept {
      setDimensions(d, dims...);
    }
    MultiDimArrayBase(const MultiDimArrayBase&) = delete;
    MultiDimArrayBase& operator=(const MultiDimArrayBase&) = delete;
    MultiDimArrayBase(MultiDimArrayBase&& other) noexcept : _values(std::move(other._values)) {
      for (int k = 0; k < DIM - 1; ++k) {
        _psizes[k] = other._psizes[k];
        other._psizes[k] = 0;
      }
    }

    MultiDimArrayBase& operator=(MultiDimArrayBase&& other) noexcept {
      if (this == &other) return *this;
      _values = std::move(other._values);
      for (int k = 0; k < DIM - 1; ++k) {
        _psizes[k] = other._psizes[k];
        other._psizes[k] = 0;
      }
      return *this;
    }

    /**
     * @brief Return an array view of the matrix row given by the DIM-1 first indices.
     *
     * Let M be the following MultiDimArray of dimension 2:
     *
     *      a00 a01 a02
     * M =  a10 a11 a12
     *      a20 a22 a22
     *
     * then
     *
     *  M.row(0) = [a00 a01 a02]
     *  M.row(1) = [a10 a11 a12]
     *  M.row(2) = [a20 a22 a22]
     *
     * The elements in the returned row are guaranteed to be stored contiguously in memory.
     *
     */
    template < class... INDICES >
    constexpr ArrayView< value_type > row(INDICES... indices) const noexcept {
      static_assert(sizeof...(INDICES) == DIM - 1, "Number of given indices mismatch the array's dimension.");
      const int k = _offset< 0 >(indices..., 0);
      return ArrayView(&_values[k], dimSize< DIM - 1 >());
    }

    /**
     * @brief Set the dimension sizes of the array.
     *
     * The function allocates enough memory to store d(0) x d(1) x ... x d(DIM-1) elements where
     * d(i) is the size of dimension i. The number of dimensions should match the array's
     * dimension, specified by `DIM`, otherwise a compile-time error is thrown.
     *
     * @tparam DIMS Variadic template that specifies the dimensions of the array.
     * @param dims Sizes of each dimension of the array.
     */
    template < class... DIMS >
    void setDimensions(int d, DIMS... dims) noexcept {
      static_assert(sizeof...(dims) + 1 == DIM, "Number of given indices mismatch the array's dimension.");
      const int i_size = d * (dims * ...);
      assert(i_size > 0);   // Null or negative dimension definition is not authorized.
      _values.removeAll();
      _values.ensureAndFill(i_size);
      _setSizes< 0 >(dims...);
    };

    /**
     * @brief Replaces the contents with a copy of the contents of from.
     *
     */
    constexpr void copyFrom(const MultiDimArrayBase& from) noexcept {
      _values.copyFrom(from._values);
      std::memcpy(_psizes, from._psizes, (DIM - 1) * sizeof(int));
    }

    /**
     * @brief Returns the number of dimensions of the array.
     *
     * @return The number of dimensions of the array as a constant expression.
     */
    constexpr int dimNum() const noexcept { return DIM; }

    /**
     * @brief Returns the size of the I-th dimension.
     *
     * @return The size of the I-th dimension.
     */
    template < int I >
    constexpr int dimSize() const noexcept {
      static_assert(I < DIM);
      if constexpr (I == 0)
        return domainSize() / _psizes[0];
      else if constexpr (I == DIM - 1)
        return _psizes[I - 1];
      else
        return _psizes[I - 1] / _psizes[I];
    }

    /**
     * @brief Returns the product over all dimension sizes.
     *
     * @return The product over all dimensions sizes.
     */
    constexpr int domainSize() const noexcept { return _values.size(); }

    /**
     * @brief Access operator to the element at the specified indices of the multi-dimensional
     * array.
     *
     * This operator allows accessing an element of the multi-dimensional array using its indices.
     * The number of given indices must match the dimension of the array.
     *
     * @tparam INDICES Types of indices to access the element of the multi-dimensional array.
     * @param indices Indices of the element to access.
     *
     */
    template < class... INDICES >
    constexpr const value_type& operator()(INDICES... indices) const noexcept {
      static_assert(sizeof...(INDICES) == DIM, "Number of given indices mismatch the array's dimension.");
      const int k = _offset< 0 >(indices...);
      return _values[k];
    }

    template < class... INDICES >
    constexpr value_type& operator()(INDICES... indices) noexcept {
      static_assert(sizeof...(INDICES) == DIM, "Number of given indices mismatch the array's dimension.");
      const int k = _offset< 0 >(indices...);
      return _values[k];
    }

    /**
     * @brief Returns a reference to the element at the specified indices of the multi-dimensional array.
     *        Same as the operator()
     *
     * @tparam INDICES Types of indices to access the element of the multi-dimensional array.
     * @param indices Indices of the element to access.
     * @return Reference to the requested element
     */
    template < class... INDICES >
    constexpr reference at(INDICES... indices) noexcept {
      return operator()(indices...);
    }

    template < class... INDICES >
    constexpr const_reference at(INDICES... indices) const noexcept {
      return operator()(indices...);
    }

    /**
     * @brief Set the given value to all elements
     *
     */
    constexpr void fill(const value_type& v) noexcept { _values.fill(v); }

    /**
     * @brief Set the given value to the element located at the given indices
     *
     */
    template < class... INDICES >
    void set(const value_type& v, INDICES... indices) {
      static_assert(sizeof...(INDICES) == DIM, "Number of given indices mismatch the array's dimension.");
      const int k = _offset< 0 >(indices...);
      _values.set(k, v);
    }
    template < class... INDICES >
    void set(value_type&& v, INDICES... indices) {
      static_assert(sizeof...(INDICES) == DIM, "Number of given indices mismatch the array's dimension.");
      const int k = _offset< 0 >(indices...);
      _values.set(k, std::forward< value_type >(v));
    }

    //---------------------------------------------------------------------------
    // STL methods
    //---------------------------------------------------------------------------

    constexpr reference       operator[](int index) noexcept { return _values[index]; }
    constexpr const_reference operator[](int index) const noexcept { return _values[index]; }

    // Iterators
    constexpr iterator       begin() noexcept { return _values.begin(); }
    constexpr const_iterator begin() const noexcept { return _values.begin(); }
    constexpr const_iterator cbegin() const noexcept { return _values.cbegin(); }

    constexpr iterator       end() noexcept { return _values.end(); }
    constexpr const_iterator end() const noexcept { return _values.end(); }
    constexpr const_iterator cend() const noexcept { return _values.cend(); }

    // Capacity
    constexpr size_type size() const noexcept { return _values.size(); }
    constexpr bool      empty() const noexcept { return _values.empty(); }

    // Element access
    constexpr pointer       data() noexcept { return _values.data(); }
    constexpr const_pointer data() const noexcept { return _values.data(); }

    //---------------------------------------------------------------------------
    // Auxiliary methods
    //---------------------------------------------------------------------------

    template < int I, class... INDICES >
    constexpr int _offset(int k, INDICES... indices) const noexcept {
      static_assert(I < DIM);
      if constexpr (I == DIM - 1)
        return k;
      else
        return k * _psizes[I] + _offset< I + 1 >(indices...);
    }

    template < int I, class... DIMS >
    constexpr void _setSizes(int k, DIMS... dims) noexcept {
      static_assert(I < DIM);
      if constexpr (sizeof...(dims) == 0)
        _psizes[I] = k;
      else {
        _setSizes< I + 1 >(dims...);
        _psizes[I] = k * _psizes[I + 1];
      }
    }
  };

  /**
   * @class TrivialMultiDimArray
   * @headerfile arrays.h
   * @brief Specialization of the MultiDimArrayBase class for trivially copyable types. See
   * MultiDimArray for more details.
   *
   * @tparam TYPE The type of the stored values
   * @tparam DIM The number of dimensions of the multi-dimensional array. Must be greater or equal
   * to 2.
   *
   */
  template < class TYPE, int DIM >
  struct TrivialMultiDimArray: MultiDimArrayBase< DIM, TrivialDynamicArray< TYPE > > {
    using Parent = MultiDimArrayBase< DIM, TrivialDynamicArray< TYPE > >;
    using Parent::Parent;

    /**
     * @brief Fills the memory with the byte pattern (static_cast<unsigned char>(c)).
     *
     * @param c fill byte (interpreted as a byte pattern)
     */
    constexpr void fillBytes(int c) noexcept { Parent::_values.fillBytes(c); }

    /**
     * @brief Set every bit of the memory pointed by Datas to 0.
     *
     */
    constexpr void setBitsToZero() noexcept { Parent::_values.setBitsToZero(); }

    /**
     * @brief Set every bit of the memory pointed by Datas to 1.
     *
     */
    constexpr void setBitsToOne() noexcept { Parent::_values.setBitsToOne(); }
  };


  /**
   * @class LazyMultiDimArray
   * @headerfile arrays.h
   * @brief Specialization of the MultiDimArrayBase class for non-trivial types. See MultiDimArray
   * for more details.
   *
   * @tparam TYPE The type of the stored values
   * @tparam DIM The number of dimensions of the multi-dimensional array. Must be greater or equal
   * to 2.
   *
   */
  template < class TYPE, int DIM >
  struct LazyMultiDimArray: MultiDimArrayBase< DIM, LazyDynamicArray< TYPE > > {
    using Parent = MultiDimArrayBase< DIM, LazyDynamicArray< TYPE > >;
    using Parent::Parent;
  };


  /**
   * @class MultiDimArray
   * @headerfile arrays.h
   * @brief Represents a multi-dimensional array.
   *
   * Values are stored in a single array in row-major order.
   *
   * Example of usage:
   *
   * @code
   * #include "networktools/core/format.h"
   * #include "networktools/core/arrays.h"
   *
   * int main() {
   *  MultiDimArray<int, 3> m;
   *  m.setDimensions(4, 5, 2); // Defines a 4x5x2 multidimensional matrix
   *  m(3, 4, 1) = 1;           // Set the value 1 at position (3,4,1)
   *  m.setDimensions(2, 3, 5); // Reset the dimensions to 2x3x5. All previously stored values are
   * lost.
   *
   *  // Iterate and print all matrix values
   *  for (int i = 0; i < m.dimSize< 0 >(); ++i)
   *    for (int j = 0; j < m.dimSize< 1 >(); ++j)
   *      for (int k = 0; k < m.dimSize< 2 >(); ++k)
   *        nt::printf("m[{},{},{}] = {}\n", i, j, k, m(i, j, k));
   * }
   * @endcode
   *
   * @tparam TYPE The type of the stored values
   * @tparam DIM The number of dimensions of the multi-dimensional array. Must be greater or equal
   * to 2.
   */
  template < class TYPE, int DIM >
  struct MultiDimArray:
      std::conditional< std::is_trivial< TYPE >::value,
                        TrivialMultiDimArray< TYPE, DIM >,
                        LazyMultiDimArray< TYPE, DIM > >::type {
    using Parent = typename std::conditional< std::is_trivial< TYPE >::value,
                                              TrivialMultiDimArray< TYPE, DIM >,
                                              LazyMultiDimArray< TYPE, DIM > >::type;
    using Parent::Parent;
  };

  //---------------------------------------------------------------------------
  // Commonly used dynamic array types
  //---------------------------------------------------------------------------

  using ByteDynamicArray = TrivialDynamicArray< std::int8_t >;
  using UbyteDynamicArray = TrivialDynamicArray< std::uint8_t >;
  using Int16DynamicArray = TrivialDynamicArray< std::int16_t >;
  using Uint16DynamicArray = TrivialDynamicArray< std::uint16_t >;
  using IntDynamicArray = TrivialDynamicArray< std::int32_t >;
  using Int64DynamicArray = TrivialDynamicArray< std::int64_t >;
  using UintDynamicArray = TrivialDynamicArray< std::uint32_t >;
  using Uint64DynamicArray = TrivialDynamicArray< std::uint64_t >;
  using FloatDynamicArray = TrivialDynamicArray< float >;
  using DoubleDynamicArray = TrivialDynamicArray< double >;
  using LongDoubleDynamicArray = TrivialDynamicArray< long double >;

  //---------------------------------------------------------------------------
  // Commonly used array view types
  //---------------------------------------------------------------------------

  using ByteArrayView = ArrayView< std::int8_t >;
  using UbyteArrayView = ArrayView< std::uint8_t >;
  using Int16ArrayView = ArrayView< std::int16_t >;
  using Uint16ArrayView = ArrayView< std::uint16_t >;
  using IntArrayView = ArrayView< std::int32_t >;
  using Int64ArrayView = ArrayView< std::int64_t >;
  using UintArrayView = ArrayView< std::uint32_t >;
  using Uint64ArrayView = ArrayView< std::uint64_t >;
  using FloatArrayView = ArrayView< float >;
  using DoubleArrayView = ArrayView< double >;
  using LongDoubleArrayView = ArrayView< long double >;

  //---------------------------------------------------------------------------
  // Commonly used multidim array types
  //---------------------------------------------------------------------------

  template < int DIM >
  using ByteMultiDimArray = TrivialMultiDimArray< std::int8_t, DIM >;
  template < int DIM >
  using UbyteMultiDimArray = TrivialMultiDimArray< std::uint8_t, DIM >;
  template < int DIM >
  using Int16MultiDimArray = TrivialMultiDimArray< std::int16_t, DIM >;
  template < int DIM >
  using Uint16MultiDimArray = TrivialMultiDimArray< std::uint16_t, DIM >;
  template < int DIM >
  using IntMultiDimArray = TrivialMultiDimArray< std::int32_t, DIM >;
  template < int DIM >
  using Int64MultiDimArray = TrivialMultiDimArray< std::int64_t, DIM >;
  template < int DIM >
  using UintMultiDimArray = TrivialMultiDimArray< std::uint32_t, DIM >;
  template < int DIM >
  using Uint64MultiDimArray = TrivialMultiDimArray< std::uint64_t, DIM >;
  template < int DIM >
  using FloatMultiDimArray = TrivialMultiDimArray< float, DIM >;
  template < int DIM >
  using DoubleMultiDimArray = TrivialMultiDimArray< double, DIM >;
  template < int DIM >
  using LongDoubleMultiDimArray = TrivialMultiDimArray< long double, DIM >;
  template < int DIM >
  using BoolMultiDimArray = TrivialMultiDimArray< bool, DIM >;   // TODO(perf): should use BitArray

}   // namespace nt

#endif
