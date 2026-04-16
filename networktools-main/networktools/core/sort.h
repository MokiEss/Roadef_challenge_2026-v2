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
 *
 * ============================================================================
 *
 * This file incorporates work from the LEMON graph library (LEMON library).
 *
 * Original LEMON Copyright Notice:
 * Copyright (C) 2003-2013 Egervary Jeno Kombinatorikus Optimalizalasi
 * Kutatocsoport (Egervary Research Group on Combinatorial Optimization, EGRES).
 *
 * Permission to use, modify and distribute this software is granted provided
 * that this copyright notice appears in all copies. For precise terms see the
 * accompanying LICENSE file.
 *
 * This software is provided "AS IS" with no warranty of any kind, express or
 * implied, and with no claim as to its suitability for any purpose.
 *
 * ============================================================================
 * 
 * List of modifications:
 *   - Changed namespace from 'lemon' to 'nt'
 *   - Replaced std::vector with nt custom containers
 *   - Converted typedef declarations to C++11 using declarations
 *   - Added constexpr qualifiers for compile-time optimization
 *   - Modified exception handling strategy
 */

/**
 * @ingroup auxalg
 * @file
 * @brief Various sort algorithms
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_SORT_H_
#define _NT_SORT_H_

// Custom sort.h to avoid STL dependency (<algorithm> header)

#include "arrays.h"

#include <algorithm>   // TODO: to be replaced

namespace nt {

  /**
   *  @brief This does what you think it does.
   *  @param  a A thing of arbitrary type.
   *  @param  b Another thing of arbitrary type.
   *  @return The lesser of the parameters.
   *
   *  This is the simple classic generic implementation.  It will work on
   *  temporary expressions, since they are only evaluated once, unlike a
   *  preprocessor macro.
   */
  template < typename T >
  constexpr inline const T& min(const T& a, const T& b) {
    if (b < a) return b;
    return a;
  }

  /**
   *  @brief This does what you think it does.
   *  @param  a A thing of arbitrary type.
   *  @param  b Another thing of arbitrary type.
   *  @return The greater of the parameters.
   *
   *  This is the simple classic generic implementation.  It will work on
   *  temporary expressions, since they are only evaluated once, unlike a
   *  preprocessor macro.
   */
  template < typename T >
  constexpr inline const T& max(const T& a, const T& b) {
    if (a < b) return b;
    return a;
  }

  /**
   *  @brief Sort the elements of a sequence using a predicate for comparison.
   *  @param  array   An iterator.
   *  @param  compare    A comparison functor.
   *  @return  Nothing.
   *
   *  Sorts the elements in the array in ascending order,
   *  such that *(i+1)<*i is false for every iterator @e i.
   *
   */
  template < typename ARRAY >
  constexpr inline void sort(ARRAY& array) {
    std::sort(array.data(), array.data() + array.size());
  }

  /**
   *  @brief Sort the elements of a sequence using a predicate for comparison.
   *  @param  array   An iterator.
   *  @param  compare    A comparison functor.
   *  @return  Nothing.
   *
   *  Sorts the elements in the array in ascending order,
   *  such that @p compare(*(i+1),*i) is false for every iterator @e i.
   *
   */
  template < typename ARRAY, typename COMPARE >
  constexpr inline void sort(ARRAY& array, COMPARE compare) {
    //   // TODO : For integrals, use std::sort for "small" arrays and radix sort for "large" array.

    //   // if constexpr (std::is_integral< T_TYPE >::value) {
    //   //   if (m.Count < STD_SORT_THRESHOLD)
    //   //     std::sort(first, last);
    //   //   else
    //   //     radix_sort(first, last)
    //   // } else
    std::sort(array.data(), array.data() + array.size(), compare);
  }

  /**
   * @brief Reorder the elements in array according to the provided permutation.
   * For instance, let tab = [a b c] be an array, then a call to tab.reorder([1 0 2])
   * will result in tab = [b a c].
   *
   * This function modifies the permutation array.
   * Use reorderStable if you need to keep permutation untouched.
   *
   * @param array The array to be reordered
   * @param permutation The array containing the permutation
   *
   */
  template < typename ARRAY >
  constexpr inline void reorder(ARRAY& array, IntDynamicArray& permutation) noexcept {
    assert(array.size() == permutation.size());
    for (int k = 0; k < array.size(); ++k) {
      while (permutation[k] != k) {
        std::swap(array[permutation[k]], array[k]);
        std::swap(permutation[permutation[k]], permutation[k]);
      }
    }
  }

  /**
   * @brief Same as reorder() but do not modify the permutation array
   *
   * @param array The array to be reordered
   * @param permutation The array containing the permutation
   */
  template < typename ARRAY >
  inline void reorderStable(ARRAY& array, IntArrayView permutation) noexcept {
    IntDynamicArray permutationCpy(array.size());
    permutationCpy.copyFrom(permutation);
    reorder(array, permutationCpy);
  }

  namespace _radix_sort_bits {

    template < typename Iterator >
    bool unitRange(Iterator first, Iterator last) {
      ++first;
      return first == last;
    }

    template < typename Value >
    struct Identity {
      const Value& operator()(const Value& val) { return val; }
    };

    template < typename Value, typename Iterator, typename Functor >
    Iterator radixSortPartition(Iterator first, Iterator last, Functor functor, Value mask) {
      while (first != last && !(functor(*first) & mask)) {
        ++first;
      }
      if (first == last) { return first; }
      --last;
      while (first != last && (functor(*last) & mask)) {
        --last;
      }
      if (first == last) { return first; }
      std::iter_swap(first, last);
      ++first;
      while (true) {
        while (!(functor(*first) & mask)) {
          ++first;
        }
        --last;
        while (functor(*last) & mask) {
          --last;
        }
        if (unitRange(last, first)) { return first; }
        std::iter_swap(first, last);
        ++first;
      }
    }

    template < typename Iterator, typename Functor >
    Iterator radixSortSignPartition(Iterator first, Iterator last, Functor functor) {
      while (first != last && functor(*first) < 0) {
        ++first;
      }
      if (first == last) { return first; }
      --last;
      while (first != last && functor(*last) >= 0) {
        --last;
      }
      if (first == last) { return first; }
      std::iter_swap(first, last);
      ++first;
      while (true) {
        while (functor(*first) < 0) {
          ++first;
        }
        --last;
        while (functor(*last) >= 0) {
          --last;
        }
        if (unitRange(last, first)) { return first; }
        std::iter_swap(first, last);
        ++first;
      }
    }

    template < typename Value, typename Iterator, typename Functor >
    void radixIntroSort(Iterator first, Iterator last, Functor functor, Value mask) {
      while (mask != 0 && first != last && !unitRange(first, last)) {
        Iterator cut = radixSortPartition(first, last, functor, mask);
        mask >>= 1;
        radixIntroSort(first, cut, functor, mask);
        first = cut;
      }
    }

    template < typename Value, typename Iterator, typename Functor >
    void radixSignedSort(Iterator first, Iterator last, Functor functor) {
      Iterator cut = radixSortSignPartition(first, last, functor);

      Value    mask;
      int      max_digit;
      Iterator it;

      mask = ~0;
      max_digit = 0;
      for (it = first; it != cut; ++it) {
        while ((mask & functor(*it)) != mask) {
          ++max_digit;
          mask <<= 1;
        }
      }
      radixIntroSort(first, cut, functor, 1 << max_digit);

      mask = 0;
      max_digit = 0;
      for (it = cut; it != last; ++it) {
        while ((mask | functor(*it)) != mask) {
          ++max_digit;
          mask <<= 1;
          mask |= 1;
        }
      }
      radixIntroSort(cut, last, functor, 1 << max_digit);
    }

    template < typename Value, typename Iterator, typename Functor >
    void radixUnsignedSort(Iterator first, Iterator last, Functor functor) {
      Value mask = 0;
      int   max_digit = 0;

      Iterator it;
      for (it = first; it != last; ++it) {
        while ((mask | functor(*it)) != mask) {
          ++max_digit;
          mask <<= 1;
          mask |= 1;
        }
      }
      radixIntroSort(first, last, functor, 1 << max_digit);
    }

    template < typename Value, bool sign = std::numeric_limits< Value >::is_signed >
    struct RadixSortSelector {
      template < typename Iterator, typename Functor >
      static void sort(Iterator first, Iterator last, Functor functor) {
        radixSignedSort< Value >(first, last, functor);
      }
    };

    template < typename Value >
    struct RadixSortSelector< Value, false > {
      template < typename Iterator, typename Functor >
      static void sort(Iterator first, Iterator last, Functor functor) {
        radixUnsignedSort< Value >(first, last, functor);
      }
    };

  }   // namespace _radix_sort_bits

  /**
   * @ingroup auxalg
   *
   * @brief Sorts the STL compatible range into ascending order.
   *
   * The \c radixSort sorts an STL compatible range into ascending
   * order.  The radix sort algorithm can sort items which are mapped
   * to integers with an adaptable unary function \c functor and the
   * order will be ascending according to these mapped values.
   *
   * It is also possible to use a normal function instead
   * of the functor object. If the functor is not given it will use
   * the identity function instead.
   *
   * This is a special quick sort algorithm where the pivot
   * values to split the items are choosen to be 2<sup>k</sup>
   * for each \c k.
   * Therefore, the time complexity of the algorithm is O(log(c)*n) and
   * it uses O(log(c)) additional space, where \c c is the maximal value
   * and \c n is the number of the items in the container.
   *
   * @param first The begin of the given range.
   * @param last The end of the given range.
   * @param functor An adaptible unary function or a normal function
   * which maps the items to any integer type which can be either
   * signed or unsigned.
   *
   * \sa stableRadixSort()
   */
  template < typename Iterator, typename Functor >
  void radixSort(Iterator first, Iterator last, Functor functor) {
    using namespace _radix_sort_bits;
    using Value = typename Functor::result_type;
    RadixSortSelector< Value >::sort(first, last, functor);
  }

  template < typename Iterator, typename Value, typename Key >
  void radixSort(Iterator first, Iterator last, Value (*functor)(Key)) {
    using namespace _radix_sort_bits;
    RadixSortSelector< Value >::sort(first, last, functor);
  }

  template < typename Iterator, typename Value, typename Key >
  void radixSort(Iterator first, Iterator last, Value& (*functor)(Key)) {
    using namespace _radix_sort_bits;
    RadixSortSelector< Value >::sort(first, last, functor);
  }

  template < typename Iterator, typename Value, typename Key >
  void radixSort(Iterator first, Iterator last, Value (*functor)(Key&)) {
    using namespace _radix_sort_bits;
    RadixSortSelector< Value >::sort(first, last, functor);
  }

  template < typename Iterator, typename Value, typename Key >
  void radixSort(Iterator first, Iterator last, Value& (*functor)(Key&)) {
    using namespace _radix_sort_bits;
    RadixSortSelector< Value >::sort(first, last, functor);
  }

  template < typename Iterator >
  void radixSort(Iterator first, Iterator last) {
    using namespace _radix_sort_bits;
    using Value = typename std::iterator_traits< Iterator >::value_type;
    RadixSortSelector< Value >::sort(first, last, Identity< Value >());
  }

}   // namespace nt

#endif
