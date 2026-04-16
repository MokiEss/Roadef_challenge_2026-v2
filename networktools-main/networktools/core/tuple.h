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
 * @brief Generic tuple data structure.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_TUPLE_H_
#define _NT_TUPLE_H_

#include <utility>
#include <tuple>

namespace nt {
  /**
   * @brief Generic Pair data structure.
   *
   * @tparam FIRST The type of the first element in the pair.
   * @tparam SECOND The type of the second element in the pair.
   */
  template < class FIRST, class SECOND >
  struct Pair {
    using First = FIRST;
    using Second = SECOND;
    using first_type = FIRST;
    using second_type = SECOND;

    First  first;
    Second second;

    Pair() = default;
    Pair(const First& f, const Second& s) : first(f), second(s) {}
    template<class U1, class U2>
    Pair(U1&& f, U2&& s) : first(std::forward<U1>(f)), second(std::forward<U2>(s)) {}
  };

  /**
   * @brief Generic Triple data structure.
   *
   * @tparam FIRST The type of the first element in the triple.
   * @tparam SECOND The type of the second element in the triple.
   * @tparam THIRD The type of the third element in the triple.
   */
  template < class FIRST, class SECOND, class THIRD >
  struct Triple {
    using First = FIRST;
    using Second = SECOND;
    using Third = THIRD;
    using first_type = FIRST;
    using second_type = SECOND;
    using third_type = THIRD;

    First  first;
    Second second;
    Third  third;

    Triple() = default;
    Triple(const First& f, const Second& s, const Third& t) : first(f), second(s), third(t) {}
    template<class U1, class U2, class U3>
    Triple(U1&& f, U2&& s, U3&& t) : first(std::forward<U1>(f)), second(std::forward<U2>(s)), third(std::forward<U3>(t)) {}
  };

}   // namespace nt

#endif