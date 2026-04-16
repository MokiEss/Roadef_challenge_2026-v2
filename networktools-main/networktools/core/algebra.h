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
 * @brief Implements a fixed-dimension math vector
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_ALGEBRA_H_
#define _NT_ALGEBRA_H_

#include <cmath>     // std::sqrt()
#include <cstring>   // std::memset()
#include <cassert>   // assert()

#include "sort.h"

namespace nt {

  /**
   * @class Vec
   * @brief Implements a fixed-dimension math vector
   *
   * This class provides a simple implementation of a math vector with a fixed number of
   * dimensions. Notice that no efforts have been made to optimize the math operations as modern
   * compilers optimize them well enough.
   *
   * @tparam V The type of value used in the vector. Must be an arithmetic type.
   * @tparam T The number of dimensions in the vector. Must be greater than 0.
   *
   */
  template < class V, int T >
  struct Vec {
    static_assert(T > 0, "Vec : Dimension must be at least 1");
    static_assert(std::is_arithmetic< V >::value, "Vec : Non-numerical value type");

    using value_type = V;
    using size_type = int;

    value_type _datas[T];

    Vec() = default;
    constexpr Vec(const value_type& rhs) noexcept {
      for (int k = 0; k < T; ++k)
        _datas[k] = rhs;
    }
    constexpr Vec(value_type (&&array)[T]) noexcept {
      for (int k = 0; k < T; ++k)
        _datas[k] = array[k];
    }
    // TODO: This must be removed, it may generate unwanted copies
    constexpr Vec(const TrivialDynamicArray< V >& array) noexcept { copyFrom(array); }

    template < class ARRAY >
    constexpr void copyFrom(const ARRAY& array) noexcept {
      zeros();
      for (int k = 0; k < T && k < array.size(); ++k)
        _datas[k] = array[k];
    }

    constexpr value_type& operator[](int index) noexcept {
      assert(index < T && index >= 0);
      return _datas[index];
    }

    constexpr value_type operator[](int index) const noexcept {
      assert(index < T && index >= 0);
      return _datas[index];
    }

    static constexpr int size() noexcept { return T; }

    // Explicit conversion operator to value_type (returns first element)
    // Use explicit to avoid ambiguity in comparisons and assignments
    explicit constexpr operator value_type() const noexcept { return _datas[0]; }

    constexpr void zeros() noexcept { std::memset((void*)_datas, 0, sizeof(value_type) * T); }

    constexpr value_type max() const noexcept {
      value_type max = 0;
      for (int k = 0; k < T; ++k)
        if (max < _datas[k]) max = _datas[k];
      return max;
    }

    constexpr value_type min() const noexcept {
      value_type min = _datas[0];
      for (int k = 1; k < T; ++k)
        if (min > _datas[k]) min = _datas[k];
      return min;
    }

    constexpr value_type magnitudeSquared() const noexcept {
      value_type r = 0;
      for (int k = 0; k < T; ++k)
        r += _datas[k] * _datas[k];
      return r;
    }

    constexpr value_type magnitude() const noexcept { return std::sqrt(magnitudeSquared()); }

    constexpr void normalize() noexcept {
      const value_type l = magnitude();
      for (int k = 0; k < T; ++k)
        _datas[k] /= l;
    }

    constexpr friend bool operator==(const Vec& lhs, const Vec& rhs) noexcept {
      for (int k = 0; k < T; ++k)
        if (lhs[k] != rhs[k]) return false;
      return true;
    }
    constexpr friend bool operator!=(const Vec& lhs, const Vec& rhs) noexcept { return !(lhs == rhs); }

    constexpr friend bool operator<(const Vec& lhs, const Vec& rhs) noexcept {
      for (int k = 0; k < T; ++k)
        if (lhs[k] >= rhs[k]) return false;
      return true;
    }
    constexpr friend bool operator>(const Vec& lhs, const Vec& rhs) noexcept { return rhs < lhs; }
    constexpr friend bool operator<=(const Vec& lhs, const Vec& rhs) noexcept { return !(lhs > rhs); }
    constexpr friend bool operator>=(const Vec& lhs, const Vec& rhs) noexcept { return !(lhs < rhs); }

    constexpr Vec operator-() const noexcept {
      Vec r;
      for (int k = 0; k < T; ++k)
        r[k] = -_datas[k];
      return r;
    }

    constexpr Vec& operator-=(const Vec& rhs) noexcept {
      for (int k = 0; k < T; ++k)
        _datas[k] -= rhs[k];
      return *this;
    }

    constexpr Vec& operator/=(const Vec& rhs) noexcept {
      for (int k = 0; k < T; ++k)
        _datas[k] /= rhs[k];
      return *this;
    }

    constexpr Vec& operator+=(const Vec& rhs) noexcept {
      for (int k = 0; k < T; ++k)
        _datas[k] += rhs[k];
      return *this;
    }

    constexpr Vec& operator*=(const Vec& rhs) noexcept {
      for (int k = 0; k < T; ++k)
        _datas[k] *= rhs[k];
      return *this;
    }

    template < class U >
    constexpr Vec& operator-=(const U rhs) noexcept {
      static_assert(std::is_arithmetic< U >::value, "Vec : Non-numerical value type");
      for (int k = 0; k < T; ++k)
        _datas[k] -= rhs;
      return *this;
    }

    template < class U >
    constexpr Vec& operator/=(const U rhs) noexcept {
      static_assert(std::is_arithmetic< U >::value, "Vec : Non-numerical value type");
      for (int k = 0; k < T; ++k)
        _datas[k] /= rhs;
      return *this;
    }

    template < class U >
    constexpr Vec& operator+=(const U rhs) noexcept {
      static_assert(std::is_arithmetic< U >::value, "Vec : Non-numerical value type");
      for (int k = 0; k < T; ++k)
        _datas[k] += rhs;
      return *this;
    }

    template < class U >
    constexpr Vec& operator*=(const U rhs) noexcept {
      static_assert(std::is_arithmetic< U >::value, "Vec : Non-numerical value type");
      for (int k = 0; k < T; ++k)
        _datas[k] *= rhs;
      return *this;
    }

    template < class U >
    constexpr friend Vec operator-(Vec lhs, const U& rhs) noexcept {
      lhs -= rhs;
      return lhs;
    }

    template < class U >
    constexpr friend Vec operator/(Vec lhs, const U& rhs) noexcept {
      lhs /= rhs;
      return lhs;
    }

    template < class U >
    constexpr friend Vec operator+(Vec lhs, const U& rhs) noexcept {
      lhs += rhs;
      return lhs;
    }

    template < class U >
    constexpr friend Vec operator*(Vec lhs, const U& rhs) noexcept {
      lhs *= rhs;
      return lhs;
    }
  };

  using Vec2Df = Vec< float, 2 >;
  using Vec2Dd = Vec< double, 2 >;
  using Vec2Di = Vec< int, 2 >;

  /**
   * @brief Compute a normal vector of v
   *
   */
  template < class V >
  constexpr Vec< V, 2 > normal2D(const Vec< V, 2 >& v) noexcept {
    return Vec< V, 2 >({-v[1], v[0]});
  }

  namespace details {
    template < typename T, bool IsScalar >
    struct ValueTypeHelper {
      using type = typename T::value_type;
    };

    template < typename T >
    struct ValueTypeHelper< T, true > {
      using type = T;
    };
  }   // namespace details

  /**
   * @brief Traits class for vector-like types
   *
   * This traits class provides a uniform interface for both vector types (like Vec<T, N>)
   * and scalar types (like double). It enables algorithms to work with either type
   * through the same interface.
   *
   * @tparam VEC The vector or scalar type
   *
   * Usage example:
   * @code
   * using VecTraits = VectorTraits<VEC>;
   * typename VecTraits::value_type val;
   * int size = VecTraits::size();
   * val = VecTraits::getElement(vec, 0);
   * @endcode
   */
  template < typename VEC >
  struct VectorTraits {
    // Check if VEC is an arithmetic type (scalar) or a vector type
    static constexpr bool is_scalar = std::is_arithmetic< VEC >::value;

    /**
     * @brief The element type of the vector (or the scalar type itself)
     *
     * For Vec<double, 1>, this is double.
     * For double, this is double.
     */
    using value_type = typename details::ValueTypeHelper< VEC, is_scalar >::type;

    /**
     * @brief Get the size of the vector (1 for scalars, VEC::size() for vectors)
     *
     * @return The number of elements in the vector
     */
    static constexpr int size() noexcept {
      if constexpr (is_scalar) {
        return 1;
      } else {
        return VEC::size();
      }
    }

    /**
     * @brief Access element at index t
     *
     * For scalars, always returns the scalar value (ignores t).
     * For vectors, returns v[t].
     *
     * @param v The vector or scalar value
     * @param t The index (ignored for scalars)
     * @return The element at index t
     */
    static constexpr value_type getElement(const VEC& v, int t) noexcept {
      if constexpr (is_scalar) {
        (void)t;   // Suppress unused parameter warning
        return v;
      } else {
        return v[t];
      }
    }

    /**
     * @brief Set element at index t
     *
     * For scalars, sets the entire value (ignores t).
     * For vectors, sets v[t].
     *
     * @param v The vector or scalar value (reference)
     * @param t The index (ignored for scalars)
     * @param val The value to set
     */
    static constexpr void setElement(VEC& v, int t, value_type val) noexcept {
      if constexpr (is_scalar) {
        (void)t;   // Suppress unused parameter warning
        v = val;
      } else {
        v[t] = val;
      }
    }

    /**
     * @brief Get the maximum element in the vector
     *
     * @param v The vector or scalar value (reference)
     */
    static constexpr value_type max(const VEC& v) noexcept {
      if constexpr (is_scalar)
        return v;
      else
        return v.max();
    }

    /**
     * @brief Component-wise max operator
     *
     */
    constexpr static inline VEC maxVector(const VEC& a, const VEC& b) noexcept {
      if constexpr (is_scalar)
        return nt::max(a, b);
      else {
        VEC max_vector;
        for (int k = 0; k < size(); ++k)
          max_vector[k] = nt::max(a[k], b[k]);
        return max_vector;
      }
    }
  };

  /**
   * @brief Calculates the number of digits needed to write a non-negative integer in base 10.
   * Note(user): max(1, log(0) + 1) == max(1, -inf) == 1.
   * Initial version of this code from Google OR-tools (https://github.com/google/or-tools)
   *
   * @param n The non-negative integer.
   *
   * @return The number of digits needed to write the integer in base 10.
   */
  inline int numDigits(int n) {
#if defined(_MSC_VER)
    return static_cast< int >(nt::max(1.0L, std::log(1.0L * n) / log(10.0L) + 1.0));
#else
    return static_cast< int >(nt::max(1.0, std::log10(static_cast< double >(n)) + 1.0));
#endif
  }
}   // namespace nt

#endif
