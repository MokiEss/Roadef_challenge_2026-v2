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
 * @brief Basic macros and functions used throughout the library.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_COMMON_H_
#define _NT_COMMON_H_

#include "../ntconfig.h"

#include <bit>      // endianess
#include <limits>   // std::numeric_limits
#include "tuple.h"
#include "assert.h"

#if TRACY_ENABLE
#  include <tracy/Tracy.hpp>
#else
#  define ZoneScoped
#  define ZoneScopedN(x)
#  define ZoneScopedC(x)
#  define ZoneScopedNC(x, y)

#  define TracyAlloc(x, y)
#  define TracyFree(x)
#  define TracySecureAlloc(x, y)
#  define TracySecureFree(x)

#  define TracyAllocN(x, y, z)
#  define TracyFreeN(x, y)
#  define TracySecureAllocN(x, y, z)
#  define TracySecureFreeN(x, y)
#endif

#if defined(NT_USE_SIMD)
#  if (_MSC_VER >= 1800)
#    include <intrin.h>
#  else
#    include <x86intrin.h>
#  endif
#endif

// Macro to warn about unused results.
#if __cplusplus >= 201703L
#  define NT_NODISCARD [[__nodiscard__]]
#else
#  define NT_NODISCARD
#endif

// Use this macro when declaring a constexpr method that uses SIMD instructions because it may not be compatible with
// constexpr.
#if defined(NT_USE_SIMD)
#  define NT_SIMD_CONSTEXPR
#else
#  define NT_SIMD_CONSTEXPR constexpr
#endif

// Compile-time values
#define NT_BOOL_MAX ((std::numeric_limits< bool >::max)())                 // Maximum value of a bool
#define NT_INT8_MAX ((std::numeric_limits< std::int8_t >::max)())          // Maximum value of an 8-bits integer : 127
#define NT_UINT8_MAX ((std::numeric_limits< std::uint8_t >::max)())        // Maximum value of an unsigned 8-bits integer : 255
#define NT_INT16_MAX ((std::numeric_limits< std::int16_t >::max)())        // Maximum value of an 16-bits integer : 32,767
#define NT_UINT16_MAX ((std::numeric_limits< std::uint16_t >::max)())      // Maximum value of an unsigned 16-bits integer : 65,535
#define NT_INT32_MAX ((std::numeric_limits< std::int32_t >::max)())        // Maximum value of an 32-bits integer  : 2,147,483,647
#define NT_UINT32_MAX ((std::numeric_limits< std::uint32_t >::max)())      // Maximum value of an unsigned 32-bits integer : 4,294,967,295
#define NT_INT64_MAX ((std::numeric_limits< std::int64_t >::max)())        // Maximum value of an 64-bits integer : 9,223,372,036,854,775,807
#define NT_UINT64_MAX ((std::numeric_limits< std::uint64_t >::max)())      // Maximum value of an unsigned 64-bits integer : 18,446,744,073,709,551,615
#define NT_FLOAT_MAX ((std::numeric_limits< float >::max)())               // Maximum value of a float : 3.40282e+38
#define NT_FLOAT_INFINITY ((std::numeric_limits< float >::infinity)())     // Infinity representation for float
#define NT_FLOAT_NAN ((std::numeric_limits< float >::quiet_NaN)())         // NaN representation for float
#define NT_DOUBLE_MAX ((std::numeric_limits< double >::max)())             // Maximum value of a double : 1.79769e+308
#define NT_DOUBLE_INFINITY ((std::numeric_limits< double >::infinity)())   // Infinity representation for double
#define NT_DOUBLE_NAN ((std::numeric_limits< double >::quiet_NaN)())       // NaN representation for double
#define NT_IS_BIG_ENDIAN (std::endian::native == std::endian::big)         // Is equal to true if scalars are big endian

// Quick bit manipulation macros
#define NT_SIZEOF_BITS(X) (sizeof(X) * 8)                                                            // Give the number of bits used to represent the given number
#define NT_IS_POW_2(X) (X == 0 ? 0 : !(X & (X - 0x1u)))                                              // True if X is a power of 2
#define NT_TWO(c) (0x1u << (c))                                                                      // Returns the value 2^c
#define NT_TWO_64(c) (static_cast< uint64_t >(1) << (c))                                             // Returns the value 2^c as an uint64_t
#define NT_CONCAT_INT_32(X, Y) (static_cast< uint64_t >((X)) << 32) | static_cast< uint64_t >((Y))   // Concatenate the 32 bits integer X and Y into a single 64 bits integer Z=X.Y
#define NT_CONCAT_INT_16(X, Y, Z, W) (static_cast< uint64_t >((X)) << 48) | static_cast< uint64_t >((Y)) << 32) | static_cast< uint64_t >((Z)) << 16) | static_cast< uint64_t >((W))   // Concatenate the 16 bits integer X, Y, Z, W into a single 64 bits integer Z=X.Y.Z.W
#define NT_DIVIDE_BY_64(x) ((x) >> 6)                   // Divide an integer by 64
#define NT_MODULO_64(x) ((x)&63)                        // Returns x modulo 64
#define NT_MODULO_32(x) ((x)&31)                        // Returns x modulo 32
#define NT_IS_ONE_AT(x, p) ((x) & (0x1u << (p)))        // True if the p-th bit of x is one
#define NT_IS_EVEN(x) (static_cast< bool >((x)&0x1u))   // True if X is even
#define NT_IS_ODD(x) (!NT_IS_EVEN(x))                   // True if X is odd

// Clamp value between min and max
#define NT_CLAMP(value, min, max) \
  if (min > value)                \
    value = min;                  \
  else if (max < value)           \
    value = max;

namespace nt {
  /**
   * @class True
   * @brief Basic type for defining "tags". A "YES" condition for enable_if.
   *        It is also a functor that always returns true for any arguments.
   *
   */
  struct True {
    static constexpr bool value = true;
    constexpr             operator bool() const noexcept { return value; }
    template < typename... Args >
    constexpr bool operator()(Args&&...) const noexcept { return value; }
  };

  /**
   * @class False
   * @brief Basic type for defining "tags". A "NO" condition for enable_if.
   *        It is also a functor that always returns false for any arguments.
   *
   */
  struct False {
    static constexpr bool value = false;
    constexpr             operator bool() const noexcept { return value; }
    template < typename... Args >
    constexpr bool operator()(Args&&...) const noexcept { return value; }
  };

  template < typename Type, typename T = void >
  struct exists {
    typedef T type;
  };

  // using True = std::true_type;
  // using False = std::false_type;

  /**
   * @brief Dummy type to make it easier to create invalid iterators.
   *
   * Dummy type to make it easier to create invalid iterators.
   * See @ref INVALID for the usage.
   */
  struct Invalid {
    constexpr bool operator==(Invalid) const noexcept { return true; }
    constexpr bool operator!=(Invalid) const noexcept { return false; }
    constexpr bool operator<(Invalid) const noexcept { return false; }
  };

  /**
   * @brief Invalid iterators.
   *
   * @ref Invalid is a global type that converts to each iterator
   * in such a way that the value of the target iterator will be invalid.
   */
  constexpr Invalid INVALID = Invalid();

  /** integral_constant */
  template < typename _Tp, _Tp __v >
  struct integral_constant {
    static constexpr _Tp                  value = __v;
    typedef _Tp                           value_type;
    typedef integral_constant< _Tp, __v > type;
    constexpr                             operator value_type() const noexcept { return value; }
    constexpr value_type                  operator()() const noexcept { return value; }
  };

#if !__cpp_inline_variables
  template < typename _Tp, _Tp __v >
  constexpr _Tp integral_constant< _Tp, __v >::value;
#endif

  /** The type used as a compile-time boolean with true value. */
  using true_type = integral_constant< bool, true >;

  /** The type used as a compile-time boolean with false value. */
  using false_type = integral_constant< bool, false >;

  /** is_lvalue_reference */
  template < typename >
  struct is_lvalue_reference: public false_type {};

  template < typename _Tp >
  struct is_lvalue_reference< _Tp& >: public true_type {};

  /** remove_reference */
  template < typename _Tp >
  struct remove_reference {
    typedef _Tp type;
  };

  template < typename _Tp >
  struct remove_reference< _Tp& > {
    typedef _Tp type;
  };

  template < typename _Tp >
  struct remove_reference< _Tp&& > {
    typedef _Tp type;
  };

  /**
   *  @addtogroup utilities
   *  @{
   */

  /**
   *  @brief  Forward an lvalue.
   *  @return The parameter cast to the specified type.
   *
   *  This function is used to implement "perfect forwarding".
   */
  template < typename _Tp >
  NT_NODISCARD constexpr _Tp&& forward(typename nt::remove_reference< _Tp >::type& __t) noexcept {
    return static_cast< _Tp&& >(__t);
  }

  /**
   *  @brief  Forward an rvalue.
   *  @return The parameter cast to the specified type.
   *
   *  This function is used to implement "perfect forwarding".
   */
  template < typename _Tp >
  NT_NODISCARD constexpr _Tp&& forward(typename nt::remove_reference< _Tp >::type&& __t) noexcept {
    static_assert(!nt::is_lvalue_reference< _Tp >::value,
                  "nt::forward must not be used to convert an rvalue to an lvalue");
    return static_cast< _Tp&& >(__t);
  }

  /**
   *  @brief  Convert a value to an rvalue.
   *  @param  __t  A thing of arbitrary type.
   *  @return The parameter cast to an rvalue-reference to allow moving it.
   */
  template < typename _Tp >
  NT_NODISCARD constexpr typename nt::remove_reference< _Tp >::type&& move(_Tp&& __t) noexcept {
    return static_cast< typename nt::remove_reference< _Tp >::type&& >(__t);
  }

  /**
   * @brief Swaps the values of two objects.
   *
   * @tparam T The type of the objects to be swapped.
   * @param a The first object.
   * @param b The second object.
   */
  template < typename T >
  inline void swap(T& a, T& b) noexcept {
    T t(nt::move(a));
    a = nt::move(b);
    b = nt::move(t);
  }

  /**
   * @brief Swaps the values of two Triple objects.
   *
   * @tparam A The type of the first element in the Triple.
   * @tparam B The type of the second element in the Triple.
   * @tparam C The type of the third element in the Triple.
   * @param a The first Triple object.
   * @param b The second Triple object.
   */
  template < typename A, typename B, typename C >
  inline void swap(Triple< A, B, C >& a, Triple< A, B, C >& b) noexcept {
    Triple< A, B, C > temp = nt::move(a);
    a = nt::move(b);
    b = nt::move(temp);
  }

  /**
   * @brief Swaps the values of two Pair objects.
   *
   * @tparam A The type of the first element in the Pair.
   * @tparam B The type of the second element in the Pair.
   * @param a The first Pair object.
   * @param b The second Pair object.
   */
  template < typename A, typename B >
  inline void swap(Pair< A, B >& a, Pair< A, B >& b) noexcept {
    Pair< A, B > temp = nt::move(a);
    a = nt::move(b);
    b = nt::move(temp);
  }

  namespace details {
    template < class, class >
    struct is_same {
      static constexpr bool value = false;
    };

    template < class T >
    struct is_same< T, T > {
      static constexpr bool value = true;
    };

    template < class T, class U >
    inline constexpr bool is_same_v = is_same< T, U >::value;
  }   // namespace details

  template < class T, class U >
  concept same_as = details::is_same_v< T, U > && details::is_same_v< U, T >;

  /** @cond undocumented */
  template < typename _Tp >
  struct __declval_protector {
    static const bool __stop = false;
  };
  /** @endcond */

  /** Utility to simplify expressions used in unevaluated operands
   *  @since C++11
   *  @ingroup utilities
   */
  template < typename _Tp >
  auto declval() noexcept -> decltype(__declval< _Tp >(0)) {
    static_assert(__declval_protector< _Tp >::__stop, "declval() must not be used!");
    return __declval< _Tp >(0);
  }

  /*
   * "inline" is used for ignore_unused_variable_warning()
   * and function_requires() to make sure there is no
   * overtarget with g++.
   */

  template < class T >
  inline void ignore_unused_variable_warning(const T&) {}
  template < class T1, class T2 >
  inline void ignore_unused_variable_warning(const T1&, const T2&) {}
  template < class T1, class T2, class T3 >
  inline void ignore_unused_variable_warning(const T1&, const T2&, const T3&) {}
  template < class T1, class T2, class T3, class T4 >
  inline void ignore_unused_variable_warning(const T1&, const T2&, const T3&, const T4&) {}
  template < class T1, class T2, class T3, class T4, class T5 >
  inline void ignore_unused_variable_warning(const T1&, const T2&, const T3&, const T4&, const T5&) {}
  template < class T1, class T2, class T3, class T4, class T5, class T6 >
  inline void ignore_unused_variable_warning(const T1&, const T2&, const T3&, const T4&, const T5&, const T6&) {}


  template < class Concept >
  inline void function_requires() {
#if !defined(NDEBUG)
    void (Concept::*x)() = &Concept::constraints;
    ::nt::ignore_unused_variable_warning(x);
#endif
  }


  template < typename Concept, typename Type >
  inline void checkConcept() {
#if !defined(NDEBUG)
    using ConceptCheck = typename Concept::template Constraints< Type >;
    void (ConceptCheck::*x)() = &ConceptCheck::constraints;
    ::nt::ignore_unused_variable_warning(x);
#endif
  }

  namespace details {

    /**
     * @brief  Dummy class to avoid ambiguity
     *
     * @tparam T
     */
    template < int T >
    struct Dummy {
      Dummy(int) {}
    };
  }   // namespace details

  /**
   * @brief Create a value-preserving copy of @p src.
   *
   * If you want a move instead, do not call `clone`; move directly at the call site.
   *
   * @tparam T  The type of the object to clone.
   * @param src The source object to clone.
   * @return A new object that is a value-preserving copy of @p src.
   *
   * @code
   * // Move-only string that supports explicit copy via copyFrom:
   * nt::String s1 = ...;                // move-only (no copy ctor/assign)
   * nt::String s2 = nt::clone(s1);      // explicit copy via s2.copyFrom(s1)
   *
   * // Works with const rvalues too (still copies, by design):
   * nt::String s3 = nt::clone(static_cast<const nt::String&>(s1));
   * @endcode
   */
  template < typename T >
  T clone(const T& src) noexcept {
    if constexpr (std::is_arithmetic_v< T > || std::is_same_v< T, bool >) {
      return src;
    } else {
      T dst{};
      dst.copyFrom(src);
      return dst;
    }
  }
}   // namespace nt

#endif