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
 * This file incorporates work from the LEMON graph library (tolerance.h).
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
 *   - Added constexpr qualifiers for compile-time optimization
 *   - Added noexcept specifiers for better exception safety
 *   - Significant code refactoring and restructuring
 *   - Updated header guard macros
 *   - Added equal()/isZero()/positiveOrZero()/negativeOrZero() methods
 *   - Added Kahan summation compensation algorithm (AccurateSum)
 */

/**
 * @ingroup misc
 * @file
 * @brief A basic tool to handle the anomalies of calculation with
 *        floating point numbers.
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_TOLERANCE_H_
#define _NT_TOLERANCE_H_

namespace nt {

  /**
   * \addtogroup misc
   * @{
   */

  /**
   * @brief A class to provide a basic way to
   * handle the comparison of numbers that are obtained
   * as a result of a probably inexact computation.
   *
   * @ref Tolerance is a class to provide a basic way to
   * handle the comparison of numbers that are obtained
   * as a result of a probably inexact computation.
   *
   * The general implementation is suitable only if the data type is exact,
   * like the integer types, otherwise a specialized version must be
   * implemented. These specialized classes like
   * Tolerance<double> may offer additional tuning parameters.
   *
   * \sa Tolerance<float>
   * \sa Tolerance<double>
   * \sa Tolerance<long double>
   */

  template < typename T >
  struct Tolerance {
    using Value = T;

    /**
     * \name Comparisons
     * The concept is that these bool functions return \c true only if
     * the related comparisons hold even if some numerical error appeared
     * during the computations.
     */

    /** @{ */

    /**
     * Returns \c true if \c a is \e surely strictly less than \c b
     */
    static constexpr bool less(Value a, Value b) noexcept { return a < b; }
    /** Returns \c true if \c a is \e surely different from \c b */
    static constexpr bool different(Value a, Value b) noexcept { return a != b; }
    /** Returns \c true if \c a is \e surely equal to \c b */
    constexpr bool equal(Value a, Value b) const noexcept { return !different(a, b); }
    /** Returns \c true if \c a is \e surely positive */
    static constexpr bool positive(Value a) noexcept { return static_cast< Value >(0) < a; }
    /** Returns \c true if \c a is \e surely positive or null */
    static constexpr bool positiveOrZero(Value a) noexcept { return static_cast< Value >(0) <= a; }
    /** Returns \c true if \c a is \e surely negative */
    static constexpr bool negative(Value a) noexcept { return a < static_cast< Value >(0); }
    /** Returns \c true if \c a is \e surely negative or null */
    static constexpr bool negativeOrZero(Value a) noexcept { return static_cast< Value >(0) >= a; }
    /** Returns \c true if \c a is \e surely non-zero */
    static constexpr bool nonZero(Value a) noexcept { return a != static_cast< Value >(0); }
    /** Returns \c true if \c a is \e surely zero */
    static constexpr bool isZero(Value a) noexcept { return a == static_cast< Value >(0); }
    /** @} */

    /** Returns the zero value. */
    static constexpr Value zero() noexcept { return static_cast< Value >(0); }

    //   static bool finite(Value a) {}
    //   static Value big() {}
    //   static Value negativeBig() {}
  };

  /** Float specialization of Tolerance. */

  /**
   * Float specialization of Tolerance.
   * \sa Tolerance
   * \relates Tolerance
   */
  template <>
  struct Tolerance< float > {
    constexpr static float def_epsilon = static_cast< float >(1e-4);
    float                  _epsilon;

    
    using Value = float;

    /**
     * Constructor setting the epsilon tolerance to the default value.
     */
    constexpr Tolerance() : _epsilon(def_epsilon) {}
    /**
     * Constructor setting the epsilon tolerance to the given value.
     */
    Tolerance(float e) : _epsilon(e) {}

    /** Returns the epsilon value. */
    constexpr Value epsilon() const noexcept { return _epsilon; }
    /** Sets the epsilon value. */
    constexpr void epsilon(Value e) noexcept { _epsilon = e; }

    /** Returns the default epsilon value. */
    static constexpr Value defaultEpsilon() noexcept { return def_epsilon; }
    /** Sets the default epsilon value. */
    // static void defaultEpsilon(Value e) { def_epsilon = e; }

    /**
     * \name Comparisons
     * See @ref lemon::Tolerance "Tolerance" for more details.
     */

    /** @{ */

    /**
     * Returns \c true if \c a is \e surely strictly less than \c b
     */
    constexpr bool less(Value a, Value b) const noexcept { return a + _epsilon < b; }
    /** Returns \c true if \c a is \e surely different from \c b */
    constexpr bool different(Value a, Value b) const noexcept { return less(a, b) || less(b, a); }
    /** Returns \c true if \c a is \e surely equal to \c b */
    constexpr bool equal(Value a, Value b) const noexcept { return !different(a, b); }
    /** Returns \c true if \c a is \e surely positive */
    constexpr bool positive(Value a) const noexcept { return _epsilon < a; }
    /** Returns \c true if \c a is \e surely positive or null */
    constexpr bool positiveOrZero(Value a) const noexcept { return -_epsilon < a; }
    /** Returns \c true if \c a is \e surely negative */
    constexpr bool negative(Value a) const noexcept { return -_epsilon > a; }
    /** Returns \c true if \c a is \e surely positive or null */
    constexpr bool negativeOrZero(Value a) const noexcept { return _epsilon > a; }
    /** Returns \c true if \c a is \e surely non-zero */
    constexpr bool nonZero(Value a) const noexcept { return positive(a) || negative(a); }
    /** Returns \c true if \c a is \e surely zero */
    constexpr bool isZero(Value a) const noexcept { return !nonZero(a); }
    /** @} */

    /** Returns zero */
    static constexpr Value zero() noexcept { return 0; }
  };

  /** Double specialization of Tolerance. */

  /**
   * Double specialization of Tolerance.
   * \sa Tolerance
   * \relates Tolerance
   */
  template <>
  struct Tolerance< double > {
    constexpr static double def_epsilon = 1e-10;
    double                  _epsilon;

    
    using Value = double;

    /**
     * Constructor setting the epsilon tolerance to the default value.
     */
    constexpr Tolerance() : _epsilon(def_epsilon) {}
    /**
     * Constructor setting the epsilon tolerance to the given value.
     */
    Tolerance(double e) : _epsilon(e) {}

    /** Returns the epsilon value. */
    constexpr Value epsilon() const noexcept { return _epsilon; }
    /** Sets the epsilon value. */
    constexpr void epsilon(Value e) noexcept { _epsilon = e; }

    /** Returns the default epsilon value. */
    static constexpr Value defaultEpsilon() noexcept { return def_epsilon; }
    /** Sets the default epsilon value. */
    // static void defaultEpsilon(Value e) { def_epsilon = e; }

    /**
     * \name Comparisons
     * See @ref lemon::Tolerance "Tolerance" for more details.
     */

    /** @{ */

    /**
     * Returns \c true if \c a is \e surely strictly less than \c b
     */
    constexpr bool less(Value a, Value b) const noexcept { return a + _epsilon < b; }
    /** Returns \c true if \c a is \e surely different from \c b */
    constexpr bool different(Value a, Value b) const noexcept { return less(a, b) || less(b, a); }
    /** Returns \c true if \c a is \e surely equal to \c b */
    constexpr bool equal(Value a, Value b) const noexcept { return !different(a, b); }
    /** Returns \c true if \c a is \e surely positive */
    constexpr bool positive(Value a) const noexcept { return _epsilon < a; }
    /** Returns \c true if \c a is \e surely positive or null */
    constexpr bool positiveOrZero(Value a) const noexcept { return -_epsilon < a; }
    /** Returns \c true if \c a is \e surely negative */
    constexpr bool negative(Value a) const noexcept { return -_epsilon > a; }
    /** Returns \c true if \c a is \e surely positive or null */
    constexpr bool negativeOrZero(Value a) const noexcept { return _epsilon > a; }
    /** Returns \c true if \c a is \e surely non-zero */
    constexpr bool nonZero(Value a) const noexcept { return positive(a) || negative(a); }
    /** Returns \c true if \c a is \e surely zero */
    constexpr bool isZero(Value a) const noexcept { return !nonZero(a); }
    /** @} */

    /** Returns zero */
    static constexpr Value zero() noexcept { return 0; }
  };

  /** Long double specialization of Tolerance. */

  /**
   * Long double specialization of Tolerance.
   * \sa Tolerance
   * \relates Tolerance
   */
  template <>
  struct Tolerance< long double > {
    constexpr static long double def_epsilon = 1e-14;
    long double                  _epsilon;

    public:
    
    using Value = long double;

    /**
     * Constructor setting the epsilon tolerance to the default value.
     */
    constexpr Tolerance() : _epsilon(def_epsilon) {}
    /**
     * Constructor setting the epsilon tolerance to the given value.
     */
    Tolerance(long double e) : _epsilon(e) {}

    /** Returns the epsilon value. */
    constexpr Value epsilon() const noexcept { return _epsilon; }
    /** Sets the epsilon value. */
    constexpr void epsilon(Value e) noexcept { _epsilon = e; }

    /** Returns the default epsilon value. */
    constexpr static Value defaultEpsilon() noexcept { return def_epsilon; }
    /** Sets the default epsilon value. */
    // static void defaultEpsilon(Value e) { def_epsilon = e; }

    /**
     * \name Comparisons
     * See @ref lemon::Tolerance "Tolerance" for more details.
     */

    /** @{ */

    /**
     * Returns \c true if \c a is \e surely strictly less than \c b
     */
    constexpr bool less(Value a, Value b) const noexcept { return a + _epsilon < b; }
    /** Returns \c true if \c a is \e surely different from \c b */
    constexpr bool different(Value a, Value b) const noexcept { return less(a, b) || less(b, a); }
    /** Returns \c true if \c a is \e surely equal to \c b */
    constexpr bool equal(Value a, Value b) const noexcept { return !different(a, b); }
    /** Returns \c true if \c a is \e surely positive */
    constexpr bool positive(Value a) const noexcept { return _epsilon < a; }
    /** Returns \c true if \c a is \e surely positive or null */
    constexpr bool positiveOrZero(Value a) const noexcept { return -_epsilon < a; }
    /** Returns \c true if \c a is \e surely negative */
    constexpr bool negative(Value a) const noexcept { return -_epsilon > a; }
    /** Returns \c true if \c a is \e surely positive or null */
    constexpr bool negativeOrZero(Value a) const noexcept { return _epsilon > a; }
    /** Returns \c true if \c a is \e surely non-zero */
    constexpr bool nonZero(Value a) const noexcept { return positive(a) || negative(a); }
    /** Returns \c true if \c a is \e surely zero */
    constexpr bool isZero(Value a) const noexcept { return !nonZero(a); }

    /** @} */

    /** Returns zero */
    static constexpr Value zero() noexcept { return 0; }
  };

  /** @} */


  /**
   * @brief Kahan summation compensation algorithm.
   *
   * This algorithm reduces the error in floating-point summation.
   * For more details, see: http://en.wikipedia.org/wiki/Kahan_summation_algorithm.
   *
   * Initial version of this code taken from Google OR-tools (https://github.com/google/or-tools)
   *
   * @tparam FpNumber The floating-point number type.
   */
  template < typename FpNumber >
  struct AccurateSum {
    FpNumber _sum;         //< The sum of values.
    FpNumber _error_sum;   //< The error in the sum.

    /**
     * @brief Constructs an AccurateSum object.
     *
     * You may copy-construct an AccurateSum.
     *
     */
    AccurateSum() : _sum(), _error_sum() {}

    /**
     * @brief Adds a value to the sum.
     *
     * @param value The value to be added to the sum.
     */
    void add(const FpNumber& value) noexcept {
      _error_sum += value;
      const FpNumber new_sum = _sum + _error_sum;
      _error_sum += _sum - new_sum;
      _sum = new_sum;
    }

    /**
     * @brief Gets the value of the sum.
     *
     * @return The value of the sum.
     */
    FpNumber value() const noexcept { return _sum; }
  };

}   // namespace nt

#endif
