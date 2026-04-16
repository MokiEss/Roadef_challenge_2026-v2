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
 * This file incorporates work from the LEMON graph library (counter.h).
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
 *   - Converted typedef declarations to C++11 using declarations
 *   - Added constexpr qualifiers for compile-time optimization
 *   - Added noexcept specifiers for better exception safety
 *   - Significant code refactoring and restructuring
 *   - Updated header guard macros
 */

/**
 * @ingroup timecount
 * @file
 * @brief Tools for counting steps and events
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_COUNTER_H_
#define _NT_COUNTER_H_

#include "string.h"

namespace nt {

  template < class P, class OutputStream >
  class _NoSubCounter;

  template < class P, class OutputStream >
  class _SubCounter {
    P&            _parent;
    nt::String    _title;
    OutputStream& _os;
    int           count;

    public:
    using SubCounter = _SubCounter< _SubCounter< P, OutputStream >, OutputStream >;
    using NoSubCounter = _NoSubCounter< _SubCounter< P, OutputStream >, OutputStream >;

    _SubCounter(P& parent, OutputStream& os) : _parent(parent), _title(), _os(os), count(0) {}
    _SubCounter(P& parent, const nt::String& title, OutputStream& os) :
        _parent(parent), _title(nt::clone(title)), _os(os), count(0) {}
    _SubCounter(P& parent, const char* title, OutputStream& os) : _parent(parent), _title(title), _os(os), count(0) {}
    ~_SubCounter() {
      _os << _title << count << '\n';
      _os.flush();
      _parent += count;
    }
    constexpr _SubCounter& operator++() noexcept {
      count++;
      return *this;
    }
    constexpr int          operator++(int) noexcept { return count++; }
    constexpr _SubCounter& operator--() noexcept {
      count--;
      return *this;
    }
    constexpr int          operator--(int) noexcept { return count--; }
    constexpr _SubCounter& operator+=(int c) noexcept {
      count += c;
      return *this;
    }
    constexpr _SubCounter& operator-=(int c) noexcept {
      count -= c;
      return *this;
    }
    constexpr operator int() noexcept { return count; }
  };

  template < class P, class OutputStream >
  class _NoSubCounter {
    P& _parent;

    public:
    using SubCounter = _NoSubCounter< _NoSubCounter< P, OutputStream >, OutputStream >;
    using NoSubCounter = _NoSubCounter< _NoSubCounter< P, OutputStream >, OutputStream >;

    _NoSubCounter(P& parent) : _parent(parent) {}
    _NoSubCounter(P& parent, const nt::String&, OutputStream&) : _parent(parent) {}
    _NoSubCounter(P& parent, const nt::String&) : _parent(parent) {}
    _NoSubCounter(P& parent, const char*, OutputStream&) : _parent(parent) {}
    _NoSubCounter(P& parent, const char*) : _parent(parent) {}
    ~_NoSubCounter() {}
    constexpr _NoSubCounter& operator++() noexcept {
      ++_parent;
      return *this;
    }
    constexpr int operator++(int) noexcept {
      _parent++;
      return 0;
    }
    constexpr _NoSubCounter& operator--() noexcept {
      --_parent;
      return *this;
    }
    constexpr int operator--(int) noexcept {
      _parent--;
      return 0;
    }
    constexpr _NoSubCounter& operator+=(int c) noexcept {
      _parent += c;
      return *this;
    }
    constexpr _NoSubCounter& operator-=(int c) noexcept {
      _parent -= c;
      return *this;
    }
    constexpr operator int() noexcept { return 0; }
  };

  /**
   * \addtogroup timecount
   * @{
   */

  /** A counter class */

  /**
   * This class makes it easier to count certain events (e.g. for debug
   * reasons).
   * You can increment or decrement the counter using \c operator++,
   * \c operator--, \c operator+= and \c operator-=. You can also
   * define subcounters for the different phases of the algorithm or
   * for different types of operations.
   * A report containing the given title and the value of the counter
   * is automatically printed on destruction.
   *
   * The following example shows the usage of counters and subcounters.
   * @code
   * // Bubble sort
   * std::vector<T> v;
   * ...
   * Counter op("Operations: ");
   * Counter::SubCounter as(op, "Assignments: ");
   * Counter::SubCounter co(op, "Comparisons: ");
   * for (int i = v.size()-1; i > 0; --i) {
   *  for (int j = 0; j < i; ++j) {
   *    if (v[j] > v[j+1]) {
   *      T tmp = v[j];
   *      v[j] = v[j+1];
   *      v[j+1] = tmp;
   *      as += 3;          // three assignments
   *    }
   *    ++co;               // one comparison
   *   }
   * }
   * @endcode
   *
   * This code prints out something like that:
   * @code
   * Comparisons: 45
   * Assignments: 57
   * Operations: 102
   * @endcode
   *
   * \sa NoCounter
   */
  template < class OutputStream >
  class Counter {
    nt::String    _title;
    OutputStream& _os;
    int           count;

    public:
    /** SubCounter class */

    /**
     * This class can be used to setup subcounters for a @ref Counter
     * to have finer reports. A subcounter provides exactly the same
     * operations as the main @ref Counter, but it also increments and
     * decrements the value of its parent.
     * Subcounters can also have subcounters.
     *
     * The parent counter must be given as the first parameter of the
     * constructor. Apart from that a title and an \c ostream object
     * can also be given just like for the main @ref Counter.
     *
     * A report containing the given title and the value of the
     * subcounter is automatically printed on destruction. If you
     * would like to turn off this report, use @ref NoSubCounter
     * instead.
     *
     * \sa NoSubCounter
     */
    using SubCounter = _SubCounter< Counter, OutputStream >;

    /** SubCounter class without printing report on destruction */

    /**
     * This class can be used to setup subcounters for a @ref Counter.
     * It is the same as @ref SubCounter but it does not print report
     * on destruction. (It modifies the value of its parent, so 'No'
     * only means 'do not print'.)
     *
     * Replacing @ref SubCounter "SubCounter"s with @ref NoSubCounter
     * "NoSubCounter"s makes it possible to turn off reporting
     * subcounter values without actually removing the definitions
     * and the increment or decrement operators.
     *
     * \sa SubCounter
     */
    using NoSubCounter = _NoSubCounter< Counter, OutputStream >;

    /** Constructor. */
    Counter(OutputStream& os) : _title(), _os(os), count(0) {}
    /** Constructor. */
    Counter(const nt::String& title, OutputStream& os) : _title(nt::clone(title)), _os(os), count(0) {}
    /** Constructor. */
    Counter(const char* title, OutputStream& os) : _title(title), _os(os), count(0) {}
    /**
     * Destructor. Prints the given title and the value of the counter.
     */
    ~Counter() { _os << _title << count << std::endl; }
    
    constexpr Counter& operator++() noexcept {
      count++;
      return *this;
    }
    
    constexpr int operator++(int) noexcept { return count++; }
    
    Counter& operator--() {
      count--;
      return *this;
    }
    
    constexpr int operator--(int) noexcept { return count--; }
    
    constexpr Counter& operator+=(int c) noexcept {
      count += c;
      return *this;
    }
    
    constexpr Counter& operator-=(int c) noexcept {
      count -= c;
      return *this;
    }
    /** Resets the counter to the given value. */

    /**
     * Resets the counter to the given value.
     * @note This function does not reset the values of
     * @ref SubCounter "SubCounter"s but it resets @ref NoSubCounter
     * "NoSubCounter"s along with the main counter.
     */
    constexpr void reset(int c = 0) noexcept { count = c; }
    /** Returns the value of the counter. */
    constexpr operator int() noexcept { return count; }
  };


  /** 'Do nothing' version of Counter. */

  /**
   * This class can be used in the same way as @ref Counter, but it
   * does not count at all and does not print report on destruction.
   *
   * Replacing a @ref Counter with a @ref NoCounter makes it possible
   * to turn off all counting and reporting (SubCounters should also
   * be replaced with NoSubCounters), so it does not affect the
   * efficiency of the program at all.
   *
   * \sa Counter
   */
  template < class OutputStream >
  class NoCounter {
    public:
    using SubCounter = _NoSubCounter< NoCounter, OutputStream >;
    using NoSubCounter = _NoSubCounter< NoCounter, OutputStream >;

    constexpr NoCounter() noexcept {}
    constexpr NoCounter(nt::String, OutputStream&) noexcept {}
    constexpr NoCounter(const char*, OutputStream&) noexcept {}
    constexpr NoCounter(const nt::String&) noexcept {}
    constexpr NoCounter(const char*) noexcept {}
    constexpr NoCounter& operator++() noexcept { return *this; }
    constexpr int        operator++(int) noexcept { return 0; }
    constexpr NoCounter& operator--() noexcept { return *this; }
    constexpr int        operator--(int) noexcept { return 0; }
    constexpr NoCounter& operator+=(int) noexcept { return *this; }
    constexpr NoCounter& operator-=(int) noexcept { return *this; }
    constexpr void       reset(int) noexcept {}
    constexpr void       reset() noexcept {}
    constexpr            operator int() noexcept { return 0; }
  };

  /** @} */
}   // namespace nt
#endif
