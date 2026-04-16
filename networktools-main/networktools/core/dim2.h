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
 * This file incorporates work from the LEMON graph library (dim2.h).
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
 *   - Updated header guard macros
 *   - Converted typedef declarations to C++11 using declarations
 *   - Added noexcept specifiers for better exception safety
 *   - Added constexpr qualifiers for compile-time optimization
 */

/**
 * @ingroup geomdat
 * @file
 * @brief A simple two dimensional vector and a bounding box implementation
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_DIM2_H_
#define _NT_DIM2_H_

#include <algorithm>
#include <iostream>

namespace nt {

  /** Tools for handling two dimensional coordinates */

  /**
   * This namespace is a storage of several
   * tools for handling two dimensional coordinates
   */
  namespace dim2 {

    /**
     * \addtogroup geomdat
     * @{
     */

    /** Two dimensional vector (plain vector) */

    /**
     * A simple two dimensional vector (plain vector) implementation
     * with the usual vector operations.
     */
    template < typename T >
    struct Point {
      using Value = T;

      /** First coordinate */
      T x;
      /** Second coordinate */
      T y;

      /** Default constructor */
      Point() = default;

      /** Construct an instance from coordinates */
      constexpr Point(T a, T b) noexcept : x(a), y(b) {}

      /** Returns the dimension of the vector (i.e. returns 2). */

      /**
       * The dimension of the vector.
       * This function always returns 2.
       */
      constexpr int size() const noexcept { return 2; }

      /** Subscripting operator */

      /**
       * \c p[0] is \c p.x and \c p[1] is \c p.y
       *
       */
      constexpr T& operator[](int idx) noexcept { return idx == 0 ? x : y; }

      /** Const subscripting operator */

      /**
       * \c p[0] is \c p.x and \c p[1] is \c p.y
       *
       */
      constexpr const T& operator[](int idx) const noexcept { return idx == 0 ? x : y; }

      /** Conversion constructor */
      template < class TT >
      constexpr Point(const Point< TT >& p) noexcept : x(p.x), y(p.y) {}
      /** Give back the square of the norm of the vector */
      constexpr T normSquare() const noexcept { return x * x + y * y; }

      /** Increment the left hand side by \c u */
      constexpr Point< T >& operator+=(const Point< T >& u) noexcept {
        x += u.x;
        y += u.y;
        return *this;
      }

      /** Decrement the left hand side by \c u */
      constexpr Point< T >& operator-=(const Point< T >& u) noexcept {
        x -= u.x;
        y -= u.y;
        return *this;
      }

      /** Multiply the left hand side with a scalar */
      constexpr Point< T >& operator*=(const T& u) noexcept {
        x *= u;
        y *= u;
        return *this;
      }

      /** Divide the left hand side by a scalar */
      constexpr Point< T >& operator/=(const T& u) noexcept {
        x /= u;
        y /= u;
        return *this;
      }

      /** Return the scalar product of two vectors */
      constexpr T operator*(const Point< T >& u) const noexcept { return x * u.x + y * u.y; }

      /** Return the sum of two vectors */
      constexpr Point< T > operator+(const Point< T >& u) const noexcept {
        Point< T > b = *this;
        return b += u;
      }

      /** Return the negative of the vector */
      constexpr Point< T > operator-() const noexcept {
        Point< T > b = *this;
        b.x = -b.x;
        b.y = -b.y;
        return b;
      }

      /** Return the difference of two vectors */
      constexpr Point< T > operator-(const Point< T >& u) const noexcept {
        Point< T > b = *this;
        return b -= u;
      }

      /** Return a vector multiplied by a scalar */
      constexpr Point< T > operator*(const T& u) const noexcept {
        Point< T > b = *this;
        return b *= u;
      }

      /** Return a vector divided by a scalar */
      constexpr Point< T > operator/(const T& u) const noexcept {
        Point< T > b = *this;
        return b /= u;
      }

      /** Test equality */
      constexpr bool operator==(const Point< T >& u) const noexcept { return (x == u.x) && (y == u.y); }

      /** Test inequality */
      constexpr bool operator!=(const Point< T >& u) const noexcept { return (x != u.x) || (y != u.y); }
    };

    /** Return a Point */

    /**
     * Return a Point.
     * \relates Point
     */
    template < typename T >
    inline Point< T > makePoint(const T& x, const T& y) {
      return Point< T >(x, y);
    }

    /** Return a vector multiplied by a scalar */

    /**
     * Return a vector multiplied by a scalar.
     * \relates Point
     */
    template < typename T >
    constexpr Point< T > operator*(const T& u, const Point< T >& x) noexcept {
      return x * u;
    }

    /** Read a plain vector from a stream */

    /**
     * Read a plain vector from a stream.
     * \relates Point
     *
     */
    template < typename T >
    inline std::istream& operator>>(std::istream& is, Point< T >& z) {
      char c;
      if (is >> c) {
        if (c != '(') is.putback(c);
      } else {
        is.clear();
      }
      if (!(is >> z.x)) return is;
      if (is >> c) {
        if (c != ',') is.putback(c);
      } else {
        is.clear();
      }
      if (!(is >> z.y)) return is;
      if (is >> c) {
        if (c != ')') is.putback(c);
      } else {
        is.clear();
      }
      return is;
    }

    /** Write a plain vector to a stream */

    /**
     * Write a plain vector to a stream.
     * \relates Point
     *
     */
    template < typename T >
    inline std::ostream& operator<<(std::ostream& os, const Point< T >& z) {
      os << "(" << z.x << "," << z.y << ")";
      return os;
    }

    /** Rotate by 90 degrees */

    /**
     * Returns the parameter rotated by 90 degrees in positive direction.
     * \relates Point
     *
     */
    template < typename T >
    inline Point< T > rot90(const Point< T >& z) {
      return Point< T >(-z.y, z.x);
    }

    /** Rotate by 180 degrees */

    /**
     * Returns the parameter rotated by 180 degrees.
     * \relates Point
     *
     */
    template < typename T >
    inline Point< T > rot180(const Point< T >& z) {
      return Point< T >(-z.x, -z.y);
    }

    /** Rotate by 270 degrees */

    /**
     * Returns the parameter rotated by 90 degrees in negative direction.
     * \relates Point
     *
     */
    template < typename T >
    inline Point< T > rot270(const Point< T >& z) {
      return Point< T >(z.y, -z.x);
    }

    /** Bounding box of plain vectors (points). */

    /**
     * A class to calculate or store the bounding box of plain vectors
     * (@ref Point "points").
     */
    template < typename T >
    class Box {
      Point< T > _bottom_left, _top_right;
      bool       _empty;

      public:
      /** Default constructor: creates an empty box */
      constexpr Box() noexcept { _empty = true; }

      /** Construct a box from one point */
      constexpr Box(Point< T > a) noexcept {
        _bottom_left = _top_right = a;
        _empty = false;
      }

      /** Construct a box from two points */

      /**
       * Construct a box from two points.
       * @param a The bottom left corner.
       * @param b The top right corner.
       * @warning The coordinates of the bottom left corner must be no more
       * than those of the top right one.
       */
      constexpr Box(Point< T > a, Point< T > b) noexcept {
        _bottom_left = a;
        _top_right = b;
        _empty = false;
      }

      /** Construct a box from four numbers */

      /**
       * Construct a box from four numbers.
       * @param l The left side of the box.
       * @param b The bottom of the box.
       * @param r The right side of the box.
       * @param t The top of the box.
       * @warning The left side must be no more than the right side and
       * bottom must be no more than the top.
       */
      constexpr Box(T l, T b, T r, T t) noexcept {
        _bottom_left = Point< T >(l, b);
        _top_right = Point< T >(r, t);
        _empty = false;
      }

      /** Return \c true if the box is empty. */

      /**
       * Return \c true if the box is empty (i.e. return \c false
       * if at least one point was added to the box or the coordinates of
       * the box were set).
       *
       * The coordinates of an empty box are not defined.
       */
      constexpr bool empty() const noexcept { return _empty; }

      /** Make the box empty */
      constexpr void clear() noexcept { _empty = true; }

      /** Give back the bottom left corner of the box */

      /**
       * Give back the bottom left corner of the box.
       * If the box is empty, then the return value is not defined.
       */
      constexpr Point< T > bottomLeft() const noexcept { return _bottom_left; }

      /** Set the bottom left corner of the box */

      /**
       * Set the bottom left corner of the box.
       * @pre The box must not be empty.
       */
      constexpr void bottomLeft(Point< T > p) noexcept { _bottom_left = p; }

      /** Give back the top right corner of the box */

      /**
       * Give back the top right corner of the box.
       * If the box is empty, then the return value is not defined.
       */
      constexpr Point< T > topRight() const noexcept { return _top_right; }

      /** Set the top right corner of the box */

      /**
       * Set the top right corner of the box.
       * @pre The box must not be empty.
       */
      constexpr void topRight(Point< T > p) noexcept { _top_right = p; }

      /** Give back the bottom right corner of the box */

      /**
       * Give back the bottom right corner of the box.
       * If the box is empty, then the return value is not defined.
       */
      constexpr Point< T > bottomRight() const noexcept { return Point< T >(_top_right.x, _bottom_left.y); }

      /** Set the bottom right corner of the box */

      /**
       * Set the bottom right corner of the box.
       * @pre The box must not be empty.
       */
      constexpr void bottomRight(Point< T > p) noexcept {
        _top_right.x = p.x;
        _bottom_left.y = p.y;
      }

      /** Give back the top left corner of the box */

      /**
       * Give back the top left corner of the box.
       * If the box is empty, then the return value is not defined.
       */
      constexpr Point< T > topLeft() const noexcept { return Point< T >(_bottom_left.x, _top_right.y); }

      /** Set the top left corner of the box */

      /**
       * Set the top left corner of the box.
       * @pre The box must not be empty.
       */
      constexpr void topLeft(Point< T > p) noexcept {
        _top_right.y = p.y;
        _bottom_left.x = p.x;
      }

      /** Give back the bottom of the box */

      /**
       * Give back the bottom of the box.
       * If the box is empty, then the return value is not defined.
       */
      constexpr T bottom() const noexcept { return _bottom_left.y; }

      /** Set the bottom of the box */

      /**
       * Set the bottom of the box.
       * @pre The box must not be empty.
       */
      constexpr void bottom(T t) noexcept { _bottom_left.y = t; }

      /** Give back the top of the box */

      /**
       * Give back the top of the box.
       * If the box is empty, then the return value is not defined.
       */
      constexpr T top() const noexcept { return _top_right.y; }

      /** Set the top of the box */

      /**
       * Set the top of the box.
       * @pre The box must not be empty.
       */
      constexpr void top(T t) noexcept { _top_right.y = t; }

      /** Give back the left side of the box */

      /**
       * Give back the left side of the box.
       * If the box is empty, then the return value is not defined.
       */
      constexpr T left() const noexcept { return _bottom_left.x; }

      /** Set the left side of the box */

      /**
       * Set the left side of the box.
       * @pre The box must not be empty.
       */
      constexpr void left(T t) noexcept { _bottom_left.x = t; }

      /** Give back the right side of the box */

      /**
       * Give back the right side of the box.
       * If the box is empty, then the return value is not defined.
       */
      constexpr T right() const noexcept { return _top_right.x; }

      /** Set the right side of the box */

      /**
       * Set the right side of the box.
       * @pre The box must not be empty.
       */
      constexpr void right(T t) noexcept { _top_right.x = t; }

      /** Give back the height of the box */

      /**
       * Give back the height of the box.
       * If the box is empty, then the return value is not defined.
       */
      constexpr T height() const noexcept { return _top_right.y - _bottom_left.y; }

      /** Give back the width of the box */

      /**
       * Give back the width of the box.
       * If the box is empty, then the return value is not defined.
       */
      constexpr T width() const noexcept { return _top_right.x - _bottom_left.x; }

      /** Checks whether a point is inside the box */
      constexpr bool inside(const Point< T >& u) const noexcept {
        if (_empty)
          return false;
        else {
          return ((u.x - _bottom_left.x) * (_top_right.x - u.x) >= 0
                  && (u.y - _bottom_left.y) * (_top_right.y - u.y) >= 0);
        }
      }

      /** Increments the box with a point */

      /**
       * Increments the box with a point.
       *
       */
      constexpr Box& add(const Point< T >& u) noexcept {
        if (_empty) {
          _bottom_left = _top_right = u;
          _empty = false;
        } else {
          if (_bottom_left.x > u.x) _bottom_left.x = u.x;
          if (_bottom_left.y > u.y) _bottom_left.y = u.y;
          if (_top_right.x < u.x) _top_right.x = u.x;
          if (_top_right.y < u.y) _top_right.y = u.y;
        }
        return *this;
      }

      /** Increments the box to contain another box */

      /**
       * Increments the box to contain another box.
       *
       */
      constexpr Box& add(const Box& u) noexcept {
        if (!u.empty()) {
          add(u._bottom_left);
          add(u._top_right);
        }
        return *this;
      }

      /** Intersection of two boxes */

      /**
       * Intersection of two boxes.
       *
       */
      constexpr Box operator&(const Box& u) const noexcept {
        Box b;
        if (_empty || u._empty) {
          b._empty = true;
        } else {
          b._bottom_left.x = std::max(_bottom_left.x, u._bottom_left.x);
          b._bottom_left.y = std::max(_bottom_left.y, u._bottom_left.y);
          b._top_right.x = std::min(_top_right.x, u._top_right.x);
          b._top_right.y = std::min(_top_right.y, u._top_right.y);
          b._empty = b._bottom_left.x > b._top_right.x || b._bottom_left.y > b._top_right.y;
        }
        return b;
      }

    };   // class Box

    /** Read a box from a stream */

    /**
     * Read a box from a stream.
     * \relates Box
     */
    template < typename T >
    inline std::istream& operator>>(std::istream& is, Box< T >& b) {
      char       c;
      Point< T > p;
      if (is >> c) {
        if (c != '(') is.putback(c);
      } else {
        is.clear();
      }
      if (!(is >> p)) return is;
      b.bottomLeft(p);
      if (is >> c) {
        if (c != ',') is.putback(c);
      } else {
        is.clear();
      }
      if (!(is >> p)) return is;
      b.topRight(p);
      if (is >> c) {
        if (c != ')') is.putback(c);
      } else {
        is.clear();
      }
      return is;
    }

    /** Write a box to a stream */

    /**
     * Write a box to a stream.
     * \relates Box
     */
    template < typename T >
    inline std::ostream& operator<<(std::ostream& os, const Box< T >& b) {
      os << "(" << b.bottomLeft() << "," << b.topRight() << ")";
      return os;
    }

    /** Map of x-coordinates of a <tt>Point</tt>-map */

    /**
     * Map of x-coordinates of a @ref Point "Point"-map.
     *
     */
    template < class M >
    class XMap {
      M& _map;

      public:
      using Value = typename M::Value::Value;
      using Key = typename M::Key;

      constexpr XMap(M& map) noexcept : _map(map) {}
      constexpr Value operator[](Key k) const noexcept { return _map[k].x; }
      constexpr void  set(Key k, Value v) noexcept { _map.set(k, typename M::Value(v, _map[k].y)); }
    };

    /** Returns an XMap class */

    /**
     * This function just returns an XMap class.
     * \relates XMap
     */
    template < class M >
    inline XMap< M > xMap(M& m) {
      return XMap< M >(m);
    }

    template < class M >
    inline XMap< M > xMap(const M& m) {
      return XMap< M >(m);
    }

    /** Constant (read only) version of XMap */

    /**
     * Constant (read only) version of XMap.
     *
     */
    template < class M >
    class ConstXMap {
      const M& _map;

      public:
      using Value = typename M::Value::Value;
      using Key = typename M::Key;

      constexpr ConstXMap(const M& map) noexcept : _map(map) {}
      constexpr Value operator[](Key k) const noexcept { return _map[k].x; }
    };

    /** Returns a ConstXMap class */

    /**
     * This function just returns a ConstXMap class.
     * \relates ConstXMap
     */
    template < class M >
    inline ConstXMap< M > xMap(const M& m) {
      return ConstXMap< M >(m);
    }

    /** Map of y-coordinates of a <tt>Point</tt>-map */

    /**
     * Map of y-coordinates of a @ref Point "Point"-map.
     *
     */
    template < class M >
    class YMap {
      M& _map;

      public:
      using Value = typename M::Value::Value;
      using Key = typename M::Key;

      constexpr YMap(M& map) noexcept : _map(map) {}
      constexpr Value operator[](Key k) const noexcept { return _map[k].y; }
      constexpr void  set(Key k, Value v) noexcept { _map.set(k, typename M::Value(_map[k].x, v)); }
    };

    /** Returns a YMap class */

    /**
     * This function just returns a YMap class.
     * \relates YMap
     */
    template < class M >
    inline YMap< M > yMap(M& m) {
      return YMap< M >(m);
    }

    template < class M >
    inline YMap< M > yMap(const M& m) {
      return YMap< M >(m);
    }

    /** Constant (read only) version of YMap */

    /**
     * Constant (read only) version of YMap.
     *
     */
    template < class M >
    class ConstYMap {
      const M& _map;

      public:
      using Value = typename M::Value::Value;
      using Key = typename M::Key;

      constexpr ConstYMap(const M& map) noexcept : _map(map) {}
      constexpr Value operator[](Key k) const noexcept { return _map[k].y; }
    };

    /** Returns a ConstYMap class */

    /**
     * This function just returns a ConstYMap class.
     * \relates ConstYMap
     */
    template < class M >
    inline ConstYMap< M > yMap(const M& m) {
      return ConstYMap< M >(m);
    }

    /**
     * @brief Map of the normSquare() of a <tt>Point</tt>-map
     *
     * Map of the @ref Point::normSquare() "normSquare()"
     * of a @ref Point "Point"-map.
     */
    template < class M >
    class NormSquareMap {
      const M& _map;

      public:
      using Value = typename M::Value::Value;
      using Key = typename M::Key;

      constexpr NormSquareMap(const M& map) noexcept : _map(map) {}
      constexpr Value operator[](Key k) const noexcept { return _map[k].normSquare(); }
    };

    /** Returns a NormSquareMap class */

    /**
     * This function just returns a NormSquareMap class.
     * \relates NormSquareMap
     */
    template < class M >
    inline NormSquareMap< M > normSquareMap(const M& m) {
      return NormSquareMap< M >(m);
    }

    /** @} */

  }   // namespace dim2

}   // namespace nt

#endif   // LEMON_DIM2_H
