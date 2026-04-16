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
 * @brief
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_STATIC_MAP_H_
#define _NT_STATIC_MAP_H_

namespace nt {
  namespace graphs {

    namespace details {
      template < typename GR, typename T >
      struct NumOf {
        constexpr static int v(const GR& g) noexcept {
          assert(false);
          return 0;
        }
        constexpr static int maxId(const GR& g) noexcept {
          assert(false);
          return -1;
        }
      };

      template < typename GR >
      struct NumOf< GR, typename GR::Arc > {
        constexpr static int v(const GR& g) noexcept { return g.arcNum(); }
        constexpr static int maxId(const GR& g) noexcept { return g.maxArcId(); }
      };

      template < typename GR >
      struct NumOf< GR, typename GR::Node > {
        constexpr static int v(const GR& g) noexcept { return g.nodeNum(); }
        constexpr static int maxId(const GR& g) noexcept { return g.maxNodeId(); }
      };

      template < typename GR >
      struct NumOf< GR, typename GR::Edge > {
        constexpr static int v(const GR& g) noexcept { return g.edgeNum(); }
        constexpr static int maxId(const GR& g) noexcept { return g.maxEdgeId(); }
      };

      template < typename GR, typename K, typename ARRAY >
      struct StaticMapBase {
        using Digraph = GR;
        using Key = K;
        using Array = ARRAY;
        using Value = typename Array::value_type;
        using Reference = typename Array::reference;
        using ConstReference = typename Array::const_reference;
        using Item = K;
        using ReferenceMapTag = nt::True;

        ARRAY _values;

        StaticMapBase() = default;
        StaticMapBase(const Digraph& g) { build(g); }
        StaticMapBase(const Digraph& g, const Value& v) { build(g, v); }

        StaticMapBase(StaticMapBase&& other) noexcept : _values(std::move(other._values)) {}
        StaticMapBase& operator=(StaticMapBase&& other) noexcept {
          if (this != &other) _values = std::move(other._values);
          return *this;
        }

        StaticMapBase(const StaticMapBase& other) = delete;
        StaticMapBase& operator=(const StaticMapBase& other) = delete;

        constexpr void build(int i_size) noexcept {
          _values.removeAll();
          _values.ensureAndFill(i_size);
        }

        constexpr void build(int i_size, const Value& v) noexcept {
          _values.removeAll();
          _values.ensureAndFill(i_size, v);
        }

        template < typename... Args >
        constexpr void buildEmplace(int i_size, Args&&... args) {
          _values.removeAll();
          _values.ensureAndFillEmplace(i_size, std::forward< Args >(args)...);
        }

        constexpr void build(const Digraph& g) noexcept { build(details::NumOf< Digraph, K >::maxId(g) + 1); }
        constexpr void build(const Digraph& g, const Value& v) noexcept {
          build(details::NumOf< Digraph, K >::maxId(g) + 1, v);
        }
        template < typename... Args >
        constexpr void buildEmplace(const Digraph& g, Args&&... args) {
          buildEmplace(details::NumOf< Digraph, K >::maxId(g) + 1, std::forward< Args >(args)...);
        }

        constexpr Value&       operator[](int i) noexcept { return _values[i]; }
        constexpr const Value& operator[](int i) const noexcept { return _values[i]; }
        constexpr Value&       operator[](const Key& key) noexcept { return _values[Digraph::id(key)]; }
        constexpr const Value& operator[](const Key& key) const noexcept { return _values[Digraph::id(key)]; }
        constexpr void         set(const Key& key, const Value& val) noexcept { (*this)[key] = val; }
        constexpr void         set(const Key& key, Value&& val) noexcept { (*this)[key] = std::forward< Value >(val); }

        constexpr int  size() const noexcept { return _values.size(); }
        constexpr bool empty() const noexcept { return _values.empty(); }
        constexpr void copyFrom(const StaticMapBase& from) noexcept { _values.copyFrom(from._values); }

        constexpr void setBitsToZero() noexcept { _values.setBitsToZero(); }
        constexpr void setBitsToOne() noexcept { _values.setBitsToOne(); }
      };

    }   // namespace details

    /**
     * @brief
     *
     * @tparam GR
     * @tparam V
     * @tparam ELEMENTS Variadic template of graph elements: Node, Arc or Edge.
     */
    template < class GR, class V, class... ELEMENTS >
    struct StaticGraphMap {
      static_assert(sizeof...(ELEMENTS) > 0, "StaticGraphMap must be of dimension at least one.");

      using Digraph = GR;
      using Value = V;

      struct Row {
        using Value = V;

        StaticGraphMap& m;
        int             _indices[sizeof...(ELEMENTS) - 1];

        template < class... INDICES >
        Row(StaticGraphMap& m, const INDICES&... indices) : m(m) {
          static_assert(sizeof...(INDICES) == sizeof...(ELEMENTS) - 1,
                        "Number of given indices mismatch the array's dimension.");
          int i = 0;
          (..., (_indices[i++] = indices));
        }

        template < typename K >
        constexpr Value& operator[](const K& k) noexcept {
          return m._values[_offset< 0, ELEMENTS... >(Digraph::id(k))];
        }
        template < typename K >
        constexpr const Value& operator[](const K& k) const noexcept {
          return m._values[_offset< 0, ELEMENTS... >(Digraph::id(k))];
        }

        constexpr void setBitsToZero() noexcept {
          const int start = _offset< 0, ELEMENTS... >(0);
          const int count = m.dimSize< sizeof...(ELEMENTS) - 1 >();
          assert(start >= 0);
          assert(start + count <= m._values.size());
          std::memset((void*)(m._values.data() + start), 0, count * sizeof(Value));
        }

        constexpr void setBitsToOne() noexcept {
          const int start = _offset< 0, ELEMENTS... >(0);
          const int count = m.dimSize< sizeof...(ELEMENTS) - 1 >();
          assert(start >= 0);
          assert(start + count <= m._values.size());
          std::memset((void*)(m._values.data() + start), ~0, count * sizeof(Value));
        }

        template < int I, class E, class... REST >
        constexpr int _offset(int k) const noexcept {
          assert(m._p_digraph);
          static_assert(I <= sizeof...(ELEMENTS) - 1);
          if constexpr (I == sizeof...(ELEMENTS) - 1)
            return k;
          else
            return _indices[I] * details::NumOf< Digraph, E >::v(*m._p_digraph) + _offset< I + 1, REST... >(k);
        }
      };

      const Digraph*            _p_digraph;
      nt::DynamicArray< Value > _values;

      StaticGraphMap() = default;
      StaticGraphMap(const Digraph& g) : _p_digraph(&g), _values(domainSize()) {}

      constexpr void build(const Digraph& g) noexcept {
        _p_digraph = &g;
        _values.removeAll();
        _values.ensureAndFill(size());
      }

      /**
       * @brief Replaces the contents with a copy of the contents of from.
       *
       */
      constexpr void copyFrom(const StaticGraphMap& from) noexcept {
        _p_digraph = from._p_digraph;
        _values.copyFrom(from._values);
      }

      /**
       * @brief Returns the number of dimensions of the array.
       *
       * @return The number of dimensions of the array as a constant expression.
       */
      constexpr int dimNum() const noexcept { return sizeof...(ELEMENTS); }

      /**
       * @brief Returns the size of the I-th dimension.
       *
       * @return The size of the I-th dimension.
       */
      template < int I >
      constexpr int dimSize() const noexcept {
        return _dimSize< 0, I, ELEMENTS... >();
      }

      /**
       * @brief Returns the product over all dimension sizes.
       *
       * @return The product over all dimensions sizes.
       */
      constexpr int domainSize() const noexcept {
        const int i_size = _domainSize< ELEMENTS... >();
        return i_size;
      }

      constexpr Value&       operator()(const ELEMENTS&... elements) noexcept { return _values[_offset(elements...)]; }
      constexpr const Value& operator()(const ELEMENTS&... elements) const noexcept {
        return _values[_offset(elements...)];
      }

      constexpr void set(const Value& val, const ELEMENTS&... elements) noexcept {
        _values[_offset(elements...)] = val;
      }
      constexpr void set(Value&& val, const ELEMENTS&... elements) noexcept {
        _values[_offset(elements...)] = std::forward< Value >(val);
      }

      constexpr void setBitsToZero() noexcept { _values.setBitsToZero(); }
      constexpr void setBitsToOne() noexcept { _values.setBitsToOne(); }

      template < class... INDICES >
      constexpr Row row(const INDICES&... indices) noexcept {
        return Row(*this, Digraph::id(indices)...);
      }

      //---------------------------------------------------------------------------
      // STL compliance
      //---------------------------------------------------------------------------
      constexpr int size() const noexcept { return _values.size(); }

      //---------------------------------------------------------------------------
      // Protected
      //---------------------------------------------------------------------------
      template < class E >
      constexpr int _offset(E e) const noexcept {
        return Digraph::id(e);
      }

      template < class E, class EE, class... REST >
      constexpr int _offset(E e, EE ee, REST... elements) const noexcept {
        return Digraph::id(e) * details::NumOf< GR, E >::v(*_p_digraph) + _offset(ee, elements...);
      }

      template < class E >
      constexpr int _domainSize() const {
        return details::NumOf< GR, E >::v(*_p_digraph);
      }

      template < class E, class EE, class... REST >
      constexpr int _domainSize() const {
        return details::NumOf< GR, E >::v(*_p_digraph) * _domainSize< EE, REST... >();
      }

      template < int K, int I, class E, class... REST >
      constexpr int _dimSize() const noexcept {
        static_assert(I < sizeof...(ELEMENTS) && K <= I);
        if constexpr (K == I)
          return details::NumOf< GR, E >::v(*_p_digraph);
        else
          return _dimSize< K + 1, I, REST... >();
      }
    };

  }   // namespace graphs
}   // namespace nt

#endif
