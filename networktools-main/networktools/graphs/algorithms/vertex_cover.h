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
 * @brief Exact algorithm for computing the minimum vertex cover of a graph.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_VERTEX_COVER_H_
#define _NT_VERTEX_COVER_H_

#include "connectivity.h"
#include "tree_decomposition.h"

namespace nt {
  namespace graphs {

    /**
     * @class VertexCover
     * @headerfile vertex_cover.h
     * @brief Exact algorithm for computing the minimum vertex cover of a graph.
     *
     * The algorithm is based on dynamic programming using a tree decomposition of the the input graph.
     *
     * @tparam GR The type of the digraph the algorithm runs on. The digraph is assumed to be bidirected.
     *
     */
    template < class GR >
    // requires nt::concepts::Digraph< GR >
    struct VertexCover {
      using Digraph = GR;
      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

      using TD = TreeDecomposition< Digraph >;
      using Bag = typename TD::Bag;

      /**
       * @brief A coloring is a 0-1 sequence of length n that determines which vertices of the graph are part of a
       * vertex cover
       *
       */
      struct Coloring {
        nt::BitArray _col;

        explicit Coloring(const Digraph& g) { _col.extendByBits(g.nodeNum()); };

        Coloring(const Coloring&) = delete;
        Coloring& operator=(const Coloring&) = delete;

        Coloring(Coloring&& other) noexcept : _col(std::move(other._col)) {}
        Coloring& operator=(Coloring&& other) noexcept {
          _col = std::move(other._col);
          return *this;
        }

        /**
         * @brief This function extends the coloring 'i_bag_coloring' of 'bag' to the entire graph
         *
         * Suppose that we have a graph of 5 vertices, a bag containing the vertices 0, 3 and 4, and
         * a bag coloring i_bag_coloring = 7 = 111 (in binary). Then the function computes the coloring of
         * the input graph as follows
         *
         *  0 3 4     0 1 2 3 4
         *  1 1 1  => 1 0 0 1 1
         *
         */
        void fromBagColoring(unsigned int i_bag_coloring, const Bag& bag) noexcept {
          _col.setBitsToZero();
          for (int k = 0; k < bag.size(); ++k)
            if (NT_IS_ONE_AT(i_bag_coloring, k)) _col.setOneAt(Digraph::id(bag[k]));
        }

        /**
         * @brief This function returns the current coloring restricted to the vertices in 'bag'
         *
         * Suppose that we have a graph of 5 vertices, a bag containing the vertices 0, 2 and 4, and
         * a graph coloring 10011 (in binary). Then the function computes the coloring of the given bag
         * as follows
         *
         *  0 1 2 3 4     0 2 4
         *  1 0 0 1 1  => 1 0 1
         *
         */
        unsigned int getBagColoring(const Bag& bag) const noexcept {
          unsigned int i_bag_coloring = 0;
          for (int k = 0; k < bag.size(); ++k)
            if (isColored(bag[k])) i_bag_coloring |= (1 << k);
          return i_bag_coloring;
        }

        /**
         * @brief Return true if 'u' has color '1' (i.e is part of the current vertex cover), false otherwise
         *
         */
        constexpr bool isColored(Node u) const noexcept { return _col.isOne(Digraph::id(u)); }

        /**
         * @brief Return the number of vertices in the current vertex cover
         *
         */
        constexpr int value() const noexcept { return _col.countOnes(); }
      };


      /**
       * @brief A DynamicTable represents the table that stores all valid colorings associated to each bag of the tree
       * decomposition
       *
       */
      struct DynamicTable {
        using iterator = nt::DynamicArray< Coloring* >::iterator;
        using const_iterator = nt::DynamicArray< Coloring* >::const_iterator;
        using reference = nt::DynamicArray< Coloring* >::reference;
        using const_reference = nt::DynamicArray< Coloring* >::const_reference;

        // _key_2_col : replacing 'int' directly with 'Coloring*' reduces the number of indirections but
        // if we modify a value in _key_2_col we would need to modify it also in _table to keep them in sync
        // and it is not obvious how to find the value to update in _table.
        nt::NumericalMap< unsigned int, int > _key_2_col;
        nt::DynamicArray< Coloring* >         _table;

        void clear() noexcept {
          _table.removeAll();
          _key_2_col.clear();
        }

        void insert(unsigned int key, Coloring* p_coloring) noexcept {
          const int i = _table.add(p_coloring);
          _key_2_col.insert({key, i});
        }

        Coloring** findOrNull(unsigned int key) noexcept {
          if (int* p_idx = _key_2_col.findOrNull(key)) return &_table[*p_idx];
          return nullptr;
        }

        constexpr int size() const noexcept { return _table.size(); }

        constexpr reference       operator[](int index) noexcept { return _table[index]; }
        constexpr const_reference operator[](int index) const noexcept { return _table[index]; }

        iterator       begin() noexcept { return _table.begin(); }
        const_iterator begin() const noexcept { return _table.begin(); }

        iterator       end() noexcept { return _table.end(); }
        const_iterator end() const noexcept { return _table.end(); }
      };

      /**
       * @brief Memory pool to store all the coloring computed during the course of the algorithm
       *
       * TODO(Morgan) : Is there any advantage of using a memory manager for storing all solutions ?
       *
       */
      struct ColorationPool {
        nt::FixedMemoryArray< Coloring, 7 > _library;

        constexpr Coloring& create(const Digraph& g) noexcept { return _library[_library.add(g)]; }
        constexpr void      clear() noexcept { _library.removeAll(); }
        constexpr int       size() const noexcept { return _library.size(); }
      };


      const Digraph&                   _g;    // < Input graph
      TD                               _td;   // < Tree decomposition of the input graph
      nt::DynamicArray< DynamicTable > _dt;   // < Dynamic tables : _dt[i][j] = j-th valid vertex cover of the bag i
      DynamicTable                     _inter_dt;    // < Store the dynamic table between two tree decomposition node
      ColorationPool                   _cols_pool;   // < Memory pool containing the colorings

      VertexCover(const Digraph& g) : _g(g), _td(g) {}

      /**
       * @brief Computes the minimum vertex cover
       *
       * @return int The size of the minimum vertex cover
       */
      int run() noexcept {
        int min_vc = NT_INT32_MAX;

        _td.build();

        if (!_td.bagNum()) {
          assert(false);
          return min_vc;
        }

        const typename TD::Tree& tree = _td.tree();

        LOG_F(INFO, "TD width : {}", _td.width());

        _cols_pool.clear();

        // Bags initialization
        _dt.ensureAndFill(_td.bagNum());

        // Tree traversal
        _td.postOrderTraversal([&](int i_bag, int) {
          if (_td.isLeaf(i_bag))
            _processLeafNode(i_bag);
          else {
            for (int i = 0; i < _td.childNum(i_bag); ++i)
              _processInnerNode(i_bag, i);
          }
        });

        // Return the minimum vertex cover value from the root table
        DynamicTable& root_dt = _dt[0];
        for (Coloring* p_col: root_dt) {
          const int v = p_col->value();
          if (min_vc > v) min_vc = v;
        }

        return min_vc;
      }

      /**
       * @brief Get the Vertex Cover object
       *
       * @return NodeSet< Digraph >
       */
      NodeSet< Digraph > getVertexCover() noexcept {
        NodeSet< Digraph > cover(_g);
        int                min_vc = NT_INT32_MAX;

        Coloring*     p_best_col = nullptr;
        DynamicTable& root_dt = _dt[0];
        for (Coloring* p_col: root_dt) {
          const int v = p_col->value();
          if (min_vc > v) {
            min_vc = v;
            p_best_col = p_col;
          }
        }

        if (p_best_col) {
          for (NodeIt u(_g); u != nt::INVALID; ++u) {
            if (p_best_col->isColored(u)) { cover.insert(u); }
          }
        }

        return cover;
      }

      /**
       * @brief Return a string representation of the dynamic table associated to the given bag
       *
       */
      nt::String printDynamicTable(int i_bag) const noexcept {
        const Bag&          bag = _td.bag(i_bag);
        const DynamicTable& dt = _dt[i_bag];
        return printDynamicTable(bag, dt, i_bag);
      }

      /**
       * @brief
       *
       */
      nt::String printDynamicTable(const Bag& bag, const DynamicTable& dt, int i_bag = -1) const noexcept {
        nt::MemoryBuffer buffer;

        nt::format_to(std::back_inserter(buffer), "Dynamic table of bag {}\n", i_bag);
        for (Node u: bag)
          nt::format_to(std::back_inserter(buffer), "{} ", _g.id(u));
        nt::format_to(std::back_inserter(buffer), "V\n");

        for (Coloring* p_col: dt) {
          for (Node u: bag)
            nt::format_to(std::back_inserter(buffer), "{} ", p_col->isColored(u) ? "1" : "0");
          nt::format_to(std::back_inserter(buffer), "{} : {}\n", p_col->value(), p_col->_col.toString(_g.nodeNum()));
        }

        return nt::to_string(buffer);
      }

      //---------------------------------------------------------------------------
      // Auxiliary methods
      //---------------------------------------------------------------------------

      bool _isVertexCover(unsigned int i_bag_coloring, const Bag& bag) const noexcept {
        for (int i = 0; i < bag.size(); ++i) {
          if (NT_IS_ONE_AT(i_bag_coloring, i))
            continue;   // If u is in the solution then all adjacent arcs are covered.
          const Node& u = bag[i];
          for (OutArcIt arc(_g, u); arc != nt::INVALID; ++arc) {
            const Node v = _g.target(arc);
            if (!bag.contains(v)) continue;   // Make sure we check the vertex cover on the subgraph G[bag] only
            if (!NT_IS_ONE_AT(i_bag_coloring, bag.find(v))) return false;   // Is v in the vertex cover ?
          }
        }
        return true;
      }

      void _processLeafNode(int i_bag) noexcept {
        const Bag& bag = _td.bag(i_bag);
        // For each possible coloring of the vertices in the bag
        for (unsigned int i_bag_coloring = 0; i_bag_coloring < NT_TWO(bag.size()); ++i_bag_coloring) {
          // Check if the current solution is a vertex cover in the induced subgraph
          if (_isVertexCover(i_bag_coloring, bag)) {
            // Store the solution and its value
            Coloring& col = _cols_pool.create(_g);
            col.fromBagColoring(i_bag_coloring, bag);
            _dt[i_bag].insert(i_bag_coloring, &col);
          }
        }
      }


      void _processInnerNode(int i_parent_bag, int k) noexcept {
        const Bag& parent_bag = _td.bag(i_parent_bag);
        const int  i_child_bag = _td.id(_td.kthChild(i_parent_bag, k));
        const Bag& child_bag = _td.bag(i_child_bag);
        const Bag  inter_bag = parent_bag.intersectionSet(child_bag);

        _inter_dt.clear();

        for (Coloring* p_child_col: _dt[i_child_bag]) {
          const unsigned int i_bag_coloring = p_child_col->getBagColoring(inter_bag);
          if (Coloring** p_inter_col = _inter_dt.findOrNull(i_bag_coloring)) {
            if ((*p_inter_col)->value() > p_child_col->value()) (*p_inter_col) = p_child_col;
          } else {
            _inter_dt.insert(i_bag_coloring, p_child_col);
          }
        }

        const int i_num_inserted = parent_bag.size() - inter_bag.size();

        DynamicTable& parent_dt = _dt[i_parent_bag];

        if (i_num_inserted > 0) {
          for (Coloring* p_inter_col: _inter_dt) {
            const unsigned int i_bag_coloring_original = p_inter_col->getBagColoring(parent_bag);

            for (unsigned int k = 0; k < NT_TWO(i_num_inserted); ++k) {
              int l = 0;

              // Update the mask with the inserted vertices that are selected according to k
              unsigned int i_bag_coloring = i_bag_coloring_original;
              for (unsigned int i = 0; i < parent_bag.size(); ++i) {
                if (inter_bag.contains(parent_bag[i])) continue;
                if (NT_IS_ONE_AT(k, l)) i_bag_coloring |= 1 << i;
                ++l;
              }

              if (Coloring** p_parent_col = parent_dt.findOrNull(i_bag_coloring)) {
                (*p_parent_col)->_col |= p_inter_col->_col;
              } else {
                // Check whether the new solution is a valid vertex cover
                if (_isVertexCover(i_bag_coloring, parent_bag)) {
                  Coloring& parent_col = _cols_pool.create(_g);
                  parent_col.fromBagColoring(i_bag_coloring, parent_bag);
                  parent_col._col |= p_inter_col->_col;
                  parent_dt.insert(i_bag_coloring, &parent_col);
                }
              }
            }
          }
        } else {
          for (Coloring* p_inter_col: _inter_dt) {
            const unsigned int i_bag_coloring = p_inter_col->getBagColoring(parent_bag);

            if (Coloring** p_parent_col = parent_dt.findOrNull(i_bag_coloring)) {
              (*p_parent_col)->_col |= p_inter_col->_col;
            } else {
              Coloring& parent_col = _cols_pool.create(_g);
              parent_col.fromBagColoring(i_bag_coloring, parent_bag);
              parent_col._col |= p_inter_col->_col;
              parent_dt.insert(i_bag_coloring, &parent_col);
            }
          }
        }
      }
    };


    /**
     * @brief Returns an approximate minimum vertex cover.
     *
     * The number of vertices in the returned vertex cover is guaranteed
     * to be at most twice the size of the minimum vertex cover.
     *
     * @tparam GR The type of the digraph the algorithm runs on.
     * @param g The input graph
     *
     * @return A vertex cover
     */
    template < class GR >
    NodeSet< GR > greedyVertexCover(const GR& g) noexcept {
      NodeSet< GR > cover(g);

      // As a first step, we could include in 'cover' every vertex that is adjacent to a degree-1 vertex.

      for (typename GR::ArcIt arc(g); arc != nt::INVALID; ++arc) {
        const typename GR::Node u = g.source(arc);
        const typename GR::Node v = g.target(arc);
        if (cover.contains(u) || cover.contains(v)) continue;
        cover.insert(u);
        cover.insert(v);
      }

      return cover;
    }


  }   // namespace graphs
}   // namespace nt


#endif
