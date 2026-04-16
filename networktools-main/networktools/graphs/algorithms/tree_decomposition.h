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
 * @brief Tree decomposition computation for graphs.
 *
 * A tree decomposition represents a graph as a tree structure where each node (bag) contains
 * a subset of vertices. This structure enables efficient dynamic programming algorithms for
 * NP-hard problems on graphs with bounded treewidth.
 *
 * **Key concepts**:
 * - **Bag**: A subset of graph vertices forming a node in the decomposition tree
 * - **Treewidth**: Maximum bag size minus 1 (smaller is better for algorithm efficiency)
 * - **Elimination ordering**: Vertex ordering that determines the decomposition quality
 *
 * **Applications**:
 * - Vertex cover problems
 * - Network optimization with bounded treewidth
 * - Dynamic programming on graphs
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_TREE_DECOMPOSITION_H_
#define _NT_TREE_DECOMPOSITION_H_

#include "../tools.h"
#include "../graph_element_set.h"
#include "../algorithms/chordality.h"
#include "../../core/sort.h"
#include "../../core/format.h"
#include "../../core/stack.h"
#include "../../graphs/csr_graph.h"

namespace nt {
  namespace graphs {

    /**
     * @class TreeDecomposition
     * @headerfile tree_decomposition.h
     * @brief Computes and manages tree decompositions of graphs for dynamic programming algorithms.
     *
     * A tree decomposition decomposes a graph into a tree structure where each tree node (called a "bag")
     * contains a subset of graph vertices. This structure satisfies:
     * 1. Every graph vertex appears in at least one bag
     * 2. For each edge (u,v), some bag contains both u and v
     * 3. For each vertex v, bags containing v form a connected subtree
     *
     * **Algorithm**: Uses minimum degree elimination ordering with chordal graph completion
     * to compute a tree decomposition with good treewidth bounds.
     *
     * **Typical usage**:
     * ```cpp
     * TreeDecomposition<ListDigraph> td(graph);
     * td.build();
     *
     * std::cout << "Treewidth: " << td.width() << std::endl;
     * std::cout << "Number of bags: " << td.bagNum() << std::endl;
     *
     * // Traverse bags in post-order for dynamic programming
     * td.postOrderTraversal([&](int bag_id, int parent_id) {
     *   const auto& vertices = td.bag(bag_id);  // Direct int-based access
     *   // Process bag...
     * });
     * ```
     *
     * **Performance notes**:
     * - Suitable for graphs with small treewidth (< 20 typically)
     * - Width quality depends on elimination heuristic (currently minimum degree)
     * - For better width bounds, consider preprocessing with vertex ordering heuristics
     *
     * @tparam GR The type of the digraph (ListDigraph, SmartDigraph, etc.)
     *
     * @see Chordality for elimination ordering computation
     * @see MutableGraph for internal graph representation
     */
    template < class GR >
    struct TreeDecomposition {
      using Digraph = GR;
      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
      using Tree = graphs::CsrDigraph;   ///< Decomposition tree structure (CSR for efficiency)
      using Bag = NodeSet< Digraph >;    ///< Set of vertices in a bag
      using BagMap = IntNodeMap;         ///< Maps graph nodes to bag indices
      using TNode = Tree::Node;          ///< Tree node type (represents a bag)

      enum class State { NOT_BUILT, BUILT };   ///< Current build state of the decomposition

      enum class EliminationHeuristic {
        MIN_DEGREE,       ///< Minimum degree (current default)
        // MIN_FILL,         ///< Minimum fill-in edges (TODO)
        // MAX_CARDINALITY   ///< Maximum cardinality search (TODO)
      };

      /**
       * @brief Iterator over all bags in the decomposition
       */
      struct BagIterator {
        const TreeDecomposition* td;
        int                      current_id;

        BagIterator(const TreeDecomposition* td, int id) : td(td), current_id(id) {}

        TNode        operator*() const { return td->tree().nodeFromId(current_id); }
        BagIterator& operator++() {
          ++current_id;
          return *this;
        }
        bool operator!=(const BagIterator& other) const { return current_id != other.current_id; }
      };

      const Digraph&             _graph;          ///< The input graph to decompose
      Chordality< Digraph >      _chor;           ///< Chordality algorithm for elimination ordering
      Tree                       _tree;           ///< The decomposition tree structure
      Tree::StaticNodeMap< Bag > _bags;           ///< Maps each tree node to its bag of vertices
      nt::IntDynamicArray        _parent;         ///< Maps each bag to its parent bag index
      int                        _i_max_clique;   ///< Maximum clique size found during decomposition
      State                      _state;          ///< Current build state of the decomposition

      /**
       * @brief Construct a tree decomposition for the given graph.
       * @param graph The input graph (will be referenced, must remain valid)
       * @note Call build() to actually compute the decomposition
       */
      TreeDecomposition(const Digraph& graph) :
          _graph(graph), _chor(graph), _i_max_clique(-1), _state(State::NOT_BUILT){};

      TreeDecomposition(const TreeDecomposition&) = delete;
      TreeDecomposition& operator=(const TreeDecomposition&) = delete;

      /**
       * @brief Traverse the tree decomposition in post-order (children before parents).
       *
       * Post-order traversal processes each bag after all its children have been processed,
       * which is the natural order for bottom-up dynamic programming algorithms.
       *
       * **Typical usage**:
       * ```cpp
       * TreeDecomposition<Digraph> td(graph);
       * td.build();
       *
       * td.postOrderTraversal([&](int bag_id, int parent_id) {
       *   if (td.isLeaf(bag_id)) { 
       *     // Initialize DP state for leaf bags
       *   }
       *   else {
       *     // Combine DP states from children
       *     for (int k = 0; k < td.childNum(bag_id); ++k) {
       *       TNode child = td.kthChild(bag_id, k);
       *       // Process child states...
       *     }
       *   }
       * });
       * ```
       *
       * @param callback Callback function(bag_id, parent_id) called for each bag.
       *                 - bag_id: Integer ID of the current bag
       *                 - parent_id: Integer ID of the parent bag (0 for root)
       *
       * @note The root bag is visited last in post-order traversal
       */
      void postOrderTraversal(const std::function< void(int, int) >& callback) const noexcept {
        if (bagNum() == 0) return;   // Empty decomposition

        nt::Stack< Tree::OutArcIt > stack;

        Tree::OutArcIt e(_tree, _tree.nodeFromId(0));
        if (e != nt::INVALID) stack.push(std::move(e));

        while (!stack.empty()) {
          const Tree::OutArcIt& arc_it = stack.top();

          if (arc_it == nt::INVALID) {
            // All children processed, now visit this node
            stack.pop();
            if (!stack.empty()) {
              Tree::OutArcIt& parent_it = stack.top();
              Tree::Arc       arc = parent_it;
              Tree::Node      current = _tree.target(arc);
              Tree::Node      parent = _tree.source(arc);

              callback(_tree.id(current), _tree.id(parent));
              ++parent_it;   // Move to next child
            }
          } else {
            // Push children first
            const Tree::Node child = _tree.target(arc_it);
            stack.push(Tree::OutArcIt(_tree, child));
          }
        }

        // Visit root last
        callback(0, 0);
      }

      /**
       * @brief Traverse the tree decomposition in pre-order (parents before children)
       * @param callback Callback(bag_id, parent_id) called for each bag
       * @note Root is visited first in pre-order traversal
       */
      void preOrderTraversal(const std::function< void(int, int) >& callback) const {
        std::function< void(int) > visit = [&](int bag_id) {
          const TNode bag = tree().nodeFromId(bag_id);
          callback(bag_id, _parent[bag_id]);

          for (int k = 0; k < childNum(bag); ++k)
            visit(Tree::id(kthChild(bag, k)));
        };
        visit(0);   // Start from root
      }

      /**
       * @brief Compute the tree decomposition of the input graph.
       *
       * This method computes a tree decomposition using minimum degree elimination ordering.
       * The algorithm:
       * 1. Computes an elimination ordering that triangulates the graph
       * 2. Constructs bags from the chordal completion
       * 3. Builds the decomposition tree structure
       *
       * @param heuristic The vertex elimination heuristic to use
       *
       * @note Any previous decomposition is cleared before building
       * @note After calling build(), use width() to get the treewidth
       *
       * @see width() to get the computed treewidth
       * @see bagNum() to get the number of bags
       */
      void build(EliminationHeuristic heuristic = EliminationHeuristic::MIN_DEGREE) noexcept {
        // Step 0 - Reset & allocate memory
        clear();

        _bags.buildEmplace(_graph.nodeNum(), _graph);
        _parent.ensure(_graph.nodeNum());

        nt::TrivialDynamicArray< nt::Pair< int, int > > arcs(_graph.nodeNum());

        // Step 1 - Compute an elimination order and deduce the width
        TrivialDynamicArray< Node >   elim_order;
        MutableGraph< Digraph, true > chordal_g(_graph);
        _i_max_clique = _chor.findElimOrderMinDegree(elim_order, chordal_g);

        // Step 2 - Build the tree w.r.t the found elim order
        BagMap                                                   node2bag(_graph);
        typename Digraph::template StaticNodeMap< unsigned int > rank(_graph);
        rank.setBitsToOne();

        for (int k = elim_order.size() - 1; k >= 0; --k) {
          const Node& u = elim_order[k];
          const int   i_curr_rank = elim_order.size() - k - 1;
          rank[u] = i_curr_rank;

          int i_max_rank = 0;
          int i_max_bag = 0;
          Bag new_bag(_graph);
          new_bag.insert(u);
          for (const Node& v: chordal_g.neighbors(u)) {
            if (rank[v] < i_curr_rank) {
              new_bag.insert(v);
              if (rank[v] >= i_max_rank) {
                i_max_rank = rank[v];
                i_max_bag = node2bag[v];
              }
            }
          }

          //
          _bags.set(i_curr_rank, std::move(new_bag));
          node2bag[u] = i_curr_rank;
          _parent.add(i_max_bag);
          arcs[k] = {i_max_bag, i_curr_rank};
        }

        //
        _tree.build(_bags.size(), arcs.begin(), arcs.end() - 1);

        _state = State::BUILT;
      }

      /**
       * @brief Clear the tree decomposition and reset to initial state.
       *
       * After calling clear(), the decomposition must be rebuilt with build() before use.
       *
       * @post width() returns -1, bagNum() returns 0, tree is empty
       */
      void clear() noexcept {
        _parent.removeAll();
        _tree.clear();
        _i_max_clique = -1;
        _state = State::NOT_BUILT;
      }

      /**
       * @brief Check if the tree decomposition has been built.
       *
       * @return true if built, false otherwise
       */
      constexpr bool isBuilt() const noexcept { return _state == State::BUILT; }

      /**
       * @brief Get the set of vertices in a bag.
       *
       * @param bag Tree node representing the bag
       * @return Const reference to the NodeSet containing vertices in this bag
       *
       * @note Returns a set that can be iterated: `for (Node v : td.bag(bag_node))`
       */
      constexpr const Bag& bag(TNode bag) const noexcept { return _bags[_tree.id(bag)]; }

      /**
       * @brief Get the set of vertices in a bag (int-based overload).
       *
       * @param i_bag Integer ID of the bag
       * @return Const reference to the NodeSet containing vertices in this bag
       */
      constexpr const Bag& bag(int i_bag) const noexcept { return bag(tree().nodeFromId(i_bag)); }

      /**
       * @brief Get the decomposition tree structure.
       *
       * @return Const reference to the tree (CsrDigraph) representing bag parent-child relationships
       *
       * @note Use with tree navigation methods: nodeFromId(), outDegree(), etc.
       */
      constexpr const Tree& tree() const noexcept { return _tree; }

      /**
       * @brief Get the number of bags in the tree decomposition.
       *
       * @return Number of bags (tree nodes)
       *
       * @note Returns 0 if build() has not been called or after clear()
       */
      constexpr int bagNum() const noexcept { return _tree.nodeNum(); }

      /**
       * @brief Get the root bag of the decomposition tree.
       *
       * @return Integer ID of the root bag (always 0)
       *
       * @note The root is arbitrarily chosen during construction
       */
      constexpr int root() const noexcept { return 0; }

      /**
       * @brief Get the treewidth of the computed decomposition.
       *
       * Treewidth = (maximum bag size) - 1
       *
       * Lower treewidth indicates better decomposition quality and enables faster
       * dynamic programming algorithms.
       *
       * @return The treewidth, or -1 if build() has not been called
       *
       * @note Many NP-hard problems become tractable for graphs with small treewidth
       */
      int width() const {
        NT_ASSERT(isBuilt(), "TreeDecomposition::width() called before build()");
        return _i_max_clique - 1;
      }

      /**
       * @brief Compute the height (distance from root) of a bag in the tree.
       *
       * @param i_bag The bag to compute height for
       * @return Height as number of edges from root (root has height 0)
       *
       */
      int height(TNode i_bag) const noexcept {
        int h = 0;
        int i = Tree::id(i_bag);
        while (_parent[i]) {
          i = _parent[i];
          ++h;
        }
        return h;
      }

      /**
       * @brief Get the integer ID of a bag node.
       *
       * @param i_bag Tree node representing the bag
       * @return Integer ID (0-based index)
       */
      static constexpr int id(TNode i_bag) noexcept { return Tree::id(i_bag); }

      /**
       * @brief Check if a bag is a leaf (has no children).
       *
       * @param bag Tree node to check
       * @return True if the bag has no children, false otherwise
       *
       * @note Useful for base cases in dynamic programming
       */
      constexpr bool isLeaf(TNode bag) const noexcept { return _tree.outDegree(bag) == 0; }

      /**
       * @brief Check if a bag is a leaf (int-based overload).
       *
       * @param i_bag Integer ID of the bag
       * @return True if the bag has no children, false otherwise
       */
      constexpr bool isLeaf(int i_bag) const noexcept { return isLeaf(tree().nodeFromId(i_bag)); }

      /**
       * @brief Get the number of children of a bag.
       *
       * @param bag Tree node to query
       * @return Number of child bags (0 for leaf bags)
       */
      constexpr int childNum(TNode bag) const noexcept { return _tree.outDegree(bag); }

      /**
       * @brief Get the number of children of a bag (int-based overload).
       *
       * @param i_bag Integer ID of the bag
       * @return Number of child bags (0 for leaf bags)
       */
      constexpr int childNum(int i_bag) const noexcept { return childNum(tree().nodeFromId(i_bag)); }

      /**
       * @brief Get the k-th child of a bag.
       *
       * @param bag Parent bag
       * @param k Child index (0-based)
       * @return Tree node of the k-th child
       *
       * @pre k < childNum(bag)
       * @note Use with childNum() to iterate all children
       */
      constexpr TNode kthChild(TNode bag, int k) const noexcept { return _tree.kthOutNeighbor(bag, k); }

      /**
       * @brief Get the k-th child of a bag (int-based overload).
       *
       * @param i_bag Parent bag ID
       * @param k Child index (0-based)
       * @return Tree node of the k-th child
       *
       * @pre k < childNum(i_bag)
       */
      constexpr TNode kthChild(int i_bag, int k) const noexcept { return kthChild(tree().nodeFromId(i_bag), k); }

      /**
       * @brief Get the first child of a bag.
       *
       * Equivalent to kthChild(i_bag, 0).
       *
       * @param i_bag Parent bag
       * @return Tree node of the first child
       *
       * @pre childNum(i_bag) > 0 (i.e., not a leaf)
       */
      constexpr TNode firstChild(TNode i_bag) const noexcept { return _tree.kthOutNeighbor(i_bag, 0); }

      /**
       * @brief Get parent bag of a bag.
       *
       * @param bag The bag to get parent of
       * @return Parent bag node, or INVALID if bag is root
       */
      constexpr TNode parent(TNode bag) const noexcept {
        const int parent_id = _parent[Tree::id(bag)];
        return tree().nodeFromId(parent_id);
      }

      /**
       * @brief Get parent bag ID.
       *
       * @param i_bag Integer ID of the bag
       * @return Integer ID of parent bag (root's parent is 0)
       */
      constexpr int parentId(int i_bag) const noexcept { return _parent[i_bag]; }

      /**
       * @brief Check if a bag is the root.
       *
       * @param bag Tree node to check
       * @return True if the bag is the root, false otherwise
       */
      constexpr bool isRoot(TNode bag) const noexcept { return Tree::id(bag) == 0; }

      /**
       * @brief Check if a bag is the root (int-based overload).
       *
       * @param i_bag Integer ID of the bag
       * @return True if the bag is the root, false otherwise
       */
      constexpr bool isRoot(int i_bag) const noexcept { return i_bag == 0; }

      /**
       * @brief Iterate over all bags with a callback.
       *
       * @param callback Callback(TNode bag) called for each bag in order
       *
       * @note Bags are visited in ID order (0 to bagNum()-1), not traversal order
       */
      constexpr void forEachBag(const std::function< void(TNode) >& callback) const noexcept {
        for (int i = 0; i < bagNum(); ++i)
          callback(tree().nodeFromId(i));
      }

      /**
       * @brief Get maximum bag size in the decomposition.
       *
       * @return Maximum number of vertices in any bag
       *
       */
      constexpr int maxBagSize() const noexcept { return _i_max_clique; }

      /**
       * @brief Get iterator to first bag for range-based for loops.
       *
       * **Usage**:
       * ```cpp
       * for (TNode bag : td) {
       *    std::cout << "Bag " << td.id(bag) << " has "
       *              << td.bag(bag).size() << " vertices\n";
       *}
       *```
       * @return BagIterator pointing to first bag
       */
      BagIterator begin() const { return BagIterator(this, 0); }

      /**
       * @brief Get iterator past last bag for range-based for loops.
       *
       * @return BagIterator pointing past last bag
       */
      BagIterator end() const { return BagIterator(this, bagNum()); }

      /**
       * @brief Export the tree decomposition to PACE format string.
       *
       * ```
       * s td <num_bags> <treewidth> <num_vertices>
       * b <bag_id> <vertex_id>...
       * ...
       * <parent_id> <child_id>
       * ...
       * ```
       *
       * @param i_shift_id Optional offset added to all IDs (for 1-based indexing)
       * @return String representation in PACE format
       *
       * @note Useful for visualization, validation, and interoperability with other tools
       */
      nt::String toString(int i_shift_id = 0) const noexcept {
        nt::MemoryBuffer buffer;
        nt::format_to(std::back_inserter(buffer), "s td {} {} {}\n", bagNum(), width(), _graph.nodeNum());
        for (int i = 0; i < bagNum(); ++i) {
          nt::format_to(std::back_inserter(buffer), "b {} ", i + i_shift_id);
          for (const Node& v: _bags[i])
            nt::format_to(std::back_inserter(buffer), "{} ", _graph.id(v) + i_shift_id);
          nt::format_to(std::back_inserter(buffer), "\n");
        }
        for (int i = 1; i < bagNum(); ++i)
          nt::format_to(std::back_inserter(buffer), "{} {}\n", _parent[i] + i_shift_id, i + i_shift_id);
        return nt::to_string(buffer);
      }
    };

  }   // namespace graphs
}   // namespace nt

#endif
