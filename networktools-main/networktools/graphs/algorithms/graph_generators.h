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
 * This file incorporates work from the LEMON graph library (graph_generators.h).
 *
 * Original LEMON Copyright Notice:
 * Copyright (C) 2003-2009 Egervary Jeno Kombinatorikus Optimalizalasi
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
 *   - Replacing std::vector with nt::TrivialDynamicArray
 *   - Add partial k-tree random generator
 *   - directed path and tournament graph generators
 */

/**
 * @file
 * @brief Various graph generators
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_GRAPH_GENERATORS_H_
#define _NT_GRAPH_GENERATORS_H_

#include "../tools.h"
#include "../../core/random.h"
#include "../graph_element_set.h"

namespace nt {
  namespace graphs {

    /**
     * @brief Generate a random partial k-tree as described in [1]
     *
     * A partial k-tree is a graph that contains all the vertices and a subset of the edges of a k-tree.
     * G is a partial k-tree if and only if G has treewidth at most k.
     * This procedure can be used to generate random graphs of bounded treewidth.
     *
     * Example of usage :
     *
     * @code
     *  int main() {
     *    nt::graphs::ListGraph g;
     *
     *    // Generate a random partial 3-tree with 10 nodes
     *    nt::graphs::generatePartialKTree(g, 10, 3);
     *
     *   return 0;
     *}
     * @endcode
     *
     * [1] Vibhav Gogate and Rina Dechter
     *     A Complete Anytime Algorithm for Treewidth
     *     UAI 2004
     *
     * @tparam GR Type of the generated graph
     *
     * @param gr Generated graph
     * @param n Number of vertices to generate
     * @param k Desired treewidth upper bound
     * @param p Percent of edges to keep from the generated k-tree to obtain a partial k-tree
     * @param rnd an instance of Random
     */
    template < typename GR >
    void generatePartialKTree(GR& gr, int n, int k, double p = 0.6, const nt::Random& rnd = nt::Random()) noexcept {
      using Node = typename GR::Node;
      assert(n >= k + 1 && k >= 1);
      if (n < k + 1 || k < 1) return;

      // rnd(i_seed);

      const int i_clique_size = k + 1;

      // Generate a k + 1 clique
      for (int i = 0; i < i_clique_size; ++i)
        gr.addNode();

      for (int i = 0; i < i_clique_size; ++i) {
        for (int j = i + 1; j < i_clique_size; ++j) {
          const auto u = gr.nodeFromId(i);
          const auto v = gr.nodeFromId(j);
          gr.addEdge(u, v);
        }
      }

      // Add the remaining n - k - 1 vertices
      // Each vertex is made adjacent to a clique of size k selected uniformaly at random
      nt::IntDynamicArray             perm(n);
      nt::TrivialDynamicArray< Node > clique(k);

      for (int i = 0; i < n - i_clique_size; ++i) {
        // Shuffling step
        perm.removeAll();
        for (int j = 0; j < gr.nodeNum(); ++j)
          perm.add(j);
        for (int j = 1; j < gr.nodeNum(); ++j)
          std::swap(perm[j], perm[rnd.integer(j)]);

        const auto u = gr.addNode();
        clique.removeAll();
        int j = 0;
        while (clique.size() < k) {
          const auto v = gr.nodeFromId(perm[j++]);
          int        c = 0;
          for (; c < clique.size(); ++c)
            if (findArc(gr, v, clique[c]) == INVALID) break;
          if (c == clique.size()) clique.add(v);
        }

        for (const auto c: clique)
          gr.addEdge(u, c);
      }

      // Remove p percent arcs from the previous k-tree to obtain a partial k-tree
      const double i_ori_edge_num = gr.edgeNum();
      while (static_cast< double >(gr.edgeNum()) / i_ori_edge_num > p) {
        const auto e = gr.edgeFromId(rnd.integer(gr.edgeNum()));
        if (gr.valid(e)) gr.erase(e);
      }
    }

    /**
     * @brief Generate a directed path of n vertices.
     *
     * @tparam GR Type of the generated graph
     *
     * @param gr Generated graph
     * @param n Number of vertices to generate
     */
    template < typename GR >
    void directedPathGraph(GR& gr, const int n) {
      if (n < 1) return;

      typename GR::Node v = gr.addNode();
      for (int i = 1; i < n; ++i) {
        const typename GR::Node u = gr.addNode();
        gr.addArc(v, u);
        v = u;
      }
    }

    /**
     * @brief Generate a tournament of n vertices.
     *
     * @tparam GR Type of the generated graph
     *
     * @param gr Generated graph
     * @param n Number of vertices to generate
     */
    template < typename GR >
    void tournamentGraph(GR& gr, const int n) {
      for (int i = 0; i < n; ++i)
        gr.addNode();

      for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j)
          gr.addArc(i, j);
    }

    /**
     * \addtogroup generators
     * @{
     */

    /**
     * @brief Generates Erdős-Rényi random graph.
     *
     * This method generates a simple undirected random graph according to the
     * <a href="http://en.wikipedia.org/wiki/Erd%C5%91s%E2%80%93R%C3%A9nyi_model">
     * Erdős-Rényi model</a>.
     * A generated graph \f$G(n, p)\f$ contains \f$n\f$ nodes, and an edge
     * is set between each pair of nodes with equal probability \f$p\f$,
     * independently of the other edges.
     *
     * The time complexity of this method is \f$O(n^2)\f$.
     *
     * The global random generator instance \c rnd of LEMON is used
     * in this method.
     *
     * \retval gr an undirected graph structure. First, it is cleared, and
     * then the appropriate nodes and edges are added.
     * @param n the number of nodes to be generated.
     * @param p the probability of adding an individual edge.
     * @param rnd an instance of Random
     */
    template < typename GR >
    void generateErdosRenyiGraph(GR& gr, int n, double p, const nt::Random& rnd = nt::Random()) {
      TEMPLATE_GRAPH_TYPEDEFS(GR);
      gr.clear();
      for (int i = 0; i < n; i++) {
        gr.addNode();
      }
      for (NodeIt u(gr); u != INVALID; ++u) {
        for (NodeIt v(gr); v != INVALID; ++v) {
          if (u < v && rnd() < p) gr.addEdge(u, v);
        }
      }
    }

    /**
     * @brief Generates Erdős-Rényi random graph with expected average
     * node degree.
     *
     * This method generates a simple undirected random graph according to the
     * <a href="http://en.wikipedia.org/wiki/Erd%C5%91s%E2%80%93R%C3%A9nyi_model">
     * Erdős-Rényi model</a> with respect to an expected average node degree
     * \f$k\f$.
     * In fact, this method just calls @ref generateErdosRenyiGraph() with
     * \f$p = k/(n-1)\f$ parameter.
     *
     * \retval gr an undirected graph structure. First, it is cleared, and
     * then the appropriate nodes and edges are added.
     * @param n the number of nodes to be generated.
     * @param k the expected average degree of the nodes.
     */
    template < typename GR >
    void generateErdosRenyiGraphDeg(GR& gr, int n, double k, const nt::Random& rnd = nt::Random()) {
      generateErdosRenyiGraph(gr, n, (double)k / (n - 1), rnd);
    }

    /**
     * @brief Generates Barabási-Albert random graph.
     *
     * This method generates a simple undirected random graph according to the
     * <a href="http://en.wikipedia.org/wiki/Barab%C3%A1si%E2%80%93Albert_model">
     * Barabás-Albert model</a> with exact average node degree.
     *
     * The algorithm has two mandatory parameters: \f$n\f$ is the total number
     * of nodes, and \f$k\f$ is the desired average node degree
     * (it should be even).
     * An optional parameter \f$\lambda\f$ can also be given to adjust
     * attachment preference calculation (it is 0 by default).
     * The algorithm begins with a clique containing \f$k+1\f$ nodes.
     * Additional nodes are added one-by-one. Each new node is
     * connected to \f$r = k/2\f$ existing nodes.
     * The probability of connecting a new node to a pre-existing node \f$i\f$ is
     * proportional to \f$k_i+\lambda\f$, where \f$k_i\f$ denotes
     * the current degree of node \f$i\f$.
     * Note that this method ensures that the average node degree will be
     * exactly \f$k\f$.
     *
     * The expected time complexity of this method is \f$O(n(k+\lambda))\f$.
     *
     * The global random generator instance \c rnd of LEMON is used
     * in this method.
     *
     * \retval gr an undirected graph structure. First, it is cleared, and
     * then the appropriate nodes and edges are added.
     * @param n the number of nodes to be generated.
     * It should be at least \f$k+1\f$.
     * @param k the average node degree for the generated graph.
     * It should be an even integer.
     * @param lambda parameter for adjusting attachment preference calculation.
     * It may be negative, but it should be greater than \f$-k/2\f$.
     */
    template < typename GR >
    void generateBarabasiAlbertGraph(GR& gr, int n, int k, int lambda = 0, const nt::Random& rnd = nt::Random()) {
      TEMPLATE_GRAPH_TYPEDEFS(GR);
      if (k < 0) k = 0;
      int r = k / 2;
      k = r * 2;
      if (n < k + 1) n = k + 1;

      gr.clear();
      std::vector< Node > node(n);
      for (int i = 0; i != n; i++) {
        node[i] = gr.addNode();
      }
      for (int i = 0; i != k; i++) {
        for (int j = i + 1; j != k + 1; j++) {
          gr.addEdge(node[i], node[j]);
        }
      }
      std::vector< int > node_select;
      for (int i = 0; i != k + 1; i++) {
        for (int c = 0; c < k + lambda; c++) {
          node_select.push_back(i);
        }
      }

      DynArcLookUp< GR > lookup(gr);
      for (int i = k + 1; i != n; i++) {
        int d = 0;
        while (d < r) {
          int j = node_select[rnd[node_select.size()]];
          if (lookup(node[i], node[j]) == INVALID) {
            gr.addEdge(node[i], node[j]);
            node_select.push_back(j);
            d++;
          }
        }
        for (int c = 0; c < r + lambda; c++) {
          node_select.push_back(i);
        }
      }
    }

    /**
     * @brief Adds edges to a graph using Barabási-Albert method.
     *
     * This method extends a simple undirected graph by adding edges
     * according to the
     * <a href="http://en.wikipedia.org/wiki/Barab%C3%A1si%E2%80%93Albert_model">
     * Barabási-Albert model</a>.
     *
     * The input can be an arbitrary undirected graph, it need not be a
     * Barabási-Albert graph. The algorithm extends this graph with additional
     * edges applying the attachment preference rule of the Barabási-Albert
     * model. That is, both nodes of each newly added edge is selected
     * with a probability proportional to \f$k_i+\lambda\f$, where \f$k_i\f$
     * denotes the current degree of the node.
     * Loop and parallel edges are not introduced, and new nodes are not added
     * to the graph.
     *
     * This method can be used, for example, in a two-phase generation
     * procedure as follows:
     * <pre>
     * generateBarabasiAlbertGraph(gr, n, k);
     * extendBarabasiAlbertGraph(gr, m);
     * </pre>
     *
     * \retval gr an undirected graph structure.
     * @param m the desired number of edges. New edges are added to \c gr until
     * this limit is reached.
     * @param lambda parameter for adjusting attachment preference calculation.
     * @param edge_try_limit the maximum number of attempts to add a new edge.
     * If this limit is exceeded, the process is terminated with \c false
     * return value.
     * @return \c true if and only if the generation was successful.
     *
     * @see generateBarabasiAlbertGraph()
     */
    template < typename GR >
    bool extendBarabasiAlbertGraph(
       GR& gr, int m, int lambda = 0, int edge_try_limit = 1000, const nt::Random& rnd = nt::Random()) {
      TEMPLATE_GRAPH_TYPEDEFS(GR);

      std::vector< Node > node;
      std::vector< int >  node_select;
      for (NodeIt u(gr); u != INVALID; ++u) {
        const int i = node.size();
        const int score = countIncEdges(gr, u) + lambda;
        node.push_back(u);
        for (int c = 0; c < score; c++) {
          node_select.push_back(i);
        }
      }

      DynArcLookUp< GR > lookup(gr);
      int                try_cnt = 0;
      for (int cnt = countEdges(gr); cnt < m;) {
        int i = node_select[rnd[node_select.size()]];
        int j = node_select[rnd[node_select.size()]];
        if (i != j && lookup(node[i], node[j]) == INVALID) {
          gr.addEdge(node[i], node[j]);
          node_select.push_back(i);
          node_select.push_back(j);
          cnt++;
          try_cnt = 0;
        } else {
          if (++try_cnt >= edge_try_limit) return false;
        }
      }

      return true;
    }

    /**
     * @brief Generates a random graph with a given degree sequence.
     *
     * This method generates an undirected random graph with approximately
     * uniform distribution for a given degree sequence.
     *
     * <i>For the details of the applied algorithm, please refer to
     * the source code.</i>
     *
     * \retval gr an undirected graph structure. First, it is cleared, and
     * then the appropriate nodes and edges are added.
     * @param deg the degree sequence.
     * @param parallel indicates if parallel edges are allowed.
     * @param loop indicates if loops are allowed.
     * @param edge_try_limit the maximum number of attempts to add a new edge.
     * If this limit is exceeded, the generation is restarted.
     * @param restart_limit the maximum number of restarting the generation
     * process. If this limit is exceeded, the method terminates with
     * \c false return value.
     * @return \c true if and only if the generation was successful.
     * @pre The degree sequence must be feasible!
     */
    template < typename GR >
    bool generateGraphForDegreeSeq1(GR&                       gr,
                                    const std::vector< int >& deg,
                                    bool                      parallel = false,
                                    bool                      loop = false,
                                    int                       edge_try_limit = 1000,
                                    int                       restart_limit = 10,
                                    const nt::Random&         rnd = nt::Random()) {
      TEMPLATE_GRAPH_TYPEDEFS(GR);

      int n = deg.size();
      if (n < 2) return false;

      int m = 0;
      for (int i = 0; i != n; i++)
        m += deg[i];
      if (m % 2 != 0) return false;
      m /= 2;

      std::vector< Node > node(n);
      std::vector< int >  node_select;
      DynArcLookUp< GR >* lookup = NULL;

      int start_cnt = 0;

    restart_generation:

      // Add nodes and initialize the data structures
      gr.clear();
      if (lookup) delete lookup;
      lookup = new DynArcLookUp< GR >(gr);

      node_select.resize(2 * m);
      int k = 0;
      for (int i = 0; i != n; i++) {
        node[i] = gr.addNode();
        for (int j = 0; j != deg[i]; j++)
          node_select[k++] = i;
      }

      // Add edges
      int last = node_select.size() - 1;
      for (int a = 0; a < m; a++) {
        int try_cnt = 0;
        while (true) {
          int i = rnd[last + 1];
          int j = rnd[last + 1];
          int u = node_select[i];
          int v = node_select[j];
          if (i != j && (loop || u != v) && (parallel || (*lookup)(node[u], node[v]) == INVALID)) {
            gr.addEdge(node[u], node[v]);
            if (i < last - 1 && j < last - 1) {
              node_select[i] = node_select[last];
              node_select[j] = node_select[last - 1];
            } else if (i >= last - 1 && j < last - 1) {
              node_select[j] = i == last ? node_select[last - 1] : node_select[last];
            } else if (i < last - 1 && j >= last - 1) {
              node_select[i] = j == last ? node_select[last - 1] : node_select[last];
            }
            last -= 2;
            break;
          } else {
            if (++try_cnt >= edge_try_limit) {
              if (++start_cnt <= restart_limit) {
                goto restart_generation;
              } else {
                if (lookup) delete lookup;
                return false;
              }
            }
          }
        }
      }

      if (lookup) delete lookup;
      return true;
    }

    /**
     * @brief Generates a random graph with a given degree sequence.
     *
     * This method generates an undirected random graph with approximately
     * uniform distribution for a given degree sequence.
     *
     * <i>For the details of the applied algorithm, please refer to
     * the source code.</i>
     *
     * \retval gr an undirected graph structure. First, it is cleared, and
     * then the appropriate nodes and edges are added.
     * @param deg the degree sequence.
     * @param parallel indicates if parallel edges are allowed.
     * @param loop indicates if loops are allowed.
     * @param edge_try_limit the maximum number of attempts to add a new edge.
     * If this limit is exceeded, the generation is restarted.
     * @param restart_limit the maximum number of restarting the generation
     * process. If this limit is exceeded, the method terminates with
     * \c false return value.
     * @return \c true if and only if the generation was successful.
     * @pre The degree sequence must be feasible!
     */
    template < typename GR >
    bool generateGraphForDegreeSeq2(GR&                       gr,
                                    const std::vector< int >& deg,
                                    bool                      parallel = false,
                                    bool                      loop = false,
                                    int                       edge_try_limit = 1000,
                                    int                       restart_limit = 10,
                                    const nt::Random&         rnd = nt::Random()) {
      TEMPLATE_GRAPH_TYPEDEFS(GR);

      int n = deg.size();
      if (n < 2) return false;
      std::vector< int > sorted_deg(deg);
      std::sort(sorted_deg.begin(), sorted_deg.end());

      int m = 0;
      for (int i = 0; i != n; i++)
        m += sorted_deg[i];
      if (m % 2 != 0) return false;
      m /= 2;

      std::vector< Node > node(n);
      std::vector< int >  node_select;
      DynArcLookUp< GR >* lookup = NULL;

      int start_cnt = 0;

    restart_generation:

      // Add nodes and initialize the data structures
      gr.clear();
      if (lookup) delete lookup;
      lookup = new DynArcLookUp< GR >(gr);

      node_select.resize(2 * m);
      int k = 0;
      for (int i = 0; i != n; i++) {
        node[i] = gr.addNode();
        for (int j = 0; j != sorted_deg[i]; j++)
          node_select[k++] = i;
      }

      // Add edges
      int last = node_select.size() - 1;
      for (int a = 0; a < m; a++) {
        int try_cnt = 0;
        while (true) {
          int i = rnd[last];
          int u = node_select[i];
          int v = node_select[last];
          if ((loop || u != v) && (parallel || (*lookup)(node[u], node[v]) == INVALID)) {
            gr.addEdge(node[u], node[v]);
            node_select[i] = node_select[last - 1];
            last -= 2;
            break;
          } else {
            if (++try_cnt >= edge_try_limit) {
              if (++start_cnt <= restart_limit) {
                goto restart_generation;
              } else {
                if (lookup) delete lookup;
                return false;
              }
            }
          }
        }
      }

      if (lookup) delete lookup;
      return true;
    }

    /**
     * @brief Generates a random graph with a given degree sequence.
     *
     * This method generates an undirected random graph with approximately
     * uniform distribution for a given degree sequence.
     *
     * <i>For the details of the applied algorithm, please refer to
     * the source code.</i>
     *
     * \retval gr an undirected graph structure. First, it is cleared, and
     * then the appropriate nodes and edges are added.
     * @param deg the degree sequence.
     * @param parallel indicates if parallel edges are allowed.
     * @param loop indicates if loops are allowed.
     * @param edge_try_limit the maximum number of attempts to add a new edge.
     * If this limit is exceeded, the generation is restarted.
     * @param restart_limit the maximum number of restarting the generation
     * process. If this limit is exceeded, the method terminates with
     * \c false return value.
     * @return \c true if and only if the generation was successful.
     * @pre The degree sequence must be feasible!
     */
    template < typename GR >
    bool generateGraphForDegreeSeq3(GR&                       gr,
                                    const std::vector< int >& deg,
                                    bool                      parallel = false,
                                    bool                      loop = false,
                                    int                       edge_try_limit = 1000,
                                    int                       restart_limit = 10,
                                    const nt::Random&         rnd = nt::Random()) {
      TEMPLATE_GRAPH_TYPEDEFS(GR);

      int n = deg.size();
      if (n < 2) return false;

      int m = 0;
      for (int i = 0; i != n; i++)
        m += deg[i];
      if (m % 2 != 0) return false;
      m /= 2;

      std::vector< Node > node(n);
      std::vector< int >  node_select;
      DynArcLookUp< GR >* lookup = NULL;

      int start_cnt = 0;

    restart_generation:

      // Add nodes and initialize the data structures
      gr.clear();
      if (lookup) delete lookup;
      lookup = new DynArcLookUp< GR >(gr);

      node_select.resize(2 * m);
      int k = 0;
      for (int i = 0; i != n; i++) {
        node[i] = gr.addNode();
        for (int j = 0; j != deg[i]; j++)
          node_select[k++] = i;
      }

      // Split the node_select vector
      for (int i = node_select.size() - 1; i >= 0; i--) {
        int j = rnd[i + 1];
        int tmp = node_select[i];
        node_select[i] = node_select[j];
        node_select[j] = tmp;
      }
      std::sort(node_select.begin(), node_select.begin() + m, std::greater< int >());

      // Add edges
      for (int a = 0; a < m; a++) {
        int try_cnt = 0;
        while (true) {
          int j = m + rnd[m - a];
          int u = node_select[a];
          int v = node_select[j];
          if ((loop || u != v) && (parallel || (*lookup)(node[u], node[v]) == INVALID)) {
            gr.addEdge(node[u], node[v]);
            node_select[j] = node_select[2 * m - a - 1];
            break;
          } else {
            if (++try_cnt >= edge_try_limit) {
              if (++start_cnt <= restart_limit) {
                goto restart_generation;
              } else {
                if (lookup) delete lookup;
                return false;
              }
            }
          }
        }
      }

      if (lookup) delete lookup;
      return true;
    }

  }   // namespace graphs
}   // namespace nt

#endif
