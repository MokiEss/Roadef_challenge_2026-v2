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
 * @brief Graph layout algorithms. 
 *
 * @author Morgan Chopin (morgan.chopin@orange.com),
 */

#include "../../@deps/demek/interface.hpp"
#include "../../core/algebra.h"

namespace nt {
  namespace graphs {

    namespace details {

      /**
       * @brief Computes and applies a normal vector shift to the control points of an edge.
       *
       * This function is used to offset the control points of an edge in a direction
       * perpendicular to the edge's segments. It is typically used to visually separate
       * multiple parallel edges in a graph layout.
       *
       * @tparam Digraph The graph type.
       * @tparam PosMap A node position map type (default: Digraph::NodeMap<nt::Vec2Dd>).
       * @tparam CpMap A control points map type (default: Digraph::ArcMap<nt::TrivialDynamicArray<nt::Vec2Dd>>).
       *
       * @param cp_map The map storing control points for each arc.
       * @param arc The arc whose control points are being shifted.
       * @param edge The edge path containing the original control points.
       * @param shift The amount by which to shift the control points along the normal.
       */
      template < typename Digraph,
                 typename PosMap = typename Digraph::template NodeMap< nt::Vec2Dd >,
                 typename CpMap = typename Digraph::template ArcMap< nt::TrivialDynamicArray< nt::Vec2Dd > > >
      inline void
         computeNormalShifting(CpMap& cp_map, const typename Digraph::Arc& arc, const path& edge, double shift) {
        size_t edge_points_size = edge.points.size();
        cp_map[arc].ensureAndFill(edge_points_size);
        for (int i = 1; i < edge_points_size; ++i) {
          // Compute normal vec for shifting
          const vec2 cp_a = edge.points[i - 1];
          const vec2 cp_b = edge.points[i];
          nt::Vec2Df normal({-(cp_b.x - cp_a.x), cp_b.y - cp_a.y});
          normal.normalize();

          normal *= shift;

          if (i == 1) {
            cp_map[arc][0].copyFrom(nt::ArrayView({(edge.points[0].y + normal[0]), (edge.points[0].x + normal[1])}));
          }
          cp_map[arc][i].copyFrom(nt::ArrayView({(edge.points[i].y + normal[0]), (edge.points[i].x + normal[1])}));
        }
      }
    }   // namespace details


    /**
     * @brief Computes a Sugiyama layered layout for a directed acyclic graph.
     *
     * This function builds a layered graph layout using the Sugiyama algorithm.
     * It maps node positions and computes control points for arcs, handling
     * multiple arcs between the same node pairs by applying normal shifts.
     *
     * @tparam Digraph The graph type.
     * @tparam PosMap A node position map type (default: Digraph::NodeMap<nt::Vec2Dd>).
     * @tparam CpMap A control points map type (default: Digraph::ArcMap<nt::TrivialDynamicArray<nt::Vec2Dd>>).
     * @tparam DirMap A direction map type for arcs (default: Digraph::ArcMap<char>).
     *
     * @param g The input directed graph.
     * @param pos_map Output map storing 2D positions for each node.
     * @param cp_map Output map storing control points for each arc.
     * @param dir_map Output map storing direction indicators for arcs (1 for forward, -1 for backward).
     */
    template < typename Digraph,
               typename PosMap = typename Digraph::template NodeMap< nt::Vec2Dd >,
               typename CpMap = typename Digraph::template ArcMap< nt::TrivialDynamicArray< nt::Vec2Dd > >,
               typename DirMap = typename Digraph::template ArcMap< char > >
    void sugiyamaLayout(const Digraph& g, PosMap& pos_map, CpMap& cp_map, DirMap& dir_map) {
      assert(g.nodeNum());
      graph                                                demek_g;
      typename Digraph::template StaticNodeMap< vertex_t > nt_2_demek(g);
      nt::TrivialDynamicArray< typename Digraph::Node >    demek_2_nt(g.nodeNum());
      nt::graphs::NodeSet< Digraph >                       visited(g);

      for (typename Digraph::NodeIt n(g); n != nt::INVALID; ++n) {
        const vertex_t u = demek_g.add_node();
        nt_2_demek[n] = u;
        demek_2_nt[u] = n;
      }

      for (int i = 0; i < g.nodeNum(); ++i) {
        const typename Digraph::Node source = g.nodeFromId(i);
        visited.clear();
        for (typename Digraph::OutArcIt out_arc(g, source); out_arc != nt::INVALID; ++out_arc) {
          const typename Digraph::Node target = g.target(out_arc);
          if (g.id(target) < i || visited.contains(target)) continue;
          demek_g.add_edge(nt_2_demek[source], nt_2_demek[target]);
          visited.insert(target);
        }
      }

      assert(demek_g.size() == g.nodeNum());

      attributes attr;
      attr.node_size = 1;
      attr.node_dist = 20;
      attr.layer_dist = 30;

      sugiyama_layout l(demek_g, attr);

      for (typename Digraph::NodeIt n(g); n != nt::INVALID; ++n)
        const vertex_t u = nt_2_demek[n];

      l.normalize();

      for (const auto& node: l.vertices()) {
        const typename Digraph::Node n = demek_2_nt[node.u];
        pos_map[n].copyFrom(nt::ArrayView({node.pos.y, node.pos.x}));
      }

      for (auto& edge: l.edges()) {
        assert(edge.points.size() >= 2);
        const typename Digraph::Node u = demek_2_nt[edge.from];
        const typename Digraph::Node v = demek_2_nt[edge.to];

        float shift = 0.f;
        for (typename Digraph::Arc arc = nt::graphs::findArc(g, u, v); arc != nt::INVALID;
             arc = nt::graphs::findArc(g, u, v, arc)) {
          details::computeNormalShifting< Digraph >(cp_map, arc, edge, shift);
          dir_map[arc] = 1;
          shift += 0.01f;
        }

        shift = -.01f;
        for (typename Digraph::Arc arc = nt::graphs::findArc(g, v, u); arc != nt::INVALID;
             arc = nt::graphs::findArc(g, v, u, arc)) {
          details::computeNormalShifting< Digraph >(cp_map, arc, edge, shift);
          dir_map[arc] = -1;
          shift -= 0.01f;
        }
      }
    }

    /**
     * @brief Computes a force-directed layout for a graph using a spring-embedder model.
     *
     * This function simulates physical forces (repulsion and attraction) between nodes
     * and edges to compute aesthetically pleasing node positions. It normalizes the
     * final layout to fit within a unit square.
     *
     * @tparam Digraph The graph type.
     * @tparam PosMap A node position map type (default: Digraph::NodeMap<nt::Vec2Dd>).
     *
     * @param g The input graph.
     * @param pos_map Output map storing 2D positions for each node.
     * @param niter Number of iterations to run the simulation (default: 500).
     * @param start_temp Initial temperature for the simulation (default: sqrt(node count)).
     */
    template < typename Digraph, typename PosMap = typename Digraph::template NodeMap< nt::Vec2Dd > >
    void forceDirectedLayout(const Digraph& g, PosMap& pos_map, int niter = 500, double start_temp = 0) {
      assert(g.nodeNum());
      nt::Random rnd;

      const int vcount = g.nodeNum();
      const int ecount = g.arcNum();
      if (start_temp <= 0) start_temp = std::sqrt(vcount);
      nt::DoubleDynamicArray dispx(vcount, true), dispy(vcount, true);
      double                 temp = start_temp;
      double                 difftemp = start_temp / niter;
      double                 C = 0;

      const double width = std::sqrt(vcount), height = width;
      double       dminx = -width / 2, dmaxx = width / 2, dminy = -height / 2, dmaxy = height / 2;

      for (int i = 0; i < vcount; ++i) {
        pos_map[g.nodeFromId(i)][0] = rnd(dminx, dmaxx);
        pos_map[g.nodeFromId(i)][1] = rnd(dminy, dmaxy);
      }

      for (int i = 0; i < niter; ++i) {
        dispx.fillBytes(0);
        dispy.fillBytes(0);

        // compute repulsive forces
        for (int v = 0; v < vcount; ++v) {
          for (int u = v + 1; u < vcount; ++u) {
            double dx = pos_map[g.nodeFromId(v)][0] - pos_map[g.nodeFromId(u)][0];
            double dy = pos_map[g.nodeFromId(v)][1] - pos_map[g.nodeFromId(u)][1];
            double dlen = dx * dx + dy * dy;

            while (dlen == 0) {
              dx = rnd(-1e-9, 1e-9);
              dy = rnd(-1e-9, 1e-9);
              dlen = dx * dx + dy * dy;
            }

            dispx[v] += dx / dlen;
            dispy[v] += dy / dlen;
            dispx[u] -= dx / dlen;
            dispy[u] -= dy / dlen;
          }
        }

        // compute attractive forces
        for (int e = 0; e < ecount; ++e) {
          int    v = g.id(g.source(g.arcFromId(e)));
          int    u = g.id(g.target(g.arcFromId(e)));
          double dx = pos_map[g.nodeFromId(v)][0] - pos_map[g.nodeFromId(u)][0];
          double dy = pos_map[g.nodeFromId(v)][1] - pos_map[g.nodeFromId(u)][1];
          double dlen = std::sqrt(dx * dx + dy * dy);
          dispx[v] -= (dx * dlen);
          dispy[v] -= (dy * dlen);
          dispx[u] += (dx * dlen);
          dispy[u] += (dy * dlen);
        }

        for (int v = 0; v < vcount; ++v) {
          double dx = dispx[v] + rnd(-1e-9, 1e-9);
          double dy = dispy[v] + rnd(-1e-9, 1e-9);
          double displen = std::sqrt(dx * dx + dy * dy);

          if (displen > temp) {
            dx *= temp / displen;
            dy *= temp / displen;
          }

          if (displen > 0) {
            pos_map[g.nodeFromId(v)][0] += dx;
            pos_map[g.nodeFromId(v)][1] += dy;
          }
        }

        temp -= difftemp;
      }

      // Normalize
      dminx = pos_map[g.nodeFromId(0)][0];
      dmaxx = dminx;
      dminy = pos_map[g.nodeFromId(0)][1];
      dmaxy = dminy;

      for (int v = 1; v < vcount; ++v) {
        const double x = pos_map[g.nodeFromId(v)][0];
        if (dminx > x) dminx = x;
        if (dmaxx < x) dmaxx = x;

        const double y = pos_map[g.nodeFromId(v)][1];
        if (dminy > y) dminy = y;
        if (dmaxy < y) dmaxy = y;
      }

      const double dx = std::abs(dmaxx - dminx);
      const double dy = std::abs(dminy - dmaxy);

      for (int v = 0; v < vcount; ++v) {
        double& x = pos_map[g.nodeFromId(v)][0];
        x = (x - dminx) / dx;

        double& y = pos_map[g.nodeFromId(v)][1];
        y = (y - dminy) / dy;
      }
    }
  }   // namespace graphs
}   // namespace nt
