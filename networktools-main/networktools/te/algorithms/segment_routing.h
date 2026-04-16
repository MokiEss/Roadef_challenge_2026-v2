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

#ifndef _NT_SEGMENT_ROUTING_H_
#define _NT_SEGMENT_ROUTING_H_

#include "../../core/format.h"
#include "../../core/queue.h"

#include "spr.h"
#include "dynamic_spr.h"

namespace nt {
  namespace te {

    /**
     * @class SrPath
     * @headerfile segment_routing.h
     * @brief A class representing a Segment Routing path (or SR-path) in a network G = (V, A).
     * An SR-path is a sequence ⟨s, w1, ..., wℓ, t⟩ of segments where s ∈ V is the source of the
     * segment path and t ∈ V the target. Each segment wi is either a node or an arc of G.
     * The terminology is based on [1].
     *
     * [1] François Aubry:
     *     Models and Algorithms for Network Optimization with Segment Routing
     *     PhD Thesis, Catholic University of Louvain, (2020)
     *
     * Example of usage :
     *
     * @code
     * #include "networktools.h"
     *
     * int main() {
     *   using Digraph = nt::graphs::SmartDigraph;
     *   DIGRAPH_TYPEDEFS(Digraph);
     *
     *   Digraph digraph;
     *
     *   Node v0 = digraph.addNode();
     *   Node v1 = digraph.addNode();
     *   Node v2 = digraph.addNode();
     *   Node v3 = digraph.addNode();
     *   Arc  a0 = digraph.addArc(v1, v0);
     *   Arc  a1 = digraph.addArc(v1, v2);
     *   Arc  a2 = digraph.addArc(v2, v3);
     *   Arc  a3 = digraph.addArc(v0, v3);
     *
     *   // Create an SrPath composed of two nodes
     *   nt::te::SrPath< Digraph > sr_path;
     *   sr_path.addSegment(v1);
     *   sr_path.addSegment(v3);
     *
     *   std::cout << "SR Path: " << sr_path.toString(digraph) << std::endl;
     *   std::cout << "Number of segments: " << sr_path.segmentNum() << std::endl;
     *
     *   for (int i = 0; i < sr_path.segmentNum(); ++i) {
     *     const nt::te::SrPath< Digraph >::Segment seg = sr_path[i];
     *     if (seg.isArc()) {
     *       std::cout << "Segment " << i << " is an Arc: " << digraph.id(seg.toArc()) << std::endl;
     *     } else {
     *       std::cout << "Segment " << i << " is a Node: " << digraph.id(seg.toNode()) << std::endl;
     *     }
     *   }
     *
     *   sr_path.reverse();
     *   std::cout << "Reversed SR Path: " << sr_path.toString(digraph) << std::endl;
     *
     *   return 0;
     * }
     * @endcode
     *
     * @tparam GR The type of the directed graph representing the network.
     * @tparam SSO The size of the pre-allocated memory on the stack.
     *
     */
    template < typename GR, int SSO = 3 >
    struct SrPath {
      using Digraph = GR;
      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

      /**
       * @brief A struct representing a segment in the SR path. A segment can be either a node or an arc.
       *
       */
      struct Segment {
        int _id;

        Segment() = default;

        /**
         * @brief Construct a new Segment object with a given id.
         *
         * @param id The identifier of the segment.
         */
        constexpr Segment(int id) noexcept : _id(id) {}

        /**
         * @brief Construct a new Segment object from an arc.
         *
         * @param arc The arc to be converted to a segment.
         */
        constexpr Segment(Arc arc) noexcept : _id(~Digraph::id(arc)) {}

        /**
         * @brief Construct a new Segment object from a node.
         *
         * @param node The node to be converted to a segment.
         */
        constexpr Segment(Node node) noexcept : _id(Digraph::id(node)) {}

        /**
         * @brief Construct a new Segment object representing an invalid segment.
         */
        constexpr Segment(nt::Invalid) noexcept : _id(NT_INT32_MAX) {}

        /**
         * @brief Equality operator for Segment.
         *
         * @param seg The segment to compare with.
         * @return True if segments are equal, otherwise false.
         */
        constexpr bool operator==(const Segment& seg) const noexcept { return _id == seg._id; }

        /**
         * @brief Inequality operator for Segment.
         *
         * @param seg The segment to compare with.
         * @return True if segments are not equal, otherwise false.
         */
        constexpr bool operator!=(const Segment& seg) const noexcept { return _id != seg._id; }

        /**
         * @brief Check if the segment represents an arc.
         *
         * @return True if the segment is an arc, otherwise false.
         */
        constexpr bool isArc() const noexcept { return _id < 0; }

        /**
         * @brief Check if the segment represents a node.
         *
         * @return True if the segment is a node, otherwise false.
         */
        constexpr bool isNode() const noexcept { return !isArc(); }

        /**
         * @brief Convert the segment to an arc.
         *
         * @return The corresponding arc.
         */
        constexpr Arc toArc() const noexcept {
          assert(isArc());
          return Digraph::arcFromId(~_id);
        }

        /**
         * @brief Convert the segment to a node.
         *
         * @return The corresponding node.
         */
        constexpr Node toNode() const noexcept {
          assert(!isArc());
          return Digraph::nodeFromId(_id);
        }

        /**
         * @brief Get the source node of a segment. If the segment is an arc, the function
         *  returns the source of that arc. If the segment is a node, the function returns that node.
         *
         * @param g The graph that contains the segment.
         *
         * @return The source node.
         */
        constexpr Node source(const Digraph& g) const noexcept {
          if (isArc()) return g.source(toArc());
          return toNode();
        }

        /**
         * @brief Get the target node of a segment. If the segment is an arc, the function
         *  returns the target of that arc. If the segment is a node, the function returns that node.
         *
         * @param g The graph that contains the segment.
         *
         * @return The target node.
         */
        constexpr Node target(const Digraph& g) const noexcept {
          if (isArc()) return g.target(toArc());
          return toNode();
        }
      };

      /**
       * @brief A struct representing an iterator over the segments of a SR path.
       */
      struct SegmentIt: public Segment {
        using Parent = Segment;

        int           _k;   //< The index of the current segment in the SR path.
        const SrPath& _sr_path;

        explicit SegmentIt(const SrPath& sr_path) : _sr_path(sr_path) {
          if (!_segments.size()) {
            static_cast< Segment& >(*this) = Segment(nt::INVALID);
          } else {
            _k = _segments.size() - 1;
            Parent::_id = _sr_path._segments[_k]._id;
          }
        }
        SegmentIt& operator++() {
          if (!_k) {
            static_cast< Segment& >(*this) = Segment(nt::INVALID);
          } else {
            --_k;
            Parent::_id = _sr_path._segments[_k]._id;
          }
          return *this;
        }
      };

      nt::TrivialDynamicArray< Segment, SSO > _segments;   //< Array of segments in the SR path.

      SrPath() = default;
      constexpr SrPath(std::initializer_list< Segment > l) noexcept : _segments(l) {}
      SrPath(SrPath&& other) noexcept : _segments(std::move(other._segments)) {}

      SrPath& operator=(SrPath&& other) noexcept {
        _segments = std::move(other._segments);
        return *this;
      }

      /**
       * @brief Copy
       *
       */
      constexpr void copyFrom(const SrPath& other) noexcept { _segments.copyFrom(other._segments); }

      /**
       * @brief Reserve memory for a given number of segments.
       *
       * @param i_num_segments The number of segments to reserve memory for.
       */
      void reserve(int i_num_segments) noexcept { _segments.ensure(i_num_segments); }

      /**
       * @brief Add an arc as a segment to the SR path.
       *
       * @param arc The arc to be added as a segment.
       */
      void addSegment(Arc arc) noexcept { _segments.add(arc); }

      /**
       * @brief Add a node as a segment to the SR path.
       *
       * @param node The node to be added as a segment.
       */
      void addSegment(Node node) noexcept { _segments.add(node); }

      /**
       * @brief Get the number of segments in the SR path.
       *
       * @return The number of segments.
       */
      constexpr int segmentNum() const noexcept { return _segments._i_count; }

      /**
       * @brief Reverse the order of segments in the SR path.
       */
      void reverse() noexcept {
        for (int k = 0; k < (_segments._i_count >> 1); ++k)
          std::swap(_segments[k], _segments[_segments._i_count - k - 1]);
      }

      /**
       * @brief Check whether the SR path is empty
       *
       * @return true if the segment path is empty, false otherwise
       */
      constexpr bool empty() const noexcept { return segmentNum() == 0; }

      /**
       * @brief Get the first segment in the SR path.
       *
       * @return The first segment.
       */
      constexpr const Segment& front() const noexcept { return _segments.front(); }

      /**
       * @brief Get the last segment in the SR path.
       *
       * @return The last segment.
       */
      constexpr const Segment& back() const noexcept { return _segments.back(); }


      /**
       * @brief Remove all segments in the SR path.
       */
      constexpr void clear() noexcept { _segments.removeAll(); }

      /**
       * @brief Access the segment at a given index.
       *
       * @param k The index of the segment.
       * @return Reference to the segment.
       */
      Segment& operator[](int k) noexcept { return _segments[k]; }

      /**
       * @brief Access the segment at a given index (const version).
       *
       * @param k The index of the segment.
       * @return Reference to the segment.
       */
      const Segment& operator[](int k) const noexcept { return _segments[k]; }

      /**
       * @brief Check whether an SR-path is valid with respect to a given demand d. An SR-path is valid w.r.t d if
       * the source of the first segment is the same as the demand origin and the target of the last segment is
       * equal to the demand destination.
       *
       * @param g The graph that contains the SR-path.
       * @param dg The demand graph to which the demand belong to
       * @param demand_arc A demand arc
       *
       * @return True if the SR-path is valid w.r.t to the demand, false otherwise.
       */
      template < typename DemandGraph >
      constexpr bool
         isValid(const Digraph& g, const DemandGraph& dg, const typename DemandGraph::Arc demand_arc) const noexcept {
        if (empty()) return false;

        const Segment& first = front();
        const Segment& last = back();

        const Node source = first.source(g);
        const Node target = last.target(g);

        return dg.source(demand_arc) == source && dg.target(demand_arc) == target;
      }

      /**
       * @brief Function that iterates over the segments of an SR-path and apply callbacks along the way
       *
       * @param g The graph that contains the SR-path.
       * @param onConsecutiveSegments Function to be called between two consecutive segments
       * @param onVisitArc Function to be called on each arc-segment
       *
       * @return Status of the iteration operation
       */
      template < typename ConsecutiveSegmentsVisitor, typename ArcVisitor >
      bool forEachSegment(const Digraph&             g,
                          ConsecutiveSegmentsVisitor onConsecutiveSegments,
                          ArcVisitor                 onVisitArc) const noexcept {
        if (empty()) return true;

        for (int i_seg = 1; i_seg < segmentNum(); ++i_seg) {
          const Segment& prev_seg = _segments[i_seg - 1];
          const Node     source = prev_seg.target(g);
          const Segment& curr_seg = _segments[i_seg];
          const Node     target = curr_seg.source(g);

          if (source != target && !onConsecutiveSegments(source, target)) return false;

          if (prev_seg.isArc()) onVisitArc(prev_seg.toArc());
        }

        const Segment& last_seg = back();
        if (last_seg.isArc()) onVisitArc(last_seg.toArc());

        return true;
      }

      /**
       * @brief Convert an SR path to its string representation.
       *
       * @param g The graph that contains the SR path.
       * @return A string representation of the SR path.
       */
      nt::String toString(const Digraph& g) const {
        nt::MemoryBuffer buffer;
        nt::format_to(std::back_inserter(buffer), "<");
        for (int i_seg = 0; i_seg < segmentNum(); ++i_seg) {
          const Segment& seg = _segments[i_seg];
          if (seg.isArc()) {
            const Arc arc = seg.toArc();
            nt::format_to(
               std::back_inserter(buffer), "{} -> {} ({})", g.id(g.source(arc)), g.id(g.target(arc)), g.id(arc));
          } else {
            const Node node = seg.toNode();
            nt::format_to(std::back_inserter(buffer), "{}", g.id(node));
          }
          if (i_seg < segmentNum() - 1) nt::format_to(std::back_inserter(buffer), ",");
        }
        nt::format_to(std::back_inserter(buffer), ">");
        return nt::to_string(buffer);
      }
    };

    /**
     * @brief Array that contains SR paths
     *
     * @tparam GR The type of the directed graph representing the network.
     */
    template < typename GR >
    using SrPathArray = nt::TrivialDynamicArray< SrPath< GR > >;

    /**
     * @brief Preprocessing algorithms for segment routing optimization
     *
     * This class provides offline preprocessing methods for segment routing,
     * such as computing non-dominated segment sets and other analysis tasks
     * that are performed before actual routing operations.
     *
     * @tparam RoutingProtocol Segment routing class (e.g., SegmentRouting, DynamicSegmentRouting)
     */
    template < typename RoutingProtocol >
    struct SegmentRoutingPreprocessing {
      using Digraph = typename RoutingProtocol::Digraph;
      TEMPLATE_DIGRAPH_TYPEDEFS_EX(Digraph);

      using Segment = typename RoutingProtocol::Segment;
      using FlowValue = typename RoutingProtocol::FlowValue;
      using FlowArcMap = typename Digraph::template StaticArcMap< FlowValue >;

      struct SrPathInfo {
        FlowArcMap                _ratio_map;
        graphs::ArcSet< Digraph > _arcs;
        SrPathInfo(const Digraph& g) : _ratio_map(g), _arcs(g) { _ratio_map.setBitsToZero(); }

        constexpr void insertArc(const Arc arc, const FlowValue f_split) noexcept {
          _ratio_map[arc] += f_split;
          _arcs.insert(arc);
        }
        constexpr int                              arcNum() const noexcept { return _arcs.size(); }
        constexpr const graphs::ArcSet< Digraph >& arcs() const noexcept { return _arcs; }
        constexpr const FlowValue                  ratio(const Arc arc) const noexcept { return _ratio_map[arc]; }
      };

      RoutingProtocol&           _routing_protocol;
      DynamicArray< SrPathInfo > _srpath_infos;

      /**
       * @brief Constructor
       *
       * @param rp Reference to the routing protocol instance
       */
      explicit SegmentRoutingPreprocessing(RoutingProtocol& rp) : _routing_protocol(rp) {}

      /**
       * @brief Return the set of non-dominated node segments between source and target based on [1]
       *
       * [1] Hugo Callebaut, Jérôme De Boeck, Bernard Fortz:
       *     Preprocessing for segment routing optimization.
       *     Networks 82(4): 459-478 (2023)
       *
       * @param source Source of the demand
       * @param target Target of the demand
       *
       * @return An array of non-dominated node segments
       */
      TrivialDynamicArray< Segment > nonDominatedNodeSegments(const Node source, const Node target) noexcept {
        TrivialDynamicArray< Segment > candidates;
        _srpath_infos.removeAll();

        for (NodeIt v(_routing_protocol.digraph()); v != nt::INVALID; ++v) {
          if (v == source /*|| v == target*/) continue;
          const SrPath< Digraph > p({source, v, target});
          SrPathInfo              srpath_info_p(_routing_protocol.digraph());

          _routing_protocol.ecmpTraversal(
             p, 1, [&srpath_info_p](const Arc arc, const FlowValue f_split) { srpath_info_p.insertArc(arc, f_split); });

          bool is_p_dominated = false;
          for (int k = candidates.size() - 1; k >= 0; --k) {
            const Segment     w = candidates[k];
            const SrPathInfo& srpath_info_q = _srpath_infos[k];
            bool              is_q_dominated = false;

            if (srpath_info_p.arcNum() < srpath_info_q.arcNum()) {   // p cannot be dominated by q
              is_q_dominated = true;                                 // assume p dominates q
              for (const Arc arc: srpath_info_p.arcs()) {
                if (srpath_info_p.ratio(arc) > srpath_info_q.ratio(arc)) {
                  is_q_dominated = false;   // p does not dominate q
                  break;
                }
              }
            } else if (srpath_info_p.arcNum() > srpath_info_q.arcNum()) {   // q cannot be dominated by p
              is_p_dominated = true;                                        // assume q dominates p
              for (const Arc arc: srpath_info_q.arcs()) {
                if (srpath_info_p.ratio(arc) < srpath_info_q.ratio(arc)) {
                  is_p_dominated = false;   // q does not dominate p
                  break;
                }
              }
            } else {
              is_q_dominated = true;
              is_p_dominated = true;
              for (const Arc arc: srpath_info_q.arcs()) {
                const FlowValue ratio_p = srpath_info_p.ratio(arc);
                const FlowValue ratio_q = srpath_info_q.ratio(arc);

                if (ratio_p > ratio_q) {
                  is_q_dominated = false;
                  break;
                } else if (ratio_p < ratio_q) {
                  is_p_dominated = false;
                  break;
                }
              }
              for (const Arc arc: srpath_info_p.arcs()) {
                const FlowValue ratio_p = srpath_info_p.ratio(arc);
                const FlowValue ratio_q = srpath_info_q.ratio(arc);

                if (ratio_p < ratio_q) {
                  is_p_dominated = false;
                  break;
                } else if (ratio_p > ratio_q) {
                  is_q_dominated = false;
                  break;
                }
              }

              if (is_q_dominated == true
                  && is_p_dominated == true) {   // If both SR paths induces exactly the same SP graphs and loads
                is_q_dominated = false;          // then keep only q and delete p
              }
            }

            assert(is_q_dominated == false || is_p_dominated == false);
            if (is_q_dominated) {
              candidates.removeLast();
              _srpath_infos.removeLast();
            }
            if (is_p_dominated) break;
          }

          if (!is_p_dominated) {
            candidates.add(v);
            _srpath_infos.add(std::move(srpath_info_p));
          }
        }

        return candidates;
      }
    };

    /**
     * @brief Class that is used to extend a routing protocol with segment routing capabilities.
     *
     * @tparam UnderlyingProtocol A class that implements the underlying protocol
     * on top of which segment routing is based.
     *
     */
    template < typename UnderlyingProtocol >
    struct SegmentRoutingExtender: public UnderlyingProtocol {
      using Parent = UnderlyingProtocol;

      using Digraph = typename Parent::Digraph;
      TEMPLATE_DIGRAPH_TYPEDEFS_EX(Digraph);

      template < typename T >
      using ArcMap = typename Digraph::template ArcMap< T >;

      using Vector = typename Parent::Vector;
      using FlowValue = typename Parent::FlowValue;

      using Segment = typename SrPath< Digraph >::Segment;
      using Metric = typename Parent::Metric;
      using MetricMap = typename Parent::MetricMap;

      using ShortestPathGraphTraverser = typename Parent::ShortestPathGraphTraverser;

      /**
       * @brief Constructor
       *
       * @param g A const reference to the digraph the algorithm runs on.
       * @param mm A const reference to the metric map.
       */
      SegmentRoutingExtender(const Digraph& g, const MetricMap& mm) : Parent(g, mm) {}

      /**
       * @brief Get the underlying routing protocol on top of which segment routing is executed
       *
       * @return A reference to the underlying routing protocol
       */
      constexpr Parent& underlyingProtocol() const noexcept { return *this; }
      constexpr Parent& underlyingProtocol() noexcept { return *this; }

      /**
       * @brief Run the segment routing algorithm, i.e., route all the demands along the associated SR-paths.
       *
       * @param dg  The demand graph.
       * @param dvm The demand volume map.
       * @param sm  The map storing the SR-paths. One SR-path per demand.
       *
       * @tparam DemandGraph The type of the demand graph.
       * @tparam DemandValueMap A concepts::ReadMap "readable" arc map storing the demand values.
       * @tparam SrPathMap A concepts::ReadMap "readable" arc map storing the SR-paths.
       *
       * @return int The number of demands successfully routed.
       */
      template < typename DemandGraph, typename DemandValueMap, typename SrPathMap >
      int run(const DemandGraph& dg, const DemandValueMap& dvm, const SrPathMap& sm) noexcept {
        return _run(
           dg,
           dvm,
           sm,
           [&](const SrPath< Digraph >& sr_path, typename DemandGraph::Arc demand_arc) {
             return routeFlow(sr_path, dvm[demand_arc]);
           },
           [&](const Node source, const Node target, typename DemandGraph::Arc demand_arc) {
             return Parent::routeFlow(source, target, dvm[demand_arc]);
           });
      }

      /**
       * @brief Run the segment routing algorithm, i.e., route all the demands along the associated SR-paths.
       * Furthermore, the set of demands routing over each arc is stored in the passed arc map 'dpa'
       * i.e dpa[a] is an array containing the demands using arc 'a' (dpa : demands per arc)
       *
       * @param dg  The demand graph.
       * @param dvm The demand volume map.
       * @param sm  The map storing the SR-paths. One SR-path per demand.
       * @param dpa The map storing the set of demands routed on each arc
       *
       * @tparam DemandGraph The type of the demand graph.
       * @tparam DemandValueMap A concepts::ReadMap "readable" arc map storing the demand values.
       * @tparam SrPathMap A concepts::ReadMap "readable" arc map storing the SR-paths.
       * @tparam DemandsPerArc A concepts::ReadMap "readable" arc map storing the demands over each arc
       *
       * @return int The number of demands successfully routed.
       */
      template < typename DemandGraph, typename DemandValueMap, typename SrPathMap, typename DemandsPerArc >
      int run(const DemandGraph& dg, const DemandValueMap& dvm, const SrPathMap& sm, DemandsPerArc& dpa) noexcept {
        return _run(
           dg,
           dvm,
           sm,
           [&](const SrPath< Digraph >& sr_path, typename DemandGraph::Arc demand_arc) {
             return routeFlow(sr_path, dvm[demand_arc], demand_arc, dpa);
           },
           [&](const Node source, const Node target, typename DemandGraph::Arc demand_arc) {
             return Parent::routeFlow(source, target, dvm[demand_arc], demand_arc, dpa);
           });
      }

      /**
       * @brief Route a flow value along the given SR-path. The provided SR-path is assumed
       * to be valid w.r.t to the given demand arc (see isValid())
       *
       * @param sr_path The SR-path.
       * @param flow The flow value to be routed.
       * @param demand_arc  The demand to be routed
       * @param dpa  The map storing the set of demands routed on each arc
       *
       * @return true if the flow was succesfully routed, false otherwise
       */
      template < typename DemandArc, typename DemandsPerArc >
      bool routeFlow(const SrPath< Digraph >& sr_path,
                     const FlowValue          flow,
                     const DemandArc          demand_arc,
                     DemandsPerArc&           dpa) noexcept {
        return sr_path.forEachSegment(
           Parent::digraph(),
           [&](const Node source, const Node target) {
             return Parent::routeFlow(source, target, flow, demand_arc, dpa);
           },
           [&](const Arc arc) { Parent::routeStaticFlow(arc, flow); });
      }

      /**
       * @brief Route a flow value along the given SR-path.
       *
       * @param sr_path The SR-path.
       * @param flow The flow value to be routed.
       *
       * @return true if the flow was succesfully routed, false otherwise
       */
      bool routeFlow(const SrPath< Digraph >& sr_path, const FlowValue flow) noexcept {
        return sr_path.forEachSegment(
           Parent::digraph(),
           [this, &flow](const Node source, const Node target) { return Parent::routeFlow(source, target, flow); },
           [this, &flow](const Arc arc) { Parent::routeStaticFlow(arc, flow); });
      }

      /**
       * @brief Remove a given amount of flow along the given SR-path.
       *
       * @param sr_path The SR-path.
       * @param flow The quantity of flow to be removed.
       *
       */
      void removeFlow(const SrPath< Digraph >& sr_path, const FlowValue flow) noexcept { routeFlow(sr_path, -flow); }

      /**
       * @brief Remove a given amount of flow along the shortect paths from source to target
       *
       */
      void removeFlow(const Node source, const Node target, const FlowValue flow) noexcept {
        Parent::removeFlow(source, target, flow);
      }

      /**
       * @brief Compute the split factor (or ratio) of every arc that belongs to the given SR path.
       *
       * @param sr_path The segment routing path.
       * @param ratio_map The map to store the computed ratios.
       *
       * @tparam MAP A concepts::ReadMap "readable" arc map storing the ratio values.
       *
       * @return true if the flow was succesfully routed, false otherwise
       */
      template < typename MAP >
      bool computeRatios(const SrPath< Digraph >& sr_path, MAP& ratio_map) noexcept {
        ratio_map.setBitsToZero();
        return ecmpTraversal(
           sr_path, 1, [&ratio_map](const Arc arc, const FlowValue f_split) { ratio_map[arc] += f_split; });
      }

      /**
       * @brief
       *
       * @tparam MAP
       * @param source
       * @param target
       * @param ratio_map
       * @return
       */
      template < typename MAP >
      bool computeRatios(const Node source, const Node target, MAP& ratio_map) noexcept {
        return Parent::computeRatios(source, target, ratio_map);
      }

      /**
       * @brief
       *
       * @tparam MAP
       * @param source
       * @param target
       * @param ratio_map
       * @return
       */
      template < typename MAP >
      bool computeRatios(const Node                  source,
                         const Node                  target,
                         MAP&                        ratio_map,
                         TrivialDynamicArray< Arc >& sp_arcs) noexcept {
        return Parent::computeRatios(source, target, ratio_map, sp_arcs);
      }

      /**
       * @brief Perform ECMP traversal along the given SR path.
       *
       * @param sr_path The segment routing path.
       * @param flow The flow value to be traversed.
       * @param doVisitArc Callback function called every that time a new arc a is traversed. It
       * takes as input the arc and the total amount of flow routed along that arc. This function
       * returns nothing.
       *
       * @return bool Status of the traversal operation.
       */
      template < typename F >
      bool ecmpTraversal(const SrPath< Digraph >& sr_path, const FlowValue flow, F doVisitArc) noexcept {
        return sr_path.forEachSegment(
           Parent::digraph(),
           [&](const Node source, const Node target) {
             return Parent::ecmpTraversal(source, target, flow, doVisitArc);
           },
           [&](const Arc arc) { doVisitArc(arc, flow); });
      }

      /**
       * @brief Check whether an SR-path is simple. An SR-path is simple if no vertex appears more than once
       * on the paths routing the demand from its origin to its destination using the given SR-path.
       *
       * @param sr_path The segment routing path to be tested.
       * @param traverser The shortest path traverser to be used to explore the shortest paths between segments.
       * @param visited A static node map used to store the visited nodes during the traversal.
       *
       * @return Status of the SR-path.
       *
       */
      template < typename ArcFilter = nt::True >
      bool isSimple(const SrPath< Digraph >&    sr_path,
                    ShortestPathGraphTraverser& traverser,
                    IntStaticNodeMap&           visited,
                    ArcFilter                   arcOk = {}) noexcept {
        return sr_path.forEachSegment(
           Parent::digraph(),
           [&](const Node source, const Node target) {
             return Parent::forEachNode(
                source,
                target,
                [&](const Node n) {
                  int&      m = visited[n];
                  const int hash = Digraph::id(target) + 1;
                  if (m && m != hash) return false;
                  if (n != target) m = hash;
                  return true;
                },
                traverser,
                arcOk);
           },
           [](const Arc) {});
      }

      /**
       * @brief Check whether an SR-path is simple. An SR-path is simple if no vertex appears more than once
       * on the paths routing the demand from its origin to its destination using the given SR-path.
       *
       * @param sr_path The segment routing path to be tested.
       * @param arcOk Optional arc filter; if provided, returns false if any blacklisted arc appears
       *              in the ECMP shortest-path graph between consecutive waypoints.
       *
       * @return Status of the SR-path.
       *
       */
      template < typename ArcFilter = nt::True >
      bool isSimple(const SrPath< Digraph >& sr_path, ArcFilter arcOk = {}) noexcept {
        IntStaticNodeMap           visited(Parent::digraph(), 0);
        ShortestPathGraphTraverser traverser(Parent::digraph());
        return isSimple(sr_path, traverser, visited, arcOk);
      }

      // private

      template < typename DemandGraph, typename DemandValueMap, typename SrPathMap, typename F, typename G >
      int _run(const DemandGraph& dg, const DemandValueMap& dvm, const SrPathMap& sm, F f, G g) noexcept {
        int i_num_routed = 0;
        for (typename DemandGraph::ArcIt demand_arc(dg); demand_arc != nt::INVALID; ++demand_arc) {
          const SrPath< Digraph >& sr_path = sm[demand_arc];
          if (!sr_path.empty())
            i_num_routed += f(sr_path, demand_arc);
          else {
            const Node source = dg.source(demand_arc);
            const Node target = dg.target(demand_arc);
            i_num_routed += g(source, target, demand_arc);
          }
        }
        return i_num_routed;
      }
    };


    /**
     * @class SegmentRouting
     * @headerfile segment_routing.h
     * @brief Implementation of the segment routing protocol based on the computation
     * of shortest paths between segments (with ECMP rule).
     *
     * Example of usage :
     *
     * @code
     * #include "networktools.h"
     *
     * int main() {
     *   // Define aliases for convenience
     *   using Digraph = nt::graphs::SmartDigraph;
     *   using DemandGraph = nt::graphs::DemandGraph< Digraph >;
     *   using DemandArc = DemandGraph::Arc;
     *   using SrPath = nt::te::SrPath< Digraph >;
     *   using SegmentRouting = nt::te::SegmentRouting< Digraph >;
     *
     *   // Define the basic graph types: Node, Arc, ...
     *   DIGRAPH_TYPEDEFS(Digraph);
     *
     *   // Build a simple network
     *   Digraph digraph;
     *
     *   Node v0 = digraph.addNode();
     *   Node v1 = digraph.addNode();
     *   Node v2 = digraph.addNode();
     *   Node v3 = digraph.addNode();
     *   Arc  a0 = digraph.addArc(v1, v0);
     *   Arc  a1 = digraph.addArc(v1, v2);
     *   Arc  a2 = digraph.addArc(v2, v3);
     *   Arc  a3 = digraph.addArc(v0, v3);
     *
     *   // Create an arc map to store the arc capacity and set them to 1
     *   Digraph::ArcMap< double > capacities(digraph, 1);
     *
     *   // Build the demand graph
     *   DemandGraph demand_graph(digraph);
     *
     *   const DemandArc darc0 = demand_graph.addArc(v0, v3);
     *   const DemandArc darc1 = demand_graph.addArc(v1, v3);
     *
     *   DemandGraph::ArcMap< double > demand_values(demand_graph);
     *   demand_values[darc0] = 1.0;
     *   demand_values[darc1] = 1.0;
     *
     *   // Create an SrPath for each demand
     *   DemandGraph::ArcMap< SrPath > sr_paths(demand_graph);
     *
     *   SrPath& sr_path_darc0 = sr_paths[darc0];
     *   sr_path_darc0.addSegment(v1);
     *   sr_path_darc0.addSegment(v2);
     *   sr_path_darc0.addSegment(v3);
     *
     *   SrPath& sr_path_darc1 = sr_paths[darc1];
     *   sr_path_darc1.addSegment(v1);
     *   sr_path_darc1.addSegment(v2);
     *   sr_path_darc1.addSegment(v3);
     *
     *   // Print the SR paths
     *   std::cout << "SR Path for demand 0: " << sr_path_darc0.toString(digraph) << std::endl;
     *   std::cout << "SR Path for demand 1: " << sr_path_darc1.toString(digraph) << std::endl;
     *
     *   // Create an instance of SegmentRouting
     *   SegmentRouting sr(digraph);
     *
     *   // Run the segment routing algorithm that routes all demands in 'demand_graph'
     *   // with their associated volume values in 'demand_values' and according to the provided SR paths in 'sr_paths'
     *   const int i_num_routed = sr.run(demand_graph, demand_values, sr_paths);
     *
     *   std::cout << "Number of demands routed: " << i_num_routed << std::endl;
     *   std::cout << "Max saturation : " << sr.maxSaturation(capacities).max() << std::endl;
     *
     *   return 0;
     * }
     * @endcode
     *
     * @tparam GR The type of the directed graph representing the network.
     * @tparam FPTYPE floating-point precision could be float, double, long double, ...
     * @tparam VEC The type of the vector used to store demands, flows, saturation values.
     * @tparam MM A concepts::ReadMap "readable" arc map storing the metric values.
     */
    template < typename GR,
               typename FPTYPE = double,
               typename VEC = nt::Vec< FPTYPE, 1 >,
               typename MM = typename GR::template ArcMap< FPTYPE > >
    struct SegmentRouting: SegmentRoutingExtender< ShortestPathRouting< GR, FPTYPE, VEC, MM > > {
      using Parent = SegmentRoutingExtender< ShortestPathRouting< GR, FPTYPE, VEC, MM > >;
      using Digraph = GR;
      using MetricMap = MM;
      using Metric = typename Parent::Metric;

      SegmentRouting(const Digraph& g, const MetricMap& mm) : Parent(g, mm) {}
    };

    /**
     * @class DynamicSegmentRouting
     * @headerfile segment_routing.h
     * @brief Implementation of the segment routing protocol based on the computation
     * of shortest paths between segments (with ECMP rule). Compare to SegmentRouting, this class allows dynamic updates
     * of the shortest paths after arc metrics changes.
     *
     * Example of usage :
     *
     * @code
     * #include "networktools.h"
     *
     * int main() {
     *   // Define aliases for convenience
     *   using Digraph = nt::graphs::SmartDigraph;
     *   using DemandGraph = nt::graphs::DemandGraph< Digraph >;
     *   using DemandArc = DemandGraph::Arc;
     *   using SrPath = nt::te::SrPath< Digraph >;
     *   using DynamicSegmentRouting = nt::te::DynamicSegmentRouting< Digraph >;
     *
     *   // Define the basic graph types: Node, Arc, ...
     *   DIGRAPH_TYPEDEFS(Digraph);
     *
     *   // Build a simple network
     *   Digraph digraph;
     *
     *   Node v0 = digraph.addNode();
     *   Node v1 = digraph.addNode();
     *   Node v2 = digraph.addNode();
     *   Node v3 = digraph.addNode();
     *   Arc  a0 = digraph.addArc(v1, v0);
     *   Arc  a1 = digraph.addArc(v1, v2);
     *   Arc  a2 = digraph.addArc(v2, v3);
     *   Arc  a3 = digraph.addArc(v0, v3);
     *
     *   // Create an arc map to store the arc capacity and set them to 1
     *   Digraph::ArcMap< double > capacities(digraph, 1);
     *
     *   // Build the demand graph
     *   DemandGraph demand_graph(digraph);
     *
     *   const DemandArc darc0 = demand_graph.addArc(v0, v3);
     *   const DemandArc darc1 = demand_graph.addArc(v1, v3);
     *
     *   DemandGraph::ArcMap< double > demand_values(demand_graph);
     *   demand_values[darc0] = 1.0;
     *   demand_values[darc1] = 1.0;
     *
     *   // Create an SrPath for each demand
     *   DemandGraph::ArcMap< SrPath > sr_paths(demand_graph);
     *
     *   SrPath& sr_path_darc0 = sr_paths[darc0];
     *   sr_path_darc0.addSegment(v1);
     *   sr_path_darc0.addSegment(v2);
     *   sr_path_darc0.addSegment(v3);
     *
     *   SrPath& sr_path_darc1 = sr_paths[darc1];
     *   sr_path_darc1.addSegment(v1);
     *   sr_path_darc1.addSegment(v2);
     *   sr_path_darc1.addSegment(v3);
     *
     *   // Print the SR paths
     *   std::cout << "SR Path for demand 0: " << sr_path_darc0.toString(digraph) << std::endl;
     *   std::cout << "SR Path for demand 1: " << sr_path_darc1.toString(digraph) << std::endl;
     *
     *   // Create an instance of DynamicSegmentRouting
     *   DynamicSegmentRouting sr(digraph);
     *
     *   // Run the segment routing algorithm that routes all demands in 'demand_graph'
     *   // with their associated volume values in 'demand_values' and according to the provided SR paths in 'sr_paths'
     *   const int i_num_routed = sr.run(demand_graph, demand_values, sr_paths);
     *
     *   std::cout << "Number of demands routed: " << i_num_routed << std::endl;
     *   std::cout << "Max saturation : " << sr.maxSaturation(capacities).max() << std::endl;
     *
     *   // Reset previous routing
     *   sr.clear();
     *
     *   // Change the SR path for demand 1
     *   sr_path_darc1.clear();
     *   sr_path_darc1.addSegment(v1);
     *   sr_path_darc1.addSegment(v0);
     *   sr_path_darc1.addSegment(v3);
     *
     *   std::cout << "SR Path for demand 0: " << sr_path_darc0.toString(digraph) << std::endl;
     *   std::cout << "SR Path for demand 1: " << sr_path_darc1.toString(digraph) << std::endl;
     *
     *   std::cout << "Number of demands routed: " << sr.run(demand_graph, demand_values, sr_paths) << std::endl;
     *   std::cout << "Max saturation : " << sr.maxSaturation(capacities).max() << std::endl;
     *
     *   return 0;
     * }
     * @endcode
     *
     * @tparam GR The type of the directed graph representing the network.
     * @tparam FPTYPE floating-point precision could be float, double, long double, ...
     * @tparam VEC The type of the vector used to store demands, flows, saturation values.
     * @tparam MM A concepts::ReadMap "readable" arc map storing the metric values.
     */
    template < typename GR,
               typename FPTYPE = double,
               typename VEC = nt::Vec< FPTYPE, 1 >,
               typename MM = typename GR::template ArcMap< FPTYPE > >
    struct DynamicSegmentRouting: SegmentRoutingExtender< DynamicShortestPathRouting< GR, FPTYPE, VEC, MM > > {
      using Parent = SegmentRoutingExtender< DynamicShortestPathRouting< GR, FPTYPE, VEC, MM > >;
      using Digraph = GR;
      using MetricMap = MM;
      using Metric = typename Parent::Metric;

      DynamicSegmentRouting(const Digraph& g, const MetricMap& mm) : Parent(g, mm) {}
    };
  }   // namespace te
}   // namespace nt
#endif