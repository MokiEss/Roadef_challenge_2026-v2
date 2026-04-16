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
 * @brief Parse networks and demand graphs data from SNDlib XML format
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_SNDLIB_READER_H_
#define _NT_SNDLIB_READER_H_

#include "../../core/maps.h"
#include "../../core/xml.h"
#include "../../core/string.h"
#include "../../core/sets.h"
#include "../tools.h"
#include "../demand_graph.h"

namespace nt {
  namespace graphs {
    /**
     * @brief Parse networks and demand graphs data from SNDlib XML format
     * (http://sndlib.zib.de/html/docu/io-formats/xml/index.html).
     *
     * Example :
     *
     * @code
     *  using Digraph = nt::graphs::SmartDigraph;
     *  using DemandGraph = graphs::DemandGraph< Digraph >;
     *  using Reader = nt::graphs::SndlibReader< Digraph >;
     *
     *  Reader reader;
     *  reader.loadFile("abilene.xml");
     *
     *  // Read the network and store its capacities in cap_map
     *  Digraph network;
     *  Digraph::ArcMap< double > cap_map(network);
     *  reader.readNetwork(network, cap_map);
     *
     *  // Read the demand graph and store its demand values in dv_map
     *  DemandGraph demand_graph(network);
     *  DemandGraph::ArcMap< double > dv_map(demand_graph);
     *  reader.readDemandGraph(demand_graph, dv_map);
     * @endcode
     *
     * @tparam GR The type of the (di)graph to be read.
     * @tparam DG The type of the digraph representing the demand graph.
     * It must be defined on the same node set as GR.
     *
     */
    template < typename GR, typename DG = graphs::DemandGraph< GR > >
    struct SndlibReader {
      using Digraph = GR;
      using DemandGraph = DG;
      using Node = typename Digraph::Node;
      using NodeIt = typename Digraph::NodeIt;

      nt::ByteBuffer                                  _xml_file;
      nt::XMLDocument                                 _document;
      nt::CstringMap< Node >                          _router_name_2_nodes_map;
      nt::NumericalSet< uint64_t >                    _num_arc_duplicates;
      nt::TrivialDynamicArray< nt::Pair< int, int > > _links_buffer;
      int                                             _i_num_nodes;

      SndlibReader() : _i_num_nodes(0) {}

      /**
       * @brief Load an XML file in SNDlib XML format.
       *
       * @param sz_fileName Name of the filename to be loaded
       *
       * @return true if the the loaded succeed, false otherwise.
       */
      bool loadFile(const char* sz_fileName) noexcept {
        if (!_xml_file.load(sz_fileName)) return false;
        _document.clear();
        return _document.parse< 0 >(_xml_file.data());
      }

      /**
       * @brief Load a buffer containing a network in SNDlib XML format.
       *
       * @param sz_stream Stream buffer
       *
       * @return true if the the loaded succeed, false otherwise.
       */
      bool loadFromStream(const char* sz_stream) noexcept {
        if (!_xml_file.copyFrom(sz_stream)) return false;
        _document.clear();
        return _document.parse< 0 >(_xml_file.data());
      }

      /**
       * @brief Read the network topology from the previously loaded file.
       *
       * @param digraph Reference to the digraph
       * @param b_ensure_simply_bidirected Ensure that the graph has no duplicated arcs and is bidirected.
       * Each arc uv added to make the graph bidirected has the same capacity as the opposite arc vu.
       *
       * @return true if the parsing succeed, false otherwise.
       */
      bool readNetwork(Digraph& digraph, bool b_ensure_simply_bidirected = false) noexcept {
        digraph.clear();

        nt::XMLElement *p_nodes_elt, *p_links_elt;

        if (!_initReadNetwork(&p_nodes_elt, &p_links_elt)) return false;

        return _readNetwork(p_nodes_elt, p_links_elt, digraph, b_ensure_simply_bidirected);
      }

      /**
       * @brief Read the network topology from the previously loaded file.
       *
       * @param digraph Reference to the digraph
       * @param cm Reference to the arc/edge map that will receive the capacity values.
       * @param b_ensure_simply_bidirected Ensure that the graph has no duplicated arcs and is bidirected.
       * Each arc uv added to make the graph bidirected has the same capacity as the opposite arc vu.
       *
       * @return true if the parsing succeed, false otherwise.
       */
      template < typename CapacityMap >
      bool readNetwork(Digraph& digraph, CapacityMap& cm, bool b_ensure_simply_bidirected = false) noexcept {
        digraph.clear();

        nt::XMLElement *p_nodes_elt, *p_links_elt;

        if (!_initReadNetwork(&p_nodes_elt, &p_links_elt)) return false;

        if (!_readNetwork(p_nodes_elt, p_links_elt, digraph, b_ensure_simply_bidirected)) return false;

        _iterateOverLinks(
           p_links_elt,
           [this, &digraph, &cm, b_ensure_simply_bidirected](const nt::XMLElement* p_link_elt, Node u, Node v) {
             const double f_capacity = this->_extractModuleCapacity(p_link_elt);
             if (f_capacity < 0) return false;

             if constexpr (nt::concepts::UndirectedTagIndicator< Digraph >) {
               const typename Digraph::Edge uv = findEdge(digraph, u, v);
               assert(uv != INVALID);
               cm.set(uv, f_capacity);
             } else {
               const typename Digraph::Arc uv = findArc(digraph, u, v);
               assert(uv != INVALID);
               cm.set(uv, f_capacity);
               if (b_ensure_simply_bidirected) {
                 const typename Digraph::Arc vu = findArc(digraph, v, u);
                 assert(vu != INVALID);
                 cm.set(vu, f_capacity);
               }
             }

             return true;
           });

        return true;
      }

      /**
       * @brief Read the demand graph from the previously loaded file. You can call
       * this function only after a call to readNetwork.
       *
       * @param demand_graph Reference to the digraph
       *
       * @return true if the parsing succeed, false otherwise.
       */
      bool readDemandGraph(DemandGraph& demand_graph) noexcept {
        demand_graph.clear();

        nt::XMLElement* p_demands_elt;

        if (!_initReadDemandGraph(&p_demands_elt)) return false;

        if (!_readDemandGraph(p_demands_elt, demand_graph)) return false;

        return true;
      }

      /**
       * @brief Read the demand graph from the previously loaded file. You can call
       * this function only after a call to readNetwork.
       *
       * @param demand_graph Reference to the digraph
       * @param dm Reference to the arc/edge map that will receive the demand values.
       *
       * @return true if the parsing succeed, false otherwise.
       */
      template < typename DemandValueMap >
      bool readDemandGraph(DemandGraph& demand_graph, DemandValueMap& dm) noexcept {
        demand_graph.clear();

        nt::XMLElement* p_demands_elt;

        if (!_initReadDemandGraph(&p_demands_elt)) return false;

        if (!_readDemandGraph(p_demands_elt, demand_graph)) return false;

        _iterateOverDemands(
           p_demands_elt, [this, &demand_graph, &dm](const nt::XMLElement* p_demand_elt, Node u, Node v) {
             const typename DemandGraph::Arc uv = findArc(demand_graph, u, v);
             assert(uv != nt::INVALID);
             double f_demand_value = -1.f;
             nt::XMLUtil::toDouble(p_demand_elt->first_node("demandValue")->first_node()->value(), &f_demand_value);
             dm.set(uv, f_demand_value);
             return true;
           });

        return true;
      }

      // Private methods

      bool _initReadDemandGraph(nt::XMLElement** p_demands_elt) noexcept {
        const nt::XMLElement* p_network_elt = _document.first_node("network");
        if (!p_network_elt) return false;

        *p_demands_elt = p_network_elt->first_node("demands");
        if (!(*p_demands_elt)) return false;

        return true;
      }

      bool _readDemandGraph(const nt::XMLElement* p_demands_elt, DemandGraph& demand_graph) noexcept {
        if constexpr (nt::concepts::BuildTagIndicator< DemandGraph >) {
          _links_buffer.removeAll();
          _links_buffer.ensure(_i_num_nodes);

          _iterateOverDemands(p_demands_elt, [this](const nt::XMLElement*, Node u, Node v) {
            this->_links_buffer.add({DemandGraph::id(u), DemandGraph::id(v)});
            return true;
          });

          demand_graph.build(_i_num_nodes, _links_buffer.begin(), _links_buffer.end());

        } else {
          _iterateOverDemands(p_demands_elt, [this, &demand_graph](const nt::XMLElement* p_demand_elt, Node u, Node v) {
            const typename DemandGraph::Arc uv = demand_graph.addArc(u, v);
            return true;
          });
        }

        return true;
      }

      bool _initReadNetwork(nt::XMLElement** p_nodes_elt, nt::XMLElement** p_links_elt) noexcept {
        assert(p_nodes_elt && p_links_elt);

        _router_name_2_nodes_map.clear();
        _num_arc_duplicates.clear();
        _i_num_nodes = 0;

        const nt::XMLElement* p_network_elt = _document.first_node("network");
        if (!p_network_elt) return false;

        const nt::XMLElement* p_network_structure_elt = p_network_elt->first_node("networkStructure");
        if (!p_network_structure_elt) return false;

        *p_nodes_elt = p_network_structure_elt->first_node("nodes");
        if (!(*p_nodes_elt)) return false;

        *p_links_elt = p_network_structure_elt->first_node("links");
        if (!(*p_links_elt)) return false;

        return true;
      }

      bool _readNetwork(const nt::XMLElement* p_nodes_elt,
                        const nt::XMLElement* p_links_elt,
                        Digraph&              digraph,
                        bool                  b_ensure_simply_bidirected = false) noexcept {
        // Is the graph buildable ?
        if constexpr (nt::concepts::BuildTagIndicator< Digraph >) {
          _i_num_nodes = _iterateOverNodes(p_nodes_elt, [](int i_count) { return i_count; });
          if (!_i_num_nodes) return false;

          _links_buffer.removeAll();
          _links_buffer.ensure(_i_num_nodes);

          _iterateOverLinks(p_links_elt, [this, b_ensure_simply_bidirected](const nt::XMLElement*, Node u, Node v) {
            if constexpr (nt::concepts::UndirectedTagIndicator< Digraph >) {
              this->_links_buffer.add({Digraph::id(u), Digraph::id(v)});
            } else {
              if (b_ensure_simply_bidirected) {
                const int i = Digraph::id(u);
                const int j = Digraph::id(v);

                const uint64_t i_link_hash = (i <= j ? NT_CONCAT_INT_32(i, j) : NT_CONCAT_INT_32(j, i));
                if (!this->_num_arc_duplicates.contains(i_link_hash)) {
                  this->_num_arc_duplicates.insert(i_link_hash);
                  this->_links_buffer.add({Digraph::id(u), Digraph::id(v)});
                  this->_links_buffer.add({Digraph::id(v), Digraph::id(u)});
                }
              } else {
                this->_links_buffer.add({Digraph::id(u), Digraph::id(v)});
              }
            }

            return true;
          });

          digraph.build(_i_num_nodes, _links_buffer.begin(), _links_buffer.end());

        } else {
          // Insert the vertices
          _i_num_nodes = _iterateOverNodes(p_nodes_elt, [this, &digraph](int) { return digraph.addNode(); });
          if (!_i_num_nodes) return false;

          _iterateOverLinks(
             p_links_elt,
             [this, &digraph, b_ensure_simply_bidirected](const nt::XMLElement* p_link_elt, Node u, Node v) {
               if constexpr (nt::concepts::UndirectedTagIndicator< Digraph >) {
                 const typename Digraph::Edge uv = digraph.addEdge(u, v);
               } else {
                 if (b_ensure_simply_bidirected) {
                   const int i = Digraph::id(u);
                   const int j = Digraph::id(v);

                   const uint64_t i_link_hash = i <= j ? NT_CONCAT_INT_32(i, j) : NT_CONCAT_INT_32(j, i);
                   if (!this->_num_arc_duplicates.contains(i_link_hash)) {
                     this->_num_arc_duplicates.insert(i_link_hash);
                     const typename Digraph::Arc uv = digraph.addArc(u, v);
                     const typename Digraph::Arc vu = digraph.addArc(v, u);
                   }
                 } else {
                   const typename Digraph::Arc uv = digraph.addArc(u, v);
                 }
               }

               return true;
             });
        }

        return true;
      }

      double _extractModuleCapacity(const nt::XMLElement* p_link_elt) const noexcept {
        double f_capacity = -1.;
        if (const nt::XMLElement* p_pre_installed_module_elt = p_link_elt->first_node("preInstalledModule")) {
          nt::XMLUtil::toDouble(p_pre_installed_module_elt->first_node("capacity")->first_node()->value(), &f_capacity);
        } else if (const nt::XMLElement* p_additional_modules_elt = p_link_elt->first_node("additionalModules")) {
          // We choose the first available module
          const nt::XMLElement* p_module_elt = p_additional_modules_elt->first_node("addModule");
          nt::XMLUtil::toDouble(p_module_elt->first_node("capacity")->first_node()->value(), &f_capacity);
        }
        return f_capacity;
      }

      template < typename F >
      int _iterateOverNodes(const nt::XMLElement* p_nodes_elt, F hookFunc) noexcept {
        int i_count = 0;
        for (const nt::XMLElement* p_node_elt = p_nodes_elt->first_node("node"); p_node_elt;
             p_node_elt = p_node_elt->next_sibling("node")) {
          nt::XMLAttributte* p_id_attr = p_node_elt->first_attribute("id");
          if (!p_id_attr) {
            LOG_F(ERROR, "Invalid id for a Node");
            return 0;
          }
          const char* sz_router_name = p_id_attr->value();
          _router_name_2_nodes_map[sz_router_name] = hookFunc(i_count);
          ++i_count;
        }

        return i_count;
      }

      template < typename F >
      bool _iterateOverLinks(const nt::XMLElement* p_links_elt, F hookFunc) noexcept {
        for (const nt::XMLElement* p_link_elt = p_links_elt->first_node("link"); p_link_elt;
             p_link_elt = p_link_elt->next_sibling("link")) {
          const nt::XMLElement* p_source_name = p_link_elt->first_node("source")->first_node();
          const nt::XMLElement* p_target_name = p_link_elt->first_node("target")->first_node();

          const char* szU = p_source_name->value();
          const char* szV = p_target_name->value();
          const Node  u = _router_name_2_nodes_map[szU];
          const Node  v = _router_name_2_nodes_map[szV];

          if (!hookFunc(p_link_elt, u, v)) return false;
        }
        return true;
      }

      template < typename F >
      bool _iterateOverDemands(const nt::XMLElement* p_demands_elt, F hookFunc) noexcept {
        for (const nt::XMLElement* p_demand_elt = p_demands_elt->first_node("demand"); p_demand_elt;
             p_demand_elt = p_demand_elt->next_sibling("demand")) {
          const nt::XMLElement* p_source_name = p_demand_elt->first_node("source")->first_node();
          const nt::XMLElement* p_target_name = p_demand_elt->first_node("target")->first_node();

          const char* szU = p_source_name->value();
          const char* szV = p_target_name->value();

          const Node u = _router_name_2_nodes_map.findWithDefault(szU, INVALID);
          const Node v = _router_name_2_nodes_map.findWithDefault(szV, INVALID);

          if (u == INVALID) return false;
          if (v == INVALID) return false;

          if (!hookFunc(p_demand_elt, u, v)) return false;
        }
        return true;
      }
    };

  }   // namespace graphs
}   // namespace nt


#endif