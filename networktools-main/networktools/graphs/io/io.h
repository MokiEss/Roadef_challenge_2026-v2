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

#ifndef _NT_IO_H_
#define _NT_IO_H_

#include "networkx_io.h"
#include "jgf_io.h"
#include "binary_io.h"
#include "sndlib_reader.h"
#include "../../core/csv.h"

namespace nt {
  namespace graphs {

    /**
     * @brief
     *
     */
    enum class GraphFormat {
      /** SNDLib XML format */
      // SNDLIB,
      /** JSON format as stored in NetworkX library */
      NETWORKX,
      /** JSON Graph Format v2 (https://jsongraphformat.info/) */
      // JGF,
      /**
       * Binary graph-tool format (https://graph-tool.skewed.de/static/doc/gt_format.html)
       */
      BINARY,
      /**
       * XML graph format as described in http://graphml.graphdrawing.org/
       */
      // GRAPHML
    };

    /**
     * @brief Function-type interface for reading various graph file formats.
     *
     * @tparam Digraph
     * @param sz_filename
     * @param gf
     * @param g
     * @param p_properties
     * @return true
     * @return false
     */
    template < typename Digraph >
    inline bool read(const char*                 sz_filename,
                     GraphFormat                 gf,
                     Digraph&                    g,
                     GraphProperties< Digraph >* p_properties = nullptr) noexcept {
      nt::ByteBuffer buffer;
      if (!buffer.load(sz_filename)) return false;
      switch (gf) {
        case GraphFormat::NETWORKX: {
          NetworkxIo< Digraph > graph_io;
          return graph_io.read(buffer, g, p_properties);
        }
        case GraphFormat::BINARY: {
          BinaryIo< Digraph > graph_io;
          return graph_io.read(buffer, g, p_properties);
        }
          // case GraphFormat::GRAPHML: {
          //   GraphMLIO< Digraph > graph_io;
          //   return graph_io.read(buffer, g, p_properties);
          // }
      }
      return false;
    }

    /**
     * @brief Function-type interface for writing to various graph file formats.
     *
     * @tparam Digraph
     * @param sz_filename
     * @param gf
     * @param g
     * @param p_properties
     * @return true
     * @return false
     */
    template < typename Digraph >
    inline bool write(const char*                       sz_filename,
                      GraphFormat                       gf,
                      const Digraph&                    g,
                      const GraphProperties< Digraph >* p_properties = nullptr) noexcept {
      nt::ByteBuffer buffer;
      switch (gf) {
        case GraphFormat::NETWORKX: {
          NetworkxIo< Digraph > graph_io;
          if (!graph_io.write(buffer, g, p_properties)) return false;
        } break;
        case GraphFormat::BINARY: {
          BinaryIo< Digraph > graph_io;
          if (!graph_io.write(buffer, g, p_properties)) return false;
        } break;
          // case GraphFormat::GRAPHML: {
          //   GraphMLIO< Digraph > graph_io;
          //   if (!graph_io.write(buffer, g, p_properties)) return false;
          // } break;
      }
      return buffer.save(sz_filename);
    }

    /**
     * @brief Validates graph file format and properties without loading data.
     *
     * This function performs a dry-run validation of a graph file to check:
     * - File format validity
     * - Presence of requested properties/attributes in GraphProperties filters
     * - Data type compatibility
     *
     * It logs detailed information about missing or incompatible fields at high verbosity levels
     * and returns a boolean indicating whether all requested properties are available.
     *
     * @tparam Digraph The graph type
     * @param sz_filename Path to the graph file to validate
     * @param gf Graph format (NETWORKX, BINARY, etc.)
     * @param p_properties Pointer to GraphProperties containing the filters/maps to validate
     * @return true if all requested properties are available and compatible, false otherwise
     */
    template < typename Digraph >
    inline bool validateRead(const char*                       sz_filename,
                             GraphFormat                       gf,
                             const GraphProperties< Digraph >* p_properties = nullptr) noexcept {
      nt::ByteBuffer buffer;
      if (!buffer.load(sz_filename)) return false;
      switch (gf) {
        case GraphFormat::NETWORKX: {
          NetworkxIo< Digraph > graph_io;
          return graph_io.validateRead(buffer, p_properties);
        }
        case GraphFormat::BINARY: {
          BinaryIo< Digraph > graph_io;
          return graph_io.validateRead(buffer, p_properties);
        }
          // case GraphFormat::GRAPHML: {
          //   GraphMLIO< Digraph > graph_io;
          //   return graph_io.validateRead(buffer, p_properties);
          // }
      }
      return false;
    }

    namespace details {
      template < typename OutputStream, typename Digraph, typename Map, typename Iterator >
      inline void writeGraphMap(OutputStream& buffer, const Digraph& g, const Map& map) noexcept {
        for (Iterator it(g); it != nt::INVALID; ++it)
          nt::format_to(std::back_inserter(buffer), "{};{}\n", g.id(it), map[it]);
      }

      template < typename Digraph, typename Map, typename Iterator >
      inline void writeGraphMapFile(const char* sz_filename, const Digraph& g, const Map& map) noexcept {
        TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

        nt::MemoryBuffer buffer;

        if constexpr (std::is_same< Iterator, NodeIt >::value) {
          nt::format_to(std::back_inserter(buffer), "0;{}\n", sz_filename);
        } else if constexpr (std::is_same< Iterator, ArcIt >::value) {
          nt::format_to(std::back_inserter(buffer), "1;{}\n", sz_filename);
        } else {
          nt::format_to(std::back_inserter(buffer), "2;{}\n", sz_filename);
        }

        details::writeGraphMap< nt::MemoryBuffer, Digraph, Map, Iterator >(buffer, g, map);
        std::ofstream file;
        file.open(sz_filename);
        file << nt::to_string(buffer);
        file.close();
      }
    }   // namespace details

    /**
     * @brief
     *
     * @tparam OutputStream
     * @tparam Digraph
     * @tparam Map
     * @param buffer
     * @param g
     * @param map
     * @return
     */
    template < typename OutputStream, typename Digraph, typename Map >
    inline void writeNodeMap(OutputStream& buffer, const Digraph& g, const Map& map) noexcept {
      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
      details::writeGraphMap< OutputStream, Digraph, Map, NodeIt >(buffer, g, map);
    }

    /**
     * @brief
     *
     * @tparam Digraph
     * @tparam Map
     * @param sz_filename
     * @param g
     * @param map
     * @return true
     * @return false
     */
    template < typename Digraph, typename Map >
    inline void writeNodeMapFile(const char* sz_filename, const Digraph& g, const Map& map) noexcept {
      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
      details::writeGraphMapFile< Digraph, Map, NodeIt >(sz_filename, g, map);
    }

    /**
     * @brief
     *
     * @tparam OutputStream
     * @tparam Digraph
     * @tparam Map
     * @param buffer
     * @param g
     * @param map
     * @return
     */
    template < typename OutputStream, typename Digraph, typename Map >
    inline void writeArcMap(OutputStream& buffer, const Digraph& g, const Map& map) noexcept {
      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
      details::writeGraphMap< OutputStream, Digraph, Map, ArcIt >(buffer, g, map);
    }

    /**
     * @brief
     *
     * @tparam Digraph
     * @tparam Map
     * @param sz_filename
     * @param g
     * @param map
     * @return true
     * @return false
     */
    template < typename Digraph, typename Map >
    inline void writeArcMapFile(const char* sz_filename, const Digraph& g, const Map& map) noexcept {
      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
      details::writeGraphMapFile< Digraph, Map, ArcIt >(sz_filename, g, map);
    }

    /**
     * @brief
     *
     * @tparam OutputStream
     * @tparam Graph
     * @tparam Map
     * @param buffer
     * @param g
     * @param map
     * @return true
     * @return false
     */
    template < typename OutputStream, typename Graph, typename Map >
    inline void writeEdgeMap(OutputStream& buffer, const Graph& g, const Map& map) noexcept {
      TEMPLATE_GRAPH_TYPEDEFS(Graph);
      details::writeGraphMap< OutputStream, Graph, Map, EdgeIt >(buffer, g, map);
    }

    /**
     * @brief
     *
     * @tparam Graph
     * @tparam Map
     * @param sz_filename
     * @param g
     * @param map
     * @return true
     * @return false
     */
    template < typename Graph, typename Map >
    inline void writeEdgeMapFile(const char* sz_filename, const Graph& g, const Map& map) noexcept {
      TEMPLATE_GRAPH_TYPEDEFS(Graph);
      details::writeGraphMapFile< Graph, Map, EdgeIt >(sz_filename, g, map);
    }

  }   // namespace graphs
}   // namespace nt

#endif
