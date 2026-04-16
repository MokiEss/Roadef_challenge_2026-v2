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
 * @brief RepetitaIo class for reading and writing graphs in the REPETITA graph format
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_REPETITA_IO_H_
#define _NT_REPETITA_IO_H_

#include <fstream>

#include "../tools.h"
#include "../edge_graph.h"
#include "../demand_graph.h"

namespace nt {
  namespace graphs {

    /**
     * @brief RepetitaIo class for reading and writing graphs in the REPETITA graph format
     * (https://github.com/svissicchio/Repetita/wiki/Adding-Problem-Instances).
     *
     * This class provides functionality for reading and writing graphs in REPETITA format.
     *
     * Example of usage:
     *
     * @code
     *  using Digraph = nt::graphs::SmartDigraph;
     *  Digraph g;
     *
     *  nt::graphs::RepetitaIo< Digraph > rep_io;
     *
     *  CapacityMap capacities(g);
     *  MetricMap   metrics(g);
     *  DelayMap    delays(g);
     *  NameMap     names(g);
     *
     *  rep_io.readNetworkFile("Abilene.graph", g);
     *  rep_io.readDemandsFile("Abilene.0000.demands", g);
     * @endcode
     *
     * @tparam GR The type of the graph.
     */
    template < typename GR, typename DG = graphs::DemandGraph< GR > >
    // requires nt::concepts::Digraph< GR >
    struct RepetitaIo {
      static_assert(!nt::concepts::BuildTagIndicator< GR >, "RepetitaIo does not support graphs with a BuildTag");
      using Digraph = GR;
      using DemandGraph = DG;
      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

      /*
       * The different states the syntax checker may find itself in while checking the
       * syntax of a *.gr- or *.td-file.
       */
      enum State { COMMENT_SECTION, EDGES, NODES, DEMANDS };

      /*
       * Messages associated with the respective exceptions to be thrown while
       * checking the files
       */
      const char* INV_FMT = "Invalid format";
      const char* INV_EDGE = "Invalid edge";
      const char* INV_BAG_INDEX = "Invalid bag index";
      const char* FILE_ERROR = "Could not open file";
      const char* EMPTY_LINE = "No empty lines allowed";
      const char* INV_PROB = "Invalid p-line";

      /*
       * The state the syntax checker is currently in; this variable is used for both
       * checking the tree *and* the graph
       */
      State _current_state;

      /**
       * @brief Constructs a RepetitaIo object with default options.
       *
       */
      RepetitaIo() : _current_state(COMMENT_SECTION){};

      /**
       * @brief Reads the graph from a file.
       *
       * @param sz_filename Name of the file to read from
       * @param g
       * @return True if the read operation was successful, false otherwise.
       */
      template < typename NameMap, typename WeightMap, typename CapacityMap, typename DelayMap >
      bool readNetworkFile(
         const char* sz_filename, Digraph& g, NameMap& nm, WeightMap& wm, CapacityMap& cm, DelayMap& dm) noexcept {
        std::basic_ifstream< char > stream(sz_filename);
        if (!stream) return false;
        stream.unsetf(std::ios::skipws);
        return readNetwork< std::basic_ifstream< char > >(stream, g, nm, wm, cm, dm);
      }

      /**
       * @brief Reads the graph from a stream.
       *
       * @return True if the read operation was successful, false otherwise.
       */
      template < typename InputStream, typename NameMap, typename WeightMap, typename CapacityMap, typename DelayMap >
      bool readNetwork(
         InputStream& buffer, Digraph& g, NameMap& nm, WeightMap& wm, CapacityMap& cm, DelayMap& dm) noexcept {
        buffer.seekg(0);

        _current_state = COMMENT_SECTION;
        unsigned n_arcs = -1;
        unsigned n_nodes = -1;

        if (!buffer.is_open()) {
          LOG_F(ERROR, FILE_ERROR);
          return false;
        }

        std::string                    line;
        nt::DynamicArray< nt::String > tokens;
        int                            k = 0;
        while (std::getline(buffer, line)) {
          nt::strtok(std::move(line), tokens);
          if (tokens.size() == 2 && tokens[0][0] == 'N') {
            _current_state = State::NODES;
            n_nodes = nt::pure_stou(tokens[1]);
            std::getline(buffer, line);   // Next line (memo line) is useless to parse
          } else if (tokens.size() == 2 && tokens[0][0] == 'E') {
            if (n_nodes < 0) {
              LOG_F(ERROR, "Empty graph");
              return false;
            }
            _current_state = EDGES;
            n_arcs = nt::pure_stou(tokens[1]);
            std::getline(buffer, line);   // Next line (memo line) is useless to parse
          } else if (tokens.size() == 3 && _current_state == State::NODES) {
            const Node n = g.addNode();
            nm[n].assign(tokens[0].c_str());
          } else if (tokens.size() == 6 && _current_state == State::EDGES) {
            const unsigned u = nt::pure_stou(tokens[1]);
            const unsigned v = nt::pure_stou(tokens[2]);
            if (u < 0 || v < 0 || u >= n_nodes || v >= n_nodes) {
              LOG_F(ERROR, "Line {}: {}: {} {}. ", k, INV_EDGE, tokens[0], tokens[1]);
              return false;
            }
            const Arc arc = g.addArc(g.nodeFromId(u), g.nodeFromId(v));
            wm[arc] = nt::pure_stou(tokens[3]);
            cm[arc] = nt::pure_stou(tokens[4]);
            dm[arc] = nt::pure_stou(tokens[5]);
          }
          ++k;
        }

        if (g.nodeNum() != n_nodes) {
          LOG_F(ERROR, "{} (incorrect number of nodes)", INV_PROB);
          return false;
        }

        if (g.arcNum() != n_arcs) {
          LOG_F(ERROR, "{} (incorrect number of arcs)", INV_PROB);
          return false;
        }

        return true;
      }

      /**
       * @brief Reads the graph from a file.
       *
       * @param sz_filename Name of the file to read from
       * @param g
       * @return True if the read operation was successful, false otherwise.
       */
      template < typename DemandValueMap >
      bool readDemandGraphFile(const char* sz_filename, DemandGraph& dg, DemandValueMap& dvm) noexcept {
        std::basic_ifstream< char > stream(sz_filename);
        if (!stream) return false;
        stream.unsetf(std::ios::skipws);
        return readDemandGraph< std::basic_ifstream< char > >(stream, dg, dvm);
      }

      /**
       * @brief Reads the graph from a stream.
       *
       * @return True if the read operation was successful, false otherwise.
       */
      template < typename InputStream, typename DemandValueMap >
      bool readDemandGraph(InputStream& buffer, DemandGraph& dg, DemandValueMap& dvm) noexcept {
        buffer.seekg(0);

        _current_state = COMMENT_SECTION;
        unsigned n_demands = -1;

        if (!buffer.is_open()) {
          LOG_F(ERROR, FILE_ERROR);
          return false;
        }

        std::string                    line;
        nt::DynamicArray< nt::String > tokens;
        int                            k = 0;
        while (std::getline(buffer, line)) {
          nt::strtok(std::move(line), tokens);
          if (tokens.size() == 2 && tokens[0][0] == 'D') {
            _current_state = State::DEMANDS;
            n_demands = nt::pure_stou(tokens[1]);
            std::getline(buffer, line);   // Next line (memo line) is useless to parse
          } else if (tokens.size() == 4 && _current_state == State::DEMANDS) {
            const unsigned u = nt::pure_stou(tokens[1]);
            const unsigned v = nt::pure_stou(tokens[2]);
            if (u < 0 || v < 0 || u >= dg.nodeNum() || v >= dg.nodeNum()) {
              LOG_F(ERROR, "Line {}: {}: {} {}. ", k, INV_EDGE, tokens[0], tokens[1]);
              return false;
            }
            const typename DemandGraph::Arc arc = dg.addArc(dg.nodeFromId(u), dg.nodeFromId(v));
            dvm[arc] = nt::pure_stou(tokens[3]);
          }
          ++k;
        }

        if (dg.arcNum() != n_demands) {
          LOG_F(ERROR, "{} (incorrect number of arcs)", INV_PROB);
          return false;
        }

        return true;
      }

      /**
       * @brief
       *
       * @param sz_filename
       * @param g
       * @return true
       * @return false
       */
      bool writeFile(const char* sz_filename, const Digraph& g) noexcept {
        nt::ByteBuffer buffer;
        return write< nt::ByteBuffer >(buffer, g) && buffer.save(sz_filename);
      }

      /**
       * @brief Writes the graph to a stream.
       *
       * @return
       */
      template < typename OutputStream >
      bool write(OutputStream& buffer, const Digraph& g) noexcept {
        assert(false);   // Not yet implemented
        return false;
      }
    };

  }   // namespace graphs
}   // namespace nt


#endif