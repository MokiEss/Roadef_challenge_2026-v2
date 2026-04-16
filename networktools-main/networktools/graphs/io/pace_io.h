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
 * @brief PaceIo class for reading and writing graphs in the PACE graph format.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_PACE_IO_H_
#define _NT_PACE_IO_H_

#include <fstream>

#include "../tools.h"
#include "../edge_graph.h"

namespace nt {
  namespace graphs {

    /**
     * @brief PaceIo class for reading and writing graphs in the PACE graph format.
     *
     * This class provides functionality for reading and writing graphs in PACE format.
     * The code is originally taken and adapted from https://github.com/holgerdell/td-validate/
     *
     * Example of usage:
     *
     * @code
     *  using Graph = nt::graphs::SmartGraph;
     *  Graph g;
     *
     *  PaceIo<Graph> pace_io;
     *
     *  pace_io.readFile("graph.gr", g);
     * @endcode
     *
     * @tparam GR The type of the graph.
     */
    template < typename GR >
    // requires nt::concepts::Digraph< GR >
    struct PaceIo {
      static_assert(!nt::concepts::BuildTagIndicator< GR >, "PaceIo does not support graphs with a BuildTag");
      using Graph = GR;
      TEMPLATE_GRAPH_TYPEDEFS(Graph);

      /*
       * The different states the syntax checker may find itself in while checking the
       * syntax of a *.gr- or *.td-file.
       */
      enum State { COMMENT_SECTION, S_LINE, BAGS, EDGES, P_LINE };

      /*
       * Messages associated with the respective exceptions to be thrown while
       * checking the files
       */
      const char* INV_FMT = "Invalid format";
      const char* INV_SOLN = "Invalid s-line";
      const char* INV_SOLN_BAGSIZE = "Invalid s-line: Reported bagsize and actual bagsize differ";
      const char* SOLN_MISSING = "s-line is missing";
      const char* INV_EDGE = "Invalid edge";
      const char* INV_BAG = "Invalid bag";
      const char* INV_BAG_INDEX = "Invalid bag index";
      const char* INC_SOLN = "Inconsistent values in s-line";
      const char* NO_BAG_INDEX = "No bag index given";
      const char* BAG_MISSING = "Bag missing";
      const char* FILE_ERROR = "Could not open file";
      const char* EMPTY_LINE = "No empty lines allowed";
      const char* INV_PROB = "Invalid p-line";

      /*
       * The state the syntax checker is currently in; this variable is used for both
       * checking the tree *and* the graph
       */
      State _current_state;

      /* The number of vertices of the graph as stated in the *.gr-file */
      unsigned n_vertices;

      /* The number of edges of the graph as stated in the *.gr-file */
      unsigned n_edges;

      /**
       * @brief Constructs a PaceIo object with default options.
       *
       */
      PaceIo() : _current_state(COMMENT_SECTION){};

      /**
       * @brief Reads the graph from a file.
       *
       * @param sz_filename Name of the file to read from
       * @param g
       * @return True if the read operation was successful, false otherwise.
       */
      bool readFile(const char* sz_filename, Graph& g) noexcept {
        std::basic_ifstream< char > stream(sz_filename);
        if (!stream) return false;
        stream.unsetf(std::ios::skipws);
        return read< std::basic_ifstream< char > >(stream, g);
      }

      /**
       * @brief Reads the graph from a stream.
       *
       *
       * @return True if the read operation was successful, false otherwise.
       */
      template < typename InputStream >
      bool read(InputStream& buffer, Graph& g) noexcept {
        buffer.seekg(0);

        _current_state = COMMENT_SECTION;
        n_edges = -1;
        n_vertices = -1;

        if (!buffer.is_open()) {
          LOG_F(ERROR, FILE_ERROR);
          return false;
        }

        std::string line;

        while (std::getline(buffer, line)) {
          nt::DynamicArray< std::string > tokens = _tokenize(line);
          if (tokens.empty()) {
            LOG_F(ERROR, EMPTY_LINE);
            return false;
          } else if (tokens[0] == "p") {
            _read_problem(tokens, g);
          } else if (tokens.size() == 2) {
            _read_graph_edge(tokens, g);
          } else if (tokens[0] != "c") {
            LOG_F(ERROR, "{} in line {}", INV_FMT, line);
            return false;
          }
        }

        if (g.edgeNum() != n_edges) {
          LOG_F(ERROR, "{} (incorrect number of edges)", INV_PROB);
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
      bool writeFile(const char* sz_filename, const Graph& g) noexcept {
        nt::ByteBuffer buffer;
        return write< nt::ByteBuffer >(buffer, g) && buffer.save(sz_filename);
      }

      /**
       * @brief Writes the graph to a stream.
       *
       * @return
       */
      template < typename OutputStream >
      bool write(OutputStream& buffer, const Graph& g) noexcept {
        nt::format_to(std::back_inserter(buffer), "p tw {} {}\n", g.nodeNum(), g.edgeNum());
        for (EdgeIt e(g); e != INVALID; ++e)
          nt::format_to(std::back_inserter(buffer), "{} {}\n", g.id(g.u(e)) + 1, g.id(g.v(e)) + 1);
        return true;
      }

      // Private

      /**
       * @brief Given a string, split it at ' ' and return vector of words
       *
       * @param line
       * @return nt::DynamicArray< std::string >
       */
      nt::DynamicArray< std::string > _tokenize(const std::string& line) const noexcept {
        nt::DynamicArray< std::string > tokens;
        size_t                          oldpos = 0, newpos = 0;
        while (newpos != std::string::npos) {
          newpos = line.find(" ", oldpos);
          tokens.push_back(line.substr(oldpos, newpos - oldpos));
          oldpos = newpos + 1;
        }
        return tokens;
      }


      /**
       * @brief Given the tokens from one line (split on whitespace), this reads the
       * p-line from these tokens and initializes the corresponding globals
       *
       * @param tokens
       * @param g
       * @return true
       * @return false
       */
      bool _read_problem(const nt::DynamicArray< std::string >& tokens, Graph& g) noexcept {
        if (_current_state != COMMENT_SECTION) {
          LOG_F(ERROR, INV_FMT);
          return false;
        }
        _current_state = P_LINE;

        if (tokens.size() != 4 /*|| tokens[1] != "tw"*/) {
          LOG_F(ERROR, INV_PROB);
          return false;
        }

        n_vertices = _pure_stou(tokens[2]);
        n_edges = _pure_stou(tokens[3]);

        while (g.nodeNum() < n_vertices)
          g.addNode();

        return true;
      }

      /**
       * @brief Given the tokens from one line (split on whitespace) and a tree
       * decomposition, this reads the edge (in the graph) represented by this line
       * and adds the respective edge to the graph
       *
       * @param tokens
       * @param g
       * @return true
       * @return false
       */
      bool _read_graph_edge(const nt::DynamicArray< std::string >& tokens, Graph& g) noexcept {
        if (_current_state == P_LINE) _current_state = EDGES;

        if (_current_state != EDGES) {
          LOG_F(ERROR, INV_FMT);
          return false;
        }

        const unsigned s = _pure_stou(tokens[0]);
        const unsigned d = _pure_stou(tokens[1]);

        if (s < 1 || d < 1 || s > n_vertices || d > n_vertices) {
          LOG_F(ERROR, "{}: {} {}. ", INV_EDGE, tokens[0], tokens[1]);
          return false;
        }

        g.addEdge(g.nodeFromId(s - 1), g.nodeFromId(d - 1));

        return true;
      }

      /**
       * @brief An unsigned variant of std::stoi that also checks if the input consists
       * entirely of base-10 digits
       *
       * @param s
       * @return unsigned
       */
      unsigned _pure_stou(const std::string& s) const noexcept {
        unsigned result = 0;
        if (s.empty()) {
          LOG_F(ERROR, "Empty string");
          return ~0;
        }
        for (unsigned i = 0; i < s.length(); i++) {
          char c = s[i];
          if (c < '0' || c > '9') {
            LOG_F(ERROR, "Non-numeric entry '{}'", s.c_str());
            return ~0;
          }
          result = 10 * result + (c - '0');
        }
        return result;
      }
    };

  }   // namespace graphs
}   // namespace nt


#endif