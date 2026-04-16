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
 * @brief BinaryIo class for reading and writing graphs in binary format.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_BINARY_IO_H_
#define _NT_BINARY_IO_H_

#include "../../core/common.h"

namespace nt {
  namespace graphs {

    namespace details {

      template < bool BE, typename T >
      inline void byteSwap(T& p) noexcept {
        if constexpr (BE != NT_IS_BIG_ENDIAN) {
          // p = std::byteswap(p); // C++23 only
          char& r = reinterpret_cast< char& >(p);
          std::reverse(&r, &r + sizeof(T));
        }
      }

      template < typename T, typename InputStream >
      inline void writeScalar(InputStream& stream, T v) {
        stream.write(reinterpret_cast< const char* >(&v), sizeof(T));
      };

      template < typename InputStream >
      inline void writeString(InputStream& stream, const nt::String& s) noexcept {
        details::writeScalar(stream, static_cast< std::uint64_t >(s.size()));
        stream.write(s.c_str(), s.size());
      };

      template < typename T, typename InputStream >
      inline void writeArray(InputStream& stream, nt::ArrayView< T > v) noexcept {
        const uint64_t size = v.size();
        details::writeScalar(stream, size);
        stream.write(reinterpret_cast< const char* >(v.data()), sizeof(T) * v.size());
      };

      template < typename InputStream >
      inline void writeStringArray(InputStream& stream, nt::ArrayView< nt::String > v) noexcept {
        const uint64_t size = v.size();
        details::writeScalar(stream, size);
        for (const nt::String& str: v)
          details::writeString(stream, str);
      };

      inline uint8_t typeid2gttype(nt::Type t) noexcept {
        switch (t) {
          case nt::Type::UINT32:
          case nt::Type::INT32: return 0x02; break;
          case nt::Type::UINT64:
          case nt::Type::INT64: return 0x03; break;
          case nt::Type::FLOAT: return 0x0f; break;
          case nt::Type::DOUBLE: return 0x04; break;
          case nt::Type::BOOL: return 0x00; break;
          case nt::Type::STRING: return 0x06; break;
          case nt::Type::ARRAY_UINT32:
          case nt::Type::ARRAY_INT32: return 0x09; break;
          case nt::Type::ARRAY_UINT64:
          case nt::Type::ARRAY_INT64: return 0x0a; break;
          case nt::Type::ARRAY_FLOAT: return 0x10; break;
          case nt::Type::ARRAY_DOUBLE: return 0x0b; break;
          case nt::Type::ARRAY_BOOL: return 0x07; break;
          case nt::Type::ARRAY_STRING: return 0x0d; break;
          default: assert(false); return -1;
        }
      }

      template < typename InputStream >
      inline void writeAny(const nt::AnyView& any, InputStream& stream) noexcept {
        switch (any.type()) {
          case nt::Type::INT64: {
            const std::int64_t v = any.get< std::int64_t >(NT_INT64_MAX);
            details::writeScalar(stream, v);
          } break;
          case nt::Type::INT32: {
            const std::int32_t v = any.get< std::int32_t >(NT_INT32_MAX);
            details::writeScalar(stream, v);
          } break;
          case nt::Type::UINT32: {
            const std::uint32_t v = any.get< std::uint32_t >(NT_UINT32_MAX);
            details::writeScalar(stream, v);
          } break;
          case nt::Type::UINT64: {
            const std::uint64_t v = any.get< std::uint64_t >(NT_UINT64_MAX);
            details::writeScalar(stream, v);
          } break;
          case nt::Type::FLOAT: {
            const float v = any.get< float >(NT_FLOAT_MAX);
            details::writeScalar(stream, v);
          } break;
          case nt::Type::DOUBLE: {
            const double v = any.get< double >(NT_DOUBLE_MAX);
            details::writeScalar(stream, v);
          } break;
          case nt::Type::BOOL: {
            const bool v = any.get< bool >(NT_BOOL_MAX);
            stream.put(static_cast< char >(v));
          } break;
          case nt::Type::STRING: {
            const nt::String& s = any.get< nt::String >(nt::String());
            details::writeString(stream, s);
          } break;
          case nt::Type::ARRAY_INT32: {
            const nt::IntDynamicArray& v = any.get< nt::IntDynamicArray >(nt::IntDynamicArray());
            details::writeArray< std::int32_t >(stream, v);
          } break;
          case nt::Type::ARRAY_INT64: {
            const nt::Int64DynamicArray& v = any.get< nt::Int64DynamicArray >(nt::Int64DynamicArray());
            details::writeArray< std::int64_t >(stream, v);
          } break;
          case nt::Type::ARRAY_UINT32: {
            const nt::UintDynamicArray& v = any.get< nt::UintDynamicArray >(nt::UintDynamicArray());
            details::writeArray< std::uint32_t >(stream, v);
          } break;
          case nt::Type::ARRAY_UINT64: {
            const nt::Uint64DynamicArray& v = any.get< nt::Uint64DynamicArray >(nt::Uint64DynamicArray());
            details::writeArray< std::uint64_t >(stream, v);
          } break;
          case nt::Type::ARRAY_FLOAT: {
            const nt::FloatDynamicArray& v = any.get< nt::FloatDynamicArray >(nt::FloatDynamicArray());
            details::writeArray< float >(stream, v);
          } break;
          case nt::Type::ARRAY_DOUBLE: {
            const nt::DoubleDynamicArray& v = any.get< nt::DoubleDynamicArray >(nt::DoubleDynamicArray());
            details::writeArray< double >(stream, v);
          } break;
          case nt::Type::ARRAY_BOOL: {
            const nt::TrivialDynamicArray<bool>& v = any.get< nt::TrivialDynamicArray<bool> >(nt::TrivialDynamicArray<bool>());
            details::writeArray< bool >(stream, v);
          } break;
          case nt::Type::ARRAY_STRING: {
            const nt::DynamicArray< nt::String >& v =
               any.get< nt::DynamicArray< nt::String > >(nt::DynamicArray< nt::String >());
            details::writeStringArray(stream, v);
          } break;
          default: assert(false);
        }
      }

      template < bool BE, typename T, typename InputStream >
      inline void readScalar(InputStream& stream, T& v) noexcept {
        stream.read(reinterpret_cast< char* >(&v), sizeof(T));
        details::byteSwap< BE >(v);
      };

      template < bool BE, typename InputStream >
      inline void readString(InputStream& stream, nt::String& str) noexcept {
        uint64_t i_length = 0;
        details::readScalar< BE >(stream, i_length);   // read length (8 bytes)
        assert(i_length > 0);
        str.resize(i_length);
        stream.read(&str[0], i_length);   // read string (length bytes)
      }

      template < bool BE, typename Array, typename InputStream >
      inline void readArray(InputStream& stream, Array& v) noexcept {
        using T = typename Array::value_type;
        std::uint64_t size = 0;
        details::readScalar< BE >(stream, size);
        v.removeAll();
        v.ensureAndFill(size);
        stream.read(reinterpret_cast< char* >(v.data()), sizeof(T) * v.size());
        for (int k = 0; k < v.size(); ++k)
          details::byteSwap< BE >(v[k]);
      };

      template < bool BE, typename InputStream >
      inline void readStringArray(InputStream& stream, nt::DynamicArray< nt::String >& v) noexcept {
        uint64_t size = 0;
        details::readScalar< BE >(stream, size);
        v.removeAll();
        v.ensureAndFill(size);
        for (int k = 0; k < v.size(); ++k)
          details::readString< BE >(stream, v[k]);
      };

      template < bool BE, typename InputStream >
      inline void readAny(nt::AnyView& any, InputStream& stream, uint8_t type) noexcept {
        switch (type) {
          case 0x00: {   // bool (1 byte)
            char v;
            stream.read(&v, 1);
            any.set(static_cast< std::uint8_t >(v));
          } break;
          case 0x01: {   // int16_t (2 byte)
            std::int16_t v = 0;
            details::readScalar< BE >(stream, v);
            any.set(std::move(v));
          } break;
          case 0x02: {   // int32_t (4 byte)
            std::int32_t v = 0;
            details::readScalar< BE >(stream, v);
            any.set(std::move(v));
          } break;
          case 0x03: {   // int64_t (8 byte)
            std::int64_t v = 0;
            details::readScalar< BE >(stream, v);
            any.set(std::move(v));
          } break;
          case 0x04: {   // double (8 byte)
            double v = 0.;
            details::readScalar< BE >(stream, v);
            any.set(std::move(v));
          } break;
          case 0x05: {   // long double (16 byte)
            long double v = 0.;
            details::readScalar< BE >(stream, v);
            any.set(std::move(v));
          } break;
          case 0x06: {   // string (8 byte + length)
            nt::String str;
            details::readString< BE >(stream, str);
            any.set(std::move(str));
          } break;
          case 0x07: {   // vector<bool> (8 byte + length)
            nt::TrivialDynamicArray<bool> v;
            details::readArray< BE >(stream, v);
            any.set(std::move(v));
          } break;
          case 0x08: {   // vector<int16_t> (8 + 2 * length)
            nt::Int16DynamicArray v;
            details::readArray< BE >(stream, v);
            any.set(std::move(v));
          } break;
          case 0x09: {   // vector<int32_t> (8 + 4 * length)
            nt::IntDynamicArray v;
            details::readArray< BE >(stream, v);
            any.set(std::move(v));
          } break;
          case 0x0a: {   // vector<int64_t> (8 + 8 * length)
            nt::Int64DynamicArray v;
            details::readArray< BE >(stream, v);
            any.set(std::move(v));
          } break;
          case 0x0b: {   // vector<double> (8 + 8 * length)
            nt::DoubleDynamicArray v;
            details::readArray< BE >(stream, v);
            any.set(std::move(v));
          } break;
          case 0x0c: {   // vector<long double> (8 + 16 * length)
            nt::LongDoubleDynamicArray v;
            details::readArray< BE >(stream, v);
            any.set(std::move(v));
          } break;
          case 0x0d: {   // vector<string> (8 + <variable>)
            nt::DynamicArray< nt::String > v;
            details::readStringArray< BE >(stream, v);
            any.set(std::move(v));
          } break;
          case 0x0e: {   // python::object (8 + length)
            assert(false);
          } break;
          case 0x0f: {   // float (4 byte)
            float v = 0.;
            details::readScalar< BE >(stream, v);
            any.set(std::move(v));
          } break;
          case 0x10: {   // vector<float> (8 + 4 * length)
            nt::FloatDynamicArray v;
            details::readArray< BE >(stream, v);
            any.set(std::move(v));
          } break;
          default: assert(false);
        }
      }

      template < typename GR, bool UNDIR >
      struct BinaryIoSelector {
        TEMPLATE_GRAPH_TYPEDEFS(GR);

        template < typename INT, typename OutputStream >
        void _writeAdjacencyDispatch(const GR& g, OutputStream& stream) const noexcept {
          // Get the size in bytes to encode a node index
          // const int i_idx_size = _encodingSize(i_num_nodes);

          // Get the size in bytes to encode a node out degree
#if 0
        int i_max_out_deg = 0;
        for (NodeIt u(g); u != INVALID; ++u) {
          const int i_out_deg = countOutArcs(g, u);
          if (i_out_deg > i_max_out_deg) i_max_out_deg = i_out_deg;
        }
        const int i_deg_size = _encodingSize(i_max_out_deg);
#else
          const int i_deg_size = 8;
#endif

          for (uint64_t u = 0; u < g.nodeNum(); ++u) {
            const Node node = g.nodeFromId(u);
            // Write the degree
            uint64_t i_count = 0;
            for (IncEdgeIt edge(g, node); edge != INVALID; ++edge) {
              const int v = g.id(g.oppositeNode(node, edge));
              if (v > u) continue;
              ++i_count;
            }
            details::writeScalar(stream, i_count);

            // Write neighbors
            for (IncEdgeIt edge(g, node); edge != INVALID; ++edge) {
              const uint64_t v = g.id(g.oppositeNode(node, edge));
              if (v > u) continue;   // Avoid adding edge uv and vu
              INT k = static_cast< INT >(v);
              details::writeScalar(stream, k);
            }
          }
        }

        template < bool BE, typename INT, typename InputStream >
        void _readAdjacencyDispatch(GR& g, uint64_t i_num_nodes, InputStream& stream) const noexcept {
          for (uint64_t u = 0; u < i_num_nodes; ++u) {
            // Read the out degree
            uint64_t i_deg = 0;
            details::readScalar< BE >(stream, i_deg);

            // Read out neighbors
            for (uint64_t d = 0; d < i_deg; ++d) {
              INT v = 0;
              details::readScalar< BE >(stream, v);
              g.addEdge(g.nodeFromId(u), g.nodeFromId(v));
            }
          }
        }

        template < typename InputStream >
        void _writeArcEdgeMaps(const GR& g, const GraphProperties< GR >& gp, InputStream& stream) const noexcept {
          for (typename GraphProperties< GR >::EdgeMaps::const_iterator it = gp.edgeMaps().begin();
               it != gp.edgeMaps().end();
               ++it) {
            const nt::details::MapStorage< Edge >& map = (*it->second);

            stream.put(static_cast< char >(2));   // Map type (1 byte)

            const std::uint64_t l = std::strlen(it->first);
            details::writeScalar(stream, l);
            stream.write(it->first, l);                        // Map name
            stream.put(details::typeid2gttype(map.value()));   // value type (1 byte)

            for (EdgeIt edge(g); edge != INVALID; ++edge) {
              const nt::AnyView any = map.get(edge);
              details::writeAny(any, stream);
            }
          }
        }

        template < bool BE, typename InputStream >
        void _readArcEdgeMap(GR&                    g,
                             const nt::String&      caption,
                             GraphProperties< GR >& gp,
                             uint8_t                type,
                             InputStream&           stream) const noexcept {
          if (nt::details::MapStorage< Edge >** p_map_storage = gp.edgeMaps().findOrNull(caption.c_str())) {
            for (EdgeIt edge(g); edge != INVALID; ++edge) {
              nt::AnyView any = (*p_map_storage)->get(edge);
              details::readAny< BE >(any, stream, type);
            }
          } else {
            for (EdgeIt edge(g); edge != INVALID; ++edge) {
              nt::AnyView any;
              details::readAny< BE >(any, stream, type);
            }
          }
        }
      };


      template < typename GR >
      struct BinaryIoSelector< GR, false > {
        TEMPLATE_DIGRAPH_TYPEDEFS(GR);

        template < typename INT, typename OutputStream >
        void _writeAdjacencyDispatch(const GR& g, OutputStream& stream) const noexcept {
          for (uint64_t u = 0; u < g.nodeNum(); ++u) {
            const Node node = g.nodeFromId(u);
            // Write the out degree
            const uint64_t i_out_deg = countOutArcs(g, node);
            details::writeScalar(stream, i_out_deg);

            // Write out neighbors
            for (OutArcIt out_arc(g, node); out_arc != INVALID; ++out_arc) {
              INT k = static_cast< INT >(g.id(g.target(out_arc)));
              details::writeScalar(stream, k);
            }
          }
        }

        template < bool BE, typename INT, typename InputStream >
        void _readAdjacencyDispatch(GR& g, uint64_t i_num_nodes, InputStream& stream) const noexcept {
          for (uint64_t u = 0; u < i_num_nodes; ++u) {
            // Read the out degree
            uint64_t i_out_deg = 0;
            details::readScalar< BE >(stream, i_out_deg);

            // Read out neighbors
            for (uint64_t d = 0; d < i_out_deg; ++d) {
              INT v = 0;
              details::readScalar< BE >(stream, v);
              g.addArc(g.nodeFromId(u), g.nodeFromId(v));
            }
          }
        }

        template < typename InputStream >
        void _writeArcEdgeMaps(const GR& g, const GraphProperties< GR >& gp, InputStream& stream) const noexcept {
          for (typename GraphProperties< GR >::ArcMaps::const_iterator it = gp.arcMaps().begin();
               it != gp.arcMaps().end();
               ++it) {
            const nt::details::MapStorage< Arc >& map = (*it->second);

            stream.put(static_cast< char >(2));   // Map type (1 byte)

            const std::uint64_t l = std::strlen(it->first);
            details::writeScalar(stream, l);
            stream.write(it->first, l);                        // Map name
            stream.put(details::typeid2gttype(map.value()));   // value type (1 byte)

            for (ArcIt arc(g); arc != INVALID; ++arc) {
              const nt::AnyView any = map.get(arc);
              details::writeAny(any, stream);
            }
          }
        }

        template < bool BE, typename InputStream >
        void _readArcEdgeMap(GR&                    g,
                             const nt::String&      caption,
                             GraphProperties< GR >& gp,
                             uint8_t                type,
                             InputStream&           stream) const noexcept {
          if (nt::details::MapStorage< Arc >** p_map_storage = gp.arcMaps().findOrNull(caption.c_str())) {
            for (ArcIt arc(g); arc != INVALID; ++arc) {
              nt::AnyView any = (*p_map_storage)->get(arc);
              details::readAny< BE >(any, stream, type);
            }
          } else {
            for (ArcIt arc(g); arc != INVALID; ++arc) {
              nt::AnyView any;
              details::readAny< BE >(any, stream, type);
            }
          }
        }
      };
    }   // namespace details

    /**
     * @brief BinaryIo class for reading and writing graphs in binary format.
     *
     * This class provides functionality for reading and writing graph in binary format.
     * This format is based on https://graph-tool.skewed.de/static/doc/gt_format.html
     *
     * Example of usage:
     *
     * @code
     * using Digraph = nt::graphs::SmartDigraph;
     * using GraphProperties = nt::graphs::GraphProperties< Digraph >;
     *
     * Digraph digraph;
     * GraphProperties digraph_props;
     *
     * Digraph::NodeMap< nt::String > node_name(digraph);
     * digraph_props.nodeMap("name", node_name);
     *
     * node_name[v0] = "v0";
     * node_name[v1] = "v1";
     * node_name[v2] = "v2";
     * node_name[v3] = "v3";
     *
     * if (!bin_io.writeFile("mygraph.bin", digraph, &digraph_props))
     *  LOG_F(ERROR, "Error while writing file.");
     * @endcode
     *
     * @tparam GR The type of the graph.
     *
     */
    template < typename GR >
    struct BinaryIo: details::BinaryIoSelector< GR, nt::concepts::UndirectedTagIndicator< GR > > {
      static_assert(!nt::concepts::BuildTagIndicator< GR >, "BinaryIo does support graphs with a BuildTag");
      using Parent = details::BinaryIoSelector< GR, nt::concepts::UndirectedTagIndicator< GR > >;
      using Graph = GR;
      using Node = typename Parent::Node;
      using NodeIt = typename Parent::NodeIt;

      constexpr static const char* sz_expected_magic = "⛾ gt";

      /**
       * @brief Read a graph from a binary file
       *
       * @param sz_filename Name of the binary file to read from
       * @param g Reference to the graph
       * @param p_properties GraphProperties that contains the maps to be read
       *
       * @return
       */
      bool
         readFile(const char* sz_filename, Graph& g, GraphProperties< Graph >* p_properties = nullptr) const noexcept {
        nt::ByteBuffer buffer;
        if (!buffer.load(sz_filename)) return false;
        return read< nt::ByteBuffer >(buffer, g, p_properties);
      }

      /**
       * @brief Read the graph from a stream
       *
       * @tparam InputStream Type of the input stream
       *
       * @param stream Stream object
       * @param g Reference to the graph
       * @param p_properties GraphProperties that contains the maps to be read
       *
       * @return
       */
      template < typename InputStream >
      bool read(InputStream& stream, Graph& g, GraphProperties< Graph >* p_properties = nullptr) const noexcept {
        // Read & check magic (6 bytes)
        constexpr int i_magic_length = 6;
        char          magic[i_magic_length];
        stream.read(magic, i_magic_length);
        if (strncmp(magic, sz_expected_magic, i_magic_length) != 0) {
          stream.seekg(0);
          return false;
        }

        // Read version (1 byte)
        char byte;
        stream.read(&byte, 1);
        if (byte != 1) {   // Only version 1 allowed
          stream.seekg(0);
          return false;
        }

        // Read endianness (1 byte)
        stream.read(&byte, 1);
        const bool BE = byte;

        if (byte) return _readDispatch< true, InputStream >(stream, g, p_properties);
        return _readDispatch< false, InputStream >(stream, g, p_properties);
      }


      /**
       * @brief Write the graph into a file
       *
       * @param sz_filename Name of the binary file
       * @param g Reference to the graph to be written
       * @param p_properties GraphProperties that contains the maps to be written
       *
       * @return true
       */
      bool writeFile(const char*               sz_filename,
                     const Graph&              g,
                     GraphProperties< Graph >* p_properties = nullptr) const noexcept {
        nt::ByteBuffer buffer;
        return write< nt::ByteBuffer >(buffer, g, p_properties) && buffer.save(sz_filename);
      }

      /**
       * @brief Write the graph into a stream
       *
       * @tparam OutputStream Type of the output stream
       *
       * @param stream Stream object
       * @param g Reference to the graph to be written
       * @param p_properties GraphProperties that contains the maps to be written
       *
       * @return
       */
      template < typename OutputStream >
      bool
         write(OutputStream& stream, const Graph& g, GraphProperties< Graph >* p_properties = nullptr) const noexcept {
        // Write magic (6 bytes)
        stream.write(sz_expected_magic, std::strlen(sz_expected_magic));

        // Write version (1 byte)
        stream.put(static_cast< char >(1));

        // Write endianness (1 byte)
        // 0x00: little-endian, 0x01: big-endian
        stream.put(static_cast< char >(NT_IS_BIG_ENDIAN));

        // Write comment string size (8 bytes)
        details::writeScalar(stream, static_cast< std::uint64_t >(0));

        // Write directed byte (1 byte)
        stream.put(static_cast< char >(!nt::concepts::UndirectedTagIndicator< Graph >));

        // Write num nodes string size (8 bytes)
        const std::uint64_t i_num_nodes = g.nodeNum();
        details::writeScalar(stream, i_num_nodes);

        // write adjacency matrix
        if (i_num_nodes <= NT_UINT8_MAX)
          Parent::template _writeAdjacencyDispatch< uint8_t >(g, stream);
        else if (i_num_nodes <= NT_UINT16_MAX)
          Parent::template _writeAdjacencyDispatch< uint16_t >(g, stream);
        else if (i_num_nodes <= NT_UINT32_MAX)
          Parent::template _writeAdjacencyDispatch< uint32_t >(g, stream);
        else
          Parent::template _writeAdjacencyDispatch< uint64_t >(g, stream);

        // Write property maps
        if (p_properties) {
          // Write the total number of maps
          uint64_t i_num_maps = p_properties->attributes().size() + p_properties->nodeMaps().size();
          if constexpr (nt::concepts::UndirectedTagIndicator< Graph >)
            i_num_maps += p_properties->edgeMaps().size();
          else
            i_num_maps += p_properties->arcMaps().size();

          details::writeScalar(stream, i_num_maps);

          // Write the maps
          _writeNodeMaps(g, *p_properties, stream);
          Parent::_writeArcEdgeMaps(g, *p_properties, stream);
          _writeAttributes(g, *p_properties, stream);
        } else {
          details::writeScalar(stream, static_cast< std::uint64_t >(0));
        }

        stream.seekg(0);

        return true;
      }

      /**
       * @brief Validates binary graph file properties without loading data.
       *
       * @tparam InputStream Type of the input stream
       * @param buffer Input stream to validate
       * @param p_properties Properties to validate (currently not supported for binary format)
       * @return true (binary format validation not implemented yet)
       */
      template < typename InputStream >
      bool validateRead(InputStream& buffer, const GraphProperties< Graph >* p_properties = nullptr) const noexcept {
        // TODO: Implement binary format validation
        // For now, binary format validation is not implemented
        LOG_F(INFO, "Binary format validation not implemented - returning true");
        return true;
      }

      // Private

      /**
       * @brief Returns the minimum number of bytes needed to encode v
       *
       * @param v
       * @return int
       */
      constexpr static int _encodingSize(int v) noexcept {
        if (v <= NT_UINT8_MAX) return 1;
        if (v <= NT_UINT16_MAX) return 2;
        if (v <= NT_UINT32_MAX) return 4;
        return 8;
      }

      template < bool BE, typename InputStream >
      bool _readDispatch(InputStream& stream, Graph& g, GraphProperties< Graph >* p_properties) const noexcept {
        char byte;

        // Read comment string size (8 bytes)
        std::uint64_t i_comment_length = 0;
        details::readScalar< BE >(stream, i_comment_length);

        // skip comment
        stream.seekg(16 + i_comment_length);

        // Read directed byte (1 byte)
        stream.read(&byte, 1);
        if (byte == nt::concepts::UndirectedTagIndicator< Graph >) {
          LOG_F(ERROR,
                "Trying to read a directed (resp. undirected) graph to an undirected (resp. "
                "directed) graph");
          return false;
        }

        // Read num nodes (8 bytes)
        uint64_t i_num_nodes = 0;
        details::readScalar< BE >(stream, i_num_nodes);

        // Add the nodes
        g.clear();
        for (uint64_t u = 0; u < i_num_nodes; ++u)
          g.addNode();

        // Read adjacency matrix
        if (i_num_nodes <= NT_UINT8_MAX)
          Parent::template _readAdjacencyDispatch< BE, uint8_t >(g, i_num_nodes, stream);
        else if (i_num_nodes <= NT_UINT16_MAX)
          Parent::template _readAdjacencyDispatch< BE, uint16_t >(g, i_num_nodes, stream);
        else if (i_num_nodes <= NT_UINT32_MAX)
          Parent::template _readAdjacencyDispatch< BE, uint32_t >(g, i_num_nodes, stream);
        else
          Parent::template _readAdjacencyDispatch< BE, uint64_t >(g, i_num_nodes, stream);

        // Read property map
        if (p_properties) {
          nt::String caption;

          //  Get the total number of property maps
          uint64_t i_num_maps = 0;
          details::readScalar< BE >(stream, i_num_maps);

          for (int k = 0; k < i_num_maps; ++k) {
            stream.read(&byte, 1);   // Map type (1 byte) : Node/Arc/Graph
            const char map_type = byte;
            details::readString< BE >(stream, caption);   // Name of the property map (length bytes)
            stream.read(&byte, 1);                        // Value type index (1 byte) : bool, int, etc...

            switch (map_type) {
              case 0:   // graph property map
                _readAttribute< BE >(g, caption, *p_properties, byte, stream);
                break;
              case 1:   // node map
                _readNodeMap< BE >(g, caption, *p_properties, byte, stream);
                break;
              case 2:   // arc/edge map
                Parent::template _readArcEdgeMap< BE >(g, caption, *p_properties, byte, stream);
                break;
              default: stream.seekg(0); return false;
            }
          }
        }

        stream.seekg(0);

        return true;
      }

      template < typename InputStream >
      void _writeNodeMaps(const Graph& g, const GraphProperties< Graph >& gp, InputStream& stream) const noexcept {
        for (typename GraphProperties< Graph >::NodeMaps::const_iterator it = gp.nodeMaps().begin();
             it != gp.nodeMaps().end();
             ++it) {
          const nt::details::MapStorage< Node >& map = (*it->second);

          stream.put(static_cast< char >(1));   // Map type (1 byte)

          const std::uint64_t l = std::strlen(it->first);
          details::writeScalar(stream, l);
          stream.write(it->first, l);                        // Map name
          stream.put(details::typeid2gttype(map.value()));   // value type (1 byte)

          for (NodeIt u(g); u != INVALID; ++u) {
            const nt::AnyView any = map.get(u);
            details::writeAny(any, stream);
          }
        }
      }

      template < typename InputStream >
      void _writeAttributes(const Graph& g, const GraphProperties< Graph >& gp, InputStream& stream) const noexcept {
        for (typename GraphProperties< Graph >::Attributes::const_iterator it = gp.attributes().begin();
             it != gp.attributes().end();
             ++it) {
          const nt::AnyView& any = it->second;

          stream.put(static_cast< char >(0));   // Attribute type (1 byte)
          const std::uint64_t l = std::strlen(it->first);
          details::writeScalar(stream, l);
          stream.write(it->first, l);                       // Attribute name
          stream.put(details::typeid2gttype(any.type()));   // value type (1 byte)
          details::writeAny(any, stream);
        }
      }

      template < bool BE, typename InputStream >
      void _readNodeMap(Graph&                    g,
                        const nt::String&         caption,
                        GraphProperties< Graph >& gp,
                        uint8_t                   type,
                        InputStream&              stream) const noexcept {
        if (nt::details::MapStorage< Node >** p_map_storage = gp.nodeMaps().findOrNull(caption.c_str())) {
          for (NodeIt u(g); u != INVALID; ++u) {
            nt::AnyView any = (*p_map_storage)->get(u);
            details::readAny< BE >(any, stream, type);
          }
        } else {
          for (NodeIt u(g); u != INVALID; ++u) {
            nt::AnyView any;
            details::readAny< BE >(any, stream, type);
          }
        }
      }

      template < bool BE, typename InputStream >
      void _readAttribute(const Graph&              g,
                          const nt::String&         caption,
                          GraphProperties< Graph >& gp,
                          uint8_t                   type,
                          InputStream&              stream) const noexcept {
        if (nt::AnyView* p_any = gp.attributes().findOrNull(caption.c_str()))
          details::readAny< BE >(*p_any, stream, type);
        else {
          nt::AnyView any;
          details::readAny< BE >(any, stream, type);
        }
      }
    };

  }   // namespace graphs
}   // namespace nt

#endif
