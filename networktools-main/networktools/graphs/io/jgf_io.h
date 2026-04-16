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
 * @brief JGF file format reader.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_JGF_IO_H_
#define _NT_JGF_IO_H_

#include "../../core/json.h"
#include "../../core/maps.h"
#include "../graph_properties.h"

#define RETURN_ON_ERROR(x) \
  if (!(x)) return false

namespace nt {
  namespace graphs {

    /**
     * @brief
     *
     */
    struct JgfVisitor {
      JgfVisitor() = default;
      virtual ~JgfVisitor() noexcept {}

      /**
       * @brief
       *
       * @param i_num_nodes
       * @param i_num_edges
       * @param sz_id
       * @param b_directed
       * @param sz_label
       * @param sz_type
       * @param p_metadata
       * @return true
       * @return false
       */
      virtual bool start(int                        i_num_nodes,
                         int                        i_num_edges,
                         const char*                sz_id,
                         bool                       b_directed,
                         const char*                sz_label,
                         const char*                sz_type,
                         const nt::JsonConstObject* p_metadata) {
        return true;
      }

      /**
       * @brief
       *
       * @param i_node
       * @param sz_name
       * @param sz_label
       * @param p_metadata
       * @return true
       * @return false
       */
      virtual bool
         processNode(int i_node, const char* sz_name, const char* sz_label, const nt::JsonConstObject* p_metadata) {
        return true;
      }

      /**
       * @brief
       *
       * @param i_num
       * @param i_source
       * @param i_target
       * @param sz_source
       * @param sz_target
       * @param sz_id
       * @param sz_relation
       * @param b_directed
       * @param sz_label
       * @param p_metadata
       * @return true
       * @return false
       */
      virtual bool processEdge(int                        i_num,
                               int                        i_source,
                               int                        i_target,
                               const char*                sz_source,
                               const char*                sz_target,
                               const char*                sz_id,
                               const char*                sz_relation,
                               bool                       b_directed,
                               const char*                sz_label,
                               const nt::JsonConstObject* p_metadata) {
        return true;
      }

      /**
       * @brief
       *
       * @param i_num
       * @param sources
       * @param targets
       * @param source_names
       * @param target_names
       * @param sz_id
       * @param sz_relation
       * @param sz_label
       * @param p_metadata
       * @return true
       * @return false
       */
      virtual bool processDirectedHyperedge(int                                           i_num,
                                            nt::IntArrayView                              sources,
                                            nt::IntArrayView                              targets,
                                            const nt::TrivialDynamicArray< const char* >& source_names,
                                            const nt::TrivialDynamicArray< const char* >& target_names,
                                            const char*                                   sz_id,
                                            const char*                                   sz_relation,
                                            const char*                                   sz_label,
                                            const nt::JsonConstObject*                    p_metadata) {
        return true;
      }

      /**
       * @brief
       *
       * @param i_num
       * @param vertices
       * @param node_names
       * @param sz_id
       * @param sz_relation
       * @param sz_label
       * @param p_metadata
       * @return true
       * @return false
       */
      virtual bool processUndirectedHyperedge(int                                           i_num,
                                              nt::IntArrayView                              vertices,
                                              const nt::TrivialDynamicArray< const char* >& node_names,
                                              const char*                                   sz_id,
                                              const char*                                   sz_relation,
                                              const char*                                   sz_label,
                                              const nt::JsonConstObject*                    p_metadata) {
        return true;
      }

      /**
       * @brief
       *
       * @return true
       * @return false
       */
      virtual bool finish() { return true; }
    };

    /**
     * @brief
     *
     */
    struct JgfTraversal {
      enum E_EDGE_TYPE { E_EDGES = 0, E_HYPEREDGES };

      nt::JSONDocument _doc;

      nt::CstringMap< int >                  _node_name_map;
      nt::IntDynamicArray                    _sourceHyperedgeVertices;
      nt::IntDynamicArray                    _targetHyperedgeVertices;
      nt::TrivialDynamicArray< const char* > _sourceHyperedgeNames;
      nt::TrivialDynamicArray< const char* > _targetHyperedgeNames;

      /**
       * @brief
       *
       * @param visitor
       * @param document
       * @return true
       * @return false
       */
      template < typename InputStream >
      bool run(InputStream& buffer, JgfVisitor& visitor) noexcept {
        using RapidJsonInputStream = rapidjson::BasicIStreamWrapper< InputStream >;
        RapidJsonInputStream wrapped_buffer(buffer);
        _doc.ParseStream(wrapped_buffer);

        if (!_isDocumentValid()) return false;

        if (_doc.HasMember("graphs")) {
          const auto& json_graphs = _doc["graphs"].GetArray();
          for (const auto& json_graph: json_graphs)
            RETURN_ON_ERROR(_readGraph(visitor, json_graph.GetObject()));
          return true;
        }

        return _readGraph(visitor, std::as_const(_doc)["graph"].GetObject());
      }

      // Private methods

      bool _isDocumentValid() noexcept {
        nt::JSONDocument sd;
        const char*      szJsonGraphFormatSchemaV2 =
           "{ \"$schema\": \"http://json-schema.org/draft-07/schema#\", \"$id\": "
           "\"http://jsongraphformat.info/v2.1/json-graph-schema.json\", \"title\": "
           "\"JSON_DOCUMENT Graph Schema\", \"oneOf\": [ { \"type\": \"object\", \"properties\": { "
           "\"graph\": { \"$ref\": \"#/definitions/graph\" } }, "
           "\"additionalProperties\": false, \"required\": [ \"graph\" ] }, { \"type\": "
           "\"object\", \"properties\": { \"graphs\": { \"type\": \"array\", "
           "\"items\": { \"$ref\": \"#/definitions/graph\" } } }, \"additionalProperties\": false "
           "} ], \"definitions\": { \"graph\": { \"oneOf\": [ { "
           "\"type\": \"object\", \"additionalProperties\": false, \"properties\": { \"id\": { "
           "\"type\": \"string\" }, \"label\": { \"type\": \"string\" }, "
           "\"directed\": { \"type\": [ \"boolean\" ], \"default\": true }, \"type\": { \"type\": "
           "\"string\" }, \"metadata\": { \"type\": [ \"object\" ] }, "
           "\"nodes\": { \"type\": \"object\", \"additionalProperties\": { \"$ref\": "
           "\"#/definitions/node\" } }, \"edges\": { \"type\": [ \"array\" ], "
           "\"items\": { \"$ref\": \"#/definitions/edge\" } } } }, { \"type\": \"object\", "
           "\"additionalProperties\": false, \"properties\": { \"id\": { "
           "\"type\": \"string\" }, \"label\": { \"type\": \"string\" }, \"directed\": { \"type\": "
           "[ \"boolean\" ], \"default\": true }, \"type\": { \"type\": "
           "\"string\" }, \"metadata\": { \"type\": [ \"object\" ] }, \"nodes\": { \"type\": "
           "\"object\", \"additionalProperties\": { \"$ref\": "
           "\"#/definitions/node\" } }, \"hyperedges\": { \"type\": [ \"array\" ], \"items\": { "
           "\"$ref\": \"#/definitions/directedhyperedge\" } } } }, { "
           "\"type\": \"object\", \"additionalProperties\": false, \"properties\": { \"id\": { "
           "\"type\": \"string\" }, \"label\": { \"type\": \"string\" }, "
           "\"directed\": { \"type\": [ \"boolean\" ], \"enum\": [false] }, \"type\": { \"type\": "
           "\"string\" }, \"metadata\": { \"type\": [ \"object\" ] }, "
           "\"nodes\": { \"type\": \"object\", \"additionalProperties\": { \"$ref\": "
           "\"#/definitions/node\" } }, \"hyperedges\": { \"type\": [ \"array\" ], "
           "\"items\": { \"$ref\": \"#/definitions/undirectedhyperedge\" } } }, \"required\": [ "
           "\"directed\" ] } ] }, \"node\": { \"type\": \"object\", "
           "\"properties\": { \"label\": { \"type\": \"string\" }, \"metadata\": { \"type\": "
           "\"object\" }, \"additionalProperties\": false } }, \"edge\": { "
           "\"type\": \"object\", \"additionalProperties\": false, \"properties\": { \"id\": { "
           "\"type\": \"string\" }, \"source\": { \"type\": \"string\" }, "
           "\"target\": { \"type\": \"string\" }, \"relation\": { \"type\": \"string\" }, "
           "\"directed\": { \"type\": [ \"boolean\" ], \"default\": true }, "
           "\"label\": { \"type\": \"string\" }, \"metadata\": { \"type\": [ \"object\" ] } }, "
           "\"required\": [ \"source\", \"target\" ] }, "
           "\"directedhyperedge\": { \"type\": \"object\", \"additionalProperties\": false, "
           "\"properties\": { \"id\": { \"type\": \"string\" }, \"source\": { "
           "\"type\": \"array\", \"items\": { \"type\": \"string\" } }, \"target\": { \"type\": "
           "\"array\", \"items\": { \"type\": \"string\" } }, "
           "\"relation\": { \"type\": \"string\" }, \"label\": { \"type\": \"string\" }, "
           "\"metadata\": { \"type\": [ \"object\" ] } }, \"required\": [ "
           "\"source\", \"target\" ] }, \"undirectedhyperedge\": { \"type\": \"object\", "
           "\"additionalProperties\": false, \"properties\": { \"id\": { "
           "\"type\": \"string\" }, \"nodes\": { \"type\": \"array\", \"items\": { \"type\": "
           "\"string\" } }, \"relation\": { \"type\": \"string\" }, "
           "\"label\": { \"type\": \"string\" }, \"metadata\": { \"type\": [ \"object\" ] } }, "
           "\"required\": [ \"nodes\" ] } } }";
        if (sd.Parse(szJsonGraphFormatSchemaV2).HasParseError()) { return false; }
        rapidjson::SchemaDocument schema(sd);

        rapidjson::SchemaValidator validator(schema);
        if (!_doc.Accept(validator)) {
          rapidjson::StringBuffer sb;
          validator.GetInvalidSchemaPointer().StringifyUriFragment(sb);
          LOG_F(ERROR, "Invalid schema: {}", sb.GetString());
          LOG_F(ERROR, "Invalid keyword: {}", validator.GetInvalidSchemaKeyword());
          sb.Clear();
          validator.GetInvalidDocumentPointer().StringifyUriFragment(sb);
          LOG_F(ERROR, "Invalid document: {}", sb.GetString());
          return false;
        }
        return true;
      }

      bool _readEdges(JgfVisitor& visitor, const nt::JsonConstArray& json_edges) noexcept {
        int i_num = 0;
        for (const auto& current_edge: json_edges) {
          const char* sz_source = current_edge["source"].GetString();
          int*        p_source = _node_name_map.findOrNull(sz_source);
          if (!p_source) {
            LOG_F(ERROR, "Invalid document: node source {} of edge {} does not exist.", sz_source, i_num);
            return false;
          }

          const char* sz_target = current_edge["target"].GetString();
          int*        p_target = _node_name_map.findOrNull(sz_target);
          if (!p_target) {
            LOG_F(ERROR, "Invalid document: node target {} of edge {} does not exist.", sz_target, i_num);
            return false;
          }

          const char* sz_label = nullptr;
          if (current_edge.HasMember("label")) sz_label = current_edge["label"].GetString();

          bool b_directed = true;
          if (current_edge.HasMember("directed")) b_directed = current_edge["label"].GetBool();

          const char* sz_relation = nullptr;
          if (current_edge.HasMember("relation")) sz_relation = current_edge["relation"].GetString();

          const char* sz_id = nullptr;
          if (current_edge.HasMember("id")) sz_id = current_edge["id"].GetString();

          if (current_edge.HasMember("metadata")) {
            const auto& metadata = current_edge["metadata"].GetObject();
            RETURN_ON_ERROR(visitor.processEdge(
               i_num, *p_source, *p_target, sz_source, sz_target, sz_id, sz_relation, b_directed, sz_label, &metadata));
          } else
            RETURN_ON_ERROR(visitor.processEdge(
               i_num, *p_source, *p_target, sz_source, sz_target, sz_id, sz_relation, b_directed, sz_label, nullptr));
          ++i_num;
        }

        return true;
      }

      bool _jsonNodeArrayToNodeIdxArray(nt::IntDynamicArray&                    idx_array,
                                        nt::TrivialDynamicArray< const char* >& names,
                                        const nt::JsonConstArray&               json_array) noexcept {
        idx_array.removeAll();
        idx_array.ensure(json_array.Size());
        names.removeAll();
        names.ensure(json_array.Size());
        for (const auto& json_string: json_array) {
          const char* sz_node_name = json_string.GetString();
          int*        p_node = _node_name_map.findOrNull(sz_node_name);
          if (!p_node) {
            LOG_F(ERROR, "Invalid document: node {} does not exist.", sz_node_name);
            return false;
          }
          idx_array.add(*p_node);
          names.add(sz_node_name);
        }
        return true;
      }

      bool _readHyperedges(JgfVisitor& visitor, const nt::JsonConstArray& json_edges) noexcept {
        int  i_num = 0;
        bool b_is_directed = true;
        for (const auto& current_edge: json_edges) {
          const char* sz_label = nullptr;
          if (current_edge.HasMember("label")) sz_label = current_edge["label"].GetString();

          const char* sz_relation = nullptr;
          if (current_edge.HasMember("relation")) sz_relation = current_edge["relation"].GetString();

          const char* sz_id = nullptr;
          if (current_edge.HasMember("id")) sz_id = current_edge["id"].GetString();

          // Undirected hyperedge case
          if (current_edge.HasMember("nodes")) {
            b_is_directed = false;
            RETURN_ON_ERROR(_jsonNodeArrayToNodeIdxArray(
               _sourceHyperedgeVertices, _sourceHyperedgeNames, current_edge["nodes"].GetArray()));
            // Directed hyperedge case
          } else if (current_edge.HasMember("source") && current_edge.HasMember("target")) {
            b_is_directed = true;
            RETURN_ON_ERROR(_jsonNodeArrayToNodeIdxArray(
               _sourceHyperedgeVertices, _sourceHyperedgeNames, current_edge["source"].GetArray()));
            RETURN_ON_ERROR(_jsonNodeArrayToNodeIdxArray(
               _targetHyperedgeVertices, _targetHyperedgeNames, current_edge["target"].GetArray()));
          } else {
            assert(false);   // Should never happen in a valid JGF json file
            return false;
          }

          if (current_edge.HasMember("metadata")) {
            const auto& metadata = current_edge["metadata"].GetObject();
            if (b_is_directed) {
              RETURN_ON_ERROR(visitor.processDirectedHyperedge(i_num,
                                                               _sourceHyperedgeVertices,
                                                               _targetHyperedgeVertices,
                                                               _sourceHyperedgeNames,
                                                               _targetHyperedgeNames,
                                                               sz_id,
                                                               sz_relation,
                                                               sz_label,
                                                               &metadata));
            } else {
              RETURN_ON_ERROR(visitor.processUndirectedHyperedge(
                 i_num, _sourceHyperedgeVertices, _sourceHyperedgeNames, sz_id, sz_relation, sz_label, &metadata));
            }
          } else {
            if (b_is_directed) {
              RETURN_ON_ERROR(visitor.processDirectedHyperedge(i_num,
                                                               _sourceHyperedgeVertices,
                                                               _targetHyperedgeVertices,
                                                               _sourceHyperedgeNames,
                                                               _targetHyperedgeNames,
                                                               sz_id,
                                                               sz_relation,
                                                               sz_label,
                                                               nullptr));
            } else {
              RETURN_ON_ERROR(visitor.processUndirectedHyperedge(
                 i_num, _sourceHyperedgeVertices, _sourceHyperedgeNames, sz_id, sz_relation, sz_label, nullptr));
            }
          }

          ++i_num;
        }

        return true;
      }

      bool _readGraph(JgfVisitor& jgf_visitor, const nt::JsonConstObject& json_graph) noexcept {
        _node_name_map.clear();

        int         i_num_edges = 0;
        E_EDGE_TYPE e_edge_type;
        if (json_graph.HasMember("edges")) {
          e_edge_type = E_EDGES;
          i_num_edges = json_graph["edges"].GetArray().Size();
        } else if (json_graph.HasMember("hyperedges")) {
          e_edge_type = E_HYPEREDGES;
          i_num_edges = json_graph["hyperedges"].GetArray().Size();
        } else {
          assert(false);   // Should never happen in a valid JGF json file
          return false;
        }

        const auto& json_nodes = json_graph["nodes"].GetObject();

        const char* sz_id = nullptr;
        if (json_graph.HasMember("id")) sz_id = json_graph["id"].GetString();

        const char* szLabel = nullptr;
        if (json_graph.HasMember("label")) szLabel = json_graph["label"].GetString();

        const char* szType = nullptr;
        if (json_graph.HasMember("type")) szType = json_graph["type"].GetString();

        bool b_directed = true;
        if (json_graph.HasMember("directed")) b_directed = json_graph["directed"].GetBool();

        if (json_graph.HasMember("metadata")) {
          const auto& Metadata = json_graph["metadata"].GetObject();
          RETURN_ON_ERROR(
             jgf_visitor.start(json_nodes.MemberCount(), i_num_edges, sz_id, b_directed, szLabel, szType, &Metadata));
        } else {
          RETURN_ON_ERROR(
             jgf_visitor.start(json_nodes.MemberCount(), i_num_edges, sz_id, b_directed, szLabel, szType, nullptr));
        }

        int i_num = 0;
        for (const auto& CurrentNode: json_nodes) {
          const auto& properties = CurrentNode.value.GetObject();
          const char* sz_name = CurrentNode.name.GetString();   // According to the JGS spec v2, this is a unique
                                                                // identifier for the node
          // (https://github.com/json_graph/json-graph-specification#nodes-object).

          if (!_node_name_map.insertNoDuplicate({sz_name, i_num})) {
            assert(false);   // Should never happen in a valid JGF json file
            return false;
          }

          const char* sz_label = nullptr;
          if (properties.HasMember("label")) sz_label = properties["label"].GetString();

          if (properties.HasMember("metadata")) {
            const auto& metadata = properties["metadata"].GetObject();
            RETURN_ON_ERROR(jgf_visitor.processNode(i_num, sz_name, sz_label, &metadata));
          } else
            RETURN_ON_ERROR(jgf_visitor.processNode(i_num, sz_name, sz_label, nullptr));
          ++i_num;
        }

        bool b_status = true;
        switch (e_edge_type) {
          case E_EDGES: b_status = _readEdges(jgf_visitor, json_graph["edges"].GetArray()); break;
          case E_HYPEREDGES: b_status = _readHyperedges(jgf_visitor, json_graph["hyperedges"].GetArray()); break;
          default:
            assert(false);
            b_status = false;
            break;
        }

        RETURN_ON_ERROR(b_status);
        RETURN_ON_ERROR(jgf_visitor.finish());

        return true;
      }
    };

    /**
     * @brief JSON Graph format reader for directed graphs.
     *
     *
     * @tparam GR
     */
    template < typename GR >
    struct JgfIO {
      using Graph = GR;
      TEMPLATE_DIGRAPH_TYPEDEFS(Graph);

      struct JgfIOVisitor: graphs::JgfVisitor {
        Graph&                    _digraph;
        GraphProperties< Graph >* _p_properties;

        JgfIOVisitor(Graph& digraph, GraphProperties< Graph >* p_properties = nullptr) :
            _digraph(digraph), _p_properties(p_properties) {}

        bool start(int i_num_nodes,
                   int i_num_edges,
                   const char*,
                   bool,
                   const char*,
                   const char*,
                   const nt::JsonConstObject* p_metadata) override {
          _digraph.clear();
          _digraph.reserveNode(i_num_nodes);
          LinkView< Graph >::reserveLink(_digraph, i_num_edges);

          if (p_metadata && _p_properties) {
            for (const auto& member: (*p_metadata)) {
              const char* sz_attr_name = member.name.GetString();
              if (auto* p_any = _p_properties->_attributes.findOrNull(sz_attr_name)) {
                const bool r = nt::details::setJsonValueToAny(*p_any, member.value);
                if (!r) {
                  LOG_F(ERROR, "Property type of '{}' is not supported.", sz_attr_name);
                  return false;
                }
              }
            }
          }

          return true;
        }

        bool processNode(int, const char*, const char*, const nt::JsonConstObject* p_metadata) override {
          const Node node = _digraph.addNode();

          if (p_metadata && _p_properties) {
            for (const auto& member: (*p_metadata)) {
              const char* sz_attr_name = member.name.GetString();
              if (const auto* p_map_storage = _p_properties->_node_maps.findOrNull(sz_attr_name)) {
                if (!nt::details::setJsonValueToMap< Node >(**p_map_storage, node, member.value)) {
                  LOG_F(ERROR, "Property type of '{}' is not supported.", sz_attr_name);
                  return false;
                }
              }
            }
          }

          return true;
        }

        bool processEdge(int,
                         int i_source,
                         int i_target,
                         const char*,
                         const char*,
                         const char*,
                         const char*,
                         bool,
                         const char*,
                         const nt::JsonConstObject* p_metadata) override {
          using Link = typename LinkView< Graph >::Link;

          const Link link =
             LinkView< Graph >::addLink(_digraph, Graph::nodeFromId(i_source), Graph::nodeFromId(i_target));

          if (p_metadata && _p_properties) {
            for (const auto& member: (*p_metadata)) {
              const char* sz_attr_name = member.name.GetString();
              if (const auto* p_map_storage = _p_properties->linkMaps().findOrNull(sz_attr_name)) {
                if (!nt::details::setJsonValueToMap< Link >(**p_map_storage, link, member.value)) {
                  LOG_F(ERROR, "Property type of '{}' is not supported.", sz_attr_name);
                  return false;
                }
              }
            }
          }

          return true;
        }
      };

      /**
       * @brief Reads the graph from a file.
       *
       * @param sz_filename Name of the file to read from
       * @param g
       * @param p_properties
       * @return True if the read operation was successful, false otherwise.
       */
      bool readFile(const char* sz_filename, Graph& g, GraphProperties< Graph >* p_properties = nullptr) noexcept {
        nt::ByteBuffer buffer;
        if (!buffer.load(sz_filename)) return false;
        return read< nt::ByteBuffer >(buffer, g, p_properties);
      }

      /**
       * @brief Reads the graph from a stream.
       *
       * @return True if the read operation was successful, false otherwise.
       */
      template < typename InputStream >
      bool read(InputStream& buffer, Graph& g, GraphProperties< Graph >* p_properties = nullptr) noexcept {
        JgfIOVisitor visitor(g, p_properties);
        return JgfTraversal().run(buffer, visitor);
      }


      /**
       * @brief Writes the graph to a file.
       *
       * @param sz_filename
       * @param g
       * @param p_properties
       * @return True if the write operation was successful, false otherwise.
       */
      bool writeFile(const char* sz_filename, Graph& g, GraphProperties< Graph >* p_properties = nullptr) noexcept {
        nt::ByteBuffer buffer;
        return write< nt::ByteBuffer >(buffer, g, p_properties) && buffer.save(sz_filename);
      }

      /**
       * @brief Writes the graph to a stream.
       *
       * @return True if the read operation was successful, false otherwise.
       */
      template < typename OutputStream >
      bool write(OutputStream& buffer, Graph& g, GraphProperties< Graph >* p_properties = nullptr) noexcept {
        assert(false);   // Not yet implemented
        return false;
      }
    };
    //---------------------------------------------------------------------------------------
  }   // namespace graphs
}   // namespace nt
#endif