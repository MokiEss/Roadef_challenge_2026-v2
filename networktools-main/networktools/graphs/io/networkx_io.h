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
 * @brief NetworkxIo class for reading and writing NetworkX-compatible JSON files.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_NETWORKX_IO_H_
#define _NT_NETWORKX_IO_H_

#include <fstream>

#include "../../core/json.h"
#include "../tools.h"
#include "../edge_graph.h"
#include "../graph_properties.h"

namespace nt {
  namespace graphs {

    /**
     * @brief NetworkxIo class for reading and writing NetworkX-compatible JSON files.
     *
     * This class provides functionality for reading and writing NetworkX-compatible JSON files that
     * represent graphs.
     *
     * Example of usage:
     *
     * @code
     *  using Digraph = nt::graphs::SmartDigraph;
     *  Digraph g;
     *
     *  NetworkxIo<Digraph> nx_json_io;
     *
     *  nx_json_io.readFile("graph.json", g);
     * @endcode
     *
     * @tparam GR The type of the graph.
     */
    template < typename GR >
    struct NetworkxIo {
      static_assert(!nt::concepts::BuildTagIndicator< GR >, "NetworkxIo does not support graphs with a BuildTag");
      using Graph = GR;
      TEMPLATE_DIGRAPH_TYPEDEFS(Graph);

      using IdNodeMap = nt::NumericalMap< uint64_t, Node >;

      nt::JSONDocument _doc;
      IdNodeMap        _id_node_map;

      // Attributes name for storing NetworkX-internal graph data
      nt::String _source_attr;
      nt::String _target_attr;
      nt::String _name_attr;
      nt::String _key_attr;
      nt::String _link_attr;

      /**
       * @brief Constructs a NetworkxIo object with default options.
       *
       */
      NetworkxIo() noexcept :
          _source_attr("source"), _target_attr("target"), _name_attr("id"), _key_attr("key"), _link_attr("links") {}


      /**
       * @brief Constructs a NetworkxIo object with attribute names.
       *
       * @param sz_source_attr The attribute name for the source of the links.
       * @param sz_target_attr The attribute name for the target of the links.
       */
      NetworkxIo(const char* sz_source_attr, const char* sz_target_attr) noexcept :
          _source_attr(sz_source_attr), _target_attr(sz_target_attr), _name_attr("id"), _key_attr("key"),
          _link_attr("links") {}

      /**
       * @brief Constructs a NetworkxIo object with attribute names.
       *
       * @param sz_source_attr The attribute name for the source of the links.
       * @param sz_target_attr The attribute name for the target of the links.
       * @param sz_name_attr The attribute name for the node IDs.
       * @param sz_key_attr The attribute name for the link keys.
       * @param sz_link_attr The attribute name for the links.
       */
      NetworkxIo(const char* sz_source_attr,
                 const char* sz_target_attr,
                 const char* sz_name_attr,
                 const char* sz_key_attr,
                 const char* sz_link_attr) noexcept :
          _source_attr(sz_source_attr),
          _target_attr(sz_target_attr), _name_attr(sz_name_attr), _key_attr(sz_key_attr), _link_attr(sz_link_attr) {}

      /**
       * @brief Reads the graph from a file.
       *
       * @tparam b_abort_on_mismatch_type If true, the function will return false if the orientation of
       * the read graph is different from the orientation of the provided template graph parameter.
       *
       * @param sz_filename Name of the file to read from
       * @param g A directed graph
       * @param p_properties Pointer to a GraphProperties object that will store the read arc/node properties
       * @return True if the read operation was successful, false otherwise.
       */
      template < bool b_abort_on_mismatch_type = true >
      bool readFile(const char* sz_filename, Graph& g, GraphProperties< Graph >* p_properties = nullptr) noexcept {
        nt::ByteBuffer buffer;
        if (!buffer.load(sz_filename)) return false;
        return read< nt::ByteBuffer, b_abort_on_mismatch_type >(buffer, g, p_properties);
      }

      /**
       * @brief Reads the graph from a stream.
       *
       * Reads the graph structure and attributes from a NetworkX-compatible JSON stream.
       *
       * @tparam InputStream Type of the input stream
       * @tparam b_abort_on_mismatch_type If true, the function will return false if the orientation of
       * the read graph is different from the orientation of the provided template graph parameter.
       *
       * @param buffer Input stream to read from
       * @param g A directed graph
       * @param p_properties Pointer to a GraphProperties object that will store the read arc/node properties
       * @return True if the read operation was successful, false otherwise.
       *
       */
      template < typename InputStream, bool b_abort_on_mismatch_type = true >
      bool read(InputStream& buffer, Graph& g, GraphProperties< Graph >* p_properties = nullptr) noexcept {
        using RapidJsonInputStream = rapidjson::BasicIStreamWrapper< InputStream >;
        buffer.seekg(0);
        RapidJsonInputStream wrapped_buffer(buffer);
        return _openDocumentFromStream(wrapped_buffer) && _readDocument< b_abort_on_mismatch_type >(g, p_properties);
      }

      /**
       * @brief Read only the links from a JSON file. It must be called after a call to readFile() or read()
       *
       * @param sz_filename Name of the file to read from
       * @param g A directed graph
       * @param p_properties Pointer to a GraphProperties object that will store the read arc/node properties
       * @return True if the read operation was successful, false otherwise.
       */
      template < typename GR0 >
      bool readLinksFromFile(const char* sz_filename, GR0& g, GraphProperties< GR0 >* p_properties = nullptr) noexcept {
        nt::ByteBuffer buffer;
        if (!buffer.load(sz_filename)) return false;
        return readLinks< nt::ByteBuffer, GR0 >(buffer, g, p_properties);
      }

      /**
       * @brief Reads only the links from a stream. It must be called after a call to readFile() or read()
       *
       * @tparam InputStream Type of the input stream
       * @tparam GR0 Type of the directed graph
       *
       * @param buffer Input stream to read from
       * @param g A directed graph
       * @param p_properties Pointer to a GraphProperties object that will store the read arc/node properties
       * @return True if the read operation was successful, false otherwise.
       *
       */
      template < typename InputStream, typename GR0 >
      bool readLinks(InputStream& buffer, GR0& g, GraphProperties< GR0 >* p_properties = nullptr) noexcept {
        using RapidJsonInputStream = rapidjson::BasicIStreamWrapper< InputStream >;
        buffer.seekg(0);
        RapidJsonInputStream wrapped_buffer(buffer);
        return _openDocumentFromStream(wrapped_buffer) && _readAttributes< GR0 >(p_properties)
               && _readLinkSet(g, p_properties);
      }

      /**
       * @brief Validates graph file properties without loading data.
       *
       * Performs a dry-run validation of the NetworkX JSON file to check:
       * - File format validity and structure
       * - Presence of requested node and link properties/attributes
       * - Data type compatibility with requested maps
       *
       * This function logs detailed information about missing or incompatible fields
       * at TRACE and DEBUG verbosity levels, allowing developers to identify
       * discrepancies between expected and available data.
       *
       * @tparam InputStream Type of the input stream
       * @param buffer Input stream to validate
       * @param p_properties Pointer to GraphProperties containing the filters/maps to validate
       * @return true if all requested properties are available and compatible, false if any are missing
       */
      template < typename InputStream >
      bool validateRead(InputStream& buffer, const GraphProperties< Graph >* p_properties = nullptr) noexcept {
        using RapidJsonInputStream = rapidjson::BasicIStreamWrapper< InputStream >;
        buffer.seekg(0);
        RapidJsonInputStream wrapped_buffer(buffer);

        if (!_openDocumentFromStream(wrapped_buffer)) {
          LOG_F(ERROR, "Failed to parse JSON document for validation");
          return false;
        }

        return _validateDocument(p_properties);
      }

      /**
       * @brief Writes the graph structure and attributes to a NetworkX-compatible JSON file.
       *
       * @param sz_filename Name of the file to write to
       * @param g A directed graph
       * @param p_properties Pointer to a GraphProperties object that will store the read arc/node properties
       * @return True if the read operation was successful, false otherwise.
       */
      bool writeFile(const char*                     sz_filename,
                     const Graph&                    g,
                     const GraphProperties< Graph >* p_properties = nullptr) noexcept {
        nt::ByteBuffer buffer;
        return write< nt::ByteBuffer >(buffer, g, p_properties) && buffer.save(sz_filename);
      }

      /**
       * @brief Writes the graph structure and attributes to a NetworkX-compatible JSON stream.
       *
       * @return The JSON data as a string if the write operation was successful, an empty string
       * otherwise.
       */
      template < typename OutputStream >
      bool
         write(OutputStream& buffer, const Graph& g, const GraphProperties< Graph >* p_properties = nullptr) noexcept {
        using RapidJsonOutputStream = rapidjson::BasicOStreamWrapper< OutputStream >;

        if (!_writeDocument(g, p_properties)) return false;
        RapidJsonOutputStream                      wrapped_buffer(buffer);
        rapidjson::Writer< RapidJsonOutputStream > writer(wrapped_buffer);
        _doc.Accept(writer);
        return true;
      }

      /**
       * @brief Sets the attribute name for the source of the links.
       *
       * @param sz_source_attr The attribute name for the source of the links.
       */
      void setSourceAttr(const char* sz_source_attr) noexcept { _source_attr.assign(sz_source_attr); }

      /**
       * @brief Sets the attribute name for the target of the links.
       *
       * @param sz_target_attr The attribute name for the target of the links.
       */
      void setTargetAttr(const char* sz_target_attr) noexcept { _target_attr.assign(sz_target_attr); }

      /**
       * @brief Sets the attribute name for the name of the nodes.
       *
       * @param sz_name_attr The attribute name for the name of the nodes.
       */
      void setNameAttr(const char* sz_name_attr) noexcept { _name_attr.assign(sz_name_attr); }

      /**
       * @brief Sets the attribute name for the key of the links.
       *
       * @param sz_key_attr The attribute name for the key of the links.
       */
      void setKeyAttr(const char* sz_key_attr) noexcept { _key_attr.assign(sz_key_attr); }

      /**
       * @brief Sets the attribute name for the link.
       *
       * @param sz_key_attr The attribute name for the link.
       */
      void setLinkAttr(const char* sz_key_attr) noexcept { _link_attr.assign(sz_key_attr); }

      /**
       * @brief Returns the ID-to-node map used by the NetworkxIo object.
       *
       * @return A const reference to the ID-to-node map.
       */
      constexpr const IdNodeMap& idNodeMap() const noexcept { return _id_node_map; }

      /**
       * @brief Return the node associated to the given name if it exists, nullptr otherwise
       *
       */
      const Node* nodeFromId(const char* sz_name) const noexcept {
        const uint64_t h = nt::xxh64::hash(sz_name, std::strlen(sz_name), 0);
        return _id_node_map.findOrNull(h);
      }

      /**
       * @brief Return the node associated to the given index if it exists, nullptr otherwise
       *
       */
      const Node* nodeFromId(int id) const noexcept {
        const uint64_t h = static_cast< uint64_t >(id);
        return _id_node_map.findOrNull(h);
      }

      // Private methods

      template < typename GR0 >
      bool _readAttributes(GraphProperties< GR0 >* p_properties) noexcept {
        if (!p_properties) return true;

        const auto& root = _doc.GetObject();

        if (p_properties->_attributes.size()) {
          for (const auto& attr: root) {
            const char* sz_attr_name = attr.name.GetString();
            if (auto* p_any = p_properties->_attributes.findOrNull(sz_attr_name)) {
              const bool r = nt::details::setJsonValueToAny(*p_any, attr.value);
              if (!r) {
                LOG_F(ERROR,
                      "Type of attribute '{}' is not supported or type '{}' cannot "
                      "be bind to the JSON value",
                      sz_attr_name,
                      typeName((*p_any).type()));
                return false;
              }
            }
          }
        } else {
          for (const auto& attr: root) {
            const char* sz_attr_name = attr.name.GetString();
            // if (attr.value.IsDouble() || attr.value.IsFloat())
            //   p_properties->attribute(sz_attr_name, attr.value.GetDouble());
            // else if (attr.value.IsBool())
            //   p_properties->attribute(sz_attr_name, attr.value.GetBool());
            // else if (attr.value.IsInt())
            //   p_properties->attribute(sz_attr_name, attr.value.GetInt());
            // else { LOG_F(WARNING, "Type of attribute '{}' is not supported and will not be loaded.", sz_attr_name); }
          }
        }

        return true;
      }

      template < bool b_abort_on_mismatch_type = true >
      bool _readDocument(Graph& g, GraphProperties< Graph >* p_properties) noexcept {
        if (!_doc.IsObject()) {
          LOG_F(ERROR, "Document is invalid, it must be an object");
          return false;
        }

        if (!_doc.HasMember("directed")) {
          LOG_F(ERROR, "Missing directed property");
          return false;
        }

        const bool is_directed = _doc["directed"].GetBool();
        if constexpr (b_abort_on_mismatch_type) {
          if (is_directed == nt::concepts::UndirectedTagIndicator< Graph >) {
            LOG_F(ERROR,
                  "Trying to read a '{}' graph file to a '{}' graph type",
                  is_directed ? "directed" : "undirected",
                  nt::concepts::UndirectedTagIndicator< Graph > ? "undirected" : "directed");
            return false;
          }
        } else {
          if (is_directed == nt::concepts::UndirectedTagIndicator< Graph >) {
            LOG_F(WARNING,
                  "Trying to read a '{}' graph file to a '{}' graph type",
                  is_directed ? "directed" : "undirected",
                  nt::concepts::UndirectedTagIndicator< Graph > ? "undirected" : "directed");
          }
        }

        if (!(_doc.HasMember("nodes") && _doc["nodes"].IsArray() && _doc.HasMember(_link_attr.c_str())
              && _doc[_link_attr.c_str()].IsArray())) {
          LOG_F(ERROR, "Either missing or invalid nodes or links arrays");
          return false;
        }

        _id_node_map.clear();
        g.clear();

        // Read graph attributes
        if (!_readAttributes< Graph >(p_properties)) return false;

        return _readNodeSet(g, p_properties) && _readLinkSet(g, p_properties);
      }

      bool _readNodeSet(Graph& g, GraphProperties< Graph >* p_properties) noexcept {
        g.reserveNode(_doc["nodes"].Size());

        const bool b_create_maps = p_properties && p_properties->nodeMaps().empty();

        const auto& json_nodes = _doc["nodes"].GetArray();
        for (auto& json_node: json_nodes) {
          if (!json_node.IsObject()) {
            LOG_F(ERROR, "A node in the nodes list is not a valid object.");
            return false;
          }
          const auto& node_data = json_node.GetObject();
          const char* sz_name_attr = _name_attr.c_str();
          if (!node_data.HasMember(sz_name_attr)) {
            LOG_F(ERROR, "Missing attribute '{}'", sz_name_attr);
            return false;
          }

          const Node node = g.addNode();

          // In NetworkX, node's id can be either a string or an integer. So we need to deal with
          // these two cases
          uint64_t sz_router_id;
          if (_node_hash(node_data[sz_name_attr], sz_router_id)) {
            _id_node_map[sz_router_id] = node;
          } else {
            LOG_F(ERROR, "Node's id must be an integer or a string");
            return false;
          }

          // Get node attributes
          if (p_properties) {
            for (const auto& attr: node_data) {
              const char*                       sz_attr_name = attr.name.GetString();
              nt::details::MapStorage< Node >** p_map_storage = p_properties->nodeMaps().findOrNull(sz_attr_name);
              nt::details::MapStorage< Node >*  p_map = p_map_storage ? *p_map_storage : nullptr;
              if (!p_map && b_create_maps) {
                const char* sz_key_attr = _key_attr.c_str();
                if (!std::strcmp(sz_attr_name, sz_name_attr)) continue;
                if (attr.value.IsNumber())
                  p_map = p_properties->template nodeMap< double >(sz_attr_name, g);
                else if (attr.value.IsBool())
                  p_map = p_properties->template nodeMap< bool >(sz_attr_name, g);
                else if (attr.value.IsString())
                  p_map = p_properties->template nodeMap< nt::String >(sz_attr_name, g);
                else {
                  LOG_F(WARNING, "Type of attribute '{}' is not supported and will not be loaded.", sz_attr_name);
                }
              }

              if (p_map && !attr.value.IsNull()) {
                if (!nt::details::setJsonValueToMap< Node >(*p_map, node, attr.value)) {
                  LOG_F(ERROR,
                        "Type of attribute '{}' is not supported or map type '{}' cannot "
                        "be bind to the JSON value",
                        sz_attr_name,
                        typeName(p_map->value()));
                  return false;
                }
              }
            }
          }
        }
        return true;
      }

      template < typename GR0 >
      bool _readLinkSet(GR0& g, GraphProperties< GR0 >* p_properties) const noexcept {
        using Link = typename LinkView< GR0 >::Link;
        const auto& json_links = _doc[_link_attr.c_str()].GetArray();
        if (json_links.Empty()) return true;

        const bool b_create_maps = p_properties && p_properties->linkMaps().empty();

        LinkView< GR0 >::reserveLink(g, _doc[_link_attr.c_str()].Size());
        for (auto& json_link: json_links) {
          if (!json_link.IsObject()) {
            LOG_F(ERROR, "Link is not a JSON object");
            return false;
          }
          const auto& link_data = json_link.GetObject();

          const char* sz_source_attr = _source_attr.c_str();
          const char* sz_target_attr = _target_attr.c_str();
          if (!link_data.HasMember(sz_source_attr) || !link_data.HasMember(sz_target_attr)) {
            LOG_F(ERROR, "Missing properties '{}' and/or '{}' for a link", sz_source_attr, sz_target_attr);
            return false;
          }

          uint64_t sz_source_id;
          uint64_t sz_target_id;
          if (!_node_hash(link_data[sz_source_attr], sz_source_id)
              || !_node_hash(link_data[sz_target_attr], sz_target_id)) {
            LOG_F(ERROR, "Node's id must be an integer or a string");
            return false;
          }

          const Node* p_source = _id_node_map.findOrNull(sz_source_id);
          if (!p_source) {
            LOG_F(ERROR, "Adding link with unknown source node id={}", sz_source_id);
            return false;
          }

          const Node* p_target = _id_node_map.findOrNull(sz_target_id);
          if (!p_target) {
            LOG_F(ERROR, "Adding link with unknown target node id={}", sz_target_id);
            return false;
          }

          const Link link = LinkView< GR0 >::addLink(g, *p_source, *p_target);

          if (p_properties) {
            for (const auto& attr: link_data) {
              const char*                       sz_attr_name = attr.name.GetString();
              nt::details::MapStorage< Link >** p_map_storage = p_properties->linkMaps().findOrNull(sz_attr_name);
              nt::details::MapStorage< Link >*  p_map = p_map_storage ? *p_map_storage : nullptr;
              if (!p_map && b_create_maps) {
                const char* sz_key_attr = _key_attr.c_str();
                if (!std::strcmp(sz_attr_name, sz_source_attr) || !std::strcmp(sz_attr_name, sz_target_attr)
                    || !std::strcmp(sz_attr_name, sz_key_attr))
                  continue;
                if (attr.value.IsNumber())
                  p_map = p_properties->template linkMap< double >(sz_attr_name, g);
                else if (attr.value.IsBool())
                  p_map = p_properties->template linkMap< bool >(sz_attr_name, g);
                else if (attr.value.IsString())
                  p_map = p_properties->template linkMap< nt::String >(sz_attr_name, g);
                else {
                  LOG_F(WARNING, "Type of attribute '{}' is not supported and will not be loaded.", sz_attr_name);
                }
              }

              if (p_map && !attr.value.IsNull()) {
                if (!nt::details::setJsonValueToMap< Link >(*p_map, link, attr.value)) {
                  LOG_F(ERROR,
                        "Type of attribute '{}' is not supported or map type '{}' cannot "
                        "be bind to the JSON value",
                        sz_attr_name,
                        typeName(p_map->value()));
                  return false;
                }
              }
            }
          }
        }

        return true;
      }

      bool _writeDocument(const Graph& g, const GraphProperties< Graph >* p_properties) noexcept {
        _doc.SetObject();
        rapidjson::Document::AllocatorType& allocator = _doc.GetAllocator();

        rapidjson::Value directed(!nt::concepts::UndirectedTagIndicator< Graph >);
        _doc.AddMember("directed", directed, allocator);

        rapidjson::Value multigraph(true);
        _doc.AddMember("multigraph", multigraph, allocator);

        rapidjson::Value graph(rapidjson::kObjectType);
        _doc.AddMember("graph", graph, allocator);

        // Write user-defined graph attributes
        if (p_properties) {
          for (typename GraphProperties< Graph >::Attributes::const_iterator it = p_properties->_attributes.begin();
               it != p_properties->_attributes.end();
               ++it) {
            rapidjson::Value json_value = nt::details::getJsonValueFromAny(it->second, allocator);
            rapidjson::Value attr_name(it->first, std::strlen(it->first));
            _doc.AddMember(attr_name, json_value, allocator);
          }
        }

        return _writeNodeSet(g, p_properties) && _writeLinkSet(g, p_properties);
      }

      bool _writeNodeSet(const Graph& g, const GraphProperties< Graph >* p_properties) noexcept {
        rapidjson::Value                    nodes(rapidjson::kArrayType);
        rapidjson::Document::AllocatorType& allocator = _doc.GetAllocator();

        for (NodeIt v(g); v != nt::INVALID; ++v) {
          rapidjson::Value node_object(rapidjson::kObjectType);

          if (!p_properties || !p_properties->_node_maps.hasKey(_name_attr.c_str())) {
            rapidjson::Value id_name(_name_attr.c_str(), _name_attr.size());
            node_object.AddMember(id_name, g.id(v), allocator);
          }

          if (p_properties) {
            for (typename GraphProperties< Graph >::NodeMaps::const_iterator it = p_properties->_node_maps.begin();
                 it != p_properties->_node_maps.end();
                 ++it) {
              rapidjson::Value json_value =
                 nt::details::getJsonValueFromMap((*it->second), static_cast< const Node& >(v), allocator);
              rapidjson::Value attr_name(it->first, std::strlen(it->first));
              node_object.AddMember(attr_name, json_value, allocator);
            }
          }
          nodes.PushBack(node_object, allocator);
        }

        _doc.AddMember("nodes", nodes, allocator);

        return true;
      }

      bool _writeLinkSet(const Graph& g, const GraphProperties< Graph >* p_properties) noexcept {
        using Link = typename LinkView< Graph >::Link;
        using LinkIt = typename LinkView< Graph >::LinkIt;
        using LinkMapsIt = typename GraphProperties< Graph >::LinkMaps::const_iterator;

        rapidjson::Value                    links(rapidjson::kArrayType);
        rapidjson::Document::AllocatorType& allocator = _doc.GetAllocator();

        for (LinkIt link(g); link != nt::INVALID; ++link) {
          const Node u = LinkView< Graph >::u(g, link);
          const Node v = LinkView< Graph >::v(g, link);

          rapidjson::Value link_object(rapidjson::kObjectType);

          if (p_properties) {
            nt::details::MapStorage< Node >* const* p_map_storage =
               p_properties->nodeMaps().findOrNull(_name_attr.c_str());
            nt::details::MapStorage< Node >* p_map = p_map_storage ? *p_map_storage : nullptr;

            if (p_map) {
              rapidjson::Value source_value = nt::details::getJsonValueFromMap((*p_map), u, allocator);
              rapidjson::Value source_attr(_source_attr.c_str(), _source_attr.size());
              link_object.AddMember(source_attr, source_value, allocator);

              rapidjson::Value target_value = nt::details::getJsonValueFromMap((*p_map), v, allocator);
              rapidjson::Value target_attr(_target_attr.c_str(), _target_attr.size());
              link_object.AddMember(target_attr, target_value, allocator);
            } else {
              if (!p_properties->nodeMaps().hasKey(_source_attr.c_str())) {
                rapidjson::Value from(g.id(u));
                rapidjson::Value source_attr(_source_attr.c_str(), _source_attr.size());
                link_object.AddMember(source_attr, from, allocator);
              }

              if (!p_properties->nodeMaps().hasKey(_target_attr.c_str())) {
                rapidjson::Value to(g.id(v));
                rapidjson::Value target_attr(_target_attr.c_str(), _target_attr.size());
                link_object.AddMember(target_attr, to, allocator);
              }
            }

            for (LinkMapsIt it = p_properties->linkMaps().begin(); it != p_properties->linkMaps().end(); ++it) {
              rapidjson::Value json_value =
                 nt::details::getJsonValueFromMap((*it->second), static_cast< const Link& >(link), allocator);
              rapidjson::Value attr_name(it->first, std::strlen(it->first));
              link_object.AddMember(attr_name, json_value, allocator);
            }
          } else {
            rapidjson::Value from(g.id(u));
            rapidjson::Value source_attr(_source_attr.c_str(), _source_attr.size());
            link_object.AddMember(source_attr, from, allocator);

            rapidjson::Value to(g.id(v));
            rapidjson::Value target_attr(_target_attr.c_str(), _target_attr.size());
            link_object.AddMember(target_attr, to, allocator);
          }

          links.PushBack(link_object, allocator);
        }

        rapidjson::Value link_attr(_link_attr.c_str(), _link_attr.size());
        _doc.AddMember(link_attr, links, allocator);

        return true;
      }

      template < typename InputStream >
      bool _openDocumentFromStream(InputStream& buffer) noexcept {
        _doc.ParseStream< rapidjson::kParseStopWhenDoneFlag >(buffer);
        if (_doc.HasParseError()) {
          LOG_F(ERROR,
                "Parse Error: Document is not a valid JSON file. Error code : {} (offset: {})",
                static_cast< int >(_doc.GetParseError()),
                _doc.GetErrorOffset());
          return false;
        }
        return true;
      }

      /**
       * @brief Validates document structure and requested properties.
       *
       * @param p_properties Properties to validate against
       * @return true if all requested properties are available, false otherwise
       */
      bool _validateDocument(const GraphProperties< Graph >* p_properties) noexcept {
        if (!p_properties) {
          LOG_F(INFO, "No properties specified for validation - validation passed");
          return true;   // No filters to check
        }

        // Basic document structure validation
        if (!_doc.IsObject()) {
          LOG_F(ERROR, "Validation failed: Document is not a valid JSON object");
          return false;
        }

        if (!_doc.HasMember("directed") || !_doc.HasMember("nodes") || !_doc.HasMember(_link_attr.c_str())) {
          LOG_F(ERROR, "Validation failed: Missing required fields (directed, nodes, {})", _link_attr.c_str());
          return false;
        }

        bool all_properties_found = true;

        // Validate node properties
        if (!p_properties->nodeMaps().empty()) { all_properties_found &= _validateNodeProperties(p_properties); }

        // Validate link properties
        if (!p_properties->linkMaps().empty()) { all_properties_found &= _validateLinkProperties(p_properties); }

        if (all_properties_found) {
          LOG_F(INFO, "All requested properties validated successfully");
        } else {
          LOG_F(INFO, "Validation completed: some requested properties were not found");
        }

        return all_properties_found;
      }

      /**
       * @brief Validates availability of requested node properties.
       */
      bool _validateNodeProperties(const GraphProperties< Graph >* p_properties) noexcept {
        const auto& json_nodes = _doc["nodes"].GetArray();
        if (json_nodes.Empty()) {
          LOG_F(ERROR, "No nodes in file - cannot validate node properties");
          return false;
        }

        bool all_found = true;
        using NodeMaps = typename GraphProperties< Graph >::NodeMaps;

        // Check each requested node property
        for (typename NodeMaps::const_iterator it = p_properties->nodeMaps().begin();
             it != p_properties->nodeMaps().end();
             ++it) {
          const char* requested_attr = it->first;
          bool        found_in_any_node = false;

          // Check if this attribute exists in any node
          for (const auto& json_node: json_nodes) {
            if (json_node.IsObject() && json_node.HasMember(requested_attr)) {
              found_in_any_node = true;
              LOG_F(INFO, "Node property '{}' found in file", requested_attr);
              break;
            }
          }

          if (!found_in_any_node) {
            LOG_F(INFO, "Requested node property '{}' not found in any nodes", requested_attr);
            all_found = false;
          }
        }

        return all_found;
      }

      /**
       * @brief Validates availability of requested link properties.
       */
      bool _validateLinkProperties(const GraphProperties< Graph >* p_properties) noexcept {
        const auto& json_links = _doc[_link_attr.c_str()].GetArray();
        if (json_links.Empty()) {
          LOG_F(ERROR, "No links in file - cannot validate link properties");
          return false;
        }

        bool all_found = true;
        using LinkMaps = typename GraphProperties< Graph >::LinkMaps;

        // Check each requested link property
        for (typename LinkMaps::const_iterator it = p_properties->linkMaps().begin();
             it != p_properties->linkMaps().end();
             ++it) {
          const char* requested_attr = it->first;

          // Skip source/target attributes as they are mandatory
          if (!std::strcmp(requested_attr, _source_attr.c_str()) || !std::strcmp(requested_attr, _target_attr.c_str())
              || !std::strcmp(requested_attr, _key_attr.c_str())) {
            continue;
          }

          bool found_in_any_link = false;

          // Check if this attribute exists in any link
          for (const auto& json_link: json_links) {
            if (json_link.IsObject() && json_link.HasMember(requested_attr)) {
              found_in_any_link = true;
              LOG_F(INFO, "Link property '{}' found in file", requested_attr);
              break;
            }
          }

          if (!found_in_any_link) {
            LOG_F(INFO, "Requested link property '{}' not found in any links", requested_attr);
            all_found = false;
          }
        }

        return all_found;
      }

      bool _node_hash(const rapidjson::Value& value, uint64_t& h) const noexcept {
        if (value.IsInt()) {
          h = static_cast< uint64_t >(value.GetInt());
          return true;
        }

        if (value.IsString()) {
          const char* bytes = value.GetString();
          h = nt::xxh64::hash(bytes, std::strlen(bytes), 0);
          return true;
        }

        return false;
      }
    };

  }   // namespace graphs
}   // namespace nt


#endif