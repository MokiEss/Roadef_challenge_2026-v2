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

#ifndef _NT_GRAPH_PROPERTIES_H_
#define _NT_GRAPH_PROPERTIES_H_

#include "tools.h"
#include "../core/maps.h"

namespace nt {
  namespace graphs {

    namespace details {
      struct MapPtr {
        nt::Type t;
        void*    p;
      };

      template < typename GR, bool UNDIR >
      struct GraphPropertiesSelector {
        TEMPLATE_GRAPH_TYPEDEFS(GR);
        using EdgeMaps = nt::CstringMap< nt::details::MapStorage< Edge >* >;
        EdgeMaps                                   _edge_maps;
        nt::TrivialDynamicArray< details::MapPtr > _edge_maps_ptr;

        using Link = Edge;
        using LinkMap = nt::details::MapStorage< Link >;
        using LinkMaps = nt::CstringMap< LinkMap* >;

        GraphPropertiesSelector() = default;
        virtual ~GraphPropertiesSelector() noexcept { clearEdgeMaps(); }

        /**
         * @brief edge map reading rule
         *
         * Add a edge map reading rule to the reader.
         *
         * @param caption
         * @param map
         * @return
         */
        template < typename Map >
        nt::details::MapStorage< Edge >* edgeMap(const char* caption, Map& map) noexcept {
          nt::details::MapStorage< Edge >* storage = new nt::details::MapStorageAdaptor< Edge, Map >(map);
          assert(storage);
          _edge_maps.insert({NT_STRDUP(caption), storage});
          return storage;
        }

        template < typename T >
        nt::details::MapStorage< Edge >* edgeMap(const char* caption, const GR& g) noexcept {
          nt::details::MapStorage< Edge >* storage = nullptr;
          details::MapPtr                  ptr = {nt::TypeId< T >::value, nullptr};

          if constexpr (nt::is_arithmetic(nt::TypeId< T >::value) && !nt::is_bool(nt::TypeId< T >::value)) {
            DoubleEdgeMap* p = new DoubleEdgeMap(g, 0);
            storage = edgeMap< DoubleEdgeMap >(caption, *p);
            ptr.p = p;
            _edge_maps_ptr.add(std::move(ptr));
          } else if constexpr (nt::is_bool(nt::TypeId< T >::value)) {
            BoolEdgeMap* p = new BoolEdgeMap(g, false);
            storage = edgeMap< BoolEdgeMap >(caption, *p);
            ptr.p = p;
            _edge_maps_ptr.add(std::move(ptr));
          } else if constexpr (nt::is_string(nt::TypeId< T >::value)) {
            StringEdgeMap* p = new StringEdgeMap(g);
            storage = edgeMap< StringEdgeMap >(caption, *p);
            ptr.p = p;
            _edge_maps_ptr.add(std::move(ptr));
          } else {
            // LOG_F(WARNING, "Cannot of attribute '{}' is not supported and will not be loaded.", sz_attr_name);
          }

          return storage;
        }

        const EdgeMaps& edgeMaps() const noexcept { return _edge_maps; }
        EdgeMaps&       edgeMaps() noexcept { return _edge_maps; }


        const LinkMaps& linkMaps() const noexcept { return edgeMaps(); }
        LinkMaps&       linkMaps() noexcept { return edgeMaps(); }

        template < typename T >
        nt::details::MapStorage< Link >* linkMap(const char* caption, const GR& g) noexcept {
          return edgeMap< T >(caption, g);
        }

        void clearLinkMaps() noexcept { clearEdgeMaps(); }

        /**
         * @brief
         *
         */
        void clearEdgeMaps() noexcept {
          for (const details::MapPtr ptr: _edge_maps_ptr) {
            if (nt::is_arithmetic(ptr.t))
              delete static_cast< DoubleEdgeMap* >(ptr.p);
            else if (nt::is_bool(ptr.t))
              delete static_cast< BoolEdgeMap* >(ptr.p);
            else if (nt::is_string(ptr.t))
              delete static_cast< StringEdgeMap* >(ptr.p);
            else
              LOG_F(ERROR,
                    "Cannot delete edge map of type '{}' ({}). Possible memory leak.",
                    nt::typeName(ptr.t),
                    static_cast< int >(ptr.t));
          }
          for (typename EdgeMaps::iterator it = _edge_maps.begin(); it != _edge_maps.end(); ++it) {
            NT_STRFREE(const_cast< char* >(it->first));
            delete it->second;
          }
          _edge_maps.clear();
          _edge_maps_ptr.removeAll();
        }
      };


      template < typename GR >
      struct GraphPropertiesSelector< GR, false > {
        TEMPLATE_DIGRAPH_TYPEDEFS(GR);
        using ArcMaps = nt::CstringMap< nt::details::MapStorage< Arc >* >;
        ArcMaps                                    _arc_maps;
        nt::TrivialDynamicArray< details::MapPtr > _arc_maps_ptr;

        using Link = Arc;
        using LinkMap = nt::details::MapStorage< Link >;
        using LinkMaps = nt::CstringMap< LinkMap* >;

        GraphPropertiesSelector() = default;
        virtual ~GraphPropertiesSelector() noexcept { clearArcMaps(); }

        /**
         * @brief Arc map reading rule
         *
         * Add a arc map reading rule to the reader.
         *
         * @param caption
         * @param map
         * @return
         */
        template < typename Map >
        nt::details::MapStorage< Arc >* arcMap(const char* caption, Map& map) noexcept {
          nt::details::MapStorage< Arc >* storage = new nt::details::MapStorageAdaptor< Arc, Map >(map);
          assert(storage);
          _arc_maps.insert({NT_STRDUP(caption), storage});
          return storage;
        }

        template < typename T >
        nt::details::MapStorage< Arc >* arcMap(const char* caption, const GR& g) noexcept {
          nt::details::MapStorage< Arc >* storage = nullptr;
          details::MapPtr                 ptr = {nt::TypeId< T >::value, nullptr};

          if constexpr (nt::is_arithmetic(nt::TypeId< T >::value) && !nt::is_bool(nt::TypeId< T >::value)) {
            DoubleArcMap* p = new DoubleArcMap(g, 0);
            storage = arcMap< DoubleArcMap >(caption, *p);
            ptr.p = p;
            _arc_maps_ptr.add(std::move(ptr));
          } else if constexpr (nt::is_bool(nt::TypeId< T >::value)) {
            BoolArcMap* p = new BoolArcMap(g, false);
            storage = arcMap< BoolArcMap >(caption, *p);
            ptr.p = p;
            _arc_maps_ptr.add(std::move(ptr));
          } else if constexpr (nt::is_string(nt::TypeId< T >::value)) {
            StringArcMap* p = new StringArcMap(g);
            storage = arcMap< StringArcMap >(caption, *p);
            ptr.p = p;
            _arc_maps_ptr.add(std::move(ptr));
          } else {
            // LOG_F(WARNING, "Cannot of attribute '{}' is not supported and will not be loaded.", sz_attr_name);
          }

          return storage;
        }

        const ArcMaps& arcMaps() const noexcept { return _arc_maps; }
        ArcMaps&       arcMaps() noexcept { return _arc_maps; }

        const LinkMaps& linkMaps() const noexcept { return arcMaps(); }
        LinkMaps&       linkMaps() noexcept { return arcMaps(); }

        template < typename T >
        nt::details::MapStorage< Arc >* linkMap(const char* caption, const GR& g) noexcept {
          return arcMap< T >(caption, g);
        }

        void clearLinkMaps() noexcept { clearArcMaps(); }

        /**
         * @brief
         *
         */
        void clearArcMaps() noexcept {
          for (const details::MapPtr ptr: _arc_maps_ptr) {
            if (nt::is_arithmetic(ptr.t))
              delete static_cast< DoubleArcMap* >(ptr.p);
            else if (nt::is_bool(ptr.t))
              delete static_cast< BoolArcMap* >(ptr.p);
            else if (nt::is_string(ptr.t))
              delete static_cast< StringArcMap* >(ptr.p);
            else
              LOG_F(ERROR,
                    "Cannot delete arc map of type '{}' ({}). Possible memory leak.",
                    nt::typeName(ptr.t),
                    static_cast< int >(ptr.t));
          }
          for (typename ArcMaps::iterator it = _arc_maps.begin(); it != _arc_maps.end(); ++it) {
            NT_STRFREE(const_cast< char* >(it->first));
            delete it->second;
          }
          _arc_maps.clear();
          _arc_maps_ptr.removeAll();
        }
      };
    }   // namespace details

    /**
     * @class GraphProperties
     *
     * @brief This class provides access to a collection of property maps attached
     * to a graph through a dynamically-typed interface. It is mostly used when reading/writing
     * graphs from/to an IO stream where the type of the attributes cannot be determined at compile
     * time
     *
     * @tparam GR The type of the graph to which the stored maps are associated
     */
    template < typename GR >
    struct GraphProperties: details::GraphPropertiesSelector< GR, nt::concepts::UndirectedTagIndicator< GR > > {
      TEMPLATE_DIGRAPH_TYPEDEFS(GR);
      using Parent = details::GraphPropertiesSelector< GR, nt::concepts::UndirectedTagIndicator< GR > >;

      using NodeMaps = nt::CstringMap< nt::details::MapStorage< Node >* >;
      using Attributes = nt::CstringMap< nt::AnyView >;

      NodeMaps   _node_maps;
      Attributes _attributes;

      nt::TrivialDynamicArray< details::MapPtr > _node_maps_ptr;

      GraphProperties() = default;
      ~GraphProperties() noexcept { clearNodeMaps(); }

      /**
       * @brief Node map reading rule
       *
       * Add a node map reading rule to the reader.
       *
       * @param caption
       * @param map
       * @return
       */
      template < typename Map >
      nt::details::MapStorage< Node >* nodeMap(const char* caption, Map& map) noexcept {
        nt::details::MapStorage< Node >* storage = new nt::details::MapStorageAdaptor< Node, Map >(map);
        assert(storage);
        _node_maps.insert({NT_STRDUP(caption), storage});
        return storage;
      }

      const NodeMaps& nodeMaps() const noexcept { return _node_maps; }
      NodeMaps&       nodeMaps() noexcept { return _node_maps; }

      template < typename T >
      nt::details::MapStorage< Node >* nodeMap(const char* caption, const GR& g) noexcept {
        nt::details::MapStorage< Node >* storage = nullptr;
        details::MapPtr                  ptr = {nt::TypeId< T >::value, nullptr};

        if constexpr (nt::is_arithmetic(nt::TypeId< T >::value) && !nt::is_bool(nt::TypeId< T >::value)) {
          DoubleNodeMap* p = new DoubleNodeMap(g, 0);
          storage = nodeMap< DoubleNodeMap >(caption, *p);
          ptr.p = p;
          _node_maps_ptr.add(std::move(ptr));
        } else if constexpr (nt::is_bool(nt::TypeId< T >::value)) {
          BoolNodeMap* p = new BoolNodeMap(g, false);
          storage = nodeMap< BoolNodeMap >(caption, *p);
          ptr.p = p;
          _node_maps_ptr.add(std::move(ptr));
        } else if constexpr (nt::is_string(nt::TypeId< T >::value)) {
          StringNodeMap* p = new StringNodeMap(g);
          storage = nodeMap< StringNodeMap >(caption, *p);
          ptr.p = p;
          _node_maps_ptr.add(std::move(ptr));
        } else {
          // LOG_F(WARNING, "Cannot of attribute '{}' is not supported and will not be loaded.", sz_attr_name);
        }

        return storage;
      }

      /**
       * @brief Attribute reading rule
       *
       * Add an attribute reading rule to the reader.
       *
       * @param caption
       * @param value
       * @return
       */
      template < typename Value >
      void attribute(const char* caption, Value& value) noexcept {
        _attributes.insert({caption, nt::AnyView(value)});
      }

      const Attributes& attributes() const noexcept { return _attributes; }
      Attributes&       attributes() noexcept { return _attributes; }

      void clearNodeMaps() noexcept {
        for (const details::MapPtr ptr: _node_maps_ptr) {
          if (nt::is_arithmetic(ptr.t))
            delete static_cast< DoubleNodeMap* >(ptr.p);
          else if (nt::is_bool(ptr.t))
            delete static_cast< BoolNodeMap* >(ptr.p);
          else if (nt::is_string(ptr.t))
            delete static_cast< StringNodeMap* >(ptr.p);
          else
            LOG_F(ERROR,
                  "Cannot delete node map of type '{}' ({}). Possible memory leak.",
                  nt::typeName(ptr.t),
                  static_cast< int >(ptr.t));
        }
        for (typename NodeMaps::iterator it = _node_maps.begin(); it != _node_maps.end(); ++it) {
          NT_STRFREE(const_cast< char* >(it->first));
          delete it->second;
        }
        _node_maps.clear();
        _attributes.clear();
        _node_maps_ptr.removeAll();
      }

      void clear() noexcept {
        clearNodeMaps();
        Parent::clearLinkMaps();
      }
    };

  }   // namespace graphs
}   // namespace nt

#endif