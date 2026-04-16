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
 *
 * ============================================================================
 *
 * This file incorporates work from the LEMON graph library (dfs.h).
 *
 * Original LEMON Copyright Notice:
 * Copyright (C) 2003-2013 Egervary Jeno Kombinatorikus Optimalizalasi
 * Kutatocsoport (Egervary Research Group on Combinatorial Optimization, EGRES).
 *
 * Permission to use, modify and distribute this software is granted provided
 * that this copyright notice appears in all copies. For precise terms see the
 * accompanying LICENSE file.
 *
 * This software is provided "AS IS" with no warranty of any kind, express or
 * implied, and with no claim as to its suitability for any purpose.
 *
 * ============================================================================
 * 
 * List of modifications:
 *   - Changed namespace from 'lemon' to 'nt'
 *   - Replaced std::vector with nt::TrivialDynamicArray/nt::DynamicArray
 *   - Updated include paths to networktools structure
 *   - Changed LEMON_ASSERT to NT_ASSERT
 *   - Changed LEMON_DEBUG to NT_DEBUG
 *   - Adapted LEMON concept checking to networktools
 *   - Converted typedef declarations to C++11 using declarations
 *   - Added constexpr qualifiers for compile-time optimization
 *   - Added noexcept specifiers for better exception safety
 *   - Adapted LEMON INVALID sentinel value handling
 *   - Updated or enhanced Doxygen documentation
 *   - Added move semantics for performance
 *   - Updated header guard macros
 */

/**
 * @file
 * @brief DFS algorithm
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_DFS_H_
#define _NT_DFS_H_

#include "../tools.h"
#include "../path.h"
#include "../../core/stack.h"
#include "../bits/path_dump.h"

#ifdef IN
#  undef IN
#endif

#ifdef OUT
#  undef OUT
#endif

namespace nt {
  namespace graphs {

    /** Default traits class of Dfs class. */

    /**
     * Default traits class of Dfs class.
     * @tparam GR Digraph type.
     */
    template < class GR >
    struct DfsDefaultTraits {
      /** The type of the digraph the algorithm runs on. */
      using Digraph = GR;

      /**
       * @brief The type of the map that stores the predecessor
       * arcs of the %DFS paths.
       *
       * The type of the map that stores the predecessor
       * arcs of the %DFS paths.
       * It must conform to the @ref concepts::WriteMap "WriteMap" concept.
       */
      using PredMap = typename Digraph::template NodeMap< typename Digraph::Arc >;
      /** Instantiates a \c PredMap. */

      /**
       * This function instantiates a @ref PredMap.
       * @param g is the digraph, to which we would like to define the
       * @ref PredMap.
       */
      static PredMap* createPredMap(const Digraph& g) { return new PredMap(g); }

      /**
       * The type of the map that indicates which nodes are processed.
       */

      /**
       * The type of the map that indicates which nodes are processed.
       * It must conform to the @ref concepts::WriteMap "WriteMap" concept.
       * By default, it is a NullMap.
       */
      using ProcessedMap = NullMap< typename Digraph::Node, bool >;
      /** Instantiates a \c ProcessedMap. */

      /**
       * This function instantiates a @ref ProcessedMap.
       * @param g is the digraph, to which
       * we would like to define the @ref ProcessedMap.
       */
#ifdef DOXYGEN
      static ProcessedMap* createProcessedMap(const Digraph& g)
#else
      static ProcessedMap* createProcessedMap(const Digraph&)
#endif
      {
        return new ProcessedMap();
      }

      /** The type of the map that indicates which nodes are reached. */

      /**
       * The type of the map that indicates which nodes are reached.
       * It must conform to
       * the @ref concepts::ReadWriteMap "ReadWriteMap" concept.
       */
      using ReachedMap = typename Digraph::template NodeMap< bool >;
      /** Instantiates a \c ReachedMap. */

      /**
       * This function instantiates a @ref ReachedMap.
       * @param g is the digraph, to which
       * we would like to define the @ref ReachedMap.
       */
      static ReachedMap* createReachedMap(const Digraph& g) { return new ReachedMap(g); }

      /** The type of the map that stores the distances of the nodes. */

      /**
       * The type of the map that stores the distances of the nodes.
       * It must conform to the @ref concepts::WriteMap "WriteMap" concept.
       */
      using DistMap = typename Digraph::template NodeMap< int >;
      /** Instantiates a \c DistMap. */

      /**
       * This function instantiates a @ref DistMap.
       * @param g is the digraph, to which we would like to define the
       * @ref DistMap.
       */
      static DistMap* createDistMap(const Digraph& g) { return new DistMap(g); }
    };

/** %DFS algorithm class. */

/**
 * @ingroup search
 * This class provides an efficient implementation of the %DFS algorithm.
 *
 * There is also a @ref dfs() "function-type interface" for the DFS
 * algorithm, which is convenient in the simplier cases and it can be
 * used easier.
 *
 * @tparam GR The type of the digraph the algorithm runs on.
 * @tparam TR The traits class that defines various types used by the
 * algorithm. By default, it is @ref DfsDefaultTraits
 * "DfsDefaultTraits<GR>".
 * In most cases, this parameter should not be set directly,
 * consider to use the named template parameters instead.
 */
#ifdef DOXYGEN
    template < typename GR, typename TR >
#else
    template < typename GR, typename TR = DfsDefaultTraits< GR > >
#endif
    class Dfs {
      public:
      /** The type of the digraph the algorithm runs on. */
      using Digraph = typename TR::Digraph;

      /**
       * @brief The type of the map that stores the predecessor arcs of the
       * DFS paths.
       */
      using PredMap = typename TR::PredMap;
      /** The type of the map that stores the distances of the nodes. */
      using DistMap = typename TR::DistMap;
      /** The type of the map that indicates which nodes are reached. */
      using ReachedMap = typename TR::ReachedMap;
      /**
       * The type of the map that indicates which nodes are processed.
       */
      using ProcessedMap = typename TR::ProcessedMap;
      /** The type of the paths. */
      using Path = PredMapPath< Digraph, PredMap >;

      /**
       * The @ref lemon::DfsDefaultTraits "traits class" of the algorithm.
       */
      using Traits = TR;

      private:
      using Node = typename Digraph::Node;
      using NodeIt = typename Digraph::NodeIt;
      using Arc = typename Digraph::Arc;
      using OutArcIt = typename Digraph::OutArcIt;

      // Pointer to the underlying digraph.
      const Digraph* G;
      // Pointer to the map of predecessor arcs.
      PredMap* _pred;
      // Indicates if _pred is locally allocated (true) or not.
      bool local_pred;
      // Pointer to the map of distances.
      DistMap* _dist;
      // Indicates if _dist is locally allocated (true) or not.
      bool local_dist;
      // Pointer to the map of reached status of the nodes.
      ReachedMap* _reached;
      // Indicates if _reached is locally allocated (true) or not.
      bool local_reached;
      // Pointer to the map of processed status of the nodes.
      ProcessedMap* _processed;
      // Indicates if _processed is locally allocated (true) or not.
      bool local_processed;

      nt::TrivialDynamicArray< typename Digraph::OutArcIt > _stack;
      int                                                   _stack_head;

      // Creates the maps if necessary.
      void create_maps() {
        if (!_pred) {
          local_pred = true;
          _pred = Traits::createPredMap(*G);
        }
        if (!_dist) {
          local_dist = true;
          _dist = Traits::createDistMap(*G);
        }
        if (!_reached) {
          local_reached = true;
          _reached = Traits::createReachedMap(*G);
        }
        if (!_processed) {
          local_processed = true;
          _processed = Traits::createProcessedMap(*G);
        }
      }

      protected:
      Dfs() {}

      public:
      using Create = Dfs;

      /** \name Named Template Parameters */

      /** @{ */

      template < class T >
      struct SetPredMapTraits: public Traits {
        using PredMap = T;
        static PredMap* createPredMap(const Digraph&) {
          NT_ASSERT(false, "PredMap is not initialized");
          return 0;   // ignore warnings
        }
      };
      /**
       * @brief @ref named-templ-param "Named parameter" for setting
       * \c PredMap type.
       *
       * @ref named-templ-param "Named parameter" for setting
       * \c PredMap type.
       * It must conform to the @ref concepts::WriteMap "WriteMap" concept.
       */
      template < class T >
      struct SetPredMap: public Dfs< Digraph, SetPredMapTraits< T > > {
        using Create = Dfs< Digraph, SetPredMapTraits< T > >;
      };

      template < class T >
      struct SetDistMapTraits: public Traits {
        using DistMap = T;
        static DistMap* createDistMap(const Digraph&) {
          NT_ASSERT(false, "DistMap is not initialized");
          return 0;   // ignore warnings
        }
      };
      /**
       * @brief @ref named-templ-param "Named parameter" for setting
       * \c DistMap type.
       *
       * @ref named-templ-param "Named parameter" for setting
       * \c DistMap type.
       * It must conform to the @ref concepts::WriteMap "WriteMap" concept.
       */
      template < class T >
      struct SetDistMap: public Dfs< Digraph, SetDistMapTraits< T > > {
        using Create = Dfs< Digraph, SetDistMapTraits< T > >;
      };

      template < class T >
      struct SetReachedMapTraits: public Traits {
        using ReachedMap = T;
        static ReachedMap* createReachedMap(const Digraph&) {
          NT_ASSERT(false, "ReachedMap is not initialized");
          return 0;   // ignore warnings
        }
      };
      /**
       * @brief @ref named-templ-param "Named parameter" for setting
       * \c ReachedMap type.
       *
       * @ref named-templ-param "Named parameter" for setting
       * \c ReachedMap type.
       * It must conform to
       * the @ref concepts::ReadWriteMap "ReadWriteMap" concept.
       */
      template < class T >
      struct SetReachedMap: public Dfs< Digraph, SetReachedMapTraits< T > > {
        using Create = Dfs< Digraph, SetReachedMapTraits< T > >;
      };

      template < class T >
      struct SetProcessedMapTraits: public Traits {
        using ProcessedMap = T;
        static ProcessedMap* createProcessedMap(const Digraph&) {
          NT_ASSERT(false, "ProcessedMap is not initialized");
          return 0;   // ignore warnings
        }
      };
      /**
       * @brief @ref named-templ-param "Named parameter" for setting
       * \c ProcessedMap type.
       *
       * @ref named-templ-param "Named parameter" for setting
       * \c ProcessedMap type.
       * It must conform to the @ref concepts::WriteMap "WriteMap" concept.
       */
      template < class T >
      struct SetProcessedMap: public Dfs< Digraph, SetProcessedMapTraits< T > > {
        using Create = Dfs< Digraph, SetProcessedMapTraits< T > >;
      };

      struct SetStandardProcessedMapTraits: public Traits {
        using ProcessedMap = typename Digraph::template NodeMap< bool >;
        static ProcessedMap* createProcessedMap(const Digraph& g) { return new ProcessedMap(g); }
      };
      /**
       * @brief @ref named-templ-param "Named parameter" for setting
       * \c ProcessedMap type to be <tt>Digraph::NodeMap<bool></tt>.
       *
       * @ref named-templ-param "Named parameter" for setting
       * \c ProcessedMap type to be <tt>Digraph::NodeMap<bool></tt>.
       * If you don't set it explicitly, it will be automatically allocated.
       */
      struct SetStandardProcessedMap: public Dfs< Digraph, SetStandardProcessedMapTraits > {
        using Create = Dfs< Digraph, SetStandardProcessedMapTraits >;
      };

      /** @} */

      public:
      /** Constructor. */

      /**
       * Constructor.
       * @param g The digraph the algorithm runs on.
       */
      Dfs(const Digraph& g) :
          G(&g), _pred(NULL), local_pred(false), _dist(NULL), local_dist(false), _reached(NULL), local_reached(false),
          _processed(NULL), local_processed(false) {}

      /** Destructor. */
      ~Dfs() noexcept {
        if (local_pred) delete _pred;
        if (local_dist) delete _dist;
        if (local_reached) delete _reached;
        if (local_processed) delete _processed;
      }

      /** Sets the map that stores the predecessor arcs. */

      /**
       * Sets the map that stores the predecessor arcs.
       * If you don't use this function before calling @ref run(Node) "run()"
       * or @ref init(), an instance will be allocated automatically.
       * The destructor deallocates this automatically allocated map,
       * of course.
       * @return <tt> (*this) </tt>
       */
      Dfs& predMap(PredMap& m) {
        if (local_pred) {
          delete _pred;
          local_pred = false;
        }
        _pred = &m;
        return *this;
      }

      /** Sets the map that indicates which nodes are reached. */

      /**
       * Sets the map that indicates which nodes are reached.
       * If you don't use this function before calling @ref run(Node) "run()"
       * or @ref init(), an instance will be allocated automatically.
       * The destructor deallocates this automatically allocated map,
       * of course.
       * @return <tt> (*this) </tt>
       */
      Dfs& reachedMap(ReachedMap& m) {
        if (local_reached) {
          delete _reached;
          local_reached = false;
        }
        _reached = &m;
        return *this;
      }

      /** Sets the map that indicates which nodes are processed. */

      /**
       * Sets the map that indicates which nodes are processed.
       * If you don't use this function before calling @ref run(Node) "run()"
       * or @ref init(), an instance will be allocated automatically.
       * The destructor deallocates this automatically allocated map,
       * of course.
       * @return <tt> (*this) </tt>
       */
      Dfs& processedMap(ProcessedMap& m) {
        if (local_processed) {
          delete _processed;
          local_processed = false;
        }
        _processed = &m;
        return *this;
      }

      /** Sets the map that stores the distances of the nodes. */

      /**
       * Sets the map that stores the distances of the nodes calculated by
       * the algorithm.
       * If you don't use this function before calling @ref run(Node) "run()"
       * or @ref init(), an instance will be allocated automatically.
       * The destructor deallocates this automatically allocated map,
       * of course.
       * @return <tt> (*this) </tt>
       */
      Dfs& distMap(DistMap& m) {
        if (local_dist) {
          delete _dist;
          local_dist = false;
        }
        _dist = &m;
        return *this;
      }

      public:
      /**
       * \name Execution Control
       * The simplest way to execute the DFS algorithm is to use one of the
       * member functions called @ref run(Node) "run()".\n
       * If you need better control on the execution, you have to call
       * @ref init() first, then you can add a source node with @ref addSource()
       * and perform the actual computation with @ref start().
       * This procedure can be repeated if there are nodes that have not
       * been reached.
       */

      /** @{ */

      /**
       * @brief Initializes the internal data structures.
       *
       * Initializes the internal data structures.
       */
      void init() {
        create_maps();
        _stack.resize(nt::graphs::countNodes(*G));
        _stack_head = -1;
        for (NodeIt u(*G); u != INVALID; ++u) {
          _pred->set(u, INVALID);
          _reached->set(u, false);
          _processed->set(u, false);
        }
      }

      /** Adds a new source node. */

      /**
       * Adds a new source node to the set of nodes to be processed.
       *
       * @pre The stack must be empty. Otherwise the algorithm gives
       * wrong results. (One of the outgoing arcs of all the source nodes
       * except for the last one will not be visited and distances will
       * also be wrong.)
       */
      void addSource(Node s) {
        NT_ASSERT(emptyQueue(), "The stack is not empty.");
        if (!(*_reached)[s]) {
          _reached->set(s, true);
          _pred->set(s, INVALID);
          OutArcIt e(*G, s);
          if (e != INVALID) {
            _stack[++_stack_head] = e;
            _dist->set(s, _stack_head);
          } else {
            _processed->set(s, true);
            _dist->set(s, 0);
          }
        }
      }

      /** Processes the next arc. */

      /**
       * Processes the next arc.
       *
       * @return The processed arc.
       *
       * @pre The stack must not be empty.
       */
      Arc processNextArc() {
        Node m;
        Arc  e = _stack[_stack_head];
        if (!(*_reached)[m = G->target(e)]) {
          _pred->set(m, e);
          _reached->set(m, true);
          ++_stack_head;
          _stack[_stack_head] = OutArcIt(*G, m);
          _dist->set(m, _stack_head);
        } else {
          m = G->source(e);
          ++_stack[_stack_head];
        }
        while (_stack_head >= 0 && _stack[_stack_head] == INVALID) {
          _processed->set(m, true);
          --_stack_head;
          if (_stack_head >= 0) {
            m = G->source(_stack[_stack_head]);
            ++_stack[_stack_head];
          }
        }
        return e;
      }

      /** Next arc to be processed. */

      /**
       * Next arc to be processed.
       *
       * @return The next arc to be processed or \c INVALID if the stack
       * is empty.
       */
      OutArcIt nextArc() const { return _stack_head >= 0 ? _stack[_stack_head] : INVALID; }

      /** Returns \c false if there are nodes to be processed. */

      /**
       * Returns \c false if there are nodes to be processed
       * in the queue (stack).
       */
      bool emptyQueue() const { return _stack_head < 0; }

      /** Returns the number of the nodes to be processed. */

      /**
       * Returns the number of the nodes to be processed
       * in the queue (stack).
       */
      int queueSize() const { return _stack_head + 1; }

      /** Executes the algorithm. */

      /**
       * Executes the algorithm.
       *
       * This method runs the %DFS algorithm from the root node
       * in order to compute the DFS path to each node.
       *
       * The algorithm computes
       * - the %DFS tree,
       * - the distance of each node from the root in the %DFS tree.
       *
       * @pre init() must be called and a root node should be
       * added with addSource() before using this function.
       *
       * @note <tt>d.start()</tt> is just a shortcut of the following code.
       * @code
       * while ( !d.emptyQueue() ) {
       * d.processNextArc();
       * }
       * @endcode
       */
      void start() {
        while (!emptyQueue())
          processNextArc();
      }

      /**
       * Executes the algorithm until the given target node is reached.
       */

      /**
       * Executes the algorithm until the given target node is reached.
       *
       * This method runs the %DFS algorithm from the root node
       * in order to compute the DFS path to \c t.
       *
       * The algorithm computes
       * - the %DFS path to \c t,
       * - the distance of \c t from the root in the %DFS tree.
       *
       * @pre init() must be called and a root node should be
       * added with addSource() before using this function.
       */
      void start(Node t) {
        while (!emptyQueue() && !(*_reached)[t])
          processNextArc();
      }

      /** Executes the algorithm until a condition is met. */

      /**
       * Executes the algorithm until a condition is met.
       *
       * This method runs the %DFS algorithm from the root node
       * until an arc \c a with <tt>am[a]</tt> true is found.
       *
       * @param am A \c bool (or convertible) arc map. The algorithm
       * will stop when it reaches an arc \c a with <tt>am[a]</tt> true.
       *
       * @return The reached arc \c a with <tt>am[a]</tt> true or
       * \c INVALID if no such arc was found.
       *
       * @pre init() must be called and a root node should be
       * added with addSource() before using this function.
       *
       * @warning Contrary to @ref Bfs and @ref Dijkstra, \c am is an arc map,
       * not a node map.
       */
      template < class ArcBoolMap >
      Arc start(const ArcBoolMap& am) {
        while (!emptyQueue() && !am[_stack[_stack_head]])
          processNextArc();
        return emptyQueue() ? INVALID : _stack[_stack_head];
      }

      /** Runs the algorithm from the given source node. */

      /**
       * This method runs the %DFS algorithm from node \c s
       * in order to compute the DFS path to each node.
       *
       * The algorithm computes
       * - the %DFS tree,
       * - the distance of each node from the root in the %DFS tree.
       *
       * @note <tt>d.run(s)</tt> is just a shortcut of the following code.
       * @code
       * d.init();
       * d.addSource(s);
       * d.start();
       * @endcode
       */
      void run(Node s) {
        init();
        addSource(s);
        start();
      }

      /** Finds the %DFS path between \c s and \c t. */

      /**
       * This method runs the %DFS algorithm from node \c s
       * in order to compute the DFS path to node \c t
       * (it stops searching when \c t is processed)
       *
       * @return \c true if \c t is reachable form \c s.
       *
       * @note Apart from the return value, <tt>d.run(s,t)</tt> is
       * just a shortcut of the following code.
       * @code
       * d.init();
       * d.addSource(s);
       * d.start(t);
       * @endcode
       */
      bool run(Node s, Node t) {
        init();
        addSource(s);
        start(t);
        return reached(t);
      }

      /** Runs the algorithm to visit all nodes in the digraph. */

      /**
       * This method runs the %DFS algorithm in order to visit all nodes
       * in the digraph.
       *
       * @note <tt>d.run()</tt> is just a shortcut of the following code.
       * @code
       * d.init();
       * for (NodeIt n(digraph); n != INVALID; ++n) {
       * if (!d.reached(n)) {
       * d.addSource(n);
       * d.start();
       * }
       * }
       * @endcode
       */
      void run() {
        init();
        for (NodeIt it(*G); it != INVALID; ++it) {
          if (!reached(it)) {
            addSource(it);
            start();
          }
        }
      }

      /** @} */

      /**
       * \name Query Functions
       * The results of the DFS algorithm can be obtained using these
       * functions.\n
       * Either @ref run(Node) "run()" or @ref start() should be called
       * before using them.
       */

      /** @{ */

      /** The DFS path to the given node. */

      /**
       * Returns the DFS path to the given node from the root(s).
       *
       * @warning \c t should be reached from the root(s).
       *
       * @pre Either @ref run(Node) "run()" or @ref init()
       * must be called before using this function.
       */
      Path path(Node t) const { return Path(*G, *_pred, t); }

      /** The distance of the given node from the root(s). */

      /**
       * Returns the distance of the given node from the root(s).
       *
       * @warning If node \c v is not reached from the root(s), then
       * the return value of this function is undefined.
       *
       * @pre Either @ref run(Node) "run()" or @ref init()
       * must be called before using this function.
       */
      int dist(Node v) const { return (*_dist)[v]; }

      /**
       * Returns the 'previous arc' of the %DFS tree for the given node.
       */

      /**
       * This function returns the 'previous arc' of the %DFS tree for the
       * node \c v, i.e. it returns the last arc of a %DFS path from a
       * root to \c v. It is \c INVALID if \c v is not reached from the
       * root(s) or if \c v is a root.
       *
       * The %DFS tree used here is equal to the %DFS tree used in
       * @ref predNode() and @ref predMap().
       *
       * @pre Either @ref run(Node) "run()" or @ref init()
       * must be called before using this function.
       */
      Arc predArc(Node v) const { return (*_pred)[v]; }

      /**
       * Returns the 'previous node' of the %DFS tree for the given node.
       */

      /**
       * This function returns the 'previous node' of the %DFS
       * tree for the node \c v, i.e. it returns the last but one node
       * of a %DFS path from a root to \c v. It is \c INVALID
       * if \c v is not reached from the root(s) or if \c v is a root.
       *
       * The %DFS tree used here is equal to the %DFS tree used in
       * @ref predArc() and @ref predMap().
       *
       * @pre Either @ref run(Node) "run()" or @ref init()
       * must be called before using this function.
       */
      Node predNode(Node v) const { return (*_pred)[v] == INVALID ? INVALID : G->source((*_pred)[v]); }

      /**
       * @brief Returns a const reference to the node map that stores the
       * distances of the nodes.
       *
       * Returns a const reference to the node map that stores the
       * distances of the nodes calculated by the algorithm.
       *
       * @pre Either @ref run(Node) "run()" or @ref init()
       * must be called before using this function.
       */
      const DistMap& distMap() const { return *_dist; }

      /**
       * @brief Returns a const reference to the node map that stores the
       * predecessor arcs.
       *
       * Returns a const reference to the node map that stores the predecessor
       * arcs, which form the DFS tree (forest).
       *
       * @pre Either @ref run(Node) "run()" or @ref init()
       * must be called before using this function.
       */
      const PredMap& predMap() const { return *_pred; }

      /** Checks if the given node. node is reached from the root(s). */

      /**
       * Returns \c true if \c v is reached from the root(s).
       *
       * @pre Either @ref run(Node) "run()" or @ref init()
       * must be called before using this function.
       */
      bool reached(Node v) const { return (*_reached)[v]; }

      /** @} */
    };

    /** Default traits class of dfs() function. */

    /**
     * Default traits class of dfs() function.
     * @tparam GR Digraph type.
     */
    template < class GR >
    struct DfsWizardDefaultTraits {
      /** The type of the digraph the algorithm runs on. */
      using Digraph = GR;

      /**
       * @brief The type of the map that stores the predecessor
       * arcs of the %DFS paths.
       *
       * The type of the map that stores the predecessor
       * arcs of the %DFS paths.
       * It must conform to the @ref concepts::WriteMap "WriteMap" concept.
       */
      using PredMap = typename Digraph::template NodeMap< typename Digraph::Arc >;
      /** Instantiates a PredMap. */

      /**
       * This function instantiates a PredMap.
       * @param g is the digraph, to which we would like to define the
       * PredMap.
       */
      static PredMap* createPredMap(const Digraph& g) { return new PredMap(g); }

      /**
       * The type of the map that indicates which nodes are processed.
       */

      /**
       * The type of the map that indicates which nodes are processed.
       * It must conform to the @ref concepts::WriteMap "WriteMap" concept.
       * By default, it is a NullMap.
       */
      using ProcessedMap = NullMap< typename Digraph::Node, bool >;
      /** Instantiates a ProcessedMap. */

      /**
       * This function instantiates a ProcessedMap.
       * @param g is the digraph, to which
       * we would like to define the ProcessedMap.
       */
#ifdef DOXYGEN
      static ProcessedMap* createProcessedMap(const Digraph& g)
#else
      static ProcessedMap* createProcessedMap(const Digraph&)
#endif
      {
        return new ProcessedMap();
      }

      /** The type of the map that indicates which nodes are reached. */

      /**
       * The type of the map that indicates which nodes are reached.
       * It must conform to
       * the @ref concepts::ReadWriteMap "ReadWriteMap" concept.
       */
      using ReachedMap = typename Digraph::template NodeMap< bool >;
      /** Instantiates a ReachedMap. */

      /**
       * This function instantiates a ReachedMap.
       * @param g is the digraph, to which
       * we would like to define the ReachedMap.
       */
      static ReachedMap* createReachedMap(const Digraph& g) { return new ReachedMap(g); }

      /** The type of the map that stores the distances of the nodes. */

      /**
       * The type of the map that stores the distances of the nodes.
       * It must conform to the @ref concepts::WriteMap "WriteMap" concept.
       */
      using DistMap = typename Digraph::template NodeMap< int >;
      /** Instantiates a DistMap. */

      /**
       * This function instantiates a DistMap.
       * @param g is the digraph, to which we would like to define
       * the DistMap
       */
      static DistMap* createDistMap(const Digraph& g) { return new DistMap(g); }

      /** The type of the DFS paths. */

      /**
       * The type of the DFS paths.
       * It must conform to the @ref concepts::Path "Path" concept.
       */
      using Path = nt::graphs::Path< Digraph >;
    };

    /** Default traits class used by DfsWizard */

    /**
     * Default traits class used by DfsWizard.
     * @tparam GR The type of the digraph.
     */
    template < class GR >
    class DfsWizardBase: public DfsWizardDefaultTraits< GR > {
      using Base = DfsWizardDefaultTraits< GR >;

      protected:
      // The type of the nodes in the digraph.
      using Node = typename Base::Digraph::Node;

      // Pointer to the digraph the algorithm runs on.
      void* _g;
      // Pointer to the map of reached nodes.
      void* _reached;
      // Pointer to the map of processed nodes.
      void* _processed;
      // Pointer to the map of predecessors arcs.
      void* _pred;
      // Pointer to the map of distances.
      void* _dist;
      // Pointer to the DFS path to the target node.
      void* _path;
      // Pointer to the distance of the target node.
      int* _di;

      public:
      /** Constructor. */

      /**
       * This constructor does not require parameters, it initiates
       * all of the attributes to \c 0.
       */
      DfsWizardBase() : _g(0), _reached(0), _processed(0), _pred(0), _dist(0), _path(0), _di(0) {}

      /** Constructor. */

      /**
       * This constructor requires one parameter,
       * others are initiated to \c 0.
       * @param g The digraph the algorithm runs on.
       */
      DfsWizardBase(const GR& g) :
          _g(reinterpret_cast< void* >(const_cast< GR* >(&g))), _reached(0), _processed(0), _pred(0), _dist(0),
          _path(0), _di(0) {}
    };

    /**
     * Auxiliary class for the function-type interface of DFS algorithm.
     */

    /**
     * This auxiliary class is created to implement the
     * @ref dfs() "function-type interface" of @ref Dfs algorithm.
     * It does not have own @ref run(Node) "run()" method, it uses the
     * functions and features of the plain @ref Dfs.
     *
     * This class should only be used through the @ref dfs() function,
     * which makes it easier to use the algorithm.
     *
     * @tparam TR The traits class that defines various types used by the
     * algorithm.
     */
    template < class TR >
    class DfsWizard: public TR {
      using Base = TR;

      using Digraph = typename TR::Digraph;

      using Node = typename Digraph::Node;
      using NodeIt = typename Digraph::NodeIt;
      using Arc = typename Digraph::Arc;
      using OutArcIt = typename Digraph::OutArcIt;

      using PredMap = typename TR::PredMap;
      using DistMap = typename TR::DistMap;
      using ReachedMap = typename TR::ReachedMap;
      using ProcessedMap = typename TR::ProcessedMap;
      using Path = typename TR::Path;

      public:
      /** Constructor. */
      DfsWizard() : TR() {}

      /** Constructor that requires parameters. */

      /**
       * Constructor that requires parameters.
       * These parameters will be the default values for the traits class.
       * @param g The digraph the algorithm runs on.
       */
      DfsWizard(const Digraph& g) : TR(g) {}

      /** Copy constructor */
      DfsWizard(const TR& b) : TR(b) {}

      ~DfsWizard() noexcept {}

      /** Runs DFS algorithm from the given source node. */

      /**
       * This method runs DFS algorithm from node \c s
       * in order to compute the DFS path to each node.
       */
      void run(Node s) {
        Dfs< Digraph, TR > alg(*reinterpret_cast< const Digraph* >(Base::_g));
        if (Base::_pred) alg.predMap(*reinterpret_cast< PredMap* >(Base::_pred));
        if (Base::_dist) alg.distMap(*reinterpret_cast< DistMap* >(Base::_dist));
        if (Base::_reached) alg.reachedMap(*reinterpret_cast< ReachedMap* >(Base::_reached));
        if (Base::_processed) alg.processedMap(*reinterpret_cast< ProcessedMap* >(Base::_processed));
        if (s != INVALID)
          alg.run(s);
        else
          alg.run();
      }

      /** Finds the DFS path between \c s and \c t. */

      /**
       * This method runs DFS algorithm from node \c s
       * in order to compute the DFS path to node \c t
       * (it stops searching when \c t is processed).
       *
       * @return \c true if \c t is reachable form \c s.
       */
      bool run(Node s, Node t) {
        Dfs< Digraph, TR > alg(*reinterpret_cast< const Digraph* >(Base::_g));
        if (Base::_pred) alg.predMap(*reinterpret_cast< PredMap* >(Base::_pred));
        if (Base::_dist) alg.distMap(*reinterpret_cast< DistMap* >(Base::_dist));
        if (Base::_reached) alg.reachedMap(*reinterpret_cast< ReachedMap* >(Base::_reached));
        if (Base::_processed) alg.processedMap(*reinterpret_cast< ProcessedMap* >(Base::_processed));
        alg.run(s, t);
        if (Base::_path) *reinterpret_cast< Path* >(Base::_path) = alg.path(t);
        if (Base::_di) *Base::_di = alg.dist(t);
        return alg.reached(t);
      }

      /** Runs DFS algorithm to visit all nodes in the digraph. */

      /**
       * This method runs DFS algorithm in order to visit all nodes
       * in the digraph.
       */
      void run() { run(INVALID); }

      template < class T >
      struct SetPredMapBase: public Base {
        using PredMap = T;
        static PredMap* createPredMap(const Digraph&) { return 0; };
        SetPredMapBase(const TR& b) : TR(b) {}
      };

      /**
       * @brief @ref named-templ-param "Named parameter" for setting
       * the predecessor map.
       *
       * @ref named-templ-param "Named parameter" function for setting
       * the map that stores the predecessor arcs of the nodes.
       */
      template < class T >
      DfsWizard< SetPredMapBase< T > > predMap(const T& t) {
        Base::_pred = reinterpret_cast< void* >(const_cast< T* >(&t));
        return DfsWizard< SetPredMapBase< T > >(*this);
      }

      template < class T >
      struct SetReachedMapBase: public Base {
        using ReachedMap = T;
        static ReachedMap* createReachedMap(const Digraph&) { return 0; };
        SetReachedMapBase(const TR& b) : TR(b) {}
      };

      /**
       * @brief @ref named-templ-param "Named parameter" for setting
       * the reached map.
       *
       * @ref named-templ-param "Named parameter" function for setting
       * the map that indicates which nodes are reached.
       */
      template < class T >
      DfsWizard< SetReachedMapBase< T > > reachedMap(const T& t) {
        Base::_reached = reinterpret_cast< void* >(const_cast< T* >(&t));
        return DfsWizard< SetReachedMapBase< T > >(*this);
      }

      template < class T >
      struct SetDistMapBase: public Base {
        using DistMap = T;
        static DistMap* createDistMap(const Digraph&) { return 0; };
        SetDistMapBase(const TR& b) : TR(b) {}
      };

      /**
       * @brief @ref named-templ-param "Named parameter" for setting
       * the distance map.
       *
       * @ref named-templ-param "Named parameter" function for setting
       * the map that stores the distances of the nodes calculated
       * by the algorithm.
       */
      template < class T >
      DfsWizard< SetDistMapBase< T > > distMap(const T& t) {
        Base::_dist = reinterpret_cast< void* >(const_cast< T* >(&t));
        return DfsWizard< SetDistMapBase< T > >(*this);
      }

      template < class T >
      struct SetProcessedMapBase: public Base {
        using ProcessedMap = T;
        static ProcessedMap* createProcessedMap(const Digraph&) { return 0; };
        SetProcessedMapBase(const TR& b) : TR(b) {}
      };

      /**
       * @brief @ref named-func-param "Named parameter" for setting
       * the processed map.
       *
       * @ref named-templ-param "Named parameter" function for setting
       * the map that indicates which nodes are processed.
       */
      template < class T >
      DfsWizard< SetProcessedMapBase< T > > processedMap(const T& t) {
        Base::_processed = reinterpret_cast< void* >(const_cast< T* >(&t));
        return DfsWizard< SetProcessedMapBase< T > >(*this);
      }

      template < class T >
      struct SetPathBase: public Base {
        using Path = T;
        SetPathBase(const TR& b) : TR(b) {}
      };
      /**
       * @brief @ref named-func-param "Named parameter"
       * for getting the DFS path to the target node.
       *
       * @ref named-func-param "Named parameter"
       * for getting the DFS path to the target node.
       */
      template < class T >
      DfsWizard< SetPathBase< T > > path(const T& t) {
        Base::_path = reinterpret_cast< void* >(const_cast< T* >(&t));
        return DfsWizard< SetPathBase< T > >(*this);
      }

      /**
       * @brief @ref named-func-param "Named parameter"
       * for getting the distance of the target node.
       *
       * @ref named-func-param "Named parameter"
       * for getting the distance of the target node.
       */
      DfsWizard dist(const int& d) {
        Base::_di = const_cast< int* >(&d);
        return *this;
      }
    };

    /** Function-type interface for DFS algorithm. */

    /**
     * @ingroup search
     * Function-type interface for DFS algorithm.
     *
     * This function also has several @ref named-func-param "named parameters",
     * they are declared as the members of class @ref DfsWizard.
     * The following examples show how to use these parameters.
     * @code
     * // Compute the DFS tree
     * dfs(g).predMap(preds).distMap(dists).run(s);
     *
     * // Compute the DFS path from s to t
     * bool reached = dfs(g).path(p).dist(d).run(s,t);
     * @endcode
     * @warning Don't forget to put the @ref DfsWizard::run(Node) "run()"
     * to the end of the parameter list.
     * \sa DfsWizard
     * \sa Dfs
     */
    template < class GR >
    DfsWizard< DfsWizardBase< GR > > dfs(const GR& digraph) {
      return DfsWizard< DfsWizardBase< GR > >(digraph);
    }

    /**
     * @brief Visitor class for DFS.
     *
     * This class defines the interface of the DfsVisit events, and
     * it could be the base of a real visitor class.
     */
    template < typename GR >
    struct DefaultDfsVisitor {
      using Digraph = GR;
      using Arc = typename Digraph::Arc;
      using Node = typename Digraph::Node;
      /**
       * @brief Called for the source node of the DFS.
       *
       * This function is called for the source node of the DFS.
       */
      constexpr void start(const Node& node) noexcept {}
      /**
       * @brief Called when the source node is leaved.
       *
       * This function is called when the source node is leaved.
       */
      constexpr void stop(const Node& node) noexcept {}
      /**
       * @brief Called when a node is reached first time.
       *
       * This function is called when a node is reached first time.
       */
      constexpr void reach(const Node& node) noexcept {}
      /**
       * @brief Called when an arc reaches a new node.
       *
       * This function is called when the DFS finds an arc whose target node
       * is not reached yet.
       */
      constexpr void discover(const Arc& arc) noexcept {}
      /**
       * @brief Called when an arc is examined but its target node is
       * already discovered.
       *
       * This function is called when an arc is examined but its target node is
       * already discovered.
       */
      constexpr void examine(const Arc& arc) noexcept {}
      /**
       * @brief Called when the DFS steps back from a node.
       *
       * This function is called when the DFS steps back from a node.
       */
      constexpr void leave(const Node& node) noexcept {}
      /**
       * @brief Called when the DFS steps back on an arc.
       *
       * This function is called when the DFS steps back on an arc.
       */
      constexpr void backtrack(const Arc& arc) noexcept {}
    };

    /**
     * @ingroup search
     *
     * @brief DFS algorithm class with visitor interface.
     *
     * This class provides an efficient implementation of the DFS algorithm
     * with visitor interface.
     *
     * The DfsVisit class provides an alternative interface to the Dfs
     * class. It works with callback mechanism, the DfsVisit object calls
     * the member functions of the \c Visitor class on every DFS event.
     *
     * This interface of the DFS algorithm should be used in special cases
     * when extra actions have to be performed in connection with certain
     * events of the DFS algorithm. Otherwise consider to use Dfs or dfs()
     * instead.
     *
     * @tparam GR The type of the digraph the algorithm runs on.
     * The value of GR is not used directly by @ref DfsVisit,
     * it is only passed to @ref DfsVisitDefaultTraits.
     * @tparam VS The Visitor type that is used by the algorithm.
     * @ref DfsVisitor "DfsVisitor<GR>" is an empty visitor, which
     * does not observe the DFS events. If you want to observe the DFS
     * events, you should implement your own visitor class.
     * @tparam TR The traits class that defines various types used by the
     * algorithm. By default, it is @ref DfsVisitDefaultTraits
     * "DfsVisitDefaultTraits<GR>".
     * In most cases, this parameter should not be set directly,
     * consider to use the named template parameters instead.
     */
    template < typename GR, typename VS = DefaultDfsVisitor< GR > >
    // requires nt::concepts::Digraph< GR >
    struct DfsVisit {
      using Digraph = GR;
      TEMPLATE_DIGRAPH_TYPEDEFS_EX(Digraph);

      using Dist = unsigned int;
      using StateMap = CharStaticNodeMap;
      using Visitor = VS;

      enum State { PRE = static_cast< char >(-1), IN = static_cast< char >(0), OUT = static_cast< char >(1) };

      const Digraph& _g;

      nt::Stack< OutArcIt >        _stack;
      StateMap                     _state_map;
      Visitor*                     _visitor;
      DefaultDfsVisitor< Digraph > _default_visitor;

      DfsVisit(const Digraph& g) : _g(g), _state_map(g), _visitor(&_default_visitor) {}
      DfsVisit(const Digraph& g, Visitor& visitor) : _g(g), _state_map(g), _visitor(&visitor) {}

      /**
       * \name Execution Control
       * The simplest way to execute the DFS algorithm is to use one of the
       * member functions called @ref run(Node) "run()".\n
       * If you need better control on the execution, you have to call
       * @ref init() first, then you can add a source node with @ref addSource()
       * and perform the actual computation with @ref start().
       * This procedure can be repeated if there are nodes that have not
       * been reached.
       */

      /** @{ */

      /**
       * @brief Initializes the internal data structures.
       *
       * Initializes the internal data structures.
       */
      constexpr void init() noexcept { clear(); }

      /**
       * @brief Adds a new source node.
       *
       * Adds a new source node to the set of nodes to be processed.
       *
       * @pre The stack must be empty. Otherwise the algorithm gives
       * wrong results. (One of the outgoing arcs of all the source nodes
       * except for the last one will not be visited and distances will
       * also be wrong.)
       */
      constexpr void addSource(Node s) noexcept {
        assert(emptyQueue());   // The stack must be empty.
        _visitor->start(s);
        _visitor->reach(s);
        OutArcIt e(_g, s);
        if (e != INVALID) {
          _stack.push(std::move(e));
          _state_map[s] = IN;
        } else {
          _visitor->leave(s);
          _state_map[s] = OUT;
          _visitor->stop(s);
        }
      }

      /**
       * @brief Processes the next arc.
       *
       * Processes the next arc.
       *
       * @return The processed arc.
       *
       * @pre The stack must not be empty.
       */
      constexpr Arc processNextArc() noexcept {
        const Arc e = _stack.top();
        Node      m = _g.target(e);

        if (!reached(m)) {
          _visitor->discover(e);
          _visitor->reach(m);
          _stack.push(OutArcIt(_g, m));
          _state_map[m] = IN;
        } else {
          _visitor->examine(e);
          m = _g.source(e);
          ++_stack.top();
        }

        while (!_stack.empty() && _stack.top() == INVALID) {
          _visitor->leave(m);
          _state_map[m] = OUT;
          _stack.pop();
          if (!_stack.empty()) {
            _visitor->backtrack(_stack.top());
            m = _g.source(_stack.top());
            ++_stack.top();
          } else {
            _visitor->stop(m);
          }
        }

        return e;
      }

      /**
       * @brief Next arc to be processed.
       *
       * Next arc to be processed.
       *
       * @return The next arc to be processed or INVALID if the stack is
       * empty.
       */
      constexpr Arc nextArc() const noexcept { return _stack.empty() ? nt::INVALID : _stack.top(); }

      /**
       * @brief Returns \c false if there are nodes
       * to be processed.
       *
       * Returns \c false if there are nodes
       * to be processed in the queue (stack).
       */
      constexpr bool emptyQueue() const noexcept { return _stack.empty(); }

      /**
       * @brief Returns the number of the nodes to be processed.
       *
       * Returns the number of the nodes to be processed in the queue (stack).
       */
      constexpr int queueSize() const noexcept { return _stack.size(); }

      /**
       * @brief Executes the algorithm.
       *
       * Executes the algorithm.
       *
       * This method runs the %DFS algorithm from the root node
       * in order to compute the %DFS path to each node.
       *
       * The algorithm computes
       * - the %DFS tree,
       * - the distance of each node from the root in the %DFS tree.
       *
       * @pre init() must be called and a root node should be
       * added with addSource() before using this function.
       *
       * @note <tt>d.start()</tt> is just a shortcut of the following code.
       * @code
       * while ( !d.emptyQueue() ) {
       * d.processNextArc();
       * }
       * @endcode
       */
      constexpr void start() noexcept {
        while (!emptyQueue())
          processNextArc();
      }

      /**
       * @brief Executes the algorithm until the given target node is reached.
       *
       * Executes the algorithm until the given target node is reached.
       *
       * This method runs the %DFS algorithm from the root node
       * in order to compute the DFS path to \c t.
       *
       * The algorithm computes
       * - the %DFS path to \c t,
       * - the distance of \c t from the root in the %DFS tree.
       *
       * @pre init() must be called and a root node should be added
       * with addSource() before using this function.
       */
      constexpr void start(Node t) noexcept {
        while (!emptyQueue() && !reached(t))
          processNextArc();
      }

      /**
       * @brief Executes the algorithm until a condition is met.
       *
       * Executes the algorithm until a condition is met.
       *
       * This method runs the %DFS algorithm from the root node
       * until an arc \c a with <tt>am[a]</tt> true is found.
       *
       * @param am A \c bool (or convertible) arc map. The algorithm
       * will stop when it reaches an arc \c a with <tt>am[a]</tt> true.
       *
       * @return The reached arc \c a with <tt>am[a]</tt> true or
       * \c INVALID if no such arc was found.
       *
       * @pre init() must be called and a root node should be added
       * with addSource() before using this function.
       *
       * @warning Contrary to @ref Bfs and @ref Dijkstra, \c am is an arc map,
       * not a node map.
       */
      template < typename AM >
      constexpr Arc start(const AM& am) noexcept {
        while (!emptyQueue() && !am[_stack.top()])
          processNextArc();
        return emptyQueue() ? INVALID : _stack.top();
      }

      /**
       * @brief Runs the algorithm from the given source node.
       *
       * This method runs the %DFS algorithm from node \c s.
       * in order to compute the DFS path to each node.
       *
       * The algorithm computes
       * - the %DFS tree,
       * - the distance of each node from the root in the %DFS tree.
       *
       * @note <tt>d.run(s)</tt> is just a shortcut of the following code.
       * @code
       * d.init();
       * d.addSource(s);
       * d.start();
       * @endcode
       */
      constexpr void run(Node s) noexcept {
        init();
        addSource(s);
        start();
      }

      /** @brief Finds the %DFS path between \c s and \c t. */

      /**
       * This method runs the %DFS algorithm from node \c s
       * in order to compute the DFS path to node \c t
       * (it stops searching when \c t is processed).
       *
       * @return \c true if \c t is reachable form \c s.
       *
       * @note Apart from the return value, <tt>d.run(s,t)</tt> is
       * just a shortcut of the following code.
       * @code
       * d.init();
       * d.addSource(s);
       * d.start(t);
       * @endcode
       */
      bool run(Node s, Node t) {
        init();
        addSource(s);
        start(t);
        return reached(t);
      }

      /**
       * @brief Runs the algorithm to visit all nodes in the digraph.
       */

      /**
       * This method runs the %DFS algorithm in order to visit all nodes
       * in the digraph.
       *
       * @note <tt>d.run()</tt> is just a shortcut of the following code.
       * @code
       * d.init();
       * for (NodeIt n(digraph); n != INVALID; ++n) {
       * if (!d.reached(n)) {
       * d.addSource(n);
       * d.start();
       * }
       * }
       * @endcode
       */
      constexpr void run() noexcept {
        init();
        for (NodeIt n(_g); n != INVALID; ++n) {
          if (!reached(n)) {
            addSource(n);
            start();
          }
        }
      }

      constexpr State state(Node v) const noexcept { return static_cast< State >(_state_map[v]); }
      constexpr bool  isIn(Node v) const noexcept { return _state_map[v] == IN; }
      constexpr bool  isOut(Node v) const noexcept { return _state_map[v] == OUT; }
      constexpr bool  isPre(Node v) const noexcept { return _state_map[v] == PRE; }

      /**
       * @brief Checks if the given node is reached from the root(s).
       *
       * Returns \c true if \c v is reached from the root(s).
       *
       * @pre Either @ref run(Node) "run()" or @ref init()
       * must be called before using this function.
       */
      constexpr bool reached(Node v) const noexcept { return state(v) != PRE; }

      /** @brief */
      constexpr void clear() noexcept {
        _stack.clear();
        _stack.reserve(nt::graphs::countNodes(_g));
        _state_map.setBitsToOne();   // Init all states to PRE (=-1)
      }
    };

  }   // namespace graphs
}   // namespace nt

#endif