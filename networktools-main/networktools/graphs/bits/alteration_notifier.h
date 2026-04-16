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
 * This file incorporates work from the LEMON graph library (alteration_notifier.h).
 *
 * Original LEMON Copyright Notice:
 * Copyright (C) 2003-2009 Egervary Jeno Kombinatorikus Optimalizalasi
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
 *   - Replaced std::list<ObserverBase*> with nt::TrivialDynamicArray<ObserverBase*>
 *   - Replaced exception-based error handling with error codes (ObserverReturnCode)
 *   - Updated include paths to networktools structure
 *   - Added STATIC template parameter for compile-time optimization
 *   - Removed locking mechanism (commented out Lock usage)
 *   - Added constexpr qualifiers for compile-time optimization
 *   - Added noexcept specifiers for better exception safety
 *   - Changed observer index from list iterator to integer index
 *   - Replaced std::vector<Item> with nt::ArrayView<Item> in observer interface
 *   - Changed virtual function signatures to return ObserverReturnCode instead of void
 *   - Replaced try-catch exception handling with error code checking
 *   - Changed from reverse iterator to index-based iteration
 *   - Removed throw() specifications (deprecated in C++11)
 *   - Added reserve() notification function
 *   - Added template specialization for STATIC=true with no-op implementation
 *   - Changed pointer initialization from 0 to nullptr
 *   - Converted typedef declarations to C++11 using declarations
 *   - Updated header guard macros
 *   - Explicitly deleted copy assignment operator using C++11 syntax
 *   - Modified detach() logic to handle array-based observer storage
 *   - Updated or enhanced Doxygen documentation
 */

/**
 * @ingroup graphbits
 * @file
 * @brief Observer notifier for graph alteration observers.
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_ALTERATION_NOTIFIER_H_
#define _NT_ALTERATION_NOTIFIER_H_

#include "../../core/common.h"
#include "../../core/arrays.h"

namespace nt {
  namespace graphs {

    /**
     * @ingroup graphbits
     *
     * @brief Notifier class to notify observers about alterations in a container.
     *
     * The simple graphs can be referred to as two containers: a node container
     * and an edge container. But they do not store values directly, they
     * are just key containers for more value containers, which are the
     * node and edge maps.
     *
     * The node and edge sets of the graphs can be changed as we add or erase
     * nodes and edges in the graph. NetworkTools handles this by notifying
     * all node and edge maps about alterations, ensuring they remain consistent.
     * Rather than checking on every access whether the map contains the current key
     * (which would cause performance drawbacks), we notify all maps about
     * alterations in the graph, which only impacts performance during graph modifications.
     *
     * This class provides an interface to a node or edge container.
     * The first() and next() member functions make it possible
     * to iterate on the keys of the container.
     * The id() function returns an integer id for each key.
     * The maxId() function gives back an upper bound of the ids.
     *
     * For proper functionality of this class, we should notify it
     * about each alteration in the container. The alterations have five types:
     * add(), erase(), build(), reserve(), and clear(). The add() and
     * erase() signals indicate that one or more items were added or erased.
     * If all items are erased from the graph or if a new graph
     * is built from an empty graph, this can be signaled with the
     * clear() and build() members. The reserve() member pre-allocates memory.
     * Important rule: when erasing items from graphs, we should first signal
     * the alteration and then erase them from the container. Conversely, for
     * item addition we should first extend the container and only after that
     * signal the alteration.
     *
     * The alteration can be observed with a class inherited from the
     * ObserverBase nested class. The signals can be handled by
     * overriding the virtual functions defined in the base class. The
     * observer base can be attached to the notifier with the
     * attach() member and can be detached with detach() function. The
     * alteration handlers should not call any function which signals
     * another alteration in the same notifier and should not
     * detach any observer from the notifier.
     *
     * Alteration observers use error codes instead of exceptions. If an add() or
     * build() function returns an error code, the remaining
     * observers will not be notified and the fulfilled additions will
     * be rolled back by calling the erase() or clear() functions.
     * The erase() and clear() functions return ObserverReturnCode and may
     * return ImmediateDetach to detach the observer from the notifier.
     *
     * There are some cases when the alteration observing is not completely
     * reliable. If we want to maintain the node degree in the graph
     * as in InDegMap and we use reverseArc(), this causes
     * unreliable functionality. Because the alteration observing signals
     * only erasing and adding but not reversing, it will store incorrect
     * degrees. Apart from that, subgraph adaptors cannot signal
     * alterations because just a setting in the filter map can modify
     * the graph and this cannot be watched in any way.
     *
     * @tparam _Container The container which is observed.
     * @tparam _Item The item type which is observed.
     * @tparam STATIC If true, uses a no-op implementation for compile-time optimization.
     */

    enum class ObserverReturnCode { Ok = 0, ImmediateDetach };

    template < typename _Container, typename _Item, bool STATIC >
    class AlterationNotifier {
      public:
      using Notifier = nt::True;
      using Container = _Container;
      using Item = _Item;

      /**
       * @brief ObserverBase is the base class for the observers.
       *
       * ObserverBase is the abstract base class for observers.
       * It will be notified when an item is inserted into or
       * erased from the graph.
       *
       * The observer interface contains pure virtual functions
       * to override. The add() and erase() functions
       * notify the observer when one or more items are added or erased.
       * All functions return ObserverReturnCode instead of throwing exceptions.
       *
       * The build() and clear() members notify the observer
       * when the container is built from an empty container or
       * is cleared to an empty container. The reserve() member
       * notifies about pre-allocation of memory.
       *
       * Returning ObserverReturnCode::ImmediateDetach from erase() or clear()
       * immediately detaches the observer from the notifier, which can be
       * used to invalidate the observer safely without exceptions.
       */
      class ObserverBase {
        protected:
        using Notifier = AlterationNotifier;

        friend class AlterationNotifier;

        /**
         * @brief Default constructor.
         *
         * Default constructor for ObserverBase.
         */
        ObserverBase() : _notifier(nullptr), _index(-1) {}

        /**
         * @brief Constructor which attach the observer into notifier.
         *
         * Constructor which attach the observer into notifier.
         */
        ObserverBase(AlterationNotifier& nf) { attach(nf); }

        /**
         * @brief Constructor which attach the observer to the same notifier.
         *
         * Constructor which attach the observer to the same notifier as
         * the other observer is attached to.
         */
        ObserverBase(const ObserverBase& copy) {
          if (copy.attached()) { attach(*copy.notifier()); }
        }

        /** @brief Destructor */
        constexpr virtual ~ObserverBase() noexcept {
          if (attached()) detach();
        }

        ObserverBase& operator=(const ObserverBase&) = delete;

        /**
         * @brief Attaches the observer into an AlterationNotifier.
         *
         * This member attaches the observer into an AlterationNotifier.
         */
        constexpr void attach(AlterationNotifier& nf) noexcept { nf.attach(*this); }

        /**
         * @brief Detaches the observer into an AlterationNotifier.
         *
         * This member detaches the observer from an AlterationNotifier.
         */
        constexpr void detach() noexcept { _notifier->detach(*this); }

        /**
         * @brief Gives back a pointer to the notifier which the map
         * attached into.
         *
         * This function gives back a pointer to the notifier which the map
         * attached into.
         */
        constexpr Notifier* notifier() const noexcept { return const_cast< Notifier* >(_notifier); }

        /**
         * Gives back true when the observer is attached into a notifier.
         */
        constexpr bool attached() const noexcept { return _notifier != nullptr; }

        protected:
        Notifier* _notifier;
        int       _index;

        /**
         * @brief The member function to notify the observer about an
         * item added to the container.
         *
         * The add() member function notifies the observer about an item
         * added to the container. It must be overridden in the
         * subclasses. Returns ObserverReturnCode::Ok on success.
         */
        constexpr virtual ObserverReturnCode add(const Item&) noexcept = 0;

        /**
         * @brief The member function to notify the observer about
         * multiple items added to the container.
         *
         * The add() member function notifies the observer about multiple items
         * added to the container. It must be overridden in the
         * subclasses. Returns ObserverReturnCode::Ok on success.
         */
        constexpr virtual ObserverReturnCode add(const nt::ArrayView< Item >& items) noexcept = 0;

        /**
         * @brief The member function to notify the observer about an
         * item erased from the container.
         *
         * The erase() member function notifies the observer about an
         * item erased from the container. It must be overridden in
         * the subclasses. May return ObserverReturnCode::ImmediateDetach to detach.
         */
        constexpr virtual ObserverReturnCode erase(const Item&) noexcept = 0;

        /**
         * @brief The member function to notify the observer about
         * multiple items erased from the container.
         *
         * The erase() member function notifies the observer about multiple items
         * erased from the container. It must be overridden in the
         * subclasses. May return ObserverReturnCode::ImmediateDetach to detach.
         */
        constexpr virtual ObserverReturnCode erase(const nt::ArrayView< Item >& items) noexcept = 0;

        /**
         * @brief The member function to notify the observer that the
         * container is built.
         *
         * The build() member function notifies the observer that the
         * container is built from an empty container. It must be
         * overridden in the subclasses. Returns ObserverReturnCode::Ok on success.
         */
        constexpr virtual ObserverReturnCode build() noexcept = 0;

        /**
         * @brief The member function to notify the observer about reserving
         * memory to accommodate n items.
         *
         * The reserve() member function notifies the observer about the pre-allocation
         * of the container to accommodate n items. It must be
         * overridden in the subclasses. Returns ObserverReturnCode::Ok on success.
         */
        constexpr virtual ObserverReturnCode reserve(int n) noexcept = 0;

        /**
         * @brief The member function to notify the observer that all
         * items are erased from the container.
         *
         * The clear() member function notifies the observer that all
         * items are erased from the container. It must be overridden in
         * the subclasses. May return ObserverReturnCode::ImmediateDetach to detach.
         */
        constexpr virtual ObserverReturnCode clear() noexcept = 0;
      };

      protected:
      const Container* container;

      using Observers = nt::TrivialDynamicArray< ObserverBase* >;
      Observers _observers;
      /** lemon::bits::Lock _lock; */

      public:
      /**
       * @brief Default constructor.
       *
       * The default constructor of the AlterationNotifier.
       * It creates an empty notifier.
       */
      AlterationNotifier() : container(nullptr) {}

      /**
       * @brief Constructor.
       *
       * Constructor with the observed container parameter.
       */
      AlterationNotifier(const Container& _container) : container(&_container) {}

      /**
       * @brief Copy Constructor of the AlterationNotifier.
       *
       * Copy constructor of the AlterationNotifier.
       * It creates only an empty notifier because the copiable
       * notifier's observers have to be registered still into that notifier.
       */
      AlterationNotifier(const AlterationNotifier& _notifier) : container(_notifier.container) {}

      AlterationNotifier& operator=(const AlterationNotifier&) = delete;

      /**
       * @brief Move Constructor of the AlterationNotifier.
       *
       * Move constructor of the AlterationNotifier.
       * It moves only the container pointer because the observers
       * have to be registered still into the new notifier.
       */
      AlterationNotifier(AlterationNotifier&& _notifier) noexcept :
          container(_notifier.container), _observers(std::move(_notifier._observers)) {
        _notifier.container = nullptr;
      }

      AlterationNotifier& operator=(AlterationNotifier&& _notifier) noexcept {
        if (this != &_notifier) {
          container = _notifier.container;
          _notifier.container = nullptr;
          _observers = std::move(_notifier._observers);
        }
        return *this;
      }

      /**
       * @brief Destructor.
       *
       * Destructor of the AlterationNotifier.
       */
      ~AlterationNotifier() noexcept { _observers.setBitsToZero(); }

      /**
       * @brief  Copies the state from another AlterationNotifier
       *
       */
      void copyFrom(const AlterationNotifier& other) noexcept {
        container = other.container;
        // _observers.copyFrom(other._observers);
      }

      /**
       * @brief Sets the container.
       *
       * Sets the container.
       */
      constexpr void setContainer(const Container& _container) noexcept { container = &_container; }

      public:
      /**
       * @brief First item in the container.
       *
       * Returns the first item in the container. Used to
       * start the iteration on the container.
       */
      constexpr void first(Item& item) const noexcept { container->first(item); }

      /**
       * @brief Next item in the container.
       *
       * Returns the next item in the container. Used to
       * iterate on the container.
       */
      constexpr void next(Item& item) const noexcept { container->next(item); }

      /**
       * @brief Returns the id of the item.
       *
       * Returns the id of the item provided by the container.
       */
      constexpr int id(const Item& item) const noexcept { return container->id(item); }

      /**
       * @brief Returns the maximum id of the container.
       *
       * Returns the maximum id of the container.
       */
      constexpr int maxId() const noexcept { return container->maxId(Item()); }

      protected:
      constexpr void attach(ObserverBase& observer) noexcept {
        /** _lock.lock(); */
        observer._index = _observers.add(&observer);
        observer._notifier = this;
        /** _lock.unlock(); */
      }

      constexpr void detach(ObserverBase& observer) noexcept {
        /** _lock.lock(); */
        _observers.remove(observer._index);
        if (observer._index < _observers.size()) _observers[observer._index]->_index = observer._index;
        observer._index = _observers.size();
        observer._notifier = nullptr;
        /** _lock.unlock(); */
      }

      public:
      /**
       * @brief Notifies all the registered observers about an item added to
       * the container.
       *
       * Notifies all registered observers about an item added to
       * the container. If any observer returns an error, the addition
       * is rolled back by calling erase() on all notified observers.
       */
      constexpr void add(const Item& item) noexcept {
        for (int i = 0; i < _observers.size(); ++i) {
          if (_observers[i]->add(item) != ObserverReturnCode::Ok) [[unlikely]] {
            /** Rollback: erase from all observers that were notified */
            for (int j = i; j >= 0; --j)
              _observers[j]->erase(item);
            return;
          }
        }
      }

      /**
       * @brief Notifies all the registered observers about multiple items added to
       * the container.
       *
       * Notifies all registered observers about multiple items added to
       * the container. If any observer returns an error, the addition
       * is rolled back by calling erase() on all notified observers.
       */
      constexpr void add(const nt::ArrayView< Item >& items) noexcept {
        for (int i = 0; i < _observers.size(); ++i) {
          if (_observers[i]->add(items) != ObserverReturnCode::Ok) [[unlikely]] {
            /** Rollback: erase from all observers that were notified */
            for (int j = i; j >= 0; --j)
              _observers[j]->erase(items);
            return;
          }
        }
      }

      /**
       * @brief Notifies all the registered observers about an item erased from
       * the container.
       *
       * Notifies all registered observers about an item erased from
       * the container. If an observer returns ImmediateDetach, it is
       * detached from the notifier. Iterates in reverse order.
       */
      constexpr void erase(const Item& item) noexcept {
        for (int i = _observers.size() - 1; i >= 0; --i) {
          ObserverBase& obs = *_observers[i];
          if (obs.erase(item) == ObserverReturnCode::ImmediateDetach) [[unlikely]] { detach(obs); }
        }
      }

      /**
       * @brief Notifies all the registered observers about multiple items erased
       * from the container.
       *
       * Notifies all registered observers about multiple items erased from
       * the container. If an observer returns ImmediateDetach, it is
       * detached from the notifier. Iterates in reverse order.
       */
      constexpr void erase(const nt::ArrayView< Item >& items) noexcept {
        for (int i = _observers.size() - 1; i >= 0; --i) {
          ObserverBase& obs = *_observers[i];
          if (obs.erase(items) == ObserverReturnCode::ImmediateDetach) [[unlikely]] { detach(obs); }
        }
      }

      /**
       * @brief Notifies all the registered observers that the container is
       * built.
       *
       * Notifies all registered observers that the container is built
       * from an empty container. If any observer returns an error, the build
       * is rolled back by calling clear() on all notified observers.
       */
      constexpr void build() noexcept {
        for (int i = 0; i < _observers.size(); ++i) {
          if (_observers[i]->build() != ObserverReturnCode::Ok) [[unlikely]] {
            /** Rollback: clear all observers that were notified */
            for (int j = i; j >= 0; --j)
              _observers[j]->clear();
            return;
          }
        }
      }

      /**
       * @brief Notifies all the registered observers about memory reservation.
       *
       * Notifies all registered observers that the container is reserving
       * memory to accommodate n items. Iterates in reverse order.
       */
      constexpr void reserve(int n) noexcept {
        for (int i = _observers.size() - 1; i >= 0; --i)
          _observers[i]->reserve(n);
      }

      /**
       * @brief Notifies all the registered observers that all items are
       * erased.
       *
       * Notifies all registered observers that all items are erased
       * from the container. If an observer returns ImmediateDetach, it is
       * detached from the notifier. Iterates in reverse order.
       */
      constexpr void clear() noexcept {
        for (int i = _observers.size() - 1; i >= 0; --i) {
          ObserverBase& obs = *_observers[i];
          if (obs.clear() == ObserverReturnCode::ImmediateDetach) [[unlikely]] { detach(obs); }
        }
      }
    };


    /**
     * @brief Template specialization for STATIC=true with no-op implementation.
     *
     * This specialization provides a compile-time optimized version that
     * performs no actual observation. All methods are constexpr no-ops.
     * Used when the graph structure is known to be static and doesn't need
     * alteration notifications.
     */
    template < typename _Container, typename _Item >
    struct AlterationNotifier< _Container, _Item, true > {
      using Notifier = nt::True;
      using Container = _Container;
      using Item = _Item;

      constexpr AlterationNotifier() = default;
      constexpr AlterationNotifier(const Container&) {}
      constexpr AlterationNotifier(const AlterationNotifier&) {}

      constexpr void setContainer(const Container&) {}

      constexpr void add(const Item&) noexcept {}
      constexpr void add(const nt::ArrayView< Item >&) noexcept {}
      constexpr void erase(const Item&) noexcept {}
      constexpr void erase(const nt::ArrayView< Item >&) noexcept {}
      constexpr void build() noexcept {}
      constexpr void clear() noexcept {}

      constexpr void first(Item&) const noexcept {}
      constexpr void next(Item&) const noexcept {}
      constexpr int  id(const Item&) const noexcept { return -1; }
      constexpr int  maxId() const noexcept { return -1; }
    };

  }   // namespace graphs
}   // namespace nt

#endif
