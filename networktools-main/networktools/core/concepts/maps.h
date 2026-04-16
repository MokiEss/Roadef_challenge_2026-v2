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
 * This file incorporates work from the LEMON graph library (maps.h).
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
 *   -
 */

/**
 * @ingroup map_concepts
 * @file
 * @brief The concept of maps.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_CONCEPTS_MAPS_H_
#define _NT_CONCEPTS_MAPS_H_

#include "../common.h"
#include "tags.h"

namespace nt {

  namespace concepts {

    /**
     * \addtogroup map_concepts
     * @{
     */

    /** Readable map concept */

    /**
     * Readable map concept.
     *
     */
    template < typename K, typename T >
    class ReadMap {
      public:
      /** The key type of the map. */
      typedef K Key;
      /**
       * @brief The value type of the map.
       * (The type of objects associated with the keys).
       */
      typedef T Value;

      /** Returns the value associated with the given key. */
      Value operator[](const Key&) const { return *(static_cast< Value* >(0) + 1); }

      template < typename _ReadMap >
      struct Constraints {
        void constraints() {
          Value val = m[key];
          val = m[key];
          typename _ReadMap::Value own_val = m[own_key];
          own_val = m[own_key];

          ::nt::ignore_unused_variable_warning(key);
          ::nt::ignore_unused_variable_warning(val);
          ::nt::ignore_unused_variable_warning(own_key);
          ::nt::ignore_unused_variable_warning(own_val);
        }
        const Key&                    key;
        const typename _ReadMap::Key& own_key;
        const _ReadMap&               m;
        Constraints() {}
      };
    };

    /** Writable map concept */

    /**
     * Writable map concept.
     *
     */
    template < typename K, typename T >
    class WriteMap {
      public:
      /** The key type of the map. */
      typedef K Key;
      /**
       * @brief The value type of the map.
       * (The type of objects associated with the keys).
       */
      typedef T Value;

      /** Sets the value associated with the given key. */
      void set(const Key&, const Value&) {}

      /** Default constructor. */
      WriteMap() {}

      template < typename _WriteMap >
      struct Constraints {
        void constraints() {
          m.set(key, val);
          m.set(own_key, own_val);

          ::nt::ignore_unused_variable_warning(key);
          ::nt::ignore_unused_variable_warning(val);
          ::nt::ignore_unused_variable_warning(own_key);
          ::nt::ignore_unused_variable_warning(own_val);
        }
        const Key&                       key;
        const Value&                     val;
        const typename _WriteMap::Key&   own_key;
        const typename _WriteMap::Value& own_val;
        _WriteMap&                       m;
        Constraints() {}
      };
    };

    /** Read/writable map concept */

    /**
     * Read/writable map concept.
     *
     */
    template < typename K, typename T >
    class ReadWriteMap: public ReadMap< K, T >, public WriteMap< K, T > {
      public:
      /** The key type of the map. */
      typedef K Key;
      /**
       * @brief The value type of the map.
       * (The type of objects associated with the keys).
       */
      typedef T Value;

      /** Returns the value associated with the given key. */
      Value operator[](const Key&) const {
        Value* r = 0;
        return *r;
      }

      /** Sets the value associated with the given key. */
      void set(const Key&, const Value&) {}

      template < typename _ReadWriteMap >
      struct Constraints {
        void constraints() {
          checkConcept< ReadMap< K, T >, _ReadWriteMap >();
          checkConcept< WriteMap< K, T >, _ReadWriteMap >();
        }
      };
    };

    /** Dereferable map concept */

    /**
     * Dereferable map concept.
     *
     */
    template < typename K, typename T, typename R, typename CR >
    class ReferenceMap: public ReadWriteMap< K, T > {
      public:
      /** Tag for reference maps. */
      typedef True ReferenceMapTag;
      /** The key type of the map. */
      typedef K Key;
      /**
       * @brief The value type of the map.
       * (The type of objects associated with the keys).
       */
      typedef T Value;
      /** The reference type of the map. */
      typedef R Reference;
      /** The const reference type of the map. */
      typedef CR ConstReference;

      public:
      /**
       * Returns a reference to the value associated with the given key.
       */
      Reference operator[](const Key&) {
        Value* r = 0;
        return *r;
      }

      /**
       * Returns a const reference to the value associated with the given key.
       */
      ConstReference operator[](const Key&) const {
        Value* r = 0;
        return *r;
      }

      /** Sets the value associated with the given key. */
      void set(const Key& k, const Value& t) { operator[](k) = t; }

      template < typename _ReferenceMap >
      struct Constraints {
        typename std::enable_if< nt::concepts::ReferenceMapIndicator< _ReferenceMap >, void >::type constraints() {
          checkConcept< ReadWriteMap< K, T >, _ReferenceMap >();
          ref = m[key];
          m[key] = val;
          m[key] = ref;
          m[key] = cref;
          own_ref = m[own_key];
          m[own_key] = own_val;
          m[own_key] = own_ref;
          m[own_key] = own_cref;
          m[key] = m[own_key];
          m[own_key] = m[key];
        }
        const Key&                             key;
        Value&                                 val;
        Reference                              ref;
        ConstReference                         cref;
        const typename _ReferenceMap::Key&     own_key;
        typename _ReferenceMap::Value&         own_val;
        typename _ReferenceMap::Reference      own_ref;
        typename _ReferenceMap::ConstReference own_cref;
        _ReferenceMap&                         m;
        Constraints() {}
      };
    };

    // @}

  }   // namespace concepts

}   // namespace nt

#endif
