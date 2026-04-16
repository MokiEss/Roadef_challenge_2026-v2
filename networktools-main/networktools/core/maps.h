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
 *   - Changed namespace from 'lemon' to 'nt'
 *   - Replaced std::vector with nt::TrivialDynamicArray/nt::DynamicArray
 *   - Updated include paths to networktools structure
 *   - Adapted LEMON concept checking to networktools
 *   - Converted typedef declarations to C++11 using declarations
 *   - Added constexpr qualifiers for compile-time optimization
 *   - Added noexcept specifiers for better exception safety
 *   - Adapted LEMON INVALID sentinel value handling
 *   - Added SFINAE/enable_if for better template constraints
 *   - Updated or enhanced Doxygen documentation
 *   - Significant code refactoring and restructuring
 *   - Added move semantics for performance
 *   - Updated header guard macros
 */

/**
 * @file
 * @brief This class acts as a base wrapper for different implementations of hash maps.
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_MAPS_H_
#define _NT_MAPS_H_

#include "common.h"
#include "string.h"
#include "arrays.h"
#include "logging.h"
#include "any.h"
#include "traits.h"
#include "../graphs/bits/alteration_notifier.h"

#include <map>

#if defined(NT_USE_STL_MAP)
#  include <unordered_map>
#else
#  include "../@deps/sparsehash/dense_hash_map"
#endif

#define DECLARE_SET_METHOD(_NAME, _TYPE) virtual bool _NAME(const Key& key, _TYPE i) noexcept = 0;

#define DEFINE_SET_METHOD(_NAME, _TYPE)                         \
  bool _NAME(const Key& key, _TYPE i) noexcept override {       \
    if constexpr (std::is_convertible< _TYPE, Value >::value) { \
      _map[key] = std::move(i);                                 \
      return true;                                              \
    }                                                           \
    return false;                                               \
  }

#define DECLARE_GET_METHOD(_NAME, _TYPE) virtual bool _NAME(const Key& key, _TYPE& i) const noexcept = 0;

#define DEFINE_GET_METHOD(_NAME, _TYPE)                          \
  bool _NAME(const Key& key, _TYPE& i) const noexcept override { \
    if constexpr (std::is_convertible< Value, _TYPE >::value) {  \
      i = _map[key];                                             \
      return true;                                               \
    }                                                            \
    return false;                                                \
  }

namespace nt {

  namespace details {
    /**
     * @class BaseMap
     * @headerfile maps.h
     * @brief This class acts as a base wrapper for different implementations of hash maps.
     * It should bot be instantiated directly but rather be specialized according to your needs.
     * Several basic maps (NumericalMap, PointerMap, ...) are implemented hereafter to cover most of
     * the use cases.
     *
     * By default, networktools uses the google::dense_hash_map implementation.
     *
     */
    template < class T_KEY,
               class V,
               class T_HASH_FUNC = std::hash< T_KEY >,
               class T_EQUAL_KEY = std::equal_to< T_KEY > >
    struct BaseMap {
#if defined(NT_USE_STL_MAP)
      using HashMap = std::unordered_map< T_KEY, V, T_HASH_FUNC, T_EQUAL_KEY >;
#else
      using HashMap = google::dense_hash_map< T_KEY, V, T_HASH_FUNC, T_EQUAL_KEY >;
#endif

      using iterator = typename HashMap::iterator;
      using const_iterator = typename HashMap::const_iterator;
      using size_type = typename HashMap::size_type;
      using key_type = typename HashMap::key_type;
      using value_type = typename HashMap::value_type;
      using data_type = typename HashMap::data_type;

      using Key = key_type;
      using Value = data_type;

      HashMap _hm;

      /**
       * @brief Reserves memory for at least `n` elements in the underlying array.
       *
       * @param n The number of elements to reserve memory for.
       */
      void reserve(size_type n) { _hm.rehash(n); }

      /**
       * @brief Checks whether a key is present in the ArrayMap.
       *
       * @param key The key to check.
       * @return true if the key is present, false otherwise.
       */
      bool hasKey(const Key& key) const { return _hm.find(key) != _hm.end(); }

      /**
       * @brief Return the value associated to the key if it exists, otherwise returns nullptr.
       *
       * Useful to quickly check the existence and get a value if any.
       * Typical usage :
       *
       * if(const Value* pValue = map.findOrNull(key)) {
       *   // Code using pValue
       * }
       *
       * @param key The key to search for.
       * @return A pointer to the value if the key exists, nullptr otherwise.
       */
      const Value* findOrNull(const Key& key) const {
        const_iterator it = _hm.find(key);
        if (it != _hm.end()) return &it->second;
        return nullptr;
      }

      /**
       * @brief Returns a pointer to the value associated with the given key if it exists, otherwise
       * returns nullptr.
       *
       * @param key The key to search for.
       * @return A pointer to the value if the key exists, nullptr otherwise.
       */
      Value* findOrNull(const Key& key) {
        iterator it = _hm.find(key);
        if (it != _hm.end()) return &it->second;
        return nullptr;
      }

      /**
       * @brief Returns the value associated with the key if it exists, otherwise returns a default
       * value.
       *
       * @param key The key to search for.
       * @param v The default value to return if the key does not exist.
       * @return The value associated with the key if it exists, otherwise the default value `v`.
       */
      const Value& findWithDefault(const Key& key, const Value& v) const {
        const auto it = _hm.find(key);
        if (it == _hm.end()) return v;
        return it->second;
      }

      /**
       * @brief Insert the pair Key-Value only if there is no collision.
       *
       * To be called like so : insertNoDuplicate( { Key, Value } )
       *
       * @param pair a Key-Value pair
       * @return Value* A pointer to the inserted value
       */
      Value* insertNoDuplicate(value_type&& pair) {
        BaseMap& m = *this;
        if (hasKey(pair.first)) {
          LOG_F(WARNING, "insertNoDuplicate : inserting already inserted key.");
          return nullptr;
        }
        return &m.insert(std::move(pair)).first->second;
      }

      /**
       * @brief Inserts a key-value pair into the ArrayMap only if there is no collision.
       *
       * @param key The key to insert.
       * @param v The value to insert.
       */
      void insertNoDuplicate(const Key& key, const Value& v) {
        BaseMap& m = *this;
        if (hasKey(key)) {
          LOG_F(WARNING, "insertNoDuplicate : inserting already inserted key.");
          return;
        }
        m[key] = v;
      }

      /**
       * @brief  Returns the value associated to the Key if it exists,
       * otherwise construct and insert a value at key and returns it.
       *
       * @param key The key to look for or insert.
       *
       * @return The found value or a default-created value if key does not exist
       */
      Value& operator[](const Key& key) { return _hm[key]; }
      Value& operator[](Key&& key) { return _hm[std::move(key)]; }

      /**
       * @brief  Returns the value associated to the Key. Contrary to the previous non const []
       * operators, if the key does not exist the behavior is undefined. Prefer a call to
       * findOrNull() or findWithDefault() if the existence of the key is not guaranteed.
       *
       * @param key The key to look for
       *
       * @return The found value
       */
      const Value& operator[](const Key& key) const {
        const auto it = _hm.find(key);
        assert(it != _hm.end());
        return it->second;
      }

      /**
       * @brief  Accessors to the underlying implementation of the hashmap
       *
       */
      const HashMap& underlyingHashMap() const noexcept { return _hm; }
      HashMap&       underlyingHashMap() noexcept { return _hm; }

      //---------------------------------------------------------------------------
      // LEMON API
      //---------------------------------------------------------------------------

      /**
       * @brief Builds the map with an initial size of `i_size`.
       *
       * @param i_size The initial size of the map.
       */
      void build(int i_size) noexcept { reserve(i_size); }

      /**
       * @brief Sets the value associated with the key.
       *
       * @param key The key to set.
       * @param val The value to set.
       */
      void set(const Key& key, const Value& val) noexcept { insert({key, val}); }

      /**
       * @brief Sets the value associated with the key.
       *
       * @param key The key to set.
       * @param val The value to set.
       */
      void set(const Key& key, Value&& val) noexcept { insert({key, std::move(val)}); }

      //---------------------------------------------------------------------------
      // STL methods
      //---------------------------------------------------------------------------

      // Lookup routines
      iterator       find(const key_type& key) { return _hm.find(key); }
      const_iterator find(const key_type& key) const { return _hm.find(key); }

      // Modifiers
      void clear() noexcept { _hm.clear(); }

      std::pair< iterator, bool > insert(const value_type& obj) { return _hm.insert(obj); }
      template < typename Pair,
                 typename = typename std::enable_if< std::is_constructible< value_type, Pair&& >::value >::type >
      std::pair< iterator, bool > insert(Pair&& obj) {
        return _hm.insert(std::forward< Pair >(obj));
      }
      // overload to allow {} syntax: .insert( { {key}, {args} } )
      std::pair< iterator, bool > insert(value_type&& obj) { return _hm.insert(std::move(obj)); }
      template < class InputIt >
      void insert(InputIt first, InputIt last) {
        _hm.insert(first, last);
      }
      void     insert(const_iterator first, const_iterator last) { _hm.insert(first, last); }
      void     insert(std::initializer_list< value_type > ilist) { _hm.insert(ilist.begin(), ilist.end()); }
      iterator insert(const_iterator, const value_type& obj) { return _hm.insert(obj).first; }
      iterator insert(const_iterator, value_type&& obj) { return _hm.insert(std::move(obj)).first; }
      template < class P,
                 class = typename std::enable_if< std::is_constructible< value_type, P&& >::value
                                                  && !std::is_same< value_type, P >::value >::type >
      iterator insert(const_iterator, P&& obj) {
        return _hm.insert(std::forward< P >(obj)).first;
      }

      // Capacity
      size_type size() const noexcept { return _hm.size(); }
      size_type max_size() const noexcept { return _hm.max_size(); }
      bool      empty() const noexcept { return _hm.empty(); }

      // Iterators
      iterator       begin() noexcept { return _hm.begin(); }
      iterator       end() noexcept { return _hm.end(); }
      const_iterator begin() const noexcept { return _hm.begin(); }
      const_iterator end() const noexcept { return _hm.end(); }
      const_iterator cbegin() const noexcept { return _hm.begin(); }
      const_iterator cend() const noexcept { return _hm.end(); }

      protected:
      BaseMap(){};   // This class serves only as a base class and must not be instantiated
                     // directly. One reason for this choice is that some implementation (like
                     // google's hashmap) requires to call some init methods before use.
    };
  }   // namespace details

  /**
   * @class NumericalMap
   * @headerfile maps.h
   * @brief Use this map if the keys are numerical values (char, int, float, etc...)
   *
   */
  template < class T_KEY, class V >
  struct NumericalMap: public details::BaseMap< T_KEY, V > {
    //--
    static_assert(std::is_arithmetic< T_KEY >::value, "NumericalMap : Non-numerical key type");
    //--
    NumericalMap() {
      NumericalMap& m = *this;
#if !defined(NT_USE_STL_MAP)
      // google::dense_hash_map requires the definition of a value to identify
      // free spaces. Here, we use the maximum possible value of T_KEY.
      m._hm.set_empty_key(std::numeric_limits< T_KEY >::max());
#endif
    }
    //--
  };

  /**
   * @class PointerMap
   * @headerfile maps.h
   * @brief Use this map if the keys are pointers (int*, Type*, ...)
   *
   */
  template < class T_KEY, class V >
  struct PointerMap: public details::BaseMap< T_KEY, V > {
    //--
    static_assert(std::is_pointer< T_KEY >::value, "PointerMap : Non-pointer key type");
    //--
    PointerMap() {
      PointerMap& m = *this;
#if !defined(NT_USE_STL_MAP)
      // google::dense_hash_map requires the definition of a value to identify
      // free spaces. Here, we use the null pointer value of T_KEY.
      m._hm.set_empty_key(nullptr);
#endif
    }
    //--
  };

  namespace details {
    struct SzCstringEq {
      bool operator()(const char* s1, const char* s2) const {
        return (s1 == s2) || (s1 && s2 && std::strcmp(s1, s2) == 0);
      }
    };

    struct SzCstringHash {
      uint64_t operator()(const char* bytes) const noexcept { return nt::xxh64::hash(bytes, std::strlen(bytes), 0); }
    };
  }   // namespace details

  /**
   * @class CstringMap
   * @headerfile maps.h
   * @brief Use this map if the keys are C-string (char*, const char*)
   *
   */
  template < class V >
  struct CstringMap: public details::BaseMap< const char*, V, details::SzCstringHash, details::SzCstringEq > {
    //--
    CstringMap() {
      CstringMap& m = *this;
#if !defined(NT_USE_STL_MAP)
      // google::dense_hash_map requires the definition of a value to identify
      // free spaces. Here, we use the null pointer value of T_KEY.
      m._hm.set_empty_key(nullptr);
#endif
    }
  };


  /**
   * @class ArrayMap
   * @headerfile maps.h
   * @brief An ArrayMap allows to manipulate an array as an hashmap.
   * The main difference between an array and an array map is the behavior of the [] operator.
   * In a array map, this operator will automatically allocate enough memory to access the i-th
   * element. Typical usage :
   *
   * ArrayMap<int> map;
   * map[10] = 1; // Allocate an array of size at least 10+1, and set the 10-th to 1.
   *
   */
  template < class V >
  struct ArrayMap {
    static_assert(std::is_arithmetic< V >::value, "ArrayMap : Non-numerical values");

    using Array = nt::TrivialDynamicArray< V >;

    using iterator = typename Array::iterator;
    using const_iterator = typename Array::const_iterator;
    using size_type = int;
    using key_type = int;
    using value_type = typename Array::value_type;
    using data_type = V;

    using Key = key_type;
    using Value = data_type;

    Array _array;

    /**
     * @brief Reserves memory for at least `n` elements in the underlying array.
     *
     * @param n The number of elements to reserve memory for.
     */
    void reserve(int n) noexcept { _array.ensureAndFillBytes(n, (~static_cast< V >(0))); }

    /**
     * @brief Checks whether a key is present in the ArrayMap.
     *
     * @param key The key to check.
     * @return true if the key is present, false otherwise.
     */
    constexpr bool hasKey(Key key) const noexcept {
      return key < _array.size() && _array[key] != (~static_cast< V >(0));
    }

    /**
     * @brief Return the value associated to the key if it exists, otherwise returns nullptr.
     *
     * Useful to quickly check the existence and get a value if any.
     * Typical usage :
     *
     * if(const Value* pValue = map.findOrNull(key)) {
     *   // Code using pValue
     * }
     *
     * @param key The key to search for.
     * @return A pointer to the value if the key exists, nullptr otherwise.
     */
    const Value* findOrNull(const Key& key) const noexcept {
      if (!hasKey(key)) return nullptr;
      return &_array[key];
    }

    /**
     * @brief Returns a pointer to the value associated with the given key if it exists, otherwise
     * returns nullptr.
     *
     * @param key The key to search for.
     * @return A pointer to the value if the key exists, nullptr otherwise.
     */
    Value* findOrNull(const Key& key) noexcept {
      if (!hasKey(key)) return nullptr;
      return &_array[key];
    }

    /**
     * @brief Returns the value associated with the key if it exists, otherwise returns a default
     * value.
     *
     * @param key The key to search for.
     * @param v The default value to return if the key does not exist.
     * @return The value associated with the key if it exists, otherwise the default value `v`.
     */
    const Value& findWithDefault(const Key& key, const Value& v) const noexcept {
      if (!hasKey(key)) return v;
      return _array[key].v;
    }

    /**
     * @brief Inserts a key-value pair into the ArrayMap.
     *
     * @param key The key to insert.
     * @param v The value to insert.
     */
    void insert(Key key, const Value& v) {
      _growIfNecessary(key);
      _array[key] = v;
    }

    /**
     * @brief Inserts a key-value pair into the ArrayMap.
     *
     * @param key The key to insert.
     * @param v The value to insert.
     */
    void insert(Key key, Value&& v) {
      _growIfNecessary(key);
      _array[key] = std::move(v);
    }

    /**
     * @brief Returns a reference to the value associated with the key.
     * If the key does not exist, it will be created.
     *
     * @param key The key to access.
     * @return A reference to the value associated with the key.
     */
    Value& operator[](Key key) noexcept {
      _growIfNecessary(key);
      return _array[key];
    }

    /**
     * @brief Returns a constant reference to the value associated with the key.
     * If the key does not exist, undefined behavior occurs.
     *
     * TODO(Morgan) : too much ambiguous with the other [] operator => to be put in another
     * function
     *
     * @param key The key to access.
     * @return A constant reference to the value associated with the key.
     */
    const Value& operator[](Key key) const noexcept { return _array[key]; }

    /**
     * @brief Returns a const reference to the underlying Array.
     *
     * @return A const reference to the underlying Array.
     */
    const Array& underlyingArray() const noexcept { return _array; }

    //---------------------------------------------------------------------------
    // LEMON API
    //---------------------------------------------------------------------------

    /**
     * @brief Builds the ArrayMap with an initial size of `i_size`.
     *
     * @param i_size The initial size of the ArrayMap.
     */
    void build(int i_size) noexcept { reserve(i_size); }

    /**
     * @brief Sets the value associated with the key.
     *
     * @param key The key to set.
     * @param val The value to set.
     */
    void set(const Key& key, const Value& val) noexcept { insert(key, val); }

    /**
     * @brief Sets the value associated with the key.
     *
     * @param key The key to set.
     * @param val The value to set.
     */
    void set(const Key& key, Value&& val) noexcept { insert(key, std::move(val)); }

    /**
     * @brief Sets all values in the ArrayMap to zero.
     */
    void setBitsToZero() noexcept { _array.setBitsToZero(); }

    /**
     * @brief Sets all values in the ArrayMap to one.
     */
    void setBitsToOne() noexcept { _array.setBitsToOne(); }

    // Private methods

    /**
     * @brief Grows the underlying array if necessary to accommodate the given index.
     *
     * @param i The index to grow the array for.
     */
    void _growIfNecessary(int i) { _array.ensureAndFillBytes(i + 1, (~static_cast< V >(0))); }

    //---------------------------------------------------------------------------
    // STL methods
    //---------------------------------------------------------------------------

    // Lookup routines

    /**
     * @brief Finds an iterator pointing to the key in the ArrayMap.
     *
     * @param key The key to search for.
     * @return An iterator pointing to the key if found, otherwise an iterator pointing to the
     * end.
     */
    iterator find(const key_type& key) {
      if (!hasKey(key)) return end();
      return iterator(_array._p_data + key);
    }

    /**
     * @brief Finds a constant iterator pointing to the key in the ArrayMap.
     *
     * @param key The key to search for.
     * @return A constant iterator pointing to the key if found, otherwise a constant iterator
     * pointing to the end.
     */
    const_iterator find(const key_type& key) const {
      if (!hasKey(key)) return end();
      return const_iterator(_array._p_data + key);
    }

    // Modifiers

    /**
     * @brief Clears all elements from the ArrayMap.
     */
    void clear() noexcept {
      _array.removeAll();
      _array.fillBytes(~static_cast< V >(0));
    }

    // Capacity

    /**
     * @brief Returns the number of elements in the ArrayMap.
     *
     * @return The number of elements in the ArrayMap.
     */
    size_type size() const noexcept {
      size_type s = 0;
      for (int i = 0; i < _array.size(); ++i)
        s += (_array[i] != (~static_cast< V >(0)));
      return s;
    }

    /**
     * @brief Returns the maximum number of elements the ArrayMap can hold.
     *
     * @return The maximum number of elements the ArrayMap can hold.
     */
    size_type max_size() const noexcept { return _array.capacity(); }

    /**
     * @brief Checks whether the ArrayMap is empty.
     *
     * @return true if the ArrayMap is empty, false otherwise.
     */
    bool empty() const noexcept { return _array.empty(); }

    // Iterators
    iterator       begin() noexcept { return _array.begin(); }
    iterator       end() noexcept { return _array.end(); }
    const_iterator begin() const noexcept { return _array.begin(); }
    const_iterator end() const noexcept { return _array.end(); }
    const_iterator cbegin() const noexcept { return _array.begin(); }
    const_iterator cend() const noexcept { return _array.end(); }
  };

  /**
   * @class BoolMap
   * @headerfile maps.h
   * @brief This data structure works similarly to an ArrayMap but is optimized for storing
   * boolean values.
   *
   * TODO(Morgan) :
   *  - complete STL interface
   *  - increase memory automatically when calling [] operator (same as ArrayMap)
   *
   */
  struct BoolMap {
    using ARRAY = nt::BitArray;
    using Key = int;
    using Value = bool;

    nt::BitArray _array;

    /**
     * @brief
     *
     * @param n
     */
    void reserve(int n) { _array.extendByBits(n); }

    /**
     * @brief
     *
     * @param key
     * @param v
     *
     */
    void insert(Key key, Value v) {
      if (v)
        _array.setOneAt(key);
      else
        _array.setZeroAt(key);
    }

    /**
     * @brief  Returns bit value associated to the key
     *
     * @param key
     * @return
     */
    Value operator[](Key k) const { return _array.isOne(k); }

    /**
     * @brief  Accessors to the underlying array
     *
     */
    const nt::BitArray& underlyingArray() const noexcept { return _array; }

    /**
     * @brief Set all map values to true
     *
     */
    void setTrue() noexcept { _array.setBitsToOne(); }

    /**
     * @brief Set
     *
     */
    void setTrue(Key key) noexcept { _array.setOneAt(key); }

    /**
     * @brief Set all map values to false
     *
     */
    void setFalse() noexcept { _array.setBitsToZero(); }

    /**
     * @brief Set
     *
     */
    void setFalse(Key key) noexcept { _array.setZeroAt(key); }

    //---------------------------------------------------------------------------
    // STL methods
    //---------------------------------------------------------------------------

    // Modifiers
    void clear() noexcept { _array.setBitsToZero(); }
  };

  namespace details {
    template < typename K >
    struct MapStorage {
      using Key = K;

      MapStorage() = default;
      MapStorage(const MapStorage&) = delete;
      constexpr virtual ~MapStorage() {}

      virtual AnyView get(const Key& key) const noexcept = 0;
      virtual bool    set(const Key& key, Any&& value) noexcept = 0;
      // virtual bool            set(const Key& key, const String& value) noexcept = 0;
      constexpr virtual void  copy(const Key& a, const Key& b) noexcept = 0;
      constexpr virtual void* getStoredMapPtr() const noexcept = 0;

      AnyView operator[](const Key& key) const noexcept { return get(key); }

      DECLARE_SET_METHOD(setBool, bool)
      DECLARE_SET_METHOD(setInt, int)
      DECLARE_SET_METHOD(setUint, unsigned)
      DECLARE_SET_METHOD(setInt64, int64_t)
      DECLARE_SET_METHOD(setUint64, uint64_t)
      DECLARE_SET_METHOD(setDouble, double)
      DECLARE_SET_METHOD(setFloat, float)
      DECLARE_SET_METHOD(setString, nt::String&&)

      DECLARE_GET_METHOD(getBool, bool)
      DECLARE_GET_METHOD(getInt, int)
      DECLARE_GET_METHOD(getUint, unsigned)
      DECLARE_GET_METHOD(getInt64, int64_t)
      DECLARE_GET_METHOD(getUint64, uint64_t)
      DECLARE_GET_METHOD(getDouble, double)
      DECLARE_GET_METHOD(getFloat, float)
      DECLARE_GET_METHOD(getString, nt::String)

      constexpr virtual Type key() const noexcept = 0;
      constexpr virtual Type value() const noexcept = 0;
    };

    template < typename K, typename M >
    struct MapStorageAdaptor: public MapStorage< typename M::Key > {
      using Parent = MapStorage< typename M::Key >;
      using Map = M;
      using Key = typename Map::Key;
      using Value = typename Map::Value;

      Map& _map;

      MapStorageAdaptor(Map& map) : _map(map) {}

      AnyView get(const Key& key) const noexcept override { return _map[key]; }

      bool set(const Key& key, Any&& value) noexcept override {
        Value* v = value.get< Value >();
        if (!v) return false;
        _map[key] = std::move(*v);
        return true;
      }

      // bool set(const Key& key, const String& value) noexcept override {
      //   std::istringstream is(value);
      //   Value              v;
      //   if (!(is >> v)) return false;

      //   char c;
      //   if (is >> std::ws >> c) return false;

      //   _map[key] = std::move(v);

      //   return true;
      // }

      /**
       * @brief Get the pointer to the underlying map.
       *
       * @return constexpr void*
       */
      constexpr void* getStoredMapPtr() const noexcept override { return &_map; }

      DEFINE_SET_METHOD(setBool, bool)
      DEFINE_SET_METHOD(setInt, int)
      DEFINE_SET_METHOD(setUint, unsigned)
      DEFINE_SET_METHOD(setInt64, int64_t)
      DEFINE_SET_METHOD(setUint64, uint64_t)
      DEFINE_SET_METHOD(setDouble, double)
      DEFINE_SET_METHOD(setFloat, float)
      DEFINE_SET_METHOD(setString, nt::String&&)

      DEFINE_GET_METHOD(getBool, bool)
      DEFINE_GET_METHOD(getInt, int)
      DEFINE_GET_METHOD(getUint, unsigned)
      DEFINE_GET_METHOD(getInt64, int64_t)
      DEFINE_GET_METHOD(getUint64, uint64_t)
      DEFINE_GET_METHOD(getDouble, double)
      DEFINE_GET_METHOD(getFloat, float)

      bool getString(const Key& key, nt::String& v) const noexcept override {
        if constexpr (nt::is_string(TypeId< Value >::value)) {
          v.assign(_map[key]);
          return true;
        }
        return false;
      }

      constexpr void copy(const Key& a, const Key& b) noexcept override {
        if constexpr (nt::is_array(TypeId< Value >::value) || nt::is_vector(TypeId< Value >::value)) {
          _map[a].copyFrom(_map[b]);
        } else if constexpr (nt::is_string(TypeId< Value >::value)) {
          _map[a].assign(_map[b]);
        } else {
          _map[a] = _map[b];
        }
      }

      constexpr Type key() const noexcept override { return TypeId< Key >::value; }
      constexpr Type value() const noexcept override { return TypeId< Value >::value; }

      // Map&       map() noexcept { return _map; }
      // const Map& map() const noexcept { return _map; }
    };
  }   // namespace details

  /** Base class of maps. */

  /**
   * Base class of maps. It provides the necessary type definitions
   * required by the map %concepts.
   */
  template < typename K, typename V >
  class MapBase {
    public:
    /** @brief The key type of the map. */
    using Key = K;
    /**
     * @brief The value type of the map.
     * (The type of objects associated with the keys).
     */
    using Value = V;
  };

  /** Null map. (a.k.a. DoNothingMap) */

  /**
   * This map can be used if you have to provide a map only for
   * its type definitions, or if you have to provide a writable map,
   * but data written to it is not required (i.e. it will be sent to
   * <tt>/dev/null</tt>).
   * It conforms to the @ref concepts::ReadWriteMap "ReadWriteMap" concept.
   *
   * \sa ConstMap
   */
  template < typename K, typename V >
  class NullMap: public MapBase< K, V > {
    public:
    
    using Key = K;
    
    using Value = V;

    /** Gives back a default constructed element. */
    constexpr Value operator[](const Key&) const { return Value(); }
    /** Absorbs the value. */
    constexpr void set(const Key&, const Value&) {}
  };

  /** Returns a \c NullMap class */

  /**
   * This function just returns a \c NullMap class.
   * \relates NullMap
   */
  template < typename K, typename V >
  NullMap< K, V > nullMap() {
    return NullMap< K, V >();
  }

  /** Constant map. */

  /**
   * This @ref concepts::ReadMap "readable map" assigns a specified
   * value to each key.
   *
   * In other aspects it is equivalent to \c NullMap.
   * So it conforms to the @ref concepts::ReadWriteMap "ReadWriteMap"
   * concept, but it absorbs the data written to it.
   *
   * The simplest way of using this map is through the constMap()
   * function.
   *
   * \sa NullMap
   * \sa IdentityMap
   */
  template < typename K, typename V >
  class ConstMap: public MapBase< K, V > {
    private:
    V _value;

    public:
    
    using Key = K;
    
    using Value = V;

    /** Default constructor */

    /**
     * Default constructor.
     * The value of the map will be default constructed.
     */
    ConstMap() {}

    /** Constructor with specified initial value */

    /**
     * Constructor with specified initial value.
     * @param v The initial value of the map.
     */
    ConstMap(const Value& v) : _value(v) {}

    /** Gives back the specified value. */
    Value operator[](const Key&) const { return _value; }

    /** Absorbs the value. */
    void set(const Key&, const Value&) {}

    /** Sets the value that is assigned to each key. */
    void setAll(const Value& v) { _value = v; }

    template < typename V1 >
    ConstMap(const ConstMap< K, V1 >&, const Value& v) : _value(v) {}
  };

  /** Returns a \c ConstMap class */

  /**
   * This function just returns a \c ConstMap class.
   * \relates ConstMap
   */
  template < typename K, typename V >
  inline ConstMap< K, V > constMap(const V& v) {
    return ConstMap< K, V >(v);
  }

  template < typename K, typename V >
  inline ConstMap< K, V > constMap() {
    return ConstMap< K, V >();
  }

  template < typename T, T v >
  struct Const {};

  /** Constant map with inlined constant value. */

  /**
   * This @ref concepts::ReadMap "readable map" assigns a specified
   * value to each key.
   *
   * In other aspects it is equivalent to \c NullMap.
   * So it conforms to the @ref concepts::ReadWriteMap "ReadWriteMap"
   * concept, but it absorbs the data written to it.
   *
   * The simplest way of using this map is through the constMap()
   * function.
   *
   * \sa NullMap
   * \sa IdentityMap
   */
  template < typename K, typename V, V v >
  class ConstMap< K, Const< V, v > >: public MapBase< K, V > {
    public:
    
    using Key = K;
    
    using Value = V;

    /** Constructor. */
    ConstMap() {}

    /** Gives back the specified value. */
    Value operator[](const Key&) const { return v; }

    /** Absorbs the value. */
    void set(const Key&, const Value&) {}
  };

  /** Returns a \c ConstMap class with inlined constant value */

  /**
   * This function just returns a \c ConstMap class with inlined
   * constant value.
   * \relates ConstMap
   */
  template < typename K, typename V, V v >
  inline ConstMap< K, Const< V, v > > constMap() {
    return ConstMap< K, Const< V, v > >();
  }

  /** Identity map. */

  /**
   * This @ref concepts::ReadMap "read-only map" gives back the given
   * key as value without any modification.
   *
   * \sa ConstMap
   */
  template < typename T >
  class IdentityMap: public MapBase< T, T > {
    public:
    
    using Key = T;
    
    using Value = T;

    /** Gives back the given value without any modification. */
    Value operator[](const Key& k) const { return k; }
  };

  /** Returns an \c IdentityMap class */

  /**
   * This function just returns an \c IdentityMap class.
   * \relates IdentityMap
   */
  template < typename T >
  inline IdentityMap< T > identityMap() {
    return IdentityMap< T >();
  }

  /**
   * @brief Map for storing values for integer keys from the range
   * <tt>[0..size-1]</tt>.
   *
   * This map is essentially a wrapper for \c nt::DynamicArray. It assigns
   * values to integer keys from the range <tt>[0..size-1]</tt>.
   * It can be used together with some data structures, e.g.
   * heap types and \c UnionFind, when the used items are small
   * integers. This map conforms to the @ref concepts::ReferenceMap
   * "ReferenceMap" concept.
   *
   * The simplest way of using this map is through the rangeMap()
   * function.
   */
  template < typename V >
  class RangeMap: public MapBase< int, V > {
    template < typename V1 >
    friend class RangeMap;

    private:
    using Vector = nt::DynamicArray< V >;
    Vector _vector;

    public:
    /** Key type */
    using Key = int;
    /** Value type */
    using Value = V;
    /** Reference type */
    using Reference = typename Vector::reference;
    /** Const reference type */
    using ConstReference = typename Vector::const_reference;

    using ReferenceMapTag = True;

    public:
    /** Default constructor. */
    RangeMap() = default;

    /**
     * Constructor with specified size.
     * @param size The size of the map.
     */
    RangeMap(int size) : _vector(size) {}

    /**
     * Constructs the map with specified size and default value.
     * @param size The size of the map.
     * @param value The default value to assign to all elements.
     */
    RangeMap(int size, const Value& value) : _vector(size, value) {}

    /** Constructs the map from an appropriate \c nt::DynamicArray. */
    RangeMap(nt::DynamicArray< V >&& vector) : _vector(std::move(vector)) {}

    /** Constructs the map from another \c RangeMap. */
    RangeMap(RangeMap< V >&& c) : _vector(std::move(c._vector)) {}

    // Copy constructor/operator are deleted to enforce the use of copyFrom().
    // This design choice prevents accidental (and possibly costly) copies
    // with the = operator.
    RangeMap(const RangeMap&) = delete;
    RangeMap& operator=(const RangeMap&) = delete;

    /** Returns the size of the map. */
    constexpr int size() noexcept { return _vector.size(); }

    /** Resizes the map. */

    /**
     * Resizes the underlying \c nt::DynamicArray container, so changes the
     * keyset of the map.
     * @param size The new size of the map. The new keyset will be the
     * range <tt>[0..size-1]</tt>.
     * @param value The default value to assign to the new keys.
     */
    constexpr void resize(int size, const Value& value = Value()) noexcept { _vector.resize(size, value); }

    /**
     * Resizes the map and constructs new elements in-place.
     * @param size The new size of the map. The new keyset will be the
     * range <tt>[0..size-1]</tt>.
     * @param args Arguments to forward to the constructor of the new elements.
     */
    template < typename... Args >
    void resizeEmplace(int size, Args&&... args) noexcept {
      _vector.resizeEmplace(size, std::forward< Args >(args)...);
    }

    public:
    
    constexpr void copyFrom(const DynamicArray< V >& other) noexcept { _vector.copyFrom(other); }
    constexpr void copyFrom(const RangeMap< V >& other) noexcept { _vector.copyFrom(other._vector); }


    constexpr Reference operator[](const Key& k) noexcept { return _vector[k]; }
    constexpr ConstReference operator[](const Key& k) const noexcept { return _vector[k]; }

    constexpr void set(const Key& k, const Value& v) noexcept { _vector[k] = v; }
  };

  /** Returns a \c RangeMap class */

  /**
   * This function just returns a \c RangeMap class.
   * \relates RangeMap
   */
  template < typename V >
  inline RangeMap< V > rangeMap(int size = 0, const V& value = V()) noexcept {
    return RangeMap< V >(size, value);
  }

  /**
   * @brief Returns a \c RangeMap class created from an appropriate
   * \c nt::ArrayView
   */

  /**
   * This function just returns a \c RangeMap class created from an
   * appropriate \c nt::ArrayView.
   * \relates RangeMap
   */
  template < typename V >
  inline RangeMap< V > rangeMap(nt::DynamicArray< V >&& vector) noexcept {
    return RangeMap< V >(std::move(vector));
  }


  /** Map type based on \c std::map */

  /**
   * This map is essentially a wrapper for \c std::map with addition
   * that you can specify a default value for the keys that are not
   * stored actually. This value can be different from the default
   * contructed value (i.e. \c %Value()).
   * This type conforms to the @ref concepts::ReferenceMap "ReferenceMap"
   * concept.
   *
   * This map is useful if a default value should be assigned to most of
   * the keys and different values should be assigned only to a few
   * keys (i.e. the map is "sparse").
   * The name of this type also refers to this important usage.
   *
   * Apart form that, this map can be used in many other cases since it
   * is based on \c std::map, which is a general associative container.
   * However, keep in mind that it is usually not as efficient as other
   * maps.
   *
   * The simplest way of using this map is through the sparseMap()
   * function.
   */
  template < typename K, typename V, typename Comp = std::less< K > >
  class SparseMap: public MapBase< K, V > {
    template < typename K1, typename V1, typename C1 >
    friend class SparseMap;

    public:
    /** Key type */
    using Key = K;
    /** Value type */
    using Value = V;
    /** Reference type */
    using Reference = Value&;
    /** Const reference type */
    using ConstReference = const Value&;

    using ReferenceMapTag = True;

    private:
    using Map = std::map< K, V, Comp >;
    Map   _map;
    Value _value;

    public:
    /** @brief Constructor with specified default value. */
    SparseMap(const Value& value = Value()) : _value(value) {}
    /**
     * @brief Constructs the map from an appropriate \c std::map, and
     * explicitly specifies a default value.
     */
    template < typename V1, typename Comp1 >
    SparseMap(const std::map< Key, V1, Comp1 >& map, const Value& value = Value()) :
        _map(map.begin(), map.end()), _value(value) {}

    /** @brief Constructs the map from another \c SparseMap. */
    template < typename V1, typename Comp1 >
    SparseMap(const SparseMap< Key, V1, Comp1 >& c) : _map(c._map.begin(), c._map.end()), _value(c._value) {}

    private:
    SparseMap& operator=(const SparseMap&);

    public:
    
    Reference operator[](const Key& k) {
      typename Map::iterator it = _map.lower_bound(k);
      if (it != _map.end() && !_map.key_comp()(k, it->first))
        return it->second;
      else
        return _map.insert(it, std::make_pair(k, _value))->second;
    }

    
    ConstReference operator[](const Key& k) const {
      typename Map::const_iterator it = _map.find(k);
      if (it != _map.end())
        return it->second;
      else
        return _value;
    }

    
    void set(const Key& k, const Value& v) {
      typename Map::iterator it = _map.lower_bound(k);
      if (it != _map.end() && !_map.key_comp()(k, it->first))
        it->second = v;
      else
        _map.insert(it, std::make_pair(k, v));
    }

    
    void setAll(const Value& v) {
      _value = v;
      _map.clear();
    }
  };

  /** Returns a \c SparseMap class */

  /**
   * This function just returns a \c SparseMap class with specified
   * default value.
   * \relates SparseMap
   */
  template < typename K, typename V, typename Compare >
  inline SparseMap< K, V, Compare > sparseMap(const V& value = V()) {
    return SparseMap< K, V, Compare >(value);
  }

  template < typename K, typename V >
  inline SparseMap< K, V, std::less< K > > sparseMap(const V& value = V()) {
    return SparseMap< K, V, std::less< K > >(value);
  }

  /**
   * @brief Returns a \c SparseMap class created from an appropriate
   * \c std::map
   */

  /**
   * This function just returns a \c SparseMap class created from an
   * appropriate \c std::map.
   * \relates SparseMap
   */
  template < typename K, typename V, typename Compare >
  inline SparseMap< K, V, Compare > sparseMap(const std::map< K, V, Compare >& map, const V& value = V()) {
    return SparseMap< K, V, Compare >(map, value);
  }

  /** @} */

  /**
   * \addtogroup map_adaptors
   * @{
   */

  /** Composition of two maps */

  /**
   * This @ref concepts::ReadMap "read-only map" returns the
   * composition of two given maps. That is to say, if \c m1 is of
   * type \c M1 and \c m2 is of \c M2, then for
   * @code
   * ComposeMap<M1, M2> cm(m1,m2);
   * @endcode
   * <tt>cm[x]</tt> will be equal to <tt>m1[m2[x]]</tt>.
   *
   * The \c Key type of the map is inherited from \c M2 and the
   * \c Value type is from \c M1.
   * \c M2::Value must be convertible to \c M1::Key.
   *
   * The simplest way of using this map is through the composeMap()
   * function.
   *
   * \sa CombineMap
   */
  template < typename M1, typename M2 >
  class ComposeMap: public MapBase< typename M2::Key, typename M1::Value > {
    const M1& _m1;
    const M2& _m2;

    public:
    
    using Key = typename M2::Key;
    
    using Value = typename M1::Value;

    /** Constructor */
    ComposeMap(const M1& m1, const M2& m2) : _m1(m1), _m2(m2) {}

    
    typename MapTraits< M1 >::ConstReturnValue operator[](const Key& k) const { return _m1[_m2[k]]; }
  };

  /** Returns a \c ComposeMap class */

  /**
   * This function just returns a \c ComposeMap class.
   *
   * If \c m1 and \c m2 are maps and the \c Value type of \c m2 is
   * convertible to the \c Key of \c m1, then <tt>composeMap(m1,m2)[x]</tt>
   * will be equal to <tt>m1[m2[x]]</tt>.
   *
   * \relates ComposeMap
   */
  template < typename M1, typename M2 >
  inline ComposeMap< M1, M2 > composeMap(const M1& m1, const M2& m2) {
    return ComposeMap< M1, M2 >(m1, m2);
  }

  /** Combination of two maps using an STL (binary) functor. */

  /**
   * This @ref concepts::ReadMap "read-only map" takes two maps and a
   * binary functor and returns the combination of the two given maps
   * using the functor.
   * That is to say, if \c m1 is of type \c M1 and \c m2 is of \c M2
   * and \c f is of \c F, then for
   * @code
   * CombineMap<M1,M2,F,V> cm(m1,m2,f);
   * @endcode
   * <tt>cm[x]</tt> will be equal to <tt>f(m1[x],m2[x])</tt>.
   *
   * The \c Key type of the map is inherited from \c M1 (\c M1::Key
   * must be convertible to \c M2::Key) and the \c Value type is \c V.
   * \c M2::Value and \c M1::Value must be convertible to the
   * corresponding input parameter of \c F and the return type of \c F
   * must be convertible to \c V.
   *
   * The simplest way of using this map is through the combineMap()
   * function.
   *
   * \sa ComposeMap
   */
  template < typename M1, typename M2, typename F, typename V = typename F::result_type >
  class CombineMap: public MapBase< typename M1::Key, V > {
    const M1& _m1;
    const M2& _m2;
    F         _f;

    public:
    
    using Key = typename M1::Key;
    
    using Value = V;

    /** Constructor */
    CombineMap(const M1& m1, const M2& m2, const F& f = F()) : _m1(m1), _m2(m2), _f(f) {}
    
    Value operator[](const Key& k) const { return _f(_m1[k], _m2[k]); }
  };

  /** Returns a \c CombineMap class */

  /**
   * This function just returns a \c CombineMap class.
   *
   * For example, if \c m1 and \c m2 are both maps with \c double
   * values, then
   * @code
   * combineMap(m1,m2,std::plus<double>())
   * @endcode
   * is equivalent to
   * @code
   * addMap(m1,m2)
   * @endcode
   *
   * This function is specialized for adaptable binary function
   * classes and C++ functions.
   *
   * \relates CombineMap
   */
  template < typename M1, typename M2, typename F, typename V >
  inline CombineMap< M1, M2, F, V > combineMap(const M1& m1, const M2& m2, const F& f) {
    return CombineMap< M1, M2, F, V >(m1, m2, f);
  }

  template < typename M1, typename M2, typename F >
  inline CombineMap< M1, M2, F, typename F::result_type > combineMap(const M1& m1, const M2& m2, const F& f) {
    return combineMap< M1, M2, F, typename F::result_type >(m1, m2, f);
  }

  template < typename M1, typename M2, typename K1, typename K2, typename V >
  inline CombineMap< M1, M2, V (*)(K1, K2), V > combineMap(const M1& m1, const M2& m2, V (*f)(K1, K2)) {
    return combineMap< M1, M2, V (*)(K1, K2), V >(m1, m2, f);
  }

  /** Converts an STL style (unary) functor to a map */

  /**
   * This @ref concepts::ReadMap "read-only map" returns the value
   * of a given functor. Actually, it just wraps the functor and
   * provides the \c Key and \c Value typedefs.
   *
   * Template parameters \c K and \c V will become its \c Key and
   * \c Value. In most cases they have to be given explicitly because
   * a functor typically does not provide \c argument_type and
   * \c result_type typedefs.
   * Parameter \c F is the type of the used functor.
   *
   * The simplest way of using this map is through the functorToMap()
   * function.
   *
   * \sa MapToFunctor
   */
  template < typename F, typename K = typename F::argument_type, typename V = typename F::result_type >
  class FunctorToMap: public MapBase< K, V > {
    F _f;

    public:
    
    using Key = K;
    
    using Value = V;

    /** Constructor */
    FunctorToMap(const F& f = F()) : _f(f) {}
    
    Value operator[](const Key& k) const { return _f(k); }
  };

  /** Returns a \c FunctorToMap class */

  /**
   * This function just returns a \c FunctorToMap class.
   *
   * This function is specialized for adaptable binary function
   * classes and C++ functions.
   *
   * \relates FunctorToMap
   */
  template < typename K, typename V, typename F >
  inline FunctorToMap< F, K, V > functorToMap(const F& f) {
    return FunctorToMap< F, K, V >(f);
  }

  template < typename F >
  inline FunctorToMap< F, typename F::argument_type, typename F::result_type > functorToMap(const F& f) {
    return FunctorToMap< F, typename F::argument_type, typename F::result_type >(f);
  }

  template < typename K, typename V >
  inline FunctorToMap< V (*)(K), K, V > functorToMap(V (*f)(K)) {
    return FunctorToMap< V (*)(K), K, V >(f);
  }

  /** Converts a map to an STL style (unary) functor */

  /**
   * This class converts a map to an STL style (unary) functor.
   * That is it provides an <tt>operator()</tt> to read its values.
   *
   * For the sake of convenience it also works as a usual
   * @ref concepts::ReadMap "readable map", i.e. <tt>operator[]</tt>
   * and the \c Key and \c Value typedefs also exist.
   *
   * The simplest way of using this map is through the mapToFunctor()
   * function.
   *
   * \sa FunctorToMap
   */
  template < typename M >
  class MapToFunctor: public MapBase< typename M::Key, typename M::Value > {
    const M& _m;

    public:
    
    using Key = typename M::Key;
    
    using Value = typename M::Value;

    using argument_type = typename M::Key;
    using result_type = typename M::Value;

    /** Constructor */
    MapToFunctor(const M& m) : _m(m) {}
    
    Value operator()(const Key& k) const { return _m[k]; }
    
    Value operator[](const Key& k) const { return _m[k]; }
  };

  /** Returns a \c MapToFunctor class */

  /**
   * This function just returns a \c MapToFunctor class.
   * \relates MapToFunctor
   */
  template < typename M >
  inline MapToFunctor< M > mapToFunctor(const M& m) {
    return MapToFunctor< M >(m);
  }

  /**
   * @brief Map adaptor to convert the \c Value type of a map to
   * another type using the default conversion.
   */

  /**
   * Map adaptor to convert the \c Value type of a @ref concepts::ReadMap
   * "readable map" to another type using the default conversion.
   * The \c Key type of it is inherited from \c M and the \c Value
   * type is \c V.
   * This type conforms to the @ref concepts::ReadMap "ReadMap" concept.
   *
   * The simplest way of using this map is through the convertMap()
   * function.
   */
  template < typename M, typename V >
  class ConvertMap: public MapBase< typename M::Key, V > {
    const M& _m;

    public:
    
    using Key = typename M::Key;
    
    using Value = V;

    /** Constructor */

    /**
     * Constructor.
     * @param m The underlying map.
     */
    ConvertMap(const M& m) : _m(m) {}

    
    Value operator[](const Key& k) const { return _m[k]; }
  };

  /** Returns a \c ConvertMap class */

  /**
   * This function just returns a \c ConvertMap class.
   * \relates ConvertMap
   */
  template < typename V, typename M >
  inline ConvertMap< M, V > convertMap(const M& map) {
    return ConvertMap< M, V >(map);
  }

  /** Applies all map setting operations to two maps */

  /**
   * This map has two @ref concepts::WriteMap "writable map" parameters
   * and each write request will be passed to both of them.
   * If \c M1 is also @ref concepts::ReadMap "readable", then the read
   * operations will return the corresponding values of \c M1.
   *
   * The \c Key and \c Value types are inherited from \c M1.
   * The \c Key and \c Value of \c M2 must be convertible from those
   * of \c M1.
   *
   * The simplest way of using this map is through the forkMap()
   * function.
   */
  template < typename M1, typename M2 >
  class ForkMap: public MapBase< typename M1::Key, typename M1::Value > {
    M1& _m1;
    M2& _m2;

    public:
    
    using Key = typename M1::Key;
    
    using Value = typename M1::Value;

    /** Constructor */
    ForkMap(M1& m1, M2& m2) : _m1(m1), _m2(m2) {}
    /**
     * Returns the value associated with the given key in the first map.
     */
    Value operator[](const Key& k) const { return _m1[k]; }
    /** Sets the value associated with the given key in both maps. */
    void set(const Key& k, const Value& v) {
      _m1.set(k, v);
      _m2.set(k, v);
    }
  };

  /** Returns a \c ForkMap class */

  /**
   * This function just returns a \c ForkMap class.
   * \relates ForkMap
   */
  template < typename M1, typename M2 >
  inline ForkMap< M1, M2 > forkMap(M1& m1, M2& m2) {
    return ForkMap< M1, M2 >(m1, m2);
  }

  /** Sum of two maps */

  /**
   * This @ref concepts::ReadMap "read-only map" returns the sum
   * of the values of the two given maps.
   * Its \c Key and \c Value types are inherited from \c M1.
   * The \c Key and \c Value of \c M2 must be convertible to those of
   * \c M1.
   *
   * If \c m1 is of type \c M1 and \c m2 is of \c M2, then for
   * @code
   * AddMap<M1,M2> am(m1,m2);
   * @endcode
   * <tt>am[x]</tt> will be equal to <tt>m1[x]+m2[x]</tt>.
   *
   * The simplest way of using this map is through the addMap()
   * function.
   *
   * \sa SubMap, MulMap, DivMap
   * \sa ShiftMap, ShiftWriteMap
   */
  template < typename M1, typename M2 >
  class AddMap: public MapBase< typename M1::Key, typename M1::Value > {
    const M1& _m1;
    const M2& _m2;

    public:
    
    using Key = typename M1::Key;
    
    using Value = typename M1::Value;

    /** Constructor */
    AddMap(const M1& m1, const M2& m2) : _m1(m1), _m2(m2) {}
    
    Value operator[](const Key& k) const { return _m1[k] + _m2[k]; }
  };

  /** Returns an \c AddMap class */

  /**
   * This function just returns an \c AddMap class.
   *
   * For example, if \c m1 and \c m2 are both maps with \c double
   * values, then <tt>addMap(m1,m2)[x]</tt> will be equal to
   * <tt>m1[x]+m2[x]</tt>.
   *
   * \relates AddMap
   */
  template < typename M1, typename M2 >
  inline AddMap< M1, M2 > addMap(const M1& m1, const M2& m2) {
    return AddMap< M1, M2 >(m1, m2);
  }

  /** Difference of two maps */

  /**
   * This @ref concepts::ReadMap "read-only map" returns the difference
   * of the values of the two given maps.
   * Its \c Key and \c Value types are inherited from \c M1.
   * The \c Key and \c Value of \c M2 must be convertible to those of
   * \c M1.
   *
   * If \c m1 is of type \c M1 and \c m2 is of \c M2, then for
   * @code
   * SubMap<M1,M2> sm(m1,m2);
   * @endcode
   * <tt>sm[x]</tt> will be equal to <tt>m1[x]-m2[x]</tt>.
   *
   * The simplest way of using this map is through the subMap()
   * function.
   *
   * \sa AddMap, MulMap, DivMap
   */
  template < typename M1, typename M2 >
  class SubMap: public MapBase< typename M1::Key, typename M1::Value > {
    const M1& _m1;
    const M2& _m2;

    public:
    
    using Key = typename M1::Key;
    
    using Value = typename M1::Value;

    /** Constructor */
    SubMap(const M1& m1, const M2& m2) : _m1(m1), _m2(m2) {}
    
    Value operator[](const Key& k) const { return _m1[k] - _m2[k]; }
  };

  /** Returns a \c SubMap class */

  /**
   * This function just returns a \c SubMap class.
   *
   * For example, if \c m1 and \c m2 are both maps with \c double
   * values, then <tt>subMap(m1,m2)[x]</tt> will be equal to
   * <tt>m1[x]-m2[x]</tt>.
   *
   * \relates SubMap
   */
  template < typename M1, typename M2 >
  inline SubMap< M1, M2 > subMap(const M1& m1, const M2& m2) {
    return SubMap< M1, M2 >(m1, m2);
  }

  /** Product of two maps */

  /**
   * This @ref concepts::ReadMap "read-only map" returns the product
   * of the values of the two given maps.
   * Its \c Key and \c Value types are inherited from \c M1.
   * The \c Key and \c Value of \c M2 must be convertible to those of
   * \c M1.
   *
   * If \c m1 is of type \c M1 and \c m2 is of \c M2, then for
   * @code
   * MulMap<M1,M2> mm(m1,m2);
   * @endcode
   * <tt>mm[x]</tt> will be equal to <tt>m1[x]*m2[x]</tt>.
   *
   * The simplest way of using this map is through the mulMap()
   * function.
   *
   * \sa AddMap, SubMap, DivMap
   * \sa ScaleMap, ScaleWriteMap
   */
  template < typename M1, typename M2 >
  class MulMap: public MapBase< typename M1::Key, typename M1::Value > {
    const M1& _m1;
    const M2& _m2;

    public:
    
    using Key = typename M1::Key;
    
    using Value = typename M1::Value;

    /** Constructor */
    MulMap(const M1& m1, const M2& m2) : _m1(m1), _m2(m2) {}
    
    Value operator[](const Key& k) const { return _m1[k] * _m2[k]; }
  };

  /** Returns a \c MulMap class */

  /**
   * This function just returns a \c MulMap class.
   *
   * For example, if \c m1 and \c m2 are both maps with \c double
   * values, then <tt>mulMap(m1,m2)[x]</tt> will be equal to
   * <tt>m1[x]*m2[x]</tt>.
   *
   * \relates MulMap
   */
  template < typename M1, typename M2 >
  inline MulMap< M1, M2 > mulMap(const M1& m1, const M2& m2) {
    return MulMap< M1, M2 >(m1, m2);
  }

  /** Quotient of two maps */

  /**
   * This @ref concepts::ReadMap "read-only map" returns the quotient
   * of the values of the two given maps.
   * Its \c Key and \c Value types are inherited from \c M1.
   * The \c Key and \c Value of \c M2 must be convertible to those of
   * \c M1.
   *
   * If \c m1 is of type \c M1 and \c m2 is of \c M2, then for
   * @code
   * DivMap<M1,M2> dm(m1,m2);
   * @endcode
   * <tt>dm[x]</tt> will be equal to <tt>m1[x]/m2[x]</tt>.
   *
   * The simplest way of using this map is through the divMap()
   * function.
   *
   * \sa AddMap, SubMap, MulMap
   */
  template < typename M1, typename M2 >
  class DivMap: public MapBase< typename M1::Key, typename M1::Value > {
    const M1& _m1;
    const M2& _m2;

    public:
    
    using Key = typename M1::Key;
    
    using Value = typename M1::Value;

    /** Constructor */
    DivMap(const M1& m1, const M2& m2) : _m1(m1), _m2(m2) {}
    
    Value operator[](const Key& k) const { return _m1[k] / _m2[k]; }
  };

  /** Returns a \c DivMap class */

  /**
   * This function just returns a \c DivMap class.
   *
   * For example, if \c m1 and \c m2 are both maps with \c double
   * values, then <tt>divMap(m1,m2)[x]</tt> will be equal to
   * <tt>m1[x]/m2[x]</tt>.
   *
   * \relates DivMap
   */
  template < typename M1, typename M2 >
  inline DivMap< M1, M2 > divMap(const M1& m1, const M2& m2) {
    return DivMap< M1, M2 >(m1, m2);
  }

  /** Shifts a map with a constant. */

  /**
   * This @ref concepts::ReadMap "read-only map" returns the sum of
   * the given map and a constant value (i.e. it shifts the map with
   * the constant). Its \c Key and \c Value are inherited from \c M.
   *
   * Actually,
   * @code
   * ShiftMap<M> sh(m,v);
   * @endcode
   * is equivalent to
   * @code
   * ConstMap<M::Key, M::Value> cm(v);
   * AddMap<M, ConstMap<M::Key, M::Value> > sh(m,cm);
   * @endcode
   *
   * The simplest way of using this map is through the shiftMap()
   * function.
   *
   * \sa ShiftWriteMap
   */
  template < typename M, typename C = typename M::Value >
  class ShiftMap: public MapBase< typename M::Key, typename M::Value > {
    const M& _m;
    C        _v;

    public:
    
    using Key = typename M::Key;
    
    using Value = typename M::Value;

    /** Constructor */

    /**
     * Constructor.
     * @param m The undelying map.
     * @param v The constant value.
     */
    ShiftMap(const M& m, const C& v) : _m(m), _v(v) {}
    
    Value operator[](const Key& k) const { return _m[k] + _v; }
  };

  /** Shifts a map with a constant (read-write version). */

  /**
   * This @ref concepts::ReadWriteMap "read-write map" returns the sum
   * of the given map and a constant value (i.e. it shifts the map with
   * the constant). Its \c Key and \c Value are inherited from \c M.
   * It makes also possible to write the map.
   *
   * The simplest way of using this map is through the shiftWriteMap()
   * function.
   *
   * \sa ShiftMap
   */
  template < typename M, typename C = typename M::Value >
  class ShiftWriteMap: public MapBase< typename M::Key, typename M::Value > {
    M& _m;
    C  _v;

    public:
    
    using Key = typename M::Key;
    
    using Value = typename M::Value;

    /** Constructor */

    /**
     * Constructor.
     * @param m The undelying map.
     * @param v The constant value.
     */
    ShiftWriteMap(M& m, const C& v) : _m(m), _v(v) {}
    
    Value operator[](const Key& k) const { return _m[k] + _v; }
    
    void set(const Key& k, const Value& v) { _m.set(k, v - _v); }
  };

  /** Returns a \c ShiftMap class */

  /**
   * This function just returns a \c ShiftMap class.
   *
   * For example, if \c m is a map with \c double values and \c v is
   * \c double, then <tt>shiftMap(m,v)[x]</tt> will be equal to
   * <tt>m[x]+v</tt>.
   *
   * \relates ShiftMap
   */
  template < typename M, typename C >
  inline ShiftMap< M, C > shiftMap(const M& m, const C& v) {
    return ShiftMap< M, C >(m, v);
  }

  /** Returns a \c ShiftWriteMap class */

  /**
   * This function just returns a \c ShiftWriteMap class.
   *
   * For example, if \c m is a map with \c double values and \c v is
   * \c double, then <tt>shiftWriteMap(m,v)[x]</tt> will be equal to
   * <tt>m[x]+v</tt>.
   * Moreover it makes also possible to write the map.
   *
   * \relates ShiftWriteMap
   */
  template < typename M, typename C >
  inline ShiftWriteMap< M, C > shiftWriteMap(M& m, const C& v) {
    return ShiftWriteMap< M, C >(m, v);
  }

  /** Scales a map with a constant. */

  /**
   * This @ref concepts::ReadMap "read-only map" returns the value of
   * the given map multiplied from the left side with a constant value.
   * Its \c Key and \c Value are inherited from \c M.
   *
   * Actually,
   * @code
   * ScaleMap<M> sc(m,v);
   * @endcode
   * is equivalent to
   * @code
   * ConstMap<M::Key, M::Value> cm(v);
   * MulMap<ConstMap<M::Key, M::Value>, M> sc(cm,m);
   * @endcode
   *
   * The simplest way of using this map is through the scaleMap()
   * function.
   *
   * \sa ScaleWriteMap
   */
  template < typename M, typename C = typename M::Value >
  class ScaleMap: public MapBase< typename M::Key, typename M::Value > {
    const M& _m;
    C        _v;

    public:
    
    using Key = typename M::Key;
    
    using Value = typename M::Value;

    /** Constructor */

    /**
     * Constructor.
     * @param m The undelying map.
     * @param v The constant value.
     */
    ScaleMap(const M& m, const C& v) : _m(m), _v(v) {}
    
    Value operator[](const Key& k) const { return _v * _m[k]; }
  };

  /** Scales a map with a constant (read-write version). */

  /**
   * This @ref concepts::ReadWriteMap "read-write map" returns the value of
   * the given map multiplied from the left side with a constant value.
   * Its \c Key and \c Value are inherited from \c M.
   * It can also be used as write map if the \c / operator is defined
   * between \c Value and \c C and the given multiplier is not zero.
   *
   * The simplest way of using this map is through the scaleWriteMap()
   * function.
   *
   * \sa ScaleMap
   */
  template < typename M, typename C = typename M::Value >
  class ScaleWriteMap: public MapBase< typename M::Key, typename M::Value > {
    M& _m;
    C  _v;

    public:
    
    using Key = typename M::Key;
    
    using Value = typename M::Value;

    /** Constructor */

    /**
     * Constructor.
     * @param m The undelying map.
     * @param v The constant value.
     */
    ScaleWriteMap(M& m, const C& v) : _m(m), _v(v) {}
    
    Value operator[](const Key& k) const { return _v * _m[k]; }
    
    void set(const Key& k, const Value& v) { _m.set(k, v / _v); }
  };

  /** Returns a \c ScaleMap class */

  /**
   * This function just returns a \c ScaleMap class.
   *
   * For example, if \c m is a map with \c double values and \c v is
   * \c double, then <tt>scaleMap(m,v)[x]</tt> will be equal to
   * <tt>v*m[x]</tt>.
   *
   * \relates ScaleMap
   */
  template < typename M, typename C >
  inline ScaleMap< M, C > scaleMap(const M& m, const C& v) {
    return ScaleMap< M, C >(m, v);
  }

  /** Returns a \c ScaleWriteMap class */

  /**
   * This function just returns a \c ScaleWriteMap class.
   *
   * For example, if \c m is a map with \c double values and \c v is
   * \c double, then <tt>scaleWriteMap(m,v)[x]</tt> will be equal to
   * <tt>v*m[x]</tt>.
   * Moreover it makes also possible to write the map.
   *
   * \relates ScaleWriteMap
   */
  template < typename M, typename C >
  inline ScaleWriteMap< M, C > scaleWriteMap(M& m, const C& v) {
    return ScaleWriteMap< M, C >(m, v);
  }

  /** Negative of a map */

  /**
   * This @ref concepts::ReadMap "read-only map" returns the negative
   * of the values of the given map (using the unary \c - operator).
   * Its \c Key and \c Value are inherited from \c M.
   *
   * If M::Value is \c int, \c double etc., then
   * @code
   * NegMap<M> neg(m);
   * @endcode
   * is equivalent to
   * @code
   * ScaleMap<M> neg(m,-1);
   * @endcode
   *
   * The simplest way of using this map is through the negMap()
   * function.
   *
   * \sa NegWriteMap
   */
  template < typename M >
  class NegMap: public MapBase< typename M::Key, typename M::Value > {
    const M& _m;

    public:
    
    using Key = typename M::Key;
    
    using Value = typename M::Value;

    /** Constructor */
    NegMap(const M& m) : _m(m) {}
    
    Value operator[](const Key& k) const { return -_m[k]; }
  };

  /** Negative of a map (read-write version) */

  /**
   * This @ref concepts::ReadWriteMap "read-write map" returns the
   * negative of the values of the given map (using the unary \c -
   * operator).
   * Its \c Key and \c Value are inherited from \c M.
   * It makes also possible to write the map.
   *
   * If M::Value is \c int, \c double etc., then
   * @code
   * NegWriteMap<M> neg(m);
   * @endcode
   * is equivalent to
   * @code
   * ScaleWriteMap<M> neg(m,-1);
   * @endcode
   *
   * The simplest way of using this map is through the negWriteMap()
   * function.
   *
   * \sa NegMap
   */
  template < typename M >
  class NegWriteMap: public MapBase< typename M::Key, typename M::Value > {
    M& _m;

    public:
    
    using Key = typename M::Key;
    
    using Value = typename M::Value;

    /** Constructor */
    NegWriteMap(M& m) : _m(m) {}
    
    Value operator[](const Key& k) const { return -_m[k]; }
    
    void set(const Key& k, const Value& v) { _m.set(k, -v); }
  };

  /** Returns a \c NegMap class */

  /**
   * This function just returns a \c NegMap class.
   *
   * For example, if \c m is a map with \c double values, then
   * <tt>negMap(m)[x]</tt> will be equal to <tt>-m[x]</tt>.
   *
   * \relates NegMap
   */
  template < typename M >
  inline NegMap< M > negMap(const M& m) {
    return NegMap< M >(m);
  }

  /** Returns a \c NegWriteMap class */

  /**
   * This function just returns a \c NegWriteMap class.
   *
   * For example, if \c m is a map with \c double values, then
   * <tt>negWriteMap(m)[x]</tt> will be equal to <tt>-m[x]</tt>.
   * Moreover it makes also possible to write the map.
   *
   * \relates NegWriteMap
   */
  template < typename M >
  inline NegWriteMap< M > negWriteMap(M& m) {
    return NegWriteMap< M >(m);
  }

  /** Absolute value of a map */

  /**
   * This @ref concepts::ReadMap "read-only map" returns the absolute
   * value of the values of the given map.
   * Its \c Key and \c Value are inherited from \c M.
   * \c Value must be comparable to \c 0 and the unary \c -
   * operator must be defined for it, of course.
   *
   * The simplest way of using this map is through the absMap()
   * function.
   */
  template < typename M >
  class AbsMap: public MapBase< typename M::Key, typename M::Value > {
    const M& _m;

    public:
    
    using Key = typename M::Key;
    
    using Value = typename M::Value;

    /** Constructor */
    AbsMap(const M& m) : _m(m) {}
    
    Value operator[](const Key& k) const {
      Value tmp = _m[k];
      return tmp >= 0 ? tmp : -tmp;
    }
  };

  /** Returns an \c AbsMap class */

  /**
   * This function just returns an \c AbsMap class.
   *
   * For example, if \c m is a map with \c double values, then
   * <tt>absMap(m)[x]</tt> will be equal to <tt>m[x]</tt> if
   * it is positive or zero and <tt>-m[x]</tt> if <tt>m[x]</tt> is
   * negative.
   *
   * \relates AbsMap
   */
  template < typename M >
  inline AbsMap< M > absMap(const M& m) {
    return AbsMap< M >(m);
  }

  /** @} */

  // Logical maps and map adaptors:

  /**
   * \addtogroup maps
   * @{
   */

  /** Constant \c true map. */

  /**
   * This @ref concepts::ReadMap "read-only map" assigns \c true to
   * each key.
   *
   * Note that
   * @code
   * TrueMap<K> tm;
   * @endcode
   * is equivalent to
   * @code
   * ConstMap<K,bool> tm(true);
   * @endcode
   *
   * \sa FalseMap
   * \sa ConstMap
   */
  template < typename K >
  class TrueMap: public MapBase< K, bool > {
    public:
    
    using Key = K;
    
    using Value = bool;

    /** Gives back \c true. */
    Value operator[](const Key&) const { return true; }
  };

  /** Returns a \c TrueMap class */

  /**
   * This function just returns a \c TrueMap class.
   * \relates TrueMap
   */
  template < typename K >
  inline TrueMap< K > trueMap() {
    return TrueMap< K >();
  }

  /** Constant \c false map. */

  /**
   * This @ref concepts::ReadMap "read-only map" assigns \c false to
   * each key.
   *
   * Note that
   * @code
   * FalseMap<K> fm;
   * @endcode
   * is equivalent to
   * @code
   * ConstMap<K,bool> fm(false);
   * @endcode
   *
   * \sa TrueMap
   * \sa ConstMap
   */
  template < typename K >
  class FalseMap: public MapBase< K, bool > {
    public:
    
    using Key = K;
    
    using Value = bool;

    /** Gives back \c false. */
    Value operator[](const Key&) const { return false; }
  };

  /** Returns a \c FalseMap class */

  /**
   * This function just returns a \c FalseMap class.
   * \relates FalseMap
   */
  template < typename K >
  inline FalseMap< K > falseMap() {
    return FalseMap< K >();
  }

  /** @} */

  /**
   * \addtogroup map_adaptors
   * @{
   */

  /** Logical 'and' of two maps */

  /**
   * This @ref concepts::ReadMap "read-only map" returns the logical
   * 'and' of the values of the two given maps.
   * Its \c Key type is inherited from \c M1 and its \c Value type is
   * \c bool. \c M2::Key must be convertible to \c M1::Key.
   *
   * If \c m1 is of type \c M1 and \c m2 is of \c M2, then for
   * @code
   * AndMap<M1,M2> am(m1,m2);
   * @endcode
   * <tt>am[x]</tt> will be equal to <tt>m1[x]&&m2[x]</tt>.
   *
   * The simplest way of using this map is through the andMap()
   * function.
   *
   * \sa OrMap
   * \sa NotMap, NotWriteMap
   */
  template < typename M1, typename M2 >
  class AndMap: public MapBase< typename M1::Key, bool > {
    const M1& _m1;
    const M2& _m2;

    public:
    
    using Key = typename M1::Key;
    
    using Value = bool;

    /** Constructor */
    AndMap(const M1& m1, const M2& m2) : _m1(m1), _m2(m2) {}
    
    Value operator[](const Key& k) const { return _m1[k] && _m2[k]; }
  };

  /** Returns an \c AndMap class */

  /**
   * This function just returns an \c AndMap class.
   *
   * For example, if \c m1 and \c m2 are both maps with \c bool values,
   * then <tt>andMap(m1,m2)[x]</tt> will be equal to
   * <tt>m1[x]&&m2[x]</tt>.
   *
   * \relates AndMap
   */
  template < typename M1, typename M2 >
  inline AndMap< M1, M2 > andMap(const M1& m1, const M2& m2) {
    return AndMap< M1, M2 >(m1, m2);
  }

  /** Logical 'or' of two maps */

  /**
   * This @ref concepts::ReadMap "read-only map" returns the logical
   * 'or' of the values of the two given maps.
   * Its \c Key type is inherited from \c M1 and its \c Value type is
   * \c bool. \c M2::Key must be convertible to \c M1::Key.
   *
   * If \c m1 is of type \c M1 and \c m2 is of \c M2, then for
   * @code
   * OrMap<M1,M2> om(m1,m2);
   * @endcode
   * <tt>om[x]</tt> will be equal to <tt>m1[x]||m2[x]</tt>.
   *
   * The simplest way of using this map is through the orMap()
   * function.
   *
   * \sa AndMap
   * \sa NotMap, NotWriteMap
   */
  template < typename M1, typename M2 >
  class OrMap: public MapBase< typename M1::Key, bool > {
    const M1& _m1;
    const M2& _m2;

    public:
    
    using Key = typename M1::Key;
    
    using Value = bool;

    /** Constructor */
    OrMap(const M1& m1, const M2& m2) : _m1(m1), _m2(m2) {}
    
    Value operator[](const Key& k) const { return _m1[k] || _m2[k]; }
  };

  /** Returns an \c OrMap class */

  /**
   * This function just returns an \c OrMap class.
   *
   * For example, if \c m1 and \c m2 are both maps with \c bool values,
   * then <tt>orMap(m1,m2)[x]</tt> will be equal to
   * <tt>m1[x]||m2[x]</tt>.
   *
   * \relates OrMap
   */
  template < typename M1, typename M2 >
  inline OrMap< M1, M2 > orMap(const M1& m1, const M2& m2) {
    return OrMap< M1, M2 >(m1, m2);
  }

  /** Logical 'not' of a map */

  /**
   * This @ref concepts::ReadMap "read-only map" returns the logical
   * negation of the values of the given map.
   * Its \c Key is inherited from \c M and its \c Value is \c bool.
   *
   * The simplest way of using this map is through the notMap()
   * function.
   *
   * \sa NotWriteMap
   */
  template < typename M >
  class NotMap: public MapBase< typename M::Key, bool > {
    const M& _m;

    public:
    
    using Key = typename M::Key;
    
    using Value = bool;

    /** Constructor */
    NotMap(const M& m) : _m(m) {}
    
    Value operator[](const Key& k) const { return !_m[k]; }
  };

  /** Logical 'not' of a map (read-write version) */

  /**
   * This @ref concepts::ReadWriteMap "read-write map" returns the
   * logical negation of the values of the given map.
   * Its \c Key is inherited from \c M and its \c Value is \c bool.
   * It makes also possible to write the map. When a value is set,
   * the opposite value is set to the original map.
   *
   * The simplest way of using this map is through the notWriteMap()
   * function.
   *
   * \sa NotMap
   */
  template < typename M >
  class NotWriteMap: public MapBase< typename M::Key, bool > {
    M& _m;

    public:
    
    using Key = typename M::Key;
    
    using Value = bool;

    /** Constructor */
    NotWriteMap(M& m) : _m(m) {}
    
    Value operator[](const Key& k) const { return !_m[k]; }
    
    void set(const Key& k, bool v) { _m.set(k, !v); }
  };

  /** Returns a \c NotMap class */

  /**
   * This function just returns a \c NotMap class.
   *
   * For example, if \c m is a map with \c bool values, then
   * <tt>notMap(m)[x]</tt> will be equal to <tt>!m[x]</tt>.
   *
   * \relates NotMap
   */
  template < typename M >
  inline NotMap< M > notMap(const M& m) {
    return NotMap< M >(m);
  }

  /** Returns a \c NotWriteMap class */

  /**
   * This function just returns a \c NotWriteMap class.
   *
   * For example, if \c m is a map with \c bool values, then
   * <tt>notWriteMap(m)[x]</tt> will be equal to <tt>!m[x]</tt>.
   * Moreover it makes also possible to write the map.
   *
   * \relates NotWriteMap
   */
  template < typename M >
  inline NotWriteMap< M > notWriteMap(M& m) {
    return NotWriteMap< M >(m);
  }

  /** Combination of two maps using the \c == operator */

  /**
   * This @ref concepts::ReadMap "read-only map" assigns \c true to
   * the keys for which the corresponding values of the two maps are
   * equal.
   * Its \c Key type is inherited from \c M1 and its \c Value type is
   * \c bool. \c M2::Key must be convertible to \c M1::Key.
   *
   * If \c m1 is of type \c M1 and \c m2 is of \c M2, then for
   * @code
   * EqualMap<M1,M2> em(m1,m2);
   * @endcode
   * <tt>em[x]</tt> will be equal to <tt>m1[x]==m2[x]</tt>.
   *
   * The simplest way of using this map is through the equalMap()
   * function.
   *
   * \sa LessMap
   */
  template < typename M1, typename M2 >
  class EqualMap: public MapBase< typename M1::Key, bool > {
    const M1& _m1;
    const M2& _m2;

    public:
    
    using Key = typename M1::Key;
    
    using Value = bool;

    /** Constructor */
    EqualMap(const M1& m1, const M2& m2) : _m1(m1), _m2(m2) {}
    
    Value operator[](const Key& k) const { return _m1[k] == _m2[k]; }
  };

  /** Returns an \c EqualMap class */

  /**
   * This function just returns an \c EqualMap class.
   *
   * For example, if \c m1 and \c m2 are maps with keys and values of
   * the same type, then <tt>equalMap(m1,m2)[x]</tt> will be equal to
   * <tt>m1[x]==m2[x]</tt>.
   *
   * \relates EqualMap
   */
  template < typename M1, typename M2 >
  inline EqualMap< M1, M2 > equalMap(const M1& m1, const M2& m2) {
    return EqualMap< M1, M2 >(m1, m2);
  }

  /** Combination of two maps using the \c < operator */

  /**
   * This @ref concepts::ReadMap "read-only map" assigns \c true to
   * the keys for which the corresponding value of the first map is
   * less then the value of the second map.
   * Its \c Key type is inherited from \c M1 and its \c Value type is
   * \c bool. \c M2::Key must be convertible to \c M1::Key.
   *
   * If \c m1 is of type \c M1 and \c m2 is of \c M2, then for
   * @code
   * LessMap<M1,M2> lm(m1,m2);
   * @endcode
   * <tt>lm[x]</tt> will be equal to <tt>m1[x]<m2[x]</tt>.
   *
   * The simplest way of using this map is through the lessMap()
   * function.
   *
   * \sa EqualMap
   */
  template < typename M1, typename M2 >
  class LessMap: public MapBase< typename M1::Key, bool > {
    const M1& _m1;
    const M2& _m2;

    public:
    
    using Key = typename M1::Key;
    
    using Value = bool;

    /** Constructor */
    LessMap(const M1& m1, const M2& m2) : _m1(m1), _m2(m2) {}
    
    Value operator[](const Key& k) const { return _m1[k] < _m2[k]; }
  };

  /** Returns an \c LessMap class */

  /**
   * This function just returns an \c LessMap class.
   *
   * For example, if \c m1 and \c m2 are maps with keys and values of
   * the same type, then <tt>lessMap(m1,m2)[x]</tt> will be equal to
   * <tt>m1[x]<m2[x]</tt>.
   *
   * \relates LessMap
   */
  template < typename M1, typename M2 >
  inline LessMap< M1, M2 > lessMap(const M1& m1, const M2& m2) {
    return LessMap< M1, M2 >(m1, m2);
  }

  namespace _maps_bits {

    template < typename _Iterator, typename Enable = void >
    struct IteratorTraits {
      using Value = typename std::iterator_traits< _Iterator >::value_type;
    };

    template < typename _Iterator >
    struct IteratorTraits< _Iterator, typename exists< typename _Iterator::container_type >::type > {
      using Value = typename _Iterator::container_type::value_type;
    };

  }   // namespace _maps_bits

/** @} */

/**
 * \addtogroup maps
 * @{
 */

/**
 * @brief Writable bool map for logging each \c true assigned element
 *
 * A @ref concepts::WriteMap "writable" bool map for logging
 * each \c true assigned element, i.e it copies subsequently each
 * keys set to \c true to the given iterator.
 * The most important usage of it is storing certain nodes or arcs
 * that were marked \c true by an algorithm.
 *
 * There are several algorithms that provide solutions through bool
 * maps and most of them assign \c true at most once for each key.
 * In these cases it is a natural request to store each \c true
 * assigned elements (in order of the assignment), which can be
 * easily done with LoggerBoolMap.
 *
 * The simplest way of using this map is through the loggerBoolMap()
 * function.
 *
 * @tparam IT The type of the iterator.
 * @tparam KEY The key type of the map. The default value set
 * according to the iterator type should work in most cases.
 *
 * @note The container of the iterator must contain enough space
 * for the elements or the iterator should be an inserter iterator.
 */
#ifdef DOXYGEN
  template < typename IT, typename KEY >
#else
  template < typename IT, typename KEY = typename _maps_bits::IteratorTraits< IT >::Value >
#endif
  class LoggerBoolMap: public MapBase< KEY, bool > {
    public:
    
    using Key = KEY;
    
    using Value = bool;
    
    using Iterator = IT;

    /** Constructor */
    LoggerBoolMap(Iterator it) : _begin(it), _end(it) {}

    /** Gives back the given iterator set for the first key */
    Iterator begin() const { return _begin; }

    /** Gives back the the 'after the last' iterator */
    Iterator end() const { return _end; }

    /** The set function of the map */
    void set(const Key& key, Value value) {
      if (value) { *_end++ = key; }
    }

    private:
    Iterator _begin;
    Iterator _end;
  };

  /** Returns a \c LoggerBoolMap class */

  /**
   * This function just returns a \c LoggerBoolMap class.
   *
   * The most important usage of it is storing certain nodes or arcs
   * that were marked \c true by an algorithm.
   * For example, it makes easier to store the nodes in the processing
   * order of Dfs algorithm, as the following examples show.
   * @code
   * nt::DynamicArray<Node> v;
   * dfs(g).processedMap(loggerBoolMap(std::back_inserter(v))).run(s);
   * @endcode
   * @code
   * nt::DynamicArray<Node> v(lemon::countNodes(g));
   * dfs(g).processedMap(loggerBoolMap(v.begin())).run(s);
   * @endcode
   *
   * @note The container of the iterator must contain enough space
   * for the elements or the iterator should be an inserter iterator.
   *
   * @note LoggerBoolMap is just @ref concepts::WriteMap "writable", so
   * it cannot be used when a readable map is needed, for example, as
   * \c ReachedMap for \c Bfs, \c Dfs and \c Dijkstra algorithms.
   *
   * \relates LoggerBoolMap
   */
  template < typename Iterator >
  inline LoggerBoolMap< Iterator > loggerBoolMap(Iterator it) {
    return LoggerBoolMap< Iterator >(it);
  }

  /** @} */

}   // namespace nt

#endif
