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
 * @brief A class for type-safe storage and retrieval of any data type.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_ANY_H_
#define _NT_ANY_H_

#include "typeid.h"
#include <type_traits>   // std::enable_if

namespace nt {

  /**
   * @class Any
   * @brief A class for type-safe storage and retrieval of any data type.
   *
   * The Any class allows you to store and retrieve values of any data type in a type-safe manner.
   * It uses an internal storage class ValueStorage to store the actual value and provides methods
   * to access and manipulate the stored value.
   *
   * @code
   *  // Create an Any object and store an integer in it.
   *  int x = 42;
   *  Any any_x(x);
   *
   *  // Retrieve and modify the stored integer.
   *  if (int* p = any_x.get<int>())
   *    *p = 99;
   *
   *  // Retrieve and print the modified integer.
   *  if (const int* p = any_x.get<int>()) {
   *    std::cout << "The stored integer is: " << *p << std::endl;
   *  } else {
   *    std::cout << "Failed to retrieve the stored integer." << std::endl;
   *  }
   * @endcode
   *
   */
  struct Any {
    /**
     * @brief Base class for storing values of different types.
     */
    struct ValueStorageBase {
      constexpr virtual ~ValueStorageBase() {}
      constexpr virtual Type type() const noexcept = 0;
    };

    /**
     * @brief Template class for storing values of a specific type.
     * @tparam V The type of the value to be stored.
     */
    template < typename V >
    struct ValueStorage: public ValueStorageBase {
      using Value = V;
      Value _value;

      /**
       * @brief Constructor that initializes the stored value.
       * @param value The value to be stored.
       */
      ValueStorage(Value& value) : _value(value) {}

      /**
       * @brief Move constructor that initializes the stored value.
       * @param value The value to be stored.
       */
      ValueStorage(Value&& value) : _value(std::forward< Value >(value)) {}

      /**
       * @brief Get the type of the stored value.
       * @return The type of the stored value.
       */
      constexpr Type type() const noexcept override { return TypeId< Value >::value; }
    };

    ValueStorageBase* _p_content;

    /**
     * @brief Default constructor to create an empty Any object.
     */
    constexpr Any() noexcept : _p_content(nullptr) {}

    /**
     * @brief Constructor to store a value of a specific type in the Any object.
     * @tparam Value The type of the value to be stored.
     * @param value The value to be stored.
     */
    template < typename Value >
    Any(Value& value) : _p_content(new ValueStorage< Value >(value)) {}

    Any(const Any& other) = delete;

    template < typename Value >
    Any(Value&& value,
        // enable if value has not type `any&`
        typename std::enable_if< !std::is_same_v< Any&, Value > >::type* = 0,
        // enable if value has not type `const Value&&`
        typename std::enable_if< !std::is_const_v< Value > >::type* = 0) :
        _p_content(new ValueStorage< typename std::decay< Value >::type >(static_cast< Value&& >(value))) {}
    Any(Any&& other) noexcept : _p_content(other._p_content) { other._p_content = nullptr; }

    template < class Value >
    Any& operator=(Value&& rhs) {
      Any(static_cast< Value&& >(rhs)).swap(*this);
      return *this;
    }
    ~Any() noexcept { delete _p_content; }

    /**
     * @brief Get the type of the stored value.
     * @return The type of the stored value.
     */
    constexpr Type type() const noexcept { return _p_content->type(); }

    /**
     * @brief Swap the contents of two Any objects.
     * @param rhs The other Any object to swap with.
     * @return A reference to the modified Any object.
     */
    Any& swap(Any& rhs) noexcept {
      ValueStorageBase* tmp = _p_content;
      _p_content = rhs._p_content;
      rhs._p_content = tmp;
      return *this;
    }

    /**
     * @brief Get a pointer to the stored value of a specific type.
     * @tparam Value The type of the value to retrieve.
     * @return A pointer to the stored value if compatible with requested type, otherwise nullptr.
     */
    template < typename Value >
    Value* get() noexcept {
      if (_p_content && is_compatible< Value >(type()))
        return std::addressof(static_cast< ValueStorage< Value >* >(_p_content)->_value);
      return nullptr;
    }
    template < typename Value >
    const Value* get() const noexcept {
      return get< Value >();
    }

    /**
     * @brief Get a reference to the stored value of a specific type or the passed value if.
     * @tparam Value The type of the value to retrieve.
     * @return A reference to the stored value if compatible with requested type, otherwise return
     * v.
     */
    template < typename Value >
    Value& get(const Value& v) noexcept {
      if (Value* p_val = get< Value >()) return *p_val;
      return v;
    }
    template < typename Value >
    const Value& get(const Value& v) const noexcept {
      return get< Value >(v);
    }

    /**
     * @brief Set the stored value to a new value of a specific type.
     * @tparam Value The type of the new value.
     * @param v The new value to be set.
     * @return True if the operation was successful, false otherwise.
     */
    template < typename Value >
    bool set(const Value& v) noexcept {
      if (_p_content && is_compatible< Value >(type())) {
        static_cast< ValueStorage< Value >* >(_p_content)->_value = v;
        return true;
      }
      return false;
    }
  };

  /**
   * @class AnyView
   * @brief A class for manipulating values of any data type without ownership.
   *
   * Contrary to Any class, AnyView allows you to manipulate values of any data type
   * without owning or copying the data. It provides methods to safely access and modify the
   * stored values.
   *
   */
  struct AnyView {
    void*      _p_content;   //< Pointer to the stored value.
    const Type _t;           //< Type information of the stored value.

    /**
     * @brief Default constructor to create an empty AnyView object.
     */
    AnyView() : _p_content(nullptr), _t(Type::UNKNOWN){};

    /**
     * @brief Constructor to create a AnyView with the provided value.
     * @tparam Value The type of the value.
     * @param value A reference to the value.
     */
    template < typename Value >
    AnyView(Value& value) : _p_content(static_cast< void* >(std::addressof(value))), _t(TypeId< Value >::value) {}

    /**
     * @brief Get the type of the stored value.
     * @return The type of the stored value.
     */
    constexpr Type type() const noexcept { return _t; }

    /**
     * @brief Get a pointer to the stored value.
     * @tparam Value The type of the value to retrieve.
     * @return A pointer to the stored value if compatible, otherwise nullptr.
     */
    template < typename Value >
    const Value* get() const noexcept {
      if (is_compatible< Value >(type())) return static_cast< const Value* >(_p_content);
      return nullptr;
    }

    /**
     * @brief Get the stored value or a default value if not compatible.
     * @tparam Value The type of the value to retrieve.
     * @param v The default value to return if the stored value is not compatible.
     * @return The stored value if compatible, otherwise the default value.
     */
    template < typename Value >
    const Value& get(const Value& v) const noexcept {
      if (const Value* p_val = get< Value >()) return *p_val;
      return v;
    }

    /**
     * @brief Set the stored value to a new value.
     * @tparam Value The type of the new value.
     * @param v The new value to be set.
     * @return True if the operation was successful, false otherwise.
     */
    template < typename Value >
    bool set(Value&& v) noexcept {
      if (is_compatible< Value >(type())) {
        *(static_cast< Value* >(_p_content)) = std::forward< Value >(v);
        return true;
      }
      return false;
    }
  };

}   // namespace nt

#endif
