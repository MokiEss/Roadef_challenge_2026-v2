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
 * @brief typeid.h
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_TYPEID_H_
#define _NT_TYPEID_H_

#include <cstdint>
#include "string.h"
#include "algebra.h"

#ifndef COMMA
#  define COMMA ,
#endif

#define TYPEID_DECLARE_STRUCT(_TYPE, _TYPE_ENUM) \
  template <>                                    \
  struct TypeId< _TYPE > {                       \
    static constexpr Type value = _TYPE_ENUM;    \
  };

namespace nt {

  enum class Type {
    INT32 = 0,
    INT64,
    UINT32,
    UINT64,
    BOOL,
    FLOAT,
    DOUBLE,
    STRING,   // DO NOT ADD NEW TYPE BEFORE STRING (see is_arithmetic())
    ARRAY_INT32,
    ARRAY_INT64,
    ARRAY_UINT32,
    ARRAY_UINT64,
    ARRAY_FLOAT,
    ARRAY_DOUBLE,
    ARRAY_BOOL,
    ARRAY_STRING,
    VECTOR1_FLOAT,
    VECTOR2_FLOAT,
    VECTOR24_FLOAT,
    VECTOR1_DOUBLE,
    VECTOR2_DOUBLE,
    VECTOR24_DOUBLE,
    UNKNOWN
  };

  static constexpr const char* TypeName[] = {
     "INT32",          "INT64",          "UINT32",         "UINT64",          "BOOL",          "FLOAT",
     "DOUBLE",         "STRING",         "ARRAY_INT32",    "ARRAY_INT64",     "ARRAY_UINT32",  "ARRAY_UINT64",
     "ARRAY_FLOAT",    "ARRAY_DOUBLE",   "ARRAY_BOOL",     "ARRAY_STRING",    "VECTOR1_FLOAT", "VECTOR2_FLOAT",
     "VECTOR24_FLOAT", "VECTOR1_DOUBLE", "VECTOR2_DOUBLE", "VECTOR24_DOUBLE", "UNKNOWN"};

  template < typename T >
  struct TypeId {
    static constexpr Type value = Type::UNKNOWN;
  };

  // Primitive types
  TYPEID_DECLARE_STRUCT(std::int32_t, Type::INT32);
  TYPEID_DECLARE_STRUCT(std::int64_t, Type::INT64);
  TYPEID_DECLARE_STRUCT(std::uint32_t, Type::UINT32);
  TYPEID_DECLARE_STRUCT(std::uint64_t, Type::UINT64);
  TYPEID_DECLARE_STRUCT(float, Type::FLOAT);
  TYPEID_DECLARE_STRUCT(double, Type::DOUBLE);
  TYPEID_DECLARE_STRUCT(bool, Type::BOOL);
  TYPEID_DECLARE_STRUCT(nt::String, Type::STRING);

  // Arrays
  TYPEID_DECLARE_STRUCT(nt::DynamicArray< nt::String >, Type::ARRAY_STRING);
  TYPEID_DECLARE_STRUCT(nt::DoubleDynamicArray, Type::ARRAY_DOUBLE);
  TYPEID_DECLARE_STRUCT(nt::FloatDynamicArray, Type::ARRAY_FLOAT);
  TYPEID_DECLARE_STRUCT(nt::Vec< double COMMA 1 >, Type::VECTOR1_DOUBLE);
  TYPEID_DECLARE_STRUCT(nt::Vec< double COMMA 2 >, Type::VECTOR2_DOUBLE);
  TYPEID_DECLARE_STRUCT(nt::Vec< float COMMA 1 >, Type::VECTOR1_FLOAT);
  TYPEID_DECLARE_STRUCT(nt::Vec< float COMMA 2 >, Type::VECTOR2_FLOAT);
  TYPEID_DECLARE_STRUCT(nt::Vec< double COMMA 24 >, Type::VECTOR24_DOUBLE);
  TYPEID_DECLARE_STRUCT(nt::Vec< float COMMA 24 >, Type::VECTOR24_FLOAT);

  inline constexpr bool is_arithmetic(const Type t) noexcept { return t < Type::STRING; }
  inline constexpr bool is_integer(const Type t) noexcept { return t < Type::FLOAT; }
  inline constexpr bool is_real(const Type t) noexcept { return is_arithmetic(t) && !is_integer(t); }
  inline constexpr bool is_bool(const Type t) noexcept { return t == Type::BOOL; }
  inline constexpr bool is_string(const Type t) noexcept { return t == Type::STRING; }
  inline constexpr bool is_array(const Type t) noexcept { return t < Type::VECTOR1_FLOAT && t > Type::STRING; }
  inline constexpr bool is_vector(const Type t) noexcept { return t < Type::UNKNOWN && t > Type::ARRAY_STRING; }

  template < typename T >
  inline constexpr bool is_compatible(const Type t) noexcept {
    return t == TypeId< T >::value;
  }

  inline const char* typeName(Type t) { return TypeName[static_cast< int >(t)]; }

}   // namespace nt

#endif
