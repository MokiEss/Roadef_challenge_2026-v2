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
 * @brief This file defines wrapper classes/functions to the RapidJSON library. The goal is to avoid
 *        networktools to be dependent of the RapidJSON's API. Ideally, any call to a JSON class/function
 *        should be done through the present wrappers.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_JSON_H_
#define _NT_JSON_H_

#include "../@deps/rapidjson/document.h"
#include "../@deps/rapidjson/writer.h"
#include "../@deps/rapidjson/schema.h"
#include "../@deps/rapidjson/istreamwrapper.h"
#include "../@deps/rapidjson/ostreamwrapper.h"

#include "string.h"
#include "maps.h"

#ifdef GetObject
#  pragma push_macro("GetObject")
#  undef GetObject
#endif

namespace nt {

  using JsonConstObject = rapidjson::Value::ConstObject;
  using JsonObject = rapidjson::Value::Object;
  using JsonConstArray = rapidjson::Value::ConstArray;
  using JsonArray = rapidjson::Value::Array;
  using JsonValue = rapidjson::Value;
  using JsonSizeType = rapidjson::SizeType;
  using JSONSchemaDocument = rapidjson::SchemaDocument;
  using JSONSchemaValidator = rapidjson::SchemaValidator;

  enum class JSONValueType {
    NULL_TYPE = rapidjson::kNullType,
    FALSE_TYPE = rapidjson::kFalseType,
    TRUE_TYPE = rapidjson::kTrueType,
    OBJECT_TYPE = rapidjson::kObjectType,
    ARRAY_TYPE = rapidjson::kArrayType,
    STRING_TYPE = rapidjson::kStringType,
    NUMBER_TYPE = rapidjson::kNumberType
  };

  struct JSONValue: rapidjson::Value {
    explicit JSONValue(JSONValueType type) noexcept :
        rapidjson::Value(static_cast< rapidjson::Type >(static_cast< std::underlying_type_t< JSONValueType > >(type))) {
    }
    explicit JSONValue(int i) noexcept : rapidjson::Value(i) {}
    explicit JSONValue(bool i) noexcept : rapidjson::Value(i) {}
    explicit JSONValue(double i) noexcept : rapidjson::Value(i) {}
    explicit JSONValue(float i) noexcept : rapidjson::Value(i) {}
    explicit JSONValue(const Ch* s, rapidjson::SizeType length) noexcept : rapidjson::Value(s, length) {}
    explicit JSONValue(const Ch* s, AllocatorType& allocator) noexcept : rapidjson::Value(s, allocator) {}
  };

  /**
   * @brief This class acts as a wrapper for different implementations of third-party JSON
   * libraries.
   *
   * Default one is rapidjson.
   *
   */
  struct JSONDocument: rapidjson::Document {
    nt::ByteBuffer buffer;

    bool ParseFile(const char* sz_filename) {
      using RapidJsonInputStream = rapidjson::BasicIStreamWrapper< nt::ByteBuffer >;
      buffer.reset();
      if (!buffer.load(sz_filename)) {  LOG_F(ERROR, "Cannot load file '{}'", sz_filename); return false; }
      buffer.seekg(0);

      RapidJsonInputStream wrapped_buffer(buffer);
      ParseStream< rapidjson::kParseStopWhenDoneFlag >(wrapped_buffer);
      return !HasParseError();
    }

    static nt::ByteBuffer toByteBuffer(const JSONDocument& doc, int i_max_decimal_places = -1) {
      using OutputStream = rapidjson::BasicOStreamWrapper< nt::ByteBuffer >;
      using Writer = rapidjson::Writer< OutputStream,
                                        rapidjson::UTF8<>,
                                        rapidjson::UTF8<>,
                                        rapidjson::CrtAllocator,
                                        rapidjson::kWriteNanAndInfFlag >;

      nt::ByteBuffer buffer;
      OutputStream   wrapped_buffer(buffer);
      Writer         writer(wrapped_buffer);

      writer.SetMaxDecimalPlaces(i_max_decimal_places >= 0 ? i_max_decimal_places : Writer::kDefaultMaxDecimalPlaces);
      doc.Accept(writer);
      return buffer;
    }

    static nt::String toString(const JSONDocument& doc, int i_max_decimal_places = -1) {
      nt::ByteBuffer buffer(toByteBuffer(doc, i_max_decimal_places));
      return buffer.data();
    }
  };

  namespace details {
    template < typename K >
    inline bool setJsonValueToMap(MapStorage< K >& map_storage, const K& key, const rapidjson::Value& value) noexcept {
      switch (map_storage.value()) {
        case Type::INT64: {
          return map_storage.setInt64(key, value.GetInt64());
        } break;
        case Type::INT32: {
          return map_storage.setInt(key, value.GetInt());
        } break;
        case Type::UINT32: {
          return map_storage.setUint(key, value.GetUint());
        } break;
        case Type::UINT64: {
          return map_storage.setUint64(key, value.GetUint64());
        } break;
        case Type::FLOAT: {
          return map_storage.setFloat(key, value.GetFloat());
        } break;
        case Type::DOUBLE: {
          return map_storage.setDouble(key, value.GetDouble());
        } break;
        case Type::STRING: {
          return map_storage.setString(key, nt::String(value.GetString()));
        } break;
        case Type::BOOL: {
          return map_storage.setBool(key, value.GetBool());
        } break;
        case Type::ARRAY_DOUBLE: {
          const auto&        json_array = value.GetArray();
          DoubleDynamicArray array;
          array.reserve(json_array.Size());
          for (const auto& v: json_array)
            array.push_back(v.GetDouble());
          return map_storage.set(key, std::move(array));
        } break;
        case Type::ARRAY_FLOAT: {
          const auto&       json_array = value.GetArray();
          FloatDynamicArray array;
          array.reserve(json_array.Size());
          for (const auto& v: json_array)
            array.push_back(v.GetFloat());
          return map_storage.set(key, std::move(array));
        } break;
        case Type::VECTOR1_DOUBLE: {
          const auto&          json_array = value.GetArray();
          nt::Vec< double, 1 > vec;
          int                  k = 0;
          for (const auto& v: json_array) {
            vec[k++] = v.GetDouble();
            if (k >= 1) break;
          }
          return map_storage.set(key, std::move(vec));
        } break;
        case Type::VECTOR24_DOUBLE: {
          const auto&           json_array = value.GetArray();
          nt::Vec< double, 24 > vec;
          int                   k = 0;
          for (const auto& v: json_array) {
            vec[k++] = v.GetDouble();
            if (k >= 24) break;
          }
          return map_storage.set(key, std::move(vec));
        } break;
        case Type::VECTOR1_FLOAT: {
          const auto&         json_array = value.GetArray();
          nt::Vec< float, 1 > vec;
          int                 k = 0;
          for (const auto& v: json_array) {
            vec[k++] = v.GetFloat();
            if (k >= 1) break;
          }
          return map_storage.set(key, std::move(vec));
        } break;
        case Type::VECTOR24_FLOAT: {
          const auto&          json_array = value.GetArray();
          nt::Vec< float, 24 > vec;
          int                  k = 0;
          for (const auto& v: json_array) {
            vec[k++] = v.GetFloat();
            if (k >= 24) break;
          }
          return map_storage.set(key, std::move(vec));
        } break;
        case Type::ARRAY_STRING: {
          const auto&            json_array = value.GetArray();
          DynamicArray< String > array;
          array.reserve(json_array.Size());
          for (const auto& v: json_array)
            array.push_back(nt::String(v.GetString()));
          return map_storage.set(key, std::move(array));
        } break;
        case Type::ARRAY_INT32: {
          const auto&     json_array = value.GetArray();
          IntDynamicArray array;
          array.reserve(json_array.Size());
          for (const auto& v: json_array)
            array.push_back(v.GetInt());
          return map_storage.set(key, std::move(array));
        } break;
        case Type::ARRAY_INT64: {
          const auto&       json_array = value.GetArray();
          Int64DynamicArray array;
          array.reserve(json_array.Size());
          for (const auto& v: json_array)
            array.push_back(v.GetInt64());
          return map_storage.set(key, std::move(array));
        } break;
        case Type::ARRAY_UINT32: {
          const auto&      json_array = value.GetArray();
          UintDynamicArray array;
          array.reserve(json_array.Size());
          for (const auto& v: json_array)
            array.push_back(v.GetUint());
          return map_storage.set(key, std::move(array));
        } break;
        case Type::ARRAY_UINT64: {
          const auto&        json_array = value.GetArray();
          Uint64DynamicArray array;
          array.reserve(json_array.Size());
          for (const auto& v: json_array)
            array.push_back(v.GetUint64());
          return map_storage.set(key, std::move(array));
        } break;
        case Type::ARRAY_BOOL: {
          const auto&                 json_array = value.GetArray();
          TrivialDynamicArray< bool > array;
          array.reserve(json_array.Size());
          for (const auto& v: json_array)
            array.push_back(v.GetBool());
          return map_storage.set(key, std::move(array));
        } break;
        default: assert(false);
      }
      return false;
    }

    inline bool setJsonValueToAny(nt::AnyView& any, const rapidjson::Value& value) noexcept {
      switch (any.type()) {
        case Type::INT64: {
          return any.set(value.GetInt64());
        } break;
        case Type::INT32: {
          return any.set(value.GetInt());
        } break;
        case Type::UINT32: {
          return any.set(value.GetUint());
        } break;
        case Type::UINT64: {
          return any.set(value.GetUint64());
        } break;
        case Type::FLOAT: {
          return any.set(value.GetFloat());
        } break;
        case Type::DOUBLE: {
          return any.set(value.GetDouble());
        } break;
        case Type::STRING: {
          return any.set(nt::String(value.GetString()));
        } break;
        case Type::BOOL: {
          return any.set(value.GetBool());
        } break;
        case Type::ARRAY_DOUBLE: {
          const auto&        json_array = value.GetArray();
          DoubleDynamicArray array;
          array.reserve(json_array.Size());
          for (const auto& v: json_array)
            array.push_back(v.GetDouble());
          return any.set(std::move(array));
        } break;
        case Type::ARRAY_FLOAT: {
          const auto&       json_array = value.GetArray();
          FloatDynamicArray array;
          array.reserve(json_array.Size());
          for (const auto& v: json_array)
            array.push_back(v.GetFloat());
          return any.set(std::move(array));
        } break;
        case Type::ARRAY_STRING: {
          const auto&            json_array = value.GetArray();
          DynamicArray< String > array;
          array.reserve(json_array.Size());
          for (const auto& v: json_array)
            array.push_back(nt::String(v.GetString()));
          return any.set(std::move(array));
        } break;
        case Type::ARRAY_INT32: {
          const auto&     json_array = value.GetArray();
          IntDynamicArray array;
          array.reserve(json_array.Size());
          for (const auto& v: json_array)
            array.push_back(v.GetInt());
          return any.set(std::move(array));
        } break;
        case Type::ARRAY_INT64: {
          const auto&       json_array = value.GetArray();
          Int64DynamicArray array;
          array.reserve(json_array.Size());
          for (const auto& v: json_array)
            array.push_back(v.GetInt64());
          return any.set(std::move(array));
        } break;
        case Type::ARRAY_UINT32: {
          const auto&      json_array = value.GetArray();
          UintDynamicArray array;
          array.reserve(json_array.Size());
          for (const auto& v: json_array)
            array.push_back(v.GetUint());
          return any.set(std::move(array));
        } break;
        case Type::ARRAY_UINT64: {
          const auto&        json_array = value.GetArray();
          Uint64DynamicArray array;
          array.reserve(json_array.Size());
          for (const auto& v: json_array)
            array.push_back(v.GetUint64());
          return any.set(std::move(array));
        } break;
        case Type::ARRAY_BOOL: {
          const auto&                 json_array = value.GetArray();
          TrivialDynamicArray< bool > array;
          array.reserve(json_array.Size());
          for (const auto& v: json_array)
            array.push_back(v.GetBool());
          return any.set(std::move(array));
        } break;
        default: assert(false);
      }
      return false;
    }

    inline rapidjson::Value getJsonValueFromAny(const nt::AnyView&                  any,
                                                rapidjson::Document::AllocatorType& allocator) noexcept {
      switch (any.type()) {
        case Type::INT64: {
          return rapidjson::Value(any.get< std::int64_t >(NT_INT64_MAX));
        } break;
        case Type::INT32: {
          return rapidjson::Value(any.get< std::int32_t >(NT_INT32_MAX));
        } break;
        case Type::UINT32: {
          return rapidjson::Value(any.get< std::uint32_t >(NT_UINT32_MAX));
        } break;
        case Type::UINT64: {
          return rapidjson::Value(any.get< std::uint64_t >(NT_UINT64_MAX));
        } break;
        case Type::FLOAT: {
          return rapidjson::Value(any.get< float >(NT_FLOAT_MAX));
        } break;
        case Type::DOUBLE: {
          return rapidjson::Value(any.get< double >(NT_DOUBLE_MAX));
        } break;
        case Type::STRING: {
          const nt::String& s = any.get< nt::String >(nt::String());
          rapidjson::Value  str;
          str.SetString(s.c_str(), s.size(), allocator);
          return str;
        } break;
        case Type::BOOL: {
          return rapidjson::Value(any.get< bool >(NT_BOOL_MAX));
        } break;
        case Type::ARRAY_DOUBLE: {
          rapidjson::Value          json_array(rapidjson::kArrayType);
          const DoubleDynamicArray& array = any.get< DoubleDynamicArray >(DoubleDynamicArray());
          json_array.Reserve(array.size(), allocator);
          for (auto v: array)
            json_array.PushBack(v, allocator);
          return json_array;
        } break;
        case Type::ARRAY_FLOAT: {
          rapidjson::Value         json_array(rapidjson::kArrayType);
          const FloatDynamicArray& array = any.get< FloatDynamicArray >(FloatDynamicArray());
          json_array.Reserve(array.size(), allocator);
          for (auto v: array)
            json_array.PushBack(v, allocator);
          return json_array;
        } break;
        case Type::ARRAY_STRING: {
          rapidjson::Value              json_array(rapidjson::kArrayType);
          const DynamicArray< String >& array = any.get< DynamicArray< String > >(DynamicArray< String >());
          json_array.Reserve(array.size(), allocator);
          for (const String& v: array) {
            rapidjson::Value str;
            str.SetString(v.c_str(), v.size(), allocator);
            json_array.PushBack(std::move(str), allocator);
          }
          return json_array;
        } break;
        case Type::ARRAY_INT32: {
          rapidjson::Value       json_array(rapidjson::kArrayType);
          const IntDynamicArray& array = any.get< IntDynamicArray >(IntDynamicArray());
          json_array.Reserve(array.size(), allocator);
          for (auto v: array)
            json_array.PushBack(v, allocator);
          return json_array;
        } break;
        case Type::ARRAY_INT64: {
          rapidjson::Value         json_array(rapidjson::kArrayType);
          const Int64DynamicArray& array = any.get< Int64DynamicArray >(Int64DynamicArray());
          json_array.Reserve(array.size(), allocator);
          for (auto v: array)
            json_array.PushBack(v, allocator);
          return json_array;
        } break;
        case Type::ARRAY_UINT32: {
          rapidjson::Value        json_array(rapidjson::kArrayType);
          const UintDynamicArray& array = any.get< UintDynamicArray >(UintDynamicArray());
          json_array.Reserve(array.size(), allocator);
          for (auto v: array)
            json_array.PushBack(v, allocator);
          return json_array;
        } break;
        case Type::ARRAY_UINT64: {
          rapidjson::Value          json_array(rapidjson::kArrayType);
          const Uint64DynamicArray& array = any.get< Uint64DynamicArray >(Uint64DynamicArray());
          json_array.Reserve(array.size(), allocator);
          for (auto v: array)
            json_array.PushBack(v, allocator);
          return json_array;
        } break;
        case Type::ARRAY_BOOL: {
          rapidjson::Value                   json_array(rapidjson::kArrayType);
          const TrivialDynamicArray< bool >& array =
             any.get< TrivialDynamicArray< bool > >(TrivialDynamicArray< bool >());
          json_array.Reserve(array.size(), allocator);
          for (auto v: array)
            json_array.PushBack(v, allocator);
          return json_array;
        } break;
      }
      assert(false);
      return rapidjson::Value();
    }

    template < typename K >
    inline rapidjson::Value getJsonValueFromMap(const MapStorage< K >&              map_storage,
                                                const K&                            key,
                                                rapidjson::Document::AllocatorType& allocator) noexcept {
      const AnyView any = map_storage.get(key);
      return getJsonValueFromAny(any, allocator);
    }

  }   // namespace details

}   // namespace nt


#ifdef RAPIDJSON_WINDOWS_GETOBJECT_WORKAROUND_APPLIED
#  pragma pop_macro("GetObject")
#endif

#endif
