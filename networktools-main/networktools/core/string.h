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
 * @brief String utility functions
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_STRING_H_
#define _NT_STRING_H_

#include <string>
#include <cstring>
#include <fstream>

#include "common.h"
#include "arrays.h"

#ifndef NT_STRDUP
#  define NT_STRDUP(sz) strdup(sz)
#endif
#ifndef NT_STRFREE
#  define NT_STRFREE(sz) free(sz)
#endif

namespace nt {

  /**
   * @brief Converts a byte array to a value of the specified type.
   *
   * This function converts a byte array to a value of the specified type.
   * The byte array is interpreted according to the specified endianness.
   *
   * Example of usage:
   *
   * @code
   *   char* sz_num = "10";
   *   uint64_t i_num = nt::bytes_to< uint64_t >(sz_num, false); // i_num = 10
   * @endcode
   *
   * @tparam TYPE The type of the value to be converted.
   * @param p_buff A pointer to the byte array.
   * @param b_big_endian Flag indicating whether the byte array is in big-endian format.
   * @return The converted value of type `TYPE`.
   */
  template < class TYPE >
  TYPE bytes_to(char* p_buff, const bool b_big_endian) {
    TYPE          output = 0;
    constexpr int i_size = sizeof(TYPE);
    const int     i_big_endian = b_big_endian ? 1 : 0;
    for (int i = 0; i < i_size; ++i)
      output |= ((unsigned char)p_buff[(i_size - i) * i_big_endian + i * (1 - i_big_endian)]) << i * 8;
    return output;
  }

  /**
   * @brief Converts a byte array to a value of the specified type.
   *
   * This function converts a byte array to a value of the specified type. 
   * The byte array is interpreted according to the endianness of the system.
   *
   * Example of usage:
   *
   * @code
   *   char* sz_num = "10";
   *   uint64_t i_num = nt::bytes_to< uint64_t >(sz_num); // i_num = 10
   * @endcode
   *
   * @tparam TYPE The type of the value to be converted.
   * @param p_buff A pointer to the byte array.
   * @return The converted value of type `TYPE`.
   */
  template < class TYPE >
  TYPE bytes_to(char* p_buff) {
    TYPE          output = 0;
    constexpr int i_size = sizeof(TYPE);
    constexpr int i_big_endian = NT_IS_BIG_ENDIAN ? 1 : 0;
    for (int i = 0; i < i_size; ++i)
      output |= ((unsigned char)p_buff[(i_size - i) * i_big_endian + i * (1 - i_big_endian)]) << i * 8;
    return output;
  }

  /**
   * @brief Converts a value of any type to an array of bytes.
   *
   * This function copies the bytes representing the given value to the specified buffer.
   *
   * Example of usage:
   *
   * @code
   *   char* buff[8];
   *   uint64_t i_num = 10;
   *   nt::to_bytes(i_num, sz_num);
   * @endcode
   *
   *
   * @tparam TYPE The type of the value to convert.
   * @param value The value to convert to bytes.
   * @param p_buff A pointer to the destination buffer to store the byte array.
   *
   * @note The destination buffer should have enough space to store the converted value.
   */
  template < typename TYPE >
  void to_bytes(const TYPE& value, char* p_buff) {
    std::memcpy(p_buff, &value, sizeof(TYPE));
  }

  /**
   * @brief Hash functions, see the appropriate copyright below.
   *
   * Copyright (c) 2015 Daniel Kirchner
   *
   * Permission is hereby granted, free of charge, to any person obtaining a copy
   * of this software and associated documentation files (the "Software"), to deal
   * in the Software without restriction, including without limitation the rights
   * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   * copies of the Software, and to permit persons to whom the Software is
   * furnished to do so, subject to the following conditions:
   *
   * The above copyright notice and this permission notice shall be included in
   * all copies or substantial portions of the Software.
   *
   * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   * THE SOFTWARE.
   */
  struct xxh64 {
    static constexpr uint64_t hash(const char* p, uint64_t len, uint64_t seed) {
      return finalize((len >= 32 ? h32bytes(p, len, seed) : seed + PRIME5) + len, p + (len & ~0x1F), len & 0x1F);
    }
    static uint64_t hash(const char* p) { return hash(p, std::strlen(p), 0); }

    private:
    static constexpr uint64_t PRIME1 = 11400714785074694791ULL;
    static constexpr uint64_t PRIME2 = 14029467366897019727ULL;
    static constexpr uint64_t PRIME3 = 1609587929392839161ULL;
    static constexpr uint64_t PRIME4 = 9650029242287828579ULL;
    static constexpr uint64_t PRIME5 = 2870177450012600261ULL;

    static constexpr uint64_t rotl(uint64_t x, int r) noexcept { return ((x << r) | (x >> (64 - r))); }
    static constexpr uint64_t mix1(const uint64_t h, const uint64_t prime, int rshift) noexcept {
      return (h ^ (h >> rshift)) * prime;
    }
    static constexpr uint64_t mix2(const uint64_t p, const uint64_t v = 0) noexcept {
      return rotl(v + p * PRIME2, 31) * PRIME1;
    }
    static constexpr uint64_t mix3(const uint64_t h, const uint64_t v) noexcept {
      return (h ^ mix2(v)) * PRIME1 + PRIME4;
    }

    static constexpr uint32_t endian32(const char* v) noexcept {
      if constexpr (NT_IS_BIG_ENDIAN)
        return uint32_t(uint8_t(v[3])) | (uint32_t(uint8_t(v[2])) << 8) | (uint32_t(uint8_t(v[1])) << 16)
               | (uint32_t(uint8_t(v[0])) << 24);
      else
        return uint32_t(uint8_t(v[0])) | (uint32_t(uint8_t(v[1])) << 8) | (uint32_t(uint8_t(v[2])) << 16)
               | (uint32_t(uint8_t(v[3])) << 24);
    }

    static constexpr uint64_t endian64(const char* v) noexcept {
      if constexpr (NT_IS_BIG_ENDIAN)
        return uint64_t(uint8_t(v[7])) | (uint64_t(uint8_t(v[6])) << 8) | (uint64_t(uint8_t(v[5])) << 16)
               | (uint64_t(uint8_t(v[4])) << 24) | (uint64_t(uint8_t(v[3])) << 32) | (uint64_t(uint8_t(v[2])) << 40)
               | (uint64_t(uint8_t(v[1])) << 48) | (uint64_t(uint8_t(v[0])) << 56);
      else
        return uint64_t(uint8_t(v[0])) | (uint64_t(uint8_t(v[1])) << 8) | (uint64_t(uint8_t(v[2])) << 16)
               | (uint64_t(uint8_t(v[3])) << 24) | (uint64_t(uint8_t(v[4])) << 32) | (uint64_t(uint8_t(v[5])) << 40)
               | (uint64_t(uint8_t(v[6])) << 48) | (uint64_t(uint8_t(v[7])) << 56);
    }

    static constexpr uint64_t fetch64(const char* p, const uint64_t v = 0) noexcept { return mix2(endian64(p), v); }
    static constexpr uint64_t fetch32(const char* p) noexcept { return uint64_t(endian32(p)) * PRIME1; }
    static constexpr uint64_t fetch8(const char* p) noexcept { return uint8_t(*p) * PRIME5; }
    static constexpr uint64_t finalize(const uint64_t h, const char* p, uint64_t len) noexcept {
      return (len >= 8) ? (finalize(rotl(h ^ fetch64(p), 27) * PRIME1 + PRIME4, p + 8, len - 8))
                        : ((len >= 4) ? (finalize(rotl(h ^ fetch32(p), 23) * PRIME2 + PRIME3, p + 4, len - 4))
                                      : ((len > 0) ? (finalize(rotl(h ^ fetch8(p), 11) * PRIME1, p + 1, len - 1))
                                                   : (mix1(mix1(mix1(h, PRIME2, 33), PRIME3, 29), 1, 32))));
    }
    static constexpr uint64_t h32bytes(const char*    p,
                                       uint64_t       len,
                                       const uint64_t v1,
                                       const uint64_t v2,
                                       const uint64_t v3,
                                       const uint64_t v4) noexcept {
      return (len >= 32)
                ? h32bytes(
                   p + 32, len - 32, fetch64(p, v1), fetch64(p + 8, v2), fetch64(p + 16, v3), fetch64(p + 24, v4))
                : mix3(mix3(mix3(mix3(rotl(v1, 1) + rotl(v2, 7) + rotl(v3, 12) + rotl(v4, 18), v1), v2), v3), v4);
    }
    static constexpr uint64_t h32bytes(const char* p, uint64_t len, const uint64_t seed) noexcept {
      return h32bytes(p, len, seed + PRIME1 + PRIME2, seed + PRIME2, seed, seed - PRIME1);
    }
  };

  /**
   * @brief 64-bit FNV-1a hash function
   *
   * @param s
   * @return uint64_t
   */
  static inline uint64_t fnv1a64(const char* s) {
    constexpr uint64_t FNV_OFFSET = 0xCBF29CE484222325;
    constexpr uint64_t FNV_PRIME = 0x100000001B3;
    uint64_t           h = FNV_OFFSET;
    for (const unsigned char* p = reinterpret_cast< const unsigned char* >(s); *p; ++p) {
      h ^= (uint64_t)(*p);
      h *= FNV_PRIME;
    }
    return h;
  }

  /**
   * @brief Safely copies a C-style string to a destination buffer with a specified size.
   *
   * @param to Pointer to the destination buffer.
   * @param from Pointer to the source string.
   * @param to_size Size of the destination buffer.
   *
   * @return char* Pointer to the destination buffer.
   */
  inline char* strcpy_safe(char* to, const char* from, size_t to_size) {
    char* p_result = std::strncpy(to, from, to_size - 1);
    to[to_size - 1] = '\0';
    return p_result;
  }

  namespace details {
    template < class IMPL >
    struct StringBase {
      using Impl = IMPL;
      Impl _impl;

      static const size_t npos = Impl::npos;

      using value_type = typename Impl::value_type;
      using traits_type = typename Impl::traits_type;
      using allocator_type = typename Impl::allocator_type;
      using size_type = typename Impl::size_type;
      using difference_type = typename Impl::difference_type;
      using reference = typename Impl::reference;
      using const_reference = typename Impl::const_reference;
      using pointer = typename Impl::pointer;
      using const_pointer = typename Impl::const_pointer;
      using iterator = typename Impl::iterator;
      using const_iterator = typename Impl::const_iterator;
      using reverse_iterator = typename Impl::reverse_iterator;
      using const_reverse_iterator = typename Impl::const_reverse_iterator;

      // Constructors
      StringBase() = default;
      StringBase(const char* s) : _impl(s) {}
      StringBase(const char* s, size_t n) : _impl(s, n) {}
      StringBase(Impl&& s) : _impl(std::forward< Impl >(s)) {}
      StringBase(StringBase&& other) noexcept = default;
      StringBase(size_t count, char ch) : _impl(count, ch) {}

      // Remove copy assignments/constructors. Use copyFrom() or assign()
      StringBase(const StringBase& other) = delete;
      StringBase& operator=(const StringBase& other) = delete;
      StringBase(const Impl& s) = delete;
      StringBase& operator=(const Impl& s) = delete;

      // Assignment operators
      StringBase& operator=(StringBase&& other) noexcept = default;
      StringBase& operator=(const char* s) {
        _impl = s;
        return *this;
      }
      // StringBase& operator=(const Impl& s) {
      //   _impl = s;
      //   return *this;
      // }
      StringBase& operator=(Impl&& s) {
        _impl = std::move(s);
        return *this;
      }
      StringBase& operator=(char c) {
        _impl = c;
        return *this;
      }

      // Assign
      StringBase& assign(const StringBase& str) {
        _impl.assign(str._impl);
        return *this;
      }
      StringBase& assign(const char* s) {
        _impl.assign(s);
        return *this;
      }
      StringBase& assign(const char* s, size_t n) {
        _impl.assign(s, n);
        return *this;
      }
      StringBase& assign(size_t count, char ch) {
        _impl.assign(count, ch);
        return *this;
      }

      constexpr void copyFrom(const StringBase& from) noexcept { assign(from); }
      constexpr void copyFrom(const char* s) noexcept { assign(s); }
      constexpr void copyFrom(const char* s, size_t n) noexcept { assign(s, n); }
      constexpr void copyFrom(size_t count, char ch) noexcept { assign(count, ch); }

      // Element access
      char&       at(size_t pos) { return _impl.at(pos); }
      const char& at(size_t pos) const { return _impl.at(pos); }
      char&       operator[](size_t pos) { return _impl[pos]; }
      const char& operator[](size_t pos) const { return _impl[pos]; }
      char&       front() { return _impl.front(); }
      const char& front() const { return _impl.front(); }
      char&       back() { return _impl.back(); }
      const char& back() const { return _impl.back(); }
      const char* c_str() const noexcept { return _impl.c_str(); }
      const char* data() const noexcept { return _impl.data(); }
      char*       data() noexcept { return &_impl[0]; }

      // Iterators
      iterator               begin() { return _impl.begin(); }
      const_iterator         begin() const { return _impl.begin(); }
      iterator               end() { return _impl.end(); }
      const_iterator         end() const { return _impl.end(); }
      reverse_iterator       rbegin() { return _impl.rbegin(); }
      const_reverse_iterator rbegin() const { return _impl.rbegin(); }
      reverse_iterator       rend() { return _impl.rend(); }
      const_reverse_iterator rend() const { return _impl.rend(); }
      const_iterator         cbegin() const { return _impl.cbegin(); }
      const_iterator         cend() const { return _impl.cend(); }
      const_reverse_iterator crbegin() const { return _impl.crbegin(); }
      const_reverse_iterator crend() const { return _impl.crend(); }

      // Capacity
      size_t size() const noexcept { return _impl.size(); }
      size_t length() const noexcept { return _impl.length(); }
      size_t max_size() const noexcept { return _impl.max_size(); }
      void   resize(size_t n) { _impl.resize(n); }
      void   resize(size_t n, char c) { _impl.resize(n, c); }
      size_t capacity() const noexcept { return _impl.capacity(); }
      void   reserve(size_t n = 0) { _impl.reserve(n); }
      void   shrink_to_fit() { _impl.shrink_to_fit(); }
      bool   empty() const noexcept { return _impl.empty(); }
      void   clear() noexcept { _impl.clear(); }

      // Modifiers
      void        push_back(char c) { _impl.push_back(c); }
      void        pop_back() { _impl.pop_back(); }
      StringBase& append(const StringBase& str) {
        _impl.append(str._impl);
        return *this;
      }
      StringBase& append(const char* s) {
        _impl.append(s);
        return *this;
      }
      StringBase& append(const char* s, size_t n) {
        _impl.append(s, n);
        return *this;
      }
      StringBase& append(size_t count, char ch) {
        _impl.append(count, ch);
        return *this;
      }
      StringBase& operator+=(const StringBase& str) {
        _impl += str._impl;
        return *this;
      }
      StringBase& operator+=(const char* s) {
        _impl += s;
        return *this;
      }
      StringBase& operator+=(char c) {
        _impl += c;
        return *this;
      }
      StringBase& insert(size_t pos, const StringBase& str) {
        _impl.insert(pos, str._impl);
        return *this;
      }
      StringBase& insert(size_t pos, const char* s) {
        _impl.insert(pos, s);
        return *this;
      }
      StringBase& insert(size_t pos, const char* s, size_t n) {
        _impl.insert(pos, s, n);
        return *this;
      }
      StringBase& insert(size_t pos, size_t count, char ch) {
        _impl.insert(pos, count, ch);
        return *this;
      }
      StringBase& erase(size_t pos = 0, size_t len = npos) {
        _impl.erase(pos, len);
        return *this;
      }
      void swap(StringBase& other) noexcept { _impl.swap(other._impl); }

      // String operations
      size_t copy(char* dest, size_t count, size_t pos = 0) const { return _impl.copy(dest, count, pos); }
      size_t find(const StringBase& str, size_t pos = 0) const { return _impl.find(str._impl, pos); }
      size_t find(const char* s, size_t pos = 0) const { return _impl.find(s, pos); }
      size_t find(const char* s, size_t pos, size_t n) const { return _impl.find(s, pos, n); }
      size_t find(char c, size_t pos = 0) const { return _impl.find(c, pos); }
      size_t rfind(const StringBase& str, size_t pos = npos) const { return _impl.rfind(str._impl, pos); }
      size_t rfind(const char* s, size_t pos = npos) const { return _impl.rfind(s, pos); }
      size_t rfind(const char* s, size_t pos, size_t n) const { return _impl.rfind(s, pos, n); }
      size_t rfind(char c, size_t pos = npos) const { return _impl.rfind(c, pos); }
      size_t find_first_of(const StringBase& str, size_t pos = 0) const { return _impl.find_first_of(str._impl, pos); }
      size_t find_first_of(const char* s, size_t pos = 0) const { return _impl.find_first_of(s, pos); }
      size_t find_first_of(const char* s, size_t pos, size_t n) const { return _impl.find_first_of(s, pos, n); }
      size_t find_first_of(char c, size_t pos = 0) const { return _impl.find_first_of(c, pos); }
      size_t find_last_of(const StringBase& str, size_t pos = npos) const { return _impl.find_last_of(str._impl, pos); }
      size_t find_last_of(const char* s, size_t pos = npos) const { return _impl.find_last_of(s, pos); }
      size_t find_last_of(const char* s, size_t pos, size_t n) const { return _impl.find_last_of(s, pos, n); }
      size_t find_last_of(char c, size_t pos = npos) const { return _impl.find_last_of(c, pos); }
      size_t find_first_not_of(const StringBase& str, size_t pos = 0) const {
        return _impl.find_first_not_of(str._impl, pos);
      }
      size_t find_first_not_of(const char* s, size_t pos = 0) const { return _impl.find_first_not_of(s, pos); }
      size_t find_first_not_of(const char* s, size_t pos, size_t n) const { return _impl.find_first_not_of(s, pos, n); }
      size_t find_first_not_of(char c, size_t pos = 0) const { return _impl.find_first_not_of(c, pos); }
      size_t find_last_not_of(const StringBase& str, size_t pos = npos) const {
        return _impl.find_last_not_of(str._impl, pos);
      }
      size_t find_last_not_of(const char* s, size_t pos = npos) const { return _impl.find_last_not_of(s, pos); }
      size_t find_last_not_of(const char* s, size_t pos, size_t n) const { return _impl.find_last_not_of(s, pos, n); }
      size_t find_last_not_of(char c, size_t pos = npos) const { return _impl.find_last_not_of(c, pos); }
      int    compare(const StringBase& str) const { return _impl.compare(str._impl); }
      int    compare(const char* s) const { return _impl.compare(s); }
      StringBase substr(size_t pos = 0, size_t len = npos) const { return _impl.substr(pos, len); }

      // Conversion
      operator const Impl&() const noexcept { return _impl; }
      operator Impl&() noexcept { return _impl; }

      // Underlying implementation access
      constexpr Impl&       getUnderlyingImpl() noexcept { return _impl; }
      constexpr const Impl& getUnderlyingImpl() const noexcept { return _impl; }

      // Comparison operators
      bool operator==(const StringBase& other) const { return _impl == other._impl; }
      bool operator!=(const StringBase& other) const { return _impl != other._impl; }
      bool operator<(const StringBase& other) const { return _impl < other._impl; }
      bool operator>(const StringBase& other) const { return _impl > other._impl; }
      bool operator<=(const StringBase& other) const { return _impl <= other._impl; }
      bool operator>=(const StringBase& other) const { return _impl >= other._impl; }

      bool operator==(const char* other) const { return _impl == other; }
      bool operator!=(const char* other) const { return _impl != other; }
      bool operator<(const char* other) const { return _impl < other; }
      bool operator>(const char* other) const { return _impl > other; }
      bool operator<=(const char* other) const { return _impl <= other; }
      bool operator>=(const char* other) const { return _impl >= other; }

      // Stream output
      inline friend std::ostream& operator<<(std::ostream& Out, const StringBase& m) { return Out << m._impl; }
    };
  }   // namespace details

  /**
   * @class String
   * @headerfile string.h
   * @brief This class acts as a wrapper for different implementations of third-party string
   * libraries. Mainly to facilitate the change to another string lib if better performance is
   * required.
   *
   */
  using String = details::StringBase< std::string >;

  /**
   * @brief Given a string, split it at ' ' and store the words in the provided tokens array
   *
   * @param line
   * @return number of words
   */
  inline int strtok(const String& line, DynamicArray< String >& tokens) noexcept {
    tokens.removeAll();
    size_t oldpos = 0, newpos = 0;
    while (newpos != String::npos) {
      newpos = line._impl.find(" ", oldpos);
      tokens.push_back(line._impl.substr(oldpos, newpos - oldpos));
      oldpos = newpos + 1;
    }
    return tokens.size();
  }

  /**
   * @brief Given a string, split it at ' ' and return vector of words
   *
   * @param line
   * @return DynamicArray< String >
   */
  inline DynamicArray< String > strtok(const String& line) noexcept {
    DynamicArray< String > tokens;
    strtok(line, tokens);
    return tokens;
  }

  /**
   * @brief An unsigned variant of std::stoi that also checks if the input consists
   * entirely of base-10 digits
   *
   * @param s
   * @return unsigned
   */
  inline unsigned pure_stou(const String& s) noexcept {
    unsigned result = 0;
    if (s.empty()) return ~0;
    for (unsigned i = 0; i < s.size(); ++i) {
      char c = s[i];
      if (c < '0' || c > '9') return ~0;
      result = 10 * result + (c - '0');
    }
    return result;
  }

  /**
   * @class ByteBuffer
   * @brief Represents an in-memory read-write string stream. This class is a partial
   * implementation of the class template std::basic_iostream.
   *
   */
  struct ByteBuffer {
    using char_type = char;   //< needed by rapidjson
    using value_type = char_type;

    TrivialDynamicArray< char, 8 > _buffer;
    int                            _head;
    int                            _gcount;

    ByteBuffer() : _head(-1), _gcount(0) { _buffer._p_data[0] = 0; }

    /**
     * @brief Loads the content of a file into the string buffer.
     *
     * @param sz_filename Filename to load.
     *
     * @return true
     * @return false
     */
    bool load(const char* sz_filename) noexcept {
      // Open stream
      std::basic_ifstream< char > stream(sz_filename, std::ios::binary);
      if (!stream) return false;
      stream.unsetf(std::ios::skipws);
      reset();

      // Determine stream size
      stream.seekg(0, std::ios::end);
      const std::size_t i_size = stream.tellg();
      stream.seekg(0);

      // Load data if the file is not empty
      if (i_size > 0) {
        _buffer.ensure(i_size + 1);
        stream.read(_buffer._p_data, static_cast< std::streamsize >(i_size));
        _buffer._p_data[i_size] = 0;
        _buffer._i_count = i_size;
        _head = 0;
      }

      return true;
    }

    /**
     * @brief Saves the content of the string buffer into a binary file.
     *
     * @param sz_filename Name of the binary file
     *
     * @return true
     * @return false
     */
    bool save(const char* sz_filename) noexcept {
      std::basic_ofstream< char > stream(sz_filename, std::ios::binary);
      if (!stream) return false;
      stream.write(_buffer._p_data, _buffer.size());
      stream.close();
      return stream.operator bool();
    }

    /**
     * @brief Copy the content of a null-terminated string
     *
     * @param sz_stream a null-terminated string.
     * @return true
     * @return false
     */
    ByteBuffer& copyFrom(const char* sz_stream) noexcept { return copyFrom(sz_stream, std::strlen(sz_stream)); }

    /**
     * @brief Copy the content of the first n bytes of a string
     *
     * @param stream a string.
     * @return true
     * @return false
     */
    ByteBuffer& copyFrom(const char* stream, int n) noexcept {
      if (n > 0) {
        _buffer.copyFrom(ArrayView< char >(stream, n));
        _appendNullChar();
        _head = 0;
      }
      return *this;
    }

    /**
     * @brief Copy content from another ByteBuffer
     *
     * @param other a ByteBuffer.
     */
    ByteBuffer& copyFrom(const ByteBuffer& other) noexcept {
      _buffer.copyFrom(other._buffer);
      _head = other._head;
      return *this;
    }

    // Read methods

    /**
     * @brief Returns the number of characters extracted by the last unformatted input operation.
     *
     * @return The number of characters extracted.
     */
    int gcount() const noexcept { return _gcount; }

    /**
     * @brief Returns the size of the underlying buffer
     *
     */
    int size() const noexcept { return _buffer.size(); }

    /**
     * @brief Reads characters from the stream and stores them into the specified character array.
     *
     * @param s Pointer to an array of characters where the read characters are stored.
     * @param n Maximum number of characters to read.
     */
    ByteBuffer& read(char* s, int n) noexcept {
      if (_head < 0 || n <= 0) return *this;

      _gcount = std::min(_buffer.size() - _head, n);
      if (_gcount > 0) std::memcpy(s, &_buffer[_head], _gcount);
      _head += n;

      if (_head > _buffer.size()) _head = -1;

      return *this;
    }

    /**
     * @brief Sets the position of the next character to be read from the stream.
     *
     * @param pos The new absolute position within the stream (relative to the beginning).
     */
    ByteBuffer& seekg(int pos) noexcept {
      _head = -1;
      if (pos < _buffer.size()) _head = pos;
      return *this;
    }

    /**
     * @brief Returns the position of the current character in the input stream.
     *
     * @return int
     */
    int tellg() const noexcept { return _head; }

    // Write methods

    /**
     * @brief Writes a character to the stream.
     *
     * @param c The character to be written.
     */
    constexpr ByteBuffer& put(char c) noexcept {
      _buffer.add(c);
      _appendNullChar();
      return *this;
    };

    /**
     * @brief Alias to put(). Allows to use ByteBuffer with std::back_inserter().
     *
     * @param c The character to be written.
     */
    constexpr void push_back(char c) noexcept { put(c); }

    /**
     * @brief Writes a sequence of characters to the stream.
     *
     * @param s Pointer to an array of characters to be written.
     * @param n Number of characters to be written.
     */
    ByteBuffer& write(const char* s, int n) noexcept {
      _buffer.append(ArrayView(s, n));
      _appendNullChar();
      return *this;
    }

    /**
     * @brief Flushes the stream.
     */
    void flush() noexcept {}

    /**
     * @brief Returns a pointer to the underlying character array.
     *
     * @return A pointer to the character array.
     */
    const char* data() const noexcept { return _buffer.data(); }
    char*       data() noexcept { return _buffer.data(); }

    /**
     * @brief Remove the content of the stream buffer.
     */
    void reset() noexcept {
      _buffer.removeAll();
      _head = -1;
      _gcount = 0;
      assert(_buffer.capacity());
      _buffer._p_data[0] = 0;
    }

    /**
     * @brief Checks whether the buffer has no errors.
     *
     */
    explicit operator bool() const { return _head >= 0; }

    // Private methods

    constexpr void _appendNullChar() noexcept {
      _buffer._growIfNecessary();
      assert(_buffer.capacity() > _buffer.size());
      _buffer._p_data[_buffer.size()] = 0;
    }
  };
}   // namespace nt

#endif
