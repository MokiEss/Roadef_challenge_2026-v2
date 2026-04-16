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
 * @brief networktools assertion handling
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_ASSERT_H_
#  define _NT_ASSERT_H_

#  include <cassert>
#  include <cstdlib>
#  include <iostream>

namespace nt {
  namespace _assert_bits {
    inline const char* cstringify(const std::string& str) { return str.c_str(); }

    inline const char* cstringify(const char* str) { return str; }
  }   // namespace _assert_bits

  inline void nt_default_assert_handler(
     const char* file, int line, const char* function, const char* message, const char* expression) {
    std::cerr << file << ":" << line << ": ";
    if (function) { std::cerr << "In function '" << function << "': "; }
    std::cerr << "Assertion '" << expression << "' failed";
    if (message && message[0] != '\0') { std::cerr << ": " << message; }
    std::cerr << std::endl;
    std::abort();
  }
}   // namespace nt

#endif   // _NT_ASSERT_H_

#undef NT_ASSERT
#undef NT_ASSERT_HANDLER
#undef NT_DEBUG
#undef NT_FUNCTION_NAME

#if defined(__GNUC__)
#  define NT_FUNCTION_NAME __PRETTY_FUNCTION__
#elif defined(_MSC_VER)
#  define NT_FUNCTION_NAME __FUNCSIG__
#else
#  define NT_FUNCTION_NAME __func__
#endif

#if defined(NT_DISABLE_ASSERTS)
#  define NT_ASSERT_HANDLER(file, line, func, msg, exp) ((void)0)
#  define NT_ASSERT(expr, msg) ((void)0)
#elif defined(NT_ASSERT_CUSTOM)
#  ifndef NT_CUSTOM_ASSERT_HANDLER
#    error "NT_ASSERT_CUSTOM defined but NT_CUSTOM_ASSERT_HANDLER not provided"
#  endif
#  define NT_ASSERT_HANDLER NT_CUSTOM_ASSERT_HANDLER
#  define NT_ASSERT(expr, msg) \
    (static_cast< void >(      \
       !!(expr)                \
          ? 0                  \
          : (NT_ASSERT_HANDLER(__FILE__, __LINE__, NT_FUNCTION_NAME, ::nt::_assert_bits::cstringify(msg), #expr), 0)))
#elif defined(NDEBUG)
#  define NT_ASSERT_HANDLER(file, line, func, msg, exp) ((void)0)
#  define NT_ASSERT(expr, msg) ((void)0)
#  define NT_ASSERT_PTR(ptr) (ptr)
#else
#  define NT_ASSERT_HANDLER nt_default_assert_handler
#  define NT_ASSERT(expr, msg) \
    (static_cast< void >(      \
       !!(expr)                \
          ? 0                  \
          : (NT_ASSERT_HANDLER(__FILE__, __LINE__, NT_FUNCTION_NAME, ::nt::_assert_bits::cstringify(msg), #expr), 0)))
#  define NT_ASSERT_PTR(ptr) (NT_ASSERT(ptr, "Null Pointer"), ptr)
#endif

#if defined(NT_DISABLE_ASSERTS)
#  undef NT_ASSERT_PTR
#  define NT_ASSERT_PTR(ptr) (ptr)
#endif
#define NT_DEBUG(exp, msg)