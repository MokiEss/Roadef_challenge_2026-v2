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
 * @brief This file defines wrapper classes/functions to the fmt library. The goal is to avoid
 *        networktools to be dependent of the fmt's API. Ideally, any call to a fmt class/function
 *        should be done through the present wrappers.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_FORMAT_H_
#define _NT_FORMAT_H_

#ifndef FMT_HEADER_ONLY
#  define FMT_HEADER_ONLY 1
#endif

// #ifndef FMT_STATIC_THOUSANDS_SEPARATOR
// #  define FMT_STATIC_THOUSANDS_SEPARATOR
// #endif

#include "../@deps/fmt/format.h"
#include "../@deps/fmt/os.h"
#include "../@deps/fmt/printf.h"

#include "string.h"

template <>
struct fmt::formatter< nt::String >: formatter< std::string_view > {
  auto format(const nt::String& s, format_context& ctx) const -> format_context::iterator {
    return formatter< string_view >::format(s.getUnderlyingImpl(), ctx);
  }
};

namespace nt {

  using MemoryBuffer = fmt::memory_buffer;   ///< Alias for fmt::memory_buffer.

  /**
   * @brief Prints formatted data to ``stdout``.
   *
   * Example of usage:
   *
   * @code
   * fmt::printf("Elapsed time: %.2f seconds", 1.23);
   * @endcode
   *
   */
  template < typename... Args >
  inline auto printf(Args&&... args) -> decltype(fmt::printf(std::forward< Args >(args)...)) {
    return fmt::printf(std::forward< Args >(args)...);
  }

  /**
   * @brief Opens a file for writing. Supported parameters passed in *params*:
   *      - ``<integer>``: Flags passed to `open
   *   <https://pubs.opengroup.org/onlinepubs/007904875/functions/open.html>`_
   *   (``file::WRONLY | file::CREATE | file::TRUNC`` by default)
   *      - ``buffer_size=<integer>``: Output buffer size
   *
   * Example of usage:
   *
   * @code
   *   auto out = nt::output_file("guide.txt");
   *   out.print("Don't {}", "Panic");
   * @endcode
   *
   */
  template < typename... Args >
  inline auto output_file(Args&&... args) -> decltype(fmt::output_file(std::forward< Args >(args)...)) {
    return fmt::output_file(std::forward< Args >(args)...);
  }

  /**
   * @brief  Formats ``args`` according to specifications in ``fmt`` and returns the result as a
   * string.
   *
   * Example of usage:
   *
   * @code
   * std::string message = nt::format("The answer is {}.", 42);
   * @endcode
   *
   * Note: The constructor of fmt::format_string is consteval (C++20) and hence it requires to
   * take a fmt::format_string< Args... > as argument instead of a generic type i.e we cannot
   * simply declare the format function as follows
   *
   * auto format(Args&&... args) ->
   * decltype(fmt::format(std::forward< Args >(args)...)) {
   *  return fmt::format(std::forward< Args >(args)...);
   * }
   *
   */
  template < typename... Args >
  inline auto format(fmt::format_string< Args... > s, Args&&... args)
     -> decltype(fmt::format(s, std::forward< Args >(args)...)) {
    return fmt::format(s, std::forward< Args >(args)...);
  }

  /**
   * @brief Formats ``args`` according to specifications in ``fmt``, writes the result to
   * the output iterator ``out`` and returns the iterator past the end of the output
   * range. `format_to` does not append a terminating null character.
   *
   * Example of usage:
   *
   * @code
   *  auto out = std::vector<char>();
   *  nt::format_to(std::back_inserter(out), "{}", 42);
   * @endcode
   *
   */
  template < typename OutputIt, typename... Args >
  inline auto format_to(OutputIt out, fmt::format_string< Args... > fmt, Args&&... args)
     -> decltype(fmt::format_to(out, fmt, std::forward< Args >(args)...)) {
    return fmt::format_to(out, fmt, std::forward< Args >(args)...);
  }

  /**
   * @brief Converts *value* to ``std::string`` using the default format for type *T*.
   *
   * Example of usage:
   *
   * @code
   *  std::string answer = nt::to_string(42);
   * @endcode
   */
  template < typename... Args >
  inline auto to_string(Args&&... args) -> decltype(fmt::to_string(std::forward< Args >(args)...)) {
    return fmt::to_string(std::forward< Args >(args)...);
  }

}   // namespace nt

#endif
