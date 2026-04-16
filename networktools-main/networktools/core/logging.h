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
 * @brief C++ wrapper of loguru (see https://github.com/emilk/loguru for more details)
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_LOGGING_H_
#define _NT_LOGGING_H_

#ifndef LOGURU_USE_FMTLIB
#  define LOGURU_USE_FMTLIB 1
#endif

#include "../@deps/loguru/loguru.h"

namespace nt {

  /**
   * @class Logging
   * @headerfile logging.h
   * @brief C++ wrapper of loguru (see https://github.com/emilk/loguru for more details)
   *
   * Example of usage :
   *
   *   int main(int argc, char* argv[]) {
   *     // Initialize loguru
   *     Logging::init(argc, argv);
   *
   *     // Add a log file
   *     Logging::addFile("log.txt", FileMode::Append, Verbosity::Info);
   *
   *     // Log messages with different verbosity levels
   *     Logging::log(Verbosity::Info, __FILE__, __LINE__,
   *                  "This is an informational message; argc = {}", argc);
   *     Logging::log(Verbosity::Warning, __FILE__, __LINE__,
   *                  "This is a warning message; argc = {}", argc);
   *     Logging::log(Verbosity::Error, __FILE__, __LINE__,
   *                  "This is an error message; argc = {}", argc);
   *
   *     // For convenience, macros can be used
   *     LOG_F(INFO, "This is an informational message; argc = {}", argc);
   *     LOG_F(WARNING, "This is a warning message; argc = {}", argc);
   *     LOG_F(ERROR, "This is an error message; argc = {}", argc);
   *
   *     return 0;
   *   }
   *
   *
   */
  struct Logging {
    using Options = loguru::Options;       ///< Alias for loguru::Options.
    using FileMode = loguru::FileMode;     ///< Alias for loguru::FileMode.
    using Verbosity = loguru::Verbosity;   ///< Alias for loguru::Verbosity.
    enum NamedVerbosity : Verbosity {
      Verbosity_INVALID = loguru::Verbosity_INVALID,
      Verbosity_OFF = loguru::Verbosity_OFF,
      Verbosity_FATAL = loguru::Verbosity_FATAL,
      Verbosity_ERROR = loguru::Verbosity_ERROR,
      Verbosity_WARNING = loguru::Verbosity_WARNING,
      Verbosity_INFO = loguru::Verbosity_INFO,
      Verbosity_0 = loguru::Verbosity_0,
      Verbosity_1 = loguru::Verbosity_1,
      Verbosity_2 = loguru::Verbosity_2,
      Verbosity_3 = loguru::Verbosity_3,
      Verbosity_4 = loguru::Verbosity_4,
      Verbosity_5 = loguru::Verbosity_5,
      Verbosity_6 = loguru::Verbosity_6,
      Verbosity_7 = loguru::Verbosity_7,
      Verbosity_8 = loguru::Verbosity_8,
      Verbosity_9 = loguru::Verbosity_9,
      Verbosity_MAX = loguru::Verbosity_MAX,
    };

    /**
     * @brief Initializes loguru with command-line arguments and options.
     *
     * @param argc The number of command-line arguments.
     * @param argv An array of command-line arguments.
     * @param options The options to configure loguru (optional).
     */
    static void init(int& argc, char* argv[], const Options& options = {}) { loguru::init(argc, argv, options); }

    /**
     * @brief Adds a file for logging with a specified file mode and verbosity.
     *
     * @param sz_path The path to the log file.
     * @param mode The file mode for logging (e.g., FileMode::Truncate, FileMode::Append).
     * @param verbosity The verbosity level for the file.
     * @return true if the file was successfully added, false otherwise.
     */
    static bool addFile(const char* sz_path, FileMode mode, Verbosity verbosity) {
      return loguru::add_file(sz_path, mode, verbosity);
    }

    /**
     * @brief Logs a message with the specified verbosity level, file, line number, and format
     * string.
     *
     * @tparam Args The types of the variadic arguments.
     * @param verbosity The verbosity level for the log message.
     * @param file The source file name where the log message originated.
     * @param line The line number in the source file where the log message originated.
     * @param format The format string for the log message.
     * @param args The variadic arguments to format the log message.
     */
    template < typename... Args >
    static void log(Verbosity verbosity, const char* file, unsigned line, const char* format, const Args&... args) {
      return loguru::log(verbosity, file, line, format, std::forward< Args >(args)...);
    }

    /**
     * @brief Set the verbosity level
     *
     * @param verbosity The verbosity level to be set
     */
    static void setVerbosity(Verbosity verbosity) { loguru::g_stderr_verbosity = verbosity; }
  };

}   // namespace nt

#endif
