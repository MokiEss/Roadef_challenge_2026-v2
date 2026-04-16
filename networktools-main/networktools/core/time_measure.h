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
 * This file incorporates work from the LEMON graph library (time_measure.h).
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
 *   - Updated include paths to networktools structure
 *   - Added constexpr qualifiers for compile-time optimization
 *   - Updated or enhanced Doxygen documentation
 *   - Updated header guard macros
 *   - Added WallTimer class taken from Google OR-tools
 */

/**
 * @ingroup timecount
 * @file
 * @brief Tools for measuring cpu usage
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_TIME_MEASURE_H_
#define _NT_TIME_MEASURE_H_

#ifdef WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif

#  ifndef NOMINMAX
#    define NOMINMAX
#  endif

#  ifdef UNICODE
#    undef UNICODE
#  endif

#  include <windows.h>

#  ifdef LOCALE_INVARIANT
#    define MY_LOCALE LOCALE_INVARIANT
#  else
#    define MY_LOCALE LOCALE_NEUTRAL
#  endif
#else
#  include <ctime>
#  include <sys/time.h>
#  include <sys/times.h>
#  include <unistd.h>
#endif

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

namespace nt {

  /**
   * @struct WallTimer
   * @brief A wall timer for measuring elapsed time.
   *
   * The WallTimer class provides functionality to measure elapsed time using the wall clock. It
   * can be used to measure the duration between certain points in the code or to track the total
   * time spent on a specific task.
   *
   * Initial version of this code taken from Google OR-tools (https://github.com/google/or-tools)
   *
   * Example of usage:
   *
   * @code
   *   int main() {
   *     WallTimer timer;
   *
   *     // Start the timer
   *     timer.start();
   *
   *     // Perform some time-consuming operation
   *     for (int i = 0; i < 100000000; ++i) {
   *       // Perform computation
   *     }
   *
   *     // Stop the timer
   *     timer.stop();
   *
   *     // Get the elapsed time in seconds
   *     double elapsedSeconds = timer.get();
   *
   *     std::cout << "Elapsed time: " << elapsedSeconds << " seconds" << std::endl;
   *
   *     return 0;
   *   }
   * @endcode
   */
  struct WallTimer {
    bool    _running;
    int64_t _start;
    int64_t _sum;

    /**
     * @brief Default constructor.
     *
     * Constructs a WallTimer object and initializes it with zero elapsed time.
     */
    WallTimer() { reset(); }

    /**
     * @brief Resets the timer to zero elapsed time.
     *
     * This function sets the internal state of the timer to its initial state, with no elapsed
     * time recorded.
     */
    void reset() {
      _running = false;
      _sum = 0;
    }

    /**
     * @brief Starts the timer.
     *
     * This function starts the timer and begins measuring elapsed time. If the timer was already
     * running, the previous start time is discarded, and the new start time is used.
     */
    void start() {
      _running = true;
      _start = _getCurrentTimeNanos();
    }

    /**
     * @brief Restarts the timer.
     *
     * This function resets the timer to zero elapsed time and starts it immediately. It is
     * equivalent to calling `reset()` followed by `start()`.
     */
    void restart() {
      _sum = 0;
      start();
    }

    /**
     * @brief Stops the timer.
     *
     * This function stops the timer and records the elapsed time since the last start. If the
     * timer is not running, this function has no effect.
     */
    void stop() {
      if (_running) {
        _sum += _getCurrentTimeNanos() - _start;
        _running = false;
      }
    }

    /**
     * @brief Returns the elapsed time in seconds.
     *
     * This function returns the total elapsed time in seconds as a floating-point value.
     *
     * @return The elapsed time in seconds.
     */
    double get() const { return _getNanos() * 1e-9; }

    /**
     * @brief Returns the elapsed time in milliseconds.
     *
     * This function returns the total elapsed time in milliseconds as a 64-bit signed integer.
     *
     * @return The elapsed time in milliseconds.
     */
    int64_t getInMs() const { return _getNanos() / 1000000; }

    /**
     * @brief Returns the elapsed time in microseconds.
     *
     * This function returns the total elapsed time in microseconds as a 64-bit signed integer.
     *
     * @return The elapsed time in microseconds.
     */
    int64_t getInUsec() const { return _getNanos() / 1000; }
    // inline absl::Duration GetDuration() const { return absl::Nanoseconds(_getNanos()); }

    // Private methods

    /**
     * @brief Returns the total elapsed time in nanoseconds.
     *
     * This function returns the total elapsed time in nanoseconds as a 64-bit signed integer.
     *
     * @return The elapsed time in nanoseconds.
     */
    int64_t _getNanos() const { return _running ? _getCurrentTimeNanos() - _start + _sum : _sum; }

    /**
     * @brief Returns the current time in nanoseconds.
     *
     * This function retrieves the current time from the system clock and returns it as a 64-bit
     * signed integer representing the time in nanoseconds since the epoch.
     *
     * @return The current time in nanoseconds.
     */
    int64_t _getCurrentTimeNanos() const {
      auto now = std::chrono::system_clock::now();
      return std::chrono::duration_cast< std::chrono::nanoseconds >(now.time_since_epoch()).count();
    }
  };

#if 0
    /**
     * @struct Duration
     * @brief Represents a duration of time.
     *
     * The `Duration` struct is used to represent a duration of time, measured in some unit
     * (e.g., seconds, milliseconds, etc.). It consists of two members: `_rep_hi` and `_rep_lo`,
     * which store the high and low parts of the duration's representation respectively.
     *
     * The `Duration` struct provides static member functions to create special duration values,
     * such as an infinite duration or a zero duration.
     *
     * Initial version of this code taken from Google OR-tools (https://github.com/google/or-tools)
     *
     * Example of usage:
     *
     * @code
     * Duration d = Duration::infinite(); // Represents an infinite duration
     * Duration zero_duration = Duration::zero(); // Represents a zero duration
     * Duration ms = Duratio::milliseconds(1000); // Represents a 1000ms duration
     * @endcode
     *
     * Note: The exact unit and interpretation of the duration is not specified in this struct and
     * should be defined based on the context where the `Duration` is used.
     */
    struct Duration {
      static constexpr int64_t kTicksPerNanosecond = 4;
      static constexpr int64_t kTicksPerSecond = 1000 * 1000 * 1000 * kTicksPerNanosecond;

      int64_t  _rep_hi;   //<< The high part of the duration's representation.
      uint32_t _rep_lo;   //<< The low part of the duration's representation.

      constexpr Duration() : _rep_hi(0), _rep_lo(0) {}
      constexpr Duration(int64_t hi, uint32_t lo) : _rep_hi(hi), _rep_lo(lo) {}

      static constexpr Duration MakeNormalizedDuration(int64_t sec, int64_t ticks) { return (ticks < 0) ? MakeDuration(sec - 1, ticks + kTicksPerSecond) : MakeDuration(sec, ticks); }
      static constexpr Duration MakeDuration(int64_t hi, uint32_t lo = 0) { return Duration(hi, lo); }
      static constexpr Duration MakeDuration(int64_t hi, int64_t lo) { return MakeDuration(hi, static_cast< uint32_t >(lo)); }
      template < std::intmax_t N >
      static constexpr Duration fromInt64(int64_t v, std::ratio< 1, N >) {
        static_assert(0 < N && N <= 1000 * 1000 * 1000, "Unsupported ratio");
        return MakeNormalizedDuration(v / N, v % N * kTicksPerNanosecond * 1000 * 1000 * 1000 / N);
      }

      /**
       * @brief
       *
       * @tparam T
       * @param n
       * @return constexpr Duration
       */
      template < typename T >
      static constexpr Duration milliseconds(T n) noexcept {
        return fromInt64(n, std::milli{});
      }

      /**
       * @brief Creates a duration representing an infinite time.
       *
       * @return The infinite duration.
       */
      static constexpr Duration infinite() noexcept { return {NT_INT64_MAX, ~0U}; }

      /**
       * @brief Creates a duration representing a zero time.
       *
       * @return The zero duration.
       */
      static constexpr Duration zero() noexcept { return {0, 0}; }

      /**
       * @brief
       *
       * @return constexpr int64_t
       */
      constexpr int64_t toMilliseconds() const noexcept {
        if (_rep_hi >= 0 && _rep_hi >> 53 == 0) return (_rep_hi * 1000) + (_rep_lo / (kTicksPerNanosecond * 1000 * 1000));
        return (*this) / fromInt64(1, std::milli{});
      }

      // private
    };

    // Relational Operators
    constexpr bool operator<(Duration lhs, Duration rhs) {
      return lhs._rep_hi != rhs._rep_hi ? lhs._rep_hi < rhs._rep_hi : lhs._rep_hi == (std::numeric_limits< int64_t >::min)() ? lhs._rep_lo + 1 < rhs._rep_lo + 1 : lhs._rep_lo < rhs._rep_lo;
    }
    constexpr bool operator>(Duration lhs, Duration rhs) { return rhs < lhs; }
    constexpr bool operator>=(Duration lhs, Duration rhs) { return !(lhs < rhs); }
    constexpr bool operator<=(Duration lhs, Duration rhs) { return !(rhs < lhs); }
    constexpr bool operator==(Duration lhs, Duration rhs) { return lhs._rep_hi == rhs._rep_hi && lhs._rep_lo == rhs._rep_lo; }
    constexpr bool operator!=(Duration lhs, Duration rhs) { return !(lhs == rhs); }

    inline int64_t operator/(Duration lhs, Duration rhs) { return IDivDuration(lhs, rhs, &lhs); }
#endif

  /**
   * \addtogroup timecount
   * @{
   */

  /** A class to store (cpu)time instances. */

  /**
   * This class stores five time values.
   * - a real time
   * - a user cpu time
   * - a system cpu time
   * - a user cpu time of children
   * - a system cpu time of children
   *
   * TimeStamp's can be added to or substracted from each other and
   * they can be pushed to a stream.
   *
   * In most cases, perhaps the @ref Timer or the @ref TimeReport
   * class is what you want to use instead.
   */

  class TimeStamp {
    double utime;
    double stime;
    double cutime;
    double cstime;
    double rtime;

    public:
    /** Display format specifier */

    /**
     * \e
     *
     */
    enum Format {
      /** Reports all measured values */
      NORMAL = 0,
      /** Only real time and an error indicator is displayed */
      SHORT = 1
    };

    private:
    constexpr static Format _format = TimeStamp::NORMAL;

    constexpr void _reset() noexcept { utime = stime = cutime = cstime = rtime = 0; }

    public:
    /** Set output format */

    /**
     * Set output format.
     *
     * The output format is global for all timestamp instances.
     */
    // static void format(Format f) { _format = f; }
    /** Retrieve the current output format */

    /**
     * Retrieve the current output format
     *
     * The output format is global for all timestamp instances.
     */
    static constexpr Format format() noexcept { return _format; }

    /** Read the current time values of the process */
    void stamp() noexcept {
#ifndef WIN32
      timeval tv;
      gettimeofday(&tv, 0);
      rtime = tv.tv_sec + double(tv.tv_usec) / 1e6;

      tms    ts;
      double tck = sysconf(_SC_CLK_TCK);
      times(&ts);
      utime = ts.tms_utime / tck;
      stime = ts.tms_stime / tck;
      cutime = ts.tms_cutime / tck;
      cstime = ts.tms_cstime / tck;
#else
      static const double ch = 4294967296.0e-7;
      static const double cl = 1.0e-7;

      FILETIME system;
      GetSystemTimeAsFileTime(&system);
      rtime = ch * system.dwHighDateTime + cl * system.dwLowDateTime;

      FILETIME create, exit, kernel, user;
      if (GetProcessTimes(GetCurrentProcess(), &create, &exit, &kernel, &user)) {
        utime = ch * user.dwHighDateTime + cl * user.dwLowDateTime;
        stime = ch * kernel.dwHighDateTime + cl * kernel.dwLowDateTime;
        cutime = 0;
        cstime = 0;
      } else {
        rtime = 0;
        utime = 0;
        stime = 0;
        cutime = 0;
        cstime = 0;
      }
#endif
    }

    /** Constructor initializing with zero */
    constexpr TimeStamp() noexcept { _reset(); }
    /**
     * Constructor initializing with the current time values of the process
     */
    TimeStamp(void*) noexcept { stamp(); }

    /** Set every time value to zero */
    constexpr TimeStamp& reset() noexcept {
      _reset();
      return *this;
    }


    constexpr TimeStamp& operator+=(const TimeStamp& b) noexcept {
      utime += b.utime;
      stime += b.stime;
      cutime += b.cutime;
      cstime += b.cstime;
      rtime += b.rtime;
      return *this;
    }

    constexpr TimeStamp operator+(const TimeStamp& b) const noexcept {
      TimeStamp t(*this);
      return t += b;
    }

    constexpr TimeStamp& operator-=(const TimeStamp& b) noexcept {
      utime -= b.utime;
      stime -= b.stime;
      cutime -= b.cutime;
      cstime -= b.cstime;
      rtime -= b.rtime;
      return *this;
    }

    constexpr TimeStamp operator-(const TimeStamp& b) const noexcept {
      TimeStamp t(*this);
      return t -= b;
    }

    constexpr TimeStamp& operator*=(double b) noexcept {
      utime *= b;
      stime *= b;
      cutime *= b;
      cstime *= b;
      rtime *= b;
      return *this;
    }

    constexpr TimeStamp operator*(double b) const noexcept {
      TimeStamp t(*this);
      return t *= b;
    }
    friend constexpr TimeStamp operator*(double b, const TimeStamp& t) noexcept;
    constexpr TimeStamp&       operator/=(double b) noexcept {
            utime /= b;
            stime /= b;
            cutime /= b;
            cstime /= b;
            rtime /= b;
            return *this;
    }

    constexpr TimeStamp operator/(double b) const noexcept {
      TimeStamp t(*this);
      return t /= b;
    }
    /** The time ellapsed since the last call of stamp() */
    TimeStamp ellapsed() const noexcept {
      TimeStamp t(NULL);
      return t - *this;
    }

    friend std::ostream& operator<<(std::ostream& os, const TimeStamp& t);

    /** Gives back the user time of the process */
    double userTime() const { return utime; }
    /** Gives back the system time of the process */
    double systemTime() const { return stime; }
    /** Gives back the user time of the process' children */

    /**
     * @note On <tt>WIN32</tt> platform this value is not calculated.
     *
     */
    constexpr double cUserTime() const noexcept { return cutime; }
    /** Gives back the user time of the process' children */

    /**
     * @note On <tt>WIN32</tt> platform this value is not calculated.
     *
     */
    constexpr double cSystemTime() const noexcept { return cstime; }
    /** Gives back the real time */
    constexpr double realTime() const noexcept { return rtime; }
  };

  inline constexpr TimeStamp operator*(double b, const TimeStamp& t) noexcept { return t * b; }

  /** Prints the time counters */

  /**
   * Prints the time counters in the following form:
   *
   * <tt>u: XX.XXs s: XX.XXs cu: XX.XXs cs: XX.XXs real: XX.XXs</tt>
   *
   * where the values are the
   * \li \c u: user cpu time,
   * \li \c s: system cpu time,
   * \li \c cu: user cpu time of children,
   * \li \c cs: system cpu time of children,
   * \li \c real: real time.
   * \relates TimeStamp
   * @note On <tt>WIN32</tt> platform the cummulative values are not
   * calculated.
   */
  inline std::ostream& operator<<(std::ostream& os, const TimeStamp& t) {
    switch (t._format) {
      case TimeStamp::NORMAL:
        os << "u: " << t.userTime() << "s, s: " << t.systemTime() << "s, cu: " << t.cUserTime()
           << "s, cs: " << t.cSystemTime() << "s, real: " << t.realTime() << "s";
        break;
      case TimeStamp::SHORT:
        double total = t.userTime() + t.systemTime() + t.cUserTime() + t.cSystemTime();
        os << t.realTime() << "s (err: " << round((t.realTime() - total) / t.realTime() * 10000) / 100 << "%)";
        break;
    }
    return os;
  }

  /**
   * Class for measuring the cpu time and real time usage of the process
   */

  /**
   * Class for measuring the cpu time and real time usage of the process.
   * It is quite easy-to-use, here is a short example.
   * @code
   * #include"time_measure.h"
   * #include<iostream>
   *
   * int main()
   * {
   *
   * ...
   *
   * Timer t;
   * doSomething();
   * std::cout << t << '\n';
   * t.restart();
   * doSomethingElse();
   * std::cout << t << '\n';
   *
   * ...
   *
   * }
   * @endcode
   *
   * The @ref Timer can also be @ref stop() "stopped" and
   * @ref start() "started" again, so it is possible to compute collected
   * running times.
   *
   * @warning Depending on the operation system and its actual configuration
   * the time counters have a certain (10ms on a typical Linux system)
   * granularity.
   * Therefore this tool is not appropriate to measure very short times.
   * Also, if you start and stop the timer very frequently, it could lead to
   * distorted results.
   *
   * @note If you want to measure the running time of the execution of a certain
   * function, consider the usage of @ref TimeReport instead.
   *
   * \sa TimeReport
   */
  class Timer {
    int       _running;     // Timer is running iff _running>0; (_running>=0 always holds)
    TimeStamp start_time;   // This is the relativ start-time if the timer
                            // is _running, the collected _running time otherwise.

    constexpr void _reset() noexcept {
      if (_running)
        start_time.stamp();
      else
        start_time.reset();
    }

    public:
    /** Constructor. */

    /**
     * @param run indicates whether or not the timer starts immediately.
     *
     */
    constexpr Timer(bool run = true) noexcept : _running(run) { _reset(); }

    /**
     * \name Control the State of the Timer
     * Basically a Timer can be either running or stopped,
     * but it provides a bit finer control on the execution.
     * The @ref lemon::Timer "Timer" also counts the number of
     * @ref lemon::Timer::start() "start()" executions, and it stops
     * only after the same amount (or more) @ref lemon::Timer::stop()
     * "stop()"s. This can be useful e.g. to compute the running time
     * of recursive functions.
     */

    /** @{ */

    /** Reset and stop the time counters */

    /**
     * This function resets and stops the time counters
     * \sa restart()
     */
    constexpr void reset() noexcept {
      _running = 0;
      _reset();
    }

    /** Start the time counters */

    /**
     * This function starts the time counters.
     *
     * If the timer is started more than ones, it will remain running
     * until the same amount of @ref stop() is called.
     * \sa stop()
     */
    constexpr void start() noexcept {
      if (_running)
        _running++;
      else {
        _running = 1;
        TimeStamp t;
        t.stamp();
        start_time = t - start_time;
      }
    }

    /** Stop the time counters */

    /**
     * This function stops the time counters. If start() was executed more than
     * once, then the same number of stop() execution is necessary the really
     * stop the timer.
     *
     * \sa halt()
     * \sa start()
     * \sa restart()
     * \sa reset()
     */

    constexpr void stop() noexcept {
      if (_running && !--_running) {
        TimeStamp t;
        t.stamp();
        start_time = t - start_time;
      }
    }

    /** Halt (i.e stop immediately) the time counters */

    /**
     * This function stops immediately the time counters, i.e. <tt>t.halt()</tt>
     * is a faster
     * equivalent of the following.
     * @code
     * while(t.running()) t.stop()
     * @endcode
     *
     *
     * \sa stop()
     * \sa restart()
     * \sa reset()
     */

    constexpr void halt() noexcept {
      if (_running) {
        _running = 0;
        TimeStamp t;
        t.stamp();
        start_time = t - start_time;
      }
    }

    /** Returns the running state of the timer */

    /**
     * This function returns the number of stop() exections that is
     * necessary to really stop the timer.
     * For example, the timer
     * is running if and only if the return value is \c true
     * (i.e. greater than
     * zero).
     */
    constexpr int running() const noexcept { return _running; }

    /** Restart the time counters */

    /**
     * This function is a shorthand for
     * a reset() and a start() calls.
     *
     */
    constexpr void restart() noexcept {
      reset();
      start();
    }

    /** @} */

    /** \name Query Functions for the Ellapsed Time */

    /** @{ */

    /** Gives back the ellapsed user time of the process */
    double userTime() const noexcept { return operator TimeStamp().userTime(); }
    /** Gives back the ellapsed system time of the process */
    double systemTime() const noexcept { return operator TimeStamp().systemTime(); }
    /** Gives back the ellapsed user time of the process' children */

    /**
     * @note On <tt>WIN32</tt> platform this value is not calculated.
     *
     */
    double cUserTime() const noexcept { return operator TimeStamp().cUserTime(); }
    /** Gives back the ellapsed user time of the process' children */

    /**
     * @note On <tt>WIN32</tt> platform this value is not calculated.
     *
     */
    double cSystemTime() const noexcept { return operator TimeStamp().cSystemTime(); }
    /** Gives back the ellapsed real time */
    double realTime() const noexcept { return operator TimeStamp().realTime(); }
    /** Computes the ellapsed time */

    /**
     * This conversion computes the ellapsed time, therefore you can print
     * the ellapsed time like this.
     * @code
     * Timer t;
     * doSomething();
     * std::cout << t << '\n';
     * @endcode
     */
    operator TimeStamp() const noexcept {
      TimeStamp t;
      t.stamp();
      return _running ? t - start_time : start_time;
    }

    /** @} */
  };

  /** Same as Timer but prints a report on destruction. */

  /**
   * Same as @ref Timer but prints a report on destruction.
   * This example shows its usage.
   * @code
   * void myAlg(ListGraph &g,int n)
   * {
   * TimeReport tr("Running time of myAlg: ");
   * ... //Here comes the algorithm
   * }
   * @endcode
   *
   * \sa Timer
   * \sa NoTimeReport
   */
  class TimeReport: public Timer {
    std::string   _title;
    std::ostream& _os;
    bool          _active;

    public:
    /** Constructor */

    /**
     * Constructor.
     * @param title This text will be printed before the ellapsed time.
     * @param os The stream to print the report to.
     * @param run Sets whether the timer should start immediately.
     * @param active Sets whether the report should actually be printed
     * on destruction.
     */
    TimeReport(std::string title, std::ostream& os = std::cerr, bool run = true, bool active = true) noexcept :
        Timer(run), _title(title), _os(os), _active(active) {}
    /** Destructor that prints the ellapsed time */
    ~TimeReport() noexcept {
      if (_active) _os << _title << *this << std::endl;
    }

    /** Retrieve the activity status */

    /**
     * \e
     *
     */
    constexpr bool active() const noexcept { return _active; }
    /** Set the activity status */

    /**
     * This function set whether the time report should actually be printed
     * on destruction.
     */
    constexpr void active(bool a) noexcept { _active = a; }
  };

  /** 'Do nothing' version of TimeReport */

  /**
   * \sa TimeReport
   *
   */
  class NoTimeReport {
    public:
    NoTimeReport(std::string, std::ostream&, bool) noexcept {}

    NoTimeReport(std::string, std::ostream&) noexcept {}

    NoTimeReport(std::string) noexcept {}
    /** \e Do nothing. */
    ~NoTimeReport() noexcept {}

    operator TimeStamp() const noexcept { return TimeStamp(); }

    constexpr void   reset() noexcept {}
    constexpr void   start() noexcept {}
    constexpr void   stop() noexcept {}
    constexpr void   halt() noexcept {}
    constexpr int    running() noexcept { return 0; }
    constexpr void   restart() noexcept {}
    constexpr double userTime() const noexcept { return 0; }
    constexpr double systemTime() const noexcept { return 0; }
    constexpr double cUserTime() const noexcept { return 0; }
    constexpr double cSystemTime() const noexcept { return 0; }
    constexpr double realTime() const noexcept { return 0; }
  };

  /** Tool to measure the running time more exactly. */

  /**
   * This function calls \c f several times and returns the average
   * running time. The number of the executions will be choosen in such a way
   * that the full real running time will be roughly between \c min_time
   * and <tt>2*min_time</tt>.
   * @param f the function object to be measured.
   * @param min_time the minimum total running time.
   * \retval num if it is not \c NULL, then the actual
   * number of execution of \c f will be written into <tt>*num</tt>.
   * \retval full_time if it is not \c NULL, then the actual
   * total running time will be written into <tt>*full_time</tt>.
   * @return The average running time of \c f.
   */

  template < class F >
  TimeStamp runningTimeTest(F f, double min_time = 10, unsigned int* num = NULL, TimeStamp* full_time = NULL) {
    TimeStamp    full;
    unsigned int total = 0;
    Timer        t;
    for (unsigned int tn = 1; tn <= 1U << 31 && full.realTime() <= min_time; tn *= 2) {
      for (; total < tn; total++)
        f();
      full = t;
    }
    if (num) *num = total;
    if (full_time) *full_time = full;
    return full / total;
  }

  /** @} */

}   // namespace nt

#endif   // _NT_TIME_MEASURE_H_
