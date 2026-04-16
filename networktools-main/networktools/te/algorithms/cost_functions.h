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
 * @brief Various cost functions for traffic engineering.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_COST_FUNCTIONS_H_
#define _NT_COST_FUNCTIONS_H_

#include <algorithm>
#include <numeric>
#include <cmath>

namespace nt {
  namespace te {

    /**
     * @brief Calculates the Fortz-Thorup arc load cost based on saturation.
     * @param f_sat The current saturation value.
     * @return The Fortz-Thorup arc load cost.
     */
    inline int fortzThorupArcLoadCost(float f_sat) noexcept {
      if (f_sat <= 0.) return 0;
      if (f_sat < 1 / 3.) return 1;
      if (f_sat < 2 / 3.) return 3;
      if (f_sat < 0.9) return 10;
      if (f_sat < 1.) return 70;
      if (f_sat < 1.1) return 500;
      return 5000;
    }

    /**
     * @brief Calculates the Fortz-Thorup arc load cost based on flow and capacity.
     * @param f_flow The current flow value.
     * @param f_capacity The capacity value.
     * @return The Fortz-Thorup arc load cost.
     */
    inline int fortzThorupArcLoadCost(float f_flow, float f_capacity) noexcept {
      if (f_flow <= 0. || f_capacity <= 0.) return 0;
      const float f_sat = f_flow / f_capacity;
      return fortzThorupArcLoadCost(f_sat);
    }

    /**
     * @brief Calculates the exponential arc load cost.
     * @param f_sat The current saturation value.
     * @return The exponential arc load cost.
     */
    inline int expoArcLoadCost(float f_sat) noexcept {
      if (f_sat <= 0.) return 0;
      if (f_sat > 1.) return 5000;
      return 1 << static_cast< int >(f_sat * 10);
    }

    /**
     * @brief Calculates the inverse arc load cost.
     * @param f_sat The current saturation value.
     * @return The inverse arc load cost.
     */
    inline double invArcLoadCost(float f_sat) noexcept {
      if (f_sat <= 0.) return 0;
      if (f_sat >= 1.) return NT_DOUBLE_INFINITY;
      return (1. / (1 - f_sat)) - 1.;
    }

    /**
     * @brief Calculates the inverse arc load cost normalized by a maximum saturation value.
     * @param f_sat The current saturation value.
     * @param f_max The maximum saturation value for normalization.
     * @return The normalized inverse arc load cost.
     */
    inline double invArcLoadCostNormalized(float f_sat, float f_max = 1.f) noexcept {
      if (f_sat <= 0.) return 0;
      const float f_ratio = f_sat / f_max;
      if (f_ratio > 1.) return NT_DOUBLE_INFINITY;
      return (1. / (1 - f_ratio)) - 1.;
    }

    /**
     * @brief Calculates the Jain's fairness index for a given array of values.
     *
     * The formula is: J(x) = (sum(x_i))^2 / (n * sum(x_i^2))
     * The result ranges from 1/n (worst case) to 1 (best case).
     * It is maximum when all users receive the same allocation.
     *
     * @param arr An array view of double values.
     * @return The Jain's fairness index, or 0.0 if the array is empty.
     */
    inline double jainIndex(nt::ArrayView< double > arr) noexcept {
      if (arr.empty()) return 0.;

      double sum = 0.;
      double sum_of_squares = 0.;
      for (double x: arr) {
        sum += x;
        sum_of_squares += x * x;
      }

      // Avoid division by zero
      if (sum_of_squares == 0.) return 0.;

      const double n = static_cast< double >(arr.size());
      return (sum * sum) / (n * sum_of_squares);
    }

    // Helper: Fortz-Thorup cost as double
    inline double fortzThorupArcLoadCostDouble(float f_sat) {
      return static_cast< double >(fortzThorupArcLoadCost(f_sat));
    }

    // Helper: Exponential cost as double
    inline double expoArcLoadCostDouble(float f_sat) { return static_cast< double >(expoArcLoadCost(f_sat)); }

    // Helper: Inverse arc load cost with linear mode from 99%
    inline double invArcLoadCostLinear(float f_sat) {
      if (f_sat <= 0.0f) return 0.0;

      static constexpr float  LINEAR_THRESHOLD = 0.99f;
      static constexpr double COST_AT_THRESHOLD = (1.0 / (1.0 - LINEAR_THRESHOLD)) - 1.0;
      static constexpr double SLOPE_AT_THRESHOLD = 1.0 / ((1.0 - LINEAR_THRESHOLD) * (1.0 - LINEAR_THRESHOLD));

      if (f_sat < LINEAR_THRESHOLD) {
        return (1.0 / (1.0 - f_sat)) - 1.0;
      } else {
        return COST_AT_THRESHOLD + SLOPE_AT_THRESHOLD * (f_sat - LINEAR_THRESHOLD);
      }
    }

    // Helper: normalized Shannon entropy
    inline double calculateNormalizedEntropy(nt::ArrayView< double > values) {
      if (values.empty()) return 0.0;

      double sum = std::accumulate(values.begin(), values.end(), 0.0);
      if (sum == 0.0) return 0.0;

      double entropy = 0.0;
      for (double value: values) {
        double p = value / sum;
        if (p > 0.0) { entropy -= p * std::log2(p); }
      }

      return entropy / std::log2(values.size());
    }

  }   // namespace te
}   // namespace nt

#endif   // _NT_COST_FUNCTIONS_H_