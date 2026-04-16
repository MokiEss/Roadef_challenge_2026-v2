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
 * @brief
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_MP_SOLVER_BASE_H_
#define _NT_MP_SOLVER_BASE_H_

#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <optional>
#include <cmath>
#include <cstddef>

#if !defined(_MSC_VER)
#  include <unistd.h>
#endif

#if _WIN32
#  undef CALLBACK
#endif

#include "../../ntconfig.h"
#include "../../core/format.h"
#include "../../core/logging.h"
#include "../../core/arrays.h"
#include "../../core/tuple.h"
#include "../../core/maps.h"
#include "../../core/sort.h"
#include "../../core/json.h"
#include "../../core/time_measure.h"
#include "../../core/tolerance.h"

#include "../mp.h"

namespace nt {
  namespace mp {
    namespace details {

      template < int DIM, typename MPVariable >
      struct MPMultiDimVariable: nt::TrivialMultiDimArray< MPVariable*, DIM > {
        using Parent = nt::TrivialMultiDimArray< MPVariable*, DIM >;
      };

      template < typename MPVariable >
      struct MPMultiDimVariable< 1, MPVariable >: nt::TrivialDynamicArray< MPVariable* > {
        using Parent = nt::TrivialDynamicArray< MPVariable* >;
        constexpr const typename Parent::value_type& operator()(int k) const noexcept { return (*this)[k]; }
        constexpr typename Parent::value_type&       operator()(int k) noexcept { return (*this)[k]; }
      };

      // TODO(Morgan) : implement the equivalent of MPMultiDimVariable for constraints (e.g useful for McfPath)
      // template < int DIM, typename MPConstraint >
      // struct MPMultiDimConstraint: nt::TrivialMultiDimArray< MPConstraint*, DIM > {
      //   using Parent = nt::TrivialMultiDimArray< MPConstraint*, DIM >;
      // };

      // template < typename MPConstraint >
      // struct MPMultiDimConstraint< 1, MPConstraint >: nt::TrivialDynamicArray< MPConstraint* > {
      //   using Parent = nt::TrivialDynamicArray< MPConstraint* >;
      //   constexpr const typename Parent::value_type& operator()(int k) const noexcept { return (*this)[k]; }
      //   constexpr typename Parent::value_type&       operator()(int k) noexcept { return (*this)[k]; }
      // };

      template < typename FloatType >
      inline bool IsPositiveOrNegativeInfinity(FloatType x) {
        return x == std::numeric_limits< FloatType >::infinity() || x == -std::numeric_limits< FloatType >::infinity();
      }

      // Tests whether x and y are close to one another using absolute and relative
      // tolerances.
      // Returns true if |x - y| <= a (with a being the absolute_tolerance).
      // The above case is useful for values that are close to zero.
      // Returns true if |x - y| <= max(|x|, |y|) * r. (with r being the relative
      //                                                tolerance.)
      // The cases for infinities are treated separately to avoid generating NaNs.
      template < typename FloatType >
      inline bool AreWithinAbsoluteOrRelativeTolerances(FloatType x,
                                                        FloatType y,
                                                        FloatType relative_tolerance,
                                                        FloatType absolute_tolerance) {
        DCHECK_LE_F(0.0, relative_tolerance);
        DCHECK_LE_F(0.0, absolute_tolerance);
        DCHECK_GT_F(1.0, relative_tolerance);
        if (IsPositiveOrNegativeInfinity(x) || IsPositiveOrNegativeInfinity(y)) { return x == y; }
        const FloatType difference = fabs(x - y);
        if (difference <= absolute_tolerance) { return true; }
        const FloatType largest_magnitude = std::max(fabs(x), fabs(y));
        return difference <= largest_magnitude * relative_tolerance;
      }

      /**
       * @brief Mathematical Programming (MP) solver
       *
       * Main interface for building and solving LP/MIP optimization problems.
       *
       * @tparam MPSolverInterface Solver backend implementation
       *
       * @par Basic Usage:
       * @code
       * MPSolver solver("my_problem");
       * // Create variables
       * auto x = solver.makeNumVar(0, 10, "x");
       * auto y = solver.makeIntVar(0, 5, "y");
       *
       * // Add constraints
       * auto c1 = solver.makeRowConstraint(0, 15, "c1");
       * c1->setCoefficient(x, 2);
       * c1->setCoefficient(y, 1);
       *
       * // Set objective
       * auto obj = solver.mutableObjective();
       * obj->setCoefficient(x, 3);
       * obj->setCoefficient(y, 2);
       * obj->setMaximization();
       *
       * // solve
       * solver.solve();
       * std::cout << "Optimal value: " << solver.objective_value() << "\n";
       * std::cout << "x = " << x->solution_value() << "\n";
       * std::cout << "y = " << y->solution_value() << "\n";
       * @endcode
       *
       * @par Model Building:
       * - Variables: makeVar(), makeNumVar(), makeIntVar(), makeBoolVar()
       * - Constraints: makeRowConstraint()
       * - objective: mutableObjective()
       *
       * @par Solving:
       * - Configure: setParameters(), setTimeLimit()
       * - solve: solve()
       * - Query: result_status(), objective_value(), best_objective_bound()
       *
       * @see MPVariable, MPConstraint, MPObjective
       */
      template < typename MPSolverInterface >
      struct MPSolver {
        template < typename TYPE >
        struct MemoryManager {
          nt::FixedMemoryArray< TYPE, 19 > _library;

          template < typename... Args >
          constexpr TYPE* create(Args&&... args) noexcept {
            return &_library[_library.add(std::forward< Args >(args)...)];
          }

          const void     reserve(int i_size) noexcept { _library.reserve(i_size); }
          constexpr void clear() noexcept { _library.removeAll(); }
          constexpr int  size() const noexcept { return _library.size(); }
        };

        /**
         * @brief Decision variable in an optimization model
         *
         * Represents a decision variable with bounds, integrality constraints,
         * and solution information. Variables belong to exactly one MPSolver instance.
         */
        struct MPVariable {
          const int         _index;
          double            _lb;
          double            _ub;
          bool              _integer;
          const std::string _name;
          double            _solution_value;
          double            _reduced_cost;
          int               _branching_priority;
          MPSolver&         _solver;

          /**
           * @brief Construct a decision variable
           * @param index Variable index in solver
           * @param lb Lower bound
           * @param ub Upper bound
           * @param integer True if integer variable, false if continuous
           * @param name Variable name (auto-generated if empty)
           * @param solver Parent solver instance
           */
          MPVariable(int index, double lb, double ub, bool integer, const std::string& name, MPSolver& solver) :
              _index(index), _lb(lb), _ub(ub), _integer(integer),
              _name(name.empty() ? nt::format("auto_v_%09d", index) : name), _solution_value(0.0), _reduced_cost(0.0),
              _branching_priority(0), _solver(solver) {}

          MPVariable(const MPVariable&) = delete;
          MPVariable& operator=(const MPVariable&) = delete;
          MPVariable(MPVariable&& other) = default;
          MPVariable& operator=(MPVariable&& other) = default;

          /**
           * @brief Get variable name
           * @return Variable name (auto-generated if not provided at construction)
           */
          const std::string& name() const { return _name; }

          /**
           * @brief Set integrality requirement
           * @param integer True for integer variable, false for continuous
           */
          void setInteger(bool integer) {
            if (_integer != integer) {
              _integer = integer;
              if (_solver.variable_is_extracted(_index)) {
                _solver.getInterface()->_impl_SetVariableInteger(_index, integer);
              }
            }
          }

          /**
           * @brief Check if variable is integer
           * @return True if integer variable, false if continuous
           */
          bool integer() const { return _integer; }

          /**
           * @brief Solution value for this variable
           *
           * @return Variable value in current solution (rounded for integer vars)
           *
           * Call after MPSolver::solve(). Integer variables automatically
           * rounded to nearest integer.
           */
          double solution_value() const {
            if (!_solver.checkSolutionIsSynchronizedAndExists()) return 0.0;
            // If the underlying solver supports integer variables, and this is an integer
            // variable, we round the solution value (i.e., clients usually expect precise
            // integer values for integer variables).
            return (_integer && _solver.isMIP()) ? round(_solution_value) : _solution_value;
          }

          /**
           * @brief Variable index in solver
           * @return Index in MPSolver::_variables vector
           */
          int index() const { return _index; }

          /** @brief Lower bound */
          double lb() const { return _lb; }

          /** @brief Upper bound */
          double ub() const { return _ub; }

          /**
           * @brief Set lower bound
           * @param lb New lower bound
           */
          void setLB(double lb) { setBounds(lb, _ub); }

          /**
           * @brief Set upper bound
           * @param ub New upper bound
           */
          void setUB(double ub) { setBounds(_lb, ub); }

          /**
           * @brief Set both bounds
           * @param lb Lower bound
           * @param ub Upper bound
           */
          void setBounds(double lb, double ub) {
            const bool change = lb != _lb || ub != _ub;
            _lb = lb;
            _ub = ub;
            if (change && _solver.variable_is_extracted(_index)) {
              _solver.getInterface()->_impl_SetVariableBounds(_index, _lb, _ub);
            }
          }

          /**
           * @brief Unrounded solution value
           *
           * @return Exact solver value without rounding (even for integer vars)
           *
           * @warning Advanced usage. Use solution_value() for normal cases.
           */
          double unrounded_solution_value() const {
            if (!_solver.checkSolutionIsSynchronizedAndExists()) return 0.0;
            return _solution_value;
          }

          /**
           * @brief Reduced cost at current solution
           *
           * @return Reduced cost (LP only, not for MIP)
           *
           * @warning Advanced usage. Only for continuous problems.
           */
          double reduced_cost() const {
            if (!_solver.isContinuous()) {
              LOG_F(FATAL, "Reduced cost only available for continuous problems");
              return 0.0;
            }
            if (!_solver.checkSolutionIsSynchronizedAndExists()) return 0.0;
            return _reduced_cost;
          }

          /**
           * @brief Basis status at current solution
           *
           * @return Basis status (LP only, not for MIP)
           *
           * @see BasisStatus
           * @warning Advanced usage. Only for continuous problems.
           */
          BasisStatus basis_status() const {
            if (!_solver.isContinuous()) {
              LOG_F(FATAL, "Basis status only available for continuous problems");
              return BasisStatus::FREE;
            }
            if (!_solver.checkSolutionIsSynchronizedAndExists()) { return BasisStatus::FREE; }
            // This is done lazily as this method is expected to be rarely used.
            return _solver.getInterface()->_impl_column_status(_index);
          }

          /**
           * @brief Branching priority for MIP
           *
           * @return Priority (0 = default, higher = prioritized)
           *
           * @warning Advanced MIP usage. Supported by Gurobi, SCIP.
           * Ignored by other solvers.
           */
          int branching_priority() const { return _branching_priority; }

          /**
           * @brief Set branching priority
           * @param priority Priority value (0 = default)
           */
          void setBranchingPriority(int priority) {
            if (priority == _branching_priority) return;
            _branching_priority = priority;
            _solver.getInterface()->_impl_BranchingPriorityChangedForVariable(_index);
          }

          // protected:
          void set_solution_value(double value) { _solution_value = value; }
          void set_reduced_cost(double reduced_cost) { _reduced_cost = reduced_cost; }
        };

        template < int DIM >
        using MPMultiDimVariable = details::MPMultiDimVariable< DIM, MPVariable >;

        // template < int DIM >
        // struct MPMultiDimVariable: nt::TrivialMultiDimArray< MPVariable*, DIM > {
        //   using Parent = nt::TrivialMultiDimArray< MPVariable*, DIM >;
        // };

        // template <>
        // struct MPMultiDimVariable< 1 >: nt::TrivialDynamicArray< MPVariable* > {
        //   using Parent = nt::TrivialDynamicArray< MPVariable* >;
        // };

        template < class GR, class... ELEMENTS >
        struct MPGraphVariable: GR::template StaticGraphMap< MPVariable*, ELEMENTS... > {
          using Parent = typename GR::template StaticGraphMap< MPVariable*, ELEMENTS... >;
          using Digraph = typename Parent::Digraph;
          using Parent::Parent;
        };

        /**
         * @brief Linear expression builder for optimization models
         *
         * Provides syntactic sugar for building linear expressions naturally, like
         * mathematical equations. This is an overlay on the MPSolver API with no
         * additional functionality, optimized for both simplicity and big-O efficiency.
         *
         * @par Key Classes:
         * - LinearExpr: Represents `offset + sum_i(a_i * x_i)`
         * - LinearRange: Represents `lb <= sum_i(a_i * x_i) <= ub`
         *
         * @par Recommended Usage:
         * @code
         * MPSolver solver = ...;
         * const LinearExpr x = solver.makeVar(...);  // Note: implicit conversion
         * const LinearExpr y = solver.makeVar(...);
         * const LinearExpr z = solver.makeVar(...);
         * const LinearExpr e1 = x + y;
         * const LinearExpr e2 = (e1 + 7.0 + z) / 3.0;
         * const LinearRange r = e1 <= e2;
         * solver.makeRowConstraint(r);
         * @endcode
         *
         * @warning Pointer Arithmetic Trap
         *
         * WRONG (pointer arithmetic instead of expression building):
         * @code
         * MPVariable* x = solver.makeVar(...);
         * LinearExpr y = x + 5;  // BAD: treats x as pointer!
         * @endcode
         *
         * @par Best Practices:
         * 1. Use double literals, not ints (e.g., `x + 5.0`, not `x + 5`)
         * 2. Immediately convert MPVariable* to LinearExpr:
         *    `const LinearExpr x = solver.makeVar(...);`
         * 3. Hold references to LinearExpr, not raw MPVariable pointers
         *
         * @par Non-Associative Trap:
         *
         * While `LinearExpr(x) + y + 5` is valid, it's fragile because
         * reordering like `y + 5 + LinearExpr(x)` fails (pointer arithmetic).
         * Always wrap variables in LinearExpr immediately to avoid this.
         */

        /**
         * @brief Linear expression in decision variables
         *
         * Models: `offset + sum_i(a_i * x_i)` where x_i are MPVariables
         *
         * offset + sum_{i in S} a_i*x_i,
         *
         * where the a_i and offset are constants and the x_i are MPVariables. You can
         * use a LinearExpr "linear_expr" with an MPSolver "solver" to:
         *   * Set as the objective of your optimization problem, e.g.
         *
         *     solver.mutableObjective()->maximizeLinearExpr(linear_expr);
         *
         *   * Create a constraint in your optimization, e.g.
         *
         *     solver.makeRowConstraint(linear_expr1 <= linear_expr2);
         *
         *   * Get the value of the quantity after solving, e.g.
         *
         *     solver.solve();
         *     linear_expr.solutionValue();
         *
         * LinearExpr is allowed to delete variables with coefficient zero from the map,
         * but is not obligated to do so.
         */
        struct LinearExpr {
          // public:
          LinearExpr() : LinearExpr(0.0) {}
          /** Possible implicit conversions are intentional. */
          LinearExpr(double constant) : _offset(constant), _terms() {}   // NOLINT

          /***
           * Possible implicit conversions are intentional.
           *
           * Warning: var is not owned.
           */
          LinearExpr(const MPVariable* var) : LinearExpr(0.0) { _terms[var] = 1.0; }   // NOLINT

          /**
           * Returns 1-var.
           *
           * NOTE(user): if var is binary variable, this corresponds to the logical
           * negation of var.
           * Passing by value is intentional, see the discussion on binary ops.
           */
          static LinearExpr notVar(LinearExpr var) {
            var *= -1;
            var += 1;
            return var;
          }

          LinearExpr& operator+=(const LinearExpr& rhs) {
            for (const auto& kv: rhs._terms) {
              _terms[kv.first] += kv.second;
            }
            _offset += rhs._offset;
            return *this;
          }

          LinearExpr& operator-=(const LinearExpr& rhs) {
            for (const auto& kv: rhs._terms) {
              _terms[kv.first] -= kv.second;
            }
            _offset -= rhs._offset;
            return *this;
          }

          LinearExpr& operator*=(double rhs) {
            if (rhs == 0) {
              _terms.clear();
              _offset = 0;
            } else if (rhs != 1) {
              for (auto& kv: _terms) {
                kv.second *= rhs;
              }
              _offset *= rhs;
            }
            return *this;
          }

          LinearExpr& operator/=(double rhs) {
            DCHECK_NE_F(rhs, 0);
            return (*this) *= 1 / rhs;
          }

          LinearExpr operator-() const { return (*this) * -1; }

          friend std::ostream& operator<<(std::ostream& stream, const LinearExpr& linear_expr) {
            stream << linear_expr.toString();
            return stream;
          }

          // NOTE(user): in the ops below, the non-"const LinearExpr&" are intentional.
          // We need to create a new LinearExpr for the result, so we lose nothing by
          // passing one argument by value, mutating it, and then returning it. In
          // particular, this allows (with move semantics and RVO) an optimized
          // evaluation of expressions such as
          // a + b + c + d
          // (see http://en.cppreference.com/w/cpp/language/operators).

          friend LinearExpr operator+(LinearExpr lhs, const LinearExpr& rhs) {
            lhs += rhs;
            return lhs;
          }

          friend LinearExpr operator-(LinearExpr lhs, const LinearExpr& rhs) {
            lhs -= rhs;
            return lhs;
          }

          friend LinearExpr operator*(LinearExpr lhs, double rhs) {
            lhs *= rhs;
            return lhs;
          }

          friend LinearExpr operator/(LinearExpr lhs, double rhs) {
            lhs /= rhs;
            return lhs;
          }

          friend LinearExpr operator*(double lhs, LinearExpr rhs) {
            rhs *= lhs;
            return rhs;
          }

          // TODO(user,user): explore defining more overloads to support:
          // solver.makeRowConstraint(0.0 <= x + y + z <= 1.0);

          double                                             offset() const { return _offset; }
          const nt::PointerMap< const MPVariable*, double >& terms() const { return _terms; }

          /**
           * Evaluates the value of this expression at the solution found.
           *
           * It must be called only after calling MPSolver::solve.
           */
          double solutionValue() const {
            double solution = _offset;
            for (const auto& pair: _terms) {
              solution += pair.first->solution_value() * pair.second;
            }
            return solution;
          }

          /**
           * A human readable representation of this. Variables will be printed in order
           * of lowest index first.
           */
          std::string toString() const {
            nt::MemoryBuffer buffer;
            const bool       is_first = appendAllTerms(_terms, &buffer);
            appendOffset(_offset, is_first, &buffer);
            // TODO(user): support optionally cropping long strings.
            return nt::to_string(buffer);
          }

          // private:
          double                                      _offset;
          nt::PointerMap< const MPVariable*, double > _terms;
        };

        /**
         * @brief Range constraint expression
         *
         * Represents: `lower_bound <= sum_i(a_i*x_i) <= upper_bound`
         *
         * Created by comparison operators:
         * @code
         * auto range1 = x + y <= 10;  // upper bound only
         * auto range2 = x + y == 5;   // equality
         * auto range3 = x + y >= 3;   // lower bound only
         * @endcode
         *
         * Add to model with MPSolver::makeRowConstraint(range, name).
         */
        struct LinearRange {
          /** @brief Default constructor (0 <= 0 <= 0) */
          LinearRange() : _lower_bound(0), _upper_bound(0) {}

          /**
           * @brief Constructs bounded range
           *
           * @param lower_bound Lower bound
           * @param linear_expr Linear expression
           * @param upper_bound Upper bound
           *
           * Automatically adjusts bounds to absorb expression offset.
           */
          LinearRange(double lower_bound, const LinearExpr& linear_expr, double upper_bound) :
              _lower_bound(lower_bound), _linear_expr(linear_expr), _upper_bound(upper_bound) {
            _lower_bound -= _linear_expr.offset();
            _upper_bound -= _linear_expr.offset();
            _linear_expr -= _linear_expr.offset();
          }

          /** @brief Lower bound of range */
          double lower_bound() const { return _lower_bound; }
          /** @brief Linear expression */
          const LinearExpr& linear_expr() const { return _linear_expr; }
          /** @brief Upper bound of range */
          double upper_bound() const { return _upper_bound; }

          friend LinearRange operator<=(const LinearExpr& lhs, const LinearExpr& rhs) {
            return LinearRange(-NT_DOUBLE_INFINITY, lhs - rhs, 0);
          }
          friend LinearRange operator==(const LinearExpr& lhs, const LinearExpr& rhs) {
            return LinearRange(0, lhs - rhs, 0);
          }
          friend LinearRange operator>=(const LinearExpr& lhs, const LinearExpr& rhs) {
            return LinearRange(0, lhs - rhs, NT_DOUBLE_INFINITY);
          }

          // private:
          double _lower_bound;
          // invariant: _linear_expr.offset() == 0.
          LinearExpr _linear_expr;
          double     _upper_bound;
        };

        /**
         * @brief Linear objective function
         *
         * Represents the objective to optimize:
         * `offset + sum_i(coeff_i * var_i)`
         *
         * Can be minimized or maximized.
         */
        struct MPObjective {
          // Mapping var -> coefficient.
          nt::PointerMap< const MPVariable*, double > _coefficients;

          // Constant term.
          double _offset;

          //
          MPSolver& _solver;

          // Constructor. An objective points to a single MPSolverInterface
          // that is specified in the constructor. An objective cannot belong
          // to several models.
          // At construction, an MPObjective has no terms (which is equivalent
          // on having a coefficient of 0 for all variables), and an offset of 0.
          explicit MPObjective(MPSolver& solver) : _solver(solver), _offset(0.0) {}

          MPObjective(const MPObjective&) = delete;
          MPObjective& operator=(const MPObjective&) = delete;
          MPObjective(MPObjective&& other) = default;
          MPObjective& operator=(MPObjective&& other) = default;

          /**
           * @brief Clears the objective function
           *
           * Removes all variable coefficients and resets offset to 0.
           * Optimization direction is preserved.
           */
          void clear() {
            _solver.getInterface()->_impl_ClearObjective();
            _coefficients.clear();
            _offset = 0.0;
            // setMinimization();
          }

          /**
           * @brief Sets variable coefficient in objective
           *
           * @param var Variable whose coefficient to set
           * @param coeff Coefficient value (0 removes variable from objective)
           *
           * Variable must belong to this solver.
           */
          void setCoefficient(const MPVariable* const var, double coeff) {
            LOG_IF_F(FATAL, !_solver.ownsVariable(var), "setCoefficient %s", var->name());
            if (var == nullptr) return;
            if (coeff == 0.0) {
              auto it = _coefficients.find(var);
              // See the discussion on MPConstraint::setCoefficient() for 0 coefficients,
              // the same reasoning applies here.
              if (it == _coefficients.end() || it->second == 0.0) return;
              it->second = 0.0;
            } else {
              _coefficients[var] = coeff;
            }
            _solver.getInterface()->_impl_SetObjectiveCoefficient(var, coeff);
          }

          /**
           * @brief Gets variable coefficient in objective
           *
           * @param var Variable to query
           * @return Coefficient value (0 if variable not in objective)
           */
          double getCoefficient(const MPVariable* const var) const {
            LOG_IF_F(FATAL, !_solver.ownsVariable(var), "getCoefficient %s", var->name());
            if (var == nullptr) return 0.0;
            return _coefficients.findWithDefault(var, 0.0);
          }

          /**
           * @brief Returns all objective coefficients
           *
           * @return Map from variables to coefficients (absent = 0)
           */
          const nt::PointerMap< const MPVariable*, double >& terms() const { return _coefficients; }

          /**
           * @brief Sets constant term in objective
           * @param value Offset value
           */
          void setOffset(double value) {
            _offset = value;
            _solver.getInterface()->_impl_SetObjectiveOffset(_offset);
          }

          /**
           * @brief Gets constant term in objective
           * @return Offset value
           */
          double offset() const { return _offset; }

          void checkLinearExpr(const MPSolver& solver, const LinearExpr& linear_expr) {
            for (auto var_value_pair: linear_expr.terms()) {
              CHECK_F(solver.ownsVariable(var_value_pair.first),
                      "Bad MPVariable* in LinearExpr, did you try adding an integer to an "
                      "MPVariable* directly?");
            }
          }

          /**
           * @brief Sets objective from linear expression
           *
           * @param linear_expr Expression to optimize
           * @param is_maximization True to maximize, false to minimize
           *
           * Replaces current objective completely.
           */
          void optimizeLinearExpr(const LinearExpr& linear_expr, bool is_maximization) {
            checkLinearExpr(_solver, linear_expr);
            _solver.getInterface()->_impl_ClearObjective();
            _coefficients.clear();
            setOffset(linear_expr.offset());
            for (const auto& kv: linear_expr.terms()) {
              setCoefficient(kv.first, kv.second);
            }
            setOptimizationDirection(is_maximization);
          }

          /**
           * @brief Maximize linear expression
           * @param linear_expr Expression to maximize
           */
          void maximizeLinearExpr(const LinearExpr& linear_expr) { optimizeLinearExpr(linear_expr, true); }

          /**
           * @brief Minimize linear expression
           * @param linear_expr Expression to minimize
           */
          void minimizeLinearExpr(const LinearExpr& linear_expr) { optimizeLinearExpr(linear_expr, false); }

          /**
           * @brief Adds linear expression to objective
           *
           * @param linear_expr Expression to add
           *
           * Preserves optimization direction.
           */
          void addLinearExpr(const LinearExpr& linear_expr) {
            checkLinearExpr(_solver, linear_expr);
            setOffset(_offset + linear_expr.offset());
            for (const auto& kv: linear_expr.terms()) {
              setCoefficient(kv.first, getCoefficient(kv.first) + kv.second);
            }
          }

          /**
           * @brief Sets optimization direction
           * @param maximize True to maximize, false to minimize
           */
          void setOptimizationDirection(bool maximize) {
            // Note(user): The _maximize bool would more naturally belong to the
            // MPObjective, but it actually has to be a member of MPSolverInterface,
            // because some implementations (such as GLPK) need that bool for the
            // MPSolverInterface constructor, i.e at a time when the MPObjective is not
            // constructed yet (MPSolverInterface is always built before MPObjective
            // when a new MPSolver is constructed).
            _solver._maximize = maximize;
            _solver.getInterface()->_impl_SetOptimizationDirection(maximize);
          }

          /** @brief Set to minimize */
          void setMinimization() { setOptimizationDirection(false); }

          /** @brief Set to maximize */
          void setMaximization() { setOptimizationDirection(true); }

          /** @brief Is maximization? */
          bool maximization() const { return _solver._maximize; }

          /** @brief Is minimization? */
          bool minimization() const { return !_solver._maximize; }

          /**
           * @brief Returns objective value of best solution found
           *
           * @return Optimal objective value if solved to optimality
           *
           * @warning May differ slightly from manual calculation due to
           * numerical precision. Use MPVariable::solution_value() carefully.
           */
          double value() const {
            // Note(user): implementation-wise, the objective value belongs more
            // naturally to the MPSolverInterface, since all of its implementations write
            // to it directly.
            return _solver.objective_value();
          }

          /**
           * @brief Returns best objective bound
           *
           * @return Lower bound (minimization) or upper bound (maximization)
           * on optimal solution. Only for integer problems.
           */
          double bestBound() const {
            // Note(user): the best objective bound belongs to the interface for the
            // same reasons as the objective value does.
            return _solver.best_objective_bound();
          }
        };


        /**
         * @brief Linear constraint in an optimization model
         *
         * Represents a linear constraint of the form:
         * `lb <= sum_i(coeff_i * var_i) <= ub`
         *
         * Constraints belong to exactly one MPSolver instance.
         */
        struct MPConstraint {
          // Mapping var -> coefficient.
          nt::PointerMap< const MPVariable*, double > _coefficients;

          const int _index;   // See index().

          // The lower bound for the linear constraint.
          double _lb;

          // The upper bound for the linear constraint.
          double _ub;

          // name.
          const std::string _name;

          // True if the constraint is "lazy", i.e. the constraint is added to the
          // underlying Linear Programming solver only if it is violated.
          // By default this parameter is 'false'.
          bool _is_lazy;

          bool _active;

          // If given, this constraint is only active if `_indicator_variable`'s value
          // is equal to `_indicator_value`.
          const MPVariable* _indicator_variable;
          bool              _indicator_value;


          double _dual_value;
          double _dual_farkas_value;

          MPSolver& _solver;


          /**
           * @brief Construct a linear constraint
           * @param index Constraint index in solver
           * @param lb Lower bound
           * @param ub Upper bound
           * @param name Constraint name (auto-generated if empty)
           * @param solver Parent solver instance
           */
          MPConstraint(int index, double lb, double ub, const std::string& name, MPSolver& solver) :
              _index(index), _lb(lb), _ub(ub), _name(name.empty() ? nt::format("auto_c_%09d", index) : name),
              _is_lazy(false), _active(true), _indicator_variable(nullptr), _dual_value(0.0), _dual_farkas_value(0.0),
              _solver(solver) {}

          MPConstraint(const MPConstraint&) = delete;
          MPConstraint& operator=(const MPConstraint&) = delete;
          MPConstraint(MPConstraint&& other) = default;
          MPConstraint& operator=(MPConstraint&& other) = default;

          /** @brief Constraint name */
          const std::string& name() const { return _name; }

          /**
           * @brief Clears all variable coefficients
           *
           * Removes all terms. Bounds are preserved.
           */
          void clear() {
            _solver.getInterface()->_impl_ClearConstraint(this);
            _coefficients.clear();
          }

          /**
           * @brief Sets variable coefficient in constraint
           *
           * @param var Variable to set coefficient for
           * @param coeff Coefficient value (0 removes variable)
           *
           * Variable must belong to this solver.
           */
          void setCoefficient(const MPVariable* const var, double coeff) {
            LOG_IF_F(FATAL, !_solver.ownsVariable(var), "setCoefficient %s", var->name());
            if (var == nullptr) return;
            if (coeff == 0.0) {
              auto it = _coefficients.find(var);
              // If setting a coefficient to 0 when this coefficient did not
              // exist or was already 0, do nothing: skip
              // _solver.getInterface()->_impl_SetCoefficient() and do not store a coefficient in
              // the map.  Note that if the coefficient being set to 0 did exist
              // and was not 0, we do have to keep a 0 in the _coefficients map,
              // because the extraction of the constraint might rely on it,
              // depending on the underlying solver.
              if (it != _coefficients.end() && it->second != 0.0) {
                const double old_value = it->second;
                it->second = 0.0;
                _solver.getInterface()->_impl_SetCoefficient(this, var, 0.0, old_value);
              }
              return;
            }
            auto         insertion_result = _coefficients.insert(std::make_pair(var, coeff));
            const double old_value = insertion_result.second ? 0.0 : insertion_result.first->second;
            insertion_result.first->second = coeff;
            _solver.getInterface()->_impl_SetCoefficient(this, var, coeff, old_value);
          }

          /**
           * @brief Gets variable coefficient in constraint
           *
           * @param var Variable to query
           * @return Coefficient value (0 if not in constraint)
           */
          double getCoefficient(const MPVariable* const var) const {
            LOG_IF_F(FATAL, !_solver.ownsVariable(var), "getCoefficient %s", var->name());
            if (var == nullptr) return 0.0;
            return _coefficients.findWithDefault(var, 0.0);
          }

          /**
           * @brief Returns all constraint coefficients
           *
           * @return Map from variables to coefficients (absent = 0)
           */
          const nt::PointerMap< const MPVariable*, double >& terms() const { return _coefficients; }

          /** @brief Lower bound */
          double lb() const { return _lb; }

          /** @brief Upper bound */
          double ub() const { return _ub; }

          /**
           * @brief Set lower bound
           * @param lb New lower bound
           */
          void setLB(double lb) { setBounds(lb, _ub); }

          /**
           * @brief Set upper bound
           * @param ub New upper bound
           */
          void setUB(double ub) { setBounds(_lb, ub); }

          /**
           * @brief Set both bounds
           * @param lb Lower bound
           * @param ub Upper bound
           */
          void setBounds(double lb, double ub) {
            const bool change = lb != _lb || ub != _ub;
            _lb = lb;
            _ub = ub;
            if (change && _solver.constraint_is_extracted(_index)) {
              _solver.getInterface()->_impl_SetConstraintBounds(_index, _lb, _ub);
            }
          }

          /**
           * @brief Is constraint lazy?
           *
           * @return True if lazy constraint
           *
           * @warning Advanced MIP usage
           */
          bool is_lazy() const { return _is_lazy; }

          /**
           * @brief Deactivate constraint
           *
           * Temporarily disables constraint without removing it.
           * Reactivate with activate().
           */
          void deactivate() {
            if (_active && _solver.constraint_is_extracted(_index)) {
              _solver.getInterface()->_impl_SetConstraintBounds(_index, -mp::infinity(), mp::infinity());
            }
            _active = false;
          }

          /** @brief Reactivate constraint */
          void activate() {
            if (!_active && _solver.constraint_is_extracted(_index)) {
              _solver.getInterface()->_impl_SetConstraintBounds(_index, _lb, _ub);
            }
            _active = true;
          }

          /** @brief Is constraint active? */
          bool isActive() const { return _active; }

          /**
           * @brief Set constraint as lazy
           *
           * @param laziness True for lazy constraint
           *
           * @warning Advanced MIP usage. Only supported by SCIP.
           * Lazy constraints added only when violated, improving performance.
           */
          void set_is_lazy(bool laziness) { _is_lazy = laziness; }

          /** @brief Indicator variable for conditional constraint */
          const MPVariable* indicator_variable() const { return _indicator_variable; }

          /** @brief Indicator value for conditional constraint */
          bool indicator_value() const { return _indicator_value; }

          /**
           * @brief Constraint index in solver
           * @return Index in MPSolver::_constraints vector
           */
          int index() const { return _index; }

          /**
           * Advanced usage: returns the dual value of the constraint in the current
           * solution (only available for continuous problems).
           */
          double dual_value() const {
            if (!_solver.isContinuous()) [[unlikely]] {
              LOG_F(FATAL, "Dual value only available for continuous problems");
              return 0.0;
            }
            if (!_solver.checkSolutionIsSynchronizedAndExists()) [[unlikely]]
              return 0.0;
            return _dual_value;
          }

          /**
           * Advanced usage: returns the dual farkas value of the constraint in the
           * current solution (only available for continuous problems).
           */
          double dual_farkas_value() const {
            if (!_solver.isContinuous()) [[unlikely]] {
              LOG_F(FATAL, "Dual farkas value only available for continuous problems");
              return 0.0;
            }
            if (!_solver.checkSolutionIsSynchronized()) [[unlikely]]
              return 0.0;
            return _dual_farkas_value;
          }

          /**
           * Advanced usage: returns the basis status of the constraint.
           *
           * It is only available for continuous problems).
           *
           * Note that if a constraint "linear_expression in [lb, ub]" is transformed
           * into "linear_expression + slack = 0" with slack in [-ub, -lb], then this
           * status is the same as the status of the slack variable with AT_UPPER_BOUND
           * and AT_LOWER_BOUND swapped.
           *
           * @see MPSolver::BasisStatus.
           */
          BasisStatus basis_status() const {
            if (!_solver.isContinuous()) [[unlikely]] {
              LOG_F(FATAL, "Basis status only available for continuous problems");
              return BasisStatus::FREE;
            }
            if (!_solver.checkSolutionIsSynchronizedAndExists()) [[unlikely]] { return BasisStatus::FREE; }
            // This is done lazily as this method is expected to be rarely used.
            return _solver.getInterface()->_impl_row_status(_index);
          }

          // protected:

          void set_dual_value(double dual_value) { _dual_value = dual_value; }

          void set_dual_farkas_value(double dual_farkas_value) { _dual_farkas_value = dual_farkas_value; }


          // private:
          // Returns true if the constraint contains variables that have not
          // been extracted yet.
          bool containsNewVariables() {
            const int last_variable_index = _solver.last_variable_index();
            for (const auto& entry: _coefficients) {
              const int variable_index = entry.first->index();
              if (variable_index >= last_variable_index || !_solver.variable_is_extracted(variable_index)) {
                return true;
              }
            }
            return false;
          }
        };

        // When querying solution values or modifying the model during a callback, use
        // this API, rather than manipulating MPSolver directly.  You should only
        // interact with this object from within MPCallback::runCallback().
        template < class Derived >
        struct MPCallbackContextInterface {
          // What the solver is currently doing.  How you can interact with the solver
          // from the callback depends on this value.
          MPCallbackEvent event() { return static_cast< Derived* >(this)->event(); }

          // Always false if event is not kMipSolution or kMipNode, otherwise behavior
          // may be solver dependent.
          //
          // For Gurobi, under kMipNode, may be false if the node was not solved to
          // optimality, see MIPNODE_REL here for details:
          // http://www.gurobi.com/documentation/8.0/refman/callback_codes.html
          bool canQueryVariableValues() { return static_cast< Derived* >(this)->canQueryVariableValues(); }

          // Returns the value of variable from the solver's current state.
          //
          // Call only when canQueryVariableValues() is true.
          //
          // At kMipSolution, the solution is integer feasible, while at kMipNode, the
          // solution solves the current node's LP relaxation (so integer variables may
          // be fractional).
          double variableValue(const MPVariable* variable) {
            return static_cast< Derived* >(this)->variableValue(variable);
          }

          // Adds a constraint to the model that strengths the LP relaxation.
          //
          // Call only when the event is kMipNode.
          //
          // Requires that MPCallback::might_add_cuts() is true.
          //
          // This constraint must not cut off integer solutions, it should only
          // strengthen the LP (behavior is undefined otherwise).  Use
          // MPCallbackContext::AddLazyConstriant() if you are cutting off integer
          // solutions.
          void addCut(const LinearRange& cutting_plane) { static_cast< Derived* >(this)->addCut(cutting_plane); }

          // Adds a constraint to the model that cuts off an undesired integer solution.
          //
          // Call only when the event is kMipSolution or kMipNode.
          //
          // Requires that MPCallback::might_add_lazy_constraints() is true.
          //
          // Use this to avoid adding a large number of constraints to the model where
          // most are expected to not be needed.
          //
          // Given an integral solution, addLazyConstraint() MUST be able to detect if
          // there is a violated constraint, and it is guaranteed that every integer
          // solution will be checked by addLazyConstraint().
          //
          // Warning(rander): in some solvers, e.g. Gurobi, an integer solution may not
          // respect a previously added lazy constraint, so you may need to add a
          // constraint more than once (e.g. due to threading issues).
          void addLazyConstraint(const LinearRange& lazy_constraint) {
            static_cast< Derived* >(this)->addLazyConstraint(lazy_constraint);
          }

          // TODO
          void addPricedVariable(const MPVariable* variable) {
            static_cast< Derived* >(this)->addPricedVariable(variable);
          }

          // Suggests a (potentially partial) variable assignment to the solver, to be
          // used as a feasible solution (or part of one). If the assignment is partial,
          // certain solvers (e.g. Gurobi) will try to compute a feasible solution from
          // the partial assignment. Returns the objective value of the solution if the
          // solver supports it.
          //
          // Call only when the event is kMipNode.
          double suggestSolution(const nt::PointerMap< const MPVariable*, double >& solution) {
            return static_cast< Derived* >(this)->suggestSolution(solution);
          }

          // Returns the number of nodes explored so far in the branch and bound tree,
          // which 0 at the root node and > 0 otherwise.
          //
          // Call only when the event is kMipSolution or kMipNode.
          int64_t numExploredNodes() { return static_cast< Derived* >(this)->numExploredNodes(); }
        };


        // Extend this class with model specific logic, and register through
        // MPSolver::setCallback, passing a pointer to this object.
        //
        template < typename MPCallbackContext >
        struct MPCallback {
          bool might_add_cuts_;
          bool might_add_lazy_constraints_;
          bool might_add_priced_variables_;

          virtual ~MPCallback() = default;

          // If you intend to call call MPCallbackContext::addCut(), you must set
          // might_add_cuts below to be true.  Likewise for
          // MPCallbackContext::addLazyConstraint() and might_add_lazy_constraints.
          MPCallback(bool might_add_cuts, bool might_add_lazy_constraints, bool might_add_priced_variables) :
              might_add_cuts_(might_add_cuts), might_add_lazy_constraints_(might_add_lazy_constraints),
              might_add_priced_variables_(might_add_priced_variables) {}

          // Threading behavior may be solver dependent:
          //   * Gurobi: runCallback always runs on the same thread that you called
          //     MPSolver::solve() on, even when Gurobi uses multiple threads.
          virtual void runCallback(MPCallbackContext* callback_context) = 0;

          bool might_add_cuts() const { return might_add_cuts_; }
          bool might_add_lazy_constraints() const { return might_add_lazy_constraints_; }
          bool might_add_priced_variables() const { return might_add_priced_variables_; }
        };

        // Single callback that runs the list of callbacks given at construction, in
        // sequence.
        template < typename MPCallbackContext >
        struct MPCallbackList: MPCallback< MPCallbackContext > {
          using Callback = MPCallback< MPCallbackContext >;
          const nt::TrivialDynamicArray< Callback* > callbacks_;

          explicit MPCallbackList(nt::TrivialDynamicArray< Callback* >&& callbacks) :
              Callback(callbacksMightAddCuts(callbacks),
                       callbacksMightAddLazyConstraints(callbacks),
                       callbacksMightAddPricedVariables(callbacks)) {
            callbacks_(std::move(callbacks));
          }

          // Runs all callbacks from the list given at construction, in sequence.
          void runCallback(MPCallbackContext* context) override {
            for (Callback* callback: callbacks_) {
              callback->runCallback(context);
            }
          }

          // Returns true if any of the callbacks in a list might add cuts.
          static bool callbacksMightAddCuts(nt::ArrayView< Callback* > callbacks) noexcept {
            for (const Callback* const callback: callbacks) {
              if (callback->might_add_cuts()) { return true; }
            }
            return false;
          }

          // Returns true if any of the callbacks in a list might add lazy constraints.
          static bool callbacksMightAddLazyConstraints(nt::ArrayView< Callback* > callbacks) noexcept {
            for (const Callback* const callback: callbacks) {
              if (callback->might_add_lazy_constraints()) { return true; }
            }
            return false;
          }

          // Returns true if any of the callbacks in a list might add priced variables.
          static bool callbacksMightAddPricedVariables(nt::ArrayView< Callback* > callbacks) noexcept {
            for (const Callback* const callback: callbacks) {
              if (callback->might_add_priced_variables()) { return true; }
            }
            return false;
          }
        };

        // Allocators
        MemoryManager< MPVariable >   _var_alloc;
        MemoryManager< MPConstraint > _cons_alloc;

        // The name of the linear programming problem.
        const std::string _name;

        // The type of the linear programming problem.
        // const OptimizationProblemType problem_type_;

        // The solver interface.
        // MPSolverInterface interface_;

        // The vector of variables in the problem.
        nt::TrivialDynamicArray< MPVariable* > _variables;
        // A map from a variable's name to its index in _variables.
        mutable std::optional< nt::CstringMap< int > > _variable_name_to_index;
        // Whether variables have been extracted to the underlying interface.
        std::vector< bool > _variable_is_extracted;

        // The vector of constraints in the problem.
        nt::TrivialDynamicArray< MPConstraint* > _constraints;
        // A map from a constraint's name to its index in _constraints.
        mutable std::optional< nt::CstringMap< int > > _constraint_name_to_index;
        // Whether constraints have been extracted to the underlying interface.
        std::vector< bool > _constraint_is_extracted;

        // The linear objective function.
        MPObjective _objective;

        // Initial values for all or some of the problem variables that can be
        // exploited as a starting hint by a solver.
        //
        // Note(user): as of 05/05/2015, we can't use >> because of some SWIG errors.
        //
        // TODO(user): replace by two vectors, a std::vector<bool> to indicate if a
        // hint is provided and a std::vector<double> for the hint value.
        nt::TrivialDynamicArray< nt::Pair< const MPVariable*, double > > _solution_hint;

        int64_t _time_limit;   // In milliseconds. Default = No limit.

        // const absl::Time construction_time_;

        // Permanent storage for the number of threads.
        int _num_threads;

        // Permanent storage for setSolverSpecificParametersAsString().
        std::string _solver_specific_parameter_string;

        protected:
        /**
         * Create a solver with the given name and underlying solver backend.
         */
        MPSolver(const std::string& name) :
            _name(name),
            /*problem_type_(DUMMY_MIXED_INTEGER_PROGRAMMING), */ _objective(*getInterface()), _time_limit(NT_INT64_MAX),
            _num_threads(1), _sync_status(SynchronizationStatus::MODEL_SYNCHRONIZED),
            _result_status(ResultStatus::NOT_SOLVED), _maximize(false), _last_constraint_index(0),
            _last_variable_index(0), _objective_value(0.0), _best_objective_bound(0.0), _quiet(true) {
          // interface_.reset(BuildSolverInterface(this));
          // if (absl::GetFlag(FLAGS_linear_solver_enable_verbose_output)) { enableOutput(); }
          // _objective.reset(new MPObjective(getInterface()));
        }

        public:
        ~MPSolver() noexcept {}

        MPSolver(const MPSolver&) = delete;
        MPSolver& operator=(const MPSolver&) = delete;

        /**
         * Recommended factory method to create a MPSolver instance, especially in
         * non C++ languages.
         *
         * It returns a newly created solver instance if successful, or a nullptr
         * otherwise. This can occur if the relevant interface is not linked in, or if
         * a needed license is not accessible for commercial solvers.
         *
         * Ownership of the solver is passed on to the caller of this method.
         * It will accept both string names of the OptimizationProblemType enum, as
         * well as a short version (i.e. "SCIP_MIXED_INTEGER_PROGRAMMING" or "SCIP").
         *
         * solver_id is case insensitive, and the following names are supported:
         *   - CLP_LINEAR_PROGRAMMING or CLP
         *   - CBC_MIXED_INTEGER_PROGRAMMING or CBC
         *   - GLOP_LINEAR_PROGRAMMING or GLOP
         *   - BOP_INTEGER_PROGRAMMING or BOP
         *   - SAT_INTEGER_PROGRAMMING or SAT or CP_SAT
         *   - SCIP_MIXED_INTEGER_PROGRAMMING or SCIP
         *   - GUROBI_LINEAR_PROGRAMMING or GUROBI_LP
         *   - GUROBI_MIXED_INTEGER_PROGRAMMING or GUROBI or GUROBI_MIP
         *   - CPLEX_LINEAR_PROGRAMMING or CPLEX_LP
         *   - CPLEX_MIXED_INTEGER_PROGRAMMING or CPLEX or CPLEX_MIP
         *   - XPRESS_LINEAR_PROGRAMMING or XPRESS_LP
         *   - XPRESS_MIXED_INTEGER_PROGRAMMING or XPRESS or XPRESS_MIP
         *   - GLPK_LINEAR_PROGRAMMING or GLPK_LP
         *   - GLPK_MIXED_INTEGER_PROGRAMMING or GLPK or GLPK_MIP
         */
        // static MPSolver* CreateSolver(const std::string& solver_id);

        std::string prettyPrintVar(const MPVariable& var) const {
          const std::string prefix = "Variable '" + var.name() + "': domain = ";
          if (var.lb() >= infinity() || var.ub() <= -infinity() || var.lb() > var.ub()) {
            return prefix + "∅";   // Empty set.
          }
          // Special case: integer variable with at most two possible values
          // (and potentially none).
          if (var.integer() && var.ub() - var.lb() <= 1) {
            const int64_t lb = static_cast< int64_t >(ceil(var.lb()));
            const int64_t ub = static_cast< int64_t >(floor(var.ub()));
            if (lb > ub) {
              return prefix + "∅";
            } else if (lb == ub) {
              return nt::format("{}{{ {} }}", prefix.c_str(), lb);
            } else {
              return nt::format("{}{{ {}, {} }}", prefix.c_str(), lb, ub);
            }
          }
          // Special case: single (non-infinite) real value.
          if (var.lb() == var.ub()) { return nt::format("{}{{ {} }}", prefix.c_str(), var.lb()); }
          return prefix + (var.integer() ? "Integer" : "Real") + " in "
                 + (var.lb() <= -infinity() ? std::string("]-∞") : nt::format("[{}", var.lb())) + ", "
                 + (var.ub() >= infinity() ? std::string("+∞[") : nt::format("{}]", var.ub()));
        }


        std::string prettyPrintObjective(const MPObjective& obj) const {
          nt::MemoryBuffer buffer;

          if (obj.minimization())
            nt::format_to(std::back_inserter(buffer), "min z = ");
          else
            nt::format_to(std::back_inserter(buffer), "max z = ");

          appendAllTerms(obj.terms(), &buffer);

          return nt::to_string(buffer);
        }

        std::string prettyPrintConstraint(const MPConstraint& constraint) const {
          nt::MemoryBuffer buffer;
          nt::format_to(std::back_inserter(buffer), "Constraint '{}': ", constraint.name());

          if (constraint.lb() >= infinity() || constraint.ub() <= -infinity() || constraint.lb() > constraint.ub()) {
            nt::format_to(std::back_inserter(buffer), "ALWAYS FALSE");
            return nt::to_string(buffer);
          }
          if (constraint.lb() <= -infinity() && constraint.ub() >= infinity()) {
            nt::format_to(std::back_inserter(buffer), "ALWAYS TRUE");
            return nt::to_string(buffer);
          }

          appendAllTerms(constraint.terms(), &buffer);

          // Equality.
          if (constraint.lb() == constraint.ub()) {
            nt::format_to(std::back_inserter(buffer), " = {}", constraint.lb());
            return nt::to_string(buffer);
          }
          // Inequalities.
          if (constraint.lb() <= -infinity()) {
            nt::format_to(std::back_inserter(buffer), " ≤ {}", constraint.ub());
            return nt::to_string(buffer);
          }
          if (constraint.ub() >= infinity()) {
            nt::format_to(std::back_inserter(buffer), " ≥ {}", constraint.lb());
            return nt::to_string(buffer);
          }
          nt::format_to(std::back_inserter(buffer), " ∈ [{}, {}]", constraint.lb(), constraint.ub());
          return nt::to_string(buffer);
        }

        std::string print() const {
          nt::MemoryBuffer buffer;
          nt::format_to(std::back_inserter(buffer), "{}\n", prettyPrintObjective(objective()));
          for (auto c: constraints())
            nt::format_to(std::back_inserter(buffer), "{}\n", prettyPrintConstraint(*c));
          return nt::to_string(buffer);
        }

        /**
         * @brief
         *
         * @return nt::JSON
         */
        nt::JSONDocument exportSolutionToJSON() const {
          nt::JSONDocument json;
          json.SetObject();

          nt::JSONDocument::AllocatorType& allocator = json.GetAllocator();
          const std::string                solver_version = solverVersion();
          json.AddMember(
             "solver_version", nt::JSONValue(solver_version.c_str(), json.GetAllocator()), json.GetAllocator());
          json.AddMember("solved", nt::JSONValue(isSolved()), json.GetAllocator());
          json.AddMember("objective", nt::JSONValue(objective().value()), json.GetAllocator());

          nt::JSONValue variables_array_json(nt::JSONValueType::ARRAY_TYPE);
          for (int i = 0; i < _variables.size(); ++i) {
            nt::JSONValue     variable_object(nt::JSONValueType::OBJECT_TYPE);
            const MPVariable* p_variable = _variables[i];
            variable_object.AddMember(
               "name", nt::JSONValue(p_variable->name().c_str(), p_variable->name().size()), allocator);
            variable_object.AddMember("integer", nt::JSONValue(p_variable->integer()), allocator);
            variable_object.AddMember("value", nt::JSONValue(p_variable->solution_value()), allocator);
            if (isContinuous())
              variable_object.AddMember("reduced_cost", nt::JSONValue(p_variable->reduced_cost()), allocator);
            variables_array_json.PushBack(std::move(variable_object), allocator);
          }
          json.AddMember("variables", std::move(variables_array_json), allocator);

          if (isContinuous()) {
            nt::JSONValue constraints_array_json(nt::JSONValueType::ARRAY_TYPE);
            for (int i = 0; i < _constraints.size(); ++i) {
              nt::JSONValue       constraint_object_json(nt::JSONValueType::OBJECT_TYPE);
              const MPConstraint* p_constraint = _constraints[i];
              constraint_object_json.AddMember(
                 "name", nt::JSONValue(p_constraint->name().c_str(), p_constraint->name().size()), allocator);
              constraint_object_json.AddMember("dual", nt::JSONValue(p_constraint->dual_value()), allocator);
              if (hasDualFarkas())
                constraint_object_json.AddMember(
                   "dual_farkas", nt::JSONValue(p_constraint->dual_farkas_value()), allocator);
              constraints_array_json.PushBack(std::move(constraint_object_json), allocator);
            }
            json.AddMember("constraints", std::move(constraints_array_json), allocator);
          }

          return json;
        }

        /**
         * Whether the given problem type is supported (this will depend on the
         * targets that you linked).
         */
        //       static bool SupportsProblemType(OptimizationProblemType problem_type) {
        // #ifdef USE_CLP
        //         if (problem_type == CLP_LINEAR_PROGRAMMING) return true;
        // #endif
        // #ifdef USE_SOPLEX
        //         if (problem_type == SOPLEX_LINEAR_PROGRAMMING) return true;
        // #endif
        // #ifdef USE_GLPK
        //         if (problem_type == GLPK_LINEAR_PROGRAMMING || problem_type ==
        //         GLPK_MIXED_INTEGER_PROGRAMMING) { return true; }
        // #endif
        //         if (problem_type == BOP_INTEGER_PROGRAMMING) return true;
        //         if (problem_type == SAT_INTEGER_PROGRAMMING) return true;
        //         if (problem_type == GLOP_LINEAR_PROGRAMMING) return true;
        // #ifdef USE_GUROBI
        //         if (problem_type == GUROBI_LINEAR_PROGRAMMING || problem_type ==
        //         GUROBI_MIXED_INTEGER_PROGRAMMING) { return
        //         MPSolver::gurobiIsCorrectlyInstalled(); }
        // #endif
        // #ifdef USE_SCIP
        //         if (problem_type == SCIP_MIXED_INTEGER_PROGRAMMING) return true;
        // #endif
        // #ifdef USE_CBC
        //         if (problem_type == CBC_MIXED_INTEGER_PROGRAMMING) return true;
        // #endif
        // #ifdef USE_XPRESS
        //         if (problem_type == XPRESS_MIXED_INTEGER_PROGRAMMING || problem_type ==
        //         XPRESS_LINEAR_PROGRAMMING) { return true; }
        // #endif
        // #if defined(NT_HAVE_CPLEX)
        //         if (problem_type == CPLEX_LINEAR_PROGRAMMING || problem_type ==
        //         CPLEX_MIXED_INTEGER_PROGRAMMING) { return true; }
        // #endif
        //         return false;
        //       }

        /**
         * Parses the name of the solver. Returns true if the solver type is
         * successfully parsed as one of the OptimizationProblemType.
         * See the documentation of CreateSolver() for the list of supported names.
         */
        // static bool ParseSolverType(const std::string& solver_id, OptimizationProblemType* type);

        /**
         * Parses the name of the solver and returns the correct optimization type or
         * dies. Invariant: ParseSolverTypeOrDie(toString(type)) = type.
         */
        // static OptimizationProblemType ParseSolverTypeOrDie(const std::string& solver_id);

        // ----- Misc -----
        // Queries problem type. For simplicity, the distinction between
        // continuous and discrete is based on the declaration of the user
        // when the solver is created (example: GLPK_LINEAR_PROGRAMMING
        // vs. GLPK_MIXED_INTEGER_PROGRAMMING), not on the actual content of
        // the model.
        // Returns true if the problem is continuous.
        bool isContinuous() const { return getInterface()->_impl_IsContinuous(); }
        // Returns true if the problem is continuous and linear.
        bool isLP() const { return getInterface()->_impl_IsLP(); }
        // Returns true if the problem is discrete and linear.
        bool isMIP() const { return getInterface()->_impl_IsMIP(); }

        /** Returns the name of the model set at construction. */
        const std::string& name() const {
          return _name;   // Set at construction.
        }

        /** Returns the optimization problem type set at construction. */
        // OptimizationProblemType ProblemType() const {
        //   return problem_type_;   // Set at construction.
        // }

        /**
         * @brief Clears entire model
         *
         * Removes all variables, constraints, and objective.
         * Solver settings (time limit, etc.) are preserved.
         */
        void clear() {
          getInterface()->_impl_Reset();
          _objective._coefficients.clear();
          _objective._offset = 0.0;
          _var_alloc.clear();
          _cons_alloc.clear();
          _variables.clear();
          if (_variable_name_to_index) { _variable_name_to_index->clear(); }
          _variable_is_extracted.clear();
          _constraints.clear();
          if (_constraint_name_to_index) { _constraint_name_to_index->clear(); }
          _constraint_is_extracted.clear();
          _solution_hint.clear();
        }

        /** @brief Number of variables */
        int numVariables() const { return _variables.size(); }

        /**
         * @brief Pre-allocate memory for model
         *
         * @param i_num_variables Expected number of variables
         * @param i_num_constraints Expected number of constraints
         *
         * Improves performance when model size is known in advance.
         */
        void reserve(int i_num_variables, int i_num_constraints) {
          _variables.reserve(i_num_variables);
          _variable_is_extracted.reserve(i_num_variables);
          _var_alloc.reserve(i_num_variables);
          _constraints.reserve(i_num_constraints);
          _constraint_is_extracted.reserve(i_num_constraints);
          _cons_alloc.reserve(i_num_constraints);
        }

        /**
         * @brief All variables in creation order
         * @return Array of variables
         */
        const nt::TrivialDynamicArray< MPVariable* >& variables() const { return _variables; }

        /**
         * @brief Find variable by name
         *
         * @param var_name Variable name to search for
         * @return Variable pointer or nullptr if not found
         *
         * @warning First call is O(n), subsequent calls O(1).
         * Crashes if variable names not unique.
         */
        MPVariable* lookupVariableOrNull(const std::string& var_name) const {
          if (!_variable_name_to_index) generateVariableNameIndex();
          int it = _variable_name_to_index->findWithDefault(var_name.c_str(), -1);
          if (it == -1) return nullptr;
          return _variables[it];
        }

        /**
         * @brief Create a decision variable
         *
         * @param lb Lower bound (can be -infinity())
         * @param ub Upper bound (can be infinity())
         * @param integer True for integer variable, false for continuous
         * @param name Variable name (auto-generated if empty)
         * @return Variable pointer (owned by solver)
         */
        MPVariable* makeVar(double lb, double ub, bool integer, const std::string& name) {
          const int   var_index = numVariables();
          MPVariable* v = _var_alloc.create(var_index, lb, ub, integer, name, *getInterface());
          if (_variable_name_to_index) { _variable_name_to_index->insertNoDuplicate(v->name().c_str(), var_index); }
          _variables.push_back(v);
          _variable_is_extracted.push_back(false);

          getInterface()->_impl_AddVariable(v);
          return v;
        }

        /**
         * @brief Create continuous variable
         * @param lb Lower bound
         * @param ub Upper bound
         * @param name Variable name
         * @return Continuous variable
         */
        MPVariable* makeNumVar(double lb, double ub, const std::string& name) { return makeVar(lb, ub, false, name); }

        /**
         * @brief Create integer variable
         * @param lb Lower bound
         * @param ub Upper bound
         * @param name Variable name
         * @return Integer variable
         */
        MPVariable* makeIntVar(double lb, double ub, const std::string& name) { return makeVar(lb, ub, true, name); }

        /**
         * @brief Create binary variable (0 or 1)
         * @param name Variable name
         * @return Boolean variable
         */
        MPVariable* makeBoolVar(const std::string& name) { return makeVar(0.0, 1.0, true, name); }

        /**
         * @brief Create a multidim boolean variable
         *
         * @param prefix_name
         * @param iterables
         * @return
         */
        template < class... DIMS >
        MPMultiDimVariable< sizeof...(DIMS) > makeMultiDimBoolVar(const std::string& prefix_name, const DIMS&... dims) {
          return makeMultiDimVar(0., 1., true, prefix_name, dims...);
        };

        /**
         * @brief Create a multidim integer variable
         *
         * @param prefix_name
         * @param dims
         * @return
         */
        template < class... DIMS >
        MPMultiDimVariable< sizeof...(DIMS) >
           makeMultiDimIntVar(double lb, double up, const std::string& prefix_name, const DIMS&... dims) {
          return makeMultiDimVar(lb, up, true, prefix_name, dims...);
        };

        /**
         * @brief Create a multidim continuous variable
         *
         * @param prefix_name
         * @param dims
         * @return
         */
        template < class... DIMS >
        MPMultiDimVariable< sizeof...(DIMS) >
           makeMultiDimNumVar(double lb, double up, const std::string& prefix_name, const DIMS&... dims) {
          return makeMultiDimVar(lb, up, false, prefix_name, dims...);
        };

        /**
         * @brief Create a multidim variable
         *
         * @param prefix_name
         * @param dims
         * @return
         */
        template < class... DIMS >
        MPMultiDimVariable< sizeof...(DIMS) + 1 > makeMultiDimVar(
           double lb, double ub, bool integer, const std::string& prefix_name, int d, const DIMS&... dims) {
          MPMultiDimVariable< sizeof...(DIMS) + 1 > vars;
          if constexpr (sizeof...(DIMS) == 0)
            vars.ensureAndFill(d);
          else
            vars.setDimensions(d, dims...);
          _var_alloc.reserve(vars.size());
          for (int i = 0; i < vars.size(); ++i) {
            if (prefix_name.empty()) {
              vars[i] = makeVar(lb, ub, integer, prefix_name);
            } else {
              const std::string vname = nt::format("{}{}", prefix_name.c_str(), i);
              vars[i] = makeVar(lb, ub, integer, vname);
            }
          }
          return vars;
        };

        /**
         * @brief Create a multidim boolean variable
         *
         * @param prefix_name
         * @param iterables
         * @return
         */
        template < class GR, class... ELEMENTS >
        MPGraphVariable< GR, ELEMENTS... > makeGraphBoolVar(const nt::String& prefix_name, const GR& g) {
          return makeGraphVar< GR, ELEMENTS... >(0., 1., true, prefix_name, g);
        };

        /**
         * @brief Create a multidim integer variable
         *
         * @param prefix_name
         * @param iterables
         * @return
         */
        template < class GR, class... ELEMENTS >
        MPGraphVariable< GR, ELEMENTS... >
           makeGraphIntVar(double lb, double up, const nt::String& prefix_name, const GR& g) {
          return makeGraphVar< GR, ELEMENTS... >(lb, up, true, prefix_name, g);
        };

        /**
         * @brief Create a multidim continuous variable
         *
         * @param prefix_name
         * @param iterables
         * @return
         */
        template < class GR, class... ELEMENTS >
        MPGraphVariable< GR, ELEMENTS... >
           makeGraphNumVar(double lb, double up, const nt::String& prefix_name, const GR& g) {
          return makeGraphVar< GR, ELEMENTS... >(lb, up, false, prefix_name, g);
        };

        /**
         * @brief Create a graph variable
         *
         * @param prefix_name
         * @param iterables
         * @return
         */
        template < class GR, class... ELEMENTS >
        MPGraphVariable< GR, ELEMENTS... >
           makeGraphVar(double lb, double ub, bool integer, const nt::String& prefix_name, const GR& g) {
          static_assert(sizeof...(ELEMENTS) > 0, "At least one iterable container must be provided.");
          MPGraphVariable< GR, ELEMENTS... > vars(g);
          for (int i = 0; i < vars.size(); ++i) {
            const std::string vname = nt::format("{}{}", prefix_name.c_str(), i);
            vars._values[i] = makeVar(lb, ub, integer, vname);
          }
          return vars;
        };


        /**
         * Creates an array of variables. All variables created have the same bounds
         * and integrality requirement. If nb <= 0, no variables are created, the
         * function crashes in non-opt mode.
         *
         * @param nb the number of variables to create.
         * @param lb the lower bound of created variables
         * @param ub the upper bound of created variables
         * @param integer controls whether the created variables are continuous or
         * integral.
         * @param name_prefix the prefix of the variable names. Variables are named
         * name_prefix0, name_prefix1, ...
         * @param[out] vars the vector of variables to fill with variables.
         */
        void makeVarArray(int                                     nb,
                          double                                  lb,
                          double                                  ub,
                          bool                                    integer,
                          const std::string&                      name_prefix,
                          nt::TrivialDynamicArray< MPVariable* >* vars) {
          CHECK_GE_F(nb, 0);
          if (nb <= 0) return;
          // const int num_digits = NumDigits(nb);
          for (int i = 0; i < nb; ++i) {
            if (name_prefix.empty()) {
              vars->push_back(makeVar(lb, ub, integer, name_prefix));
            } else {
              // std::string vname = nt::format("%s%0*d", name_prefix.c_str(), num_digits, i);
              const std::string vname = nt::format("{}{}", name_prefix.c_str(), i);
              vars->push_back(makeVar(lb, ub, integer, vname));
            }
          }
        }

        /** Creates an array of continuous variables. */
        void makeNumVarArray(
           int nb, double lb, double ub, const std::string& name, nt::TrivialDynamicArray< MPVariable* >* vars) {
          makeVarArray(nb, lb, ub, false, name, vars);
        }

        /** Creates an array of integer variables. */
        void makeIntVarArray(
           int nb, double lb, double ub, const std::string& name, nt::TrivialDynamicArray< MPVariable* >* vars) {
          makeVarArray(nb, lb, ub, true, name, vars);
        }

        /** Creates an array of boolean variables. */
        void makeBoolVarArray(int nb, const std::string& name, nt::TrivialDynamicArray< MPVariable* >* vars) {
          makeVarArray(nb, 0.0, 1.0, true, name, vars);
        }

        /** Returns the number of constraints. */
        /** @brief Number of constraints */
        int numConstraints() const { return _constraints.size(); }

        /**
         * @brief All constraints in creation order
         * @return Array of constraints
         */
        const nt::TrivialDynamicArray< MPConstraint* >& constraints() const { return _constraints; }

        /**
         * @brief Find constraint by name
         *
         * @param constraint_name Constraint name to search for
         * @return Constraint pointer or nullptr if not found
         *
         * @warning First call is O(n), subsequent calls O(1).
         * Crashes if constraint names not unique.
         */
        MPConstraint* lookupConstraintOrNull(const std::string& constraint_name) const {
          if (!_constraint_name_to_index) generateConstraintNameIndex();

          int it = _constraint_name_to_index->findWithDefault(constraint_name.c_str(), -1);
          if (it == -1) return nullptr;
          return _constraints[it];
        }

        /**
         * @brief Create constraint with bounds
         * @param lb Lower bound
         * @param ub Upper bound
         * @return Constraint pointer (owned by solver)
         */
        MPConstraint* makeRowConstraint(double lb, double ub) { return makeRowConstraint(lb, ub, ""); }

        /** @brief Create unconstrained constraint (-∞ to +∞) */
        MPConstraint* makeRowConstraint() { return makeRowConstraint(-infinity(), infinity(), ""); }

        /**
         * @brief Create named constraint with bounds
         * @param lb Lower bound (can be -infinity())
         * @param ub Upper bound (can be infinity())
         * @param name Constraint name (auto-generated if empty)
         * @return Constraint pointer (owned by solver)
         */
        MPConstraint* makeRowConstraint(double lb, double ub, const std::string& name) {
          const int           constraint_index = numConstraints();
          MPConstraint* const constraint = _cons_alloc.create(constraint_index, lb, ub, name, *getInterface());
          if (_constraint_name_to_index) {
            _constraint_name_to_index->insertNoDuplicate(constraint->name().c_str(), constraint_index);
          }
          _constraints.push_back(constraint);
          _constraint_is_extracted.push_back(false);

          getInterface()->_impl_AddRowConstraint(constraint);
          // interface_.AddRowConstraint(constraint);
          return constraint;
        }

        /**
         * @brief Create named unconstrained constraint
         * @param name Constraint name
         * @return Constraint pointer
         */
        MPConstraint* makeRowConstraint(const std::string& name) {
          return makeRowConstraint(-infinity(), infinity(), name);
        }

        /**
         * @brief Create constraint from LinearRange
         *
         * @param range Range expression (e.g., x + y <= 10)
         * @return Constraint pointer (owned by solver)
         */
        MPConstraint* makeRowConstraint(const LinearRange& range) { return makeRowConstraint(range, ""); }

        /**
         * @brief Create named constraint from LinearRange
         * @param range Range expression
         * @param name Constraint name
         * @return Constraint pointer
         */
        MPConstraint* makeRowConstraint(const LinearRange& range, const std::string& name) {
          checkLinearExpr(*this, range.linear_expr());
          MPConstraint* constraint = makeRowConstraint(range.lower_bound(), range.upper_bound(), name);
          for (const auto& kv: range.linear_expr().terms()) {
            constraint->setCoefficient(kv.first, kv.second);
          }
          return constraint;
        }

        /**
         * @brief Get objective (read-only)
         * @return objective reference
         */
        const MPObjective& objective() const { return _objective; }

        /**
         * @brief Get mutable objective
         * @return objective pointer for modification
         */
        MPObjective* mutableObjective() { return &_objective; }

        /**
         * @brief solve the optimization problem
         *
         * @return Result status (OPTIMAL, FEASIBLE, INFEASIBLE, etc.)
         *
         * Uses default solver parameters. Call after building complete model.
         */
        ResultStatus solve() {
          MPSolverParameters default_param;
          return solve(default_param);
        }

        /**
         * @brief solve with custom parameters
         *
         * @param param Solver parameters
         * @return Result status
         */
        ResultStatus solve(const MPSolverParameters& param) {
          // Special case for infeasible constraints so that all solvers have
          // the same behavior.
          // TODO(user): replace this by model extraction to proto + proto validation
          // (the proto has very low overhead compared to the wrapper, both in
          // performance and memory, so it's ok).
          if (hasInfeasibleConstraints()) {
            _result_status = ResultStatus::INFEASIBLE;
            return _result_status;
          }

          ResultStatus status = getInterface()->_impl_Solve(param);

          // if (absl::GetFlag(FLAGS_verify_solution)) {
          if (status != ResultStatus::OPTIMAL && status != ResultStatus::FEASIBLE) {
            LOG_F(1,
                  "--verify_solution enabled, but the solver did not find a solution: skipping the "
                  "verification.");
          } else if (!verifySolution(param.getDoubleParam(MPSolverParameters::PRIMAL_TOLERANCE), true)) {
            status = ResultStatus::ABNORMAL;
            _result_status = status;
          }
          //}
          CHECK_EQ_F(_result_status, status);
          return status;
        }

        /**
         * @brief Check if model was solved successfully
         * @return True if OPTIMAL or FEASIBLE solution found
         */
        bool isSolved() const {
          return _result_status == ResultStatus::OPTIMAL || _result_status == ResultStatus::FEASIBLE;
        }

        /**
         * @brief Check if Farkas dual values available
         * @return True if Farkas duals can be queried
         */
        bool hasDualFarkas() const {
          if (_result_status == ResultStatus::NOT_SOLVED) {
            LOG_F(WARNING, "Attempting to get dual farkas values while the model is not solved yet.");
            return false;
          }
          return getInterface()->_impl_HasDualFarkas();
        }

        /**
         * @brief write model to file
         *
         * @param file_name Output file path
         *
         * @warning Solver-specific. Currently only Gurobi supported.
         */
        void write(const std::string& file_name) { getInterface()->_impl_Write(file_name); }

        /**
         * Advanced usage: compute the "activities" of all constraints, which are the
         * sums of their linear terms. The activities are returned in the same order
         * as constraints(), which is the order in which constraints were added; but
         * you can also use MPConstraint::index() to get a constraint's index.
         */
        nt::DoubleDynamicArray computeConstraintActivities() const {
          // TODO(user): test this failure case.
          if (!checkSolutionIsSynchronizedAndExists()) [[unlikely]]
            return {};
          nt::DoubleDynamicArray activities(_constraints.size());
          for (int i = 0; i < _constraints.size(); ++i) {
            const MPConstraint&       constraint = *_constraints[i];
            nt::AccurateSum< double > sum;
            for (const auto& entry: constraint._coefficients)
              sum.add(entry.first->solution_value() * entry.second);
            activities[i] = sum.value();
          }
          return activities;
        }

        /**
         * Advanced usage: Verifies the *correctness* of the solution.
         *
         * It verifies that all variables must be within their domains, all
         * constraints must be satisfied, and the reported objective value must be
         * accurate.
         *
         * Usage:
         * - This can only be called after solve() was called.
         * - "tolerance" is interpreted as an absolute error threshold.
         * - For the objective value only, if the absolute error is too large,
         *   the tolerance is interpreted as a relative error threshold instead.
         * - If "log_errors" is true, every single violation will be logged.
         * - If "tolerance" is negative, it will be set to infinity().
         *
         * Most users should just set the --verify_solution flag and not bother using
         * this method directly.
         */
        // TODO(user): split.
        bool verifySolution(double tolerance, bool log_errors) const {
          double max_observed_error = 0;
          if (tolerance < 0) tolerance = infinity();
          int num_errors = 0;

          // Verify variables.
          for (int i = 0; i < _variables.size(); ++i) {
            const MPVariable& var = *_variables[i];
            const double      value = var.solution_value();
            // Check for NaN.
            if (std::isnan(value)) {
              ++num_errors;
              max_observed_error = infinity();
              LOG_IF_F(ERROR, log_errors, "NaN value for {}", prettyPrintVar(var));
              continue;
            }
            // Check lower bound.
            if (var.lb() != -infinity()) {
              if (value < var.lb() - tolerance) {
                ++num_errors;
                max_observed_error = std::max(max_observed_error, var.lb() - value);
                LOG_IF_F(ERROR, log_errors, "value {} too low for {}", value, prettyPrintVar(var));
              }
            }
            // Check upper bound.
            if (var.ub() != infinity()) {
              if (value > var.ub() + tolerance) {
                ++num_errors;
                max_observed_error = std::max(max_observed_error, value - var.ub());
                LOG_IF_F(ERROR, log_errors, "value {} too high for {}", value, prettyPrintVar(var));
              }
            }
            // Check integrality.
            if (isMIP() && var.integer()) {
              if (fabs(value - round(value)) > tolerance) {
                ++num_errors;
                max_observed_error = std::max(max_observed_error, fabs(value - round(value)));
                LOG_IF_F(ERROR, log_errors, "Non-integer value {} for {}", value, prettyPrintVar(var));
              }
            }
          }
          if (!isMIP() && hasIntegerVariables()) {
            LOG_IF_F(INFO,
                     log_errors,
                     "Skipped variable integrality check, because a continuous relaxation of the "
                     "model was solved (i.e., the selected solver does not support "
                     "integer variables).");
          }

          // Verify constraints.
          const nt::DoubleDynamicArray activities = computeConstraintActivities();
          for (int i = 0; i < _constraints.size(); ++i) {
            const MPConstraint& constraint = *_constraints[i];
            const double        activity = activities[i];
            // Re-compute the activity with a inaccurate summing algorithm.
            double inaccurate_activity = 0.0;
            for (const auto& entry: constraint._coefficients) {
              inaccurate_activity += entry.first->solution_value() * entry.second;
            }
            // Catch NaNs.
            if (std::isnan(activity) || std::isnan(inaccurate_activity)) {
              ++num_errors;
              max_observed_error = infinity();
              LOG_IF_F(ERROR, log_errors, "NaN value for {}", prettyPrintConstraint(constraint));
              continue;
            }
            // Check bounds.
            if (constraint.indicator_variable() == nullptr
                || std::round(constraint.indicator_variable()->solution_value()) == constraint.indicator_value()) {
              if (constraint.lb() != -infinity()) {
                if (activity < constraint.lb() - tolerance) {
                  ++num_errors;
                  max_observed_error = std::max(max_observed_error, constraint.lb() - activity);
                  LOG_IF_F(
                     ERROR, log_errors, "Activity {} too low for {}", activity, prettyPrintConstraint(constraint));
                } else if (inaccurate_activity < constraint.lb() - tolerance) {
                  LOG_IF_F(WARNING,
                           log_errors,
                           "Activity {}, computed with the (inaccurate) standard sum of its terms, "
                           "is too low for {}",
                           activity,
                           prettyPrintConstraint(constraint));
                }
              }
              if (constraint.ub() != infinity()) {
                if (activity > constraint.ub() + tolerance) {
                  ++num_errors;
                  max_observed_error = std::max(max_observed_error, activity - constraint.ub());
                  LOG_IF_F(
                     ERROR, log_errors, "Activity {} too high for {}", activity, prettyPrintConstraint(constraint));
                } else if (inaccurate_activity > constraint.ub() + tolerance) {
                  LOG_IF_F(WARNING,
                           log_errors,
                           "Activity {}, computed with the (inaccurate) standard sum of its terms, "
                           "is too high for {}",
                           activity,
                           prettyPrintConstraint(constraint));
                }
              }
            }
          }

          // Verify that the objective value wasn't reported incorrectly.
          const MPObjective&        obj = objective();
          nt::AccurateSum< double > obj_sum;
          obj_sum.add(obj.offset());
          double inaccurate_objective_value = obj.offset();
          for (const auto& entry: obj._coefficients) {
            const double term = entry.first->solution_value() * entry.second;
            obj_sum.add(term);
            inaccurate_objective_value += term;
          }
          const double actual_objective_value = obj_sum.value();
          if (!details::AreWithinAbsoluteOrRelativeTolerances(
                 obj.value(), actual_objective_value, tolerance, tolerance)) {
            ++num_errors;
            max_observed_error = std::max(max_observed_error, fabs(actual_objective_value - obj.value()));
            LOG_IF_F(ERROR,
                     log_errors,
                     "objective value {} isn't accurate, it should be {} (delta= {}).",
                     obj.value(),
                     actual_objective_value,
                     actual_objective_value - obj.value());
          } else if (!details::AreWithinAbsoluteOrRelativeTolerances(
                        obj.value(), inaccurate_objective_value, tolerance, tolerance)) {
            LOG_IF_F(WARNING,
                     log_errors,
                     "objective value {} doesn't correspond"
                     " to the value computed with the standard (and therefore inaccurate)"
                     " sum of its terms.",
                     obj.value());
          }
          if (num_errors > 0) {
            LOG_IF_F(ERROR,
                     log_errors,
                     "There were {} errors above the tolerance ({}), the largest was {}",
                     num_errors,
                     tolerance,
                     max_observed_error);
            return false;
          }
          return true;
        }

        /**
         * Advanced usage: resets extracted model to solve from scratch.
         *
         * This won't reset the parameters that were set with
         * setSolverSpecificParametersAsString() or set_time_limit() or even clear the
         * linear program. It will just make sure that next solve() will be as if
         * everything was reconstructed from scratch.
         */
        void reset() { getInterface()->_impl_Reset(); }

        /** Interrupts the solve() execution to terminate processing if possible.
         *
         * If the underlying interface supports interruption; it does that and returns
         * true regardless of whether there's an ongoing solve() or not. The solve()
         * call may still linger for a while depending on the conditions.  If
         * interruption is not supported; returns false and does nothing.
         */
        bool interruptSolve() { return getInterface()->_impl_InterruptSolve(); }

        /**
         * Resets values of out of bound variables to the corresponding bound and
         * returns an error if any of the variables have NaN value.
         */
        bool clampSolutionWithinBounds() {
          extractModel();
          for (MPVariable* const variable: _variables) {
            const double value = variable->solution_value();
            if (std::isnan(value)) {
              LOG_F(ERROR, "NaN value for {}", prettyPrintVar(*variable));
              return false;
            }
            if (value < variable->lb()) {
              variable->set_solution_value(variable->lb());
            } else if (value > variable->ub()) {
              variable->set_solution_value(variable->ub());
            }
          }
          _sync_status = SynchronizationStatus::SOLUTION_SYNCHRONIZED;
          return true;
        }

        /**
         * Shortcuts to the homonymous MPModelProtoExporter methods, via exporting to
         * a MPModelProto with ExportModelToProto() (see above).
         *
         * Produces empty std::string on portable platforms (e.g. android, ios).
         */
        bool exportModelAsLpFormat(bool obfuscate, std::string* model_str) const {
          assert(false);
          return false;
        }
        bool exportModelAsMpsFormat(bool fixed_format, bool obfuscate, std::string* model_str) const {
          assert(false);
          return false;
        }

        /**
         *  Sets the number of threads to use by the underlying solver.
         *
         * Returns OkStatus if the operation was successful. num_threads must be equal
         * to or greater than 1. Note that the behaviour of this call depends on the
         * underlying solver. E.g., it may set the exact number of threads or the max
         * number of threads (check the solver's interface implementation for
         * details). Also, some solvers may not (yet) support this function, but still
         * enable multi-threading via setSolverSpecificParametersAsString().
         */
        bool setNumThreads(int num_threads) {
          if (num_threads < 1) {
            LOG_F(ERROR, "num_threads must be a positive number.");
            return false;
          }
          const bool status = getInterface()->_impl_SetNumThreads(num_threads);
          if (status) { _num_threads = num_threads; }
          return status;
        }

        /** Returns the number of threads to be used during solve. */
        int getNumThreads() const { return _num_threads; }

        /**
         * Advanced usage: pass solver specific parameters in text format.
         *
         * The format is solver-specific and is the same as the corresponding solver
         * configuration file format. Returns true if the operation was successful.
         */
        // Pass solver specific parameters in text format. The format is
        // solver-specific and is the same as the corresponding solver configuration
        // file format. Returns true if the operation was successful.
        //
        // The default implementation of this method stores the parameters in a
        // temporary file and calls ReadParameterFile to import the parameter file
        // into the solver. Solvers that support passing the parameters directly can
        // override this method to skip the temporary file logic.
        bool setSolverSpecificParametersAsString(const std::string& parameters) {
          _solver_specific_parameter_string = parameters;
          return getInterface()->_impl_SetSolverSpecificParametersAsString(parameters);
        }

        std::string getSolverSpecificParametersAsString() const { return _solver_specific_parameter_string; }

        /**
         * Sets a hint for solution.
         *
         * If a feasible or almost-feasible solution to the problem is already known,
         * it may be helpful to pass it to the solver so that it can be used. A solver
         * that supports this feature will try to use this information to create its
         * initial feasible solution.
         *
         * Note: It may not always be faster to give a hint like this to the
         * solver. There is also no guarantee that the solver will use this hint or
         * try to return a solution "close" to this assignment in case of multiple
         * optimal solutions.
         */
        void setHint(nt::TrivialDynamicArray< nt::Pair< const MPVariable*, double > > hint) {
          for (const auto& var_value_pair: hint) {
            CHECK_F(ownsVariable(var_value_pair.first), "hint variable does not belong to this solver");
          }
          _solution_hint = std::move(hint);
        }

        /**
         * Advanced usage: Incrementality.
         *
         * This function takes a starting basis to be used in the next LP solve()
         * call. The statuses of a current solution can be retrieved via the
         * basis_status() function of a MPVariable or a MPConstraint.
         *
         * WARNING: With Glop, you should disable presolve when using this because
         * this information will not be modified in sync with the presolve and will
         * likely not mean much on the presolved problem.
         */
        void setStartingLpBasis(nt::ArrayView< BasisStatus > variable_statuses,
                                nt::ArrayView< BasisStatus > constraint_statuses) {
          getInterface()->_impl_SetStartingLpBasis(variable_statuses, constraint_statuses);
        }

        /**
         * Controls (or queries) the amount of output produced by the underlying
         * solver. The output can surface to LOGs, or to stdout or stderr, depending
         * on the implementation. The amount of output will greatly vary with each
         * implementation and each problem.
         *
         * Output is suppressed by default.
         */
        bool outputIsEnabled() const { return !quiet(); }

        /** @brief Enable solver logging */
        void enableOutput() { set_quiet(false); }

        /** @brief Suppress solver logging */
        void suppressOutput() { set_quiet(true); }

        /** @brief Get time limit in milliseconds */
        int64_t timeLimit() const { return _time_limit; }

        /**
         * @brief Set time limit
         * @param time_limit Time limit in milliseconds
         */
        void setTimeLimit(int64_t time_limit) {
          DCHECK_GE_F(time_limit, 0);
          _time_limit = time_limit;
        }

        // absl::Duration DurationSinceConstruction() const { return absl::Now() -
        // construction_time_; }

        /**
         * @brief Simplex iteration count
         * @return Number of iterations (LP only)
         */
        int64_t iterations() const { return getInterface()->_impl_iterations(); }

        /**
         * @brief Branch-and-bound nodes explored
         *
         * @return Node count (MIP only)
         *
         * Only available for discrete problems.
         */
        int64_t nodes() const { return getInterface()->_impl_nodes(); }

        /**
         * @brief Solver name and version
         * @return Version string
         */
        std::string solverVersion() const { return getInterface()->_impl_SolverVersion(); }

        MPSolverInterface*       getInterface() { return static_cast< MPSolverInterface* >(this); }
        const MPSolverInterface* getInterface() const { return static_cast< const MPSolverInterface* >(this); }

        /**
         * Advanced usage: returns the underlying solver.
         *
         * Returns the underlying solver so that the user can use solver-specific
         * features or features that are not exposed in the simple API of MPSolver.
         * This method is for advanced users, use at your own risk! In particular, if
         * you modify the model or the solution by accessing the underlying solver
         * directly, then the underlying solver will be out of sync with the
         * information kept in the wrapper (MPSolver, MPVariable, MPConstraint,
         * MPObjective). You need to cast the void* returned back to its original type
         * that depends on the interface (CBC: OsiClpSolverInterface*, CLP:
         * ClpSimplex*, GLPK: glp_prob*, SCIP: SCIP*).
         */
        void* underlying_solver() { return getInterface()->_impl_underlying_solver(); }

        /** Advanced usage: computes the exact condition number of the current scaled
         * basis: L1norm(B) * L1norm(inverse(B)), where B is the scaled basis.
         *
         * This method requires that a basis exists: it should be called after solve.
         * It is only available for continuous problems. It is implemented for GLPK
         * but not CLP because CLP does not provide the API for doing it.
         *
         * The condition number measures how well the constraint matrix is conditioned
         * and can be used to predict whether numerical issues will arise during the
         * solve: the model is declared infeasible whereas it is feasible (or
         * vice-versa), the solution obtained is not optimal or violates some
         * constraints, the resolution is slow because of repeated singularities.
         *
         * The rule of thumb to interpret the condition number kappa is:
         *   - o kappa <= 1e7: virtually no chance of numerical issues
         *   - o 1e7 < kappa <= 1e10: small chance of numerical issues
         *   - o 1e10 < kappa <= 1e13: medium chance of numerical issues
         *   - o kappa > 1e13: high chance of numerical issues
         *
         * The computation of the condition number depends on the quality of the LU
         * decomposition, so it is not very accurate when the matrix is ill
         * conditioned.
         */
        double computeExactConditionNumber() const { return getInterface()->_impl_ComputeExactConditionNumber(); }

        /**
         * Some solvers (MIP only, not LP) can produce multiple solutions to the
         * problem. Returns true when another solution is available, and updates the
         * MPVariable* objects to make the new solution queryable. Call only after
         * calling solve.
         *
         * The optimality properties of the additional solutions found, and whether or
         * not the solver computes them ahead of time or when nextSolution() is called
         * is solver specific.
         *
         * As of 2020-02-10, only Gurobi and SCIP support nextSolution(), see
         * linear_solver_interfaces_test for an example of how to configure these
         * solvers for multiple solutions. Other solvers return false unconditionally.
         */
        // ABSL_MUST_USE_RESULT bool nextSolution();
        bool nextSolution() { return getInterface()->_impl_NextSolution(); }

        // Does not take ownership of "mp_callback".
        //
        template < typename MPCallbackContext >
        void setCallback(MPCallback< MPCallbackContext >* mp_callback) {
          getInterface()->_impl_SetCallback(mp_callback);
        }
        bool supportsCallbacks() const { return getInterface()->_impl_SupportsCallbacks(); }

        // DEPRECATED: Use timeLimit() and setTimeLimit(absl::Duration) instead.
        // NOTE: These deprecated functions used the convention time_limit = 0 to mean
        // "no limit", which now corresponds to _time_limit = NT_INT64_MAX.
        int64_t time_limit() const { return _time_limit == NT_INT64_MAX ? 0 : _time_limit; }
        void    set_time_limit(int64_t time_limit_milliseconds) {
             setTimeLimit(time_limit_milliseconds == 0 ? NT_INT64_MAX : time_limit_milliseconds);
        }
        double time_limit_in_secs() const { return static_cast< double >(time_limit()) / 1000.0; }

        // DEPRECATED: Use DurationSinceConstruction() instead.
        // int64 wall_time() const { return absl::ToInt64Milliseconds(DurationSinceConstruction());
        // }

        // Supports search and loading Gurobi shared library.
        // static bool LoadGurobiSharedLibrary();
        // static void SetGurobiLibraryPath(const std::string& full_library_path);

        // Debugging: verify that the given MPVariable* belongs to this solver.
        bool ownsVariable(const MPVariable* var) const {
          if (var == nullptr) return false;
          if (var->index() >= 0 && var->index() < _variables.size()) {
            // Then, verify that the variable with this index has the same address.
            return _variables[var->index()] == var;
          }
          return false;
        }

        // protected:
        // Computes the size of the constraint with the largest number of
        // coefficients with index in [min_constraint_index,
        // max_constraint_index)
        int computeMaxConstraintSize(int min_constraint_index, int max_constraint_index) const {
          int max_constraint_size = 0;
          CHECK_GE_F(min_constraint_index, 0);
          CHECK_LE_F(max_constraint_index, _constraints.size());
          for (int i = min_constraint_index; i < max_constraint_index; ++i) {
            MPConstraint* const ct = _constraints[i];
            if (ct->_coefficients.size() > max_constraint_size) { max_constraint_size = ct->_coefficients.size(); }
          }
          return max_constraint_size;
        }

        // Returns true if the model has constraints with lower bound > upper bound.
        bool hasInfeasibleConstraints() const {
          bool hasInfeasibleConstraints = false;
          for (int i = 0; i < _constraints.size(); ++i) {
            if (_constraints[i]->lb() > _constraints[i]->ub()) {
              LOG_F(WARNING,
                    "Constraint {} ({}) has contradictory bounds: lower bound = {} upper bound = {}",
                    _constraints[i]->name(),
                    i,
                    _constraints[i]->lb(),
                    _constraints[i]->ub());
              hasInfeasibleConstraints = true;
            }
          }
          return hasInfeasibleConstraints;
        }

        // Returns true if the model has at least 1 integer variable.
        bool hasIntegerVariables() const {
          for (const MPVariable* const variable: _variables) {
            if (variable->integer()) return true;
          }
          return false;
        }

        // Generates the map from variable names to their indices.
        void generateVariableNameIndex() const {
          if (_variable_name_to_index) return;
          _variable_name_to_index = nt::CstringMap< int >();
          for (const MPVariable* const var: _variables)
            _variable_name_to_index->insertNoDuplicate(var->name().c_str(), var->index());
        }

        // Generates the map from constraint names to their indices.
        void generateConstraintNameIndex() const {
          if (_constraint_name_to_index) return;
          _constraint_name_to_index = nt::CstringMap< int >();
          for (const MPConstraint* const cst: _constraints)
            _constraint_name_to_index->insertNoDuplicate(cst->name().c_str(), cst->index());
        }

        // Checks licenses for commercial solver, and checks shared library loading
        // for or-tools.
        static bool gurobiIsCorrectlyInstalled();


        //---------------------------------------------------------------------------------------
        // Implementation specific methods
        //---------------------------------------------------------------------------------------

        // Indicates whether the model and the solution are synchronized.
        SynchronizationStatus _sync_status;

        // Indicates whether the solve has reached optimality,
        // infeasibility, a limit, etc.
        ResultStatus _result_status;

        // Optimization direction.
        bool _maximize;

        // Index in MPSolver::_variables of last constraint extracted.
        int _last_constraint_index;
        // Index in MPSolver::_constraints of last variable extracted.
        int _last_variable_index;

        // The value of the objective function.
        double _objective_value;

        // The value of the best objective bound. Used only for MIP solvers.
        double _best_objective_bound;

        // Boolean indicator for the verbosity of the solver output.
        bool _quiet;

        // Extracts model stored in MPSolver.
        void extractModel() {
          switch (_sync_status) {
            case SynchronizationStatus::MUST_RELOAD: {
              extractNewVariables();
              _last_variable_index = _variables.size();
              extractNewConstraints();
              _last_constraint_index = _constraints.size();
              extractObjective();
              _sync_status = SynchronizationStatus::MODEL_SYNCHRONIZED;
              break;
            }
            case SynchronizationStatus::MODEL_SYNCHRONIZED: {
              // Everything has already been extracted.
              DCHECK_EQ_F(_last_constraint_index, _constraints.size());
              DCHECK_EQ_F(_last_variable_index, _variables.size());
              break;
            }
            case SynchronizationStatus::SOLUTION_SYNCHRONIZED: {
              // Nothing has changed since last solve.
              DCHECK_EQ_F(_last_constraint_index, _constraints.size());
              DCHECK_EQ_F(_last_variable_index, _variables.size());
              break;
            }
          }
        }

        // Extracts the variables that have not been extracted yet.
        void extractNewVariables() { getInterface()->_impl_ExtractNewVariables(); }
        // Extracts the constraints that have not been extracted yet.
        void extractNewConstraints() { getInterface()->_impl_ExtractNewConstraints(); }
        // Extracts the objective.
        void extractObjective() { getInterface()->_impl_ExtractObjective(); }

        // Resets the extraction information.
        // TODO(user): remove this method.
        void resetExtractionInformation() {
          _sync_status = SynchronizationStatus::MUST_RELOAD;
          _last_constraint_index = 0;
          _last_variable_index = 0;
          _variable_is_extracted.assign(_variables.size(), false);
          _constraint_is_extracted.assign(_constraints.size(), false);
        }

        // Checks whether the solution is synchronized with the model, i.e. whether
        // the model has changed since the solution was computed last.
        // If it isn't, it crashes in NDEBUG, and returns false othwerwise.
        bool checkSolutionIsSynchronized() const {
          if (_sync_status != SynchronizationStatus::SOLUTION_SYNCHRONIZED) {
            LOG_F(WARNING,
                  "The model has been changed since the solution was last computed. "
                  "MPSolverInterface::_sync_status = {:d}",
                  static_cast< int >(_sync_status));
            return false;
          }
          return true;
        }
        // Checks whether a feasible solution exists. The behavior is similar to
        // checkSolutionIsSynchronized() above.
        // Default version that can be overwritten by a solver-specific
        // version to accommodate for the quirks of each solver.
        bool checkSolutionExists() const {
          if (_result_status != ResultStatus::OPTIMAL && _result_status != ResultStatus::FEASIBLE) {
            LOG_F(WARNING,
                  "No solution exists. MPSolverInterface::_result_status = {:d}",
                  static_cast< int >(_result_status));
            return false;
          }
          return true;
        }

        // Handy shortcut to do both checks above (it is often used).
        bool checkSolutionIsSynchronizedAndExists() const {
          return checkSolutionIsSynchronized() && checkSolutionExists();
        }

        // Change synchronization status from SOLUTION_SYNCHRONIZED to
        // MODEL_SYNCHRONIZED. To be used for model changes.
        void invalidateSolutionSynchronization() {
          if (_sync_status == SynchronizationStatus::SOLUTION_SYNCHRONIZED) {
            _sync_status = SynchronizationStatus::MODEL_SYNCHRONIZED;
          }
        }

        // Sets parameters common to LP and MIP in the underlying solver.
        void setCommonParameters(const MPSolverParameters& param) {
          // TODO(user): Overhaul the code that sets parameters to enable changing
          // GLOP parameters without issuing warnings.
          // By default, we let GLOP keep its own default tolerance, much more accurate
          // than for the rest of the solvers.
          //
          // if (ProblemType() != GLOP_LINEAR_PROGRAMMING) {
          setPrimalTolerance(param.getDoubleParam(MPSolverParameters::PRIMAL_TOLERANCE));
          setDualTolerance(param.getDoubleParam(MPSolverParameters::DUAL_TOLERANCE));
          // }
          setPresolveMode(param.getIntegerParam(MPSolverParameters::PRESOLVE));
          // TODO(user): In the future, we could distinguish between the
          // algorithm to solve the root LP and the algorithm to solve node
          // LPs. Not sure if underlying solvers support it.
          int value = param.getIntegerParam(MPSolverParameters::LP_ALGORITHM);
          if (value != MPSolverParameters::kDefaultIntegerParamValue) { setLpAlgorithm(value); }
        }

        // Sets MIP specific parameters in the underlying solver.
        void setMIPParameters(const MPSolverParameters& param) {
          // if (ProblemType() != GLOP_LINEAR_PROGRAMMING) {
          // setRelativeMipGap(param.getDoubleParam(MPSolverParameters::RELATIVE_MIP_GAP)); }
          setRelativeMipGap(param.getDoubleParam(MPSolverParameters::RELATIVE_MIP_GAP));
        }

        // Sets all parameters in the underlying solver.
        void setParameters(const MPSolverParameters& param) { getInterface()->_impl_SetParameters(param); }

        // Sets an unsupported double parameter.
        void setUnsupportedDoubleParam(MPSolverParameters::DoubleParam param) {
          LOG_F(WARNING, "Trying to set an unsupported parameter: {}.", static_cast< int >(param));
        }
        // Sets an unsupported integer parameter.
        void setUnsupportedIntegerParam(MPSolverParameters::IntegerParam param) {
          LOG_F(WARNING, "Trying to set an unsupported parameter: {}.", static_cast< int >(param));
        }
        // Sets a supported double parameter to an unsupported value.
        void setDoubleParamToUnsupportedValue(MPSolverParameters::DoubleParam param, double value) {
          LOG_F(WARNING,
                "Trying to set a supported parameter: {} to an unsupported value: {}",
                static_cast< int >(param),
                value);
        }
        // Sets a supported integer parameter to an unsupported value.
        void setIntegerParamToUnsupportedValue(MPSolverParameters::IntegerParam param, int value) {
          LOG_F(WARNING,
                "Trying to set a supported parameter: {} to an unsupported value: {}",
                static_cast< int >(param),
                value);
        }

        // Sets each parameter in the underlying solver.
        void setRelativeMipGap(double value) { getInterface()->_impl_SetRelativeMipGap(value); }
        void setPrimalTolerance(double value) { getInterface()->_impl_SetPrimalTolerance(value); }
        void setDualTolerance(double value) { getInterface()->_impl_SetDualTolerance(value); }
        void setPresolveMode(int value) { getInterface()->_impl_SetPresolveMode(value); }

        /**
         * @brief Best objective bound
         *
         * @return Lower bound (minimize) or upper bound (maximize)
         * on optimal solution value
         *
         * @warning MIP only. Crashes or returns ±∞ for LP.
         */
        double best_objective_bound() const {
          const double trivial_worst_bound = _maximize ? -NT_DOUBLE_INFINITY : NT_DOUBLE_INFINITY;
          if (!isMIP()) [[unlikely]] {
            LOG_F(FATAL, "Best objective bound only available for discrete problems.");
            return trivial_worst_bound;
          }
          if (!checkSolutionIsSynchronized()) [[unlikely]] { return trivial_worst_bound; }
          // Special case for empty model.
          if (_variables.empty() && _constraints.empty()) { return objective().offset(); }
          return _best_objective_bound;
        }

        /**
         * @brief objective value of current solution
         *
         * @return objective value (0 if no solution exists)
         *
         * Call after solve(). Returns best solution found.
         */
        double objective_value() const {
          if (!checkSolutionIsSynchronizedAndExists()) [[unlikely]]
            return 0;
          return _objective_value;
        }

        /** @brief Index of last extracted variable */
        int last_variable_index() const { return _last_variable_index; }

        /** @brief Check if variable extracted to solver */
        bool variable_is_extracted(int var_index) const { return _variable_is_extracted[var_index]; }
        /** @brief Mark variable as extracted */
        void set_variable_as_extracted(int var_index, bool extracted) { _variable_is_extracted[var_index] = extracted; }
        /** @brief Check if constraint extracted to solver */
        bool constraint_is_extracted(int ct_index) const { return _constraint_is_extracted[ct_index]; }
        /** @brief Mark constraint as extracted */
        void set_constraint_as_extracted(int ct_index, bool extracted) {
          _constraint_is_extracted[ct_index] = extracted;
        }

        /** @brief Is solver output suppressed? */
        bool quiet() const { return _quiet; }
        /** @brief Set solver output verbosity */
        void set_quiet(bool quiet_value) { _quiet = quiet_value; }

        /**
         * @brief Result status of last solve()
         * @return Status (OPTIMAL, FEASIBLE, INFEASIBLE, etc.)
         */
        ResultStatus result_status() const {
          checkSolutionIsSynchronized();
          return _result_status;
        }

        // Sets the scaling mode.
        void setScalingMode(int value) { getInterface()->_impl_SetScalingMode(value); }
        void setLpAlgorithm(int value) { getInterface()->_impl_SetLpAlgorithm(value); }

        static void
           appendTerm(const double coef, const std::string& var_name, const bool is_first, nt::MemoryBuffer* s) {
          if (is_first) {
            if (coef == 1.0) {
              s->append(var_name);
            } else if (coef == -1.0) {
              nt::format_to(std::back_inserter(*s), " -{}", var_name);
            } else {
              nt::format_to(std::back_inserter(*s), " {}*{}", coef, var_name);
            }
          } else {
            const std::string op = coef < 0 ? "-" : "+";
            const double      abs_coef = std::abs(coef);
            if (abs_coef == 1.0) {
              nt::format_to(std::back_inserter(*s), " {} {}", op, var_name);
            } else {
              nt::format_to(std::back_inserter(*s), " {} {} * {}", op, abs_coef, var_name);
            }
          }
        }

        static bool appendAllTerms(const nt::PointerMap< const MPVariable*, double >& Terms, nt::MemoryBuffer* s) {
          nt::TrivialDynamicArray< const MPVariable* > vars_in_order;
          for (const auto& var_val_pair: Terms) {
            vars_in_order.push_back(var_val_pair.first);
          }
          nt::sort(vars_in_order, [](const MPVariable* v, const MPVariable* u) { return v->index() < u->index(); });
          bool is_first = true;
          for (const MPVariable* var: vars_in_order) {
            // MPSolver gives names to all variables, even if you don't.
            DCHECK_F(!var->name().empty());
            appendTerm(Terms.findWithDefault(var, NT_DOUBLE_NAN), var->name(), is_first, s);
            is_first = false;
          }

          return is_first;
        }

        static void appendOffset(const double offset, const bool is_first, nt::MemoryBuffer* s) {
          if (is_first) {
            nt::format_to(std::back_inserter(*s), "{}", offset);
          } else {
            if (offset != 0.0) {
              const std::string op = offset < 0 ? "-" : "+";
              nt::format_to(std::back_inserter(*s), " {} {}", op, std::abs(offset));
            }
          }
        }

        static std::string toString(MPCallbackEvent event) {
          switch (event) {
            case MPCallbackEvent::kMipSolution: return "MIP_SOLUTION";
            case MPCallbackEvent::kMip: return "MIP";
            case MPCallbackEvent::kMipNode: return "MIP_NODE";
            case MPCallbackEvent::kBarrier: return "BARRIER";
            case MPCallbackEvent::kMessage: return "MESSAGE";
            case MPCallbackEvent::kPresolve: return "PRESOLVE";
            case MPCallbackEvent::kPolling: return "POLLING";
            case MPCallbackEvent::kMultiObj: return "MULTI_OBJ";
            case MPCallbackEvent::kSimplex: return "SIMPLEX";
            case MPCallbackEvent::kUnknown: return "UNKNOWN";
            default: LOG_F(FATAL, "Unrecognized callback event: %d ", static_cast< int >(event));
          }
          return "";
        }
      };
    }   // namespace details
  }     // namespace mp
}   // namespace nt

#endif
