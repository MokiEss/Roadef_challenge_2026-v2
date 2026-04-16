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
 * @file mp_gurobi.h
 * @brief Gurobi solver integration for networktools MP module
 *
 * This header enables Gurobi Optimizer support for LP and MIP problems.
 * Include this file after networktools.h to use Gurobi solver.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 *
 * @par Usage:
 * @code
 * #include "networktools.h"
 * #include "mp/mp_gurobi.h"
 *
 * nt::mp::gurobi::MIPModel model;
 * @endcode
 *
 * @par Features:
 * - Full LP and MIP support with Gurobi's powerful algorithms
 * - Incremental extraction for faster repeated solves
 * - Multi-threading support
 * - Advanced basis management and warm starts
 * - Farkas dual values for infeasibility analysis
 *
 * @par Requirements:
 * - Gurobi 9.0 or newer recommended
 * - Link with Gurobi libraries (-lgurobi_c++ -lgurobiXX)
 * - Valid Gurobi license
 *
 * @see mp.h for API documentation
 * @see README.md for compilation examples
 */

// This file contains a modified version of the source code from Google OR-Tools linear programming module.
// See the appropriate copyright notice below.

// Copyright 2010-2024 Google LLC
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Gurobi backend to MPSolver.
//
// Implementation Notes:
//
// Incrementalism (last updated June 29, 2020): For solving both LPs and MIPs,
// Gurobi attempts to reuse information from previous solves, potentially
// giving a faster solve time. MPSolver supports this for the following problem
// modification types:
//   * Adding a variable,
//   * Adding a linear constraint,
//   * Updating a variable bound,
//   * Updating an objective coefficient or the objective offset (note that in
//     Gurobi 7.5 LP solver, there is a bug if you update only the objective
//     offset and nothing else).
//   * Updating a coefficient in the constraint matrix.
//   * Updating the type of variable (integer, continuous)
//   * Changing the optimization direction.
// Updates of the following types will force a resolve from scratch:
//   * Updating the upper or lower bounds of a linear constraint. Note that in
//     MPSolver's model, this includes updating the sense (le, ge, eq, range) of
//     a linear constraint.
//   * Clearing a constraint
// Any model containing indicator constraints is considered "non-incremental"
// and will always solve from scratch.
//
// The above limitations are largely due MPSolver and this file, not Gurobi.
//
// Warning(rander): the interactions between callbacks and incrementalism are
// poorly tested, proceed with caution.

#ifndef _NT_GUROBI_INTERFACE_H_
#define _NT_GUROBI_INTERFACE_H_

#include "bits/mp_solver_base.h"

extern "C" {
#include "gurobi_c.h"
}

#if defined(_MSC_VER)
#  define GUROBI_STDCALL __stdcall
#else
#  define GUROBI_STDCALL
#endif

namespace nt {
  namespace mp {
    namespace gurobi {

      struct MPEnv {
        GRBenv* _env;

        MPEnv() noexcept {
          // Create empty environment first to set parameters before activation
          int error = GRBemptyenv(&_env);
          if (error) {
            LOG_F(ERROR, "Could not create Gurobi environment: {}", GRBgeterrormsg(_env));
            _env = nullptr;
            return;
          }
          
          // Disable output to suppress license messages
          error = GRBsetintparam(_env, GRB_INT_PAR_OUTPUTFLAG, 0);
          if (error) {
            LOG_F(WARNING, "Could not disable Gurobi output: {}", GRBgeterrormsg(_env));
          }
          
          // Activate the environment
          error = GRBstartenv(_env);
          if (error) {
            LOG_F(ERROR,
                  "Could not start Gurobi environment: is Gurobi licensed on this machine? {}",
                  GRBgeterrormsg(_env));
            GRBfreeenv(_env);
            _env = nullptr;
          }
          DCHECK_F(_env != nullptr);   // should not be NULL if status=0
        }

        ~MPEnv() noexcept { GRBfreeenv(_env); }

        constexpr bool isValid() const noexcept { return _env != nullptr; }
      };

      namespace details {

        constexpr int kGurobiOkCode = 0;
        inline void   CheckedGurobiCall(int err, GRBenv* const env) {
            CHECK_EQ_F(kGurobiOkCode, err, "Fatal error with code {}, due to {}", err, GRBgeterrormsg(env));
        }

        // For interacting directly with the Gurobi C API for callbacks.
        struct GurobiInternalCallbackContext {
          GRBmodel* model;
          void*     gurobi_internal_callback_data;
          int       where;
        };

        // Returns a human-readable listing of all gurobi parameters that are set to
        // non-default values, and their current value in the given environment. If all
        // parameters are at their default value, returns the empty string.
        // To produce a one-liner string, use `one_liner_output=true`.
        inline std::string GurobiParamInfoForLogging(GRBenv* grb, bool one_liner_output = false) {
#if 0
          const absl::ParsedFormat< 's', 's', 's' >  kExtendedFormat("  Parameter: '%s' value: %s default: %s");
          const absl::ParsedFormat< 's', 's', 's' >  kOneLinerFormat("'%s':%s (%s)");
          const absl::ParsedFormat< 's', 's', 's' >& format = one_liner_output ? kOneLinerFormat : kExtendedFormat;
          std::vector< std::string >                 changed_parameters;
          const int                                  num_parameters = GRBgetnumparams(grb);
          for (int i = 0; i < num_parameters; ++i) {
            char* param_name = nullptr;
            GRBgetparamname(grb, i, &param_name);
            const int param_type = GRBgetparamtype(grb, param_name);
            switch (param_type) {
              case 1:   // integer parameters.
              {
                int default_value;
                int min_value;
                int max_value;
                int current_value;
                GRBgetintparaminfo(grb, param_name, &current_value, &min_value, &max_value, &default_value);
                if (current_value != default_value) { changed_parameters.push_back(absl::StrFormat(format, param_name, absl::StrCat(current_value), absl::StrCat(default_value))); }
                break;
              }
              case 2:   // double parameters.
              {
                double default_value;
                double min_value;
                double max_value;
                double current_value;
                GRBgetdblparaminfo(grb, param_name, &current_value, &min_value, &max_value, &default_value);
                if (current_value != default_value) { changed_parameters.push_back(absl::StrFormat(format, param_name, absl::StrCat(current_value), absl::StrCat(default_value))); }
                break;
              }
              case 3:   // string parameters.
              {
                char current_value[GRB_MAX_STRLEN + 1];
                char default_value[GRB_MAX_STRLEN + 1];
                GRBgetstrparaminfo(grb, param_name, current_value, default_value);
                // This ensure that strcmp does not go beyond the end of the char
                // array.
                current_value[GRB_MAX_STRLEN] = '\0';
                default_value[GRB_MAX_STRLEN] = '\0';
                if (std::strcmp(current_value, default_value) != 0) { changed_parameters.push_back(absl::StrFormat(format, param_name, current_value, default_value)); }
                break;
              }
              default:   // unknown parameter types
                changed_parameters.push_back(absl::StrFormat("Parameter '%s' of unknown type %d", param_name, param_type));
            }
          }
          if (changed_parameters.empty()) return "";
          if (one_liner_output) { return absl::StrCat("GurobiParams{", absl::StrJoin(changed_parameters, ", "), "}"); }
          return absl::StrJoin(changed_parameters, "\n");
#else
          return "NotYetImplemented";
#endif
        }

        // ----- Gurobi Solver -----

        template < bool MIP >
        struct GurobiInterface: public nt::mp::details::MPSolver< GurobiInterface< MIP > > {
          using Parent = nt::mp::details::MPSolver< GurobiInterface< MIP > >;
          using MPObjective = Parent::MPObjective;
          using MPConstraint = Parent::MPConstraint;
          using MPVariable = Parent::MPVariable;
          // using MPCallback = Parent::MPCallback;
          using LinearRange = Parent::LinearRange;

          using Parent::Parent;

          struct GurobiMPCallbackContext: Parent::template MPCallbackContextInterface< GurobiMPCallbackContext > {
            GRBenv* const                    env_;
            const nt::IntDynamicArray* const mp_var_to_gurobi_var_;
            const int                        num_gurobi_vars_;

            const bool might_add_cuts_;
            const bool might_add_lazy_constraints_;

            // Stateful, updated before each call to the callback.
            GurobiInternalCallbackContext current_gurobi_internal_callback_context_;
            bool                          variable_values_extracted_ = false;
            nt::DoubleDynamicArray        gurobi_variable_values_;

            GurobiMPCallbackContext(GRBenv*                    env,
                                    const nt::IntDynamicArray* mp_var_to_gurobi_var,
                                    int                        num_gurobi_vars,
                                    bool                       might_add_cuts,
                                    bool                       might_add_lazy_constraints) :
                env_(env),
                mp_var_to_gurobi_var_(mp_var_to_gurobi_var), num_gurobi_vars_(num_gurobi_vars),
                might_add_cuts_(might_add_cuts), might_add_lazy_constraints_(might_add_lazy_constraints) {}

            // Call this method to update the internal state of the callback context
            // before passing it to MPCallback::runCallback().
            void UpdateFromGurobiState(const GurobiInternalCallbackContext& gurobi_internal_context) {
              current_gurobi_internal_callback_context_ = gurobi_internal_context;
              variable_values_extracted_ = false;
            }

            // Wraps GRBcbget(), used to query the state of the solver.  See
            // http://www.gurobi.com/documentation/8.0/refman/callback_codes.html#sec:CallbackCodes
            // for callback_code values.
            template < typename T >
            T GurobiCallbackGet(const GurobiInternalCallbackContext& gurobi_internal_context, const int callback_code) {
              T result = 0;
              nt::mp::gurobi::details::CheckedGurobiCall(GRBcbget(gurobi_internal_context.gurobi_internal_callback_data,
                                                                  gurobi_internal_context.where,
                                                                  callback_code,
                                                                  static_cast< void* >(&result)),
                                                         env_);
              return result;
            }

            template < typename GRBConstraintFunction >
            void AddGeneratedConstraint(const LinearRange&    linear_range,
                                        GRBConstraintFunction grb_constraint_function) {
              nt::IntDynamicArray    variable_indices;
              nt::DoubleDynamicArray variable_coefficients;
              const int              num_terms = linear_range.linear_expr().terms().size();
              variable_indices.reserve(num_terms);
              variable_coefficients.reserve(num_terms);
              assert(mp_var_to_gurobi_var_);
              for (const auto& var_coef_pair: linear_range.linear_expr().terms()) {
                variable_indices.push_back((*mp_var_to_gurobi_var_)[var_coef_pair.first->index()]);
                variable_coefficients.push_back(var_coef_pair.second);
              }
              if (std::isfinite(linear_range.upper_bound())) {
                nt::mp::gurobi::details::CheckedGurobiCall(
                   grb_constraint_function(current_gurobi_internal_callback_context_.gurobi_internal_callback_data,
                                           variable_indices.size(),
                                           variable_indices.data(),
                                           variable_coefficients.data(),
                                           GRB_LESS_EQUAL,
                                           linear_range.upper_bound()),
                   env_);
              }
              if (std::isfinite(linear_range.lower_bound())) {
                nt::mp::gurobi::details::CheckedGurobiCall(
                   grb_constraint_function(current_gurobi_internal_callback_context_.gurobi_internal_callback_data,
                                           variable_indices.size(),
                                           variable_indices.data(),
                                           variable_coefficients.data(),
                                           GRB_GREATER_EQUAL,
                                           linear_range.lower_bound()),
                   env_);
              }
            }

            MPCallbackEvent event() {
              switch (current_gurobi_internal_callback_context_.where) {
                case GRB_CB_POLLING: return MPCallbackEvent::kPolling;
                case GRB_CB_PRESOLVE: return MPCallbackEvent::kPresolve;
                case GRB_CB_SIMPLEX: return MPCallbackEvent::kSimplex;
                case GRB_CB_MIP: return MPCallbackEvent::kMip;
                case GRB_CB_MIPSOL: return MPCallbackEvent::kMipSolution;
                case GRB_CB_MIPNODE: return MPCallbackEvent::kMipNode;
                case GRB_CB_MESSAGE: return MPCallbackEvent::kMessage;
                case GRB_CB_BARRIER:
                  return MPCallbackEvent::kBarrier;
                  // TODO(b/112427356): in Gurobi 8.0, there is a new callback location.
                  // case GRB_CB_MULTIOBJ:
                  //   return MPCallbackEvent::kMultiObj;
                default:
                  LOG_F(ERROR, "Gurobi callback at unknown where= {}", current_gurobi_internal_callback_context_.where);
                  return MPCallbackEvent::kUnknown;
              }
            }

            bool canQueryVariableValues() {
              const MPCallbackEvent where = event();
              if (where == MPCallbackEvent::kMipSolution) { return true; }
              if (where == MPCallbackEvent::kMipNode) {
                const int gurobi_node_status =
                   GurobiCallbackGet< int >(current_gurobi_internal_callback_context_, GRB_CB_MIPNODE_STATUS);
                return gurobi_node_status == GRB_OPTIMAL;
              }
              return false;
            }

            double variableValue(const MPVariable* pVariable) {
              CHECK_F(pVariable != nullptr);
              if (!variable_values_extracted_) {
                const MPCallbackEvent where = event();
                CHECK_F(where == MPCallbackEvent::kMipSolution || where == MPCallbackEvent::kMipNode,
                        "You can only call variableValue at {} or {} but called from: {}",
                        Parent::toString(MPCallbackEvent::kMipSolution),
                        Parent::toString(MPCallbackEvent::kMipNode),
                        Parent::toString(where));
                const int gurobi_get_var_param =
                   where == MPCallbackEvent::kMipNode ? GRB_CB_MIPNODE_REL : GRB_CB_MIPSOL_SOL;

                gurobi_variable_values_.resize(num_gurobi_vars_);
                nt::mp::gurobi::details::CheckedGurobiCall(
                   GRBcbget(current_gurobi_internal_callback_context_.gurobi_internal_callback_data,
                            current_gurobi_internal_callback_context_.where,
                            gurobi_get_var_param,
                            static_cast< void* >(gurobi_variable_values_.data())),
                   env_);
                variable_values_extracted_ = true;
              }
              assert(mp_var_to_gurobi_var_);
              return gurobi_variable_values_[(*mp_var_to_gurobi_var_)[pVariable->index()]];
            }

            void addCut(const LinearRange& cutting_plane) {
              CHECK_F(might_add_cuts_);
              const MPCallbackEvent where = event();
              CHECK_F(where == MPCallbackEvent::kMipNode,
                      "Cuts can only be added at MIP_NODE, tried to add cut at: {}",
                      Parent::toString(where));
              AddGeneratedConstraint(cutting_plane, GRBcbcut);
            }

            void addPricedVariable(const MPVariable* pVariable) {
              LOG_F(FATAL, "addPricedVariable() not yet implemented for Gurobi.");
            }

            void addLazyConstraint(const LinearRange& lazy_constraint) {
              CHECK_F(might_add_lazy_constraints_);
              const MPCallbackEvent where = event();
              CHECK_F(where == MPCallbackEvent::kMipNode || where == MPCallbackEvent::kMipSolution,
                      "Lazy constraints can only be added at MIP_NODE or MIP_SOL, tried to "
                      "add lazy constraint at: {}",
                      Parent::toString(where));
              AddGeneratedConstraint(lazy_constraint, GRBcblazy);
            }

            double suggestSolution(const nt::PointerMap< const MPVariable*, double >& solution) {
              const MPCallbackEvent where = event();
              CHECK_F(where == MPCallbackEvent::kMipNode,
                      "Feasible solutions can only be added at MIP_NODE, tried to add "
                      "solution at: ",
                      Parent::toString(where));

              nt::DoubleDynamicArray full_solution;
              full_solution.ensureAndFill(num_gurobi_vars_, GRB_UNDEFINED);
              for (const auto& variable_value: solution) {
                const MPVariable* var = variable_value.first;
                assert(mp_var_to_gurobi_var_);
                full_solution[(*mp_var_to_gurobi_var_)[var->index()]] = variable_value.second;
              }

              double objval;
              nt::mp::gurobi::details::CheckedGurobiCall(
                 GRBcbsolution(current_gurobi_internal_callback_context_.gurobi_internal_callback_data,
                               full_solution.data(),
                               &objval),
                 env_);

              return objval;
            }

            int64_t numExploredNodes() {
              switch (event()) {
                case MPCallbackEvent::kMipNode:
                  return static_cast< int64_t >(
                     GurobiCallbackGet< double >(current_gurobi_internal_callback_context_, GRB_CB_MIPNODE_NODCNT));
                case MPCallbackEvent::kMipSolution:
                  return static_cast< int64_t >(
                     GurobiCallbackGet< double >(current_gurobi_internal_callback_context_, GRB_CB_MIPSOL_NODCNT));
                default:
                  LOG_F(
                     FATAL,
                     "Node count is supported only for callback events MIP_NODE and MIP_SOL, but was requested at: {}",
                     toString(event()));
              }
            }
          };

          using MPCallbackContext = GurobiMPCallbackContext;
          using MPCallback = Parent::template MPCallback< MPCallbackContext >;
          using MPCallbackList = Parent::template MPCallbackList< MPCallbackContext >;

          struct MPCallbackWithGurobiContext {
            GurobiMPCallbackContext* context;
            MPCallback*              callback;
          };

          // NOTE(user): This function must have this exact API, because we are passing
          // it to Gurobi as a callback.
          static int GUROBI_STDCALL CallbackImpl(GRBmodel* model,
                                                 void*     gurobi_internal_callback_data,
                                                 int       where,
                                                 void*     raw_model_and_callback) {
            MPCallbackWithGurobiContext* const callback_with_context =
               static_cast< MPCallbackWithGurobiContext* >(raw_model_and_callback);
            CHECK_F(callback_with_context != nullptr);
            CHECK_F(callback_with_context->context != nullptr);
            CHECK_F(callback_with_context->callback != nullptr);
            GurobiInternalCallbackContext gurobi_internal_context{model, gurobi_internal_callback_data, where};
            callback_with_context->context->UpdateFromGurobiState(gurobi_internal_context);
            callback_with_context->callback->runCallback(callback_with_context->context);
            return 0;
          }

          // Constructor that takes a name for the underlying gurobi solver.
          GurobiInterface(const std::string& name, const MPEnv& e);
          ~GurobiInterface();

          // ----- Gurobi helpers -----

          // See the implementation note at the top of file on incrementalism.
          bool ModelIsNonincremental() const noexcept {
            for (const MPConstraint* c: Parent::constraints())
              if (c->indicator_variable() != nullptr) return true;
            return false;
          }

          int  SolutionCount() const noexcept { return GetIntAttr(GRB_INT_ATTR_SOLCOUNT); }
          void CheckedGurobiCall(int err) const noexcept {
            nt::mp::gurobi::details::CheckedGurobiCall(err, global_env_);
          }
          void SetIntAttr(const char* name, int value) noexcept {
            CheckedGurobiCall(GRBsetintattr(model_, name, value));
          }
          int GetIntAttr(const char* name) const noexcept {
            int value;
            CheckedGurobiCall(GRBgetintattr(model_, name, &value));
            return value;
          }
          void SetDoubleAttr(const char* name, double value) noexcept {
            CheckedGurobiCall(GRBsetdblattr(model_, name, value));
          }
          double GetDoubleAttr(const char* name) const noexcept {
            double value;
            CheckedGurobiCall(GRBgetdblattr(model_, name, &value));
            return value;
          }
          void SetIntAttrElement(const char* name, int index, int value) noexcept {
            CheckedGurobiCall(GRBsetintattrelement(model_, name, index, value));
          }
          int GetIntAttrElement(const char* name, int index) const noexcept {
            int value;
            CheckedGurobiCall(GRBgetintattrelement(model_, name, index, &value));
            return value;
          }
          void SetDoubleAttrElement(const char* name, int index, double value) noexcept {
            CheckedGurobiCall(GRBsetdblattrelement(model_, name, index, value));
          }
          double GetDoubleAttrElement(const char* name, int index) const noexcept {
            double value;
            CheckedGurobiCall(GRBgetdblattrelement(model_, name, index, &value));
            return value;
          }
          nt::DoubleDynamicArray GetDoubleAttrArray(const char* name, int elements) noexcept {
            nt::DoubleDynamicArray results(elements);
            CheckedGurobiCall(GRBgetdblattrarray(model_, name, 0, elements, results.data()));
            return results;
          }
          void SetCharAttrElement(const char* name, int index, char value) noexcept {
            CheckedGurobiCall(GRBsetcharattrelement(model_, name, index, value));
          }
          char GetCharAttrElement(const char* name, int index) const noexcept {
            char value;
            CheckedGurobiCall(GRBgetcharattrelement(model_, name, index, &value));
            return value;
          }

          BasisStatus TransformGRBConstraintBasisStatus(int gurobi_basis_status, int constraint_index) const {
            const int grb_index = mp_cons_to_gurobi_linear_cons_[constraint_index];
            if (grb_index < 0) {
              LOG_F(FATAL, "Basis status not available for nonlinear constraints.");
              return BasisStatus::FREE;
            }
            switch (gurobi_basis_status) {
              case GRB_BASIC: return BasisStatus::BASIC;
              default: {
                // Non basic.
                double tolerance = 0.0;
                CheckedGurobiCall(GRBgetdblparam(GRBgetenv(model_), GRB_DBL_PAR_FEASIBILITYTOL, &tolerance));
                const double slack = GetDoubleAttrElement(GRB_DBL_ATTR_SLACK, grb_index);
                const char   sense = GetCharAttrElement(GRB_CHAR_ATTR_SENSE, grb_index);
                VLOG_F(4, "constraint {} , slack = {} , sense = {}", constraint_index, slack, sense);
                if (fabs(slack) <= tolerance) {
                  switch (sense) {
                    case GRB_EQUAL:
                    case GRB_LESS_EQUAL: return BasisStatus::AT_UPPER_BOUND;
                    case GRB_GREATER_EQUAL: return BasisStatus::AT_LOWER_BOUND;
                    default: return BasisStatus::FREE;
                  }
                } else {
                  return BasisStatus::FREE;
                }
              }
            }
          }

          BasisStatus TransformGRBVarBasisStatus(int gurobi_basis_status) const {
            switch (gurobi_basis_status) {
              case GRB_BASIC: return BasisStatus::BASIC;
              case GRB_NONBASIC_LOWER: return BasisStatus::AT_LOWER_BOUND;
              case GRB_NONBASIC_UPPER: return BasisStatus::AT_UPPER_BOUND;
              case GRB_SUPERBASIC: return BasisStatus::FREE;
              default: LOG_F(FATAL, "Unknown GRB basis status."); return BasisStatus::FREE;
            }
          }

          // Sets the optimization direction (min/max).
          void _impl_SetOptimizationDirection(bool maximize);

          // ----- Solve -----
          // Solve the problem using the parameter values specified.
          ResultStatus _impl_Solve(const MPSolverParameters& param);

          // ----- Model modifications and extraction -----
          // Resets extracted model
          void _impl_Reset();

          // Modify bounds.
          void _impl_SetVariableBounds(int mpsolver_var_index, double lb, double ub) {
            Parent::invalidateSolutionSynchronization();
            if (!had_nonincremental_change_ && Parent::variable_is_extracted(mpsolver_var_index)) {
              SetDoubleAttrElement(GRB_DBL_ATTR_LB, mp_var_to_gurobi_var_[mpsolver_var_index], lb);
              SetDoubleAttrElement(GRB_DBL_ATTR_UB, mp_var_to_gurobi_var_[mpsolver_var_index], ub);
            } else {
              Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
            }
          }
          void _impl_SetVariableInteger(int mpsolver_var_index, bool integer) {
            Parent::invalidateSolutionSynchronization();
            if (!had_nonincremental_change_ && Parent::variable_is_extracted(mpsolver_var_index)) {
              char type_var;
              if (integer) {
                type_var = GRB_INTEGER;
              } else {
                type_var = GRB_CONTINUOUS;
              }
              SetCharAttrElement(GRB_CHAR_ATTR_VTYPE, mp_var_to_gurobi_var_[mpsolver_var_index], type_var);
            } else {
              Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
            }
          }
          void _impl_SetConstraintBounds(int mpsolver_constraint_index, double lb, double ub) {
            Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
            if (Parent::constraint_is_extracted(mpsolver_constraint_index)) { had_nonincremental_change_ = true; }
            // TODO(user): this is nontrivial to make incremental:
            //   1. Make sure it is a linear constraint (not an indicator or indicator
            //      range constraint).
            //   2. Check if the sense of the constraint changes. If it was previously a
            //      range constraint, we can do nothing, and if it becomes a range
            //      constraint, we can do nothing. We could support range constraints if
            //      we tracked the auxiliary variable that is added with range
            //      constraints.
          }

          // Add Constraint incrementally.
          void _impl_AddRowConstraint(MPConstraint* const ct) {
            Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
          }
          // Add variable incrementally.
          void _impl_AddVariable(MPVariable* const var) { Parent::_sync_status = SynchronizationStatus::MUST_RELOAD; }
          // Change a coefficient in a constraint.
          void _impl_SetCoefficient(MPConstraint* const     constraint,
                                    const MPVariable* const variable,
                                    double                  new_value,
                                    double                  old_value) {
            Parent::invalidateSolutionSynchronization();
            if (!had_nonincremental_change_ && Parent::variable_is_extracted(variable->index())
                && Parent::constraint_is_extracted(constraint->index())) {
              // Cannot be const, GRBchgcoeffs needs non-const pointer.
              int grb_var = mp_var_to_gurobi_var_[variable->index()];
              int grb_cons = mp_cons_to_gurobi_linear_cons_[constraint->index()];
              if (grb_cons < 0) {
                had_nonincremental_change_ = true;
                Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
              } else {
                // TODO(user): investigate if this has bad performance.
                CheckedGurobiCall(GRBchgcoeffs(model_, 1, &grb_cons, &grb_var, &new_value));
              }
            } else {
              Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
            }
          }
          // Clear a constraint from all its terms.
          void _impl_ClearConstraint(MPConstraint* const constraint) {
            had_nonincremental_change_ = true;
            Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
            // TODO(user): this is difficult to make incremental, like
            //  SetConstraintBounds(), because of the auxiliary Gurobi variables that
            //  range constraints introduce.
          }
          // Change a coefficient in the linear objective
          void _impl_SetObjectiveCoefficient(const MPVariable* const variable, double coefficient) {
            Parent::invalidateSolutionSynchronization();
            if (!had_nonincremental_change_ && Parent::variable_is_extracted(variable->index())) {
              SetDoubleAttrElement(GRB_DBL_ATTR_OBJ, mp_var_to_gurobi_var_[variable->index()], coefficient);
            } else {
              Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
            }
          }
          // Change the constant term in the linear objective.
          void _impl_SetObjectiveOffset(double value) {
            Parent::invalidateSolutionSynchronization();
            if (!had_nonincremental_change_) {
              SetDoubleAttr(GRB_DBL_ATTR_OBJCON, value);
            } else {
              Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
            }
          }
          // Clear the objective from all its terms.
          void _impl_ClearObjective() {
            Parent::invalidateSolutionSynchronization();
            if (!had_nonincremental_change_) {
              _impl_SetObjectiveOffset(0.0);
              for (const auto& entry: Parent::_objective._coefficients) {
                _impl_SetObjectiveCoefficient(entry.first, 0.0);
              }
            } else {
              Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
            }
          }

          // ------ Query statistics on the solution and the solve ------
          // Number of simplex iterations
          int64_t _impl_iterations() const {
            double iter;
            if (!Parent::checkSolutionIsSynchronized()) return nt::mp::details::kUnknownNumberOfIterations;
            CheckedGurobiCall(GRBgetdblattr(model_, GRB_DBL_ATTR_ITERCOUNT, &iter));
            return static_cast< int64_t >(iter);
          }
          // Number of branch-and-bound nodes. Only available for discrete problems.
          int64_t _impl_nodes() const {
            if constexpr (_impl_IsMIP()) {
              if (!Parent::checkSolutionIsSynchronized()) return nt::mp::details::kUnknownNumberOfNodes;
              return static_cast< int64_t >(GetDoubleAttr(GRB_DBL_ATTR_NODECOUNT));
            } else {
              LOG_F(FATAL, "Number of nodes only available for discrete problems");
              return Parent::kUnknownNumberOfNodes;
            }
          }

          // Returns the basis status of a row.
          BasisStatus _impl_row_status(int constraint_index) const {
            const int optim_status = GetIntAttr(GRB_INT_ATTR_STATUS);
            if (optim_status != GRB_OPTIMAL && optim_status != GRB_SUBOPTIMAL) {
              LOG_F(FATAL, "Basis status only available after a solution has been found.");
              return BasisStatus::FREE;
            }
            if constexpr (_impl_IsMIP()) {
              LOG_F(FATAL, "Basis status only available for continuous problems.");
              return BasisStatus::FREE;
            }
            const int grb_index = mp_cons_to_gurobi_linear_cons_[constraint_index];
            if (grb_index < 0) {
              LOG_F(FATAL, "Basis status not available for nonlinear constraints.");
              return BasisStatus::FREE;
            }
            const int gurobi_basis_status = GetIntAttrElement(GRB_INT_ATTR_CBASIS, grb_index);
            return TransformGRBConstraintBasisStatus(gurobi_basis_status, constraint_index);
          }
          // Returns the basis status of a column.
          BasisStatus _impl_column_status(int variable_index) const {
            const int optim_status = GetIntAttr(GRB_INT_ATTR_STATUS);
            if (optim_status != GRB_OPTIMAL && optim_status != GRB_SUBOPTIMAL) {
              LOG_F(FATAL, "Basis status only available after a solution has been found.");
              return BasisStatus::FREE;
            }
            if constexpr (_impl_IsMIP()) {
              LOG_F(FATAL, "Basis status only available for continuous problems.");
              return BasisStatus::FREE;
            }
            const int grb_index = mp_var_to_gurobi_var_[variable_index];
            const int gurobi_basis_status = GetIntAttrElement(GRB_INT_ATTR_VBASIS, grb_index);
            return TransformGRBVarBasisStatus(gurobi_basis_status);
          }

          // Checks whether a feasible solution exists.
          bool _impl_CheckSolutionExists() const { return false; }

          bool _impl_HasDualFarkas() const { return false; }

          // ------ Parameters  -----

          void _impl_SetParameters(const MPSolverParameters& param);

          // Set each parameter in the underlying solver.
          void _impl_SetRelativeMipGap(double value) {
            if constexpr (_impl_IsMIP()) {
              CheckedGurobiCall(GRBsetdblparam(GRBgetenv(model_), GRB_DBL_PAR_MIPGAP, value));
            } else {
              LOG_F(WARNING, "The relative MIP gap is only available for discrete problems.");
            }
          }

          // Gurobi has two different types of primal tolerance (feasibility tolerance):
          // constraint and integrality. We need to set them both.
          // See:
          // http://www.gurobi.com/documentation/6.0/refman/feasibilitytol.html
          // and
          // http://www.gurobi.com/documentation/6.0/refman/intfeastol.html
          void _impl_SetPrimalTolerance(double value) {
            CheckedGurobiCall(GRBsetdblparam(GRBgetenv(model_), GRB_DBL_PAR_FEASIBILITYTOL, value));
            CheckedGurobiCall(GRBsetdblparam(GRBgetenv(model_), GRB_DBL_PAR_INTFEASTOL, value));
          }

          // As opposed to primal (feasibility) tolerance, the dual (optimality) tolerance
          // applies only to the reduced costs in the improving direction.
          // See:
          // http://www.gurobi.com/documentation/6.0/refman/optimalitytol.html
          void _impl_SetDualTolerance(double value) {
            CheckedGurobiCall(GRBsetdblparam(GRBgetenv(model_), GRB_DBL_PAR_OPTIMALITYTOL, value));
          }
          void _impl_SetPresolveMode(int value) {
            switch (value) {
              case MPSolverParameters::PRESOLVE_OFF: {
                CheckedGurobiCall(GRBsetintparam(GRBgetenv(model_), GRB_INT_PAR_PRESOLVE, false));
                break;
              }
              case MPSolverParameters::PRESOLVE_ON: {
                CheckedGurobiCall(GRBsetintparam(GRBgetenv(model_), GRB_INT_PAR_PRESOLVE, true));
                break;
              }
              default: {
                Parent::setIntegerParamToUnsupportedValue(MPSolverParameters::PRESOLVE, value);
              }
            }
          }
          void _impl_SetScalingMode(int value) {
            switch (value) {
              case MPSolverParameters::SCALING_OFF:
                CheckedGurobiCall(GRBsetintparam(GRBgetenv(model_), GRB_INT_PAR_SCALEFLAG, false));
                break;
              case MPSolverParameters::SCALING_ON:
                CheckedGurobiCall(GRBsetintparam(GRBgetenv(model_), GRB_INT_PAR_SCALEFLAG, true));
                CheckedGurobiCall(GRBsetdblparam(GRBgetenv(model_), GRB_DBL_PAR_OBJSCALE, 0.0));
                break;
              default:
                // Leave the parameters untouched.
                break;
            }
          }
          void _impl_SetLpAlgorithm(int value) {
            switch (value) {
              case MPSolverParameters::DUAL:
                CheckedGurobiCall(GRBsetintparam(GRBgetenv(model_), GRB_INT_PAR_METHOD, GRB_METHOD_DUAL));
                break;
              case MPSolverParameters::PRIMAL:
                CheckedGurobiCall(GRBsetintparam(GRBgetenv(model_), GRB_INT_PAR_METHOD, GRB_METHOD_PRIMAL));
                break;
              case MPSolverParameters::BARRIER:
                CheckedGurobiCall(GRBsetintparam(GRBgetenv(model_), GRB_INT_PAR_METHOD, GRB_METHOD_BARRIER));
                break;
              default: Parent::setIntegerParamToUnsupportedValue(MPSolverParameters::LP_ALGORITHM, value);
            }
          }

          // ----- Misc -----
          // Query problem type.
          constexpr static bool _impl_IsContinuous() { return _impl_IsLP(); }
          constexpr static bool _impl_IsLP() { return !MIP; }
          constexpr static bool _impl_IsMIP() { return MIP; }

          void _impl_ExtractNewVariables() {
            const int total_num_vars = Parent::_variables.size();
            if (total_num_vars > Parent::_last_variable_index) {
              // Define new variables.
              for (int j = Parent::_last_variable_index; j < total_num_vars; ++j) {
                const MPVariable* const var = Parent::_variables[j];
                Parent::set_variable_as_extracted(var->index(), true);
                CheckedGurobiCall(GRBaddvar(model_,
                                            0,         // numnz
                                            nullptr,   // vind
                                            nullptr,   // vval
                                            Parent::_objective.getCoefficient(var),
                                            var->lb(),
                                            var->ub(),
                                            var->integer() && _impl_IsMIP() ? GRB_INTEGER : GRB_CONTINUOUS,
                                            var->name().empty() ? nullptr : var->name().c_str()));
                mp_var_to_gurobi_var_.push_back(num_gurobi_vars_++);
              }
              CheckedGurobiCall(GRBupdatemodel(model_));
              // Add new variables to existing constraints.
              nt::IntDynamicArray    grb_cons_ind;
              nt::IntDynamicArray    grb_var_ind;
              nt::DoubleDynamicArray coef;
              for (int i = 0; i < Parent::_last_constraint_index; ++i) {
                // If there was a nonincremental change/the model is not incremental (e.g.
                // there is an indicator constraint), we should never enter this loop, as
                // _last_variable_index will be reset to zero before extractNewVariables()
                // is called.
                MPConstraint* const ct = Parent::_constraints[i];
                const int           grb_ct_idx = mp_cons_to_gurobi_linear_cons_[ct->index()];
                DCHECK_GE_F(grb_ct_idx, 0);
                DCHECK_F(ct->indicator_variable() == nullptr);
                for (const auto& entry: ct->_coefficients) {
                  const int var_index = entry.first->index();
                  DCHECK_F(Parent::variable_is_extracted(var_index));

                  if (var_index >= Parent::_last_variable_index) {
                    grb_cons_ind.push_back(grb_ct_idx);
                    grb_var_ind.push_back(mp_var_to_gurobi_var_[var_index]);
                    coef.push_back(entry.second);
                  }
                }
              }
              if (!grb_cons_ind.empty()) {
                CheckedGurobiCall(
                   GRBchgcoeffs(model_, grb_cons_ind.size(), grb_cons_ind.data(), grb_var_ind.data(), coef.data()));
              }
            }
            CheckedGurobiCall(GRBupdatemodel(model_));
            DCHECK_EQ_F(GetIntAttr(GRB_INT_ATTR_NUMVARS), num_gurobi_vars_);
          }
          void _impl_ExtractNewConstraints() {
            int total_num_rows = Parent::_constraints.size();
            if (Parent::_last_constraint_index < total_num_rows) {
              // Add each new constraint.
              for (int row = Parent::_last_constraint_index; row < total_num_rows; ++row) {
                MPConstraint* const ct = Parent::_constraints[row];
                Parent::set_constraint_as_extracted(row, true);
                const int              size = ct->_coefficients.size();
                nt::IntDynamicArray    grb_vars;
                nt::DoubleDynamicArray coefs;
                grb_vars.reserve(size);
                coefs.reserve(size);
                for (const auto& entry: ct->_coefficients) {
                  const int var_index = entry.first->index();
                  CHECK_F(Parent::variable_is_extracted(var_index));
                  grb_vars.push_back(mp_var_to_gurobi_var_[var_index]);
                  coefs.push_back(entry.second);
                }
                char* const name = ct->name().empty() ? nullptr : const_cast< char* >(ct->name().c_str());
                if (ct->indicator_variable() != nullptr) {
                  const int grb_ind_var = mp_var_to_gurobi_var_[ct->indicator_variable()->index()];
                  if (ct->lb() > -NT_DOUBLE_INFINITY) {
                    CheckedGurobiCall(GRBaddgenconstrIndicator(model_,
                                                               name,
                                                               grb_ind_var,
                                                               ct->indicator_value(),
                                                               size,
                                                               grb_vars.data(),
                                                               coefs.data(),
                                                               ct->ub() == ct->lb() ? GRB_EQUAL : GRB_GREATER_EQUAL,
                                                               ct->lb()));
                  }
                  if (ct->ub() < NT_DOUBLE_INFINITY && ct->lb() != ct->ub()) {
                    CheckedGurobiCall(GRBaddgenconstrIndicator(model_,
                                                               name,
                                                               grb_ind_var,
                                                               ct->indicator_value(),
                                                               size,
                                                               grb_vars.data(),
                                                               coefs.data(),
                                                               GRB_LESS_EQUAL,
                                                               ct->ub()));
                  }
                  mp_cons_to_gurobi_linear_cons_.push_back(-1);
                } else {
                  // Using GRBaddrangeconstr for constraints that don't require it adds
                  // a slack which is not always removed by presolve.
                  if (ct->lb() == ct->ub()) {
                    CheckedGurobiCall(
                       GRBaddconstr(model_, size, grb_vars.data(), coefs.data(), GRB_EQUAL, ct->lb(), name));
                  } else if (ct->lb() == -NT_DOUBLE_INFINITY) {
                    CheckedGurobiCall(
                       GRBaddconstr(model_, size, grb_vars.data(), coefs.data(), GRB_LESS_EQUAL, ct->ub(), name));
                  } else if (ct->ub() == NT_DOUBLE_INFINITY) {
                    CheckedGurobiCall(
                       GRBaddconstr(model_, size, grb_vars.data(), coefs.data(), GRB_GREATER_EQUAL, ct->lb(), name));
                  } else {
                    CheckedGurobiCall(
                       GRBaddrangeconstr(model_, size, grb_vars.data(), coefs.data(), ct->lb(), ct->ub(), name));
                    // NOTE(user): range constraints implicitly add an extra variable
                    // to the model.
                    num_gurobi_vars_++;
                  }
                  mp_cons_to_gurobi_linear_cons_.push_back(num_gurobi_linear_cons_++);
                }
              }
            }
            CheckedGurobiCall(GRBupdatemodel(model_));
            DCHECK_EQ_F(GetIntAttr(GRB_INT_ATTR_NUMCONSTRS), num_gurobi_linear_cons_);
          }
          void _impl_ExtractObjective() {
            SetIntAttr(GRB_INT_ATTR_MODELSENSE, Parent::_maximize ? GRB_MAXIMIZE : GRB_MINIMIZE);
            SetDoubleAttr(GRB_DBL_ATTR_OBJCON, Parent::objective().offset());
          }

          std::string _impl_SolverVersion() const noexcept {
            int major, minor, technical;
            GRBversion(&major, &minor, &technical);
            return nt::format("Gurobi library version {}.{}.{}\n", major, minor, technical);
          }

          // Iterates through the solutions in Gurobi's solution pool.
          bool _impl_NextSolution() {
            // Next solution only supported for MIP
            if (!_impl_IsMIP()) return false;

            // Make sure we have successfully solved the problem and not modified it.
            if (!Parent::checkSolutionIsSynchronizedAndExists()) { return false; }
            // Check if we are out of solutions.
            if (current_solution_index_ + 1 >= SolutionCount()) { return false; }
            current_solution_index_++;

            CheckedGurobiCall(GRBsetintparam(GRBgetenv(model_), GRB_INT_PAR_SOLUTIONNUMBER, current_solution_index_));

            Parent::_objective_value = GetDoubleAttr(GRB_DBL_ATTR_POOLOBJVAL);
            const nt::DoubleDynamicArray grb_variable_values = GetDoubleAttrArray(GRB_DBL_ATTR_XN, num_gurobi_vars_);

            for (int i = 0; i < Parent::_variables.size(); ++i) {
              MPVariable* const var = Parent::_variables[i];
              var->set_solution_value(grb_variable_values[mp_var_to_gurobi_var_[i]]);
            }
            // TODO(user): This reset may not be necessary, investigate.
            GRBresetparams(GRBgetenv(model_));
            return true;
          }

          bool _impl_SupportsCallbacks() const { return true; }
          void _impl_SetCallback(MPCallback* mp_callback) noexcept { callback_ = mp_callback; };

          void* _impl_underlying_solver() { return reinterpret_cast< void* >(model_); }

          bool _impl_InterruptSolve() {
            // const absl::MutexLock lock(&hold_interruptions_mutex_);
            if (model_ != nullptr) GRBterminate(model_);
            return true;
          }

          void _impl_Write(const std::string& filename) {
            if (Parent::_sync_status == SynchronizationStatus::MUST_RELOAD) { _impl_Reset(); }
            Parent::extractModel();
            // Sync solver.
            CheckedGurobiCall(GRBupdatemodel(model_));
            VLOG_F(1, "Writing Gurobi model file '{}'.", filename);
            const int status = GRBwrite(model_, filename.c_str());
            if (status) { LOG_F(WARNING, "Failed to write MIP. {}", GRBgeterrormsg(global_env_)); }
          }

          double _impl_ComputeExactConditionNumber() const {
            if constexpr (!_impl_IsContinuous()) {
              LOG_F(FATAL, "ComputeExactConditionNumber not implemented for GUROBI_MIXED_INTEGER_PROGRAMMING");
              return 0.0;
            }

            // TODO(user): Not yet working.
            LOG_F(FATAL, "ComputeExactConditionNumber not implemented for GUROBI_LINEAR_PROGRAMMING");
            return 0.0;

            // double cond = 0.0;
            // const int status = GRBgetdblattr(model_, GRB_DBL_ATTR_KAPPA, &cond);
            // if (0 == status) {
            //   return cond;
            // } else {
            //   LOG(DFATAL) << "Condition number only available for "
            //               << "continuous problems";
            //   return 0.0;
            // }
          }


          GRBmodel* model_;
          // The environment used to create model_. Note that once the model is created,
          // it used a copy of the environment, accessible via GRBgetenv(model_), which
          // you should use for setting parameters. Use global_env_ only to create a new
          // model or for GRBgeterrormsg().
          GRBenv* global_env_;
          // bool        mip_;
          int         current_solution_index_;
          MPCallback* callback_ = nullptr;
          bool        update_branching_priorities_ = false;
          // Has length equal to the number of MPVariables in
          // MPSolverInterface::solver_. Values are the index of the corresponding
          // Gurobi variable. Note that Gurobi may have additional auxiliary variables
          // not represented by MPVariables, such as those created by two-sided range
          // constraints.
          nt::IntDynamicArray mp_var_to_gurobi_var_;
          // Has length equal to the number of MPConstraints in
          // MPSolverInterface::solver_. Values are the index of the corresponding
          // linear (or range) constraint in Gurobi, or -1 if no such constraint exists
          // (e.g. for indicator constraints).
          nt::IntDynamicArray mp_cons_to_gurobi_linear_cons_;
          // Should match the Gurobi model after it is updated.
          int num_gurobi_vars_ = 0;
          // Should match the Gurobi model after it is updated.
          // NOTE(user): indicator constraints are not counted below.
          int num_gurobi_linear_cons_ = 0;
          // See the implementation note at the top of file on incrementalism.
          bool had_nonincremental_change_ = false;

          // Mutex is held to prevent InterruptSolve() to call GRBterminate() when
          // model_ is not completely built. It also prevents model_ to be changed
          // during the execution of GRBterminate().
          // mutable absl::Mutex hold_interruptions_mutex_;
        };

        // Creates a LP/MIP instance with the specified name and minimization objective.
        template < bool MIP >
        GurobiInterface< MIP >::GurobiInterface(const std::string& name, const MPEnv& e) :
            Parent(name), model_(nullptr), global_env_(nullptr), current_solution_index_(0) {
          // if (GRBloadenv(&global_env_, nullptr) != 0 || global_env_ == nullptr)
          //   LOG_F(ERROR, "Could not create Gurobi environment: is Gurobi licensed on this machine? {}",
          //   GRBgeterrormsg(global_env_));
          global_env_ = e._env;
          if (!global_env_) LOG_F(ERROR, "Gurobi environment not initialized.");

          CheckedGurobiCall(GRBnewmodel(global_env_,
                                        &model_,
                                        Parent::_name.c_str(),
                                        0,           // numvars
                                        nullptr,     // obj
                                        nullptr,     // lb
                                        nullptr,     // ub
                                        nullptr,     // vtype
                                        nullptr));   // varnanes
          CheckedGurobiCall(
             GRBsetintattr(model_, GRB_INT_ATTR_MODELSENSE, Parent::_maximize ? GRB_MAXIMIZE : GRB_MINIMIZE));
          CheckedGurobiCall(GRBsetintparam(GRBgetenv(model_), GRB_INT_PAR_OUTPUTFLAG, 0));
          //   CheckedGurobiCall(GRBsetintparam(GRBgetenv(model_), GRB_INT_PAR_THREADS,
          //   absl::GetFlag(FLAGS_num_gurobi_threads)));
        }

        template < bool MIP >
        GurobiInterface< MIP >::~GurobiInterface() {
          CheckedGurobiCall(GRBfreemodel(model_));
          // GRBfreeenv(global_env_);
        }

        template < bool MIP >
        inline void GurobiInterface< MIP >::_impl_Reset() {
          // We hold calls to GRBterminate() until the new model_ is ready.
          //   const absl::MutexLock lock(&hold_interruptions_mutex_);

          GRBmodel* old_model = model_;
          CheckedGurobiCall(GRBnewmodel(global_env_,
                                        &model_,
                                        Parent::_name.c_str(),
                                        0,           // numvars
                                        nullptr,     // obj
                                        nullptr,     // lb
                                        nullptr,     // ub
                                        nullptr,     // vtype
                                        nullptr));   // varnames

          // Copy all existing parameters from the previous model to the new one. This
          // ensures that if a user calls multiple times
          // SetSolverSpecificParametersAsString() and then Reset() is called, we still
          // take into account all parameters.
          //
          // The current code only reapplies the parameters stored in
          // solver_specific_parameter_string_ at the start of the solve; other
          // parameters set by previous calls are only kept in the Gurobi model.
          //
          // TODO - b/328604189: Fix logging issue upstream, switch to a different API
          // for copying parameters, or avoid calling Reset() in more places.
          CheckedGurobiCall(GRBcopyparams(GRBgetenv(model_), GRBgetenv(old_model)));

          CheckedGurobiCall(GRBfreemodel(old_model));
          old_model = nullptr;

          Parent::resetExtractionInformation();
          mp_var_to_gurobi_var_.clear();
          mp_cons_to_gurobi_linear_cons_.clear();
          num_gurobi_vars_ = 0;
          num_gurobi_linear_cons_ = 0;
          had_nonincremental_change_ = false;
        }

        // ------ Parameters  -----
        template < bool MIP >
        inline void GurobiInterface< MIP >::_impl_SetParameters(const MPSolverParameters& param) {
          Parent::setCommonParameters(param);
          if constexpr (_impl_IsMIP()) Parent::setMIPParameters(param);
        }

        // ------ Model modifications and extraction -----

        // Not cached
        template < bool MIP >
        inline void GurobiInterface< MIP >::_impl_SetOptimizationDirection(bool maximize) {
          Parent::invalidateSolutionSynchronization();
          CheckedGurobiCall(GRBsetintattr(model_, GRB_INT_ATTR_MODELSENSE, maximize ? GRB_MAXIMIZE : GRB_MINIMIZE));
        }


        // Solve the problem using the parameter values specified.
        template < bool MIP >
        inline ResultStatus GurobiInterface< MIP >::_impl_Solve(const MPSolverParameters& param) {
          nt::WallTimer timer;
          timer.start();

          if (param.getIntegerParam(MPSolverParameters::INCREMENTALITY) == MPSolverParameters::INCREMENTALITY_OFF
              || ModelIsNonincremental() || had_nonincremental_change_) {
            _impl_Reset();
          }

          // Set log level.
          CheckedGurobiCall(GRBsetintparam(GRBgetenv(model_), GRB_INT_PAR_OUTPUTFLAG, !Parent::quiet()));

          Parent::extractModel();

          // Sync solver.
          CheckedGurobiCall(GRBupdatemodel(model_));
          VLOG_F(1, "Model built in {} seconds.", timer.get());

          // Set solution hints if any.
          for (const nt::Pair< const MPVariable*, double >& p: Parent::_solution_hint)
            SetDoubleAttrElement(GRB_DBL_ATTR_START, mp_var_to_gurobi_var_[p.first->index()], p.second);

          // Pass branching priority annotations if at least one has been updated.
          if (update_branching_priorities_) {
            for (const MPVariable* var: Parent::_variables) {
              SetIntAttrElement(
                 GRB_INT_ATTR_BRANCHPRIORITY, mp_var_to_gurobi_var_[var->index()], var->branching_priority());
            }
            update_branching_priorities_ = false;
          }

          // Time limit.
          if (Parent::time_limit() != 0) {
            VLOG_F(1, "Setting time limit = {} ms ", Parent::time_limit());
            CheckedGurobiCall(GRBsetdblparam(GRBgetenv(model_), GRB_DBL_PAR_TIMELIMIT, Parent::time_limit_in_secs()));
          }

          // We first set our internal MPSolverParameters from 'param' and then set
          // any user-specified internal solver parameters via
          // solver_specific_parameter_string_.
          // Default MPSolverParameters can override custom parameters (for example for
          // presolving) and therefore we apply MPSolverParameters first.
          _impl_SetParameters(param);
          // Parent::SetSolverSpecificParametersAsString(Parent::solver_specific_parameter_string_);

          std::unique_ptr< GurobiMPCallbackContext > gurobi_context;
          MPCallbackWithGurobiContext                mp_callback_with_context;
          int                                        gurobi_precrush = 0;
          int                                        gurobi_lazy_constraint = 0;
          if (callback_ == nullptr) {
            CheckedGurobiCall(GRBsetcallbackfunc(model_, nullptr, nullptr));
          } else {
            gurobi_context = std::make_unique< GurobiMPCallbackContext >(global_env_,
                                                                         &mp_var_to_gurobi_var_,
                                                                         num_gurobi_vars_,
                                                                         callback_->might_add_cuts(),
                                                                         callback_->might_add_lazy_constraints());
            mp_callback_with_context.context = gurobi_context.get();
            mp_callback_with_context.callback = callback_;
            CheckedGurobiCall(
               GRBsetcallbackfunc(model_, CallbackImpl, static_cast< void* >(&mp_callback_with_context)));
            gurobi_precrush = callback_->might_add_cuts();
            gurobi_lazy_constraint = callback_->might_add_lazy_constraints();
          }
          CheckedGurobiCall(GRBsetintparam(GRBgetenv(model_), GRB_INT_PAR_PRECRUSH, gurobi_precrush));
          CheckedGurobiCall(GRBsetintparam(GRBgetenv(model_), GRB_INT_PAR_LAZYCONSTRAINTS, gurobi_lazy_constraint));

          // Logs all parameters not at default values in the model environment.
          if (!Parent::quiet()) {
            LOG_F(INFO,
                  "{}",
                  GurobiParamInfoForLogging(GRBgetenv(model_),
                                            /*one_liner_output=*/true));
          }

          // Solve
          timer.restart();
          const int status = GRBoptimize(model_);

          if (status) {
            VLOG_F(1, "Failed to optimize MIP. Error {}", GRBgeterrormsg(global_env_));
          } else {
            VLOG_F(1, "Solved in {} seconds.", timer.get());
          }

          // Get the status.
          const int optimization_status = GetIntAttr(GRB_INT_ATTR_STATUS);
          VLOG_F(1, "Solution status {}.", optimization_status);
          const int solution_count = SolutionCount();

          switch (optimization_status) {
            case GRB_OPTIMAL: Parent::_result_status = ResultStatus::OPTIMAL; break;
            case GRB_INFEASIBLE: Parent::_result_status = ResultStatus::INFEASIBLE; break;
            case GRB_UNBOUNDED: Parent::_result_status = ResultStatus::UNBOUNDED; break;
            case GRB_INF_OR_UNBD:
              // TODO(user): We could introduce our own "infeasible or
              // unbounded" status.
              Parent::_result_status = ResultStatus::INFEASIBLE;
              break;
            default: {
              if (solution_count > 0) {
                Parent::_result_status = ResultStatus::FEASIBLE;
              } else {
                Parent::_result_status = ResultStatus::NOT_SOLVED;
              }
              break;
            }
          }

          if (_impl_IsMIP()
              && (Parent::_result_status != ResultStatus::UNBOUNDED
                  && Parent::_result_status != ResultStatus::INFEASIBLE)) {
            const int error = GRBgetdblattr(model_, GRB_DBL_ATTR_OBJBOUND, &(Parent::_best_objective_bound));
            LOG_IF_F(WARNING,
                     error != 0,
                     "Best objective bound is not available, error={}, message={}",
                     error,
                     GRBgeterrormsg(global_env_));
            VLOG_F(1, "best bound = {}", Parent::_best_objective_bound);
          }

          if (solution_count > 0
              && (Parent::_result_status == ResultStatus::FEASIBLE
                  || Parent::_result_status == ResultStatus::OPTIMAL)) {
            current_solution_index_ = 0;
            // Get the results.
            Parent::_objective_value = GetDoubleAttr(GRB_DBL_ATTR_OBJVAL);
            VLOG_F(1, "objective = {}", Parent::_objective_value);

            {
              const nt::DoubleDynamicArray grb_variable_values = GetDoubleAttrArray(GRB_DBL_ATTR_X, num_gurobi_vars_);
              for (int i = 0; i < Parent::_variables.size(); ++i) {
                MPVariable* const var = Parent::_variables[i];
                const double      val = grb_variable_values[mp_var_to_gurobi_var_[i]];
                var->set_solution_value(val);
                VLOG_F(3, "{}, value = {}", var->name(), val);
              }
            }
            if constexpr (!_impl_IsMIP()) {
              {
                const nt::DoubleDynamicArray grb_reduced_costs = GetDoubleAttrArray(GRB_DBL_ATTR_RC, num_gurobi_vars_);
                for (int i = 0; i < Parent::_variables.size(); ++i) {
                  MPVariable* const var = Parent::_variables[i];
                  const double      rc = grb_reduced_costs[mp_var_to_gurobi_var_[i]];
                  var->set_reduced_cost(rc);
                  VLOG_F(4, "{}, reduced cost = {}", var->name(), rc);
                }
              }

              {
                nt::DoubleDynamicArray grb_dual_values = GetDoubleAttrArray(GRB_DBL_ATTR_PI, num_gurobi_linear_cons_);
                for (int i = 0; i < Parent::_constraints.size(); ++i) {
                  MPConstraint* const ct = Parent::_constraints[i];
                  const double        dual_value = grb_dual_values[mp_cons_to_gurobi_linear_cons_[i]];
                  ct->set_dual_value(dual_value);
                  VLOG_F(4, "row {}, dual value = {}", ct->index(), dual_value);
                }
              }
            }
          }

          Parent::_sync_status = SynchronizationStatus::SOLUTION_SYNCHRONIZED;
          GRBresetparams(GRBgetenv(model_));
          return Parent::_result_status;
        }


      }   // namespace details

      using LPModel = details::GurobiInterface< false >;
      using MIPModel = details::GurobiInterface< true >;

      inline constexpr bool hasMIP() noexcept { return true; }
      inline constexpr bool hasLP() noexcept { return true; }

    }   // namespace gurobi
  }     // namespace mp
}   // namespace nt

#endif