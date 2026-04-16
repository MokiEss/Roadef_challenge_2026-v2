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
 * @file mp_dummy.h
 * @brief Dummy solver implementation for networktools MP module
 *
 * This header provides a no-op solver for testing and development.
 * Include this file when you need to test MP code without a real solver.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 *
 * @par Usage:
 * @code
 * #include "networktools.h"
 * #include "mp/mp_dummy.h"
 *
 * nt::mp::dummy::LPModel lp_model;
 * nt::mp::dummy::MIPModel mip_model;
 * @endcode
 *
 * @par Purpose:
 * - Testing the MP API without solver dependencies
 * - Development and debugging of MP-based algorithms
 * - Placeholder when no real solver is available
 * - Model validation and syntax checking
 *
 * @par Behavior:
 * - Always returns MODEL_INVALID status
 * - Does not perform any actual optimization
 * - No external library dependencies
 * - Implements the full MP interface as no-ops
 *
 * @warning Not for production use - provides no actual solving capability
 * @see mp.h for API documentation
 * @see README.md for compilation examples
 */

#ifndef _NT_DUMMY_INTERFACE_H_
#define _NT_DUMMY_INTERFACE_H_

#include "bits/mp_solver_base.h"

namespace nt {
  namespace mp {
    namespace dummy {

      struct MPEnv {
        constexpr bool isValid() const noexcept { return true; }
      };

      namespace details {

        template < bool MIP >
        struct DUMMYSolver: nt::mp::details::MPSolver< DUMMYSolver< MIP > > {
          using Parent = nt::mp::details::MPSolver< DUMMYSolver< MIP > >;
          using MPObjective = Parent::MPObjective;
          using MPConstraint = Parent::MPConstraint;
          using MPVariable = Parent::MPVariable;
          using LinearRange = Parent::LinearRange;

          using Parent::Parent;

          struct MPCallbackContext: Parent::template MPCallbackContextInterface< MPCallbackContext > {
            MPCallbackEvent event() { return MPCallbackEvent::kUnknown; }
            bool            canQueryVariableValues() { return false; }
            double          variableValue(const MPVariable* variable) { return 0.; }
            void            addCut(const LinearRange& cutting_plane) {}
            void            addLazyConstraint(const LinearRange& lazy_constraint) {}
            void            addPricedVariable(const MPVariable* variable) {}
            double          suggestSolution(const nt::PointerMap< const MPVariable*, double >& solution) { return 0.; }
            int64_t         numExploredNodes() { return 0; }
          };

          using MPCallback = Parent::template MPCallback< MPCallbackContext >;
          using MPCallbackList = Parent::template MPCallbackList< MPCallbackContext >;

          int _dummy;

          DUMMYSolver(const std::string& name, const MPEnv& e) : Parent(name) {}

          void _impl_SetOptimizationDirection(bool maximize) {}

          ResultStatus _impl_Solve(const MPSolverParameters& param) {
            Parent::_result_status = ResultStatus::MODEL_INVALID;
            Parent::_sync_status = SynchronizationStatus::SOLUTION_SYNCHRONIZED;
            return Parent::_result_status;
          }

          void _impl_Reset() { Parent::resetExtractionInformation(); }
          void _impl_FreeModel() {}

          void _impl_SetVariableBounds(int var_index, double lb, double ub) {}
          void _impl_SetVariableInteger(int var_index, bool integer) {
            LOG_F(FATAL, "Attempt to change variable to integer in non-MIP problem!");
          }
          void _impl_SetConstraintBounds(int row_index, double lb, double ub) {}

          void _impl_AddRowConstraint(MPConstraint* ct) { Parent::_sync_status = SynchronizationStatus::MUST_RELOAD; }
          void _impl_AddVariable(MPVariable* var) { Parent::_sync_status = SynchronizationStatus::MUST_RELOAD; }
          void _impl_SetCoefficient(MPConstraint*     constraint,
                                    const MPVariable* variable,
                                    double            new_value,
                                    double            old_value) {}
          void _impl_ClearConstraint(MPConstraint* constraint) {}
          void _impl_SetObjectiveCoefficient(const MPVariable* variable, double coefficient) {
            Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
          }
          void _impl_SetObjectiveOffset(double value) { Parent::_sync_status = SynchronizationStatus::MUST_RELOAD; }
          void _impl_ClearObjective() {}

          int64_t _impl_iterations() const { return Parent::kUnknownNumberOfIterations; }
          int64_t _impl_nodes() const {
            LOG_F(FATAL, "Number of nodes only available for discrete problems");
            return Parent::kUnknownNumberOfNodes;
          }
          BasisStatus _impl_row_status(int constraint_index) const {
            LOG_F(FATAL, "row_status not yet implemented for Dummy");
            return Parent::FREE;
          }
          BasisStatus _impl_column_status(int variable_index) const {
            LOG_F(FATAL, "column_status not yet implemented for Dummy");
            return Parent::FREE;
          }

          bool _impl_HasDualFarkas() const { return false; }

          bool _impl_IsContinuous() const { return _impl_IsLP(); }
          bool _impl_IsLP() const { return !MIP; }
          bool _impl_IsMIP() const { return MIP; }

          void _impl_ExtractNewVariables() {}
          void _impl_ExtractNewConstraints() {}
          void _impl_ExtractObjective() {}

          std::string _impl_SolverVersion() const { return nt::format("Dummy Solver"); }

          void* _impl_underlying_solver() { return reinterpret_cast< void* >(&_dummy); }

          // See MPSolver::SetCallback() for details.
          void _impl_SetCallback(MPCallback* mp_callback) {}
          bool _impl_SupportsCallbacks() const { return true; }

          // private:
          void _impl_SetParameters(const MPSolverParameters& param) { Parent::SetCommonParameters(param); }
          void _impl_SetRelativeMipGap(double value) {
            LOG_F(WARNING, "The relative MIP gap is only available for discrete problems.");
          }
          void _impl_SetPrimalTolerance(double value) {}
          void _impl_SetDualTolerance(double value) {}
          void _impl_SetPresolveMode(int presolve) {}
          void _impl_SetScalingMode(int scaling) { Parent::SetUnsupportedIntegerParam(MPSolverParameters::SCALING); }
          void _impl_SetLpAlgorithm(int lp_algorithm) {}

          bool _impl_SetNumThreads(int num_threads) {
            LOG_F(ERROR, "SetNumThreads() not currently supported for Dummy.");
            return false;
          }
          bool _impl_SetSolverSpecificParametersAsString(const std::string& parameters) {
            LOG_F(ERROR, "SetSolverSpecificParametersAsString() not currently supported for Dummy.");
            return false;
          }
        };


      }   // namespace details

      using LPModel = details::DUMMYSolver< false >;
      using MIPModel = details::DUMMYSolver< true >;

      inline constexpr bool hasMIP() noexcept { return true; }
      inline constexpr bool hasLP() noexcept { return true; }

    }   // namespace dummy

  }   // namespace mp
}   // namespace nt

#endif
