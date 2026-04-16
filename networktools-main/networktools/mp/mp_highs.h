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
 * @file mp_highs.h
 * @brief HiGHS solver integration for networktools MP module
 *
 * This header enables HiGHS support for both LP and MIP problems.
 * Include this file after networktools.h to use HiGHS solver.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 *
 * @par Usage:
 * @code
 * #include "networktools.h"
 * #include "mp/mp_highs.h"
 *
 * nt::mp::highs::LPModel lp_model;
 * nt::mp::highs::MIPModel mip_model;
 * @endcode
 *
 * @par Features:
 * - Both LP and MIP support
 * - Modern open-source solver (MIT license)
 * - Excellent performance, competitive with commercial solvers
 * - Parallel simplex and IPM methods
 * - Active development and maintained by leading researchers
 *
 * @par Requirements:
 * - HiGHS library
 * - Link with HiGHS library (-lhighs)
 *
 * @note HiGHS often outperforms traditional open-source solvers
 * @see mp.h for API documentation
 * @see README.md for compilation examples
 */

#ifndef _NT_HIGHS_INTERFACE_H_
#define _NT_HIGHS_INTERFACE_H_

#include "bits/mp_solver_base.h"

#include "Highs.h"

namespace nt {
  namespace mp {
    namespace highs {

      struct MPEnv {
        constexpr bool isValid() const noexcept { return true; }
      };

      namespace details {

        inline BasisStatus TransformHIGHSBasisStatus(HighsBasisStatus highs_basis_status) noexcept {
          switch (highs_basis_status) {
            case HighsBasisStatus::kBasic: return BasisStatus::BASIC;
            case HighsBasisStatus::kLower: return BasisStatus::AT_LOWER_BOUND;
            case HighsBasisStatus::kUpper: return BasisStatus::AT_UPPER_BOUND;
            case HighsBasisStatus::kZero: return BasisStatus::FREE;
            case HighsBasisStatus::kNonbasic: return BasisStatus::FIXED_VALUE;
            default: LOG_F(FATAL, "Unknown HIGHS basis status"); return BasisStatus::FREE;
          }
        }

        template < bool MIP >
        struct HighsSolver: nt::mp::details::MPSolver< HighsSolver< MIP > > {
          using Parent = nt::mp::details::MPSolver< HighsSolver< MIP > >;
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

          HighsSolver(const std::string& name, const MPEnv& e) : Parent(name) {
            _model.lp_.model_name_ = name;
            _model.lp_.sense_ = ObjSense::kMinimize;
          }

          ~HighsSolver() = default;

          void _impl_SetOptimizationDirection(bool maximize) noexcept {
            Parent::invalidateSolutionSynchronization();
            if (Parent::_sync_status == SynchronizationStatus::MODEL_SYNCHRONIZED) {
              _model.lp_.sense_ = maximize ? ObjSense::kMaximize : ObjSense::kMinimize;
            } else {
              Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
            }
          }

          // Solve the LP/MIP. Returns true only if the optimal solution was revealed.
          // Returns the status of the search.
          ResultStatus _impl_Solve(const MPSolverParameters& param) noexcept {
            nt::WallTimer timer;
            timer.start();

            // Note that HIGHS does not provide any incrementality.
            if (param.getIntegerParam(MPSolverParameters::INCREMENTALITY) == MPSolverParameters::INCREMENTALITY_OFF) {
              Parent::reset();
            }

            if (Parent::_variables.empty() && Parent::_constraints.empty()) {
              Parent::_sync_status = SynchronizationStatus::SOLUTION_SYNCHRONIZED;
              Parent::_result_status = ResultStatus::OPTIMAL;
              Parent::_objective_value = Parent::objective().offset();
              Parent::_best_objective_bound = Parent::objective().offset();
              return Parent::_result_status;
            }

            // Finish preparing the problem.
            switch (Parent::_sync_status) {
              case SynchronizationStatus::MUST_RELOAD: {
                Parent::reset();

                _model.lp_.offset_ = Parent::objective().offset();
                _model.lp_.num_col_ = Parent::_variables.size();
                _model.lp_.num_row_ = Parent::_constraints.size();

                // Here the orientation of the matrix is row-wise
                _model.lp_.a_matrix_.format_ = MatrixFormat::kRowwise;

                // Define variables.
                const int nb_vars = Parent::_variables.size();

                _model.lp_.col_cost_.resize(nb_vars);
                _model.lp_.col_lower_.resize(nb_vars);
                _model.lp_.col_upper_.resize(nb_vars);
                _model.lp_.integrality_.resize(nb_vars);
                _model.lp_.col_names_.resize(nb_vars);

                for (int i = 0; i < nb_vars; ++i) {
                  MPVariable* const var = Parent::_variables[i];
                  assert(var && var->index() == i);

                  Parent::set_variable_as_extracted(i, true);

                  _model.lp_.col_cost_[i] = Parent::objective().getCoefficient(var);
                  _model.lp_.col_lower_[i] = var->lb();
                  _model.lp_.col_upper_[i] = var->ub();
                  _model.lp_.integrality_[i] = var->integer() ? HighsVarType::kInteger : HighsVarType::kContinuous;
                  _model.lp_.col_names_[i].assign(var->name());
                }

                // Define constraints.
                const int nb_cons = Parent::_constraints.size();

                _model.lp_.row_lower_.resize(nb_cons);
                _model.lp_.row_upper_.resize(nb_cons);
                _model.lp_.row_names_.resize(nb_cons);

                _model.lp_.a_matrix_.start_.resize(nb_cons + 1);

                for (int i = 0; i < nb_cons; ++i) {
                  MPConstraint* const ct = Parent::_constraints[i];
                  assert(ct && ct->index() == i);

                  Parent::set_constraint_as_extracted(i, true);

                  _model.lp_.row_lower_[i] = ct->lb();
                  _model.lp_.row_upper_[i] = ct->ub();
                  _model.lp_.row_names_[i].assign(ct->name());

                  _model.lp_.a_matrix_.start_[i] = _model.lp_.a_matrix_.index_.size();

                  const int size = ct->_coefficients.size();
                  for (const auto& entry: ct->_coefficients) {
                    _model.lp_.a_matrix_.index_.push_back(entry.first->index());
                    _model.lp_.a_matrix_.value_.push_back(entry.second);
                  }
                }

                _model.lp_.a_matrix_.start_[nb_cons] = _model.lp_.a_matrix_.index_.size();
                break;
              }
              case SynchronizationStatus::MODEL_SYNCHRONIZED: {
                break;
              }
              case SynchronizationStatus::SOLUTION_SYNCHRONIZED: {
                break;
              }
            }

            _model.lp_.sense_ = Parent::_maximize ? ObjSense::kMaximize : ObjSense::kMinimize;

            Parent::_sync_status = SynchronizationStatus::MODEL_SYNCHRONIZED;
            VLOG_F(1, "Model built in {} seconds.", timer.get());

            ResetBestObjectiveBound();

            // Pass the model to HiGHS
            HighsStatus return_status = _highs.passModel(_model);
            assert(return_status == HighsStatus::kOk);

            // Get a const reference to the LP data in HiGHS
            const HighsLp& lp = _highs.getLp();

            // Set log level.
            _highs.setOptionValue("output_flag", !Parent::quiet());
            _highs.setOptionValue("parallel", "on");

            // Time limit.
            if (Parent::time_limit() != 0) {
              VLOG_F(1, "Setting time limit = {} ms", Parent::time_limit());
              _highs.setOptionValue("time_limit", Parent::time_limit_in_secs());
            }

            // And solve.
            timer.restart();

            return_status = _highs.run();
            assert(return_status == HighsStatus::kOk);

            VLOG_F(1, "Solved in {} seconds.", timer.get());

            // Check the status: optimal, infeasible, etc.
            // Get the model status
            const HighsModelStatus& model_status = _highs.getModelStatus();
            VLOG_F(1, "Highs result status: {}", _highs.modelStatusToString(model_status));

            switch (model_status) {
              case HighsModelStatus::kOptimal: Parent::_result_status = ResultStatus::OPTIMAL; break;
              case HighsModelStatus::kUnboundedOrInfeasible:
              case HighsModelStatus::kUnbounded: Parent::_result_status = ResultStatus::UNBOUNDED; break;
              case HighsModelStatus::kInfeasible: Parent::_result_status = ResultStatus::INFEASIBLE; break;
              case HighsModelStatus::kModelError: Parent::_result_status = ResultStatus::MODEL_INVALID; break;
              default: Parent::_result_status = ResultStatus::ABNORMAL; break;
            }

            // Get the solution information
            const HighsInfo& info = _highs.getInfo();

            const bool has_values = info.primal_solution_status;
            const bool has_duals = info.dual_solution_status;
            const bool has_basis = info.basis_validity;

            Parent::_objective_value = info.objective_function_value;
            VLOG_F(1, "objective={}", Parent::_objective_value);

            if (Parent::_result_status == ResultStatus::OPTIMAL || Parent::_result_status == ResultStatus::FEASIBLE) {
              // Get the solution values and basis
              const HighsSolution& solution = _highs.getSolution();
              // const HighsBasis&    basis = _highs.getBasis();

              // Report the primal and solution values and basis
              for (int col = 0; col < lp.num_col_; ++col) {
                MPVariable* const var = Parent::_variables[col];
                assert(var);
                if (has_values) {
                  const double val = solution.col_value[col];
                  var->set_solution_value(val);
                  VLOG_F(3, "{}={}", var->name(), val);
                }
                if (has_duals) var->set_reduced_cost(solution.col_dual[col]);
                // if (has_basis) cout << "; status: " << _highs.basisStatusToString(basis.col_status[col]);
              }

              for (int row = 0; row < lp.num_row_; ++row) {
                if (has_duals) Parent::_constraints[row]->set_dual_value(solution.row_dual[row]);
                // if (has_basis) cout << "; status: " << _highs.basisStatusToString(basis.row_status[row]);
              }

            } else {
              VLOG_F(1, "No feasible solution found.");
            }

            // iterations_ = model.getIterationCount();
            // nodes_ = model.getNodeCount();
            // Parent::_best_objective_bound = model.getBestPossibleObjValue();
            // VLOG_F(1, "best objective bound={}", Parent::_best_objective_bound);

            Parent::_sync_status = SynchronizationStatus::SOLUTION_SYNCHRONIZED;

            return Parent::_result_status;
          }

          void _impl_Reset() noexcept {
            _model.clear();
            Parent::resetExtractionInformation();
          }

          void _impl_SetVariableBounds(int var_index, double lb, double ub) noexcept {
            Parent::invalidateSolutionSynchronization();
            if (Parent::_sync_status == SynchronizationStatus::MODEL_SYNCHRONIZED) {
              _model.lp_.col_lower_[var_index] = lb;
              _model.lp_.col_upper_[var_index] = ub;
            } else {
              Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
            }
          }
          void _impl_SetVariableInteger(int var_index, bool integer) noexcept {
            Parent::invalidateSolutionSynchronization();
            // TODO(user) : Check if this is actually a change.
            if (Parent::_sync_status == SynchronizationStatus::MODEL_SYNCHRONIZED) {
              _model.lp_.integrality_[var_index] = integer ? HighsVarType::kInteger : HighsVarType::kContinuous;
            } else {
              Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
            }
          }
          void _impl_SetConstraintBounds(int row_index, double lb, double ub) noexcept {
            Parent::invalidateSolutionSynchronization();
            if (Parent::_sync_status == SynchronizationStatus::MODEL_SYNCHRONIZED) {
              _model.lp_.row_lower_[row_index] = lb;
              _model.lp_.row_upper_[row_index] = ub;
            } else {
              Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
            }
          }

          void _impl_AddRowConstraint(MPConstraint* ct) noexcept {
            Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
          }
          void _impl_AddVariable(MPVariable* var) noexcept {
            Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
          }
          void _impl_SetCoefficient(MPConstraint*     constraint,
                                    const MPVariable* variable,
                                    double            new_value,
                                    double            old_value) noexcept {
            Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
          }
          void _impl_ClearConstraint(MPConstraint* constraint) noexcept {
            Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
          }
          void _impl_SetObjectiveCoefficient(const MPVariable* variable, double coefficient) noexcept {
            Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
          }
          void _impl_SetObjectiveOffset(double value) noexcept {
            Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
          }
          void _impl_ClearObjective() noexcept { Parent::_sync_status = SynchronizationStatus::MUST_RELOAD; }

          int64_t _impl_iterations() const noexcept {
            if (!Parent::checkSolutionIsSynchronized()) return nt::mp::details::kUnknownNumberOfIterations;
            return _highs.getInfo().simplex_iteration_count;
          }

          int64_t _impl_nodes() const noexcept {
            if (!Parent::checkSolutionIsSynchronized()) return nt::mp::details::kUnknownNumberOfNodes;
            return nt::mp::details::kUnknownNumberOfNodes;
          }
          BasisStatus _impl_row_status(int constraint_index) const noexcept {
            LOG_F(FATAL, "Basis status only available for continuous problems");
            return BasisStatus::FREE;
          }
          BasisStatus _impl_column_status(int variable_index) const noexcept {
            LOG_F(FATAL, "Basis status only available for continuous problems");
            DCHECK_GE_F(variable_index, 0);
            DCHECK_LT_F(variable_index, Parent::_last_variable_index);
            // const int highs_basis_status = ...;
            // return TransformHIGHSBasisStatus();
            return BasisStatus::FREE;
          }

          bool _impl_HasDualFarkas() const noexcept { return false; }

          constexpr static bool _impl_IsContinuous() { return _impl_IsLP(); }
          constexpr static bool _impl_IsLP() { return !MIP; }
          constexpr static bool _impl_IsMIP() { return MIP; }

          void _impl_ExtractNewVariables() noexcept {}
          void _impl_ExtractNewConstraints() noexcept {}
          void _impl_ExtractObjective() noexcept {}

          std::string _impl_SolverVersion() const noexcept { return highsVersion(); }

          void* _impl_underlying_solver() noexcept { return reinterpret_cast< void* >(&_model); }

          Highs& get_highs() noexcept { return _highs; }

          // See MPSolver::SetCallback() for details.
          void _impl_SetCallback(MPCallback* mp_callback) noexcept {}
          bool _impl_SupportsCallbacks() const noexcept { return false; }

          // private:
          void _impl_SetParameters(const MPSolverParameters& param) noexcept {
            Parent::SetCommonParameters(param);
            Parent::SetMIPParameters(param);
          }
          void _impl_SetRelativeMipGap(double value) noexcept {
            CHECK_GE_F(value, 0);
            _highs.setOptionValue("mip_rel_gap", value);
          }
          void _impl_SetPrimalTolerance(double value) noexcept {
            // Skip the warning for the default value as it coincides with
            // the default value in HIGHS.
            if (value != MPSolverParameters::kDefaultPrimalTolerance) {
              _highs.setOptionValue("primal_feasibility_tolerance", value);
            }
          }
          void _impl_SetDualTolerance(double value) noexcept {
            // Skip the warning for the default value as it coincides with
            // the default value in HIGHS.
            if (value != MPSolverParameters::kDefaultDualTolerance) {
              _highs.setOptionValue("dual_feasibility_tolerance", value);
            }
          }
          void _impl_SetPresolveMode(int value) noexcept {
            switch (value) {
              case MPSolverParameters::PRESOLVE_ON: {
                _highs.setOptionValue("presolve", "on");
                break;
              }
              case MPSolverParameters::PRESOLVE_OFF: {
                _highs.setOptionValue("presolve", "off");
                break;
              }
              default: {
                Parent::SetUnsupportedIntegerParam(MPSolverParameters::PRESOLVE);
              }
            }
          }
          void _impl_SetScalingMode(int scaling) noexcept {
            Parent::SetUnsupportedIntegerParam(MPSolverParameters::SCALING);
          }
          void _impl_SetLpAlgorithm(int lp_algorithm) noexcept {
            // https://ergo-code.github.io/HiGHS/dev/options/definitions/#option-simplex_strategy
            // Strategy for simplex solver 0 => Choose; 1 => Dual (serial); 2 => Dual (PAMI); 3 => Dual (SIP); 4 =>
            // Primal PAMI : parallelism across multiple iterations SIP : single iteration parallelism
            Parent::SetUnsupportedIntegerParam(MPSolverParameters::LP_ALGORITHM);
            switch (lp_algorithm) {
              case MPSolverParameters::DUAL: {
                _highs.setOptionValue("simplex_strategy", 1);
                break;
              }
              case MPSolverParameters::PRIMAL: {
                _highs.setOptionValue("simplex_strategy", 4);
                break;
              }
              default: {
                Parent::SetUnsupportedIntegerParam(MPSolverParameters::PRESOLVE);
              }
            }
          }

          bool _impl_SetNumThreads(int num_threads) noexcept {
            CHECK_GE_F(num_threads, 1);
            _highs.setOptionValue("threads", num_threads);
            return true;
          }
          bool _impl_SetSolverSpecificParametersAsString(const std::string& parameters) noexcept {
            LOG_F(ERROR, "SetSolverSpecificParametersAsString() not currently supported for Dummy.");
            return false;
          }

          void ResetBestObjectiveBound() noexcept {
            if (Parent::_maximize) {
              Parent::_best_objective_bound = NT_DOUBLE_INFINITY;
            } else {
              Parent::_best_objective_bound = -NT_DOUBLE_INFINITY;
            }
          }

          void _impl_Write(const std::string& filename) {
            if (Parent::_sync_status == SynchronizationStatus::MUST_RELOAD) { _impl_Reset(); }
            Parent::extractModel();
            const HighsStatus r = _highs.writeModel(filename);
          }

          HighsModel _model;
          Highs      _highs;
        };


      }   // namespace details

      using MIPModel = details::HighsSolver< true >;
      using LPModel = details::HighsSolver< false >;

      inline constexpr bool hasMIP() noexcept { return true; }
      inline constexpr bool hasLP() noexcept { return true; }

    }   // namespace highs

  }   // namespace mp
}   // namespace nt


#endif