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
 * @file mp_glpk.h
 * @brief GLPK (GNU Linear Programming Kit) solver integration for networktools MP module
 *
 * This header enables GLPK support for both LP and MIP problems.
 * Include this file after networktools.h to use GLPK solver.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 *
 * @par Usage:
 * @code
 * #include "networktools.h"
 * #include "mp/mp_glpk.h"
 *
 * nt::mp::glpk::LPModel lp_model;
 * nt::mp::glpk::MIPModel mip_model;
 * @endcode
 *
 * @par Features:
 * - Both LP and MIP support
 * - Fully open-source (GNU GPL)
 * - Good for educational and research use
 * - No licensing restrictions
 * - Simplex and interior point methods
 *
 * @par Requirements:
 * - GLPK library (GNU Linear Programming Kit)
 * - Link with GLPK library (-lglpk)
 *
 * @note GLPK is slower than commercial solvers but has no licensing costs
 * @see mp.h for API documentation
 * @see README.md for compilation examples
 */

// This file contains a modified version of the source code from Google OR-Tools linear programming
// module. See the appropriate copyright notice below.

// Copyright 2010-2021 Google LLC
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


#ifndef _NT_GLPK_INTERFACE_H_
#define _NT_GLPK_INTERFACE_H_

#include "bits/mp_solver_base.h"

extern "C" {
#include "glpk.h"
}

namespace nt {
  namespace mp {
    namespace glpk {

      struct MPEnv {
        constexpr bool isValid() const noexcept { return true; }
      };

      namespace details {

        namespace {
          // GLPK indexes its variables and constraints starting at 1.
          inline int MPSolverIndexToGlpkIndex(int index) { return index + 1; }
        }   // namespace

        // Function to be called in the GLPK callback
        template < typename GLPKInformation, typename GlpkMPCallbackContext >
        inline void GLPKGatherInformationCallback(glp_tree* tree, void* info) {
          CHECK_F(tree != nullptr);
          CHECK_F(info != nullptr);
          GLPKInformation*      glpk_info = reinterpret_cast< GLPKInformation* >(info);
          const int             reason = glp_ios_reason(tree);
          GlpkMPCallbackContext mp_context(tree, reason);

          switch (reason) {
            // The best bound and the number of nodes change only when GLPK
            // branches, generates cuts or finds an integer solution.
            case GLP_IBINGO:
            case GLP_ICUTGEN:
            case GLP_IROWGEN:
              if (glpk_info->mp_callback_) glpk_info->mp_callback_->runCallback(&mp_context);
            case GLP_ISELECT: {
              // Get total number of nodes
              glp_ios_tree_size(tree, nullptr, nullptr, &glpk_info->num_all_nodes_);
              // Get best bound
              int node_id = glp_ios_best_node(tree);
              if (node_id > 0) { glpk_info->_best_objective_bound = glp_ios_node_bound(tree, node_id); }
              break;
            }
            default: break;
          }
        }

        // ----- GLPK Solver -----

        template < bool MIP >
        struct GLPKInterface: public nt::mp::details::MPSolver< GLPKInterface< MIP > > {
          using Parent = nt::mp::details::MPSolver< GLPKInterface< MIP > >;
          using MPObjective = Parent::MPObjective;
          using MPConstraint = Parent::MPConstraint;
          using MPVariable = Parent::MPVariable;
          // using MPCallback = Parent::MPCallback;
          using LinearRange = Parent::LinearRange;

          using Parent::Parent;

          struct GlpkMPCallbackContext: Parent::template MPCallbackContextInterface< GlpkMPCallbackContext > {
            glp_tree* tree;
            int       reason;
            bool      bConstraintsAdded;

            nt::TrivialDynamicArray< double > Values;
            nt::TrivialDynamicArray< int >    CutInd;
            nt::TrivialDynamicArray< double > CutVal;

            GlpkMPCallbackContext(glp_tree* tree, int reason) : tree(tree), reason(reason), bConstraintsAdded(false) {}

            MPCallbackEvent event() {
              switch (reason) {
                case GLP_IBINGO:
                case GLP_ICUTGEN:
                case GLP_IROWGEN:
                  // GLPK has found an integer feasible solution, and the user has the
                  // option to generate a lazy constraint not satisfied by that solution.
                  return MPCallbackEvent::kMipSolution;

                default: return MPCallbackEvent::kUnknown;
              }

              return MPCallbackEvent::kUnknown;
            }

            bool canQueryVariableValues() { return true; }

            double variableValue(const MPVariable* pVariable) {
              CHECK_F(canQueryVariableValues());
              if (!pVariable) return nt::mp::infinity();
              glp_prob*    lp = glp_ios_get_prob(tree);
              const double val = glp_get_col_prim(lp, MPSolverIndexToGlpkIndex(pVariable->index()));
              return val;
            }

            void addCut(const LinearRange& cutting_plane) { LOG_F(FATAL, "addCut() not yet implemented for GLPK."); }

            void addPricedVariable(const MPVariable* pVariable) {
              LOG_F(FATAL, "addPricedVariable() not yet implemented for GLPK.");
            }

            void addLazyConstraint(const LinearRange& lazy_constraint) {
              glp_prob* lp = glp_ios_get_prob(tree);

              // Add a new row
              const int i = glp_add_rows(lp, 1);   // Todo : Add a generic method to reserve a chunk of rows
                                                   // instead of calling  glp_add_rows(lp, 1) multiple times.
              glp_set_row_name(lp, i, nt::format("lazy_{}", i).c_str());

              // Set the bounds
              const double ub = lazy_constraint.upper_bound();
              const double lb = lazy_constraint.lower_bound();
              const double infinity = nt::mp::infinity();
              if (lb != -infinity) {
                if (ub != infinity) {
                  if (lb == ub) {
                    glp_set_row_bnds(lp, i, GLP_FX, lb, ub);
                  } else {
                    glp_set_row_bnds(lp, i, GLP_DB, lb, ub);
                  }
                } else {
                  glp_set_row_bnds(lp, i, GLP_LO, lb, 0.0);
                }
              } else if (ub != infinity) {
                glp_set_row_bnds(lp, i, GLP_UP, 0.0, ub);
              } else {
                glp_set_row_bnds(lp, i, GLP_FR, 0.0, 0.0);
              }

              // Set the coeffcient, GLPK convention is to start indexing at 1... ¯\_(ツ)_/¯
              const nt::PointerMap< const MPVariable*, double >& Terms = lazy_constraint.linear_expr().terms();
              const int                                          iNumTerms = Terms.size();
              CutInd.ensureAndFillBytes(iNumTerms + 1, 0);
              CutVal.ensureAndFillBytes(iNumTerms + 1, 0);
              int k = 1;
              for (const auto& e: Terms) {
                CutInd[k] = MPSolverIndexToGlpkIndex(e.first->index());
                CutVal[k] = e.second;
                ++k;
              }

              glp_set_mat_row(lp, i, k - 1, CutInd._p_data, CutVal._p_data);

              bConstraintsAdded = true;
            }

            double suggestSolution(const nt::PointerMap< const MPVariable*, double >& solution) {
              LOG_F(FATAL, "suggestSolution() not currently supported for GLPK.");
              return 0.;
            }

            int64_t numExploredNodes() {
              int iNodeCount;
              glp_ios_tree_size(tree, nullptr, nullptr, &iNodeCount);
              return iNodeCount;
            }
          };

          using MPCallbackContext = GlpkMPCallbackContext;
          using MPCallback = Parent::template MPCallback< MPCallbackContext >;
          using MPCallbackList = Parent::template MPCallbackList< MPCallbackContext >;

          // Class to store information gathered in the callback
          struct GLPKInformation {
            explicit GLPKInformation(bool maximize) : num_all_nodes_(0), mp_callback_(nullptr) {
              ResetBestObjectiveBound(maximize);
            }
            void Reset(bool maximize) {
              num_all_nodes_ = 0;
              ResetBestObjectiveBound(maximize);
            }
            void ResetBestObjectiveBound(bool maximize) {
              if (maximize) {
                _best_objective_bound = nt::mp::infinity();
              } else {
                _best_objective_bound = -nt::mp::infinity();
              }
            }
            int         num_all_nodes_;
            double      _best_objective_bound;
            MPCallback* mp_callback_;
          };

          // Constructor that takes a name for the underlying glpk solver.
          GLPKInterface(const std::string& name, const MPEnv& e);
          ~GLPKInterface();

          // Sets the optimization direction (min/max).
          void _impl_SetOptimizationDirection(bool maximize);

          // ----- Solve -----
          // Solve the problem using the parameter values specified.
          ResultStatus _impl_Solve(const MPSolverParameters& param);

          // ----- Model modifications and extraction -----
          // Resets extracted model
          void _impl_Reset();

          void _impl_FreeModel();

          // Modify bounds.
          void _impl_SetVariableBounds(int mpsolver_var_index, double lb, double ub);
          void _impl_SetVariableInteger(int mpsolver_var_index, bool integer);
          void _impl_SetConstraintBounds(int mpsolver_constraint_index, double lb, double ub);

          // Add Constraint incrementally.
          void _impl_AddRowConstraint(MPConstraint* const ct);
          // Add variable incrementally.
          void _impl_AddVariable(MPVariable* const var);
          // Change a coefficient in a constraint.
          void _impl_SetCoefficient(MPConstraint* const     constraint,
                                    const MPVariable* const variable,
                                    double                  new_value,
                                    double                  old_value);
          // Clear a constraint from all its terms.
          void _impl_ClearConstraint(MPConstraint* const constraint);
          // Change a coefficient in the linear objective
          void _impl_SetObjectiveCoefficient(const MPVariable* const variable, double coefficient);
          // Change the constant term in the linear objective.
          void _impl_SetObjectiveOffset(double value);
          // Clear the objective from all its terms.
          void _impl_ClearObjective();

          // ------ Query statistics on the solution and the solve ------
          // Number of simplex iterations
          int64_t _impl_iterations() const;
          // Number of branch-and-bound nodes. Only available for discrete problems.
          int64_t _impl_nodes() const;

          // Returns the basis status of a row.
          BasisStatus _impl_row_status(int constraint_index) const;
          // Returns the basis status of a column.
          BasisStatus _impl_column_status(int variable_index) const;

          // Checks whether a feasible solution exists.
          bool _impl_CheckSolutionExists() const;

          bool _impl_HasDualFarkas() const { return false; }

          // ----- Misc -----
          // Query problem type.
          constexpr static bool _impl_IsContinuous() { return _impl_IsLP(); }
          constexpr static bool _impl_IsLP() { return !MIP; }
          constexpr static bool _impl_IsMIP() { return MIP; }

          void _impl_ExtractNewVariables();
          void _impl_ExtractNewConstraints();
          void _impl_ExtractObjective();

          std::string _impl_SolverVersion() const { return nt::format("GLPK {}", glp_version()); }

          bool _impl_SupportsCallbacks() const { return true; }

          void _impl_SetCallback(MPCallback* mp_callback) {
            if (callback_ != nullptr) { callback_reset_ = true; }
            callback_ = mp_callback;
          };


          void* _impl_underlying_solver() { return reinterpret_cast< void* >(lp_); }

          double _impl_ComputeExactConditionNumber() const;

          // private:

          // Configure the solver's parameters.
          void ConfigureGLPKParameters(const MPSolverParameters& param);

          // Set all parameters in the underlying solver.
          void _impl_SetParameters(const MPSolverParameters& param);
          // Set each parameter in the underlying solver.
          void _impl_SetRelativeMipGap(double value);
          void _impl_SetPrimalTolerance(double value);
          void _impl_SetDualTolerance(double value);
          void _impl_SetPresolveMode(int value);
          void _impl_SetScalingMode(int value);
          void _impl_SetLpAlgorithm(int value);

          void ExtractOldConstraints();
          void ExtractOneConstraint(MPConstraint* const constraint, int* const indices, double* const coefs);
          // Transforms basis status from GLPK integer code to BasisStatus.
          BasisStatus TransformGLPKBasisStatus(int glpk_basis_status) const;

          // Computes the L1-norm of the current scaled basis.
          // The L1-norm |A| is defined as max_j sum_i |a_ij|
          // This method is available only for continuous problems.
          double ComputeScaledBasisL1Norm(int     num_rows,
                                          int     num_cols,
                                          double* row_scaling_factor,
                                          double* column_scaling_factor) const;

          // Computes the L1-norm of the inverse of the current scaled
          // basis.
          // This method is available only for continuous problems.
          double ComputeInverseScaledBasisL1Norm(int     num_rows,
                                                 int     num_cols,
                                                 double* row_scaling_factor,
                                                 double* column_scaling_factor) const;

          glp_prob* lp_;

          // Parameters
          glp_smcp lp_param_;
          glp_iocp mip_param_;
          // For the callback
          std::unique_ptr< GLPKInformation > mip_callback_info_;
          MPCallback*                        callback_ = nullptr;
          bool                               callback_reset_ = false;
        };

        // Creates a LP/MIP instance with the specified name and minimization objective.
        template < bool MIP >
        inline GLPKInterface< MIP >::GLPKInterface(const std::string& name, const MPEnv& e) :
            Parent(name), lp_(nullptr) {
          // Make sure glp_free_env() is called at the exit of the current thread.
          // SetupGlpkEnvAutomaticDeletion();

          lp_ = glp_create_prob();
          glp_set_prob_name(lp_, Parent::_name.c_str());
          glp_set_obj_dir(lp_, GLP_MIN);
          mip_callback_info_ = std::make_unique< GLPKInformation >(Parent::_maximize);
        }

        // Frees the LP memory allocations.
        template < bool MIP >
        inline GLPKInterface< MIP >::~GLPKInterface() {
          CHECK_F(lp_ != nullptr);
          glp_delete_prob(lp_);
          lp_ = nullptr;
        }

        template < bool MIP >
        inline void GLPKInterface< MIP >::_impl_Reset() {
          CHECK_F(lp_ != nullptr);
          glp_delete_prob(lp_);
          lp_ = glp_create_prob();
          glp_set_prob_name(lp_, Parent::_name.c_str());
          glp_set_obj_dir(lp_, Parent::_maximize ? GLP_MAX : GLP_MIN);
          Parent::resetExtractionInformation();
        }


        // ------ Model modifications and extraction -----

        // Not cached
        template < bool MIP >
        inline void GLPKInterface< MIP >::_impl_SetOptimizationDirection(bool maximize) {
          Parent::invalidateSolutionSynchronization();
          glp_set_obj_dir(lp_, maximize ? GLP_MAX : GLP_MIN);
        }

        template < bool MIP >
        inline void GLPKInterface< MIP >::_impl_SetVariableBounds(int mpsolver_var_index, double lb, double ub) {
          Parent::invalidateSolutionSynchronization();
          if (!Parent::variable_is_extracted(mpsolver_var_index)) {
            Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
            return;
          }
          // Not cached if the variable has been extracted.
          DCHECK_F(lp_ != nullptr);
          const double infinity = nt::mp::infinity();
          const int    glpk_var_index = MPSolverIndexToGlpkIndex(mpsolver_var_index);
          if (lb != -infinity) {
            if (ub != infinity) {
              if (lb == ub) {
                glp_set_col_bnds(lp_, glpk_var_index, GLP_FX, lb, ub);
              } else {
                glp_set_col_bnds(lp_, glpk_var_index, GLP_DB, lb, ub);
              }
            } else {
              glp_set_col_bnds(lp_, glpk_var_index, GLP_LO, lb, 0.0);
            }
          } else if (ub != infinity) {
            glp_set_col_bnds(lp_, glpk_var_index, GLP_UP, 0.0, ub);
          } else {
            glp_set_col_bnds(lp_, glpk_var_index, GLP_FR, 0.0, 0.0);
          }
        }

        template < bool MIP >
        inline void GLPKInterface< MIP >::_impl_SetVariableInteger(int mpsolver_var_index, bool integer) {
          Parent::invalidateSolutionSynchronization();
          if constexpr (_impl_IsMIP()) {
            if (Parent::variable_is_extracted(mpsolver_var_index)) {
              // Not cached if the variable has been extracted.
              glp_set_col_kind(lp_, MPSolverIndexToGlpkIndex(mpsolver_var_index), integer ? GLP_IV : GLP_CV);
            } else {
              Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
            }
          }
        }

        template < bool MIP >
        inline void
           GLPKInterface< MIP >::_impl_SetConstraintBounds(int mpsolver_constraint_index, double lb, double ub) {
          Parent::invalidateSolutionSynchronization();
          if (!Parent::constraint_is_extracted(mpsolver_constraint_index)) {
            Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
            return;
          }
          // Not cached if the row has been extracted
          const int glpk_constraint_index = MPSolverIndexToGlpkIndex(mpsolver_constraint_index);
          DCHECK_F(lp_ != nullptr);
          const double infinity = nt::mp::infinity();
          if (lb != -infinity) {
            if (ub != infinity) {
              if (lb == ub) {
                glp_set_row_bnds(lp_, glpk_constraint_index, GLP_FX, lb, ub);
              } else {
                glp_set_row_bnds(lp_, glpk_constraint_index, GLP_DB, lb, ub);
              }
            } else {
              glp_set_row_bnds(lp_, glpk_constraint_index, GLP_LO, lb, 0.0);
            }
          } else if (ub != infinity) {
            glp_set_row_bnds(lp_, glpk_constraint_index, GLP_UP, 0.0, ub);
          } else {
            glp_set_row_bnds(lp_, glpk_constraint_index, GLP_FR, 0.0, 0.0);
          }
        }

        template < bool MIP >
        inline void GLPKInterface< MIP >::_impl_SetCoefficient(MPConstraint* const     constraint,
                                                               const MPVariable* const variable,
                                                               double                  new_value,
                                                               double                  old_value) {
          Parent::invalidateSolutionSynchronization();
          // GLPK does not allow to modify one coefficient at a time, so we
          // extract the whole constraint again, if it has been extracted
          // already and if it does not contain new variables. Otherwise, we
          // cache the modification.
          if (Parent::constraint_is_extracted(constraint->index())
              && (Parent::_sync_status == SynchronizationStatus::MODEL_SYNCHRONIZED
                  || !constraint->containsNewVariables())) {
            const int                   size = constraint->_coefficients.size();
            std::unique_ptr< int[] >    indices(new int[size + 1]);
            std::unique_ptr< double[] > coefs(new double[size + 1]);
            ExtractOneConstraint(constraint, indices.get(), coefs.get());
          }
        }

        // Not cached
        template < bool MIP >
        inline void GLPKInterface< MIP >::_impl_ClearConstraint(MPConstraint* const constraint) {
          Parent::invalidateSolutionSynchronization();
          // Constraint may have not been extracted yet.
          if (Parent::constraint_is_extracted(constraint->index())) {
            glp_set_mat_row(lp_, MPSolverIndexToGlpkIndex(constraint->index()), 0, nullptr, nullptr);
          }
        }

        // Cached
        template < bool MIP >
        inline void GLPKInterface< MIP >::_impl_SetObjectiveCoefficient(const MPVariable* const variable,
                                                                        double                  coefficient) {
          Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
        }

        // Cached
        template < bool MIP >
        inline void GLPKInterface< MIP >::_impl_SetObjectiveOffset(double value) {
          Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
        }

        // Clear objective of all its terms (linear)
        template < bool MIP >
        inline void GLPKInterface< MIP >::_impl_ClearObjective() {
          Parent::invalidateSolutionSynchronization();
          for (const auto& entry: Parent::_objective._coefficients) {
            const int mpsolver_var_index = entry.first->index();
            // Variable may have not been extracted yet.
            if (!Parent::variable_is_extracted(mpsolver_var_index)) {
              DCHECK_NE_F(SynchronizationStatus::MODEL_SYNCHRONIZED, Parent::_sync_status);
            } else {
              glp_set_obj_coef(lp_, MPSolverIndexToGlpkIndex(mpsolver_var_index), 0.0);
            }
          }
          // Constant term.
          glp_set_obj_coef(lp_, 0, 0.0);
        }

        template < bool MIP >
        inline void GLPKInterface< MIP >::_impl_AddRowConstraint(MPConstraint* const ct) {
          Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
        }

        template < bool MIP >
        inline void GLPKInterface< MIP >::_impl_AddVariable(MPVariable* const var) {
          Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
        }

        // Define new variables and add them to existing constraints.
        template < bool MIP >
        inline void GLPKInterface< MIP >::_impl_ExtractNewVariables() {
          int total_num_vars = Parent::_variables.size();
          if (total_num_vars > Parent::_last_variable_index) {
            glp_add_cols(lp_, total_num_vars - Parent::_last_variable_index);
            for (int j = Parent::_last_variable_index; j < Parent::_variables.size(); ++j) {
              MPVariable* const var = Parent::_variables[j];
              Parent::set_variable_as_extracted(j, true);
              if (!var->name().empty()) { glp_set_col_name(lp_, MPSolverIndexToGlpkIndex(j), var->name().c_str()); }
              _impl_SetVariableBounds(/*mpsolver_var_index=*/j, var->lb(), var->ub());
              _impl_SetVariableInteger(/*mpsolver_var_index=*/j, var->integer());

              // The true objective coefficient will be set later in ExtractObjective.
              double tmp_obj_coef = 0.0;
              glp_set_obj_coef(lp_, MPSolverIndexToGlpkIndex(j), tmp_obj_coef);
            }
            // Add new variables to the existing constraints.
            ExtractOldConstraints();
          }
        }

        // Extract again existing constraints if they contain new variables.
        template < bool MIP >
        inline void GLPKInterface< MIP >::ExtractOldConstraints() {
          const int max_constraint_size = Parent::computeMaxConstraintSize(0, Parent::_last_constraint_index);
          // The first entry in the following arrays is dummy, to be
          // consistent with glpk API.
          std::unique_ptr< int[] >    indices(new int[max_constraint_size + 1]);
          std::unique_ptr< double[] > coefs(new double[max_constraint_size + 1]);

          for (int i = 0; i < Parent::_last_constraint_index; ++i) {
            MPConstraint* const ct = Parent::_constraints[i];
            DCHECK_F(Parent::constraint_is_extracted(i));
            const int size = ct->_coefficients.size();
            if (size == 0) { continue; }
            // Update the constraint's coefficients if it contains new variables.
            if (ct->containsNewVariables()) { ExtractOneConstraint(ct, indices.get(), coefs.get()); }
          }
        }

        // Extract one constraint. Arrays indices and coefs must be
        // preallocated to have enough space to contain the constraint's
        // coefficients.
        template < bool MIP >
        inline void GLPKInterface< MIP >::ExtractOneConstraint(MPConstraint* const constraint,
                                                               int* const          indices,
                                                               double* const       coefs) {
          // GLPK convention is to start indexing at 1.
          int k = 1;
          for (const auto& entry: constraint->_coefficients) {
            DCHECK_F(Parent::variable_is_extracted(entry.first->index()));
            indices[k] = MPSolverIndexToGlpkIndex(entry.first->index());
            coefs[k] = entry.second;
            ++k;
          }
          glp_set_mat_row(lp_, MPSolverIndexToGlpkIndex(constraint->index()), k - 1, indices, coefs);
        }

        // Define new constraints on old and new variables.
        template < bool MIP >
        inline void GLPKInterface< MIP >::_impl_ExtractNewConstraints() {
          int total_num_rows = Parent::_constraints.size();
          if (Parent::_last_constraint_index < total_num_rows) {
            // Define new constraints
            glp_add_rows(lp_, total_num_rows - Parent::_last_constraint_index);
            int num_coefs = 0;
            for (int i = Parent::_last_constraint_index; i < total_num_rows; ++i) {
              MPConstraint* ct = Parent::_constraints[i];
              Parent::set_constraint_as_extracted(i, true);
              if (ct->name().empty()) {
                glp_set_row_name(lp_, MPSolverIndexToGlpkIndex(i), nt::format("ct_{}", i).c_str());
              } else {
                glp_set_row_name(lp_, MPSolverIndexToGlpkIndex(i), ct->name().c_str());
              }
              // All constraints are set to be of the type <= limit_ .
              _impl_SetConstraintBounds(/*mpsolver_constraint_index=*/i, ct->lb(), ct->ub());
              num_coefs += ct->_coefficients.size();
            }

            // Fill new constraints with coefficients
            if (Parent::_last_variable_index == 0 && Parent::_last_constraint_index == 0) {
              // Faster extraction when nothing has been extracted yet: build
              // and load whole matrix at once instead of constructing rows
              // separately.

              // The first entry in the following arrays is dummy, to be
              // consistent with glpk API.
              std::unique_ptr< int[] >    variable_indices(new int[num_coefs + 1]);
              std::unique_ptr< int[] >    constraint_indices(new int[num_coefs + 1]);
              std::unique_ptr< double[] > coefs(new double[num_coefs + 1]);
              int                         k = 1;
              for (int i = 0; i < Parent::_constraints.size(); ++i) {
                MPConstraint* ct = Parent::_constraints[i];
                for (const auto& entry: ct->_coefficients) {
                  DCHECK_F(Parent::variable_is_extracted(entry.first->index()));
                  constraint_indices[k] = MPSolverIndexToGlpkIndex(ct->index());
                  variable_indices[k] = MPSolverIndexToGlpkIndex(entry.first->index());
                  coefs[k] = entry.second;
                  ++k;
                }
              }
              CHECK_EQ_F(num_coefs + 1, k);
              glp_load_matrix(lp_, num_coefs, constraint_indices.get(), variable_indices.get(), coefs.get());
            } else {
              // Build each new row separately.
              int max_constraint_size =
                 Parent::computeMaxConstraintSize(Parent::_last_constraint_index, total_num_rows);
              // The first entry in the following arrays is dummy, to be
              // consistent with glpk API.
              std::unique_ptr< int[] >    indices(new int[max_constraint_size + 1]);
              std::unique_ptr< double[] > coefs(new double[max_constraint_size + 1]);
              for (int i = Parent::_last_constraint_index; i < total_num_rows; i++) {
                ExtractOneConstraint(Parent::_constraints[i], indices.get(), coefs.get());
              }
            }
          }
        }

        template < bool MIP >
        inline void GLPKInterface< MIP >::_impl_ExtractObjective() {
          // Linear objective: set objective coefficients for all variables
          // (some might have been modified).
          for (const auto& entry: Parent::_objective._coefficients) {
            glp_set_obj_coef(lp_, MPSolverIndexToGlpkIndex(entry.first->index()), entry.second);
          }
          // Constant term.
          glp_set_obj_coef(lp_, 0, Parent::objective().offset());
        }

        // Solve the problem using the parameter values specified.
        template < bool MIP >
        inline ResultStatus GLPKInterface< MIP >::_impl_Solve(const MPSolverParameters& param) {
          nt::WallTimer timer;
          timer.start();

          // Note that GLPK provides incrementality for LP but not for MIP.
          if (param.getIntegerParam(MPSolverParameters::INCREMENTALITY) == MPSolverParameters::INCREMENTALITY_OFF) {
            _impl_Reset();
          }

          // Set log level.
          if (Parent::quiet()) {
            glp_term_out(GLP_OFF);
          } else {
            glp_term_out(GLP_ON);
          }

          Parent::extractModel();
          VLOG_F(1, "Model built in {} seconds.", timer.get());

          // Configure parameters at every solve, even when the model has not
          // been changed, in case some of the parameters such as the time
          // limit have been changed since the last solve.
          ConfigureGLPKParameters(param);

          // Solve
          timer.restart();
          int solver_status = glp_simplex(lp_, &lp_param_);
          if constexpr (_impl_IsMIP()) {
            // glp_intopt requires to solve the root LP separately.
            // If the root LP was solved successfully, solve the MIP.
            if (solver_status == 0) {
              solver_status = glp_intopt(lp_, &mip_param_);
            } else {
              // Something abnormal occurred during the root LP solve. It is
              // highly unlikely that an integer feasible solution is
              // available at this point, so we don't put any effort in trying
              // to recover it.
              Parent::_result_status = ResultStatus::ABNORMAL;
              if (solver_status == GLP_ETMLIM) { Parent::_result_status = ResultStatus::NOT_SOLVED; }
              Parent::_sync_status = SynchronizationStatus::SOLUTION_SYNCHRONIZED;
              return Parent::_result_status;
            }
          }
          VLOG_F(1, nt::format("GLPK Status: {} (time spent: {} seconds).", solver_status, timer.get()).c_str());

          // Get the results.
          if constexpr (_impl_IsMIP()) {
            Parent::_objective_value = glp_mip_obj_val(lp_);
            Parent::_best_objective_bound = mip_callback_info_->_best_objective_bound;
          } else {
            Parent::_objective_value = glp_get_obj_val(lp_);
          }
          VLOG_F(1, "objective= {}, bound={}", Parent::_objective_value, Parent::_best_objective_bound);
          for (int i = 0; i < Parent::_variables.size(); ++i) {
            MPVariable* const var = Parent::_variables[i];
            double            val;
            if constexpr (_impl_IsMIP()) {
              val = glp_mip_col_val(lp_, MPSolverIndexToGlpkIndex(i));
            } else {
              val = glp_get_col_prim(lp_, MPSolverIndexToGlpkIndex(i));
            }
            var->set_solution_value(val);
            VLOG_F(3, "{}: value ={}", var->name(), val);
            if constexpr (!_impl_IsMIP()) {
              double reduced_cost;
              reduced_cost = glp_get_col_dual(lp_, MPSolverIndexToGlpkIndex(i));
              var->set_reduced_cost(reduced_cost);
              VLOG_F(4, "{}: reduced cost = {}", var->name(), reduced_cost);
            }
          }
          for (int i = 0; i < Parent::_constraints.size(); ++i) {
            MPConstraint* const ct = Parent::_constraints[i];
            if constexpr (!_impl_IsMIP()) {
              const double dual_value = glp_get_row_dual(lp_, MPSolverIndexToGlpkIndex(i));
              ct->set_dual_value(dual_value);
              VLOG_F(4, "row {}: dual value = {}", MPSolverIndexToGlpkIndex(i), dual_value);
            }
          }

          // Check the status: optimal, infeasible, etc.
          if constexpr (_impl_IsMIP()) {
            int tmp_status = glp_mip_status(lp_);
            VLOG_F(1, "GLPK result status: {}", tmp_status);
            if (tmp_status == GLP_OPT) {
              Parent::_result_status = ResultStatus::OPTIMAL;
            } else if (tmp_status == GLP_FEAS) {
              Parent::_result_status = ResultStatus::FEASIBLE;
            } else if (tmp_status == GLP_NOFEAS) {
              // For infeasible problems, GLPK actually seems to return
              // GLP_UNDEF. So this is never (?) reached.  Return infeasible
              // in case GLPK returns a correct status in future versions.
              Parent::_result_status = ResultStatus::INFEASIBLE;
            } else if (solver_status == GLP_ETMLIM) {
              Parent::_result_status = ResultStatus::NOT_SOLVED;
            } else {
              Parent::_result_status = ResultStatus::ABNORMAL;
              // GLPK does not have a status code for unbounded MIP models, so
              // we return an abnormal status instead.
            }
          } else {
            int tmp_status = glp_get_status(lp_);
            VLOG_F(1, "GLPK result status: {}", tmp_status);
            if (tmp_status == GLP_OPT) {
              Parent::_result_status = ResultStatus::OPTIMAL;
            } else if (tmp_status == GLP_FEAS) {
              Parent::_result_status = ResultStatus::FEASIBLE;
            } else if (tmp_status == GLP_NOFEAS || tmp_status == GLP_INFEAS) {
              // For infeasible problems, GLPK actually seems to return
              // GLP_UNDEF. So this is never (?) reached.  Return infeasible
              // in case GLPK returns a correct status in future versions.
              Parent::_result_status = ResultStatus::INFEASIBLE;
            } else if (tmp_status == GLP_UNBND) {
              // For unbounded problems, GLPK actually seems to return
              // GLP_UNDEF. So this is never (?) reached.  Return unbounded
              // in case GLPK returns a correct status in future versions.
              Parent::_result_status = ResultStatus::UNBOUNDED;
            } else if (solver_status == GLP_ETMLIM) {
              Parent::_result_status = ResultStatus::NOT_SOLVED;
            } else {
              Parent::_result_status = ResultStatus::ABNORMAL;
            }
          }

          Parent::_sync_status = SynchronizationStatus::SOLUTION_SYNCHRONIZED;

          return Parent::_result_status;
        }

        template < bool MIP >
        inline BasisStatus GLPKInterface< MIP >::TransformGLPKBasisStatus(int glpk_basis_status) const {
          switch (glpk_basis_status) {
            case GLP_BS: return BasisStatus::BASIC;
            case GLP_NL: return BasisStatus::AT_LOWER_BOUND;
            case GLP_NU: return BasisStatus::AT_UPPER_BOUND;
            case GLP_NF: return BasisStatus::FREE;
            case GLP_NS: return BasisStatus::FIXED_VALUE;
            default: LOG_F(FATAL, "Unknown GLPK basis status"); return BasisStatus::FREE;
          }
        }

        // ------ Query statistics on the solution and the solve ------

        template < bool MIP >
        inline int64_t GLPKInterface< MIP >::_impl_iterations() const {
#if GLP_MAJOR_VERSION == 4 && GLP_MINOR_VERSION < 49
          if (!_impl_IsMIP() && Parent::checkSolutionIsSynchronized()) { return lpx_get_int_parm(lp_, LPX_K_ITCNT); }
#elif (GLP_MAJOR_VERSION == 4 && GLP_MINOR_VERSION >= 53) || GLP_MAJOR_VERSION >= 5
          if (!_impl_IsMIP() && Parent::checkSolutionIsSynchronized()) { return glp_get_it_cnt(lp_); }
#endif
          LOG_F(WARNING, "Total number of iterations is not available");
          return nt::mp::details::kUnknownNumberOfIterations;
        }

        template < bool MIP >
        inline int64_t GLPKInterface< MIP >::_impl_nodes() const {
          if constexpr (_impl_IsMIP()) {
            if (!Parent::checkSolutionIsSynchronized()) return nt::mp::details::kUnknownNumberOfNodes;
            return mip_callback_info_->num_all_nodes_;
          } else {
            LOG_F(FATAL, "Number of nodes only available for discrete problems");
            return nt::mp::details::kUnknownNumberOfNodes;
          }
        }

        template < bool MIP >
        inline BasisStatus GLPKInterface< MIP >::_impl_row_status(int constraint_index) const {
          DCHECK_GE_F(constraint_index, 0);
          DCHECK_LT_F(constraint_index, Parent::_last_constraint_index);
          const int glpk_basis_status = glp_get_row_stat(lp_, MPSolverIndexToGlpkIndex(constraint_index));
          return TransformGLPKBasisStatus(glpk_basis_status);
        }

        template < bool MIP >
        inline BasisStatus GLPKInterface< MIP >::_impl_column_status(int variable_index) const {
          DCHECK_GE_F(variable_index, 0);
          DCHECK_LT_F(variable_index, Parent::_last_variable_index);
          const int glpk_basis_status = glp_get_col_stat(lp_, MPSolverIndexToGlpkIndex(variable_index));
          return TransformGLPKBasisStatus(glpk_basis_status);
        }

        template < bool MIP >
        inline bool GLPKInterface< MIP >::_impl_CheckSolutionExists() const {
          if (Parent::_result_status == ResultStatus::ABNORMAL) {
            LOG_F(WARNING,
                  "Ignoring ABNORMAL status from GLPK: This status may or may not indicate that a "
                  "solution exists.");
            return true;
          } else {
            // Call default implementation
            return Parent::checkSolutionExists();
          }
        }

        template < bool MIP >
        inline double GLPKInterface< MIP >::_impl_ComputeExactConditionNumber() const {
          if (!_impl_IsContinuous()) {
            // TODO(user): support MIP.
            LOG_F(FATAL, "ComputeExactConditionNumber not implemented for GLPK_MIXED_INTEGER_PROGRAMMING");
            return 0.0;
          }
          if (!Parent::checkSolutionIsSynchronized()) return 0.0;
          // Simplex is the only LP algorithm supported in the wrapper for
          // GLPK, so when a solution exists, a basis exists.
          _impl_CheckSolutionExists();
          const int num_rows = glp_get_num_rows(lp_);
          const int num_cols = glp_get_num_cols(lp_);
          // GLPK indexes everything starting from 1 instead of 0.
          std::unique_ptr< double[] > row_scaling_factor(new double[num_rows + 1]);
          std::unique_ptr< double[] > column_scaling_factor(new double[num_cols + 1]);
          for (int row = 1; row <= num_rows; ++row) {
            row_scaling_factor[row] = glp_get_rii(lp_, row);
          }
          for (int col = 1; col <= num_cols; ++col) {
            column_scaling_factor[col] = glp_get_sjj(lp_, col);
          }
          return ComputeInverseScaledBasisL1Norm(
                    num_rows, num_cols, row_scaling_factor.get(), column_scaling_factor.get())
                 * ComputeScaledBasisL1Norm(num_rows, num_cols, row_scaling_factor.get(), column_scaling_factor.get());
        }

        template < bool MIP >
        inline double GLPKInterface< MIP >::ComputeScaledBasisL1Norm(int     num_rows,
                                                                     int     num_cols,
                                                                     double* row_scaling_factor,
                                                                     double* column_scaling_factor) const {
          double                      norm = 0.0;
          std::unique_ptr< double[] > values(new double[num_rows + 1]);
          std::unique_ptr< int[] >    indices(new int[num_rows + 1]);
          for (int col = 1; col <= num_cols; ++col) {
            const int glpk_basis_status = glp_get_col_stat(lp_, col);
            // Take into account only basic columns.
            if (glpk_basis_status == GLP_BS) {
              // Compute L1-norm of column 'col': sum_row |a_row,col|.
              const int num_nz = glp_get_mat_col(lp_, col, indices.get(), values.get());
              double    column_norm = 0.0;
              for (int k = 1; k <= num_nz; k++) {
                column_norm += fabs(values[k] * row_scaling_factor[indices[k]]);
              }
              column_norm *= fabs(column_scaling_factor[col]);
              // Compute max_col column_norm
              norm = std::max(norm, column_norm);
            }
          }
          // Slack variables.
          for (int row = 1; row <= num_rows; ++row) {
            const int glpk_basis_status = glp_get_row_stat(lp_, row);
            // Take into account only basic slack variables.
            if (glpk_basis_status == GLP_BS) {
              // Only one non-zero coefficient: +/- 1.0 in the corresponding
              // row. The row has a scaling coefficient but the slack variable
              // is never scaled on top of that.
              const double column_norm = fabs(row_scaling_factor[row]);
              // Compute max_col column_norm
              norm = std::max(norm, column_norm);
            }
          }
          return norm;
        }

        template < bool MIP >
        inline double GLPKInterface< MIP >::ComputeInverseScaledBasisL1Norm(int     num_rows,
                                                                            int     num_cols,
                                                                            double* row_scaling_factor,
                                                                            double* column_scaling_factor) const {
          // Compute the LU factorization if it doesn't exist yet.
          if (!glp_bf_exists(lp_)) {
            const int factorize_status = glp_factorize(lp_);
            switch (factorize_status) {
              case GLP_EBADB: {
                LOG_F(FATAL, "Not able to factorize: error GLP_EBADB.");
                break;
              }
              case GLP_ESING: {
                LOG_F(WARNING,
                      "Not able to factorize: the basis matrix is singular within the working "
                      "precision.");
                return nt::mp::infinity();
              }
              case GLP_ECOND: {
                LOG_F(WARNING, "Not able to factorize: the basis matrix is ill-conditioned.");
                return nt::mp::infinity();
              }
              default: break;
            }
          }
          std::unique_ptr< double[] > right_hand_side(new double[num_rows + 1]);
          double                      norm = 0.0;
          // Iteratively solve B x = e_k, where e_k is the kth unit vector.
          // The result of this computation is the kth column of B^-1.
          // glp_ftran works on original matrix. Scale input and result to
          // obtain the norm of the kth column in the inverse scaled
          // matrix. See glp_ftran documentation in glpapi12.c for how the
          // scaling is done: inv(B'') = inv(SB) * inv(B) * inv(R) where:
          // o B'' is the scaled basis
          // o B is the original basis
          // o R is the diagonal row scaling matrix
          // o SB consists of the basic columns of the augmented column
          // scaling matrix (auxiliary variables then structural variables):
          // S~ = diag(inv(R) | S).
          for (int k = 1; k <= num_rows; ++k) {
            for (int row = 1; row <= num_rows; ++row) {
              right_hand_side[row] = 0.0;
            }
            right_hand_side[k] = 1.0;
            // Multiply input by inv(R).
            for (int row = 1; row <= num_rows; ++row) {
              right_hand_side[row] /= row_scaling_factor[row];
            }
            glp_ftran(lp_, right_hand_side.get());
            // glp_ftran stores the result in the same vector where the right
            // hand side was provided.
            // Multiply result by inv(SB).
            for (int row = 1; row <= num_rows; ++row) {
              const int k = glp_get_bhead(lp_, row);
              if (k <= num_rows) {
                // Auxiliary variable.
                right_hand_side[row] *= row_scaling_factor[k];
              } else {
                // Structural variable.
                right_hand_side[row] /= column_scaling_factor[k - num_rows];
              }
            }
            // Compute sum_row |vector_row|.
            double column_norm = 0.0;
            for (int row = 1; row <= num_rows; ++row) {
              column_norm += fabs(right_hand_side[row]);
            }
            // Compute max_col column_norm
            norm = std::max(norm, column_norm);
          }
          return norm;
        }

        // ------ Parameters ------

        template < bool MIP >
        inline void GLPKInterface< MIP >::ConfigureGLPKParameters(const MPSolverParameters& param) {
          if constexpr (_impl_IsMIP()) {
            glp_init_iocp(&mip_param_);
            // Time limit
            if (Parent::time_limit()) {
              VLOG_F(1, "Setting time limit = {} ms ", Parent::time_limit());
              mip_param_.tm_lim = Parent::time_limit();
            }
            // Initialize structures related to the callback.
            mip_param_.cb_func = GLPKGatherInformationCallback< GLPKInformation, GlpkMPCallbackContext >;
            mip_callback_info_->Reset(Parent::_maximize);
            mip_callback_info_->mp_callback_ = callback_;
            mip_param_.cb_info = mip_callback_info_.get();
            // TODO(user): switch some cuts on? All cuts are off by default!?
          }

          // Configure LP parameters in all cases since they will be used to
          // solve the root LP in the MIP case.
          glp_init_smcp(&lp_param_);
          // Time limit
          if (Parent::time_limit()) {
            VLOG_F(1, "Setting time limit = {} ms", Parent::time_limit());
            lp_param_.tm_lim = Parent::time_limit();
          }

          // Should give a numerically better representation of the problem.
          glp_scale_prob(lp_, GLP_SF_AUTO);

          // Use advanced initial basis (options: standard / advanced / Bixby's).
          glp_adv_basis(lp_, 0);

          // Set parameters specified by the user.
          _impl_SetParameters(param);

          // If an optimal solution is found by the MIP preprocessor, glpk does not call the
          // callback which is problematic if same lazy cuts shall be added. So for now, we simply
          // disable the preprocessor in this case.
          if (callback_ && callback_->might_add_lazy_constraints()) {
            mip_param_.presolve = GLP_OFF;
            LOG_F(WARNING, "The provided callback might add lazy cuts => MIP presolve must be disabled.");
          }
        }

        template < bool MIP >
        inline void GLPKInterface< MIP >::_impl_SetParameters(const MPSolverParameters& param) {
          Parent::setCommonParameters(param);
          if constexpr (_impl_IsMIP()) Parent::setMIPParameters(param);
        }

        template < bool MIP >
        inline void GLPKInterface< MIP >::_impl_SetRelativeMipGap(double value) {
          if constexpr (_impl_IsMIP()) {
            mip_param_.mip_gap = value;
          } else {
            LOG_F(WARNING, "The relative MIP gap is only available for discrete problems.");
          }
        }

        template < bool MIP >
        inline void GLPKInterface< MIP >::_impl_SetPrimalTolerance(double value) {
          lp_param_.tol_bnd = value;
        }

        template < bool MIP >
        inline void GLPKInterface< MIP >::_impl_SetDualTolerance(double value) {
          lp_param_.tol_dj = value;
        }

        template < bool MIP >
        inline void GLPKInterface< MIP >::_impl_SetPresolveMode(int value) {
          switch (value) {
            case MPSolverParameters::PRESOLVE_OFF: {
              mip_param_.presolve = GLP_OFF;
              lp_param_.presolve = GLP_OFF;
              break;
            }
            case MPSolverParameters::PRESOLVE_ON: {
              mip_param_.presolve = GLP_ON;
              lp_param_.presolve = GLP_ON;
              break;
            }
            default: {
              Parent::setIntegerParamToUnsupportedValue(MPSolverParameters::PRESOLVE, value);
            }
          }
        }

        template < bool MIP >
        inline void GLPKInterface< MIP >::_impl_SetScalingMode(int value) {
          Parent::SetUnsupportedIntegerParam(MPSolverParameters::SCALING);
        }

        template < bool MIP >
        inline void GLPKInterface< MIP >::_impl_SetLpAlgorithm(int value) {
          switch (value) {
            case MPSolverParameters::DUAL: {
              // Use dual, and if it fails, switch to primal.
              lp_param_.meth = GLP_DUALP;
              break;
            }
            case MPSolverParameters::PRIMAL: {
              lp_param_.meth = GLP_PRIMAL;
              break;
            }
            case MPSolverParameters::BARRIER:
            default: {
              Parent::setIntegerParamToUnsupportedValue(MPSolverParameters::LP_ALGORITHM, value);
            }
          }
        }

      }   // namespace details

      using LPModel = details::GLPKInterface< false >;
      using MIPModel = details::GLPKInterface< true >;

      inline constexpr bool hasMIP() noexcept { return true; }
      inline constexpr bool hasLP() noexcept { return true; }

    }   // namespace glpk
  }     // namespace mp
}   // namespace nt
#endif
