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

#ifndef _NT_CG4SR_H_
#define _NT_CG4SR_H_

#include "../../core/json.h"
#include "../../te/algorithms/srte.h"
#include "../../graphs/algorithms/mcf.h"

namespace nt {
  namespace te {

    /**
     * @class Cg4Sr
     * @headerfile cg4sr.h
     * @brief Implementation of the SR paths optimization algorithm from [1]
     *
     * Example of usage :
     *
     * @code
     *  int main() {
     *
     *    using Digraph     = nt::graphs::SmartDigraph;
     *    using DemandGraph = nt::graphs::DemandGraph< Digraph >;
     *    using LPModel     = nt::mp::cplex::LPModel;
     *    using MIPModel    = nt::mp::cplex::MIPModel;
     *    using Cg4Sr       = nt::te::Cg4Sr< Digraph, MIPModel, LPModel >;
     *
     *    Digraph                      digraph;
     *    DemandGraph                  demand_graph(digraph);
     *    Digraph::ArcMap< float >     capacities(digraph);
     *    DemandGraph::ArcMap< float > demand_values(demand_graph);
     *
     *    LPModel                      lps("lps");
     *    MIPModel                     mips("mips");
     *
     *    read_instance(digraph, capacities, demand_graph, demand_values);
     *
     *    Cg4Sr cg4sr(digraph, demand_graph, capacities, demand_values, mips, lps);
     *
     *    Cg4Sr::Parameters params;
     *    params.setIntegerParam(Cg4Sr::Parameters::MAX_SEGMENTS, 1);
     *    cg4sr.setParameters(std::move(params));
     *
     *    cg4sr.run();
     *
     *    LOG_F(INFO, "{}", cg4sr.optimalValue());
     *
     *    return 0;
     *  }
     * @endcode
     *
     *
     * [1] Mathieu Jadin, Francois Aubry, Pierre Schaus, Olivier Bonaventure:
     *     Cg4Sr: Near Optimal Traffic Engineering for Segment Routing with Column Generation.
     *     INFOCOM 2019: 1333-1341
     *
     * @tparam GR The type of the digraph the algorithm runs on.
     * @tparam MIPS The type of the MIP solver used to solve the SRTE-UTIL-ILP model.
     * @tparam LPS The type of the LP solver used to solve the SRTE-LP model.
     * @tparam FPTYPE floating-point precision could be float, double, long double, ...
     * @tparam VEC The type of the vector used to store demands, flows, saturation values.
     * @tparam DG The type of the digraph representing the demand graph. It must be defined on the
     * same node set as GR.
     * @tparam CM A concepts::ReadMap "readable" arc map storing the capacities of the arcs.
     * @tparam DVM A concepts::ReadMap "readable" arc map storing the demand values.
     *
     */
    template < typename GR,
               typename MIPS,
               typename LPS,
               typename FPTYPE = float,
               typename VEC = nt::Vec< FPTYPE, 1 >,
               typename DG = graphs::DemandGraph< GR >,
               typename CM = typename GR::template ArcMap< FPTYPE >,
               typename MM = typename GR::template ArcMap< FPTYPE >,
               typename DVM = typename DG::template ArcMap< VEC > >
    struct Cg4Sr {
      static_assert(std::is_arithmetic_v< FPTYPE >, "Cg4Sr:Non-arithmetic weight type.");

      using Digraph = GR;
      using LPModel = LPS;
      using MIPModel = MIPS;

      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

      using DemandGraph = DG;
      using DemandOutArcIt = typename DemandGraph::OutArcIt;
      using DemandInArcIt = typename DemandGraph::InArcIt;
      using DemandArcIt = typename DemandGraph::ArcIt;
      using DemandArc = typename DemandGraph::Arc;
      using DemandArcArray = nt::TrivialDynamicArray< DemandArc >;

      using CapacityMap = CM;
      using Capacity = typename CM::Value;
      using MetricMap = MM;
      using Metric = typename MetricMap::Value;
      using DemandValueMap = DVM;
      using DemandValue = typename DVM::Value;

      using FpType = FPTYPE;

      /**
       * This class stores parameter settings for the algorithms.
       */
      struct Parameters {
        // Enumeration of parameters that take continuous values.
        enum DoubleParam {
          LAMBDA_LB = 1000,   // < The lower bound value for λ (default : 0.)
          LAMBDA_UB = 1001    // < The upper bound value for λ (default : 1.)
        };

        // Enumeration of parameters that take integer or categorical values.
        enum IntegerParam {
          MAX_ITER = 1000,   // < The maximum number of iteration in the column generation procedure (default : inf)
          MAX_GEN_COLUMNS =
             1001,   // < The maximum number of collumns to be added by the pricer at each iteration (default : inf)
          MAX_SEGMENTS = 1002,   // < The maximum number of segments per demand (default : 4)
          USE_MCF_LB =
             1003   // < If true, the lower bound for λ will be computed according the MCF value (default : true)
        };

        Parameters() :
            _i_max_iter(NT_INT32_MAX), _i_max_gen_columns(NT_INT32_MAX), _max_segments(4), _b_use_mcf_lb(true),
            _f_lower_bound(0.), _f_upper_bound(1.) {}

        /**
         * @brief Sets a double parameter to a specific value.
         */
        constexpr void setDoubleParam(DoubleParam param, double value) noexcept {
          switch (param) {
            case LAMBDA_LB: {
              _f_lower_bound = value;
              break;
            }
            case LAMBDA_UB: {
              _f_upper_bound = value;
              break;
            }
            default: {
              LOG_F(ERROR, "Trying to set an unknown parameter: {}.", static_cast< int >(param));
            }
          }
        }

        /**
         * @brief Returns the value of a double parameter.
         */
        constexpr double getDoubleParam(DoubleParam param) const noexcept {
          switch (param) {
            case LAMBDA_LB: {
              return _f_lower_bound;
            }
            case LAMBDA_UB: {
              return _f_upper_bound;
            }
            default: {
              LOG_F(ERROR, "Trying to get an unknown parameter: {}.", static_cast< int >(param));
              return NT_DOUBLE_INFINITY;
            }
          }
        }

        /**
         * @brief Sets a int parameter to a specific value.
         */
        constexpr void setIntegerParam(IntegerParam param, int value) noexcept {
          switch (param) {
            case MAX_ITER: {
              _i_max_iter = value;
              break;
            }
            case MAX_GEN_COLUMNS: {
              _i_max_gen_columns = value;
              break;
            }
            case MAX_SEGMENTS: {
              _max_segments = value;
              break;
            }
            case USE_MCF_LB: {
              _b_use_mcf_lb = static_cast< bool >(value);
              break;
            }
            default: {
              LOG_F(ERROR, "Trying to set an unknown parameter: {}.", static_cast< int >(param));
            }
          }
        }

        /**
         * @brief Returns the value of a int parameter.
         */
        constexpr int getIntegerParam(IntegerParam param) const noexcept {
          switch (param) {
            case MAX_ITER: {
              return _i_max_iter;
              break;
            }
            case MAX_GEN_COLUMNS: {
              return _i_max_gen_columns;
              break;
            }
            case MAX_SEGMENTS: {
              return _max_segments;
              break;
            }
            case USE_MCF_LB: {
              return _b_use_mcf_lb;
              break;
            }
            default: {
              LOG_F(ERROR, "Trying to get an unknown parameter: {}.", static_cast< int >(param));
              return NT_INT32_MAX;
            }
          }
        }

        int  _i_max_iter;
        int  _i_max_gen_columns;
        int  _max_segments;
        bool _b_use_mcf_lb;

        double _f_lower_bound;
        double _f_upper_bound;
      };

      const Digraph&     _graph;
      const DemandGraph& _demand_graph;

      Parameters _params;

      SrteLp< Digraph, LPModel, FpType, VEC, DG, CM, MM, DVM >       _srte;        //< SRTE-LP model (see [1])
      SrteUtilIlp< Digraph, MIPModel, FpType, VEC, DG, CM, MM, DVM > _srte_path;   //< SRTE-UTIL-ILP model (see [1])

      const CapacityMap&                               _cm;
      const DemandValueMap&                            _dvm;
      graphs::McfPath< Digraph, LPModel, FpType, VEC > _mcf;


      /**
       * @brief Construct a new Cg4Sr object
       *
       * @param Network
       * @param TrafficMatrix
       * @param lp_solver
       * @param mip_solver
       */
      explicit Cg4Sr(const Digraph&        g,
                     const DemandGraph&    dg,
                     const CapacityMap&    cm,
                     const MetricMap&      mm,
                     const DemandValueMap& dvm,
                     MIPModel&             mips,
                     LPModel&              lps) :
          _graph(g),
          _demand_graph(dg), _srte(_graph, _demand_graph, cm, mm, dvm, lps),
          _srte_path(_graph, _demand_graph, cm, mm, dvm, mips), _cm(cm), _dvm(dvm), _mcf(g, dg, lps) {}


      /**
       * @brief Set the Parameters
       *
       */
      constexpr void setParameters(const Parameters& params) noexcept { _params = params; }
      constexpr void setParameters(Parameters&& params) noexcept { _params = std::move(params); }

      /**
       * @brief Set the map that stores the arcs metrics.
       *
       */
      constexpr void metricMap(const MetricMap& mm) noexcept { _srte.metricMap(mm); }

      /**
       * @brief Get the map that stores the arcs metrics.
       *
       */
      constexpr MetricMap& metricMap() noexcept { return _srte.metricMap(); }

      /**
       * @brief Run Cg4Sr algorithm
       *
       * @return true if an optimal solution is found, false otherwise
       */
      bool run() {
        LOG_F(INFO, "Run Cg4Sr");

        FpType f_lower_bound = _params.getDoubleParam(Parameters::LAMBDA_LB);
        FpType f_upper_bound = _params.getDoubleParam(Parameters::LAMBDA_UB);

        if (_params.getIntegerParam(Parameters::USE_MCF_LB)) {
          // Compute MCF to get a lower bound on the lambda value
          LOG_F(INFO, " Compute lambda lb via MCF ...");

          if (_mcf.build(&_cm, &_dvm)) {
            _mcf.solve();
            f_lower_bound = _mcf.getLoad();
          }
        }

        if (f_upper_bound <= f_lower_bound) f_upper_bound = f_lower_bound * 1.2;

        LOG_F(INFO, " Lambda bounds : lb = {}, up = {}", f_lower_bound, f_upper_bound);

        //
        double f_sum_demands = 0.;
        for (DemandArcIt demand_arc(_demand_graph); demand_arc != nt::INVALID; ++demand_arc)
          f_sum_demands += _dvm[demand_arc].max();
        LOG_F(INFO, " Sum demands : {}", f_sum_demands);

        // Run the binary search to get the optimal lambda
        LOG_F(INFO, " Run binary search...");
        _srte.build();

        _srte._max_segments = _params.getIntegerParam(Parameters::MAX_SEGMENTS);
        _srte._i_max_gen_columns = _params.getIntegerParam(Parameters::MAX_GEN_COLUMNS);
        _srte._i_max_iter = _params.getIntegerParam(Parameters::MAX_ITER);

        double f_lambda = f_upper_bound;
        while (std::abs(f_upper_bound - f_lower_bound) > 0.001) {
          f_lambda = (f_lower_bound + f_upper_bound) * .5;

          _srte.updateLambda(f_lambda);
          const mp::ResultStatus result = _srte.solve();
          if (result != mp::ResultStatus::OPTIMAL) {
            LOG_F(ERROR, " SRTE-LP could not be solved.");
            return false;
          }
          const double f_current_value = _srte.optimalValue();

          if (f_current_value < f_sum_demands - 0.00001)
            f_lower_bound = f_lambda;
          else
            f_upper_bound = f_lambda;
        }

        LOG_F(INFO, "Lambda optimal = {} in [{}, {}]", f_lambda, f_lower_bound, f_upper_bound);

        // Solves the SRTE-UTIL-ILP model with the previously generated columns (sr-paths)
        LOG_F(INFO, "Building SRTE-UTIL-ILP...", f_lambda, f_lower_bound, f_upper_bound);
        for (const auto& c: _srte.generatedColumns())
          _srte_path.addColumn(c.demand_arc, c.sr_path, c.ratios);

        if (!_srte_path.build()) {
          LOG_F(ERROR, " SRTE-UTIL-ILP could not be built.");
          return false;
        }

        LOG_F(INFO, "Number of variables = {}", _srte_path._mips.numVariables());
        LOG_F(INFO, "Number of constraints = {}", _srte_path._mips.numConstraints());

        LOG_F(INFO, "Solving SRTE-UTIL-ILP...");

        if (_srte_path.solve() != mp::ResultStatus::OPTIMAL) {
          LOG_F(ERROR, " SRTE-UTIL-ILP could not be solved.");
          return false;
        }

        LOG_F(INFO, " done.");
        return true;
      }

      /**
       * @brief Returns the optimal SRPath of the given demand
       *
       * @param demand_arc The demand for which we ask the SRPath
       *
       * @return A pointer to the optimal SR path found for the demand, nullptr if no solution found
       */
      const SrPath< Digraph >* optimalSRPath(DemandArc demand_arc) const noexcept {
        return _srte_path.optimalSRPath(demand_arc);
      }

      /**
       * @brief Returns the optimal maximum link utilization found
       *
       */
      double optimalValue() const noexcept { return _srte_path.optimalValue(); }


      /**
       * @brief Export the solution in JSON
       *
       */
      nt::JSONDocument exportSolutionToJSON() const {
        nt::JSONDocument json;
        json.SetObject();

        nt::JSONDocument::AllocatorType& allocator = json.GetAllocator();
        json.AddMember("solution_value", nt::JSONValue(optimalValue()), allocator);

        nt::JSONValue sr_paths_array_json(nt::JSONValueType::ARRAY_TYPE);
        for (DemandArcIt demand_arc(_demand_graph); demand_arc != nt::INVALID; ++demand_arc) {
          nt::JSONValue segments_array_json(nt::JSONValueType::ARRAY_TYPE);
          if (const SrPath< Digraph >* pSRPath = optimalSRPath(demand_arc)) {
            for (int i_segment = 0; i_segment < pSRPath->segmentNum(); ++i_segment)
              segments_array_json.PushBack(nt::JSONValue((*pSRPath)[i_segment]._id), allocator);
          }

          if (segments_array_json.Size() > 2) {
            nt::JSONValue node_object(nt::JSONValueType::OBJECT_TYPE);
            node_object.AddMember("demand_id", _demand_graph.id(demand_arc), allocator);
            node_object.AddMember("segments", std::move(segments_array_json), allocator);
            sr_paths_array_json.PushBack(std::move(node_object), allocator);
          }
        }

        json.AddMember("SRPaths", std::move(sr_paths_array_json), allocator);
        return json;
      }
    };

  }   // namespace te
}   // namespace nt
#endif
