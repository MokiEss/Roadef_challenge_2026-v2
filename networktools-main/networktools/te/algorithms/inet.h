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
 * This file is based on a modified version of the code from Inet-3.0
 * available at http://topology.eecs.umich.edu/inet/inet-3.0.tar.gz
 *
 * Original Inet-3.0 Copyright Notice:
 * Copyright (c) 1999, 2000, 2002  University of Michigan, Ann Arbor.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms are permitted
 * provided that the above copyright notice and this paragraph are
 * duplicated in all such forms and that any documentation,
 * advertising materials, and other materials related to such
 * distribution and use acknowledge that the software was developed
 * by the University of Michigan, Ann Arbor. The name of the University
 * may not be used to endorse or promote products derived from this
 * software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Author: Jared Winick (jwinick@eecs.umich.edu)
 *         Cheng Jin (chengjin@eecs.umich.edu)
 *         Qian Chen (qianc@eecs.umich.edu)
 *
 * ============================================================================
 */

/**
 * @file
 * @brief
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_INET_H_
#define _NT_INET_H_

#include "../../core/arrays.h"
#include "../../core/random.h"

namespace nt {
  namespace te {

    namespace details_inet {

      typedef struct _node_type {
        int                 nid;         /* node id */
        int                 degree;      /* node degree */
        int                 free_degree; /* unconnected node degree */
        int                 x, y;        /* x, y coordinate */
        struct _node_type** nnp;
      } node_type;

      /**
       * @brief comparison function for qsorting nodes in to descending order of degrees
       *
       */
      inline int degree_compare(const void* a1, const void* a2) {
        node_type* n1 = (node_type*)a1;
        node_type* n2 = (node_type*)a2;

        if (n1->degree < n2->degree)
          return 1;
        else if (n1->degree == n2->degree)
          return 0;
        else
          return -1;
      }

      /**
       * @brief Generate the nodes degrees where the power-law coefficients
       *        are computed using nodes with degree larger than 1
       */
      inline void generate_degrees(node_type* node, int node_num, int degree_one) {
        int   i;
        int   degree, nodes, edges = 0, F;
        float t, C_t;
        int   node_num_no1, degree_two = 0, rank;
        float D_FRAC[19], base;

        float ccdfIdeal;
        float ccdfActual;
        float diff;
        int   numNodesToAdd;

        float ccdfSlope;

        /********************************************************************************/
        /* complementary cumulative distribution function of degree frequency vs. degree*/
        /*(Eq. 1 in the TR)   \bar{F}(d) = e^c * d^(a*t + b)                            */
        /********************************************************************************/

        constexpr double a = .00324;
        constexpr double b = -1.223;
        constexpr double c = -.711;

        /*************************************/
        /* pairsize within h hops growth law */
        /* P(t,h) = exp(s(h)*t)*P(0,h)       */
        /*                                   */
        /* therefore,                        */
        /* P(t,0) = exp(s(0)*t)*P(0,0)       */
        /* t is computed as:                 */
        /*   log(P(t,0)/P(0,0))/s(0)         */
        /*************************************/
        constexpr double P00 = 3037; /* P(0,0) */
        constexpr double s0 = .0281; /* s(0) */

        /***********************************/
        /* degree vs. rank growth powerlaw */
        /* d = exp(p*t + q) * r^R          */
        /***********************************/
        constexpr double R = -.87;
        constexpr double p = .0228;
        constexpr double q = 6.5285;
        constexpr double DR_Frac = .02; /* the fraction of nodes this law applies to */

        /* calculate what month  (how many months since nov. '97) */
        /*  corresponds to this topology of size node_num.       */
        t = std::log((float)node_num / (float)P00) / s0;

        /* using t calculated above, compute the exponent of the ccdf power law */
        ccdfSlope = (a * t) + b;

        /***************************/
        /* generate degree 1 nodes */
        /***************************/
        node_num_no1 = node_num - degree_one;
        nodes = 0;
        for (i = node_num - 1; i >= node_num_no1; --i) {
          node[i].nid = i;
          node[i].degree = 1;
          node[i].free_degree = 1;
        }

        nodes += degree_one;

        // for each degree see if we need to add any of it to obtain the appropriate value on the ccdf
        degree = 2;
        while (nodes <= node_num && degree < node_num) {
          // this is where we should be at
          ccdfIdeal = std::exp(c) * std::pow((float)degree, ccdfSlope);
          // this is were we are before adding nodes of this degree
          ccdfActual = (1.0 - ((float)nodes / (float)node_num));

          // calc how many nodes of this degree we need to add to get the ccdf right
          diff = ccdfActual - ccdfIdeal;
          if (diff * (float)node_num > 1.0)   // we actually need to add a node at this degree
          {
            numNodesToAdd = (int)(diff * (float)node_num);
            for (i = node_num - nodes - 1; i >= node_num - nodes - numNodesToAdd; --i) {
              node[i].nid = i;
              node[i].degree = degree;
              node[i].free_degree = degree;
            }
            nodes += numNodesToAdd;
          }
          ++degree;
        }

        /* use the degree-rank relationship to generate the top 3 degrees */
        for (i = 0; i < 3; ++i) {
          node[i].nid = i;
          rank = i + 1;
          node[i].degree =
             std::pow((float)(rank), R) * std::exp(q) * std::pow(((float)node_num / (float)P00), (p / s0));
          node[i].free_degree = node[i].degree;
        }

        std::qsort(node, node_num, sizeof(node_type), degree_compare);

        /************************************/
        /* the sum of node degrees can't be */
        /* odd -- edges come in pairs       */
        /************************************/
        for (i = 0, edges = 0; i < node_num; ++i)
          edges += node[i].degree;
        if (edges % 2 == 1) {
          node[0].degree++;
          node[0].free_degree++;
        }

        for (i = 0; i < node_num; ++i)
          node[i].nid = i;
      }

      /**
       * @brief Function that tests the connectability of the given degree distribution
       *
       */
      inline int is_graph_connectable(node_type* node, int node_num) {
        int i;
        int F_star, F = 0, degree_one = 0;

        for (i = 0; i < node_num; ++i) {
          if (node[i].degree == 1)
            ++degree_one;
          else
            F += node[i].degree;
        }

        F_star = F - 2 * (node_num - degree_one - 1);
        if (F_star < degree_one) return 0;

        return 1;
      }


      /**
       * @brief The main node connection function
       *
       */
      inline bool connect_nodes(node_type* node, int node_num, nt::Random& rnd) {
        int  i, j, k, degree_g2;
        int *G, *L, G_num, L_num, g, l;
        int *p_array, p_array_num, *id, *flag;

        int    added_node;
        int**  weighted_degree_array;
        double distance;
        int*   freqArray;

        /************************/
        /* satisfaction testing */
        /************************/
        if (!is_graph_connectable(node, node_num))
          LOG_F(WARNING, "Warning: not possible to generate a connected graph with this degree distribution.");

        p_array_num = 0;
        degree_g2 = 0;
        for (i = 0; i < node_num; ++i) {
          node[i].nnp = (node_type**)std::malloc(
             sizeof(node_type*) * node[i].degree);   // TODO(Morgan) : replace malloc() calls with networktools arrays
          if (!node[i].nnp) {
            LOG_F(
               ERROR, "connect_nodes: no memory for node[{}].nnp ({} bytes)", i, sizeof(node_type*) * node[i].degree);
            return false;
          }
          for (j = 0; j < node[i].degree; ++j)
            node[i].nnp[j] = NULL;

          /* the probability array needs be of size = the sum of all degrees of nodes that have degrees >= 2 */
          if (node[i].degree > 1) p_array_num += node[i].degree;


          /* set the position of the first node of degree 1 */
          if (node[i].degree == 1 && node[i - 1].degree != 1) degree_g2 = i;
        }

        G = (int*)std::malloc(sizeof(int) * degree_g2);
        if (!G) {
          LOG_F(ERROR, "connect_nodes: no memory for G ({} bytes)", sizeof(int) * degree_g2);
          return false;
        }

        L = (int*)std::malloc(sizeof(int) * degree_g2);
        if (!L) {
          LOG_F(ERROR, "connect_nodes: no memory for L ({} bytes)", sizeof(int) * degree_g2);
          return false;
        }

        /* we need to allocate more memory than just p_array_num because of our added weights       */
        /* 40 is an arbitrary number, probably much higher than it needs to be, but just being safe */
        p_array = (int*)std::malloc(sizeof(int) * p_array_num * 40);
        if (!p_array) {
          LOG_F(ERROR, "connect_nodes: no memory for p_array ({} bytes)", (sizeof(int) * p_array_num));
          return false;
        }

        id = (int*)std::malloc(sizeof(int) * degree_g2);
        if (!id) {
          LOG_F(ERROR, "connect_nodes: no memory for id ({} bytes)", sizeof(int) * node_num);
          return false;
        }

        flag = (int*)std::malloc(sizeof(int) * degree_g2);
        if (!flag) {
          LOG_F(ERROR, "no memory for flag ({} bytes)", sizeof(int) * degree_g2);
          return false;
        }

        /* weighted_degree_array is a matrix corresponding to the weight that we multiply  */
        /* the standard proportional probability by. so weighted_degree_array[i][j] is the */
        /* value of the weight in P(i,j). the matrix is topDegree x topDegree in size,     */
        /* where topDegree is just the degree of the node withh the highest outdegree.     */
        weighted_degree_array = (int**)std::malloc(sizeof(int*) * (node[0].degree + 1));
        for (i = 0; i <= node[0].degree; ++i) {
          weighted_degree_array[i] = (int*)std::malloc(sizeof(int) * (node[0].degree + 1));
          if (!weighted_degree_array[i]) {
            LOG_F(ERROR, "no memory for weighted_degree index {}.", i);
            return false;
          }
        }

        /* determine how many nodes of a given degree there are.  */
        freqArray = (int*)std::malloc(sizeof(int) * (node[0].degree + 1));

        // fill the freq array
        for (i = 0; i <= node[0].degree; ++i)
          freqArray[i] = 0;

        for (i = 0; i < node_num; ++i)
          freqArray[node[i].degree]++;


        /* using the frequency-degree relationship, calculate weighted_degree_array[i][j] for a all i,j */
        for (i = 1; i <= node[0].degree; ++i) {
          for (j = 1; j <= node[0].degree; ++j) {
            if (freqArray[i] && freqArray[j])   // if these degrees are present in the graph
            {
              distance = pow(
                 (pow(log((double)i / (double)j), 2.0) + pow(log((double)freqArray[i] / (double)freqArray[j]), 2.0)),
                 .5);
              if (distance < 1) distance = 1;

              weighted_degree_array[i][j] = distance / 2 * j;
            }
          }
        }


        /********************************/
        /* randomize the order of nodes */
        /********************************/
        for (i = 0; i < degree_g2; ++i)
          id[i] = i;

        i = degree_g2;
        while (i > 0) {
          j = rnd.integer(i); /* j is the index to the id array! */
          L[degree_g2 - i] = id[j];

          for (; j < i - 1; ++j)
            id[j] = id[j + 1];
          --i;
        }

        /* do not randomize the order of nodes as was done in Inet-2.2. */
        /* we just want to build the spanning tree by adding nodes in   */
        /* in monotonically decreasing order of node degree             */
        for (i = 0; i < degree_g2; ++i)
          L[i] = i;

        /************************************************/
        /* Phase 1: building a connected spanning graph */
        /************************************************/
        G_num = 1;
        G[0] = L[0];
        L_num = degree_g2 - 1;

        while (L_num > 0) {
          j = L[1];
          added_node = j;

          /******************************/
          /* fill the probability array */
          /******************************/
          l = 0;
          for (i = 0; i < G_num; ++i) {
            if (node[G[i]].free_degree) {
              if (node[G[i]].free_degree > node[G[i]].degree) {
                LOG_F(ERROR,
                      "connect_nodes: problem, node {} (nid={}), free_degree = {}, degree = {}, exiting...",
                      G[i],
                      node[G[i]].nid,
                      node[G[i]].free_degree,
                      node[G[i]].degree);
                return false;
              }

              for (j = 0; j < weighted_degree_array[node[added_node].degree][node[G[i]].degree]; ++j, ++l)
                p_array[l] = G[i];
            }
          }

          /**************************************************************/
          /* select a node randomly according to its degree proportions */
          /**************************************************************/
          g = rnd.integer(l); /* g is the index to the p_array */

          /*****************************************************/
          /* redirect -- i and j are indices to the node array */
          /*****************************************************/
          i = p_array[g];
          j = added_node;

          /*if (node[i].nid == 0)
            fprintf(stderr, "phase I: added node %d\n", node[j].nid);*/

          node[i].nnp[node[i].degree - node[i].free_degree] = node + j;
          node[j].nnp[node[j].degree - node[j].free_degree] = node + i;

          /* add l to G and remove from L */
          G[G_num] = j;
          for (l = 1; l < L_num; ++l)
            L[l] = L[l + 1];

          --(node[i].free_degree);
          --(node[j].free_degree);
          ++G_num;
          --L_num;
        }

        // fprintf(stderr, "DONE!!\n");
        /*************************************************************************/
        /* Phase II: connect degree 1 nodes proportionally to the spanning graph */
        /*************************************************************************/
        for (i = degree_g2; i < node_num; ++i) {
          /******************************/
          /* fill the probability array */
          /******************************/
          l = 0;
          for (j = 0; j < degree_g2; ++j) {
            if (node[j].free_degree) {
              for (k = 0; k < weighted_degree_array[1][node[j].degree]; ++k, ++l)
                p_array[l] = j;
            }
          }

          g = rnd.integer(l); /* g is the index to the p_array */
          j = p_array[g];

          node[i].nnp[node[i].degree - node[i].free_degree] = node + j;
          node[j].nnp[node[j].degree - node[j].free_degree] = node + i;

          --(node[i].free_degree);
          --(node[j].free_degree);
        }

        // fprintf(stderr, "DONE!!\n");
        /**********************************************************/
        /* Phase III: garbage collection to fill all free degrees */
        /**********************************************************/
        for (i = 0; i < degree_g2; ++i) {
          for (j = i + 1; j < degree_g2; ++j)
            flag[j] = 1;
          flag[i] = 0; /* no connection to itself */

          /********************************************************************/
          /* well, we also must eliminate all nodes that i is already         */
          /* connected to. bug reported by shi.zhou@elec.qmul.ac.uk on 8/6/01 */
          /********************************************************************/
          for (j = 0; j < (node[i].degree - node[i].free_degree); ++j)
            if (node[i].nnp[j] - node < degree_g2) flag[node[i].nnp[j] - node] = 0;

          if (!node[i].nnp[0]) {
            LOG_F(ERROR,
                  "i = {}, degree = {}, free_degree = {}, node[i].npp[0] null",
                  i,
                  node[i].degree,
                  node[i].free_degree);
            return false;
          }

          flag[node[i].nnp[0] - node] = 0; /* no connection to its first neighbor */

          if (node[i].free_degree < 0) {
            LOG_F(ERROR, "we have a problem, node {} free_degree {}", i, node[i].free_degree);
            return false;
          }

          while (node[i].free_degree) {
            /******************************/
            /* fill the probability array */
            /******************************/
            l = 0;
            for (j = i + 1; j < degree_g2; ++j) {
              if (node[j].free_degree && flag[j]) {
                for (k = 0; k < weighted_degree_array[node[i].degree][node[j].degree]; ++k, ++l)
                  p_array[l] = j;
              }
            }

            if (l == 0) break;

            g = rnd.integer(l); /* g is the index to the p_array */
            j = p_array[g];

            /*if ( node[i].nid == 0 )
              fprintf(stderr, "phase III: added node %d!\n", node[j].nid);*/

            node[i].nnp[node[i].degree - node[i].free_degree] = node + j;
            node[j].nnp[node[j].degree - node[j].free_degree] = node + i;

            --(node[i].free_degree);
            --(node[j].free_degree);

            flag[j] = 0;
          }

          if (node[i].free_degree) {
            LOG_F(ERROR,
                  "connect_nodes: node {} has degree of {} with {} unfilled",
                  node[i].nid,
                  node[i].degree,
                  node[i].free_degree);
            return false;
          }
        }

        free(freqArray);
        free(flag);
        free(G);
        free(L);
        free(p_array);
        free(id);

        for (i = 0; i <= node[0].degree; ++i)
          free(weighted_degree_array[i]);
        free(weighted_degree_array);

        return true;
      }
    }   // namespace details_inet

    /**
     * @brief Generate a graph that has an internet-like topology based on inet algorithm [1].
     *        This code is based on a modified version of the original source code
     *        available at http://topology.eecs.umich.edu/inet/inet-3.0.tar.gz
     *
     * Example of usage :
     *
     * @code
     *  int main() {
     *    nt::graphs::SmartDigraph g;
     *
     *    // Generate an internet-like network topology with 3037 nodes
     *    nt::te::generateInetGraph(g, 3037);
     *
     *   return 0;
     *}
     * @endcode
     *
     * [1] Jared Winick and Sugih Jamin:
     *     Inet-3.0: Internet Topology Generator.
     *     Technical Report, University of Michigan (2002)
     *
     * @tparam GR Type of the generated graph
     *
     * @param gr Generated graph
     * @param n Number of vertices to generate. Must be greater or equal to 3037.
     * @param f_fraction Percent of degree-one vertices
     * @param grid_size Size of the grid on which the vertices are placed
     */
    template < typename GR >
    void generateInetGraph(GR& gr, int n, double f_fraction = 0.3, int i_grid_size = 10000, int i_seed = 0) {
      TEMPLATE_DIGRAPH_TYPEDEFS(GR);

      // the following  constants are computed by poly-fit data of degree 1 node
      // growth from Nov 1997 to 2000 with respect to the total number of nodes
      constexpr double DEGREE_1_SLOPE = 0.292;     // poly-fit under matlab
      constexpr int    DEGREE_1_INTERCEPT = 462;   // poly-fit under matlab

      if (n < 3037) n = 3037;

      // the default is computed by a linear relationship
      int i_degree_one = (int)((float)n * f_fraction);
      if (i_degree_one == 0) i_degree_one = DEGREE_1_SLOPE * n + DEGREE_1_INTERCEPT;

      // initializes the random number generator
      nt::Random rnd(i_seed);

      nt::TrivialDynamicArray< details_inet::node_type > vertices(n);
      details_inet::generate_degrees(vertices.data(), vertices.size(), i_degree_one);

      // randomly place nodes inside a grid
      for (int i = 0; i < vertices.size(); ++i) {
        vertices[i].x = rnd.integer(i_grid_size);
        vertices[i].y = rnd.integer(i_grid_size);
      }

      details_inet::connect_nodes(vertices.data(), vertices.size(), rnd);

      int i, j, dist;
      int link_num = 0;

      for (i = 0; i < vertices.size(); ++i) {
        link_num += vertices[i].degree - vertices[i].free_degree;
        vertices[i].degree -= vertices[i].free_degree;
        gr.addNode();
      }

      if ((link_num % 2) != 0) return;

      for (i = 0; i < vertices.size(); ++i) {
        for (j = 0; j < vertices[i].degree; ++j) {
          if (vertices[i].nnp[j] && vertices[i].nnp[j]->nid > vertices[i].nid) {
            const Node u = gr.nodeFromId(vertices[i].nid);
            const Node v = gr.nodeFromId(vertices[i].nnp[j]->nid);
            if constexpr (nt::concepts::UndirectedTagIndicator< GR >) {
              gr.addEdge(u, v);
            } else {
              gr.addArc(u, v);
              gr.addArc(v, u);
            }
          }
        }
      }

      for (i = 0; i < vertices._i_count; ++i)
        free(vertices[i].nnp);

      assert(gr.nodeNum() == vertices.size());
    }

  }   // namespace te
}   // namespace nt

#endif