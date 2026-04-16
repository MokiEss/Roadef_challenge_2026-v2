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
 * This file incorporates work from the LEMON graph library (vf2pp.h).
 *
 * Original LEMON Copyright Notice:
 * Copyright (C) 2015-2017
 * EMAXA Kutato-fejleszto Kft. (EMAXA Research Ltd.)
 *
 * Permission to use, modify and distribute this software is granted
 * provided that this copyright notice appears in all copies. For
 * precise terms see the accompanying LICENSE file.
 *
 * This software is provided "AS IS" with no warranty of any kind,
 * express or implied, and with no claim as to its suitability for any
 * purpose.
 *
 * ============================================================================
 *
 * List of modifications:
 *   - fix access to index -1 in _conn[] in feas()
 *   - Replace std::vector with nt::TrivialDynamicArray
 *   - Replace std::swap with nt::swap
 *   - Replace std::pair with nt::Pair
 *   - Remove std::make_pair
 *   - Add noexcept
 *   - Use constexpr and "constexpr if" wherever possible
 *   - Replace "typedef" with "using"
 */

/**
 * @ingroup graph_isomorphism
 * @file
 * @brief %VF2 Plus Plus algorithm class \cite VF2PP
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_VF2PP_H_
#define _NT_VF2PP_H_

#include "../bits/vf2_internals.h"
#include "../../core/common.h"
#include "../../core/arrays.h"
#include "../../core/tuple.h"

namespace nt {
  namespace graphs {

    /** %VF2 Plus Plus algorithm class \cite VF2PP. */

    /**
     * @ingroup graph_isomorphism
     *
     * @brief This class provides an efficient
     * implementation of the %VF2 Plus Plus algorithm \cite VF2PP
     * for variants of the (Sub)graph Isomorphism problem.
     *
     * There is also a @ref vf2pp() "function-type interface" called
     * @ref vf2pp() for the %VF2 Plus Plus algorithm, which is probably
     * more convenient in most use cases.
     *
     * @tparam G1 The type of the graph to be embedded.
     * @tparam G2 The type of the graph g1 will be embedded into.
     * @tparam M The type of the NodeMap storing the mapping.
     * By default, it is G1::NodeMap<G2::Node>
     * @tparam M1 The type of the NodeMap storing the integer node labels of G1.
     * The labels must be the numbers {0,1,2,..,K-1}, where K is the number of
     * different labels. By default, it is G1::NodeMap<int>.
     * @tparam M2 The type of the NodeMap storing the integer node labels of G2.
     * The labels must be the numbers {0,1,2,..,K-1}, where K is the number of
     * different labels. By default, it is G2::NodeMap<int>.
     *
     * \sa vf2pp()
     */
    template < typename G1,
               typename G2,
               typename M = typename G1::template NodeMap< typename G2::Node >,
               typename M1 = typename G1::template NodeMap< int >,
               typename M2 = typename G2::template NodeMap< int > >
    // requires nt::concepts::Graph< G1 > && nt::concepts::Graph< G2 >
    class Vf2pp {
      // The graph to be embedded
      const G1& _g1;

      // The graph into which g1 is to be embedded
      const G2& _g2;

      // Current depth in the search tree
      int _depth;

      // The current mapping. _mapping[v1]=v2 iff v1 has been mapped to v2,
      // where v1 is a node of G1 and v2 is a node of G2
      M& _mapping;

      //_order[i] is a node of g1 for which a pair is searched if depth=i
      nt::TrivialDynamicArray< typename G1::Node > _order;

      //_conn[v2] = number of covered neighbours of v2
      typename G2::template NodeMap< int > _conn;

      //_currEdgeIts[i] is the last used edge iterator in the i-th
      // depth to find a pair for node _order[i]
      nt::TrivialDynamicArray< typename G2::IncEdgeIt > _currEdgeIts;

      //_rNewLabels1[v] is a pair of form
      //(label; num. of uncov. nodes with such label and no covered neighbours)
      typename G1::template NodeMap< nt::TrivialDynamicArray< nt::Pair< int, int > > > _rNewLabels1;

      //_rInOutLabels1[v] is the number of covered neighbours of v for each label
      // in form (label,number of such labels)
      typename G1::template NodeMap< nt::TrivialDynamicArray< nt::Pair< int, int > > > _rInOutLabels1;

      //_intLabels1[v]==i means that node v has label i in _g1
      //(i is in {0,1,2,..,K-1}, where K is the number of diff. labels)
      M1& _intLabels1;

      //_intLabels2[v]==i means that node v has label i in _g2
      //(i is in {0,1,2,..,K-1}, where K is the number of diff. labels)
      M2& _intLabels2;

      // largest label
      const int _maxLabel;

      // lookup tables for manipulating with label class cardinalities
      //(after use they have to be reset to 0..0)
      nt::TrivialDynamicArray< int > _labelTmp1, _labelTmp2;

      MappingType _mapping_type;


      // improved cutting function
      template < MappingType MT >
      constexpr bool cutByLabels(const typename G1::Node n1, const typename G2::Node n2) noexcept {
        for (typename G2::IncEdgeIt e2(_g2, n2); e2 != INVALID; ++e2) {
          const typename G2::Node currNode = _g2.oppositeNode(n2, e2);
          if (_conn[currNode] > 0)
            --_labelTmp1[_intLabels2[currNode]];
          else if (MT != MappingType::SUBGRAPH && _conn[currNode] == 0)
            --_labelTmp2[_intLabels2[currNode]];
        }


        for (unsigned int i = 0; i < _rInOutLabels1[n1].size(); ++i)
          _labelTmp1[_rInOutLabels1[n1][i].first] += _rInOutLabels1[n1][i].second;

        if constexpr (MT != MappingType::SUBGRAPH) {
          for (unsigned int i = 0; i < _rNewLabels1[n1].size(); ++i)
            _labelTmp2[_rNewLabels1[n1][i].first] += _rNewLabels1[n1][i].second;
        }

        bool ret = 1;

        if constexpr (MT == MappingType::INDUCED) {
          for (unsigned int i = 0; i < _rInOutLabels1[n1].size(); ++i) {
            if (_labelTmp1[_rInOutLabels1[n1][i].first] > 0) {
              ret = 0;
              break;
            }
          }
          if (ret) {
            for (unsigned int i = 0; i < _rNewLabels1[n1].size(); ++i) {
              if (_labelTmp2[_rNewLabels1[n1][i].first] > 0) {
                ret = 0;
                break;
              }
            }
          }
        } else if constexpr (MT == MappingType::SUBGRAPH) {
          for (unsigned int i = 0; i < _rInOutLabels1[n1].size(); ++i) {
            if (_labelTmp1[_rInOutLabels1[n1][i].first] > 0) {
              ret = 0;
              break;
            }
          }
        } else if constexpr (MT == MappingType::ISOMORPH) {
          for (unsigned int i = 0; i < _rInOutLabels1[n1].size(); ++i) {
            if (_labelTmp1[_rInOutLabels1[n1][i].first] != 0) {
              ret = 0;
              break;
            }
          }
          if (ret) {
            for (unsigned int i = 0; i < _rNewLabels1[n1].size(); ++i) {
              if (_labelTmp2[_rNewLabels1[n1][i].first] != 0) {
                ret = 0;
                break;
              }
            }
          }
        } else {
          return false;
        }

        for (unsigned int i = 0; i < _rInOutLabels1[n1].size(); ++i)
          _labelTmp1[_rInOutLabels1[n1][i].first] = 0;

        if constexpr (MT != MappingType::SUBGRAPH) {
          for (unsigned int i = 0; i < _rNewLabels1[n1].size(); ++i)
            _labelTmp2[_rNewLabels1[n1][i].first] = 0;
        }

        for (typename G2::IncEdgeIt e2(_g2, n2); e2 != INVALID; ++e2) {
          const typename G2::Node currNode = _g2.oppositeNode(n2, e2);
          _labelTmp1[_intLabels2[currNode]] = 0;
          // if (MT != MappingType::SUBGRAPH && _conn[currNode] == 0) _labelTmp2[_intLabels2[currNode]] = 0;
          if constexpr (MT != MappingType::SUBGRAPH) {
            if (_conn[currNode] == 0) _labelTmp2[_intLabels2[currNode]] = 0;
          }
        }

        return ret;
      }

      // try to exclude the matching of n1 and n2
      template < MappingType MT >
      constexpr bool feas(const typename G1::Node n1, const typename G2::Node n2) noexcept {
        if (_intLabels1[n1] != _intLabels2[n2]) return 0;

        for (typename G1::IncEdgeIt e1(_g1, n1); e1 != INVALID; ++e1) {
          const typename G1::Node& currNode = _g1.oppositeNode(n1, e1);
          if (_mapping[currNode] != INVALID) --_conn[_mapping[currNode]];
        }

        bool isIso = 1;
        for (typename G2::IncEdgeIt e2(_g2, n2); e2 != INVALID; ++e2) {
          int& connCurrNode = _conn[_g2.oppositeNode(n2, e2)];
          if (connCurrNode < -1)
            ++connCurrNode;
          else {
            if constexpr (MT != MappingType::SUBGRAPH) {
              if (connCurrNode == -1) {
                isIso = 0;
                break;
              }
            }
          }
        }

        if (isIso) {
          for (typename G1::IncEdgeIt e1(_g1, n1); e1 != INVALID; ++e1) {
            const typename G2::Node& currNodePair = _mapping[_g1.oppositeNode(n1, e1)];
            if (currNodePair == INVALID) continue;
            int& connCurrNodePair = _conn[currNodePair];
            if (connCurrNodePair != -1) {
              if constexpr (MT == MappingType::INDUCED || MT == MappingType::ISOMORPH) {
                isIso = 0;
              } else if constexpr (MT == MappingType::SUBGRAPH) {
                if (connCurrNodePair < -1) isIso = 0;
              }
              connCurrNodePair = -1;
            }
          }
        } else {
          for (typename G1::IncEdgeIt e1(_g1, n1); e1 != INVALID; ++e1) {
            const typename G2::Node currNode = _mapping[_g1.oppositeNode(n1, e1)];
            if (currNode != INVALID /*&&_conn[currNode]!=-1*/) _conn[currNode] = -1;
          }
        }

        return isIso && cutByLabels< MT >(n1, n2);
      }

      // maps n1 to n2
      constexpr void addPair(const typename G1::Node n1, const typename G2::Node n2) noexcept {
        _conn[n2] = -1;
        _mapping.set(n1, n2);
        for (typename G2::IncEdgeIt e2(_g2, n2); e2 != INVALID; ++e2) {
          int& currConn = _conn[_g2.oppositeNode(n2, e2)];
          if (currConn != -1) ++currConn;
        }
      }

      // removes mapping of n1 to n2
      constexpr void subPair(const typename G1::Node n1, const typename G2::Node n2) noexcept {
        _conn[n2] = 0;
        _mapping.set(n1, INVALID);
        for (typename G2::IncEdgeIt e2(_g2, n2); e2 != INVALID; ++e2) {
          int& currConn = _conn[_g2.oppositeNode(n2, e2)];
          if (currConn > 0)
            --currConn;
          else if (currConn == -1)
            ++_conn[n2];
        }
      }

      constexpr void processBfsTree(typename G1::Node                      source,
                                    unsigned int&                          orderIndex,
                                    typename G1::template NodeMap< int >&  dm1,
                                    typename G1::template NodeMap< bool >& added) noexcept {
        _order[orderIndex] = source;
        added[source] = 1;

        unsigned int endPosOfLevel = orderIndex, startPosOfLevel = orderIndex, lastAdded = orderIndex;

        typename G1::template NodeMap< int > currConn(_g1, 0);

        while (orderIndex <= lastAdded) {
          typename G1::Node currNode = _order[orderIndex];
          for (typename G1::IncEdgeIt e(_g1, currNode); e != INVALID; ++e) {
            typename G1::Node n = _g1.oppositeNode(currNode, e);
            if (!added[n]) {
              _order[++lastAdded] = n;
              added[n] = 1;
            }
          }
          if (orderIndex > endPosOfLevel) {
            for (unsigned int j = startPosOfLevel; j <= endPosOfLevel; ++j) {
              int minInd = j;
              for (unsigned int i = j + 1; i <= endPosOfLevel; ++i)
                if (currConn[_order[i]] > currConn[_order[minInd]]
                    || (currConn[_order[i]] == currConn[_order[minInd]]
                        && (dm1[_order[i]] > dm1[_order[minInd]]
                            || (dm1[_order[i]] == dm1[_order[minInd]]
                                && _labelTmp1[_intLabels1[_order[minInd]]] > _labelTmp1[_intLabels1[_order[i]]]))))
                  minInd = i;

              --_labelTmp1[_intLabels1[_order[minInd]]];
              for (typename G1::IncEdgeIt e(_g1, _order[minInd]); e != INVALID; ++e)
                ++currConn[_g1.oppositeNode(_order[minInd], e)];
              nt::swap(_order[j], _order[minInd]);
            }
            startPosOfLevel = endPosOfLevel + 1;
            endPosOfLevel = lastAdded;
          }
          ++orderIndex;
        }
      }

      // we will find pairs for the nodes of g1 in this order
      constexpr void initOrder() noexcept {
        for (typename G2::NodeIt n2(_g2); n2 != INVALID; ++n2)
          ++_labelTmp1[_intLabels2[n2]];

        typename G1::template NodeMap< int > dm1(_g1, 0);
        for (typename G1::EdgeIt e(_g1); e != INVALID; ++e) {
          ++dm1[_g1.u(e)];
          ++dm1[_g1.v(e)];
        }

        typename G1::template NodeMap< bool > added(_g1, 0);
        unsigned int                          orderIndex = 0;

        for (typename G1::NodeIt n(_g1); n != INVALID;) {
          if (!added[n]) {
            typename G1::Node minNode = n;
            for (typename G1::NodeIt n1(_g1, minNode); n1 != INVALID; ++n1)
              if (!added[n1]
                  && (_labelTmp1[_intLabels1[minNode]] > _labelTmp1[_intLabels1[n1]]
                      || (dm1[minNode] < dm1[n1] && _labelTmp1[_intLabels1[minNode]] == _labelTmp1[_intLabels1[n1]])))
                minNode = n1;
            processBfsTree(minNode, orderIndex, dm1, added);
          } else
            ++n;
        }
        _labelTmp1.setBitsToZero();
      }

      template < MappingType MT >
      constexpr bool extMatch() noexcept {
        while (_depth >= 0) {
          if (_depth == static_cast< int >(_order.size())) {
            // all nodes of g1 are mapped to nodes of g2
            --_depth;
            return true;
          }
          typename G1::Node&       nodeOfDepth = _order[_depth];
          const typename G2::Node& pairOfNodeOfDepth = _mapping[nodeOfDepth];
          typename G2::IncEdgeIt&  edgeItOfDepth = _currEdgeIts[_depth];
          // the node of g2 whose neighbours are the candidates for
          // the pair of _order[_depth]
          typename G2::Node currPNode;
          if (edgeItOfDepth == INVALID) {
            typename G1::IncEdgeIt fstMatchedE(_g1, nodeOfDepth);
            // if _mapping[_order[_depth]]!=INVALID, we don't need fstMatchedE
            if (pairOfNodeOfDepth == INVALID) {
              for (; fstMatchedE != INVALID && _mapping[_g1.oppositeNode(nodeOfDepth, fstMatchedE)] == INVALID;
                   ++fstMatchedE)
                ;   // find fstMatchedE, it could be preprocessed
            }
            if (fstMatchedE == INVALID || pairOfNodeOfDepth != INVALID) {
              // We found no covered neighbours, this means that
              // the graph is not connected (or _depth==0). Each
              // uncovered (and there are some other properties due
              // to the spec. problem types) node of g2 is
              // candidate. We can read the iterator of the last
              // tried node from the match if it is not the first
              // try (match[nodeOfDepth]!=INVALID)
              typename G2::NodeIt n2(_g2);
              // if it's not the first try
              if (pairOfNodeOfDepth != INVALID) {
                n2 = ++typename G2::NodeIt(_g2, pairOfNodeOfDepth);
                subPair(nodeOfDepth, pairOfNodeOfDepth);
              }
              for (; n2 != INVALID; ++n2)
                if (MT != MappingType::SUBGRAPH) {
                  if (_conn[n2] == 0 && feas< MT >(nodeOfDepth, n2)) break;
                } else if (_conn[n2] >= 0 && feas< MT >(nodeOfDepth, n2))
                  break;
              // n2 is the next candidate
              if (n2 != INVALID) {
                addPair(nodeOfDepth, n2);
                ++_depth;
              } else   // there are no more candidates
                --_depth;
              continue;
            } else {
              currPNode = _mapping[_g1.oppositeNode(nodeOfDepth, fstMatchedE)];
              edgeItOfDepth = typename G2::IncEdgeIt(_g2, currPNode);
            }
          } else {
            currPNode = _g2.oppositeNode(pairOfNodeOfDepth, edgeItOfDepth);
            subPair(nodeOfDepth, pairOfNodeOfDepth);
            ++edgeItOfDepth;
          }
          for (; edgeItOfDepth != INVALID; ++edgeItOfDepth) {
            const typename G2::Node currNode = _g2.oppositeNode(currPNode, edgeItOfDepth);
            if (_conn[currNode] > 0 && feas< MT >(nodeOfDepth, currNode)) {
              addPair(nodeOfDepth, currNode);
              break;
            }
          }
          edgeItOfDepth == INVALID ? --_depth : ++_depth;
        }
        return false;
      }

      // calculate the lookup table for cutting the search tree
      constexpr void initRNew1tRInOut1t() noexcept {
        typename G1::template NodeMap< int > tmp(_g1, 0);
        for (unsigned int i = 0; i < _order.size(); ++i) {
          tmp[_order[i]] = -1;
          for (typename G1::IncEdgeIt e1(_g1, _order[i]); e1 != INVALID; ++e1) {
            const typename G1::Node currNode = _g1.oppositeNode(_order[i], e1);
            if (tmp[currNode] > 0)
              ++_labelTmp1[_intLabels1[currNode]];
            else if (tmp[currNode] == 0)
              ++_labelTmp2[_intLabels1[currNode]];
          }
          //_labelTmp1[i]=number of neightbours with label i in set rInOut
          //_labelTmp2[i]=number of neightbours with label i in set rNew
          for (typename G1::IncEdgeIt e1(_g1, _order[i]); e1 != INVALID; ++e1) {
            const int& currIntLabel = _intLabels1[_g1.oppositeNode(_order[i], e1)];
            if (_labelTmp1[currIntLabel] > 0) {
              _rInOutLabels1[_order[i]].push_back({currIntLabel, _labelTmp1[currIntLabel]});
              _labelTmp1[currIntLabel] = 0;
            } else if (_labelTmp2[currIntLabel] > 0) {
              _rNewLabels1[_order[i]].push_back({currIntLabel, _labelTmp2[currIntLabel]});
              _labelTmp2[currIntLabel] = 0;
            }
          }

          for (typename G1::IncEdgeIt e1(_g1, _order[i]); e1 != INVALID; ++e1) {
            int& tmpCurrNode = tmp[_g1.oppositeNode(_order[i], e1)];
            if (tmpCurrNode != -1) ++tmpCurrNode;
          }
        }
      }

      constexpr int getMaxLabel() const noexcept {
        int m = -1;
        for (typename G1::NodeIt n1(_g1); n1 != INVALID; ++n1) {
          const int& currIntLabel = _intLabels1[n1];
          if (currIntLabel > m) m = currIntLabel;
        }
        for (typename G2::NodeIt n2(_g2); n2 != INVALID; ++n2) {
          const int& currIntLabel = _intLabels2[n2];
          if (currIntLabel > m) m = currIntLabel;
        }
        return m;
      }

      public:
      /** Constructor */

      /**
       * Constructor.
       * @param g1 The graph to be embedded.
       * @param g2 The graph \e g1 will be embedded into.
       * @param m The type of the NodeMap storing the mapping.
       * By default, it is G1::NodeMap<G2::Node>
       * @param intLabel1 The NodeMap storing the integer node labels of G1.
       * The labels must be the numbers {0,1,2,..,K-1}, where K is the number of
       * different labels.
       * @param intLabel1 The NodeMap storing the integer node labels of G2.
       * The labels must be the numbers {0,1,2,..,K-1}, where K is the number of
       * different labels.
       */
      Vf2pp(const G1& g1, const G2& g2, M& m, M1& intLabels1, M2& intLabels2) :
          _g1(g1), _g2(g2), _depth(0), _mapping(m), _order(nt::graphs::countNodes(g1)), _conn(g2, 0),
          _currEdgeIts(nt::graphs::countNodes(g1)), _rNewLabels1(_g1), _rInOutLabels1(_g1), _intLabels1(intLabels1),
          _intLabels2(intLabels2), _maxLabel(getMaxLabel()), _labelTmp1(_maxLabel + 1), _labelTmp2(_maxLabel + 1),
          _mapping_type(MappingType::SUBGRAPH), _deallocMappingAfterUse(0), _deallocLabelsAfterUse(0) {
        _order.setBitsToOne();         // Set all values to ~0 = -1 = INVALID
        _currEdgeIts.setBitsToOne();   // Set all values to ~0 = -1 = INVALID

        _labelTmp1.setBitsToZero();
        _labelTmp2.setBitsToZero();

        initOrder();
        initRNew1tRInOut1t();

        // reset mapping
        for (typename G1::NodeIt n(g1); n != INVALID; ++n)
          m[n] = INVALID;
      }

      /** Destructor */

      /**
       * Destructor.
       *
       */
      ~Vf2pp() noexcept {
        if (_deallocMappingAfterUse) delete &_mapping;
        if (_deallocLabelsAfterUse) {
          delete &_intLabels1;
          delete &_intLabels2;
        }
      }

      // indicates whether the mapping or the labels must be deleted in the
      // destructor
      bool _deallocMappingAfterUse, _deallocLabelsAfterUse;

      /** Returns the current mapping type. */

      /**
       * Returns the current mapping type.
       *
       */
      constexpr MappingType mappingType() const noexcept { return _mapping_type; }

      /** Sets the mapping type */

      /**
       * Sets the mapping type.
       *
       * The mapping type is set to @ref SUBGRAPH by default.
       *
       * \sa See @ref MappingType for the possible values.
       */
      constexpr void mappingType(MappingType m_type) noexcept { _mapping_type = m_type; }

      /** Finds a mapping. */

      /**
       * This method finds a mapping from g1 into g2 according to the mapping
       * type set by @ref mappingType(MappingType) "mappingType()".
       *
       * By subsequent calls, it returns all possible mappings one-by-one.
       *
       * \retval true if a mapping is found.
       * \retval false if there is no (more) mapping.
       */
      constexpr bool find() noexcept {
        switch (_mapping_type) {
          case MappingType::SUBGRAPH: return extMatch< MappingType::SUBGRAPH >();
          case MappingType::INDUCED: return extMatch< MappingType::INDUCED >();
          case MappingType::ISOMORPH: return extMatch< MappingType::ISOMORPH >();
          default: return false;
        }
      }
    };

    template < typename G1, typename G2 >
    class Vf2ppWizardBase {
      protected:
      using Graph1 = G1;
      using Graph2 = G2;

      const G1& _g1;
      const G2& _g2;

      MappingType _mapping_type;

      using Mapping = typename G1::template NodeMap< typename G2::Node >;
      bool           _local_mapping;
      void*          _mapping;
      constexpr void createMapping() noexcept { _mapping = new Mapping(_g1); }

      bool _local_nodeLabels;
      using NodeLabels1 = typename G1::template NodeMap< int >;
      using NodeLabels2 = typename G2::template NodeMap< int >;
      void *         _nodeLabels1, *_nodeLabels2;
      constexpr void createNodeLabels() noexcept {
        _nodeLabels1 = new NodeLabels1(_g1, 0);
        _nodeLabels2 = new NodeLabels2(_g2, 0);
      }

      Vf2ppWizardBase(const G1& g1, const G2& g2) :
          _g1(g1), _g2(g2), _mapping_type(MappingType::SUBGRAPH), _local_mapping(1), _local_nodeLabels(1) {}
    };

    /**
     * @brief Auxiliary class for the function-type interface of %VF2
     * Plus Plus algorithm.
     *
     * This auxiliary class implements the named parameters of
     * @ref vf2pp() "function-type interface" of @ref Vf2pp algorithm.
     *
     * @warning This class is not to be used directly.
     *
     * @tparam TR The traits class that defines various types used by the
     * algorithm.
     */
    template < typename TR >
    class Vf2ppWizard: public TR {
      using Base = TR;
      using Graph1 = typename TR::Graph1;
      using Graph2 = typename TR::Graph2;
      using Mapping = typename TR::Mapping;
      using NodeLabels1 = typename TR::NodeLabels1;
      using NodeLabels2 = typename TR::NodeLabels2;

      using TR::_g1;
      using TR::_g2;
      using TR::_mapping;
      using TR::_mapping_type;
      using TR::_nodeLabels1;
      using TR::_nodeLabels2;

      public:
      /** Constructor */
      Vf2ppWizard(const Graph1& g1, const Graph2& g2) : Base(g1, g2) {}

      /** Copy constructor */
      Vf2ppWizard(const Base& b) : Base(b) {}

      template < typename T >
      struct SetMappingBase: public Base {
        using Mapping = T;
        SetMappingBase(const Base& b) : Base(b) {}
      };

      /**
       * @brief @ref named-templ-param "Named parameter" for setting
       * the mapping.
       *
       * @ref named-templ-param "Named parameter" function for setting
       * the map that stores the found embedding.
       */
      template < typename T >
      constexpr Vf2ppWizard< SetMappingBase< T > > mapping(const T& t) noexcept {
        Base::_mapping = reinterpret_cast< void* >(const_cast< T* >(&t));
        Base::_local_mapping = 0;
        return Vf2ppWizard< SetMappingBase< T > >(*this);
      }

      template < typename NL1, typename NL2 >
      struct SetNodeLabelsBase: public Base {
        using NodeLabels1 = NL1;
        using NodeLabels2 = NL2;
        SetNodeLabelsBase(const Base& b) : Base(b) {}
      };

      /**
       * @brief @ref named-templ-param "Named parameter" for setting the
       * node labels.
       *
       * @ref named-templ-param "Named parameter" function for setting
       * the node labels.
       *
       * @param nodeLabels1 A @ref concepts::ReadMap "readable node map"
       * of g1 with integer values. In case of K different labels, the labels
       * must be the numbers {0,1,..,K-1}.
       * @param nodeLabels2 A @ref concepts::ReadMap "readable node map"
       * of g2 with integer values. In case of K different labels, the labels
       * must be the numbers {0,1,..,K-1}.
       */
      template < typename NL1, typename NL2 >
      constexpr Vf2ppWizard< SetNodeLabelsBase< NL1, NL2 > > nodeLabels(const NL1& nodeLabels1,
                                                                        const NL2& nodeLabels2) noexcept {
        Base::_local_nodeLabels = 0;
        Base::_nodeLabels1 = reinterpret_cast< void* >(const_cast< NL1* >(&nodeLabels1));
        Base::_nodeLabels2 = reinterpret_cast< void* >(const_cast< NL2* >(&nodeLabels2));
        return Vf2ppWizard< SetNodeLabelsBase< NL1, NL2 > >(SetNodeLabelsBase< NL1, NL2 >(*this));
      }

      /**
       * @brief @ref named-templ-param "Named parameter" for setting
       * the mapping type.
       *
       * @ref named-templ-param "Named parameter" for setting
       * the mapping type.
       *
       * The mapping type is set to @ref SUBGRAPH by default.
       *
       * \sa See @ref MappingType for the possible values.
       */
      constexpr Vf2ppWizard< Base >& mappingType(MappingType m_type) noexcept {
        _mapping_type = m_type;
        return *this;
      }

      /**
       * @brief @ref named-templ-param "Named parameter" for setting
       * the mapping type to @ref INDUCED.
       *
       * @ref named-templ-param "Named parameter" for setting
       * the mapping type to @ref INDUCED.
       */
      constexpr Vf2ppWizard< Base >& induced() noexcept {
        _mapping_type = MappingType::INDUCED;
        return *this;
      }

      /**
       * @brief @ref named-templ-param "Named parameter" for setting
       * the mapping type to @ref ISOMORPH.
       *
       * @ref named-templ-param "Named parameter" for setting
       * the mapping type to @ref ISOMORPH.
       */
      constexpr Vf2ppWizard< Base >& iso() noexcept {
        _mapping_type = MappingType::ISOMORPH;
        return *this;
      }

      /** Runs the %VF2 Plus Plus algorithm. */

      /**
       * This method runs the VF2 Plus Plus algorithm.
       *
       * \retval true if a mapping is found.
       * \retval false if there is no mapping.
       */
      constexpr bool run() noexcept {
        if (Base::_local_mapping) Base::createMapping();
        if (Base::_local_nodeLabels) Base::createNodeLabels();

        Vf2pp< Graph1, Graph2, Mapping, NodeLabels1, NodeLabels2 > alg(_g1,
                                                                       _g2,
                                                                       *reinterpret_cast< Mapping* >(_mapping),
                                                                       *reinterpret_cast< NodeLabels1* >(_nodeLabels1),
                                                                       *reinterpret_cast< NodeLabels2* >(_nodeLabels2));

        alg.mappingType(_mapping_type);

        const bool ret = alg.find();

        if (Base::_local_nodeLabels) {
          delete reinterpret_cast< NodeLabels1* >(_nodeLabels1);
          delete reinterpret_cast< NodeLabels2* >(_nodeLabels2);
        }
        if (Base::_local_mapping) delete reinterpret_cast< Mapping* >(_mapping);

        return ret;
      }

      /** Get a pointer to the generated Vf2pp object. */

      /**
       * Gives a pointer to the generated Vf2pp object.
       *
       * @return Pointer to the generated Vf2pp object.
       * @warning Don't forget to delete the referred Vf2pp object after use.
       */
      constexpr Vf2pp< Graph1, Graph2, Mapping, NodeLabels1, NodeLabels2 >* getPtrToVf2ppObject() noexcept {
        if (Base::_local_mapping) Base::createMapping();
        if (Base::_local_nodeLabels) Base::createNodeLabels();

        Vf2pp< Graph1, Graph2, Mapping, NodeLabels1, NodeLabels2 >* ptr =
           new Vf2pp< Graph1, Graph2, Mapping, NodeLabels1, NodeLabels2 >(
              _g1,
              _g2,
              *reinterpret_cast< Mapping* >(_mapping),
              *reinterpret_cast< NodeLabels1* >(_nodeLabels1),
              *reinterpret_cast< NodeLabels2* >(_nodeLabels2));
        ptr->mappingType(_mapping_type);
        if (Base::_local_mapping) ptr->_deallocMappingAfterUse = true;
        if (Base::_local_nodeLabels) ptr->_deallocLabelsAfterUse = true;

        return ptr;
      }

      /** Counts the number of mappings. */

      /**
       * This method counts the number of mappings.
       *
       * @return The number of mappings.
       */
      constexpr int count() noexcept {
        if (Base::_local_mapping) Base::createMapping();
        if (Base::_local_nodeLabels) Base::createNodeLabels();

        Vf2pp< Graph1, Graph2, Mapping, NodeLabels1, NodeLabels2 > alg(_g1,
                                                                       _g2,
                                                                       *reinterpret_cast< Mapping* >(_mapping),
                                                                       *reinterpret_cast< NodeLabels1* >(_nodeLabels1),
                                                                       *reinterpret_cast< NodeLabels2* >(_nodeLabels2));

        alg.mappingType(_mapping_type);

        int ret = 0;
        while (alg.find())
          ++ret;

        if (Base::_local_nodeLabels) {
          delete reinterpret_cast< NodeLabels1* >(_nodeLabels1);
          delete reinterpret_cast< NodeLabels2* >(_nodeLabels2);
        }
        if (Base::_local_mapping) delete reinterpret_cast< Mapping* >(_mapping);

        return ret;
      }
    };

    /** Function-type interface for VF2 Plus Plus algorithm. */

    /**
     * @ingroup graph_isomorphism
     * Function-type interface for VF2 Plus Plus algorithm.
     *
     * This function has several @ref named-func-param "named parameters"
     * declared as the members of class @ref Vf2ppWizard.
     * The following examples show how to use these parameters.
     * @code
     * ListGraph::NodeMap<ListGraph::Node> m(g);
     * // Find an embedding of graph g1 into graph g2
     * vf2pp(g1,g2).mapping(m).run();
     *
     * // Check whether graphs g1 and g2 are isomorphic
     * bool is_iso = vf2pp(g1,g2).iso().run();
     *
     * // Count the number of isomorphisms
     * int num_isos = vf2pp(g1,g2).iso().count();
     *
     * // Iterate through all the induced subgraph mappings
     * // of graph g1 into g2 using the labels c1 and c2
     * auto* myVf2pp = vf2pp(g1,g2).mapping(m).nodeLabels(c1,c2)
     * .induced().getPtrToVf2Object();
     * while(myVf2pp->find()){
     * //process the current mapping m
     * }
     * delete myVf22pp;
     * @endcode
     * @warning Don't forget to put the @ref Vf2ppWizard::run() "run()",
     * @ref Vf2ppWizard::count() "count()" or
     * the @ref Vf2ppWizard::getPtrToVf2ppObject() "getPtrToVf2ppObject()"
     * to the end of the expression.
     * \sa Vf2ppWizard
     * \sa Vf2pp
     */
    template < class G1, class G2 >
    constexpr Vf2ppWizard< Vf2ppWizardBase< G1, G2 > > vf2pp(const G1& g1, const G2& g2) noexcept {
      return Vf2ppWizard< Vf2ppWizardBase< G1, G2 > >(g1, g2);
    }

  }   // namespace graphs
}   // namespace nt
#endif
