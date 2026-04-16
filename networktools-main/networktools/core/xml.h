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
 * @brief This file defines wrapper classes/functions to the RapidXML library. The goal is to avoid
 *        networktools to be dependent of the RapidXML's API. Ideally, any call to a XML class/function
 *        should be done through the present wrappers.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_XML_H_
#define _NT_XML_H_

#define RAPIDXML_NO_EXCEPTIONS

#include "../@deps/rapidxml/rapidxml.hpp"
// #include "../@deps/rapidxml/rapidxml_utils.hpp"
// #include "../@deps/rapidxml/rapidxml_print.hpp"
// #include "../@deps/rapidxml/rapidxml_iterators.hpp"

#if defined(_MSC_VER) && (_MSC_VER >= 1400) && (!defined WINCE)
#  define RAPIDXML_SSCANF sscanf_s
#else
#  define RAPIDXML_SSCANF sscanf
#endif

namespace rapidxml {
  /**
   * @brief When exceptions are disabled by defining RAPIDXML_NO_EXCEPTIONS, this function is called
   * to notify user about the error.
   *
   * @param what Human readable description of the error.
   * @param where Pointer to character data where error was detected.
   */
  inline void parse_error_handler(const char* what, void* where) {
    LOG_F(ERROR, what);
    std::abort();
  }
}   // namespace rapidxml

namespace nt {

  enum class XMLNodeType {
    NODE_DOCUMENT = rapidxml::node_document,
    NODE_ELEMENT = rapidxml::node_element,
    NODE_DATA = rapidxml::node_data,
    NODE_CDATA = rapidxml::node_cdata,
    NODE_COMMENT = rapidxml::node_comment,
    NODE_DECLARATION = rapidxml::node_declaration,
    NODE_DOCTYPE = rapidxml::node_doctype,
    NODE_PI = rapidxml::node_pi
  };

  using XMLElement = rapidxml::xml_node<>;
  using XMLAttributte = rapidxml::xml_attribute<>;
  using XMLNode = rapidxml::xml_node<>;

  template < class Ch = char >
  struct xml_document: rapidxml::xml_document< Ch > {
    using Parent = rapidxml::xml_document<>;

    template < int Flags >
    bool parse(Ch* text) {
      Parent::parse< Flags >(text);
      return true;
    }

    rapidxml::xml_node< Ch >* allocate_node(XMLNodeType type,
                                            const Ch*   name = 0,
                                            const Ch*   value = 0,
                                            std::size_t name_size = 0,
                                            std::size_t value_size = 0) {
      return Parent::allocate_node(
         static_cast< std::underlying_type_t< XMLNodeType > >(type), name, value, name_size, value_size);
    }
  };

  using XMLDocument = xml_document<>;


  struct XMLUtil {
    static bool IsUTF8Continuation(const char p) { return (p & 0x80) != 0; }
    static bool IsWhiteSpace(char p) { return !IsUTF8Continuation(p) && isspace(static_cast< unsigned char >(p)); }
    static const char* SkipWhiteSpace(const char* p, int* curLineNumPtr) {
      assert(p);
      while (IsWhiteSpace(*p)) {
        if (curLineNumPtr && *p == '\n') { ++(*curLineNumPtr); }
        ++p;
      }
      assert(p);
      return p;
    }
    static bool isPrefixHex(const char* p) {
      p = SkipWhiteSpace(p, 0);
      return p && *p == '0' && (*(p + 1) == 'x' || *(p + 1) == 'X');
    }

    static bool toInt(const char* str, int* value) {
      if (RAPIDXML_SSCANF(str, isPrefixHex(str) ? "%x" : "%d", value) == 1) { return true; }
      return false;
    }
    static bool toFloat(const char* str, float* value) {
      if (RAPIDXML_SSCANF(str, "%f", value) == 1) { return true; }
      return false;
    }
    static bool toDouble(const char* str, double* value) {
      if (RAPIDXML_SSCANF(str, "%lf", value) == 1) { return true; }
      return false;
    }
  };

}   // namespace nt

#endif