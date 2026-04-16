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
 * @file ntconfig.h
 * @brief Configuration options for the Networktools library.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_CONFIG_H_
#define _NT_CONFIG_H_

//-----------------------------------------------------------------------------
// COMPILE-TIME OPTIONS FOR NETWORKTOOLS

// Library version
#define NETWORKTOOLS_VERSION "1.0.1"

//---- Enable the use of SIMD instructions
// #define NT_USE_SIMD

//---- Enable the use of STL hashmap
// #define NT_USE_STL_MAP

#endif
