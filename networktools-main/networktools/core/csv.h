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
 * @brief This file defines wrapper classes/functions to the RapidCSV library. The goal is to avoid networktools to be
 *        dependent of the RapidCSV's API. Ideally, any call to a CSV class/function should be done through the present
 *        wrappers.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_CSV_H_
#define _NT_CSV_H_

#include "../@deps/rapidcsv/rapidcsv.h"

namespace nt {
  using CSVDocument = rapidcsv::Document;
  using CSVLabelParams = rapidcsv::LabelParams;
  using CSVSeparatorParams = rapidcsv::SeparatorParams;
}   // namespace nt

#endif
