/*****************************************************************************
 * This file is part of uvgVPCCenc V-PCC encoder.
 *
 * Copyright (c) 2024-present, Tampere University, ITU/ISO/IEC, project contributors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * * Neither the name of the Tampere University or ITU/ISO/IEC nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * INCLUDING NEGLIGENCE OR OTHERWISE ARISING IN ANY WAY OUT OF THE USE OF THIS
 ****************************************************************************/

/// \file Common tools used by the uvgVPCCenc library.

#pragma once

#include <array>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>

#include "uvgutils/utils.hpp"

namespace uvgvpcc_enc {

using typeGeometryInput = uint16_t;

const typeGeometryInput g_infiniteDepth = (std::numeric_limits<typeGeometryInput>::max)();  // TODO(lf)be sure it is well sync with type geo
const size_t g_infinitenumber = (std::numeric_limits<size_t>::max)();
const size_t g_valueNotSet = (std::numeric_limits<size_t>::max)();

constexpr size_t INVALID_PATCH_INDEX = std::numeric_limits<size_t>::max();
constexpr size_t PPI_NON_ASSIGNED = std::numeric_limits<size_t>::max();
constexpr size_t UNDEFINED_PARENT_PPI = std::numeric_limits<size_t>::max() - 1;  // TODO(lf) temp

// Projection Plan Index, 0-5 -> one of the six bounding box plan. 6+ -> used for slicing ppi attribution
enum class PPI : uint8_t { ppi0, ppi1, ppi2, ppi3, ppi4, ppi5, ppiBlank, notAssigned };

}  // namespace uvgvpcc_enc
