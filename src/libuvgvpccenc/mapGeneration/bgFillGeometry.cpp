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

/// \file Functions related to the background filling of the geometry maps

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>
#include <stdexcept>

#include "bgFillGeometry.hpp"
#include "utils/parameters.hpp"

using namespace uvgvpcc_enc;

namespace {

//NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions)
template<size_t BlockSize>
void bgFillGeometryPatchExtension(const std::vector<uint8_t>& occupancyMapDS,
                                                 const size_t gofMapsHeight,
                                                 std::vector<uint8_t>& geometryMap,
                                                 const size_t mapWidth,
                                                 const uint8_t backgroundValue) {

    // Algorithm from TMC2 (dilate), slight modifications in the implementation //
    // lf: We use the attribute background filling algorithm for the geometry map. We don't use the extensive geometry filling algorithm of TMC2 that relies on 3D neighboring searches within the input point-cloud.

    const size_t occupancyMapSizeU = mapWidth / BlockSize;
    const size_t occupancyMapSizeV = gofMapsHeight / BlockSize;
    constexpr size_t pixelBlockCount = BlockSize * BlockSize;
    const std::array<std::array<int64_t, 2>, 4> neighbors = {{{0, -1}, {-1, 0}, {1, 0}, {0, 1}}};

    std::array<uint32_t, pixelBlockCount> iterations{};
    std::array<size_t, pixelBlockCount> count{};
    std::array<int32_t, pixelBlockCount> values{};

    for (size_t yOM = 0; yOM < occupancyMapSizeV; ++yOM) {
        const int64_t yBlockOffset = yOM * BlockSize;
        for (size_t xOM = 0; xOM < occupancyMapSizeU; ++xOM) {
            const int64_t xBlockOffset = xOM * BlockSize;
            const size_t blockIndex = xOM + yOM * occupancyMapSizeU;

            if (occupancyMapDS[blockIndex] == 0) {
                // Fill from left or top neighbor
                if (xOM > 0) {
                    const size_t leftX = xBlockOffset - 1;
                    for (size_t v = 0; v < BlockSize; ++v) {
                        const size_t y = yBlockOffset + v;
                        const uint8_t leftVal = geometryMap[leftX + y * mapWidth];
                        uint8_t* dst = &geometryMap[xBlockOffset + y * mapWidth];
                        std::fill(dst, dst + BlockSize, leftVal);
                    }
                } else if (yOM > 0) {
                    const size_t topY = yBlockOffset - 1;
                    for (size_t u = 0; u < BlockSize; ++u) {
                        const size_t x = xBlockOffset + u;
                        const uint8_t topVal = geometryMap[x + topY * mapWidth];
                        for (size_t v = 0; v < BlockSize; ++v)
                            geometryMap[x + (yBlockOffset + v) * mapWidth] = topVal;
                    }
                }
                continue;
            }

            iterations.fill(0);
            size_t emptyPixelCount = 0;

            for (size_t v = 0; v < BlockSize; ++v) {
                for (size_t u = 0; u < BlockSize; ++u) {
                    const size_t x = xBlockOffset + u;
                    const size_t y = yBlockOffset + v;
                    const size_t idx = x + y * mapWidth;
                    const size_t localIdx = u + v * BlockSize;

                    if (geometryMap[idx] == backgroundValue) {
                        ++emptyPixelCount;
                    } else {
                        iterations[localIdx] = 1;
                    }
                }
            }

            if (emptyPixelCount == 0)
                continue;

            count.fill(0);
            values.fill(0);

            size_t iteration = 1;
            while (emptyPixelCount > 0 && iteration < pixelBlockCount) {
                for (size_t v = 0; v < BlockSize; ++v) {
                    for (size_t u = 0; u < BlockSize; ++u) {
                        const size_t localIdx = u + v * BlockSize;
                        if (iterations[localIdx] != iteration)
                            continue;

                        const int x = xBlockOffset + u;
                        const int y = yBlockOffset + v;
                        const uint8_t srcVal = geometryMap[x + y * mapWidth];

                        for (const auto& n : neighbors) {
                            const int uN = u + n[0];
                            const int vN = v + n[1];

                            if (uN < 0 || uN >= BlockSize || vN < 0 || vN >= BlockSize)
                                continue;

                            const size_t neighborIdx = uN + vN * BlockSize;
                            if (iterations[neighborIdx] != 0)
                                continue;

                            values[neighborIdx] += srcVal;
                            ++count[neighborIdx];
                        }
                    }
                }

                for (size_t v = 0; v < BlockSize; ++v) {
                    for (size_t u = 0; u < BlockSize; ++u) {
                        const size_t localIdx = u + v * BlockSize;
                        if (count[localIdx] == 0)
                            continue;

                        const size_t x = xBlockOffset + u;
                        const size_t y = yBlockOffset + v;
                        const size_t idx = x + y * mapWidth;

                        const size_t c = count[localIdx];
                        const size_t avg = (values[localIdx] + c / 2) / c;

                        geometryMap[idx] = static_cast<uint8_t>(avg);
                        iterations[localIdx] = iteration + 1;
                        --emptyPixelCount;

                        count[localIdx] = 0;
                        values[localIdx] = 0;
                    }
                }

                ++iteration;
            }
        }
    }
}
//NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions)

} // anonymous namespace

void bgFillGeometry(const std::vector<uint8_t>& occupancyMapDS,
    const size_t gofMapsHeight,
    std::vector<uint8_t>& geometryMap) {
    const size_t blockSize = p_->occupancyMapDSResolution;
    const size_t mapWidth = p_->mapWidth;
    const uint8_t backgroundValue = p_->mapGenerationBackgroundValueGeometry;

    if (blockSize == 2) {
        bgFillGeometryPatchExtension<2>(occupancyMapDS, gofMapsHeight, geometryMap, mapWidth, backgroundValue);
    } else if (blockSize == 4) {
        bgFillGeometryPatchExtension<4>(occupancyMapDS, gofMapsHeight, geometryMap, mapWidth, backgroundValue);
    } else {
        throw std::invalid_argument("Unsupported blockSize");
    }
}