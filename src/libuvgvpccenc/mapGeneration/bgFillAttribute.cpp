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

/// \file Functions related to the background filling of the attribute maps
#include "bgFillAttribute.hpp"

#include <algorithm>
#include <array>
#include <cassert>  //TODO(lf): everywhere, choose between assert or static_assert (can we add a rule in clang-format to check if the wrong one is used?)
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

#include "utils/parameters.hpp"
#include "uvgvpcc/uvgvpcc.hpp"

using namespace uvgvpcc_enc;

namespace {

// NOLINTBEGIN(cppcoreguidelines-init-variables,cppcoreguidelines-init-variables,clang-analyzer-deadcode.DeadStores,hicpp-signed-bitwise,performance-unnecessary-copy-initialization,google-readability-casting,misc-const-correctness)
int mean4w(uint8_t p1, unsigned char w1, uint8_t p2, unsigned char w2, uint8_t p3, unsigned char w3, uint8_t p4, unsigned char w4) {
    int result = (p1 * int(w1) + p2 * int(w2) + p3 * int(w3) + p4 * int(w4)) / (int(w1) + int(w2) + int(w3) + int(w4));
    return result;
}

// Generates a weighted mipmap
void pushPullMip(const std::vector<uint8_t>& image, const size_t& width, const size_t& height, const size_t& newWidth,
                 const size_t& newHeight, std::vector<uint8_t>& mip, const std::vector<uint8_t>& occupancyMapDS,
                 std::vector<uint8_t>& mipOccupancyMap) {
    unsigned char w1;
    unsigned char w2;
    unsigned char w3;
    unsigned char w4;
    unsigned char val1;
    unsigned char val2;
    unsigned char val3;
    unsigned char val4;

    // allocate the mipmap with half the resolution
    mip.resize(newWidth * newHeight * 3);
    mipOccupancyMap.resize(newWidth * newHeight, 0);
    for (size_t y = 0; y < newHeight; ++y) {
        const size_t yUp = y << 1;
        for (size_t x = 0; x < newWidth; ++x) {
            const size_t xUp = x << 1;
            if (occupancyMapDS[xUp + width * yUp] == 0) {
                w1 = 0;
            } else {
                w1 = 255;
            }
            if ((xUp + 1 >= width) || (occupancyMapDS[xUp + 1 + width * yUp] == 0)) {
                w2 = 0;
            } else {
                w2 = 255;
            }
            if ((yUp + 1 >= height) || (occupancyMapDS[xUp + width * (yUp + 1)] == 0)) {
                w3 = 0;
            } else {
                w3 = 255;
            }
            if ((xUp + 1 >= width) || (yUp + 1 >= height) || (occupancyMapDS[xUp + 1 + width * (yUp + 1)] == 0)) {
                w4 = 0;
            } else {
                w4 = 255;
            }
            if (w1 + w2 + w3 + w4 > 0) {
                for (int cc = 0; cc < 3; cc++) {
                    val1 = image[xUp + yUp * width + cc * width * height];
                    if (xUp + 1 >= width) {
                        val2 = 0;
                    } else {
                        val2 = image[(xUp + 1) + yUp * width + cc * width * height];
                    }
                    if (yUp + 1 >= height) {
                        val3 = 0;
                    } else {
                        val3 = image[xUp + (yUp + 1) * width + cc * width * height];
                    }
                    if ((xUp + 1 >= width) || (yUp + 1 >= height)) {
                        val4 = 0;
                    } else {
                        val4 = image[(xUp + 1) + (yUp + 1) * width + cc * width * height];
                    }
                    uint8_t newVal = mean4w(val1, w1, val2, w2, val3, w3, val4, w4);
                    mip[x + y * newWidth + cc * newWidth * newHeight] = newVal;
                }
                mipOccupancyMap[x + newWidth * y] = 1;
            }
        }
    }
}

// interpolate using mipmap
void pushPullFill(std::vector<uint8_t>& image, const size_t& width, const size_t& height, const size_t& widthUp, const size_t& heightUp,
                  const std::vector<uint8_t>& mip, const std::vector<uint8_t>& occupancyMapDS, int numIters) {
    //   assert( ( ( widthUp + 1 ) >> 1 ) == width );
    //   assert( ( ( heightUp + 1 ) >> 1 ) == height );
    unsigned char w1;
    unsigned char w2;
    unsigned char w3;
    unsigned char w4;
    for (int yUp = 0; yUp < heightUp; ++yUp) {
        int y = yUp >> 1;
        for (int xUp = 0; xUp < widthUp; ++xUp) {
            int x = xUp >> 1;
            if (occupancyMapDS[xUp + widthUp * yUp] == 0) {
                if ((xUp % 2 == 0) && (yUp % 2 == 0)) {
                    w1 = 144;
                    w2 = (x > 0 ? static_cast<unsigned char>(48) : 0);
                    w3 = (y > 0 ? static_cast<unsigned char>(48) : 0);
                    w4 = (((x > 0) && (y > 0)) ? static_cast<unsigned char>(16) : 0);
                    for (int cc = 0; cc < 3; cc++) {
                        uint8_t val = mip[x + y * width + cc * width * height];
                        uint8_t valLeft = (x > 0 ? mip[(x - 1) + y * width + cc * width * height] : 0);
                        uint8_t valUp = (y > 0 ? mip[x + (y - 1) * width + cc * width * height] : 0);
                        uint8_t valUpLeft = ((x > 0 && y > 0) ? mip[(x - 1) + (y - 1) * width + cc * width * height] : 0);
                        uint8_t newVal = mean4w(val, w1, valLeft, w2, valUp, w3, valUpLeft, w4);
                        image[xUp + yUp * widthUp + cc * widthUp * heightUp] = newVal;
                    }
                } else if ((xUp % 2 == 1) && (yUp % 2 == 0)) {
                    w1 = 144;
                    w2 = (x < width - 1 ? static_cast<unsigned char>(48) : 0);
                    w3 = (y > 0 ? static_cast<unsigned char>(48) : 0);
                    w4 = (((x < width - 1) && (y > 0)) ? static_cast<unsigned char>(16) : 0);
                    for (int cc = 0; cc < 3; cc++) {
                        uint8_t val = mip[x + y * width + cc * width * height];
                        uint8_t valRight = (x < width - 1 ? mip[(x + 1) + y * width + cc * width * height] : 0);
                        uint8_t valUp = (y > 0 ? mip[x + (y - 1) * width + cc * width * height] : 0);
                        uint8_t valUpRight = (((x < width - 1) && (y > 0)) ? mip[(x + 1) + (y - 1) * width + cc * width * height] : 0);
                        uint8_t newVal = mean4w(val, w1, valRight, w2, valUp, w3, valUpRight, w4);
                        image[xUp + yUp * widthUp + cc * widthUp * heightUp] = newVal;
                    }
                } else if ((xUp % 2 == 0) && (yUp % 2 == 1)) {
                    w1 = 144;
                    w2 = (x > 0 ? static_cast<unsigned char>(48) : 0);
                    w3 = (y < height - 1 ? static_cast<unsigned char>(48) : 0);
                    w4 = (((x > 0) && (y < height - 1)) ? static_cast<unsigned char>(16) : 0);
                    for (int cc = 0; cc < 3; cc++) {
                        uint8_t val = mip[x + y * width + cc * width * height];
                        uint8_t valLeft = (x > 0 ? mip[(x - 1) + y * width + cc * width * height] : 0);
                        uint8_t valDown = ((y < height - 1) ? mip[x + (y + 1) * width + cc * width * height] : 0);
                        uint8_t valDownLeft = ((x > 0 && (y < height - 1)) ? mip[(x - 1) + (y + 1) * width + cc * width * height] : 0);
                        uint8_t newVal = mean4w(val, w1, valLeft, w2, valDown, w3, valDownLeft, w4);
                        image[xUp + yUp * widthUp + cc * widthUp * heightUp] = newVal;
                    }
                } else {
                    w1 = 144;
                    w2 = (x < width - 1 ? static_cast<unsigned char>(48) : 0);
                    w3 = (y < height - 1 ? static_cast<unsigned char>(48) : 0);
                    w4 = (((x < width - 1) && (y < height - 1)) ? static_cast<unsigned char>(16) : 0);
                    for (int cc = 0; cc < 3; cc++) {
                        uint8_t val = mip[x + y * width + cc * width * height];
                        uint8_t valRight = (x < width - 1 ? mip[(x + 1) + y * width + cc * width * height] : 0);
                        uint8_t valDown = ((y < height - 1) ? mip[x + (y + 1) * width + cc * width * height] : 0);
                        uint8_t valDownRight =
                            (((x < width - 1) && (y < height - 1)) ? mip[(x + 1) + (y + 1) * width + cc * width * height] : 0);
                        uint8_t newVal = mean4w(val, w1, valRight, w2, valDown, w3, valDownRight, w4);
                        image[xUp + yUp * widthUp + cc * widthUp * heightUp] = newVal;
                    }
                }
            }
        }
    }
    auto tmpImage(image);
    for (size_t n = 0; n < numIters; n++) {
        for (int y = 0; y < heightUp; y++) {
            for (int x = 0; x < widthUp; x++) {
                if (occupancyMapDS[x + widthUp * y] == 0) {
                    int x1 = (x > 0) ? x - 1 : x;
                    int y1 = (y > 0) ? y - 1 : y;
                    int x2 = (x < widthUp - 1) ? x + 1 : x;
                    int y2 = (y < heightUp - 1) ? y + 1 : y;
                    for (size_t c = 0; c < 3; c++) {
                        int val = image[x1 + y1 * widthUp + c * widthUp * heightUp] + image[x2 + y1 * widthUp + c * widthUp * heightUp] +
                                  image[x1 + y2 * widthUp + c * widthUp * heightUp] + image[x2 + y2 * widthUp + c * widthUp * heightUp] +
                                  image[x1 + y * widthUp + c * widthUp * heightUp] + image[x2 + y * widthUp + c * widthUp * heightUp] +
                                  image[x + y1 * widthUp + c * widthUp * heightUp] + image[x + y2 * widthUp + c * widthUp * heightUp];
                        tmpImage[x + y * widthUp + c * widthUp * heightUp] = (val + 4) >> 3;
                    }
                }
            }
        }
        swap(image, tmpImage);
    }
}

void bgFillAttributePushPull(const std::vector<uint8_t>& occupancyMap, const size_t& gofMapsHeight, std::vector<uint8_t>& attributeMap) {
    // Algorithm from TMC2 (dilateSmoothedPushPull), slight modifications in the implementation //

    auto occupancyMapTemp = occupancyMap;

    int i = 0;
    std::vector<std::vector<uint8_t>> mipVec;
    std::vector<std::vector<uint8_t>> mipOccupancyMapVec;
    std::vector<size_t> widths;
    std::vector<size_t> heights;
    int div = 2;
    int miplev = 0;  // mip level
    size_t width = p_->mapWidth;
    size_t height = gofMapsHeight;
    size_t newWidth = ((width + 1) >> 1);
    size_t newHeight = ((height + 1) >> 1);

    // pull phase create the mipmap
    while (true) {
        mipVec.resize(mipVec.size() + 1);
        mipOccupancyMapVec.resize(mipOccupancyMapVec.size() + 1);
        widths.resize(widths.size() + 1);
        heights.resize(heights.size() + 1);

        widths[miplev] = newWidth;
        heights[miplev] = newHeight;

        div *= 2;

        if (miplev > 0) {
            pushPullMip(mipVec[miplev - 1], width, height, newWidth, newHeight, mipVec[miplev], mipOccupancyMapVec[miplev - 1],
                        mipOccupancyMapVec[miplev]);
        } else {
            pushPullMip(attributeMap, width, height, newWidth, newHeight, mipVec[miplev], occupancyMapTemp, mipOccupancyMapVec[miplev]);
        }

        if (newWidth <= 4 || newHeight <= 4) {
            break;
        }
        ++miplev;

        width = newWidth;
        height = newHeight;
        newWidth = ((width + 1) >> 1);
        newHeight = ((height + 1) >> 1);
    }
    miplev++;

    // push phase: refill
    int numIters = 4;

    size_t widthUp = 0;
    size_t heightUp = 0;

    for (i = miplev - 1; i >= 0; --i) {
        if (i > 0) {
            width = widths[i];
            height = heights[i];
            widthUp = widths[i - 1];
            heightUp = heights[i - 1];
            pushPullFill(mipVec[i - 1], width, height, widthUp, heightUp, mipVec[i], mipOccupancyMapVec[i - 1], numIters);
        } else {
            width = widths[i];
            height = heights[i];
            widthUp = p_->mapWidth;
            heightUp = gofMapsHeight;
            pushPullFill(attributeMap, width, height, widthUp, heightUp, mipVec[i], occupancyMapTemp, numIters);
        }
        numIters = (std::min)(numIters + 1, 16);
    }
}
// NOLINTEND(cppcoreguidelines-init-variables,cppcoreguidelines-init-variables,clang-analyzer-deadcode.DeadStores,hicpp-signed-bitwise,performance-unnecessary-copy-initialization,google-readability-casting,misc-const-correctness)

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,google-readability-casting,bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions,readability-qualified-auto)
void bgFillAttributePatchExtension(const std::vector<uint8_t>& occupancyMapDS, const size_t& gofMapsHeight,
                                       std::vector<uint8_t>& attributeMap) {
    // Algorithm from TMC2 (dilate), slight modifications in the implementation //
    const size_t blockSize = p_->occupancyMapDSResolution;
    const size_t mapWidth = p_->mapWidth;
    const size_t channelOffset = mapWidth * gofMapsHeight;
    const size_t occupancyMapSizeU = mapWidth / blockSize;
    const size_t occupancyMapSizeV = gofMapsHeight / blockSize;
    const size_t pixelBlockCount = blockSize * blockSize;
    const std::array<std::array<int64_t, 2>, 4> neighbors = {{{0, -1}, {-1, 0}, {1, 0}, {0, 1}}};

    std::vector<uint32_t> iterations(pixelBlockCount);
    std::vector<size_t> count(pixelBlockCount);
    std::vector<int32_t> valuesR(pixelBlockCount);
    std::vector<int32_t> valuesG(pixelBlockCount);
    std::vector<int32_t> valuesB(pixelBlockCount);
    for (size_t yOM = 0; yOM < occupancyMapSizeV; ++yOM) {
        const int64_t yBlockOffset = yOM * blockSize;
        for (size_t xOM = 0; xOM < occupancyMapSizeU; ++xOM) {
            const int64_t xBlockOffset = xOM * blockSize;

            if (occupancyMapDS[xOM + yOM * occupancyMapSizeU] == 0) {
                // No reconstructed points in this block. Simple copy of the value of the top or left pixel value. //
                if (xOM > 0) {
                    for (size_t v2 = 0; v2 < blockSize; ++v2) {
                        // Copy the left neighboring value to all pixel of the block line
                        const size_t x1 = xBlockOffset - 1;
                        const size_t y0 = yBlockOffset + v2;
                        const size_t locationLeft = x1 + y0 * mapWidth;
                        const uint8_t leftValueR = attributeMap[locationLeft];
                        const uint8_t leftValueG = attributeMap[locationLeft + channelOffset];
                        const uint8_t leftValueB = attributeMap[locationLeft + 2 * channelOffset];
                        for (size_t u2 = 0; u2 < blockSize; ++u2) {
                            const size_t location0 = xBlockOffset + u2 + y0 * mapWidth;
                            attributeMap[location0] = leftValueR;
                            attributeMap[location0 + channelOffset] = leftValueG;
                            attributeMap[location0 + 2 * channelOffset] = leftValueB;
                        }
                    }
                } else if (yOM > 0) {
                    // Copy the top neighboring value to all pixel of the block column
                    for (size_t u2 = 0; u2 < blockSize; ++u2) {
                        const size_t x0 = xBlockOffset + u2;
                        const size_t y1 = yBlockOffset - 1;
                        const size_t locationTop = x0 + y1 * mapWidth;
                        const uint8_t topValueR = attributeMap[locationTop];
                        const uint8_t topValueG = attributeMap[locationTop + channelOffset];
                        const uint8_t topValueB = attributeMap[locationTop + 2 * channelOffset];
                        for (size_t v2 = 0; v2 < blockSize; ++v2) {
                            const size_t location0 = x0 + (yBlockOffset + v2) * mapWidth;
                            attributeMap[location0] = topValueR;
                            attributeMap[location0 + channelOffset] = topValueG;
                            attributeMap[location0 + 2 * channelOffset] = topValueB;
                        }
                    }
                }
                continue;
            }

            // This block has points that will be reconstructed. //
            size_t emptyPixelCount = 0;
            std::fill(iterations.begin(), iterations.end(), 0);
            for (size_t v2 = 0; v2 < blockSize; ++v2) {
                for (size_t u2 = 0; u2 < blockSize; ++u2) {
                    const int64_t x0 = xBlockOffset + u2;
                    const int64_t y0 = yBlockOffset + v2;
                    const size_t location0 = y0 * mapWidth + x0;

                    // lf : notice that the following check will count as non-reconstructed a point with a R value of 128
                    // (p_->mapGenerationBackgroundValueAttribute). This would ultimately create a faulty non zero pixel counter which would
                    // create an infinite while loop later. To adress this situation, an extra condition based on the number of iterations is
                    // added in the while loop.
                    if (attributeMap[location0] == p_->mapGenerationBackgroundValueAttribute) {
                        emptyPixelCount++;
                    } else {
                        iterations[u2 + v2 * blockSize] = 1;
                    }
                }
            }

            if (emptyPixelCount == 0) {
                // All pixels of the block already have a value. //
                continue;
            }

            // Some pixels in this block need to be filled with an average value of their neighboring pixels. //
            std::fill(count.begin(), count.end(), 0);
            std::fill(valuesR.begin(), valuesR.end(), 0);
            std::fill(valuesG.begin(), valuesG.end(), 0);
            std::fill(valuesB.begin(), valuesB.end(), 0);
            size_t iteration = 1;
            while (emptyPixelCount > 0 && iteration < pixelBlockCount) {  // lf : The second check avoid infinite loop due to imperfect
                                                                          // detection of non zero pixel (c.f. previous comment).
                // assert(emptyPixelCount < pixelBlockCount);
                for (size_t v2 = 0; v2 < blockSize; ++v2) {
                    for (size_t u2 = 0; u2 < blockSize; ++u2) {
                        const int64_t x0 = xBlockOffset + u2;          // current pixel location on the occupancy map
                        const int64_t y0 = yBlockOffset + v2;          // current pixel location on the occupancy map
                        const size_t location2 = u2 + v2 * blockSize;  // current pixel location on the block
                        if (iterations[location2] == iteration) {
                            for (auto neighbor : neighbors) {
                                const int64_t x1 = x0 + neighbor[0];           // neighbor pixel location on the occupancy map
                                const int64_t y1 = y0 + neighbor[1];           // neighbor pixel location on the occupancy map
                                const int64_t u3 = u2 + neighbor[0];           // neighbor pixel location on the block
                                const int64_t v3 = v2 + neighbor[1];           // neighbor pixel location on the block
                                const size_t location3 = u3 + v3 * blockSize;  // neighbor pixel location on the block
                                if (x1 >= xBlockOffset && x1 < int64_t(xBlockOffset + blockSize) && y1 >= yBlockOffset &&
                                    y1 < int64_t(yBlockOffset + blockSize) && iterations[location3] == 0) {
                                    const size_t location0 = x0 + y0 * mapWidth;  // current pixel location on the occupancy map
                                    valuesR[location3] += attributeMap[location0];
                                    valuesG[location3] += attributeMap[location0 + channelOffset];
                                    valuesB[location3] += attributeMap[location0 + 2 * channelOffset];
                                    ++count[location3];
                                }
                            }
                        }
                    }
                }
                for (size_t v2 = 0; v2 < blockSize; ++v2) {
                    for (size_t u2 = 0; u2 < blockSize; ++u2) {
                        const size_t location2 = u2 + v2 * blockSize;  // current pixel location on the block
                        if (count[location2] > 0U) {
                            const size_t x0 = xBlockOffset + u2;          // current pixel location on the occupancy map
                            const size_t y0 = yBlockOffset + v2;          // current pixel location on the occupancy map
                            const size_t location0 = x0 + y0 * mapWidth;  // current pixel location on the occupancy map
                            const size_t c = count[location2];
                            const size_t c2 = c / 2;  // Allows better rounding of the computed average value
                            attributeMap[location0] = static_cast<uint8_t>((valuesR[location2] + c2) / c);
                            attributeMap[location0 + channelOffset] = static_cast<uint8_t>((valuesG[location2] + c2) / c);
                            attributeMap[location0 + 2 * channelOffset] = static_cast<uint8_t>((valuesB[location2] + c2) / c);
                            iterations[location2] = iteration + 1;
                            --emptyPixelCount;
                            count[location2] = 0;
                        }
                    }
                }
                ++iteration;
            }
        }
    }
}
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,google-readability-casting,bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions,readability-qualified-auto)

void attributeBgFillBBPE(uvgvpcc_enc::Frame& frame, std::vector<uint8_t>& attributeMap) {
    const size_t BBPEOccupancyWidth = p_->mapWidth / p_->blockSizeBBPE;
    const size_t BBPEOccupancyHeight = frame.mapHeight / p_->blockSizeBBPE;
    const size_t blockSizeBBPEInDSBlk = p_->blockSizeBBPE / p_->occupancyMapDSResolution;
    const size_t blockSize = p_->blockSizeBBPE;
    const size_t occupancyMapDSWidth = p_->mapWidth / p_->occupancyMapDSResolution;
    const size_t mapWidth = p_->mapWidth;
    const size_t mapHeight = frame.mapHeight;
    const size_t fullBlockPixelCount = blockSize * blockSize;
    const size_t channelOffset = mapWidth * mapHeight;

    std::vector<uint8_t> iterations(fullBlockPixelCount);
    std::vector<size_t> count(fullBlockPixelCount);
    std::vector<int32_t> valuesR(fullBlockPixelCount);
    std::vector<int32_t> valuesG(fullBlockPixelCount);
    std::vector<int32_t> valuesB(fullBlockPixelCount);

    const std::array<std::array<int64_t, 2>, 4> neighbors = {{{0, -1}, {-1, 0}, {1, 0}, {0, 1}}};

    for (size_t yBBPE = 0; yBBPE < BBPEOccupancyHeight; ++yBBPE) {
        const size_t yBBPE_DS_offset = yBBPE * blockSizeBBPEInDSBlk;
        const size_t yBBPE_Pixel_offset = yBBPE * blockSize;

        for (size_t xBBPE = 0; xBBPE < BBPEOccupancyWidth; ++xBBPE) {
            const size_t xBBPE_DS_offset = xBBPE * blockSizeBBPEInDSBlk;
            const size_t xBBPE_Pixel_offset = xBBPE * blockSize;

            // Phase 1: Check DS occupancy
            bool occupied = false;
            for (size_t j = 0; j < blockSizeBBPEInDSBlk && !occupied; ++j) {
                const size_t yDS = yBBPE_DS_offset + j;
                const size_t rowOffset = yDS * occupancyMapDSWidth;
                for (size_t i = 0; i < blockSizeBBPEInDSBlk; ++i) {
                    if (frame.occupancyMapDS[rowOffset + xBBPE_DS_offset + i] > 0U) {
                        occupied = true;
                        break;
                    }
                }
            }
            if (!occupied) {
                continue;
            }

            // Phase 2: Count full-res occupancy
            size_t occupiedPixelCount = 0;
            for (size_t j = 0; j < blockSize; ++j) {
                const size_t rowOffset = (yBBPE_Pixel_offset + j) * mapWidth + xBBPE_Pixel_offset;
                for (size_t i = 0; i < blockSize; ++i) {
                    occupiedPixelCount += frame.occupancyMap[rowOffset + i];
                }
            }

            if (occupiedPixelCount == fullBlockPixelCount) {
                continue;
            }

            // Reset buffers
            // std::fill(iterations.begin(), iterations.end(), 0);
            std::fill(count.begin(), count.end(), 0);
            std::fill(valuesR.begin(), valuesR.end(), 0);
            std::fill(valuesG.begin(), valuesG.end(), 0);
            std::fill(valuesB.begin(), valuesB.end(), 0);

            size_t emptyPixelCount = fullBlockPixelCount - occupiedPixelCount;

            // Initialization
            for (size_t j = 0; j < blockSize; ++j) {
                const size_t y = yBBPE_Pixel_offset + j;
                const size_t yStride = y * mapWidth;
                for (size_t i = 0; i < blockSize; ++i) {
                    const size_t x = xBBPE_Pixel_offset + i;
                    const size_t loc = i + j * blockSize;
                    iterations[loc] = frame.occupancyMap[x + yStride];
                }
            }

            // Propagation
            size_t iteration = 1;
            while (emptyPixelCount > 0 && iteration < fullBlockPixelCount) {
                for (size_t v = 0; v < blockSize; ++v) {
                    const size_t y0 = yBBPE_Pixel_offset + v;
                    const size_t y0Stride = y0 * mapWidth;
                    for (size_t u = 0; u < blockSize; ++u) {
                        const size_t loc = u + v * blockSize;
                        if (iterations[loc] != iteration) {
                            continue;
                        }

                        const size_t x0 = xBBPE_Pixel_offset + u;
                        const size_t loc0 = x0 + y0Stride;

                        for (const auto& [dx, dy] : neighbors) {
                            const int64_t u3 = static_cast<int64_t>(u) + dx;
                            const int64_t v3 = static_cast<int64_t>(v) + dy;
                            if (u3 < 0 || v3 < 0 || u3 >= static_cast<int64_t>(blockSize) || v3 >= static_cast<int64_t>(blockSize)) {
                                continue;
                            }

                            const size_t loc3 = static_cast<size_t>(u3) + static_cast<size_t>(v3) * blockSize;
                            if (iterations[loc3] > 0U) {
                                continue;
                            }

                            valuesR[loc3] += attributeMap[loc0];
                            valuesG[loc3] += attributeMap[loc0 + channelOffset];
                            valuesB[loc3] += attributeMap[loc0 + 2 * channelOffset];
                            ++count[loc3];
                        }
                    }
                }

                ++iteration;
                for (size_t loc = 0; loc < fullBlockPixelCount; ++loc) {
                    const size_t locCount = count[loc];
                    if (locCount > 0U) {
                        const size_t u = loc % blockSize;
                        const size_t v = loc / blockSize;
                        const size_t x0 = xBBPE_Pixel_offset + u;
                        const size_t y0 = yBBPE_Pixel_offset + v;
                        const size_t loc0 = x0 + y0 * mapWidth;

                        attributeMap[loc0] = static_cast<uint8_t>((valuesR[loc] + locCount / 2) / locCount);
                        attributeMap[loc0 + channelOffset] = static_cast<uint8_t>((valuesG[loc] + locCount / 2) / locCount);
                        attributeMap[loc0 + 2 * channelOffset] = static_cast<uint8_t>((valuesB[loc] + locCount / 2) / locCount);

                        iterations[loc] = iteration;
                        --emptyPixelCount;
                        count[loc] = 0;
                    }
                }
            }
        }
    }
}

}  // anonymous namespace

void bgFillAttribute(uvgvpcc_enc::Frame& frame, std::vector<uint8_t>& attributeMap) {
    // TODO(lf): make an enum and use a switch
    if (p_->attributeBgFill == "patchExtension") {
        bgFillAttributePatchExtension(frame.occupancyMapDS, frame.mapHeight, attributeMap);
    } else if (p_->attributeBgFill == "bbpe") {
        attributeBgFillBBPE(frame, attributeMap);
    } else if (p_->attributeBgFill == "pushPull") {
        bgFillAttributePushPull(frame.occupancyMap, frame.mapHeight, attributeMap);
    } else if (p_->attributeBgFill == "none") {
        // Skip attribute map background filling
    } else {
        throw std::runtime_error("Unknown p_->attributeBgFill: " + p_->attributeBgFill);
    }
}