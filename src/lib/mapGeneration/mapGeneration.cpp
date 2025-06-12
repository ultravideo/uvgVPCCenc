/*****************************************************************************
 * This file is part of uvgVPCCenc V-PCC encoder.
 *
 * Copyright (c) 2024, Tampere University, ITU/ISO/IEC, project contributors
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

/// \file Entry point for the map generation process. Use the 2D location of the patch obtained during patch packing to create the occupancy, geometry and attribute 2D maps.

#include "mapGeneration.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "uvgvpcc/log.hpp"
#include "uvgvpcc/uvgvpcc.hpp"
#include "utils/utils.hpp"

using namespace uvgvpcc_enc;

void MapGenerationBaseLine::initializeStaticParameters() {}


namespace {
    
    // lf: Notice that the current implementation of the occupancy map refinement does not remove the involved points from their patch.
    template <uint8_t occBlkSize>
    void occupancyMapDownscaling(const size_t& mapHeight, std::vector<uint8_t>& occupancyMap, std::vector<uint8_t>& occupancyMapDS) {
    
        const size_t mapWidth = p_->mapWidth;
        
        uint8_t* occMap = occupancyMap.data();
        uint8_t* occMapDS = occupancyMapDS.data();
        if constexpr (occBlkSize == 2) {
            const size_t mapWidthDS = mapWidth >> 1U;
            const size_t mapHeightDS = mapHeight >> 1U;
            for (size_t yDS = 0; yDS < mapHeightDS; ++yDS) {
                const size_t yOffset = (yDS * mapWidth) << 1U;
                for (size_t xDS = 0; xDS < mapWidthDS; ++xDS) {
                    const size_t xOffset = xDS << 1U;
                    uint8_t* blockPtr = occMap + yOffset + xOffset;
                    
                    const uint8_t sum =
                        blockPtr[0] + blockPtr[1] +
                        blockPtr[mapWidth] + blockPtr[mapWidth + 1];

                    if(sum >= p_->omRefinementTreshold2) {
                        occMapDS[yDS * mapWidthDS + xDS] = 1U;
                    } else {
                        occMapDS[yDS * mapWidthDS + xDS] = 0U;
                        // Update the occupancy map (lf: usefull for BBPE attribute background filling)
                        std::fill_n(blockPtr, 2, 0U);
                        std::fill_n(blockPtr + mapWidth, 2, 0U);
                    }
                }
            }
        } else if constexpr (occBlkSize == 4) {
            const size_t mapWidthDS = mapWidth >> 2U;
            const size_t mapHeightDS = mapHeight >> 2U;        
            for (size_t yDS = 0; yDS < mapHeightDS; ++yDS) {
                const size_t yOffset = (yDS * mapWidth) << 2U;
                for (size_t xDS = 0; xDS < mapWidthDS; ++xDS) {
                    const size_t xOffset = xDS << 2U;
                    uint8_t* blockPtr = occMap + yOffset + xOffset;
                    
                    const uint8_t sum =
                    blockPtr[0] + blockPtr[1] + blockPtr[2] + blockPtr[3] +
                    blockPtr[mapWidth] + blockPtr[mapWidth + 1] + blockPtr[mapWidth + 2] + blockPtr[mapWidth + 3] +
                    blockPtr[2 * mapWidth] + blockPtr[2 * mapWidth + 1] + blockPtr[2 * mapWidth + 2] + blockPtr[2 * mapWidth + 3] +
                    blockPtr[3 * mapWidth] + blockPtr[3 * mapWidth + 1] + blockPtr[3 * mapWidth + 2] + blockPtr[3 * mapWidth + 3];

                    if(sum >= p_->omRefinementTreshold4) {
                        occMapDS[yDS * mapWidthDS + xDS] = 1U;
                    } else {
                        occMapDS[yDS * mapWidthDS + xDS] = 0U;
                        // Update the occupancy map (lf: usefull for BBPE attribute background filling)
                        std::fill_n(blockPtr, 4, 0U);
                        std::fill_n(blockPtr + 1 * mapWidth, 4, 0U);
                        std::fill_n(blockPtr + 2 * mapWidth, 4, 0U);
                        std::fill_n(blockPtr + 3 * mapWidth, 4, 0U);
                    }
                }
            }
        } else {
            assert(false && "Unsupported block size for occupancy map downscaling");
        }
    }



template <bool doubleLayer, bool axisSwap>
void writePatchT(const uvgvpcc_enc::Patch& patch, const size_t& imageSize,const std::shared_ptr<uvgvpcc_enc::Frame>& frame) {
    const size_t patchWidth = patch.widthInPixel_;
    const size_t patchHeight = patch.heightInPixel_;
    const size_t omX = patch.omDSPosX_ * p_->occupancyMapDSResolution;
    const size_t omY = patch.omDSPosY_ * p_->occupancyMapDSResolution;
    const size_t mapWidth = p_->mapWidth;
    const size_t imageSize2 = 2 * imageSize;

    const auto* depthL1 = patch.depthL1_.data();
    const auto* depthPCidxL1 = patch.depthPCidxL1_.data();
    const auto* depthL2 = patch.depthL2_.data();
    const auto* depthPCidxL2 = patch.depthPCidxL2_.data();
    const auto& attributes = frame->pointsAttribute;

    auto* geomL1 = frame->geometryMapL1.data();
    auto* attrL1 = frame->attributeMapL1.data();
    auto* geomL2 = frame->geometryMapL2.data();
    auto* attrL2 = frame->attributeMapL2.data();

    for (size_t v = 0; v < patchHeight; ++v) {
        const size_t vOffset = v * patchWidth;

        for (size_t u = 0; u < patchWidth; ++u) {
            const size_t patchPos = u + vOffset;
            const auto depth = depthL1[patchPos];
            if (depth == g_infiniteDepth) continue;

            const size_t x = axisSwap ? v : u;
            const size_t y = axisSwap ? u : v;
            const size_t mapPos = omX + x + (omY + y) * mapWidth;

            const auto& attrL1Val = attributes[depthPCidxL1[patchPos]];
            geomL1[mapPos] = depth;
            attrL1[mapPos] = attrL1Val[0];
            attrL1[mapPos + imageSize] = attrL1Val[1];
            attrL1[mapPos + imageSize2] = attrL1Val[2];

            if constexpr (doubleLayer) {
                const auto& attrL2Val = attributes[depthPCidxL2[patchPos]];
                geomL2[mapPos] = depthL2[patchPos];
                attrL2[mapPos] = attrL2Val[0];
                attrL2[mapPos + imageSize] = attrL2Val[1];
                attrL2[mapPos + imageSize2] = attrL2Val[2];
            }
        }
    }
}

} // Anonymous namespace

void MapGenerationBaseLine::writePatches(const std::shared_ptr<uvgvpcc_enc::Frame>& frame, const size_t& gofMapsHeight) {
    const size_t imageSize = p_->mapWidth * gofMapsHeight;

    if(p_->doubleLayer) {
        for (const uvgvpcc_enc::Patch& patch : frame->patchList) {
            if (patch.axisSwap_) {
                writePatchT<true, true>(patch, imageSize, frame);
            } else {
                writePatchT<true, false>(patch, imageSize, frame);
            }
        }        
    } else {
        for (const uvgvpcc_enc::Patch& patch : frame->patchList) {
            if (patch.axisSwap_) {
                writePatchT<false, true>(patch, imageSize, frame);
            } else {
                writePatchT<false, false>(patch, imageSize, frame);
            }
        }
    }
}


void MapGenerationBaseLine::allocateMaps(const std::shared_ptr<uvgvpcc_enc::Frame>& frame, const size_t& gofMapsHeight) {
    // Notice that before this operation, the dimension of each frame occupancy map can be different. Thus, this OM resizing operation both
    // makes all GOF occupancy maps dimension uniform and convert them to YUV420. FYI, U and V images of the occupancy and geometry maps are
    // empty/not used by the decoder/do not cary any usefull information.

    const size_t imageSize = p_->mapWidth * gofMapsHeight;

    // The occupancy map already exist and might already has the correct size.


    // TODO(lf): is it necessary ? Yes if resizing due to bigger occupancy map (larger than minimumHeight parameter)
    // assert(frame->occupancyMap.size() == imageSize);
    if (frame->occupancyMap.size() != imageSize) {
        frame->occupancyMap.resize(imageSize,0);
    }

    const size_t imageSizeDS = imageSize / (p_->occupancyMapDSResolution*p_->occupancyMapDSResolution);
    frame->occupancyMapDS.resize(imageSizeDS + (imageSizeDS >> 1U), 0U);    


    frame->geometryMapL1.resize(imageSize + (imageSize >> 1U), p_->mapGenerationBackgroundValueGeometry);
    frame->attributeMapL1.resize(static_cast<size_t>(imageSize) * 3, p_->mapGenerationBackgroundValueAttribute);
    // TODO(lf): what is the justification for the max value ?

    if (p_->doubleLayer) {
        frame->geometryMapL2.resize(imageSize + (imageSize >> 1U), p_->mapGenerationBackgroundValueGeometry);
        frame->attributeMapL2.resize(static_cast<size_t>(imageSize) * 3, p_->mapGenerationBackgroundValueAttribute);
    }
}

namespace {


// TODO(lf): Why an integer only implementation is so bad in term of quality degradation?
void RGB444toYUV420(std::vector<uint8_t>& img, const size_t& width, const size_t& height) {
    Logger::log(LogLevel::TRACE, "MapGeneration", "RGB444toYUV420\n");

    const size_t imageSize = width * height;
    const size_t imageSizeUV = imageSize >> 2U;

    const uint8_t* rChannel = img.data();
    const uint8_t* gChannel = rChannel + imageSize;
    const uint8_t* bChannel = gChannel + imageSize;

    std::vector<uint8_t> yuv420(imageSize + imageSizeUV * 2);
    uint8_t* yChannel = yuv420.data();
    uint8_t* uChannel = yChannel + imageSize;
    uint8_t* vChannel = uChannel + imageSizeUV;

    constexpr float kYR = 0.2126F;
    constexpr float kYG = 0.7152F;
    constexpr float kYB = 0.0722F;

    constexpr float kUR = -0.114572F;
    constexpr float kUG = -0.385428F;
    constexpr float kUB = 0.5F;

    constexpr float kVR = 0.5F;
    constexpr float kVG = -0.454153F;
    constexpr float kVB = -0.045847F;

    size_t idxUV = 0;

    for (size_t y = 0; y < height; y += 2) {
        const size_t row1 = y * width;
        const size_t row2 = row1 + width;

        for (size_t x = 0; x < width; x += 2) {
            const size_t i00 = row1 + x;
            const size_t i01 = i00 + 1;
            const size_t i10 = row2 + x;
            const size_t i11 = i10 + 1;

            const float r00 = static_cast<float>(rChannel[i00]);
            const float g00 = static_cast<float>(gChannel[i00]);
            const float b00 = static_cast<float>(bChannel[i00]);

            const float r01 = static_cast<float>(rChannel[i01]);
            const float g01 = static_cast<float>(gChannel[i01]);
            const float b01 = static_cast<float>(bChannel[i01]);

            const float r10 = static_cast<float>(rChannel[i10]);
            const float g10 = static_cast<float>(gChannel[i10]);
            const float b10 = static_cast<float>(bChannel[i10]);

            const float r11 = static_cast<float>(rChannel[i11]);
            const float g11 = static_cast<float>(gChannel[i11]);
            const float b11 = static_cast<float>(bChannel[i11]);

            yChannel[i00] = static_cast<uint8_t>(kYR * r00 + kYG * g00 + kYB * b00);
            yChannel[i01] = static_cast<uint8_t>(kYR * r01 + kYG * g01 + kYB * b01);
            yChannel[i10] = static_cast<uint8_t>(kYR * r10 + kYG * g10 + kYB * b10);
            yChannel[i11] = static_cast<uint8_t>(kYR * r11 + kYG * g11 + kYB * b11);

            const float avgR = 0.25F * (r00 + r01 + r10 + r11);
            const float avgG = 0.25F * (g00 + g01 + g10 + g11);
            const float avgB = 0.25F * (b00 + b01 + b10 + b11);

            uChannel[idxUV] = static_cast<uint8_t>(kUR * avgR + kUG * avgG + kUB * avgB + 128.F);
            vChannel[idxUV] = static_cast<uint8_t>(kVR * avgR + kVG * avgG + kVB * avgB + 128.F);
            ++idxUV;
        }
    }

    img.swap(yuv420);
}

} // anonymous namespace


// TODO(lf): use copy with relevant optimal memory copy to fill second layer. Tackle the cognitive complexity accordingly
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void MapGenerationBaseLine::fillBackgroundEmptyBlock(const std::shared_ptr<uvgvpcc_enc::Frame>& frame, const size_t blockSize, const size_t imageSize,
                                                     const size_t uBlk, const size_t vBlk, const size_t uom,
                                                     const size_t vom) {
    if (uBlk > 0) {
        for (size_t j = 0; j < blockSize; ++j) {
            const size_t currentY = vom + j;
            for (size_t i = 0; i < blockSize; ++i) {
                const size_t currentPos = uom + i + currentY * p_->mapWidth;
                const size_t previousPos = uom + i - 1 + currentY * p_->mapWidth;  // pixel on left

                frame->geometryMapL1[currentPos] = frame->geometryMapL1[previousPos];

                frame->attributeMapL1[currentPos] = frame->attributeMapL1[previousPos];
                frame->attributeMapL1[currentPos + imageSize] = frame->attributeMapL1[previousPos + imageSize];
                frame->attributeMapL1[currentPos + 2 * imageSize] = frame->attributeMapL1[previousPos + 2 * imageSize];

                if (p_->doubleLayer) {
                    frame->geometryMapL2[currentPos] = frame->geometryMapL1[currentPos];
                    frame->attributeMapL2[currentPos] = frame->attributeMapL1[currentPos];
                    frame->attributeMapL2[currentPos + imageSize] = frame->attributeMapL1[currentPos + imageSize];
                    frame->attributeMapL2[currentPos + 2 * imageSize] = frame->attributeMapL1[currentPos + 2 * imageSize];
                }
            }
        }
    } else if (vBlk > 0) {  // first left column, copy top next block value
        for (size_t j = 0; j < blockSize; ++j) {
            const size_t currentY = vom + j;
            const size_t previousY = currentY - 1;
            for (size_t i = 0; i < blockSize; ++i) {
                const size_t currentPos = uom + i + currentY * p_->mapWidth;
                const size_t previousPos = uom + i + previousY * p_->mapWidth;  // pixel on top

                frame->geometryMapL1[currentPos] = frame->geometryMapL1[previousPos];
                frame->attributeMapL1[currentPos] = frame->attributeMapL1[previousPos];
                frame->attributeMapL1[currentPos + imageSize] = frame->attributeMapL1[previousPos + imageSize];
                frame->attributeMapL1[currentPos + 2 * imageSize] = frame->attributeMapL1[previousPos + 2 * imageSize];

                if (p_->doubleLayer) {
                    frame->geometryMapL2[currentPos] = frame->geometryMapL1[currentPos];
                    frame->attributeMapL2[currentPos] = frame->attributeMapL1[currentPos];
                    frame->attributeMapL2[currentPos + imageSize] = frame->attributeMapL1[currentPos + imageSize];
                    frame->attributeMapL2[currentPos + 2 * imageSize] = frame->attributeMapL1[currentPos + 2 * imageSize];
                }
            }
        }
    } else {
        // lf : In TMC2 the top left block, if it is an empty block, keep the default value. Here, we put the middle (mid-gray)
        for (size_t j = 0; j < blockSize; ++j) {
            for (size_t i = 0; i < blockSize; ++i) {
                const size_t currentPos = uom + i + (vom + j) * p_->mapWidth;
                const size_t fillingValue = 128;

                frame->geometryMapL1[currentPos] = fillingValue;
                frame->attributeMapL1[currentPos] = fillingValue;
                frame->attributeMapL1[currentPos + imageSize] = fillingValue;
                frame->attributeMapL1[currentPos + 2 * imageSize] = fillingValue;

                if (p_->doubleLayer) {
                    frame->geometryMapL2[currentPos] = fillingValue;
                    frame->attributeMapL2[currentPos] = fillingValue;
                    frame->attributeMapL2[currentPos + imageSize] = fillingValue;
                    frame->attributeMapL2[currentPos + 2 * imageSize] = fillingValue;
                }
            }
        }
    }
}

void MapGenerationBaseLine::updateSums(const std::shared_ptr<uvgvpcc_enc::Frame>& frame, const size_t blockLeft, const size_t blockTop,
                                       const size_t iBlk, const size_t jBlk, const size_t imageSize,
                                       std::vector<size_t>& iterations, const size_t blockSize, std::vector<size_t>& sumGeo,
                                       std::vector<size_t>& sumR, std::vector<size_t>& sumG, std::vector<size_t>& sumB,
                                       std::vector<size_t>& count) {
    const std::array<std::array<int8_t, 2>, 4> neighbors = {{{0, -1}, {-1, 0}, {1, 0}, {0, 1}}};

    const size_t currentXOM = blockLeft + iBlk;
    const size_t currentYOM = blockTop + jBlk;
    const size_t currentPosOM = currentXOM + currentYOM * p_->mapWidth;
    // filled pixel at the first iteration)
    for (int i = 0; i < 4; ++i) {
        const size_t neighborX = currentXOM + neighbors[i][0];
        const size_t neighborY = currentYOM + neighbors[i][1];
        const size_t currentPosBlk = iBlk + neighbors[i][0] + (jBlk + neighbors[i][1]) * blockSize;
        // lf : TODO(lf) why to check if the neighbor is in the current block ? We should check if it in the occupancy map. Otherwise, we could
        // use the pixels from other block to have more relevant values.
        if (neighborX >= blockLeft && neighborX < static_cast<size_t>(blockLeft + blockSize) && neighborY >= blockTop &&
            neighborY < static_cast<size_t>(blockTop + blockSize) && iterations[currentPosBlk] == 0) {  // missingPoint => iteration==0
            // add current border pixel value in the current neighbor sumGeo values
            sumGeo[currentPosBlk] += frame->geometryMapL1[currentPosOM];

            sumR[currentPosBlk] += frame->attributeMapL1[currentPosOM];
            sumG[currentPosBlk] += frame->attributeMapL1[currentPosOM + imageSize];
            sumB[currentPosBlk] += frame->attributeMapL1[currentPosOM + 2 * imageSize];
            ++count[currentPosBlk];
        }
    }
}

void MapGenerationBaseLine::fillBackgroundNonEmptyBlock(const std::shared_ptr<uvgvpcc_enc::Frame>& frame, const size_t blockSize, const size_t imageSize,
                                                        const size_t uom, const size_t vom, const size_t pixelBlockCount,
                                                        size_t missingPixelCount, std::vector<size_t>& iterations) {
    // lf : knowing that we should not used a occupancyMapDSResolution (precision?) higher than 4 (probably 1(no downscaling) or 2), the
    // current algorithm seems overkill. Simple use of lookup table can do it I think.

    // lf : WARNING : what happen if the true value of the geometry is the uint8 max value ?

    // lf : optimization possible -> different result, but less complex -> if there are missing pixels. Do four "passes", in each
    // direction (up to down, down to top, left to right, right to left). The 'values' for each passe will just be the one before. And
    // the count will be incremented if there is a value before. lf : the same optimization can be used using the same iteration
    // scheme. (stop propagation in a line or column after reaching one missing pixel) lf : for each pixel of the focused block
    // (uBlk,vBlk), if it is an empty pixel (no value in the occupancyMapDS  (lf OPT : check in the geometry map instead (seems to be
    // done like this in this RW function)), do nothing. lf : if it is a filled pixel, add the value of this pixel in its empty
    // neighboring pixels in the "value" array, which has the size of a block. Increase by one the count value of all neighbor pixel.

    std::vector<size_t> count(pixelBlockCount, 0);
    std::vector<size_t> sumGeo(pixelBlockCount, 0);
    std::vector<size_t> sumR(pixelBlockCount, 0);
    std::vector<size_t> sumG(pixelBlockCount, 0);
    std::vector<size_t> sumB(pixelBlockCount, 0);
    size_t iteration = 1;

    // lf TODO(lf): the fact that other block can be considered during the average computation should be improve. Only block related to
    // the same patch should be used to ensure relevant color (it might help to avoid those blue line on the face of longdress for
    // example). lf : propagation of the average to fill the missing points TODO(lf): use lookup table -> warning, pixel from
    // neighboring block are also used for the average
    while (missingPixelCount > 0 && iteration < pixelBlockCount) {
        // lf note: the second condition (iteration < pixelBlockCount) is for safety only. A deadlock can happen if some pixel got a real
        // geometry value of 128 which is the default background value. iterate over all pixels of the block to initialise the averages
        for (size_t j = 0; j < blockSize; ++j) {
            for (size_t i = 0; i < blockSize; ++i) {
                if (iterations[i + j * blockSize] == iteration) {  // lf border pixel (created from the previous iteration or simply
                    updateSums(frame, uom, vom, i, j, imageSize, iterations, blockSize, sumGeo, sumR, sumG, sumB, count);
                }
            }
        }
        // iterate over all pixels of the block to assign values if possible
        for (size_t j = 0; j < blockSize; ++j) {
            for (size_t i = 0; i < blockSize; ++i) {
                const size_t pixelPos = i + j * blockSize;
                if (count[pixelPos] != 0U) {  // lf : it has neighbors (at least one) so a value can be added
                    const size_t currentXOM = uom + i;
                    const size_t currentYOM = vom + j;
                    const size_t currentPosOM = currentXOM + currentYOM * p_->mapWidth;

                    // lf : Like in TMC2, the average computation is biased. Not sure why... To create a gradient ?

                    frame->geometryMapL1[currentPosOM] =
                        static_cast<uint8_t>((sumGeo[pixelPos] + count[pixelPos] / 2) / count[pixelPos]);

                    // TODO(lf): in TMC2 both map are doing it separately. Here, as a temporary solution, we do it only on L1

                    frame->attributeMapL1[currentPosOM] =
                        static_cast<uint8_t>((sumR[pixelPos] + count[pixelPos] / 2) / count[pixelPos]);
                    frame->attributeMapL1[currentPosOM + imageSize] =
                        static_cast<uint8_t>((sumG[pixelPos] + count[pixelPos] / 2) / count[pixelPos]);
                    frame->attributeMapL1[currentPosOM + 2 * imageSize] =
                        static_cast<uint8_t>((sumB[pixelPos] + count[pixelPos] / 2) / count[pixelPos]);

                    if (p_->doubleLayer) {
                        frame->geometryMapL2[currentPosOM] = frame->geometryMapL1[currentPosOM];
                        frame->attributeMapL2[currentPosOM] = frame->attributeMapL1[currentPosOM];
                        frame->attributeMapL2[currentPosOM + imageSize] = frame->attributeMapL1[currentPosOM + imageSize];
                        frame->attributeMapL2[currentPosOM + 2 * imageSize] = frame->attributeMapL1[currentPosOM + 2 * imageSize];
                    }

                    iterations[pixelPos] = iteration + 1;  // lf considered as a border pixel at the next iteration
                    count[pixelPos] = 0;
                    --missingPixelCount;
                }
            }
        }
        ++iteration;
    }
}

void MapGenerationBaseLine::fillBackgroundImages(const std::shared_ptr<uvgvpcc_enc::Frame>& frame, const size_t& gofMapsHeight) {
    const size_t blockSize = p_->occupancyMapDSResolution;
    const size_t occupancyMapDSWidthBlk =
        p_->mapWidth / blockSize;  // TODO(lf): this should be a frame param (yes for the height, and a static param for the width)
    const size_t occupancyMapDSHeightBlk = gofMapsHeight / blockSize;  // what is the difference with occupancyImage.getHeight() ??
    const size_t imageSize = p_->mapWidth * gofMapsHeight;
    const size_t pixelBlockCount = blockSize * blockSize;  // lf nb of pixel per block from the frameOM POV

    // If p_->mapGenerationFillEmptyBlock == false, we need still need to fill the non-empty CTU. (A CTU with at least one OM block is
    // non-empty.) A CTU is 64x64. If a CTU is empty, we can skip it (that is, keep the uniform gray background). TODO(lf): make this ctu check

    // iterate over each block of the occupancy map
    for (size_t vBlk = 0; vBlk < occupancyMapDSHeightBlk; ++vBlk) {
        const size_t vom = vBlk * blockSize;
        for (size_t uBlk = 0; uBlk < occupancyMapDSWidthBlk; ++uBlk) {
            const size_t uom = uBlk * blockSize;

            // empty block -> copy the value of previous block (one of the TMC2 solution) or do nothing (let the uniform value set during map
            // allocation)
            if (frame->occupancyMapDS[uBlk + vBlk * occupancyMapDSWidthBlk] == 0U) {
                if (p_->mapGenerationFillEmptyBlock) {
                    fillBackgroundEmptyBlock(frame, blockSize, imageSize, uBlk, vBlk, uom, vom);
                }
                continue;
            }
            // TODO(lf): lf : empty block inside a non empty CTU (64x64) should not be filled with gray but should extend the average value I
            // guess

            // non empty block -> check if all pixels of the block already have a value or not
            size_t missingPixelCount = 0;
            std::vector<size_t> iterations(pixelBlockCount, 0);
            for (size_t j = 0; j < blockSize; ++j) {
                for (size_t i = 0; i < blockSize; ++i) {
                    const size_t currentPosOM = uom + i + (vom + j) * p_->mapWidth;
                    // TODO(lf): u_int16_y should be a typedef for geometry map (different from geometry precision ?)
                    // TODO(lf): this is not a perfect detection. Indeed, what if all pixel in this block have really 128 as depth value ? lf :
                    // a safety has been added to avoid a deadlock in the filling block process
                    if (frame->geometryMapL1[currentPosOM] == p_->mapGenerationBackgroundValueGeometry) {
                        ++missingPixelCount;
                    } else {
                        iterations[i + j * blockSize] = 1;
                        // lf : first iteration of the propagation average could be done here
                    }
                }
            }

            // all pixels in the block have a value -> nothing to do
            if (missingPixelCount == 0) {
                continue;
            }

            // one or more pixel need a value. Those pixel will appear in the decoded point cloud. They result from the downscaling of the
            // occupancy map. -> Compute average value from neigboring pixels
            fillBackgroundNonEmptyBlock(frame, blockSize, imageSize, uom, vom, pixelBlockCount, missingPixelCount, iterations);
        }
    }
}

void MapGenerationBaseLine::generateFrameMaps(const std::shared_ptr<uvgvpcc_enc::Frame>& frame) {
    allocateMaps(frame, frame->mapHeight);

    if(p_->occupancyMapDSResolution == 2) {
        occupancyMapDownscaling<2>(frame->mapHeight,frame->occupancyMap,frame->occupancyMapDS);
    } else if (p_->occupancyMapDSResolution == 4) {
        occupancyMapDownscaling<4>(frame->mapHeight,frame->occupancyMap,frame->occupancyMapDS);
    } else {
        assert(false && "Unsupported block size for occupancy map downscaling");
    }

    // Geometry and attribute map generation //
    writePatches(frame, frame->mapHeight);



    // Background filling //
    fillBackgroundImages(frame, frame->mapHeight);

    RGB444toYUV420(frame->attributeMapL1, p_->mapWidth, frame->mapHeight);
    if (p_->doubleLayer) {
        RGB444toYUV420(frame->attributeMapL2, p_->mapWidth, frame->mapHeight);
    }

    if (p_->exportIntermediateMaps /*|| p_->useEncoderCommand*/) {
        writeFrameMapsYUV(frame);
    }
}

void MapGenerationBaseLine::writeFrameMapsYUV(const std::shared_ptr<uvgvpcc_enc::Frame>& frame) {
    
    // Occupancy maps
    std::streamsize streamSize = static_cast<std::streamsize>(static_cast<double>(frame->occupancyMap.size())*1.5);

    const std::shared_ptr<uvgvpcc_enc::GOF>& gof = frame->gof.lock();
    const std::string occupancyBitstreamFileName = gof->baseNameOccupancy + "_f" + std::to_string(frame->frameNumber) + ".yuv";
    std::ofstream yuvFile(occupancyBitstreamFileName, std::ios::binary);
    if (!yuvFile.is_open()) {
        throw std::runtime_error("Unable to open file: " + gof->baseNameOccupancy + "_f" + std::to_string(frame->frameNumber) + ".yuv");
    }
    
    // Convert Y0-Y1 green map into a more human friendly black and gray map
    std::vector<uint8_t> occupancyMapRecolored(static_cast<size_t>(static_cast<double>(frame->occupancyMap.size())*1.5),128);
    for (size_t i = 0; i < frame->occupancyMap.size(); ++i) {
        occupancyMapRecolored[i] = 164 * frame->occupancyMap[i];
    }
    
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) : lf Accepted for I/O operations
    yuvFile.write(reinterpret_cast<const char*>(occupancyMapRecolored.data()), streamSize);
    yuvFile.close();

    // Occupancy maps DS
    streamSize = static_cast<std::streamsize>(frame->occupancyMapDS.size());
    const std::string occupancyBitstreamFileNameDS = gof->baseNameOccupancyDS + "_f" + std::to_string(frame->frameNumber) + ".yuv";
    yuvFile = std::ofstream(occupancyBitstreamFileNameDS, std::ios::binary);
    
    if (!yuvFile.is_open()) {
        throw std::runtime_error("Unable to open file: " + occupancyBitstreamFileNameDS);
    }
    
    // Convert Y0-Y1 green map into a more human friendly black and gray map
    std::vector<uint8_t> occupancyMapRecoloredDS(frame->occupancyMapDS.size(),128);
    for (size_t i = 0; i < static_cast<size_t>(static_cast<double>(frame->occupancyMapDS.size())/1.5); ++i) {
        occupancyMapRecoloredDS[i] = 164 * frame->occupancyMapDS[i];
    }
    
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) : lf Accepted for I/O operations
    yuvFile.write(reinterpret_cast<const char*>(occupancyMapRecoloredDS.data()), streamSize);
    yuvFile.close();



    // Geometry maps
    streamSize = static_cast<std::streamsize>(frame->geometryMapL1.size());
    const std::string geometryBitstreamFileName = gof->baseNameGeometry + "_f" + std::to_string(frame->frameNumber) + ".yuv";
    yuvFile = std::ofstream(geometryBitstreamFileName, std::ios::binary);
    if (!yuvFile.is_open()) {
        throw std::runtime_error("Unable to open file: " + gof->baseNameGeometry + "_f" + std::to_string(frame->frameNumber) + ".yuv");
    }
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) : Accepted for I/O operations
    yuvFile.write(reinterpret_cast<const char*>(frame->geometryMapL1.data()), streamSize);
    if (p_->doubleLayer) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) : Accepted for I/O operations
        yuvFile.write(reinterpret_cast<const char*>(frame->geometryMapL2.data()), streamSize);
    }
    yuvFile.close();

    // Attribute maps
    streamSize = static_cast<std::streamsize>(frame->attributeMapL1.size());

    const std::string attributeBitstreamFileName = gof->baseNameAttribute + "_f" + std::to_string(frame->frameNumber) + ".yuv";
    yuvFile = std::ofstream(attributeBitstreamFileName, std::ios::binary);
    if (!yuvFile.is_open()) {
        throw std::runtime_error("Unable to open file: " + gof->baseNameAttribute + "_f" + std::to_string(frame->frameNumber) + ".yuv");
    }
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) : Accepted for I/O operations
    yuvFile.write(reinterpret_cast<const char*>(frame->attributeMapL1.data()), streamSize);
    if (p_->doubleLayer) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) : Accepted for I/O operations
        yuvFile.write(reinterpret_cast<const char*>(frame->attributeMapL2.data()), streamSize);
    }
    yuvFile.close();
}

// TODO(lf): for L2, find a way to make a copy write only the changing value between both map (same comment for geometry)
// TODO(lf): we first do YUV420 for all maps, but we might consider YUV400 for geometry and occupancy if Kvazaar can handle it and if the
// decoder can handle it too. TODO(lf): allocate all the maps of the GOF in one memory allocation ?
void MapGenerationBaseLine::initGOFMapGeneration(const std::shared_ptr<uvgvpcc_enc::GOF>& gof) {
    uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::TRACE, "MAP GENERATION", "Initialize maps of GOF " + std::to_string(gof->gofId) + ".\n");

    for (const std::shared_ptr<uvgvpcc_enc::Frame>& frame : gof->frames) {
        gof->mapHeightDSGOF = std::max(gof->mapHeightDSGOF, frame->mapHeightDS);
    }

    gof->mapHeightDSGOF = roundUp(gof->mapHeightDSGOF, static_cast<size_t>(8));

    gof->mapHeightGOF = gof->mapHeightDSGOF * p_->occupancyMapDSResolution;

    // Complete GOF file names
    gof->completeFileBaseNames(p_);

    for (const std::shared_ptr<uvgvpcc_enc::Frame>& frame : gof->frames) {
        frame->mapHeight = gof->mapHeightGOF;
    }
}

