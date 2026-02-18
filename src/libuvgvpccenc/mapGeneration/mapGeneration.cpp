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

/// \file Entry point for the map generation process. Use the 2D location of the patch obtained during patch packing to create the occupancy,
/// geometry and attribute 2D maps.

#include "mapGeneration.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "bgFillAttribute.hpp"
#include "bgFillGeometry.hpp"
#include "utils/fileExport.hpp"
#include "utils/parameters.hpp"
#include "uvgutils/log.hpp"
#include "uvgutils/utils.hpp"
#include "uvgvpcc/uvgvpcc.hpp"

using namespace uvgvpcc_enc;

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

                const uint8_t sum = blockPtr[0] + blockPtr[1] + blockPtr[mapWidth] + blockPtr[mapWidth + 1];

                if (sum >= p_->omRefinementTreshold2) {
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

                const uint8_t sum = blockPtr[0] + blockPtr[1] + blockPtr[2] + blockPtr[3] + blockPtr[mapWidth] + blockPtr[mapWidth + 1] +
                                    blockPtr[mapWidth + 2] + blockPtr[mapWidth + 3] + blockPtr[2 * mapWidth] + blockPtr[2 * mapWidth + 1] +
                                    blockPtr[2 * mapWidth + 2] + blockPtr[2 * mapWidth + 3] + blockPtr[3 * mapWidth] +
                                    blockPtr[3 * mapWidth + 1] + blockPtr[3 * mapWidth + 2] + blockPtr[3 * mapWidth + 3];

                if (sum >= p_->omRefinementTreshold4) {
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
void writePatchT(const uvgvpcc_enc::Patch& patch, const size_t& imageSize, const std::shared_ptr<uvgvpcc_enc::Frame>& frame) {
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

void writePatches(const std::shared_ptr<uvgvpcc_enc::Frame>& frame, const size_t& gofMapsHeight) {
    const size_t imageSize = p_->mapWidth * gofMapsHeight;

    if (p_->doubleLayer) {
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
    if (p_->exportIntermediateFiles) {
        FileExport::exportImageAttribute(frame);
        FileExport::exportImageGeometry(frame);
    }
}

void allocateMaps(const std::shared_ptr<uvgvpcc_enc::Frame>& frame, const size_t& gofMapsHeight) {
    // Notice that before this operation, the dimension of each frame occupancy map can be different. Thus, this OM resizing operation both
    // makes all GOF occupancy maps dimension uniform and convert them to YUV420. FYI, U and V images of the occupancy and geometry maps are
    // empty/not used by the decoder/do not cary any usefull information.

    const size_t imageSize = p_->mapWidth * gofMapsHeight;
    
    if (p_->dynamicMapHeight) {
        assert(gofMapsHeight == p_->minimumMapHeight);
    }

    // The occupancy map already exist and might already have the correct size.

    // TODO(lf): is it necessary ? Yes if resizing due to bigger occupancy map (larger than minimumHeight parameter)
    // assert(frame->occupancyMap.size() <= imageSize); //TODO(lf) there is a bug here
    if (frame->occupancyMap.size() != imageSize) {
        frame->occupancyMap.resize(imageSize, 0);
    }

    if (p_->exportIntermediateFiles) {
        FileExport::exportImageOccupancy(frame);
    }

    const size_t imageSizeDS = imageSize / (p_->occupancyMapDSResolution * p_->occupancyMapDSResolution);
    frame->occupancyMapDS.resize(imageSizeDS + (imageSizeDS >> 1U), 0U);  // TODO(lf): should be done at the down scaling function

    frame->geometryMapL1.resize(imageSize + (imageSize >> 1U), p_->mapGenerationBackgroundValueGeometry);
    frame->attributeMapL1.resize(static_cast<size_t>(imageSize) * 3, p_->mapGenerationBackgroundValueAttribute);
    // TODO(lf): what is the justification for the max value ?

    if (p_->doubleLayer) {
        frame->geometryMapL2.resize(imageSize + (imageSize >> 1U), p_->mapGenerationBackgroundValueGeometry);
        frame->attributeMapL2.resize(static_cast<size_t>(imageSize) * 3, p_->mapGenerationBackgroundValueAttribute);
    }
}

// TODO(lf): Why an integer only implementation is so bad in term of quality degradation? -> Probably because of PCQM
void RGB444toYUV420(std::vector<uint8_t>& img, const size_t& width, const size_t& height) {
    uvgutils::Logger::log<uvgutils::LogLevel::TRACE>("MapGeneration", "RGB444toYUV420\n");

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

// NOLINTBEGIN(readability-avoid-nested-conditional-operator)
inline float fMin(float a, float b) { return ((a) < (b)) ? (a) : (b); }
inline float fMax(float a, float b) { return ((a) > (b)) ? (a) : (b); }
inline float fClip(float x, float low, float high) { return fMin(fMax(x, low), high); }
int clamp(int v, int a, int b) { return ((v < a) ? a : ((v > b) ? b : v)); }
double clamp(double v, double a, double b) { return ((v < a) ? a : ((v > b) ? b : v)); }
// NOLINTEND(readability-avoid-nested-conditional-operator)

void RGBtoFloatRGB(const std::vector<uint8_t>& src, std::array<std::vector<float>, 3>& RGB444, std::size_t width, std::size_t height) {
    const std::size_t imageSize = width * height;
    const float maxValue = 255.F;
    RGB444[0].resize(imageSize);
    RGB444[1].resize(imageSize);
    RGB444[2].resize(imageSize);
    for (std::size_t i = 0; i < imageSize; ++i) {
        RGB444[0][i] = static_cast<float>(src[i]) / maxValue;
        RGB444[1][i] = static_cast<float>(src[i + imageSize]) / maxValue;
        RGB444[2][i] = static_cast<float>(src[i + 2 * imageSize]) / maxValue;
    }
}

void convertRGBToYUV(const std::vector<float>& R, const std::vector<float>& G, const std::vector<float>& B, std::vector<float>& Y,
                     std::vector<float>& U, std::vector<float>& V) {
    std::size_t const count = R.size();
    Y.resize(count);
    U.resize(count);
    V.resize(count);
    for (std::size_t i = 0; i < count; i++) {
        Y[i] = static_cast<float>(clamp(0.212600 * R[i] + 0.715200 * G[i] + 0.072200 * B[i], 0.0, 1.0));
        U[i] = static_cast<float>(clamp(-0.114572 * R[i] - 0.385428 * G[i] + 0.500000 * B[i], -0.5, 0.5));
        V[i] = static_cast<float>(clamp(0.500000 * R[i] - 0.454153 * G[i] - 0.045847 * B[i], -0.5, 0.5));
    }
}

void floatYUVToYUV(const std::vector<float>& luma, const std::vector<float>& cb, const std::vector<float>& cr, std::vector<uint8_t>& dst,
                   std::size_t width, std::size_t height) {
    const std::size_t imageSize = width * height;
    dst.resize(imageSize + (imageSize >> 1U));
    const double scale = 255.;

    // Luma //
    double offset = 0;
    for (std::size_t i = 0; i < imageSize; ++i) {
        dst[i] = static_cast<uint8_t>(
            fClip(std::round(static_cast<float>(scale * static_cast<double>(luma[i]) + offset)), 0.F, static_cast<float>(scale)));
    }

    // Chroma //
    offset = 128.;
    for (std::size_t i = 0; i < imageSize / 4; ++i) {
        dst[imageSize + i] = static_cast<uint8_t>(
            fClip(std::round(static_cast<float>(scale * static_cast<double>(cb[i]) + offset)), 0.F, static_cast<float>(scale)));
        dst[imageSize + imageSize / 4 + i] = static_cast<uint8_t>(
            fClip(std::round(static_cast<float>(scale * static_cast<double>(cr[i]) + offset)), 0.F, static_cast<float>(scale)));
    }
}

// Horizontal filter used by TMC2 (only the filter of index 4 (4 DF_GS) is used in TMC2 (in our case))
constexpr std::array<float, 15> g_filter444to420_horizontal = {
    static_cast<float>(-0.01716352771649 * 512), static_cast<float>(0.0),
    static_cast<float>(+0.04066666714886 * 512), static_cast<float>(0.0),
    static_cast<float>(-0.09154810319329 * 512), static_cast<float>(0.0),
    static_cast<float>(0.31577823859943 * 512),  static_cast<float>(0.50453345032298 * 512),
    static_cast<float>(0.31577823859943 * 512),  static_cast<float>(0.0),
    static_cast<float>(-0.09154810319329 * 512), static_cast<float>(0.0),
    static_cast<float>(0.04066666714886 * 512),  static_cast<float>(0.0),
    static_cast<float>(-0.01716352771649 * 512)};

constexpr double g_filter444to420_horizontal_shift = 9.0;

inline float downsamplingHorizontal(const std::vector<float>& img, const std::size_t width, const std::size_t i0, const std::size_t j0) {
    const float scale = 1.0F / (static_cast<float>(1U << (static_cast<std::size_t>(g_filter444to420_horizontal_shift))));
    const float offset = 0.00000000;
    const std::size_t position = static_cast<std::size_t>(g_filter444to420_horizontal.size() - 1) >> 1U;
    double value = 0;
    for (std::size_t j = 0; j < static_cast<std::size_t>(g_filter444to420_horizontal.size()); j++) {
        value +=
            static_cast<double>(g_filter444to420_horizontal[j]) *
            static_cast<double>(img[static_cast<std::size_t>(i0 * width) +
                                    static_cast<std::size_t>(clamp(static_cast<int>(j0 + j - position), 0, static_cast<int>(width - 1)))]);
    }
    return static_cast<float>((value + static_cast<double>(offset)) * static_cast<double>(scale));
}

// Vertical filter used by TMC2 (only the filter of index 4 (4 DF_GS) is used in TMC2 (in our case))
constexpr std::array<float, 16> g_filter444to420_vertical = {
    static_cast<float>(-0.00945406160902 * 512), static_cast<float>(-0.01539537217249 * 512), static_cast<float>(0.02360533018213 * 512),
    static_cast<float>(0.03519540819902 * 512),  static_cast<float>(-0.05254456550808 * 512), static_cast<float>(-0.08189331229717 * 512),
    static_cast<float>(0.14630826357715 * 512),  static_cast<float>(0.45417830962846 * 512),  static_cast<float>(0.45417830962846 * 512),
    static_cast<float>(0.14630826357715 * 512),  static_cast<float>(-0.08189331229717 * 512), static_cast<float>(-0.05254456550808 * 512),
    static_cast<float>(0.03519540819902 * 512),  static_cast<float>(0.02360533018213 * 512),  static_cast<float>(-0.01539537217249 * 512),
    static_cast<float>(-0.00945406160902 * 512)};

constexpr double g_filter444to420_vertical_shift = 9.0;

inline float downsamplingVertical(const std::vector<float>& img, const std::size_t width, const std::size_t height, const std::size_t i0,
                                  const std::size_t j0) {
    const float offset = 0;
    const float scale = 1.F / (static_cast<float>(1U << (static_cast<std::size_t>(g_filter444to420_vertical_shift))));
    const std::size_t position = static_cast<std::size_t>(g_filter444to420_vertical.size() - 1) >> 1U;
    double value = 0;
    for (std::size_t i = 0; i < static_cast<std::size_t>(g_filter444to420_vertical.size()); i++) {
        value += static_cast<double>(g_filter444to420_vertical[i]) *
                 static_cast<double>(img[clamp(static_cast<int>(i0 + i - position), 0, static_cast<int>(height - 1)) * width + j0]);
    }
    return static_cast<float>((value + static_cast<double>(offset)) * static_cast<double>(scale));
}

void downsampling(const std::vector<float>& chroma_in, std::vector<float>& chroma_out, const std::size_t widthIn,
                  const std::size_t heightIn) {
    std::size_t const widthOut = widthIn / 2;
    std::size_t const heightOut = heightIn / 2;
    std::vector<float> temp(static_cast<std::size_t>(widthOut) * static_cast<std::size_t>(heightIn));
    chroma_out.resize(static_cast<std::size_t>(widthOut) * static_cast<std::size_t>(heightOut));

    for (std::size_t i = 0; i < heightIn; i++) {
        for (std::size_t j = 0; j < widthOut; j++) {
            temp[i * widthOut + j] = downsamplingHorizontal(chroma_in, widthIn, i, j * 2);
        }
    }
    for (std::size_t i = 0; i < heightOut; i++) {
        for (std::size_t j = 0; j < widthOut; j++) {
            chroma_out[i * widthOut + j] = downsamplingVertical(temp, widthOut, heightIn, 2 * i, j);
        }
    }
}

void RGB444toYUV420TMC2(std::vector<uint8_t>& img, const std::size_t& width, const std::size_t& height) {
    uvgutils::Logger::log<uvgutils::LogLevel::TRACE>("MapGeneration", "RGB444toYUV420TMC2\n");

    std::array<std::vector<float>, 3> RGB444;
    std::array<std::vector<float>, 3> YUV444;
    std::array<std::vector<float>, 3> YUV420;

    RGBtoFloatRGB(img, RGB444, width, height);
    convertRGBToYUV(RGB444[0], RGB444[1], RGB444[2], YUV444[0], YUV444[1], YUV444[2]);
    downsampling(YUV444[1], YUV420[1], width, height);
    downsampling(YUV444[2], YUV420[2], width, height);
    floatYUVToYUV(YUV444[0], YUV420[1], YUV420[2], img, width, height);
}

}  // Anonymous namespace

void MapGeneration::generateFrameMaps(const std::shared_ptr<uvgvpcc_enc::Frame>& frame) {
    allocateMaps(frame, frame->mapHeight);

    // TODO(lf): occupancy map downscaling can be done after write patches (or in parallel) maybe
    if (p_->occupancyMapDSResolution == 2) {
        occupancyMapDownscaling<2>(frame->mapHeight, frame->occupancyMap, frame->occupancyMapDS);
    } else if (p_->occupancyMapDSResolution == 4) {
        occupancyMapDownscaling<4>(frame->mapHeight, frame->occupancyMap, frame->occupancyMapDS);
    } else {
        assert(false && "Unsupported downscaling factor for occupancy map.");
    }
    if (p_->exportIntermediateFiles) {
        FileExport::exportImageOccupancyDS(frame);
    }

    if(!p_->dynamicMapHeight) {
        frame->patchList.erase(
            std::remove_if(frame->patchList.begin(),
                        frame->patchList.end(),
                        [](const Patch& patch) {
                            return patch.isDiscarded;
                        }),
            frame->patchList.end());
    }

    // Geometry and attribute map generation //
    writePatches(frame, frame->mapHeight);

    // Geometry map background filling
    bgFillGeometry(frame->occupancyMapDS, frame->mapHeight, frame->geometryMapL1);
    if (p_->doubleLayer) {
        bgFillGeometry(frame->occupancyMapDS, frame->mapHeight, frame->geometryMapL2);
    }

    // Attribute map background filling
    bgFillAttribute(*frame, frame->attributeMapL1);
    if (p_->doubleLayer) {
        bgFillAttribute(*frame, frame->attributeMapL2);
    }

    if (p_->exportIntermediateFiles) {
        FileExport::exportImageAttributeBgFill(frame);
        FileExport::exportImageGeometryBgFill(frame);
    }

    // lf: BT.709 standard is used within TMC2 for RGB->YUV conversion. Notice that some PCC metrics also applied such conversion, but may
    // used different conversion standards, resulting in incorrect quality assessment. TODO(lf): Find the mention of the conversion standard
    // within the ISO norm.
    if (p_->useTmc2YuvDownscaling) {
        RGB444toYUV420TMC2(frame->attributeMapL1, p_->mapWidth, frame->mapHeight);
        if (p_->doubleLayer) {
            RGB444toYUV420TMC2(frame->attributeMapL2, p_->mapWidth, frame->mapHeight);
        }
    } else {
        RGB444toYUV420(frame->attributeMapL1, p_->mapWidth, frame->mapHeight);
        if (p_->doubleLayer) {
            RGB444toYUV420(frame->attributeMapL2, p_->mapWidth, frame->mapHeight);
        }
    }

    if (p_->exportIntermediateFiles) {
        FileExport::exportImageAttributeYUV(frame);
    }
    std::vector<uvgutils::VectorN<uint8_t, 3>>().swap(frame->pointsAttribute);  // Release memory TODO(lf):can be done early
}

// TODO(lf): for L2, find a way to make a copy write only the changing value between both map (same comment for geometry)
// TODO(lf): we first do YUV420 for all maps, but we might consider YUV400 for geometry and occupancy if Kvazaar can handle it and if the
// decoder can handle it too. TODO(lf): allocate all the maps of the GOF in one memory allocation ?
void MapGeneration::initGOFMapGeneration(const std::shared_ptr<uvgvpcc_enc::GOF>& gof) {
    uvgutils::Logger::log<uvgutils::LogLevel::TRACE>("MAP GENERATION", "Initialize maps of GOF " + std::to_string(gof->gofId) + ".\n");

    if (!p_->dynamicMapHeight) {
        gof->mapHeightGOF = gof->frames[0]->mapHeight;
        gof->mapHeightDSGOF = gof->frames[0]->mapHeightDS;
        return;
    }

    for (const std::shared_ptr<uvgvpcc_enc::Frame>& frame : gof->frames) {
        gof->mapHeightDSGOF = std::max(gof->mapHeightDSGOF, frame->mapHeightDS);
    }
    gof->mapHeightDSGOF = uvgutils::roundUp(gof->mapHeightDSGOF, static_cast<size_t>(8));
    gof->mapHeightGOF = gof->mapHeightDSGOF * p_->occupancyMapDSResolution;
    for (const std::shared_ptr<uvgvpcc_enc::Frame>& frame : gof->frames) {
        frame->mapHeight = gof->mapHeightGOF;
    }
    
}
