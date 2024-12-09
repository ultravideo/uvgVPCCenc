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

#include "mapGenerationBaseLine.hpp"

#include <algorithm>
#include <array>
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

void MapGenerationBaseLine::writePatch(const uvgvpcc_enc::Patch& patch, const size_t& imageSize, uvgvpcc_enc::Frame& frame) {
    for (size_t v = 0; v < patch.heightInPixel_; ++v) {
        for (size_t u = 0; u < patch.widthInPixel_; ++u) {
            size_t const patchPos = u + v * patch.widthInPixel_;
            size_t const mapPos =
                patch.omPosX_ * p_->occupancyMapResolution + u + (patch.omPosY_ * p_->occupancyMapResolution + v) * p_->mapWidth;
            if (patch.depthL1_[patchPos] != g_infiniteDepth) {
                frame.geometryMapL1[mapPos] = patch.depthL1_[patchPos];
                frame.attributeMapL1[mapPos] = frame.pointsAttribute[patch.depthPCidxL1_[patchPos]][0];
                frame.attributeMapL1[mapPos + imageSize] = frame.pointsAttribute[patch.depthPCidxL1_[patchPos]][1];
                frame.attributeMapL1[mapPos + 2 * imageSize] = frame.pointsAttribute[patch.depthPCidxL1_[patchPos]][2];

                // TODO(lf)create a const variable for the point and then access all three colors
                if (p_->doubleLayer) {
                    frame.geometryMapL2[mapPos] = patch.depthL2_[patchPos];
                    frame.attributeMapL2[mapPos] = frame.pointsAttribute[patch.depthPCidxL2_[patchPos]][0];
                    frame.attributeMapL2[mapPos + imageSize] = frame.pointsAttribute[patch.depthPCidxL2_[patchPos]][1];
                    frame.attributeMapL2[mapPos + 2 * imageSize] = frame.pointsAttribute[patch.depthPCidxL2_[patchPos]][2];
                }
            }
        }
    }
}

void MapGenerationBaseLine::writePatchAxisSwap(const uvgvpcc_enc::Patch& patch, const size_t& imageSize, uvgvpcc_enc::Frame& frame) {
    for (size_t u = 0; u < patch.widthInPixel_; ++u) {
        for (size_t v = 0; v < patch.heightInPixel_; ++v) {
            size_t const patchPos = u + v * patch.widthInPixel_;
            size_t const mapPos =
                patch.omPosX_ * p_->occupancyMapResolution + v + (patch.omPosY_ * p_->occupancyMapResolution + u) * p_->mapWidth;

            if (patch.depthL1_[patchPos] != g_infiniteDepth) {
                frame.geometryMapL1[mapPos] = patch.depthL1_[patchPos];
                frame.attributeMapL1[mapPos] = frame.pointsAttribute[patch.depthPCidxL1_[patchPos]][0];
                frame.attributeMapL1[mapPos + imageSize] = frame.pointsAttribute[patch.depthPCidxL1_[patchPos]][1];
                frame.attributeMapL1[mapPos + 2 * imageSize] = frame.pointsAttribute[patch.depthPCidxL1_[patchPos]][2];

                if (p_->doubleLayer) {
                    frame.geometryMapL2[mapPos] = patch.depthL2_[patchPos];
                    frame.attributeMapL2[mapPos] = frame.pointsAttribute[patch.depthPCidxL2_[patchPos]][0];
                    frame.attributeMapL2[mapPos + imageSize] = frame.pointsAttribute[patch.depthPCidxL2_[patchPos]][1];
                    frame.attributeMapL2[mapPos + 2 * imageSize] = frame.pointsAttribute[patch.depthPCidxL2_[patchPos]][2];
                }
            }
        }
    }
}

// TODO(lf)create a const variable for the point and then access all three colors
// TODO(lf): rename into writePatches
void MapGenerationBaseLine::mapsGeneration(uvgvpcc_enc::Frame& frame, const size_t& gofMapsHeight) {
    const size_t imageSize = p_->mapWidth * gofMapsHeight;
    for (const uvgvpcc_enc::Patch& patch : frame.patchList) {
        if (!patch.axisSwap_) {
            writePatch(patch, imageSize, frame);
        } else {
            writePatchAxisSwap(patch, imageSize, frame);
        }
    }
}

void MapGenerationBaseLine::allocateMaps(uvgvpcc_enc::Frame& frame, const size_t& gofMapsHeight) {
    // Notice that before this operation, the dimension of each frame occupancy map can be different. Thus, this OM resizing operation both
    // makes all GOF occupancy maps dimension uniform and convert them to YUV420. FYI, U and V images of the occupancy and geometry maps are
    // empty/not used by the decoder/do not cary any usefull information.

    const size_t imageSize = p_->mapWidth * gofMapsHeight;
    const size_t imageSizeOM = p_->mapWidth / p_->occupancyMapResolution * gofMapsHeight / p_->occupancyMapResolution;

    // The occupancy map already exist and might alreday has the correct size.
    const size_t occMapSize = imageSizeOM + (imageSizeOM >> 1U);
    if (frame.occupancyMap.size() != occMapSize) {
        frame.occupancyMap.resize(occMapSize, 0);  // TODO(lf): use an offset, as it is possible in the norm V-PCC I guess
    }

    frame.geometryMapL1.resize(imageSize + (imageSize >> 1U), p_->mapGenerationBackgroundValueGeometry);
    frame.attributeMapL1.resize(static_cast<size_t>(imageSize) * 3, p_->mapGenerationBackgroundValueAttribute);
    // TODO(lf): what is the justification for the max value ?

    if (p_->doubleLayer) {
        frame.geometryMapL2.resize(imageSize + (imageSize >> 1U), p_->mapGenerationBackgroundValueGeometry);
        frame.attributeMapL2.resize(static_cast<size_t>(imageSize) * 3, p_->mapGenerationBackgroundValueAttribute);
    }
}

// TODO(lf): try with integer only
void MapGenerationBaseLine::RGB444toYUV420(std::vector<uint8_t>& img, const size_t& width, const size_t& height) {
    uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::TRACE, "MapGenerationBaseLine", "RGB444toYUV420\n");

    const size_t imageSize = width * height;
    const size_t imageSizeUV = imageSize >> 2U;
    std::vector<uint8_t> uv420(imageSizeUV * 2);

    size_t idxU = 0;
    for (size_t y = 0; y < height; y += 2) {
        for (size_t x = 0; x < width; x += 2) {
            const size_t idxTL = x + y * width;
            const size_t idxTR = idxTL + 1;
            const size_t idxBL = idxTL + width;
            const size_t idxBR = idxBL + 1;

            float R = static_cast<float>(img[idxTL]);
            float G = static_cast<float>(img[idxTL + imageSize]);
            float B = static_cast<float>(img[idxTL + imageSize * 2]);
            img[idxTL] = static_cast<uint8_t>((0.212600F * R + 0.715200F * G + 0.072200F * B));

            float uvR = R;
            float uvG = G;
            float uvB = B;

            R = static_cast<float>(img[idxTR]);
            G = static_cast<float>(img[idxTR + imageSize]);
            B = static_cast<float>(img[idxTR + imageSize * 2]);
            img[idxTR] = static_cast<uint8_t>((0.212600F * R + 0.715200F * G + 0.072200F * B));

            uvR += R;
            uvG += G;
            uvB += B;

            R = static_cast<float>(img[idxBL]);
            G = static_cast<float>(img[idxBL + imageSize]);
            B = static_cast<float>(img[idxBL + imageSize * 2]);
            img[idxBL] = static_cast<uint8_t>((0.212600F * R + 0.715200F * G + 0.072200F * B));

            uvR += R;
            uvG += G;
            uvB += B;

            R = static_cast<float>(img[idxBR]);
            G = static_cast<float>(img[idxBR + imageSize]);
            B = static_cast<float>(img[idxBR + imageSize * 2]);
            img[idxBR] = static_cast<uint8_t>((0.212600F * R + 0.715200F * G + 0.072200F * B));

            uvR += R;
            uvG += G;
            uvB += B;

            uv420[idxU] = static_cast<uint8_t>(-0.028643F * uvR - 0.096357F * uvG + 0.125F * uvB + 128.F);
            uv420[idxU + imageSizeUV] = static_cast<uint8_t>(0.125F * uvR - 0.11353825F * uvG - 0.01146175F * uvB + 128.F);
            ++idxU;
        }
    }

    img.resize(imageSize + imageSizeUV * 2);
    img.shrink_to_fit();  // TODO(lf): lf wonder if elegant or useful behaviour
    std::copy(uv420.begin(), uv420.end(), &img[imageSize]);
}


// TODO(lf): use copy with relevant optimal memory copy to fill second layer. Tackle the cognitive complexity accordingly
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void MapGenerationBaseLine::fillBackgroundEmptyBlock(uvgvpcc_enc::Frame& frame, const size_t blockSize, const size_t imageSize,
                                                     const size_t uBlk, const size_t vBlk, const size_t uom,
                                                     const size_t vom) {
    if (uBlk > 0) {
        for (size_t j = 0; j < blockSize; ++j) {
            const size_t currentY = vom + j;
            for (size_t i = 0; i < blockSize; ++i) {
                const size_t currentPos = uom + i + currentY * p_->mapWidth;
                const size_t previousPos = uom + i - 1 + currentY * p_->mapWidth;  // pixel on left

                frame.geometryMapL1[currentPos] = frame.geometryMapL1[previousPos];

                frame.attributeMapL1[currentPos] = frame.attributeMapL1[previousPos];
                frame.attributeMapL1[currentPos + imageSize] = frame.attributeMapL1[previousPos + imageSize];
                frame.attributeMapL1[currentPos + 2 * imageSize] = frame.attributeMapL1[previousPos + 2 * imageSize];

                if (p_->doubleLayer) {
                    frame.geometryMapL2[currentPos] = frame.geometryMapL1[currentPos];
                    frame.attributeMapL2[currentPos] = frame.attributeMapL1[currentPos];
                    frame.attributeMapL2[currentPos + imageSize] = frame.attributeMapL1[currentPos + imageSize];
                    frame.attributeMapL2[currentPos + 2 * imageSize] = frame.attributeMapL1[currentPos + 2 * imageSize];
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

                frame.geometryMapL1[currentPos] = frame.geometryMapL1[previousPos];
                frame.attributeMapL1[currentPos] = frame.attributeMapL1[previousPos];
                frame.attributeMapL1[currentPos + imageSize] = frame.attributeMapL1[previousPos + imageSize];
                frame.attributeMapL1[currentPos + 2 * imageSize] = frame.attributeMapL1[previousPos + 2 * imageSize];

                if (p_->doubleLayer) {
                    frame.geometryMapL2[currentPos] = frame.geometryMapL1[currentPos];
                    frame.attributeMapL2[currentPos] = frame.attributeMapL1[currentPos];
                    frame.attributeMapL2[currentPos + imageSize] = frame.attributeMapL1[currentPos + imageSize];
                    frame.attributeMapL2[currentPos + 2 * imageSize] = frame.attributeMapL1[currentPos + 2 * imageSize];
                }
            }
        }
    } else {
        // lf : In TMC2 the top left block, if it is an empty block, keep the default value. Here, we put the middle (mid-gray)
        for (size_t j = 0; j < blockSize; ++j) {
            for (size_t i = 0; i < blockSize; ++i) {
                const size_t currentPos = uom + i + (vom + j) * p_->mapWidth;
                const size_t fillingValue = 128;

                frame.geometryMapL1[currentPos] = fillingValue;
                frame.attributeMapL1[currentPos] = fillingValue;
                frame.attributeMapL1[currentPos + imageSize] = fillingValue;
                frame.attributeMapL1[currentPos + 2 * imageSize] = fillingValue;

                if (p_->doubleLayer) {
                    frame.geometryMapL2[currentPos] = fillingValue;
                    frame.attributeMapL2[currentPos] = fillingValue;
                    frame.attributeMapL2[currentPos + imageSize] = fillingValue;
                    frame.attributeMapL2[currentPos + 2 * imageSize] = fillingValue;
                }
            }
        }
    }
}

void MapGenerationBaseLine::updateSums(uvgvpcc_enc::Frame& frame, const size_t blockLeft, const size_t blockTop,
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
            sumGeo[currentPosBlk] += frame.geometryMapL1[currentPosOM];

            sumR[currentPosBlk] += frame.attributeMapL1[currentPosOM];
            sumG[currentPosBlk] += frame.attributeMapL1[currentPosOM + imageSize];
            sumB[currentPosBlk] += frame.attributeMapL1[currentPosOM + 2 * imageSize];
            ++count[currentPosBlk];
        }
    }
}

void MapGenerationBaseLine::fillBackgroundNonEmptyBlock(uvgvpcc_enc::Frame& frame, const size_t blockSize, const size_t imageSize,
                                                        const size_t uom, const size_t vom, const size_t pixelBlockCount,
                                                        size_t missingPixelCount, std::vector<size_t>& iterations) {
    // lf : knowing that we should not used a occupancyMapResolution (precision?) higher than 4 (probably 1(no downscalling) or 2), the
    // current algorithm seems overkill. Simple use of lookup table can do it I think.

    // lf : WARNING : what happen if the true value of the geometry is the uint8 max value ?

    // lf : optimization possible -> different result, but less complex -> if there are missing pixels. Do four "passes", in each
    // direction (up to down, down to top, left to right, right to left). The 'values' for each passe will just be the one before. And
    // the count will be incremented if there is a value before. lf : the same optimization can be used using the same iteration
    // scheme. (stop propagation in a line or column after reaching one missing pixel) lf : for each pixel of the focused block
    // (uBlk,vBlk), if it is an empty pixel (no value in the occupancyMap  (lf OPT : check in the geometry map instead (seems to be
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

                    frame.geometryMapL1[currentPosOM] =
                        static_cast<uint8_t>((sumGeo[pixelPos] + count[pixelPos] / 2) / count[pixelPos]);

                    // TODO(lf): in TMC2 both map are doing it separately. Here, as a temporary solution, we do it only on L1

                    frame.attributeMapL1[currentPosOM] =
                        static_cast<uint8_t>((sumR[pixelPos] + count[pixelPos] / 2) / count[pixelPos]);
                    frame.attributeMapL1[currentPosOM + imageSize] =
                        static_cast<uint8_t>((sumG[pixelPos] + count[pixelPos] / 2) / count[pixelPos]);
                    frame.attributeMapL1[currentPosOM + 2 * imageSize] =
                        static_cast<uint8_t>((sumB[pixelPos] + count[pixelPos] / 2) / count[pixelPos]);

                    if (p_->doubleLayer) {
                        frame.geometryMapL2[currentPosOM] = frame.geometryMapL1[currentPosOM];
                        frame.attributeMapL2[currentPosOM] = frame.attributeMapL1[currentPosOM];
                        frame.attributeMapL2[currentPosOM + imageSize] = frame.attributeMapL1[currentPosOM + imageSize];
                        frame.attributeMapL2[currentPosOM + 2 * imageSize] = frame.attributeMapL1[currentPosOM + 2 * imageSize];
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

void MapGenerationBaseLine::fillBackgroundImages(uvgvpcc_enc::Frame& frame, const size_t& gofMapsHeight) {
    const size_t blockSize = p_->occupancyMapResolution;
    const size_t occupancyMapWidthBlk =
        p_->mapWidth / blockSize;  // TODO(lf): this should be a frame param (yes for the height, and a static param for the width)
    const size_t occupancyMapHeightBlk = gofMapsHeight / blockSize;  // what is the difference with occupancyImage.getHeight() ??
    const size_t imageSize = p_->mapWidth * gofMapsHeight;
    const size_t pixelBlockCount = blockSize * blockSize;  // lf nb of pixel per block from the frameOM POV

    // If p_->mapGenerationFillEmptyBlock == false, we need still need to fill the non-empty CTU. (A CTU with at least one OM block is
    // non-empty.) A CTU is 64x64. If a CTU is empty, we can skip it (that is, keep the uniform gray background). TODO(lf): make this ctu check

    // iterate over each block of the occupancy map
    for (size_t vBlk = 0; vBlk < occupancyMapHeightBlk; ++vBlk) {
        const size_t vom = vBlk * blockSize;
        for (size_t uBlk = 0; uBlk < occupancyMapWidthBlk; ++uBlk) {
            const size_t uom = uBlk * blockSize;

            // empty block -> copy the value of previous block (one of the TMC2 solution) or do nothing (let the uniform value set during map
            // allocation)
            if (frame.occupancyMap[uBlk + vBlk * occupancyMapWidthBlk] == 0U) {
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
                    if (frame.geometryMapL1[currentPosOM] == p_->mapGenerationBackgroundValueGeometry) {
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

            // one or more pixel need a value. Those pixel will appear in the decoded point cloud. They result from the downscalling of the
            // occupancy map. -> Compute average value from neigboring pixels
            fillBackgroundNonEmptyBlock(frame, blockSize, imageSize, uom, vom, pixelBlockCount, missingPixelCount, iterations);
        }
    }
}

void MapGenerationBaseLine::generateFrameMaps(std::shared_ptr<uvgvpcc_enc::Frame>& frame) {
    allocateMaps(*frame, frame->mapsHeight);

    // Geometry and attribute map generation //
    mapsGeneration(*frame, frame->mapsHeight);

    // Background filling //
    fillBackgroundImages(*frame, frame->mapsHeight);

    RGB444toYUV420(frame->attributeMapL1, p_->mapWidth, frame->mapsHeight);
    if (p_->doubleLayer) {
        RGB444toYUV420(frame->attributeMapL2, p_->mapWidth, frame->mapsHeight);
    }

    if (p_->exportIntermediateMaps /*|| p_->useEncoderCommand*/) {
        writeFrameMapsYUV(frame);
    }
}

void MapGenerationBaseLine::writeFrameMapsYUV(std::shared_ptr<uvgvpcc_enc::Frame>& frame) {
    
    
    // Occupancy maps
    std::streamsize streamSize = static_cast<std::streamsize>(frame->occupancyMap.size());

    const std::shared_ptr<uvgvpcc_enc::GOF> gof = frame->gof.lock();
    const std::string occupancyBitstreamFileName = gof->baseNameOccupancy + "_f" + std::to_string(frame->frameNumber) + ".yuv";
    std::ofstream yuvFile(occupancyBitstreamFileName, std::ios::binary);
    if (!yuvFile.is_open()) {
        throw std::runtime_error("Unable to open file: " + gof->baseNameOccupancy + "_f" + std::to_string(frame->frameNumber) + ".yuv");
    }
    
    // Convert Y0-Y1 green map into a more human friendly black and gray map
    std::vector<uint8_t> occupancyMapRecolored(frame->occupancyMap.size(),128);
    for (size_t i = 0; i < static_cast<size_t>(static_cast<double>(frame->occupancyMap.size())/1.5); ++i) {
        occupancyMapRecolored[i] = 164 * frame->occupancyMap[i];
    }
    
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) : lf Accepted for I/O operations
    yuvFile.write(reinterpret_cast<const char*>(occupancyMapRecolored.data()), streamSize);
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
void MapGenerationBaseLine::initGOFMapGeneration(std::shared_ptr<uvgvpcc_enc::GOF>& gof) {
    uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::TRACE, "MAP GENERATION", "Initialize maps of GOF " + std::to_string(gof->gofId) + ".\n");

    for (const std::shared_ptr<uvgvpcc_enc::Frame>& frame : gof->frames) {
        gof->occupancyMapHeight = std::max(gof->occupancyMapHeight, frame->occupancyMapHeight);
    }

    gof->occupancyMapHeight = roundUp(gof->occupancyMapHeight, static_cast<size_t>(8));

    gof->mapsHeight = gof->occupancyMapHeight * p_->occupancyMapResolution;

    // Complete GOF file names
    gof->completeFileBaseNames(p_);

    for (const std::shared_ptr<uvgvpcc_enc::Frame>& frame : gof->frames) {
        frame->mapsHeight = gof->mapsHeight;
    }
}
