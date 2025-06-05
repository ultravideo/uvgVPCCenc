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

/// \file Entry point for the patch segmentation process which create the frame patch list.

#include "patchSegmentation.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "robin_hood.h"

#include "uvgvpcc/log.hpp"
#include "uvgvpcc/uvgvpcc.hpp"
#include "utils/utils.hpp"

#include "utilsPatchGeneration.hpp"

using namespace uvgvpcc_enc;

PatchSegmentation::PatchSegmentation() = default;

// TODO(lf): why the second layers resample point are not added in the resample ?
// TODO(lf): the unordered set could be a map, the value would be the patchIndex of the key point.
// TODO(lf): find a better function name
void PatchSegmentation::resampledPointcloud(robin_hood::unordered_set<size_t>& resamplePointSet, uvgvpcc_enc::Patch& patch) {
    patch.sizeD_ = 0;
    const int16_t projectionTypeIndication =
        static_cast<int16_t>(-2 * static_cast<int>(patch.projectionMode_) + 1);  // projection=0 -> 1, projection=1 -> -1
    for (size_t v = 0; v < patch.heightInPixel_; ++v) {
        for (size_t u = 0; u < patch.widthInPixel_; ++u) {
            // TODO(lf): fill the resample when creating patches ?
            const size_t pos = v * patch.widthInPixel_ + u;
            if (patch.depthL1_[pos] < g_infiniteDepth) {
                const typeGeometryInput depth0 = patch.depthL1_[pos];
                // TODO(lf): next commented lines are deprecated. Shall be replaced by better assert
                // const size_t uom = u / p_->occupancyMapDSResolution;
                // const size_t vom = v / p_->occupancyMapDSResolution;
                // assert(uom < patch.widthInOccBlk_);
                // assert(vom < patch.heightInOccBlk_);
                patch.patchOccupancyMap_[pos] = 1;

                uvgvpcc_enc::Vector3<typeGeometryInput> point;
                // TODO(lf): verify it is the right x y and z, everywhere, according to software description
                point[patch.normalAxis_] = static_cast<typeGeometryInput>(depth0);
                point[patch.tangentAxis_] =
                    static_cast<typeGeometryInput>(u + patch.posU_);  // lf : not downscalled world, real coordinate so
                point[patch.bitangentAxis_] = static_cast<typeGeometryInput>(v + patch.posV_);

                // TODO(lf): consider using emplace_hint ?
                const size_t pointLocation1D = point[0] + (point[1] << p_->geoBitDepthInput) + (point[2] << (p_->geoBitDepthInput * 2));
                resamplePointSet.emplace(pointLocation1D);

                // lf : TODO(lf): Why are those setDepth and setSizeD done here ?
                // TODO(lf): Is there a way to avoid all those intger multiplication caused by projectionTypeIndication ? (just a x1 or x-1)
                patch.depthL1_[pos] =
                    static_cast<int16_t>(projectionTypeIndication * (patch.depthL1_[pos] - static_cast<int16_t>(patch.posD_)));
                patch.sizeD_ = std::max(static_cast<int64_t>(patch.sizeD_), static_cast<int64_t>(patch.depthL1_[pos]));

                if (p_->doubleLayer) {
                    if (point[patch.normalAxis_] != patch.depthL2_[pos]) {
                        point[patch.normalAxis_] = patch.depthL2_[pos];
                        const size_t pointLocation1DDoubleLayer =
                            point[0] + (point[1] << p_->geoBitDepthInput) + (point[2] << (p_->geoBitDepthInput * 2));
                        resamplePointSet.emplace(pointLocation1DDoubleLayer);
                    }

                    patch.depthL2_[pos] =
                        static_cast<int16_t>(projectionTypeIndication * (patch.depthL2_[pos] - static_cast<int16_t>(patch.posD_)));
                    patch.sizeD_ = std::max(static_cast<int64_t>(patch.sizeD_), static_cast<int64_t>(patch.depthL2_[pos]));
                }
            }
        }
    }
}

// TODO(lf): tackle the cognitive complexity
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void PatchSegmentation::createConnectedComponents(std::vector<std::vector<size_t>>& connectedComponents, std::vector<bool>& flags,
                                                     const std::vector<size_t>& rawPoints,
                                                     //  const std::vector<std::vector<size_t>>& pointsNNList,
                                                     const std::vector<size_t>& pointsPPIs,
                                                     robin_hood::unordered_map<size_t, size_t>& nnPropagationMapFlagTrue,
                                                     const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry) {
    std::queue<size_t> fifo;
    for (const auto pointRawIndex : rawPoints) {
        if (flags[pointRawIndex]) {
            flags[pointRawIndex] = false;
            const size_t indexCC = connectedComponents.size();
            connectedComponents.emplace_back();
            std::vector<size_t>& connectedComponent = connectedComponents[indexCC];
            const size_t ppiCC = pointsPPIs[pointRawIndex];  // ppi of the connected component

            fifo.push(pointRawIndex);
            connectedComponent.push_back(pointRawIndex);
            while (!fifo.empty()) {  // MY COMMENT : neighbor by neighbor we add in the same CC the points
                const size_t pointIndex = fifo.front();
                fifo.pop();

                const typeGeometryInput gridMaxAxisValue = (1U << p_->geoBitDepthInput) - 1;
                // TODO(lf): verify this above minus 1 is correct and that it is done everywhere it is needed
                const auto& currentPoint = pointsGeometry[pointIndex];
                size_t nnCount = 0;
                const size_t distanceSearch = p_->patchSegmentationMaxPropagationDistance;
                for (size_t dist = 0; dist < distanceSearch; ++dist) {  // dist is squared distance
                    // lf : TODO(lf): why to start at 1 ? It has been fix to 0. // lf : well, it makes sense , TODO(lf): make the change back
                    for (const auto& shift : adjacentPointsSearch[dist]) {
                        uvgvpcc_enc::Vector3<typeGeometryInput> pointAdj;
                        // TODO(lf): to discuss and verify : pointAdj need to be in signed type as the shift can generate negative values.
                        // However, such negative values, in usigned type, will be higher than the max treshold (the max boundary of the
                        // grid). By using this bit overflow, we divide by two the number of check (we don't check if the shifted point is
                        // higher than 0)
                        pointAdj[0] = currentPoint[0] + shift[0];
                        pointAdj[1] = currentPoint[1] + shift[1];
                        pointAdj[2] = currentPoint[2] + shift[2];

                        // check if valid 3D coordinate (not outside the grid) //
                        if (pointAdj[0] > gridMaxAxisValue || pointAdj[1] > gridMaxAxisValue || pointAdj[2] > gridMaxAxisValue) {
                            continue;
                        }

                        const size_t pointAdjLocation1D =
                            pointAdj[0] + (pointAdj[1] << p_->geoBitDepthInput) + (pointAdj[2] << (p_->geoBitDepthInput * 2));

                        // to do, check first in a bool array. Exactly like for the raw points filling. This flag array could be built so
                        // to remove sampled point ? Be carefull of the future patch expansion.
                        if (nnPropagationMapFlagTrue.contains(pointAdjLocation1D)) {
                            const size_t neighborIndice = nnPropagationMapFlagTrue.at(pointAdjLocation1D);
                            if (ppiCC == pointsPPIs[neighborIndice]) {
                                nnPropagationMapFlagTrue.erase(pointAdjLocation1D);
                                flags[neighborIndice] = false;
                                fifo.push(neighborIndice);
                                connectedComponent.push_back(neighborIndice);
                                ++nnCount;
                            }
                        }
                        if (nnCount == p_->maxNNCountPatchSegmentation) {
                            break;
                        }
                    }
                    if (nnCount == p_->maxNNCountPatchSegmentation) {
                        break;
                    }
                }
            }
            if (connectedComponent.size() < p_->minPointCountPerCC) {
                connectedComponents.pop_back();
                // TODO(lf): One way for a point to have a flag to false at this iteration that will become true at the next one.
            }
        }
    }
}

void PatchSegmentation::patchSplitting(std::vector<size_t>& connectedComponent, uvgvpcc_enc::Patch& patch,
                                       const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry) {
    const size_t uAxis =
        patch.tangentAxis_;  // 0, 1 or 2 // TODO(lf)use point.tangentAxis or somthing like this in the vector3 custom object
    const size_t vAxis = patch.bitangentAxis_;  // 0, 1 or 2 // TODO(lf)create enum for this type
    typeGeometryInput limitU = static_cast<typeGeometryInput>(p_->maxPatchSize + patch.posU_);
    typeGeometryInput limitV = static_cast<typeGeometryInput>(p_->maxPatchSize + patch.posV_);

    std::vector<size_t> tempCC;
    tempCC.reserve(connectedComponent.size());

    for (size_t ptIndex = 0; ptIndex < connectedComponent.size(); ++ptIndex) {
        const size_t& pointIndex = connectedComponent[ptIndex];
        const uvgvpcc_enc::Vector3<typeGeometryInput>& point = pointsGeometry[pointIndex];
        

        if (point[uAxis] < limitU && point[vAxis] < limitV) {
            tempCC.push_back(pointIndex);
        }
    }

    // lf : Very specific issue : in case the shape of the patch is like a '_|', then the min max accepted area will be on the top left corner, where no points are.
    // temporary solution is to check at the bottom right corner instead.
    if(tempCC.empty()) {
        limitU = static_cast<typeGeometryInput>(patch.posU_ + patch.widthInPixel_  - p_->maxPatchSize);
        limitV = static_cast<typeGeometryInput>(patch.posV_ + patch.heightInPixel_ - p_->maxPatchSize);

        for (size_t ptIndex = 0; ptIndex < connectedComponent.size(); ++ptIndex) {
            const size_t& pointIndex = connectedComponent[ptIndex];
            const uvgvpcc_enc::Vector3<typeGeometryInput>& point = pointsGeometry[pointIndex];
            
            if (point[uAxis] > limitU && point[vAxis] > limitV) {
                tempCC.push_back(pointIndex);
            }
        }
    }


    if(tempCC.empty()) {
        // TODO(lf): overall this cutting agorithm can create very weird and bad patches. Maybe it should be totally removed
        // If both corner does not have points, we currently do not check elsewhere.
        uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::ERROR, "PATCH SEGMENTATION",
                                    "Possible infinite loop was reach. Context : A patch is bigger than the p_->maxPatchSize. So, it is split. However, the shape of the patch makes the current spliting algorithm not working. A way to solve this issue is to increase the maximum patch size parameter (p_->maxPatchSize)\n");        
    }


    // All points outside the maximum bounding box (maxPatchSize) are removed from the CC, but available for the next generation
    // std::cout << connectedComponent.size() << " --- " << tempCC.size() << std::endl;
    connectedComponent = tempCC;
    computePatchBoundingBox(patch, connectedComponent, pointsGeometry);
}

void PatchSegmentation::computePatchBoundingBox(uvgvpcc_enc::Patch& patch, const std::vector<size_t>& connectedComponent,
                                                const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry) {
    // TODO(lf): for the moment, the depth (max and min) of the patch is not computed here

    const size_t uAxis = patch.tangentAxis_;    // 0, 1 or 2
    const size_t vAxis = patch.bitangentAxis_;  // 0, 1 or 2
    typeGeometryInput minU = std::numeric_limits<typeGeometryInput>::max();
    typeGeometryInput minV = std::numeric_limits<typeGeometryInput>::max();
    typeGeometryInput maxU = 0;
    typeGeometryInput maxV = 0;

    // TODO(lf)check if we can use c_begin everywhere, and check if vector.end() should be compute before (and put in a const vairable) instead
    // of being computed at each iteration.
    for (size_t ptIndex = 0; ptIndex < connectedComponent.size(); ++ptIndex) {
        const uvgvpcc_enc::Vector3<typeGeometryInput>& point = pointsGeometry[connectedComponent[ptIndex]];
        minU = std::min(minU, point[uAxis]);
        minV = std::min(minV, point[vAxis]);
        maxU = std::max(maxU, point[uAxis]);
        maxV = std::max(maxV, point[vAxis]);
    }

    patch.posU_ = minU;
    patch.posV_ = minV;

    // old
    // TODO(lf): explain this +1 // it might be a way like this : ceil(a/b) = 1 + a/b
    // patch.widthInPixel_ = 1 + maxU - minU;
    // patch.heightInPixel_ = 1 + maxV - minV;

    // To have a size being a multiple of the OM block size avoid some check during map generation (write patch)
    // TODO(lf): remove the +1 and justify it
    patch.widthInPixel_ = roundUp(1 + maxU - minU, p_->occupancyMapDSResolution);
    patch.heightInPixel_ = roundUp(1 + maxV - minV, p_->occupancyMapDSResolution);

    patch.area_ = patch.widthInPixel_ * patch.heightInPixel_;

    // TODO(lf)change type of all patch segmentation variable and parameter
}

// TODO(lf): find better function name
void PatchSegmentation::computePatchDepthL1(uvgvpcc_enc::Patch& patch, const std::vector<size_t>& connectedComponent,
                                            std::vector<size_t>& patchPartition,
                                            const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry,
                                            const bool isProjectionMode0) {
    const size_t partitionId = patch.patchIndex_ + 1;  // TODO(lf)explain why this +1
    typeGeometryInput minD = isProjectionMode0 ? g_infiniteDepth : 0;
    size_t sizeUom = 0;
    size_t sizeVom = 0;
    size_t size2DXInPixel = 0;
    size_t size2DYInPixel = 0;

    for (size_t ptIndex = 0; ptIndex < connectedComponent.size(); ++ptIndex) {
        const size_t& pointIndex = connectedComponent[ptIndex];
        patchPartition[pointIndex] = partitionId;  // TODO(lf): why is it done here and not after knowing it is a validate point ? Also, is it
                                                   // reversed during depth filter ? Same thing for L2
        const uvgvpcc_enc::Vector3<typeGeometryInput>& point = pointsGeometry[pointIndex];
        const typeGeometryInput d = static_cast<typeGeometryInput>(point[patch.normalAxis_]);
        const size_t u = static_cast<size_t>(point[patch.tangentAxis_] - patch.posU_);
        const size_t v = static_cast<size_t>(point[patch.bitangentAxis_] - patch.posV_);
        const size_t p = v * patch.widthInPixel_ + u;
        const typeGeometryInput patchD = patch.depthL1_[p];

        // Valid points define the patch depth //
        // Points of the same patch can be located on the same position. Depending on the patch projection mode, the further or the nearer
        // point will be keep. //
        if (isProjectionMode0) {
            if (patchD <= d) {
                continue;  // non valid point
            }
            // valid point //
            if (d < minD) {
                minD = static_cast<typeGeometryInput>((d / p_->minLevel) * p_->minLevel);
            }
        } else {
            if (patchD >= d && patchD != g_infiniteDepth) {
                continue;  // non valid point
            }
            // valid point //
            if (d > minD) {
                minD = static_cast<typeGeometryInput>(roundUp(d, p_->minLevel));  // TODO(lf) use template for roundUp
            }
        }

        // valid point //
        patch.depthL1_[p] = d;
        patch.depthPCidxL1_[p] = pointIndex;

        // TODO(lf): when creating the second patch layer, a deep copy of the first is done no ? So remove the deep copy or remove those
        // following lines
        if (p_->doubleLayer) {
            patch.depthL2_[p] = d;
            patch.depthPCidxL2_[p] = pointIndex;
        }

        size2DXInPixel = (std::max)(size2DXInPixel, u);
        size2DYInPixel = (std::max)(size2DYInPixel, v);
        sizeUom = (std::max)(sizeUom, u / p_->occupancyMapDSResolution);  // TODO(lf): u/occupancyMapDSResolution is donne so many time elsewhere,
                                                                        // that we should consider shifting or doing it once
        sizeVom = (std::max)(sizeVom, v / p_->occupancyMapDSResolution);  // TODO(lf): should be done outside the iteration, on the size2DXInPixel
                                                                        // (which is already doing the max algorithm)
    }

    patch.posD_ = minD;  // TODO(lf): minD computation has to be checked and understood. Some high value seems to make a lot of points filtered
                         // during the filterDepth function minLevel_ is so concerned
    patch.widthInOccBlk_ = sizeUom + 1;  // TODO(lf)explain the + 1 (lf : maybe to allow better packing and avoid overlapping ?)
    patch.heightInOccBlk_ = sizeVom + 1;
    //TODO(lf) why not to also update here patch.widthInPixels_ and patch.heightInPixels_ ?
    patch.patchOccupancyMap_.resize(patch.widthInPixel_ * patch.heightInPixel_,0);
    // TODO(lf): why to resize it here and not during resampledPointCloudRW ?
    // TODO(lf): add an assert to check that the new size is bigger than the current one
    
    assert(patch.widthInOccBlk_ == patch.widthInPixel_/p_->occupancyMapDSResolution && patch.heightInOccBlk_ == patch.heightInPixel_/p_->occupancyMapDSResolution );
    
}

void PatchSegmentation::computePatchDepthL2(uvgvpcc_enc::Patch& patch, const std::vector<size_t>& connectedComponent,
                                            const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry,
                                            const bool isProjectionMode0) {
    patch.depthL2_ = patch.depthL1_;  // Deep copy

    // TODO(lf): check that surfaceThickness > 0 when using double layers during parameter check

    for (size_t ptIndex = 0; ptIndex < connectedComponent.size(); ++ptIndex) {
        const size_t& pointIndex = connectedComponent[ptIndex];
        const uvgvpcc_enc::Vector3<typeGeometryInput>& point = pointsGeometry[pointIndex];
        const typeGeometryInput d = static_cast<typeGeometryInput>(point[patch.normalAxis_]);
        const size_t u = static_cast<size_t>(point[patch.tangentAxis_] - patch.posU_);
        const size_t v = static_cast<size_t>(point[patch.bitangentAxis_] - patch.posV_);
        const size_t p = v * patch.widthInPixel_ + u;
        const typeGeometryInput patchDL1 = patch.depthL1_[p];

        if (patchDL1 == g_infiniteDepth) {
            continue;
        }

        const int16_t projectionDirectionType = isProjectionMode0 ? 1 : -1;  // 1 if getProjectionMode() = 0, otherwise -1
        const int16_t deltaD = static_cast<int16_t>(projectionDirectionType * (d - patchDL1));

        // bool bsimilar = colorSimilarity( frame_pcc_color[i], frame_pcc_color[patch.getDepth0PccIdx()[p]], 128 ); // MY COMMENT : here we
        // are building the D1 map. The critter on color similarity is weird as we should expect D0 and D1 to not care about attribute. As the
        // treshold is 128, maybe it is a way to constrain the difference value in order to encode evrything on a fewer/fixed bit bool
        // bsimilar = true; // TODO(lf): do color similarity if it is found as being relevant

        if (patchDL1 < g_infiniteDepth && deltaD <= static_cast<int>(p_->surfaceThickness) && deltaD >= 0) {  // TODO(lf)add : && bsimilar
            if (projectionDirectionType * (d - patch.depthL2_[p]) > 0) {
                patch.depthL2_[p] = d;
                patch.depthPCidxL2_[p] = pointIndex;
            }
        }

        // TODO(lf): remove this following assert or check if it is really disable during compulation #NDEBUG
        assert((isProjectionMode0 && (patch.depthL2_[p] >= patchDL1)) || (!isProjectionMode0 && (patch.depthL2_[p] <= patchDL1)));
        // lf : TODO(lf), clarify the use of projection mode, make it a patch variable, remove the parameter function use. Verify that the old
        // assert (below) was incorrect assert((isProjectionMode0 && (patch.depthL2_[p] < patchDL1)) || (!isProjectionMode0 &&
        // (patch.depthL2_[p] > patchDL1)));
    }
}

void PatchSegmentation::filterDepth(uvgvpcc_enc::Patch& patch,
                                    const bool isProjectionMode0) {  // TODO(lf)usProjectionMode0 should be a patch parameter

    // This function aims to remove the points from a patch that are too far away from the projection plan. Some of those points may also be
    // under some points of the same patch, but with a separating distance way bigger than the one between the first and secon layer. For
    // example, if you project an Archimede screw on a plan perpendicular to the rotation axis. In such case, the propagation algorithm may
    // create big patches, overlapping themselves.

    // TODO(lf) : This function should be removed and replace by a much clever propagation algorithm during CC creation. During this process, the
    // current minD and maxD of the CC should be updated. Then, a neighboring points, sharing the same PPI, but not respecting the
    // maxPatchThickness (a new parameter, value of 32 in TMC2 it seems from the filterDepth function) condition, should be not accepted.
    // Let's check the color at the same moment ?

    std::vector<typeGeometryInput> peakPerBlock(patch.widthInOccBlk_ * patch.heightInOccBlk_,
                                                isProjectionMode0 ? g_infiniteDepth : 0);
    for (size_t v = 0; v < patch.heightInPixel_; ++v) {
        for (size_t u = 0; u < patch.widthInPixel_; ++u) {
            const size_t p = v * patch.widthInPixel_ + u;
            const typeGeometryInput depth = patch.depthL1_[p];
            if (depth == g_infiniteDepth) {  // TODO(lf): might be a better way to iterate over point index
                continue;
            }
            const size_t uom = u / p_->occupancyMapDSResolution;  // u on the occupancy map
            const size_t vom = v / p_->occupancyMapDSResolution;  // v on the occupancy map
            const size_t pom = vom * patch.widthInOccBlk_ + uom;
            if (isProjectionMode0) {
                peakPerBlock[pom] = (std::min)(peakPerBlock[pom], depth);
            } else {
                peakPerBlock[pom] = (std::max)(peakPerBlock[pom], depth);
            }
        }  // u
    }  // v

    const int8_t projectionDirectionType = static_cast<int8_t>(-2 * static_cast<int8_t>(patch.projectionMode_) + 1);  // 1 or -1
    const size_t geometryNominal2dBitdepth = 8; // TMC2 : Bit depth of geometry 2D (10 in TMC2 lossless, 8 otherwise)
    const size_t maxAllowedDepth = (static_cast<size_t>(1) << geometryNominal2dBitdepth) - 1;

    for (size_t v = 0; v < patch.heightInPixel_; ++v) {
        for (size_t u = 0; u < patch.widthInPixel_; ++u) {
            const size_t pos = v * patch.widthInPixel_ + u;
            const typeGeometryInput depth = patch.depthL1_[pos];
            if (depth == g_infiniteDepth) {
                continue;
            }
            const size_t uom = u / p_->occupancyMapDSResolution;
            const size_t vom = v / p_->occupancyMapDSResolution;
            const size_t pom = vom * patch.widthInOccBlk_ + uom;
            const int tmp_a = std::abs(depth - peakPerBlock[pom]);
            const int tmp_b = static_cast<int>(p_->surfaceThickness) + projectionDirectionType * depth;
            const int tmp_c = projectionDirectionType * static_cast<int>(patch.posD_) +
                              static_cast<int16_t>(maxAllowedDepth);  // TODO(lf): might be done out of the loop
            if ((tmp_a > 32) || (tmp_b > tmp_c)) {
                // TODO(lf): minLevel and minD are suspicious
                // std::cout << maxAllowedDepth_ << " & " << patch.posD_ << " & " << (int)projectionDirectionType << " -> "
                //           << "patch.depthL1_[pos] : " << patch.depthL1_[pos] << " | patch.depthPCidxL1_[pos] : " <<
                //           patch.depthPCidxL1_[pos]
                //           << std::endl;

                patch.depthL1_[pos] = g_infiniteDepth;
                patch.depthPCidxL1_[pos] = g_infinitenumber;  // TODO(lf): should be ginfinit_depth no ?

                // TODO(lf): second layer was never change at this moment, so why to reset its values ?
                if (p_->doubleLayer) {
                    patch.depthL2_[pos] = g_infiniteDepth;
                    patch.depthPCidxL2_[pos] = g_infinitenumber;
                }
            }
        }
    }
}

void PatchSegmentation::computeAdditionalPatchInfo(uvgvpcc_enc::Patch& patch) {
    // lf : geometryBitDepth2D = geometryNominal2dBitdepth   (TMC2 : Bit depth of geometry 2D)
    const size_t geometryNominal2dBitdepth = 8; // TMC2 : Bit depth of geometry 2D
    patch.sizeD_ = std::min<size_t>((1U << std::min<size_t>(p_->geoBitDepthInput, geometryNominal2dBitdepth)) - 1, patch.sizeD_);

    size_t const bitdepthD = std::min<size_t>(p_->geoBitDepthInput, geometryNominal2dBitdepth) -
                                  static_cast<size_t>(std::log2(p_->minLevel));  // TODO(lf) : minlevel will be the power of two directly
    size_t const maxDDplus1 = 1U << bitdepthD;                                   // e.g. 4
    size_t quantDD = patch.sizeD_ == 0 ? 0 : ((patch.sizeD_ - 1) / p_->minLevel + 1);
    quantDD = std::min<size_t>(quantDD, maxDDplus1 - 1);        // 1,2,3,3
    patch.sizeD_ = quantDD == 0 ? 0 : (quantDD * p_->minLevel - 1);  // 63, 127, 191, 191

    // std::cout << "\t\t Patch " << patch.patchIndex_ << " ->(d1,u1,v1)=( " << patch.posD_ << " , " << patch.posU_ << " , " << patch.posV_
    //           << " )(dd,du,dv)=( " << patch.sizeD_ << " , " << patch.widthInPixel_ << " , " << patch.heightInPixel_
    //           << " ),Normal: " << size_t(patch.normalAxis_) << " Direction: " << patch.projectionMode_ << std::endl;
}

// TODO(lf): the LUT search should be inside the possible set of points. Not outside the grid boundaries set by the input geo bit depth =>
// Warning : this is probably false, as some points are considerd as not raw but their distance is still saved in rawPointsDistance
// TODO(lf): tackle the cognitive complexity
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void PatchSegmentation::refillRawPoints(const robin_hood::unordered_set<size_t>& resamplePointSet, std::vector<size_t>& rawPoints,
                                           const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry,
                                           const size_t& pointCount, std::vector<bool>& flags,
                                           robin_hood::unordered_map<size_t, size_t>& nnPropagationMapFlagTrue) {
    // TODO(lf): why do iterate over all input points ? Why not to have a list and remove the already "in-patch" points ? Might use flags ?

    std::fill(flags.begin(), flags.end(), false);
    rawPoints.resize(0);  // TODO(lf): why not to clear ? What about the memory capacity ? Does it change ?

    for (size_t i = 0; i < pointCount; ++i) {
        const auto& point = pointsGeometry[i];

        // Check if same location (already present)
        const size_t pointLocation1D = point[0] + (point[1] << p_->geoBitDepthInput) + (point[2] << (p_->geoBitDepthInput * 2));
        if (resamplePointSet.contains(pointLocation1D)) {
            continue;
        }

        // Check distance of 1
        bool neighborFoundDist1 = false;
        for (const auto& shift : adjacentPointsSearch[0]) {
            uvgvpcc_enc::Vector3<typeGeometryInput> pointAdj;
            pointAdj[0] = point[0] + shift[0];
            pointAdj[1] = point[1] + shift[1];
            pointAdj[2] = point[2] + shift[2];

            const size_t pointAdjLocation1D =
                pointAdj[0] + (pointAdj[1] << p_->geoBitDepthInput) + (pointAdj[2] << (p_->geoBitDepthInput * 2));
            if (resamplePointSet.contains(pointAdjLocation1D)) {
                neighborFoundDist1 = true;
                break;
            }
        }
        if (neighborFoundDist1) {
            continue;
        }

        flags[i] = true;  // Point can be flagged if it not already in a patch or if is further than 1 voxel from a point within a patch // to
                          // do test this second condition, why not 0 ?

        nnPropagationMapFlagTrue.emplace(pointLocation1D, i);
        // TODO(lf): why not to remove this step, and just limit the number of iteration of the main loop ?
        bool neighborFound = false;
        for (size_t dist = 1; dist < p_->maxAllowedDist2RawPointsDetection; ++dist) {  // dist is squared distance
            for (const auto& shift : adjacentPointsSearch[dist]) {
                uvgvpcc_enc::Vector3<typeGeometryInput> pointAdj;
                pointAdj[0] = point[0] + shift[0];
                pointAdj[1] = point[1] + shift[1];
                pointAdj[2] = point[2] + shift[2];

                const size_t pointAdjLocation1D =
                    pointAdj[0] + (pointAdj[1] << p_->geoBitDepthInput) + (pointAdj[2] << (p_->geoBitDepthInput * 2));
                if (resamplePointSet.contains(pointAdjLocation1D)) {
                    neighborFound = true;
                    break;
                }
            }
            if (neighborFound) {
                break;
            }
        }

        if (!neighborFound) {
            // Distance with the nearest point within a patch (in the resample #tmc2) is greater than 9
            // (p_->maxAllowedDist2RawPointsDetection)
            rawPoints.push_back(i);
        }
    }
}


// TODO(lf): orientation and patch segmentation are both doing propagation algorithm. Maybe the correct normal flipping can be done at patch
// segmentation ? (the refine segmentation would be done on absolute normal orientation)
void PatchSegmentation::patchSegmentation(std::shared_ptr<uvgvpcc_enc::Frame>& frame, const std::vector<size_t>& pointsPPIs) {
    const size_t pointCount = frame->pointsGeometry.size();

    uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::TRACE, "PATCH GENERATION",
                             "Patch segmentation of frame " + std::to_string(frame->frameId) + "\n");
    frame->patchList.reserve(256);
    frame->patchPartition.resize(pointCount, g_infinitenumber);

    std::vector<size_t> rawPoints(pointCount);
    for (size_t i = 0; i < pointCount; ++i) {
        rawPoints[i] = i;
    }

    std::vector<bool> flags(pointCount, true);

    // replace the resample point cloud kd tree
    robin_hood::unordered_set<size_t> resamplePointSet;
    resamplePointSet.reserve(pointCount);  // TODO(lf): by construction this reserve is obviously too big. Should we consider an heuristic on
                                           // the missing point ration ?

    // Replace the kd tree temp of the input geometry for the propagation
    // Map (location1D -> pointIndex)
    // Should contain only flag[point] true
    // TODO(lf): map from location1D -> ppi of the point
    // TODO(lf): remove the use of flags vector ?
    robin_hood::unordered_map<size_t, size_t> nnPropagationMapFlagTrue;

    // static size_t frame->frameId = 0;
    nnPropagationMapFlagTrue.reserve(pointCount);
    for (size_t ptIndex = 0; ptIndex < pointCount; ++ptIndex) {
        const auto& point = frame->pointsGeometry[ptIndex];
        const size_t pointLocation1D = point[0] + (point[1] << p_->geoBitDepthInput) + (point[2] << (p_->geoBitDepthInput * 2));
        nnPropagationMapFlagTrue.emplace(pointLocation1D, ptIndex);
    }

    // Until all points are part of a patch or considered rejected //
    while (!rawPoints.empty()) {
        std::vector<std::vector<size_t>>
            connectedComponents;           // TODO(lf): why not to declare it outside the while and empty it after each iteration ?
        connectedComponents.reserve(256);  // TODO(lf)seems to big approximation

        createConnectedComponents(connectedComponents, flags, rawPoints, pointsPPIs, nnPropagationMapFlagTrue, frame->pointsGeometry);

        if (connectedComponents.empty()) {
            break;
        }  // TODO(lf)MY COMMENT : seems impossible in normal usages (but still happens with voxelization)

        for (auto& connectedComponent : connectedComponents) {
            const size_t patchIndex =
                frame->patchList.size();  // MY COMMENT : At the first iteration, patches is an array created a long time ago
                                          // just to save the patches of the frame. So, at the begining it is empty (size==0)
            frame->patchList.emplace_back();
            uvgvpcc_enc::Patch& patch = frame->patchList[patchIndex];
            patch.patchIndex_ = patchIndex;
            patch.setPatchPpi(
                pointsPPIs[connectedComponent[0]]);  // The PPI of one point of the CC (the first one for example) is also the PPI of the CC.

            computePatchBoundingBox(patch, connectedComponent, frame->pointsGeometry);
            if (p_->enablePatchSplitting &&
                (patch.widthInPixel_ > p_->maxPatchSize || patch.heightInPixel_ > p_->maxPatchSize)) {  // lf : enable in ctc
                // lf TODO(lf) : never used with maxpatchsize 1024 and small point cloud
                patchSplitting(connectedComponent, patch, frame->pointsGeometry);
                if (connectedComponent.empty()) {
                    frame->patchList.pop_back();
                    continue;  // TODO(lf): useless check ?
                }
            }

            patch.depthL1_.resize(patch.widthInPixel_ * patch.heightInPixel_,
                                  g_infiniteDepth);
            patch.depthPCidxL1_.resize(patch.widthInPixel_ * patch.heightInPixel_,
                                       g_infiniteDepth);

            if (p_->doubleLayer) {
                patch.depthL2_.resize(patch.widthInPixel_ * patch.heightInPixel_,
                                      g_infiniteDepth);
                patch.depthPCidxL2_.resize(patch.widthInPixel_ * patch.heightInPixel_,
                                           g_infiniteDepth);
            }

            // Introduction to double layer :
            // A patch encode for multiple points of surface. A connected component stock all the points of this surface. During the
            // projection of the connected component into the patch projection plane, some points can be projected at the same 2D location.
            // This leads to missing points, cracks and holes in the final decoded point cloud. To tackle those missing points, a connected
            // component can be projected into two different layer, when doubleLayer is activated. When multiple points of connected component
            // share the same 2D location after projection, the nearest and further point will be written in the first and second layer. The
            // surface thickness parameter adds a constraint, by limiting the difference, (the distance), (the delta in depth value) between
            // the first and second layer.
            //
            // When the double layer is activated, for each frame, one occupancy map, two geometry maps and two attribute maps are generated.
            // This double layer allows to fill an important number of missing points, with a little overhead:
            // The occupancy map is the same for both layers. Thus, the patch packing is done once for both layer, as well as the background
            // filling. Because each second layer is very similar than the first one, the 2D encoding can be very efficient if well
            // configured.
            //
            // Currently, the patch projection mode is defined only by the patch PPI. In the future, a more refined selection could be done.
            //
            // Building of the first layer, also called the "near layer" (computePatchDepthL1):
            // First, the whole patch layer (a rectangle defined by the bounding box of the patch), indicating for each 2D location the patch
            // depth, is filled with a default value, which indicates that no point of the patch is located here. If path projection mode is
            // 0, then the default value is +infinite. If path projection mode is 1, then the default value is 0. For each point in the
            // connected component:
            //      If there is no point already present in the patch at the 2D point location, keep the point (write its depth)
            //      If there is already a point at the 2D location, check if the current point should overwrite or not the already present
            //      point:
            //          Depending on the projection mode, the computation is not the same. However, the objective is always the same. The
            //          first map is the near map, so if the current point is nearer the projection plane, it should overwrite the already
            //          present point.
            //
            // Filter depth : to do, explain this part
            //
            // Building of the second layer, also called the "far layer" (computePatchDepthL2):
            // First, the second layer is a deep copy of the first layer. They are identical.
            // Then, for each point in the connected component:
            //      There is obvisouly already a point in the second layer at the same 2D position. This point is either the same point as the
            //      current point, or a different one. If it is the same point (it means that the value saved in the layer and the depth of
            //      the current point are equals) :
            //          Do nothing. Both layer will share the same points. Both layer encodes for the same point. During the decoding, only
            //          one point will be reconstructed.
            //      If it is not the same point :
            //          Then, the second layer should save a different point than the first layer. As it is the "far layer", the new point
            //          should be further away than the point in the same location in the first layer than the projection plan. The
            //          computation is different depending on the projection mode but the objective is the same. However, the difference in
            //          depth between the first and second layer should respect a constrain, the surface thickness.
            //      Thus, the second layer, aka the far layer, stocks for each 2D location of the patch, a point that is either the same as
            //      the first layer, or the point which is the further away from the patch projection plane among all the points of the
            //      connected component located in this 2D patch position, while respecting the surface thickness constraint.

            // TODO(lf)find a better function name (change also in the comments)
            computePatchDepthL1(patch, connectedComponent, frame->patchPartition, frame->pointsGeometry, !patch.projectionMode_);

            filterDepth(patch, !patch.projectionMode_);

            // TODO(lf): all those three functions might be merged
            if (p_->doubleLayer) {
                computePatchDepthL2(patch, connectedComponent, frame->pointsGeometry, !patch.projectionMode_);
            }

            // my comment : this function compute the patch
            // occupancy map, and seems to definitley and properly set both layers
            // TODO(lf)a map with key : pointIndex and value : patchId is enough instead of this resampled point cloud
            resampledPointcloud(resamplePointSet, patch);

            // TMC2 : note: patch.getSizeD() cannot generate maximum depth(e.g. getSizeD=255, quantDD=3, quantDD needs to be limitted
            // to satisfy the bitcount) max : (1<<std::min(geoBitDepthInput, geometryNominal2dBitdepth))
            computeAdditionalPatchInfo(patch);
        }
        refillRawPoints(resamplePointSet, rawPoints, frame->pointsGeometry, pointCount, flags, nnPropagationMapFlagTrue);
    }

    if (p_->exportIntermediatePointClouds) {
        const std::string plyFilePath =
            p_->intermediateFilesDir + "/patchSegmentation/PATCH-SEGMENTATION_f-" + uvgvpcc_enc::zeroPad(frame->frameId, 3) + ".ply";
        std::vector<uvgvpcc_enc::Vector3<uint8_t>> attributes(frame->pointsGeometry.size());
        for (size_t pointIndex = 0; pointIndex < frame->pointsGeometry.size(); ++pointIndex) {
            // Red if the point is not part of a patch before the 2D projection, otherwise random color from patchColors
            attributes[pointIndex] = frame->patchPartition[pointIndex] == g_infinitenumber
                                         ? uvgvpcc_enc::Vector3<uint8_t>(255, 0, 0)
                                         : patchColors[frame->patchPartition[pointIndex] % patchColors.size()];
        }
        exportPointCloud(plyFilePath, frame->pointsGeometry, attributes);
    }
}
