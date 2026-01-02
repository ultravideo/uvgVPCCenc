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

/// \file Entry point for the patch segmentation process which create the frame patch list.

#include "patchSegmentation.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "robin_hood.h"
#include "utils/constants.hpp"
#include "utils/fileExport.hpp"
#include "utils/parameters.hpp"
#include "utilsPatchGeneration.hpp"
#include "uvgutils/log.hpp"
#include "uvgutils/utils.hpp"
#include "uvgvpcc/uvgvpcc.hpp"
#include "utils/statsCollector.hpp"

using namespace uvgvpcc_enc;

namespace {

struct ConnectedComponent {
    std::vector<size_t> points;
    typeGeometryInput minU{};
    typeGeometryInput minV{};
    typeGeometryInput maxU{};
    typeGeometryInput maxV{};
    size_t ppi;
    size_t tangentAxis;
    size_t bitangentAxis;
    explicit ConnectedComponent(const size_t& ppi) : ppi(ppi) {
        switch (ppi) {
            case 0:
                tangentAxis = 2;
                bitangentAxis = 1;
                break;
            case 1:
                tangentAxis = 2;
                bitangentAxis = 0;
                break;
            case 2:
                tangentAxis = 0;
                bitangentAxis = 1;
                break;
            case 3:
                tangentAxis = 2;
                bitangentAxis = 1;
                break;
            case 4:
                tangentAxis = 2;
                bitangentAxis = 0;
                break;
            case 5:
                tangentAxis = 0;
                bitangentAxis = 1;
                break;
            default:
                assert(false);
                break;
        }
        points.reserve(65536);  // TODO(lf): depends on heuristic for input geometry size
    }
};

inline size_t location1DFromPoint(const uvgutils::VectorN<typeGeometryInput, 3> point) {
    return point[0] + (point[1] << p_->geoBitDepthInput) + (point[2] << (p_->geoBitDepthInput * 2));
}

inline bool findNeighborSeed(const uvgutils::VectorN<typeGeometryInput, 3>& ptSeed,
                             const robin_hood::unordered_set<size_t>& resamplePointSetLocation1D) {
    for (size_t dist = 0; dist < p_->maxAllowedDist2RawPointsDetection; ++dist) {
        for (const auto& shift : adjacentPointsSearch[dist]) {
            uvgutils::VectorN<typeGeometryInput, 3> pointAdj;
            pointAdj[0] = ptSeed[0] + shift[0];
            pointAdj[1] = ptSeed[1] + shift[1];
            pointAdj[2] = ptSeed[2] + shift[2];
            const size_t pointAdjLocation1D = location1DFromPoint(pointAdj);
            if (resamplePointSetLocation1D.contains(pointAdjLocation1D)) {
                return true;
            }
        }
    }
    return false;
}

inline void createConnectedComponent(const std::shared_ptr<uvgvpcc_enc::Frame>& frame, const size_t& seedIndexNewPerf,
                                     std::vector<bool>& pointIsInAPatchNewPerf, ConnectedComponent& cc,
                                     robin_hood::unordered_map<size_t, size_t>& mapLocation1D,
                                     const uvgutils::VectorN<typeGeometryInput, 3>& ptSeed) {
    cc.points.push_back(seedIndexNewPerf);
    pointIsInAPatchNewPerf[seedIndexNewPerf] = true;
    mapLocation1D.erase(location1DFromPoint(ptSeed));

    const size_t uAxis = cc.tangentAxis;    // 0, 1 or 2
    const size_t vAxis = cc.bitangentAxis;  // 0, 1 or 2
    cc.minU = ptSeed[uAxis];
    cc.minV = ptSeed[vAxis];
    cc.maxU = ptSeed[uAxis];
    cc.maxV = ptSeed[vAxis];
    std::vector<size_t> fifo;
    fifo.reserve(65536);
    fifo.emplace_back(seedIndexNewPerf);
    const typeGeometryInput maxVal = (1U << p_->geoBitDepthInput) - 1;
    size_t fifoReadIndex = 0;
    while (fifoReadIndex < fifo.size()) {
        const size_t idx = fifo[fifoReadIndex++];
        const auto& pt = frame->pointsGeometry[idx];

        for (size_t dist = 0; dist < p_->patchSegmentationMaxPropagationDistance; ++dist) {
            for (const auto& shift : adjacentPointsSearch[dist]) {
                uvgutils::VectorN<typeGeometryInput, 3> adjPt = {
                    static_cast<typeGeometryInput>(static_cast<typeGeometryInput>(pt[0]) + static_cast<typeGeometryInput>(shift[0])),
                    static_cast<typeGeometryInput>(static_cast<typeGeometryInput>(pt[1]) + static_cast<typeGeometryInput>(shift[1])),
                    static_cast<typeGeometryInput>(static_cast<typeGeometryInput>(pt[2]) + static_cast<typeGeometryInput>(shift[2]))};
                if (adjPt[0] > maxVal || adjPt[1] > maxVal || adjPt[2] > maxVal) continue;

                const size_t adjLoc1D = location1DFromPoint(adjPt);

                auto it = mapLocation1D.find(adjLoc1D);
                if (it != mapLocation1D.end()) {
                    const size_t adjPtIndex = it->second;
                    mapLocation1D.erase(it);

                    cc.points.push_back(adjPtIndex);
                    pointIsInAPatchNewPerf[adjPtIndex] = true;

                    fifo.emplace_back(adjPtIndex);

                    const typeGeometryInput adjU = adjPt[uAxis];
                    const typeGeometryInput adjV = adjPt[vAxis];

                    cc.minU = std::min(cc.minU, adjU);
                    cc.minV = std::min(cc.minV, adjV);
                    cc.maxU = std::max(cc.maxU, adjU);
                    cc.maxV = std::max(cc.maxV, adjV);
                }
            }
        }
    }
}

template <size_t Ppi>
constexpr size_t getPatchNormalAxis() {
    assert(Ppi < 6);
    if constexpr (Ppi == 0 || Ppi == 3) return 0;
    if constexpr (Ppi == 1 || Ppi == 4) return 1;
    return 2;
}

template <size_t Ppi>
constexpr size_t getPatchTangentAxis() {
    assert(Ppi < 6);
    if constexpr (Ppi == 2 || Ppi == 5) return 0;
    return 2;
}

template <size_t Ppi>
constexpr size_t getPatchBitangentAxis() {
    assert(Ppi < 6);
    if constexpr (Ppi == 1 || Ppi == 4) return 0;
    return 1;
}

template <size_t Ppi>
constexpr bool getPatchProjectionMode() {
    assert(Ppi < 6);
    if constexpr (Ppi == 0 || Ppi == 1 || Ppi == 2) return 0;
    return 1;
}

template <size_t NormalAxis, size_t TangentAxis, size_t BitangentAxis, bool ProjectionMode>
inline void setInitialPatchL1(Patch& patch, const ConnectedComponent& cc, std::vector<typeGeometryInput>& peakPerBlock,
                              const std::shared_ptr<uvgvpcc_enc::Frame>& frame) {
    const size_t widthInPixel = patch.widthInPixel_;
    const size_t widthInOccBlk = patch.widthInOccBlk_;
    const size_t occRes = p_->occupancyMapDSResolution;  // TODO(lf) create an associated log parameter for occupancyMapDSResolution

    for (const size_t pointIndex : cc.points) {
        const auto& point = frame->pointsGeometry[pointIndex];
        const typeGeometryInput d = static_cast<typeGeometryInput>(point[NormalAxis]);

        const size_t u = static_cast<size_t>(point[TangentAxis] - patch.posU_);
        const size_t v = static_cast<size_t>(point[BitangentAxis] - patch.posV_);
        const size_t p = v * widthInPixel + u;

        const size_t uom = u / occRes;
        const size_t vom = v / occRes;
        const size_t pom = vom * widthInOccBlk + uom;

        assert(u < widthInPixel);
        assert(p < patch.depthL1_.size());
        const typeGeometryInput patchD = patch.depthL1_[p];

        if constexpr (ProjectionMode) {
            assert(pom < peakPerBlock.size());
            peakPerBlock[pom] = std::max(peakPerBlock[pom], d);
            if (patchD >= d && patchD != g_infiniteDepth) continue;
        } else {
            peakPerBlock[pom] = std::min(peakPerBlock[pom], d);
            if (patchD <= d) continue;
        }

        // valid point for L1
        // lf : si 0, alors L1 détient les valeurs les plus petites
        // lf : si 1, alors L1 détient les valeurs les plus grandes

        patch.depthL1_[p] = d;
        patch.depthPCidxL1_[p] = pointIndex;
    }
}

template <bool ProjectionMode>
inline int getMinD(const std::vector<typeGeometryInput>& peakPerBlock) {
    const size_t minLevel = p_->minLevel;
    if constexpr (ProjectionMode) {
        auto maxIt = std::max_element(peakPerBlock.begin(), peakPerBlock.end());
        const typeGeometryInput maxVal = (maxIt != peakPerBlock.end()) ? *maxIt : 0;
        return static_cast<int>(uvgutils::roundUp(maxVal, minLevel));
    } else {
        auto minIt = std::min_element(peakPerBlock.begin(), peakPerBlock.end());
        const typeGeometryInput minVal = (minIt != peakPerBlock.end()) ? *minIt : g_infiniteDepth;
        return static_cast<int>((minVal / minLevel) * minLevel);
    }
}

template <bool ProjectionMode>
inline void setPatchL1(Patch& patch, const int& minD, const std::vector<typeGeometryInput>& peakPerBlock) {
    const size_t valueOverflowCheck =
        (1U << 8U) - 1 - p_->surfaceThickness;  // lf: In TMC2, 8 corresponds to geometryNominal2dBitdepth, which probably refers to the
                                                // geometry ouput (geometry maps use uint8)
    for (size_t v = 0; v < patch.heightInPixel_; ++v) {
        for (size_t u = 0; u < patch.widthInPixel_; ++u) {
            const size_t pos = v * patch.widthInPixel_ + u;
            const typeGeometryInput depth = patch.depthL1_[pos];
            if (depth == g_infiniteDepth) {
                continue;
            }

            // check if the current depth value is small enough to be stored in the geometry map (uint8)
            const bool overflow = ProjectionMode ? minD > valueOverflowCheck + depth : depth > valueOverflowCheck + minD;
            if (overflow) {
                patch.depthL1_[pos] = g_infiniteDepth;
                patch.depthPCidxL1_[pos] = g_infinitenumber;
                continue;
            }

            const size_t uom = u / p_->occupancyMapDSResolution;
            const size_t vom = v / p_->occupancyMapDSResolution;
            const size_t pom = vom * patch.widthInOccBlk_ + uom;
            const int tmp_a = std::abs(depth - peakPerBlock[pom]);

            // If there is a hole (a missing point) in a patch, and it happens that this patch is long and overlap itself, then this check
            // allows not to put the isolated point.
            if (tmp_a > p_->distanceFiltering) {
                patch.depthL1_[pos] = g_infiniteDepth;
                patch.depthPCidxL1_[pos] = g_infinitenumber;
                // The lowest (minimum depth) point at this position amoung all the points in the patch being at this position (1 or more) is
                // too far away (>32). So, all the remaining points at this position, if they exist, are higher than the lowest points, and so
                // they are also further away than 32.
                continue;
            }

            patch.patchOccupancyMap_[pos] = 1;
            if constexpr (ProjectionMode) {
                patch.depthL1_[pos] = static_cast<int16_t>((static_cast<int16_t>(minD) - patch.depthL1_[pos]));
            } else {
                patch.depthL1_[pos] = static_cast<int16_t>(patch.depthL1_[pos] - static_cast<int16_t>(minD));
            }
        }
    }
}

template <size_t Ppi, bool DoubleLayer>
inline void finalizePatch(const ConnectedComponent& cc, const std::shared_ptr<uvgvpcc_enc::Frame>& frame, Patch& patch,
                          robin_hood::unordered_map<size_t, size_t>& mapLocation1D, std::vector<bool>& pointIsInAPatchNewPerf,
                          const typeGeometryInput& minD, robin_hood::unordered_set<size_t>& resamplePointSetLocation1D) {
    constexpr size_t normalAxis = getPatchNormalAxis<Ppi>();
    constexpr size_t tangentAxis = getPatchTangentAxis<Ppi>();
    constexpr size_t bitangentAxis = getPatchBitangentAxis<Ppi>();
    constexpr bool projectionMode = getPatchProjectionMode<Ppi>();

    patch.sizeD_ = 0;

    if constexpr (DoubleLayer) {
        patch.depthL2_ = patch.depthL1_;            // Deep copy
        patch.depthPCidxL2_ = patch.depthPCidxL1_;  // Deep copy
    }

    for (const size_t& pointIndex : cc.points) {
        const auto& point = frame->pointsGeometry[pointIndex];
        const size_t u = static_cast<size_t>(point[tangentAxis] - patch.posU_);
        const size_t v = static_cast<size_t>(point[bitangentAxis] - patch.posV_);
        const size_t p = v * patch.widthInPixel_ + u;
        const typeGeometryInput patchDL1 = patch.depthL1_[p];
        const size_t loc1D = location1DFromPoint(point);

        if (patchDL1 == g_infiniteDepth) {
            pointIsInAPatchNewPerf[pointIndex] = false;
            mapLocation1D.emplace(loc1D, pointIndex);
            // lf: this point has been filtered (tmp_a>32). It will be processed during next iteration. There is no point in L1 here as the
            // filtering process is done on block of pixels.
            continue;
        }

        patch.sizeD_ = std::max<size_t>(patch.sizeD_, static_cast<size_t>(patchDL1));

        const typeGeometryInput d = projectionMode ? (minD - point[normalAxis]) : (point[normalAxis] - minD);

        if (patchDL1 == d) {
            // lf: this point is part of L1
            assert(patch.depthPCidxL1_[p] == pointIndex);
            resamplePointSetLocation1D.emplace(loc1D);
            continue;
        }

        if constexpr (DoubleLayer) {
            assert(d > patchDL1);
            assert(d != patch.depthL2_[p]);
            const typeGeometryInput deltaD = d - patchDL1;
            if (d < patch.depthL2_[p]) {
                // This point is between the two layers, it is discarded.
                continue;
            }

            if (deltaD <= p_->surfaceThickness) {
                if (patch.depthL2_[p] != g_infiniteDepth && patch.depthL2_[p] != patchDL1) {
                    const auto overwrittenIdx = patch.depthPCidxL2_[p];
                    const auto& overwrittenPt = frame->pointsGeometry[overwrittenIdx];
                    const size_t overwrittenLoc1D = location1DFromPoint(overwrittenPt);
                    resamplePointSetLocation1D.erase(overwrittenLoc1D);
                    // The overwritten point is between the two layers, it is discarded.
                }
                patch.depthL2_[p] = d;
                patch.depthPCidxL2_[p] = pointIndex;
                resamplePointSetLocation1D.emplace(loc1D);
                patch.sizeD_ = std::max<size_t>(patch.sizeD_, static_cast<size_t>(patch.depthL2_[p]));
                continue;
            }
            if (deltaD <= p_->maxAllowedDist2RawPointsDetection) {
                continue;
            }
            pointIsInAPatchNewPerf[pointIndex] = false;
            mapLocation1D.emplace(loc1D, pointIndex);
        } else {
            assert(d > patchDL1);
            const typeGeometryInput deltaD = d - patchDL1;
            if (deltaD < p_->surfaceThickness) {
                continue;
            }
            pointIsInAPatchNewPerf[pointIndex] = false;
            mapLocation1D.emplace(loc1D, pointIndex);
        }
    }
}

template <size_t Ppi>
inline void createPatch(Patch& patch, const ConnectedComponent& cc, const std::shared_ptr<uvgvpcc_enc::Frame>& frame,
                        std::vector<bool>& pointIsInAPatchNewPerf, robin_hood::unordered_map<size_t, size_t>& mapLocation1D,
                        robin_hood::unordered_set<size_t>& resamplePointSetLocation1D) {
    uvgutils::Logger::log<uvgutils::LogLevel::TRACE>("PATCH GENERATION", "Create patch for frame " + std::to_string(frame->frameId) + "\n");
    constexpr size_t normalAxis = getPatchNormalAxis<Ppi>();
    constexpr size_t tangentAxis = getPatchTangentAxis<Ppi>();
    constexpr size_t bitangentAxis = getPatchBitangentAxis<Ppi>();
    constexpr bool projectionMode = getPatchProjectionMode<Ppi>();

    const size_t dsRes = p_->occupancyMapDSResolution;
    const size_t width = cc.maxU - cc.minU;
    const size_t height = cc.maxV - cc.minV;

    patch.normalAxis_ = normalAxis;
    patch.tangentAxis_ = tangentAxis;
    patch.bitangentAxis_ = bitangentAxis;
    patch.projectionMode_ = projectionMode;
    patch.patchPpi_ = Ppi;
    patch.posU_ = cc.minU;
    patch.posV_ = cc.minV;
    patch.widthInOccBlk_ = width / dsRes + 1;
    patch.heightInOccBlk_ = height / dsRes + 1;
    patch.widthInPixel_ = uvgutils::roundUp(width + 1, dsRes);
    patch.heightInPixel_ = uvgutils::roundUp(height + 1, dsRes);

    const size_t patchSize = patch.widthInPixel_ * patch.heightInPixel_;
    patch.patchOccupancyMap_.assign(patchSize, 0);
    patch.area_ = patchSize;

    assert(patch.widthInOccBlk_ == patch.widthInPixel_ / dsRes && patch.heightInOccBlk_ == patch.heightInPixel_ / dsRes);

    patch.depthL1_.assign(patchSize, g_infiniteDepth);
    patch.depthPCidxL1_.assign(patchSize, g_infinitenumber);

    std::vector<typeGeometryInput> peakPerBlock(patch.widthInOccBlk_ * patch.heightInOccBlk_, !projectionMode ? g_infiniteDepth : 0);

    setInitialPatchL1<normalAxis, tangentAxis, bitangentAxis, projectionMode>(patch, cc, peakPerBlock, frame);

    const int minD = getMinD<projectionMode>(peakPerBlock);
    patch.posD_ = static_cast<size_t>(minD);

    setPatchL1<projectionMode>(patch, minD, peakPerBlock);

    if (p_->doubleLayer) {
        finalizePatch<Ppi, true>(cc, frame, patch, mapLocation1D, pointIsInAPatchNewPerf, minD, resamplePointSetLocation1D);
    } else {
        finalizePatch<Ppi, false>(cc, frame, patch, mapLocation1D, pointIsInAPatchNewPerf, minD, resamplePointSetLocation1D);
    }
}

template <bool FirstIteration>
inline void createConnectedComponents(std::vector<bool>& pointIsInAPatchNewPerf, std::vector<bool>& pointCanBeASeedNewPerf,
                                      const std::shared_ptr<uvgvpcc_enc::Frame>& frame,
                                      const robin_hood::unordered_set<size_t>& resamplePointSetLocation1D,
                                      const std::vector<size_t>& pointsPPIs,
                                      std::array<robin_hood::unordered_map<size_t, size_t>, 6>& mapList,
                                      std::vector<ConnectedComponent>& connectedComponents) {
    uvgutils::Logger::log<uvgutils::LogLevel::TRACE>("PATCH GENERATION",
                                                     "Create connected components for frame " + std::to_string(frame->frameId) + "\n");
    for (size_t seedIndexNewPerf = 0; seedIndexNewPerf < pointIsInAPatchNewPerf.size(); ++seedIndexNewPerf) {
        if (pointIsInAPatchNewPerf[seedIndexNewPerf]) continue;
        if constexpr (!FirstIteration) {
            if (!pointCanBeASeedNewPerf[seedIndexNewPerf]) continue;
        }

        const uvgutils::VectorN<typeGeometryInput, 3> ptSeed = frame->pointsGeometry[seedIndexNewPerf];
        if constexpr (!FirstIteration) {
            // Find a correct seed point to start a connected component
            if (findNeighborSeed(ptSeed, resamplePointSetLocation1D)) {
                pointCanBeASeedNewPerf[seedIndexNewPerf] = false;
                continue;
            }
        }

        // There is no neighboring point of this seed that is in the resample. It is then a correct seed.
        const size_t ppiCC = pointsPPIs[seedIndexNewPerf];
        connectedComponents.emplace_back(ppiCC);
        createConnectedComponent(frame, seedIndexNewPerf, pointIsInAPatchNewPerf, connectedComponents.back(), mapList[ppiCC], ptSeed);
    }
}

}  // Anonymous namespace

void PatchSegmentation::patchSegmentation(const std::shared_ptr<uvgvpcc_enc::Frame>& frame, const std::vector<size_t>& pointsPPIs) {
    uvgutils::Logger::log<uvgutils::LogLevel::TRACE>("PATCH GENERATION",
                                                     "Patch segmentation of frame " + std::to_string(frame->frameId) + "\n");

    const size_t pointCount = frame->pointsGeometry.size();
    frame->patchList.reserve(256);

    std::vector<bool> pointIsInAPatchNewPerf(pointCount, false);
    std::vector<bool> pointCanBeASeedNewPerf(pointCount, true);
    std::array<robin_hood::unordered_map<size_t, size_t>, 6> mapList;
    for (auto& map : mapList) {
        map.reserve(65536);
    }
    for (size_t ptIndex = 0; ptIndex < pointCount; ++ptIndex) {
        const auto& point = frame->pointsGeometry[ptIndex];
        const size_t pointLocation1D = point[0] + (point[1] << p_->geoBitDepthInput) + (point[2] << (p_->geoBitDepthInput * 2));
        assert(pointsPPIs[ptIndex] < 6);
        mapList[pointsPPIs[ptIndex]].emplace(pointLocation1D, ptIndex);
    }

    robin_hood::unordered_set<size_t> resamplePointSetLocation1D;
    resamplePointSetLocation1D.reserve(pointCount);

    std::vector<ConnectedComponent> connectedComponents;
    connectedComponents.reserve(256);

    // Connected components creation (first iteration)
    createConnectedComponents<true>(pointIsInAPatchNewPerf, pointCanBeASeedNewPerf, frame, resamplePointSetLocation1D, pointsPPIs, mapList,
                                    connectedComponents);
    while (!connectedComponents.empty()) {
        // Patches creation
        for (const ConnectedComponent& cc : connectedComponents) {
            if (cc.points.size() < p_->minPointCountPerCC) continue;
            frame->patchList.emplace_back();
            auto& patch = frame->patchList.back();
            patch.patchIndex_ = frame->patchList.size();
            switch (cc.ppi) {
                case 0:
                    createPatch<0>(patch, cc, frame, pointIsInAPatchNewPerf, mapList[0], resamplePointSetLocation1D);
                    break;
                case 1:
                    createPatch<1>(patch, cc, frame, pointIsInAPatchNewPerf, mapList[1], resamplePointSetLocation1D);
                    break;
                case 2:
                    createPatch<2>(patch, cc, frame, pointIsInAPatchNewPerf, mapList[2], resamplePointSetLocation1D);
                    break;
                case 3:
                    createPatch<3>(patch, cc, frame, pointIsInAPatchNewPerf, mapList[3], resamplePointSetLocation1D);
                    break;
                case 4:
                    createPatch<4>(patch, cc, frame, pointIsInAPatchNewPerf, mapList[4], resamplePointSetLocation1D);
                    break;
                case 5:
                    createPatch<5>(patch, cc, frame, pointIsInAPatchNewPerf, mapList[5], resamplePointSetLocation1D);
                    break;
                default:
                    assert(false);
                    break;
            }
        }

        // Connected components creation
        connectedComponents.clear();
        createConnectedComponents<false>(pointIsInAPatchNewPerf, pointCanBeASeedNewPerf, frame, resamplePointSetLocation1D, pointsPPIs,
                                         mapList, connectedComponents);
    }

    if(p_->exportStatistics){
        size_t numberOfLostPointPS = 0;
        std::vector<uvgutils::VectorN<uint8_t, 3>> attributes(frame->pointsGeometry.size());
        std::vector<bool> pointColored(frame->pointsGeometry.size(), false);
        for (const auto& patch : frame->patchList) {
            const auto color = patchColors[patch.patchIndex_ % patchColors.size()];
            for (size_t v = 0; v < patch.heightInPixel_; ++v) {
                for (size_t u = 0; u < patch.widthInPixel_; ++u) {
                    const size_t pos = v * patch.widthInPixel_ + u;
                    const typeGeometryInput depth = patch.depthL1_[pos];
                    if (depth == g_infiniteDepth) {
                        if (p_->doubleLayer) {
                            assert(patch.depthL2_[pos] == g_infiniteDepth);
                        }
                        continue;
                    }
                    const size_t ptIndexL1 = patch.depthPCidxL1_[pos];
                    attributes[ptIndexL1] = color;
                    pointColored[ptIndexL1] = true;

                    if (!p_->doubleLayer) continue;
                    const size_t ptIndexL2 = patch.depthPCidxL2_[pos];
                    if (ptIndexL1 == ptIndexL2) continue;
                    attributes[ptIndexL2] = color;
                    pointColored[ptIndexL2] = true;
                }
            }
        }
        for (int i = 0; i < frame->pointsGeometry.size(); ++i) {
            if (pointColored[i]) continue;
            // Points that are not within a patch are colored in red
            numberOfLostPointPS++;
        }
        // stats.setNumberOfLostPoints(frame->frameId, numberOfLostPointPS);
        stats.collectData(frame->frameId, DataId::NumberOfLostPoints, numberOfLostPointPS);
    }

    if (p_->exportIntermediateFiles) {
        FileExport::exportPointCloudPatchSegmentationColor(frame);
        FileExport::exportPointCloudPatchSegmentationBorder(frame);
        FileExport::exportPointCloudPatchSegmentationBorderBlank(frame);
    }
}