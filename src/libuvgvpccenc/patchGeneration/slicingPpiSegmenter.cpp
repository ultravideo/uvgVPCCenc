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

/// \file This file combine both the initial segmentation and the refine segmentation. Assign a PPI (projection plan index) to each point.

#include "slicingPpiSegmenter.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "robin_hood.h"
#include "utils/fileExport.hpp"
#include "utils/parameters.hpp"
#include "utils/constants.hpp"
#include "uvgutils/utils.hpp"
#include "utilsPatchGeneration.hpp"
#include "uvgutils/log.hpp"
#include "uvgvpcc/uvgvpcc.hpp"
#include "utils/statsCollector.hpp"

using namespace uvgvpcc_enc;

PPISegmenter_NewRS::PPISegmenter_NewRS(const std::vector<uvgutils::VectorN<typeGeometryInput, 3>>& pointsGeometry,
                           const std::vector<bool>& pointsNormals)
    : normalExists_(pointsNormals),
      pointsGeometry_(pointsGeometry),
      geoMax_([&]() {
          // lambda function to initialise the const variable geoMax_
          typeGeometryInput geoMax = pointsGeometry_[0][0];
          for (const uvgutils::VectorN<typeGeometryInput, 3>& point : pointsGeometry_) {
              geoMax = std::max(geoMax, point[0]);
              geoMax = std::max(geoMax, point[1]);
              geoMax = std::max(geoMax, point[2]);
          }
          return geoMax;
      }()),
      geoRange_([&]() {
          // lambda function to initialise the const variable geoRange_
          // TODO(lf): warning, the geo range does not take care of negatie value. Is it common for point cloud to have negative value ?
          typeGeometryInput geoRange = 1;
          for (typeGeometryInput i = geoMax_ - 1; i != 0U; i >>= 1U, geoRange <<= 1U) {
              ;
          }
          return geoRange;
      }()) {}

VoxelAttribute_NewRS::VoxelAttribute_NewRS(const size_t projectionPlaneCount_)
    : updateFlag_(false), voxClass_(VoxClass_NewRS::NO_EDGE), voxPPI_(0), voxScore_{0} {}

// TODO(lf): the number of points in the voxel is usefull only for DE-V voxel no ? So why to set the value for all voxels ?
// TODO(lf): use two flags, compute one time the flag for S or M instead of checking it like the other classification

inline void PPISegmenter_NewRS::updateVoxelAttribute_NewRS(VoxelAttribute_NewRS& voxAttribute, const std::vector<size_t>& voxPoints,
                                               const std::vector<size_t>& pointsPPIs) {
    std::array<size_t, 6>& voxScore = voxAttribute.voxScore_;

    // Single Direct Edge Voxel : One point in the voxel //
    if (voxAttribute.voxClass_ == VoxClass_NewRS::S_DIRECT_EDGE) {
        voxAttribute.voxPPI_ = pointsPPIs[voxPoints[0]];
        voxScore[voxAttribute.voxPPI_] = 1;
        return;
    }

    // No Edge Voxel or Multiple Direct Edge Voxel //
    for (const size_t& pointIndex : voxPoints) {
        ++voxScore[pointsPPIs[pointIndex]];  // histogram of the ppis inside the voxel
    }

    if (voxScore[pointsPPIs[voxPoints[0]]] == voxPoints.size()) {
        // No Edge Voxel : All points within the voxel have the same PPI //
        voxAttribute.voxClass_ = VoxClass_NewRS::NO_EDGE;
        voxAttribute.voxPPI_ = pointsPPIs[voxPoints[0]];
        return;
    }

    // Multiple Direct Edge Voxel : Points within the voxel have different PPI //
    voxAttribute.voxClass_ = VoxClass_NewRS::M_DIRECT_EDGE;

    // The voxel PPI is the most represented PPI among the points inside it //
    const auto& maxScore = std::max_element(voxScore.begin(), voxScore.end());
    voxAttribute.voxPPI_ = static_cast<size_t>(std::distance(voxScore.begin(), maxScore));
}

void PPISegmenter_NewRS::computeExtendedScore_NewRS(std::array<size_t,6>& voxExtendedScore,
                                        const std::vector<VoxelAttribute_NewRS>& voxAttributeList,
                                        const std::vector<size_t>& ADJ_ListNew) {
    std::fill(voxExtendedScore.begin(), voxExtendedScore.end(), 0);
    for (const auto& voxelIndex : ADJ_ListNew) {
        for (size_t k = 0; k < p_->projectionPlaneCount; ++k) {
            voxExtendedScore[k] += voxAttributeList[voxelIndex].voxScore_[k];
        }
    }
}

// TODO(lf)warning : adjacent (old name) voxel contain the voxel itself!
void PPISegmenter_NewRS::updateAdjacentVoxelsClass_NewRS(std::vector<VoxelAttribute_NewRS>& voxAttributeList, const std::array<size_t,6>& voxExtendedScore,
                                             const std::vector<size_t>& IDEV_List) {
    // Common and effective way to find the index of the maximum element in a C++ container
    const auto& maxScoreSmooth = std::max_element(voxExtendedScore.begin(), voxExtendedScore.end());
    const size_t ppiOfScoreSmooth = std::distance(voxExtendedScore.begin(), maxScoreSmooth);

    for (const auto& voxelIndex : IDEV_List) {
        VoxelAttribute_NewRS& adjVoxAttribute = voxAttributeList[voxelIndex];
        if (adjVoxAttribute.voxClass_ == VoxClass_NewRS::NO_EDGE && adjVoxAttribute.voxPPI_ != ppiOfScoreSmooth) {
            adjVoxAttribute.voxClass_ = VoxClass_NewRS::INDIRECT_EDGE;
        }
    }
}

inline bool PPISegmenter_NewRS::checkNEV_NewRS(const VoxClass_NewRS voxClass, const size_t voxPPI, const std::array<size_t,6>& voxExtendedScore) {
    // TODO(lf): why not to check if S_DIRECT_EDGE ?

    if (voxClass == VoxClass_NewRS::M_DIRECT_EDGE) {  // TMC2 : VoxClass::S_DIRECT_EDGE or VoxClass::INDIRECT_EDGE
        return false;
    }

    // TODO(lf): instead, use bool comparison (if 0 it is false). Might be faster
    // if validNumOfScores == 1, then there are 5 (if 6 projection planes) planes with a score of 0, meaning that all adjacent voxels share
    // the same PPI.
    size_t const validNumOfScores = p_->projectionPlaneCount - std::count(voxExtendedScore.begin(), voxExtendedScore.end(), 0);
    if (validNumOfScores != 1) {
        // Adjacent voxels does not share the same PPI //
        return false;
    }

    // If voxExtendedScore[voxPPI] == 0 -> The current voxel PPI is not present among the PPIs of the neighbor voxels //

    return voxExtendedScore[voxPPI] != 0;

    // TODO(lf)verify but it seems like if true is returned, then the current voxel is a N-EV
    // TODO(lf): we can probably skip the score update and give the right classification (set edge NEV) -> NOP because we have to wait for the
    // current iteration to finnish before to update the state of all voxels why don't we call setUpdatedFlag on this one ?
}

// TODO(lf): special algorithm trajectory for S_DIRECT_EDGE_VOXEL
inline void PPISegmenter_NewRS::refinePointsPPIs_NewRS(std::vector<size_t>& pointsPPIs, const std::vector<size_t>& pointsPPIs_origin, const std::vector<size_t>& pointsIndices,
                                         const std::array<size_t,6>& voxExtendedScore, const size_t nnPointCount) const {
    std::array<double,6> weightedScoreSmooth{0};
    for (size_t k = 0; k < p_->projectionPlaneCount; ++k) {
        weightedScoreSmooth[k] = p_->refineSegmentationLambda * static_cast<double>(voxExtendedScore[k]); 
    }
    // For each point in the current voxel //
    for (const auto& pointIndex : pointsIndices) {
        const auto& dotProductList = normalsDotProducts[pointsPPIs_origin[pointIndex]];
        const bool normalBool = normalExists_[pointIndex];
        // Here the new calculation method
        // If normalBool = false : the right member=0
        double scoreMax2 = weightedScoreSmooth[0] + static_cast<double>(normalBool) * static_cast<double>(dotProductList[0])  *  static_cast<double>(nnPointCount);
        size_t PPIscoreMax = 0;
        for (size_t k = 1; k < p_->projectionPlaneCount; ++k) {
            const double score2 = weightedScoreSmooth[k] + static_cast<double>(normalBool) * static_cast<double>(dotProductList[k])  *  static_cast<double>(nnPointCount);
            if (score2 > scoreMax2) {
                scoreMax2 = score2;
                PPIscoreMax = k;
            }
        }
        pointsPPIs[pointIndex] = PPIscoreMax;
    }
}

template<typename keyType>
void PPISegmenter_NewRS::voxelizationWithBitArray_NewRS(const std::vector<uvgutils::VectorN<typeGeometryInput, 3>>& inputPointsGeometry,
                                            std::vector<bool>& occFlagArray, robin_hood::unordered_map<keyType, size_t>& voxelIdxMap,
                                            std::vector<keyType>& filledVoxels, std::vector<std::vector<size_t>>& pointListInVoxels) {
    const size_t voxelizationShift =
        p_->geoBitDepthVoxelized - p_->geoBitDepthRefineSegmentation;  // i.e. : 9 - 8 = 1 (meaning 2x2x2 voxel dimension)
    const size_t gbdrs = p_->geoBitDepthRefineSegmentation;
    const size_t gbdrs2 = p_->geoBitDepthRefineSegmentation * 2;        

    // TODO(lf): this is a simple heuristic from longdress (10->9->8). At least round up to the power of 2
    // (as it will already be done by the reserve function)
    const size_t estimatedVoxelCount = 3U * (inputPointsGeometry.size() >> (voxelizationShift * 3U));
    voxelIdxMap.reserve(estimatedVoxelCount);
    filledVoxels.reserve(estimatedVoxelCount);
    pointListInVoxels.reserve(estimatedVoxelCount);

    const size_t estimatedPointsPerVoxel = 4;  // TODO(lf): verify and make it dependent on the chosen geoBitDepth

    // TODO(lf): the "reserve" of the pointListInVoxels is probably inneficient. As for each vector inside the vector, we call reserve. Would
    // it be better to reserve the main vector with constant sized (already allocated) smaller vectors and then call resize(0) on each of them
    // before to add the first point ?

    size_t vox_id = 0;
    for (size_t point_idx = 0; point_idx < inputPointsGeometry.size(); ++point_idx) {
        const uvgutils::VectorN<typeGeometryInput, 3>& inputPoint = inputPointsGeometry[point_idx];

        const int vx = inputPoint[0] >> voxelizationShift;
        const int vy = inputPoint[1] >> voxelizationShift;
        const int vz = inputPoint[2] >> voxelizationShift;
        const keyType pos_1D = location1DFromCoordinates<keyType>(vx, vy, vz, gbdrs,gbdrs2);

        if (!occFlagArray[pos_1D]) {
            occFlagArray[pos_1D] = true;
            voxelIdxMap.emplace(pos_1D, vox_id++);
            filledVoxels.emplace_back(pos_1D);
            pointListInVoxels.emplace_back().reserve(estimatedPointsPerVoxel);
            pointListInVoxels.back().push_back(point_idx);
        } else {
            const size_t filled_v_idx = voxelIdxMap.at(pos_1D);
            pointListInVoxels[filled_v_idx].push_back(point_idx);
        }
    }
}

/*
== Refine segmentation doc ==

Ref : V-PCC codec / TMC2 description -> ISO/IEC JTC 1/SC 29/WG 7 N00100

All point in the input point cloud (which can be voxelized) has a PPI (Plane Projection Index) when the refine segmentation occurs.
The point cloud is segmented by a grid. Each point inside a grid cell is associated with the center of the grid cell.
This grid-based segmentation can be seen as a voxelization or a point cloud down-scalling. A grid cell can be call a voxel.

The whole grid is composed of filled voxels and empty voxels, wether there is or not points inside.
Nearly all cells of the grid are empty.

Filled voxels are classified into three groups nammed DE-V, N-EV and IDE-V.

We call a voxel containing points which have the same PPI an Uppi-V (uniform PPI distribution). We can say that the PPI of this voxel is the
one of the points inside it. Most of the filled voxels are Uppi-V.

If an Uppi-V is only surrounded (Nearest Neighbors) by voxels that are all Uppi-V and if they all share the same PPI, then it is classified as
NE-V (No edge-voxel). Thus, a NE-V is not located at a patch border. It will not be took into consideration during the iterative process of
the refine segmentation. This leads to an important performance gain.

If an Uppi-V has a PPI which is not uniform compared to the surrounded voxels, it is denoted as IDE-V (Indirect Edge Voxel).
IDE-V voxels are located near patch border.

DE-V (Direct Edge Voxel) are voxels with PPI variations amoung their points. They can be either a single-point (S-DE) or multi-points (M-DE)
in a voxel. The former is usually isolated points, and the latter indicates the presence of a point cloud surface.

 */

// TODO(lf): in the whole refine segmentation, be consistent between talking about grid cell or voxel
// TODO(lf): use two flags, compute one time the flag for S or M instead of checking it like the other classification
// TODO(lf): the refine segmentation voxelization (voxel dim etc..) should depend on geometry bit, not on the max range
template<typename keyType>
void PPISegmenter_NewRS::refineSegmentation_NewRS(const std::shared_ptr<uvgvpcc_enc::Frame>& frame, std::vector<size_t>& pointsPPIs,
                                      const size_t& frameId) {
    uvgutils::Logger::log<uvgutils::LogLevel::TRACE>("PATCH GENERATION", "Refine segmentation of frame " + std::to_string(frameId) + "\n");
    const size_t gbdrs = p_->geoBitDepthRefineSegmentation;
    const size_t gbdrs2 = p_->geoBitDepthRefineSegmentation * 2;
    const size_t gridSize = 1U << gbdrs;
    const int maxVal = gridSize - 1;
    
    // One boolean for each voxel of the grid, indicating if a voxel is filled or not //
    std::vector<bool> occFlagArray(gridSize * gridSize * gridSize, false);

    robin_hood::unordered_map<keyType, size_t> voxelIdxMap;  // location1D -> index in voxel list (filledVoxels)

    std::vector<keyType> filledVoxels;                    // list of location1D
    std::vector<std::vector<size_t>> pointListInVoxels;  // for each voxel, the list of the index of the points inside

    voxelizationWithBitArray_NewRS(pointsGeometry_, occFlagArray, voxelIdxMap, filledVoxels, pointListInVoxels);

    const size_t voxelCount = filledVoxels.size();

    if(p_->exportStatistics){
        stats.collectData(frame->frameId, DataId::NumberOfVoxelsRS, voxelCount);
    }

    // The 1st classification is made here (+ score computation)
    std::vector<VoxelAttribute_NewRS> voxAttributeList(voxelCount, VoxelAttribute_NewRS(p_->projectionPlaneCount));
    for (size_t v_idx = 0; v_idx < voxelCount; ++v_idx) {
        // Iterate through all voxels to set score, classification and voxel PPI //
        // First classification : NE-V or DE-V (SDE-V or MDE-V) //
        VoxelAttribute_NewRS& voxAttribute = voxAttributeList[v_idx];
        if (pointListInVoxels[v_idx].size() == 1) {
            // Single Direct Edge Voxel : One point in the voxel //
            voxAttribute.voxClass_ = VoxClass_NewRS::S_DIRECT_EDGE;
        }
        updateVoxelAttribute_NewRS(voxAttribute, pointListInVoxels[v_idx], pointsPPIs);
    }

    const std::vector<size_t> pointsPPIs_O = pointsPPIs;
    std::vector<std::vector<size_t>> ADJ_List(voxelCount);   // large    // This is voxNeighborsList
    std::vector<std::vector<size_t>> IDEV_List(voxelCount);  // small    // This is voxAdjacentsList
    std::vector<size_t> nnPointCountList(voxelCount);

    // TODO(lf): find a way to break the refine segmentation iteration before reaching the number of iteration parameter (if number of updated
    // voxel lower than something for example)

    // TODO(lf): in the for loop over all voxel, we access a lot of list to get the related voxel element. Why not TODO(lf)a structure voxel
    // with everything at the same memory location and so reduce memory call ?

    std::array<size_t, 6> voxExtendedScore{0};
    std::vector<uint8_t> hasBeenComputed(voxelCount, 0);

    const size_t bitMask = (1U << gbdrs) - 1;
    const size_t distanceSearch = p_->refineSegmentationMaxNNVoxelDistanceLUT;

    for (size_t iter = 0; iter < p_->refineSegmentationIterationCount; ++iter) {
        for (size_t voxelIndex = 0; voxelIndex < voxelCount; ++voxelIndex) {
            // TODO(lf): should we use a stack of voxel index instead of a for loop with a lot of if(true) ?
            const VoxClass_NewRS& voxClass = voxAttributeList[voxelIndex].voxClass_;
            if (voxClass == VoxClass_NewRS::NO_EDGE) {
                if(p_->exportStatistics){
                    stats.collectData(frame->frameId, DataId::SkippedVoxels, iter);
                }
                continue;  // This voxel has been marked as NE-V before the current iteration //
            }
            
            voxExtendedScore.fill(0);
            if(hasBeenComputed[voxelIndex] == 1){
                computeExtendedScore_NewRS(voxExtendedScore, voxAttributeList, ADJ_List[voxelIndex]);
            } else {
                hasBeenComputed[voxelIndex] = 1;
            
                const int cur_pos_1D = static_cast<int>(filledVoxels[voxelIndex]);
                const int curz = cur_pos_1D >> gbdrs2;
                const int cury = (cur_pos_1D >> gbdrs) & bitMask;
                const int curx = cur_pos_1D & bitMask;
                
                size_t num_nn_points = 0;
                for (size_t dist = 0; dist < distanceSearch; ++dist) {  // dist is squared distance
                    for (const auto& shift : adjacentPointsSearch[dist]) {

                        const int x = curx + shift[0];
                        const int y = cury + shift[1];
                        const int z = curz + shift[2];

                        if (x < 0 || x > maxVal || y < 0 || y > maxVal || z < 0 || z > maxVal) continue;

                        const keyType adjLoc1D = location1DFromCoordinates<keyType>(x,y,z,gbdrs,gbdrs2);
                        if (occFlagArray[adjLoc1D]) {
                            const size_t neighbor_v_idx = voxelIdxMap.at(adjLoc1D);
                            // ADJ_List.push_back(neighbor_v_idx);  // TODO(lf): do a big check everywhere because here adjacent and neighbor are inverted
                            ADJ_List[voxelIndex].push_back(neighbor_v_idx);
            
                            // Extended score computation
                            for (size_t k = 0; k < p_->projectionPlaneCount; ++k) {
                                voxExtendedScore[k] += voxAttributeList[neighbor_v_idx].voxScore_[k];
                            }
            
                            const size_t IDEV_range = p_->refineSegmentationIDEVDist; // TODO(lf)justifiy this value, and make it dependent on the geobitdepth
                            if (dist <= IDEV_range) {
                                IDEV_List[voxelIndex].push_back(neighbor_v_idx);
                            }
                            num_nn_points += pointListInVoxels[neighbor_v_idx].size();
                        }
                    }
                }
                nnPointCountList[voxelIndex] = num_nn_points;
            }

            updateAdjacentVoxelsClass_NewRS(voxAttributeList, voxExtendedScore, IDEV_List[voxelIndex]);
            if (checkNEV_NewRS(voxClass, voxAttributeList[voxelIndex].voxPPI_, voxExtendedScore)) {
                continue;  // The current iteration found that this voxel is NE-V //
            }

            // The voxel is not NE-V, so it is D-EV or IDE-V and its points PPI can be refined //
            if(p_->exportStatistics){
                std::vector<size_t> previousPointsPPI = pointsPPIs;
                refinePointsPPIs_NewRS(pointsPPIs, pointsPPIs_O, pointListInVoxels[voxelIndex], voxExtendedScore, nnPointCountList[voxelIndex]);
                voxAttributeList[voxelIndex].updateFlag_ = true;
                for(size_t i = 0 ; i < pointsPPIs.size() ; ++i){
                    if(previousPointsPPI[i] != pointsPPIs[i]){
                        stats.collectData(frame->frameId, DataId::PpiChange, iter);
                    }
                }
            }
            else{
                refinePointsPPIs_NewRS(pointsPPIs, pointsPPIs_O, pointListInVoxels[voxelIndex], voxExtendedScore, nnPointCountList[voxelIndex]);
                voxAttributeList[voxelIndex].updateFlag_ = true;
            }
            
            if(p_->exportStatistics){
                for(size_t i = 0 ; i < pointListInVoxels[voxelIndex].size() ; ++i){
                    stats.collectData(frame->frameId, DataId::ScoreComputations, iter);
                }
                switch (voxAttributeList[voxelIndex].voxClass_){
                    case VoxClass_NewRS::NO_EDGE:       stats.collectData(frame->frameId, DataId::NoEdge_R,       iter); break;
                    case VoxClass_NewRS::INDIRECT_EDGE: stats.collectData(frame->frameId, DataId::IndirectEdge_R, iter); break;
                    case VoxClass_NewRS::S_DIRECT_EDGE: stats.collectData(frame->frameId, DataId::SingleEdge_R,   iter); break;
                    case VoxClass_NewRS::M_DIRECT_EDGE: stats.collectData(frame->frameId, DataId::MultiEdge_R,    iter); break;
                }
            }
        }

        // Update voxel classification and scores if points PPI inside have changed during the iteration //
        for (size_t voxelIndex = 0; voxelIndex < voxelCount; ++voxelIndex) {
            // TODO(lf): it might be faster to use a stack of index instead of using a flag
            if (!voxAttributeList[voxelIndex].updateFlag_) {
                continue;
            }
            voxAttributeList[voxelIndex].updateFlag_ = false;
            std::fill(voxAttributeList[voxelIndex].voxScore_.begin(), voxAttributeList[voxelIndex].voxScore_.end(), 0);
            updateVoxelAttribute_NewRS(voxAttributeList[voxelIndex], pointListInVoxels[voxelIndex], pointsPPIs);
        }

        // Compute de number of changes of classification
        if(p_->exportStatistics){
            for(auto& voxel : voxAttributeList){
                VoxClass_NewRS VC = voxel.voxClass_;
                switch (VC) {
                    case VoxClass_NewRS::NO_EDGE:       stats.collectData(frame->frameId, DataId::NoEdge,       iter); break;
                    case VoxClass_NewRS::INDIRECT_EDGE: stats.collectData(frame->frameId, DataId::IndirectEdge, iter); break;
                    case VoxClass_NewRS::S_DIRECT_EDGE: stats.collectData(frame->frameId, DataId::SingleEdge,   iter); break;
                    case VoxClass_NewRS::M_DIRECT_EDGE: stats.collectData(frame->frameId, DataId::MultiEdge,    iter); break;
                }
            }
        }
    }

    if (p_->exportIntermediateFiles) {
        FileExport::exportPointCloudRefineSegmentation(frame, pointsGeometry_, pointsPPIs);
    }
}


template void PPISegmenter_NewRS::voxelizationWithBitArray_NewRS<uint16_t>(const std::vector<uvgutils::VectorN<typeGeometryInput, 3>>& inputPointsGeometry,
                                            std::vector<bool>& occFlagArray, robin_hood::unordered_map<uint16_t, size_t>& voxelIdxMap,
                                            std::vector<uint16_t>& filledVoxels, std::vector<std::vector<size_t>>& pointListInVoxels);
template void PPISegmenter_NewRS::voxelizationWithBitArray_NewRS<uint32_t>(const std::vector<uvgutils::VectorN<typeGeometryInput, 3>>& inputPointsGeometry,
                                            std::vector<bool>& occFlagArray, robin_hood::unordered_map<uint32_t, size_t>& voxelIdxMap,
                                            std::vector<uint32_t>& filledVoxels, std::vector<std::vector<size_t>>& pointListInVoxels);
template void PPISegmenter_NewRS::voxelizationWithBitArray_NewRS<uint64_t>(const std::vector<uvgutils::VectorN<typeGeometryInput, 3>>& inputPointsGeometry,
                                            std::vector<bool>& occFlagArray, robin_hood::unordered_map<uint64_t, size_t>& voxelIdxMap,
                                            std::vector<uint64_t>& filledVoxels, std::vector<std::vector<size_t>>& pointListInVoxels);                                                                                        


template void PPISegmenter_NewRS::refineSegmentation_NewRS<uint16_t>(const std::shared_ptr<uvgvpcc_enc::Frame>& frame, std::vector<size_t>& pointsPPIs,
                                    const size_t& frameId);
template void PPISegmenter_NewRS::refineSegmentation_NewRS<uint32_t>(const std::shared_ptr<uvgvpcc_enc::Frame>& frame, std::vector<size_t>& pointsPPIs,
                                    const size_t& frameId);
template void PPISegmenter_NewRS::refineSegmentation_NewRS<uint64_t>(const std::shared_ptr<uvgvpcc_enc::Frame>& frame, std::vector<size_t>& pointsPPIs,
                                    const size_t& frameId);