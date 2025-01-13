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

/// \file This file combine both the initial segmentation and the refine segmentation. Assign a PPI (projection plan index) to each point.

#include "ppiSegmenter.hpp"

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <string>
#include <unordered_map>
#include <cstdint>
#include <vector>

#include "uvgvpcc/log.hpp"
#include "uvgvpcc/uvgvpcc.hpp"
#include "utils/utils.hpp"

#include "utilsPatchGeneration.hpp"


using namespace uvgvpcc_enc;


PPISegmenter::PPISegmenter(const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry,
                           const std::vector<uvgvpcc_enc::Vector3<double>>& pointsNormals)
    : pointsNormals_(pointsNormals),
      pointsGeometry_(pointsGeometry),
      geoMax_([&]() {
          // lambda function to initialise the const variable geoMax_
          typeGeometryInput geoMax = pointsGeometry_[0][0];
          for (const uvgvpcc_enc::Vector3<typeGeometryInput>& point : pointsGeometry_) {
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
      }())
      {}

VoxelAttribute::VoxelAttribute(const size_t projectionPlaneCount_)
    : updateFlag_(false), voxClass_(VoxClass::NO_EDGE), voxPPI_(0), voxScore_(projectionPlaneCount_, 0) {}

// TODO(lf): check if the initial segmentation can be done inside the precomputation of the refineSegmentation
// TODO(lf): use auto& : ... everywhere instead of for loop (and try avoiding using pointCount or size())
void PPISegmenter::initialSegmentation(std::vector<size_t>& pointsPPIs, const size_t& frameId) {
    uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::TRACE, "PATCH GENERATION",
                             "Initial segmentation of frame " + std::to_string(frameId) + "\n");
    for (size_t ptIndex = 0; ptIndex < pointsPPIs.size(); ++ptIndex) {
        const uvgvpcc_enc::Vector3<double>& pointNormal = pointsNormals_[ptIndex];

        size_t ppi = 0;  // TODO(lf): check if we don't call too many time array element in other for loops and use temp value like here
        double bestScore = dotProduct(pointNormal, p_->projectionPlaneOrientations[0]);
        for (size_t ppIndex = 1; ppIndex < p_->projectionPlaneCount; ++ppIndex) {
            const double score = dotProduct(pointNormal, p_->projectionPlaneOrientations[ppIndex]);
            if (score > bestScore) {
                bestScore = score;
                ppi = ppIndex;
            }
        }
        pointsPPIs[ptIndex] = ppi;
    }

    if (p_->exportIntermediatePointClouds) {
        const std::string plyFilePath =
            p_->intermediateFilesDir + "/initialSegmentation/INITIAL-SEGMENTATION_f-" + uvgvpcc_enc::zeroPad(frameId, 3) + ".ply";
        std::vector<uvgvpcc_enc::Vector3<uint8_t>> attributes(pointsGeometry_.size());
        for (size_t pointIndex = 0; pointIndex < pointsGeometry_.size(); ++pointIndex) {
            attributes[pointIndex] = ppiColors[pointsPPIs[pointIndex]];
        }
        exportPointCloud(plyFilePath, pointsGeometry_, attributes);
    }
}

// TODO(lf): the number of points in the voxel is usefull only for DE-V voxel no ? So why to set the value for all voxels ?
// TODO(lf): use two flags, compute one time the flag for S or M instead of checking it like the other classification

inline void PPISegmenter::updateVoxelAttribute(VoxelAttribute& voxAttribute, const std::vector<size_t>& voxPoints,
                                                         const std::vector<size_t>& pointsPPIs) {
    std::vector<size_t>& voxScore = voxAttribute.voxScore_;

    // Single Direct Edge Voxel : One point in the voxel //
    if (voxAttribute.voxClass_ == VoxClass::S_DIRECT_EDGE) {
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
        voxAttribute.voxClass_ = VoxClass::NO_EDGE;
        voxAttribute.voxPPI_ = pointsPPIs[voxPoints[0]];
        return;
    }

    // Multiple Direct Edge Voxel : Points within the voxel have different PPI //
    voxAttribute.voxClass_ = VoxClass::M_DIRECT_EDGE;

    // The voxel PPI is the most represented PPI among the points inside it //
    const auto& maxScore = std::max_element(voxScore.begin(), voxScore.end());
    voxAttribute.voxPPI_ = static_cast<size_t>(std::distance(voxScore.begin(), maxScore));
}

void PPISegmenter::computeExtendedScore(std::vector<size_t>& voxExtendedScore, const std::vector<size_t>& ADJ_List,
                                                  const std::vector<VoxelAttribute>& voxAttributeList) {
    std::fill(voxExtendedScore.begin(), voxExtendedScore.end(), 0);
    for (const auto& voxelIndex : ADJ_List) {
        for (size_t k = 0; k < p_->projectionPlaneCount; ++k) {
            voxExtendedScore[k] += voxAttributeList[voxelIndex].voxScore_[k];
        }
    }
}

// TODO(lf)warning : adjacent (old name) voxel contain the voxel itself!
void PPISegmenter::updateAdjacentVoxelsClass(std::vector<VoxelAttribute>& voxAttributeList,
                                                       const std::vector<size_t>& voxExtendedScore,
                                                       const std::vector<size_t>& IDEV_List) {
    // Common and effective way to find the index of the maximum element in a C++ container
    const auto& maxScoreSmooth = std::max_element(voxExtendedScore.begin(), voxExtendedScore.end());
    const size_t ppiOfScoreSmooth = std::distance(voxExtendedScore.begin(), maxScoreSmooth);

    for (const auto& voxelIndex : IDEV_List) {
        VoxelAttribute& adjVoxAttribute = voxAttributeList[voxelIndex];
        if (adjVoxAttribute.voxClass_ == VoxClass::NO_EDGE && adjVoxAttribute.voxPPI_ != ppiOfScoreSmooth) {
            adjVoxAttribute.voxClass_ = VoxClass::INDIRECT_EDGE;
        }
    }
}

inline bool PPISegmenter::checkNEV(const VoxClass voxClass, const size_t voxPPI,
                                             const std::vector<size_t>& voxExtendedScore) {
    // TODO(lf): why not to check if S_DIRECT_EDGE ?

    if (voxClass == VoxClass::M_DIRECT_EDGE) {  // TMC2 : VoxClass::S_DIRECT_EDGE or VoxClass::INDIRECT_EDGE
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
inline void PPISegmenter::refinePointsPPIs(std::vector<size_t>& pointsPPIs, const std::vector<size_t>& pointsIndices,
                                                     const double weight, const std::vector<size_t>& voxExtendedScore) const {
    std::vector<double> weightedScoreSmooth(p_->projectionPlaneCount);
    for (size_t k = 0; k < p_->projectionPlaneCount; ++k) {
        weightedScoreSmooth[k] = weight * static_cast<double>(voxExtendedScore[k]);
    }

    // For each point in the current voxel //
    for (const auto& pointIndex : pointsIndices) {
        const auto& normal = pointsNormals_[pointIndex];
        double scoreMax = weightedScoreSmooth[0] + dotProduct(normal, p_->projectionPlaneOrientations[0]);
        size_t PPIscoreMax = 0;
        for (size_t k = 1; k < p_->projectionPlaneCount; ++k) {
            double const score = weightedScoreSmooth[k] + dotProduct(normal, p_->projectionPlaneOrientations[k]);
            if (score > scoreMax) {
                scoreMax = score;
                PPIscoreMax = k;
            }
        }
        pointsPPIs[pointIndex] = PPIscoreMax;
    }
}

void PPISegmenter::voxelizationWithBitArray(const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& inputPointsGeometry,
                                            std::vector<bool>& occFlagArray, std::unordered_map<size_t, size_t>& voxelIdxMap,
                                            std::vector<size_t>& filledVoxels,
                                            std::vector<std::vector<size_t>>& pointListInVoxels) {
    const size_t voxelizationShift =
        p_->geoBitDepthVoxelized - p_->geoBitDepthRefineSegmentation;  // i.e. : 9 - 8 = 1 (meaning 2x2x2 voxel dimension)

    // TODO(lf): this is a simple heuristic from longdress (10->9->8). At least round up to the power of 2
    // (as it will already be done by the reserve function)
    const size_t estimatedVoxelCount = 3U * (inputPointsGeometry.size() >> (voxelizationShift * 3U));
    voxelIdxMap.reserve(estimatedVoxelCount);
    filledVoxels.reserve(estimatedVoxelCount);
    pointListInVoxels.reserve(estimatedVoxelCount);

    const size_t estimatedPointsPerVoxel = 4;  // TODO(lf): verify and make it dependent on the chosen geoBitDepth

    // TODO(lf): the "reserve" of the pointListInVoxels is probably inneficient. As for each vector inside the vector, we call reserve. Would it
    // be better to reserve the main vector with constant sized (already allocated) smaller vectors and then call resize(0) on each of them
    // before to add the first point ?

    size_t vox_id = 0;
    for (size_t point_idx = 0; point_idx < inputPointsGeometry.size(); ++point_idx) {
        const uvgvpcc_enc::Vector3<typeGeometryInput>& inputPoint = inputPointsGeometry[point_idx];

        const size_t vx = inputPoint[0] >> voxelizationShift;
        const size_t vy = inputPoint[1] >> voxelizationShift;
        const size_t vz = inputPoint[2] >> voxelizationShift;
        const size_t pos_1D = vx + (vy << p_->geoBitDepthRefineSegmentation) + (vz << (p_->geoBitDepthRefineSegmentation << 1U));

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

// TODO(lf): tackle the cognitive complexity
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void PPISegmenter::fillNeighborAndAdjacentLists(
    std::vector<size_t>& filledVoxels, std::vector<bool>& occFlagArray, std::unordered_map<size_t, size_t>& voxelIdxMap,
    std::vector<std::vector<size_t>>& ADJ_List, std::vector<std::vector<size_t>>& IDEV_List,
    std::vector<std::vector<size_t>>& pointListInVoxels, std::vector<double>& voxWeightListOptimPaper,
    std::vector<VoxelAttribute>& voxAttributeList, const std::vector<size_t>& pointsPPIs) {
    const typeGeometryInput gridMaxAxisValue = (1U << p_->geoBitDepthRefineSegmentation) - 1;
    // TODO(lf): verify this above minus 1 is correct and that it is done everywhere it is needed
    for (size_t v_idx = 0; v_idx < filledVoxels.size(); ++v_idx) {
        // Iterate through all voxels to set score, classification and voxel PPI //
        // First classification : NE-V or DE-V (SDE-V or MDE-V) //
        VoxelAttribute& voxAttribute = voxAttributeList[v_idx];
        if (pointListInVoxels[v_idx].size() == 1) {
            // Single Direct Edge Voxel : One point in the voxel //
            voxAttribute.voxClass_ = VoxClass::S_DIRECT_EDGE;
        }
        updateVoxelAttribute(voxAttribute, pointListInVoxels[v_idx], pointsPPIs);

        const size_t cur_pos_1D = filledVoxels[v_idx];
        // find valid 3D search range centered on cur_pos_1D //
        size_t num_nn_points = 0;  // The number of points within neighboring voxels

        // Inverse of this operation : const size_t pos_1D = x + (y << p_->geoBitDepthRefineSegmentation) + (z <<
        // (p_->geoBitDepthRefineSegmentation * 2)); For  p_->geoBitDepthRefineSegmentation==8, it is 00000000 00000000 00000000 00000000
        // 00000000 00000000 00000000 11111111
        const size_t bitMask = (1U << p_->geoBitDepthRefineSegmentation) - 1;
        const typeGeometryInput z = cur_pos_1D >> (p_->geoBitDepthRefineSegmentation * 2);
        const typeGeometryInput y = (cur_pos_1D >> p_->geoBitDepthRefineSegmentation) & bitMask;
        const typeGeometryInput x = cur_pos_1D & bitMask;

        const uvgvpcc_enc::Vector3<typeGeometryInput> currentPoint = {x, y, z};

        // TODO(lf): find a way to directly add cur_pos_1D and pointAdjLocation1D, and then check if it is a valid point without extracting the
        // x y and z values

        const size_t distanceSearch = p_->refineSegmentationMaxNNVoxelDistanceLUT;
        for (size_t dist = 0; dist < distanceSearch; ++dist) {  // dist is squared distance
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

                const size_t pointAdjLocation1D = pointAdj[0] + (pointAdj[1] << p_->geoBitDepthRefineSegmentation) +
                                                       (pointAdj[2] << (p_->geoBitDepthRefineSegmentation * 2));

                if (occFlagArray[pointAdjLocation1D]) {
                    const size_t neighbor_v_idx = voxelIdxMap.at(pointAdjLocation1D);
                    ADJ_List[v_idx].push_back(
                        neighbor_v_idx);  // TODO(lf): do a big check everywhere because here adjacent and neighbor are inverted

                    const size_t IDEV_range = 3;  // TODO(lf)justifiy thise value, and make it dependent on the geobitdepth
                    if (dist <= IDEV_range) {
                        IDEV_List[v_idx].push_back(neighbor_v_idx);
                    }

                    num_nn_points += pointListInVoxels[neighbor_v_idx].size();
                    if (num_nn_points >= p_->refineSegmentationMaxNNTotalPointCount) {
                        break;
                    }
                }
                if (num_nn_points >= p_->refineSegmentationMaxNNTotalPointCount) {
                    break;
                }
            }
        }

        voxWeightListOptimPaper[v_idx] =
            p_->refineSegmentationLambda / static_cast<double>(num_nn_points);  // NOLINT(clang-analyzer-core.DivideZero)
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

void PPISegmenter::refineSegmentation(std::vector<size_t>& pointsPPIs, const size_t& frameId) {
    uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::TRACE, "PATCH GENERATION",
                             "Refine segmentation of frame " + std::to_string(frameId) + "\n");
    // One boolean for each voxel of the grid, indicating if a voxel is filled or not //
    const size_t gridMaxAxisValue = (1U << p_->geoBitDepthRefineSegmentation);
    std::vector<bool> occFlagArray(gridMaxAxisValue * gridMaxAxisValue * gridMaxAxisValue, false);
    std::unordered_map<size_t, size_t> voxelIdxMap;  // location1D -> index in voxel list (filledVoxels)
    std::vector<size_t> filledVoxels;                     // list of location1D
    std::vector<std::vector<size_t>> pointListInVoxels;   // for each voxel, the list of the index of the points inside
    voxelizationWithBitArray(pointsGeometry_, occFlagArray, voxelIdxMap, filledVoxels, pointListInVoxels);

    const size_t voxelCount = filledVoxels.size();
    std::vector<VoxelAttribute> voxAttributeListOptimPaper(voxelCount, VoxelAttribute(p_->projectionPlaneCount));
    std::vector<std::vector<size_t>> ADJ_List(voxelCount);   // large    // This is voxNeighborsList
    std::vector<std::vector<size_t>> IDEV_List(voxelCount);  // small    // This is voxAdjacentsList
    std::vector<double> voxWeightListOptimPaper(voxelCount);
    fillNeighborAndAdjacentLists(filledVoxels, occFlagArray, voxelIdxMap, ADJ_List, IDEV_List, pointListInVoxels,
                                          voxWeightListOptimPaper, voxAttributeListOptimPaper, pointsPPIs);

    // TODO(lf): find a way to break the refine segmentation iteration before reaching the number of iteration parameter (if number of updated
    // voxel lower than something for example)

    // TODO(lf): in the for loop over all voxel, we access a lot of list to get the related voxel element. Why not TODO(lf)a structure voxel with
    // everything at the same memory location and so reduce memory call ?
    for (size_t iter = 0; iter < p_->refineSegmentationIterationCount; ++iter) {
        for (size_t voxelIndex = 0; voxelIndex < voxelCount; ++voxelIndex) {
            // TODO(lf): should we use a stack of voxel index instead of a for loop with a lot of if(true) ?
            const VoxClass& voxClass = voxAttributeListOptimPaper[voxelIndex].voxClass_;

            if (voxClass == VoxClass::NO_EDGE) {
                continue;  // This voxel has been marked as NE-V before the current iteration //
            }

            std::vector<size_t> voxExtendedScore(p_->projectionPlaneCount, 0);
            computeExtendedScore(voxExtendedScore, ADJ_List[voxelIndex], voxAttributeListOptimPaper);
            updateAdjacentVoxelsClass(voxAttributeListOptimPaper, voxExtendedScore, IDEV_List[voxelIndex]);
            if (checkNEV(voxClass, voxAttributeListOptimPaper[voxelIndex].voxPPI_, voxExtendedScore)) {
                continue;  // The current iteration found that this voxel is NE-V //
            }

            // The voxel is not NE-V, so it is D-EV or IDE-V and its points PPI can be refined //
            refinePointsPPIs(pointsPPIs, pointListInVoxels[voxelIndex], voxWeightListOptimPaper[voxelIndex], voxExtendedScore);
            voxAttributeListOptimPaper[voxelIndex].updateFlag_ = true;
        }

        // Update voxel classification and scores if points PPI inside have changed during the iteration //
        for (size_t voxelIndex = 0; voxelIndex < voxelCount; ++voxelIndex) {
            // TODO(lf): it might be faster to use a stack of index instead of using a flag
            if (!voxAttributeListOptimPaper[voxelIndex].updateFlag_) {
                continue;
            }
            voxAttributeListOptimPaper[voxelIndex].updateFlag_ = false;
            std::fill(voxAttributeListOptimPaper[voxelIndex].voxScore_.begin(), voxAttributeListOptimPaper[voxelIndex].voxScore_.end(), 0);
            updateVoxelAttribute(voxAttributeListOptimPaper[voxelIndex], pointListInVoxels[voxelIndex], pointsPPIs);
        }
    }

    if (p_->exportIntermediatePointClouds) {
        const std::string plyFilePath =
            p_->intermediateFilesDir + "/refineSegmentation/REFINE-SEGMENTATION_f-" + uvgvpcc_enc::zeroPad(frameId, 3) + ".ply";
        std::vector<uvgvpcc_enc::Vector3<uint8_t>> attributes(pointsGeometry_.size());
        for (size_t pointIndex = 0; pointIndex < pointsGeometry_.size(); ++pointIndex) {
            attributes[pointIndex] = ppiColors[pointsPPIs[pointIndex]];
        }
        exportPointCloud(plyFilePath, pointsGeometry_, attributes);
    }
}