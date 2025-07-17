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

/// \file Entry point for the whole patch generation process.

#include "patchGeneration.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "normalComputation.hpp"
#include "normalOrientation.hpp"
#include "patchGeneration/kdTree.hpp"
#include "patchSegmentation.hpp"
#include "ppiSegmenter.hpp"
#include "utilsPatchGeneration.hpp"
#include "uvgvpcc/log.hpp"
#include "uvgvpcc/uvgvpcc.hpp"
#include "utils/utils.hpp"

using namespace uvgvpcc_enc;

// TODO(lf): nearestNeighborCount should be static
void PatchGeneration::computePointsNNList(std::vector<std::vector<size_t>>& pointsNNList,
                                          const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry,
                                          const size_t& nnCount) {
    uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::TRACE>("PATCH GENERATION", "computePointsNNList.\n");
    
    KdTree const kdTree(p_->kdTreeMaxLeafSize, pointsGeometry);
    
    // Iterate over all points and find their k Nearest Neighbors //
    // const size_t nearestNeighborCount = std::max(normalComputationKnnCount_, normalOrientationKnnCount_);
    pointsNNList.resize(pointsGeometry.size(), std::vector<size_t>(nnCount));

    for (size_t ptIndex = 0; ptIndex < pointsGeometry.size(); ++ptIndex) {
        kdTree.knn(pointsGeometry[ptIndex], nnCount, pointsNNList[ptIndex]);
    }
}

// lf : This applyVoxelsDataToPoints function is done in the other direction in TMC2 -> Iterating over the input points, computing the related
// voxel coords and finding the voxel PPI through a map(voxelCoord, voxelPPI)
namespace {
inline void applyVoxelsDataToPoints(const std::vector<size_t>& voxelsPPIs, std::vector<size_t>& pointsPPIs,
                             const std::vector<std::vector<size_t>>& voxelIdToPointsId) {
    uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::TRACE>("PATCH GENERATION", "Apply voxel data to points.\n");
    for (size_t voxelIndex = 0; voxelIndex < voxelIdToPointsId.size(); ++voxelIndex) {
        for (size_t pointIndex = 0; pointIndex < voxelIdToPointsId[voxelIndex].size(); ++pointIndex) {
            pointsPPIs[voxelIdToPointsId[voxelIndex][pointIndex]] = voxelsPPIs[voxelIndex];
        }
    }
}
}  // anonymous namespace

void PatchGeneration::generateFramePatches(std::shared_ptr<uvgvpcc_enc::Frame> frame) {
    uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::TRACE>("PATCH GENERATION",
                             "Generate patches for frame " + std::to_string(frame->frameId) + ".\n");
    assert(p_->geoBitDepthInput >= p_->geoBitDepthVoxelized);
    
    // Voxelization //
    const bool useVoxelization = p_->geoBitDepthInput != p_->geoBitDepthVoxelized;                             
    std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>> voxelizedGeometryBuffer;
    std::vector<std::vector<size_t>> voxelIdToPointsId;
    const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& voxelizedPointsGeometry = useVoxelization ? voxelizedGeometryBuffer : frame->pointsGeometry;

    if (useVoxelization) {
        voxelization(frame->pointsGeometry, voxelizedGeometryBuffer, voxelIdToPointsId, p_->geoBitDepthInput,
                            p_->geoBitDepthVoxelized);
    }

    // kdtree init and knn searches //
    std::vector<std::vector<size_t>> pointsNNList;
    computePointsNNList( pointsNNList, voxelizedPointsGeometry,
        std::max(p_->normalComputationKnnCount, p_->normalOrientationKnnCount));

    // Normal computation & orientation //
    std::vector<uvgvpcc_enc::Vector3<double>> pointsNormal(voxelizedPointsGeometry.size());
    NormalComputation::computeNormals(frame, pointsNormal, voxelizedPointsGeometry, pointsNNList);
    NormalOrientation::orientNormals(frame, pointsNormal, voxelizedPointsGeometry, pointsNNList);

    // Projection Plane Index Segmentation //
    std::vector<size_t> voxelsPPIs(voxelizedPointsGeometry.size());
    PPISegmenter ppiSegmenter(voxelizedPointsGeometry, pointsNormal);
    ppiSegmenter.initialSegmentation(voxelsPPIs, frame->frameId);
    ppiSegmenter.refineSegmentation(voxelsPPIs, frame->frameId);
    
    // "De-voxelization"
    std::vector<size_t> pointsPPIsBuffer;
    const std::vector<size_t>& pointsPPIs = useVoxelization ? pointsPPIsBuffer : voxelsPPIs;
    
    if (useVoxelization) {
        pointsPPIsBuffer.resize(frame->pointsGeometry.size());
        applyVoxelsDataToPoints(voxelsPPIs, pointsPPIsBuffer, voxelIdToPointsId);
    }
    
    // Patch segmentation //
    PatchSegmentation::patchSegmentation(frame, pointsPPIs); 

    // Sort patches //
    // Sort patches from the biggest to the smallest // // TODO(lf): might be better to use area ?
    // Notice that after this sorting, the patch Id does not correspond to the position of the patch in the frame.patchList
    std::sort(frame->patchList.begin(), frame->patchList.end(), [](const uvgvpcc_enc::Patch& patchA, const uvgvpcc_enc::Patch& patchB) {
        return std::max(patchA.widthInPixel_, patchA.heightInPixel_) > std::max(patchB.widthInPixel_, patchB.heightInPixel_);
    });

    std::vector<Vector3<typeGeometryInput>>().swap(frame->pointsGeometry); // Release memory
    //TODO(lf): try using noexcept to improve performance

}
