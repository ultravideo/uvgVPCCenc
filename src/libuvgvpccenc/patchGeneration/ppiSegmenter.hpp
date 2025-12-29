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

#pragma once

#include <cstdint>
#include <vector>
#include "patchGeneration/robin_hood.h"
#include "utils/utils.hpp"
#include "uvgvpcc/uvgvpcc.hpp"

using namespace uvgvpcc_enc;


enum class VoxClass : uint8_t {
    NO_EDGE = 0x00,        // one ppi-vaue in a voxel
    INDIRECT_EDGE = 0x01,  // adjcent voxels of M_DIRECT_EDGE, S_DIRECT_EDGE
    M_DIRECT_EDGE = 0x10,  // multiple points && more than two ppi-values in a voxel TODO(lf)verify if typo -> (more than one instead no ?)
    S_DIRECT_EDGE = 0x11   // single-point in a voxel, considered as a direct edge-voxel
};
// TODO(lf): why to distinguish M and S direct edge ?
// TODO(lf): Are S DIRECT EGDE always considered as direct edge ? Even if they share the same PPI as their neighbor ? Does this mean each
// iteration focus on all single  direct edge voxel ?

struct VoxelAttribute {
    bool updateFlag_;
    // size_t nbPoint_;  // TODO(lf): not used ?
    VoxClass voxClass_;
    size_t voxPPI_;
    std::vector<size_t> voxScore_;  // TODO(lf): should be an array ?
    // Voxel score is a PPI histogram : how many points inside the voxel is associated with each projection planes //

    explicit VoxelAttribute(const size_t projectionPlaneCount_);
};

class PPISegmenter {
   public:
    PPISegmenter(const std::vector<Vector3<typeGeometryInput>>& pointsGeometry,
                 const std::vector<Vector3<double>>& pointsNormals);

    void initialSegmentation(const std::shared_ptr<uvgvpcc_enc::Frame>& frame,std::vector<size_t>& pointsPPIs, const size_t& frameId);
    void refineSegmentation(const std::shared_ptr<uvgvpcc_enc::Frame>& frame,std::vector<size_t>& pointsPPIs, const size_t& frameId);

   private:
    static void voxelizationWithBitArray(const std::vector<Vector3<typeGeometryInput>>& inputPointsGeometry,
                                         std::vector<bool>& occFlagArray, robin_hood::unordered_map<size_t, size_t>& voxelIdxMap,
                                         std::vector<size_t>& filledVoxels, std::vector<std::vector<size_t>>& pointListInVoxels);                                         

    static void fillNeighborAndAdjacentLists(
        std::vector<size_t>& filledVoxels, std::vector<bool>& occFlagArray, robin_hood::unordered_map<size_t, size_t>& voxelIdxMap,
        std::vector<std::vector<size_t>>& ADJ_List, std::vector<std::vector<size_t>>& IDEV_List,
        std::vector<std::vector<size_t>>& pointListInVoxels, std::vector<double>& voxWeightList,
        std::vector<VoxelAttribute>& voxAttributeList, const std::vector<size_t>& pointsPPIs);        

    static void computeExtendedScore(std::vector<size_t>& voxExtendedScore, const std::vector<size_t>& ADJ_List,
                                               const std::vector<VoxelAttribute>& voxAttributeList);

    static void updateAdjacentVoxelsClass(std::vector<VoxelAttribute>& voxAttributeList,
                                                    const std::vector<size_t>& voxExtendedScore,
                                                    const std::vector<size_t>& IDEV_List);
    static inline bool checkNEV(const VoxClass voxClass, const size_t voxPPI,
                                          const std::vector<size_t>& voxExtendedScore);

    inline void refinePointsPPIs(std::vector<size_t>& pointsPPIs, const std::vector<size_t>& pointsIndices,
                                           const double weight, const std::vector<size_t>& voxExtendedScore) const;
    static inline void updateVoxelAttribute(VoxelAttribute& voxAttribute, const std::vector<size_t>& voxPoints,
                                                      const std::vector<size_t>& pointsPPIs);
    
    const std::vector<Vector3<double>>& pointsNormals_;
    const std::vector<Vector3<typeGeometryInput>>& pointsGeometry_;
    const typeGeometryInput geoMax_;
    const typeGeometryInput geoRange_;
};
