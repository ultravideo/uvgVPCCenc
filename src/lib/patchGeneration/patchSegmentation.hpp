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

#pragma once

#include <unordered_map>
#include <unordered_set>

#include "uvgvpcc/uvgvpcc.hpp"

using namespace uvgvpcc_enc;


class PatchSegmentation {
   public:
    PatchSegmentation();

    static void patchSegmentation(std::shared_ptr<uvgvpcc_enc::Frame>& frame, const std::vector<std::size_t>& pointsPPIs);
    static void createConnectedComponentsLUT(std::vector<std::vector<std::size_t>>& connectedComponents, std::vector<bool>& flags,
                                             const std::vector<std::size_t>& rawPoints, const std::vector<std::size_t>& pointsPPIs,
                                             std::unordered_map<std::size_t, std::size_t>& nnPropagationMapFlagTrue,
                                             const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry);
    static void patchSplitting(std::vector<std::size_t>& connectedComponent, uvgvpcc_enc::Patch& patch,
                               const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry);
    static void computePatchBoundingBox(uvgvpcc_enc::Patch& patch, const std::vector<std::size_t>& connectedComponent,
                                        const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry);
    static void computePatchDepthL1(uvgvpcc_enc::Patch& patch, const std::vector<std::size_t>& connectedComponent,
                                    std::vector<std::size_t>& patchPartition,
                                    const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry, const bool isProjectionMode0);
    static void computePatchDepthL2(uvgvpcc_enc::Patch& patch, const std::vector<std::size_t>& connectedComponent,
                                    const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry, const bool isProjectionMode0);
    static void filterDepth(uvgvpcc_enc::Patch& patch, const bool isProjectionMode0);
    static void resampledPointcloudLUT(std::unordered_set<std::size_t>& resamplePointSet, uvgvpcc_enc::Patch& patch);  // to do const patch ?

    static void computeAdditionalPatchInfo(uvgvpcc_enc::Patch& patch);
    static void refillRawPointsLUT(const std::unordered_set<std::size_t>& resamplePointSet, std::vector<std::size_t>& rawPoints,
                                   const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry, const std::size_t& pointCount,
                                   std::vector<bool>& flags, std::unordered_map<std::size_t, std::size_t>& nnPropagationMapFlagTrue);

   private:
    
};
