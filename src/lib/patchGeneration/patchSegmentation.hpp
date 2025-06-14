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

#pragma once

#include "robin_hood.h"
#include "uvgvpcc/uvgvpcc.hpp"

using namespace uvgvpcc_enc;


class PatchSegmentation {
   public:
    PatchSegmentation();

    static void patchSegmentation(const std::shared_ptr<uvgvpcc_enc::Frame>& frame, const std::vector<size_t>& pointsPPIs);
    
    static void createConnectedComponents(std::vector<std::vector<size_t>>& connectedComponents, std::vector<bool>& flags,
                                             const std::vector<size_t>& rawPoints, const std::vector<size_t>& pointsPPIs,
                                             robin_hood::unordered_map<size_t, size_t>& nnPropagationMapFlagTrue,
                                             const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry);                                             
    static void patchSplitting(std::vector<size_t>& connectedComponent, uvgvpcc_enc::Patch& patch,
                               const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry);
    static void computePatchBoundingBox(uvgvpcc_enc::Patch& patch, const std::vector<size_t>& connectedComponent,
                                        const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry);
    static void computePatchDepthL1(uvgvpcc_enc::Patch& patch, const std::vector<size_t>& connectedComponent,
                                    std::vector<size_t>& patchPartition,
                                    const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry, const bool isProjectionMode0);
    static void computePatchDepthL2(uvgvpcc_enc::Patch& patch, const std::vector<size_t>& connectedComponent,
                                    const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry, const bool isProjectionMode0);
    static void filterDepth(uvgvpcc_enc::Patch& patch, const bool isProjectionMode0);
    static void resampledPointcloud(robin_hood::unordered_set<size_t>& resamplePointSet, uvgvpcc_enc::Patch& patch);  // TODO(lf)const patch ?

    static void computeAdditionalPatchInfo(uvgvpcc_enc::Patch& patch);
    static void refillRawPoints(const robin_hood::unordered_set<size_t>& resamplePointSet, std::vector<size_t>& rawPoints,
                                   const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry, const size_t& pointCount,
                                   std::vector<bool>& flags, robin_hood::unordered_map<size_t, size_t>& nnPropagationMapFlagTrue);                                   

    
};
