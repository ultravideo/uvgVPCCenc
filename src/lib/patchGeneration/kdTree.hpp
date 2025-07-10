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

/// \file Interface between uvgVPCCenc and the external nanoflann library.


#pragma once

#include "nanoflann.hpp"
#include "utils/utils.hpp"

using namespace uvgvpcc_enc;

using nanoflannAdaptorType =
    KDTreeVectorOfVectorsAdaptor<std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>, double, 3, nanoflann::metric_L2_Simple, size_t>;

class KdTree {
   public:
    KdTree(const size_t& kdTreeMaxLeafSize, const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry);
    void knn(const uvgvpcc_enc::Vector3<typeGeometryInput>& queryPoint, const size_t nnCount, std::vector<size_t>& nnIndices) const;
    void knnDist(const uvgvpcc_enc::Vector3<typeGeometryInput>& queryPoint, const size_t nnCount,
                 std::vector<int16_t>& out_dists_sqr) const;

    /*void knnRadius(const uvgvpcc_enc::Vector3<typeGeometryInput>& queryPoint, const int16_t maxNNCount, const uint16_t radius,
                    std::vector<size_t>& nnIndices) const;*/

   private:
    std::unique_ptr<nanoflannAdaptorType> const index_;
};