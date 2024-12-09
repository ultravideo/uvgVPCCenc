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

#include "kdTree.hpp"

// TODO(lf): nanoflann is currently returning distances as double, even if they are integer value
// TODO(lf): do not order the results of the knn (maybe useless step)
// TODO(lf): the NN indices return the index of the query point, so only 15nn instead of 16 or at least useless computation somewhere ? Start
// iterating at 1 instead of 1 ?

KdTree::KdTree(const size_t& kdTreeMaxLeafSize, const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry)
    : index_(std::make_unique<nanoflannAdaptorType>(3 /*dim*/, pointsGeometry, kdTreeMaxLeafSize /* max leaf */)) {}

void KdTree::knn(const uvgvpcc_enc::Vector3<typeGeometryInput>& queryPoint, const size_t nnCount,
                 std::vector<size_t>& nnIndices) const {
    std::vector<int16_t> out_dists_sqr(nnCount);
    nanoflann::KNNResultSet<int16_t, size_t, size_t> resultSet(nnCount);
    resultSet.init(nnIndices.data(), out_dists_sqr.data());
    const double queryPointDouble[3] = {static_cast<double>(queryPoint[0]), static_cast<double>(queryPoint[1]),
                                        static_cast<double>(queryPoint[2])};
    index_->index->findNeighbors(resultSet, queryPointDouble);
}

// TODO(lf)check with tmc2 if squared or not distance is necessary (should change power 2 the parameter instead of computing square root of
// every distance)
void KdTree::knnDist(const uvgvpcc_enc::Vector3<typeGeometryInput>& queryPoint, const size_t nnCount,
                     std::vector<int16_t>& out_dists_sqr) const {
    std::vector<size_t> nnIndices(nnCount);
    nanoflann::KNNResultSet<int16_t, size_t, size_t> resultSet(nnCount);
    resultSet.init(nnIndices.data(), out_dists_sqr.data());
    const double queryPointDouble[3] = {static_cast<double>(queryPoint[0]), static_cast<double>(queryPoint[1]),
                                        static_cast<double>(queryPoint[2])};
    index_->index->findNeighbors(resultSet, queryPointDouble);
}


/* void KdTree::knnRadius(const uvgvpcc_enc::Vector3<typeGeometryInput>& queryPoint, const int16_t maxNNCount, const uint16_t radius,
                       std::vector<size_t>& nnIndices) const {
    std::vector<int16_t> out_dists_sqr(maxNNCount);
    nanoflann::KNNResultSet<int16_t, uint32_t, uint8_t> resultSet(maxNNCount);
    resultSet.init(nnIndices.data(), out_dists_sqr.data());
    const double queryPointDouble[3] = {static_cast<double>(queryPoint[0]), static_cast<double>(queryPoint[1]),
                                        static_cast<double>(queryPoint[2])};

    nanoflann::SearchParameters sParams;
    std::vector<nanoflann::ResultItem<size_t, double>> indicesDists;
    size_t retSize = index_->index->radiusSearch(queryPointDouble, static_cast<double>(radius), indicesDists, sParams);
    // TODO(lf): why can't we use uint8 instead of size_t for retSize ?
    if (retSize > static_cast<size_t>(maxNNCount)) {
        retSize = static_cast<size_t>(maxNNCount);
    }

    nnIndices.resize(retSize);
    for (size_t i = 0; i < retSize; ++i) {
        nnIndices[i] = static_cast<size_t>(indicesDists[i].first);
    }
}*/
