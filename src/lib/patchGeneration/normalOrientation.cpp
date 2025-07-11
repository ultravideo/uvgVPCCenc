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

/// \file Entry point for orienting the normals of a point cloud frame.

#include "normalOrientation.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <queue>
#include <string>
#include <vector>
#include <cstdint>

#include "utilsPatchGeneration.hpp"
#include "uvgvpcc/log.hpp"
#include "uvgvpcc/uvgvpcc.hpp"
#include "utils/utils.hpp"


using namespace uvgvpcc_enc;

namespace NormalOrientation {

struct WeightedEdge {
    double weight_;
    size_t start_;  // TODO(lf)be sure it is pc indices
    size_t end_;

    explicit WeightedEdge(double w, size_t s, size_t e) : weight_(w), start_(s), end_(e) {};

    bool operator<(const WeightedEdge& rhs) const {
        if (weight_ == rhs.weight_) {  // TODO(lf): is it really usefull ?
            return start_ == rhs.start_ ? end_ < rhs.end_ : start_ < rhs.start_;
        }
        return weight_ < rhs.weight_;
    }
};


namespace {
void addNeighborsSeed(const std::vector<uvgvpcc_enc::Vector3<double>>& normals, const size_t currentIdx,
                      const std::vector<std::vector<size_t>>& pointsNNList, uvgvpcc_enc::Vector3<double>& accumulatedNormals,
                      size_t& numberOfNormals, const size_t nnCount, const std::vector<bool>& visited,
                      std::priority_queue<WeightedEdge>& edges) {
    // warning : we use the hypothesis that the knn search always return the query point as the first indexed point (index=0). This query
    // point is always visited at this moment. TODO(lf): this may change in the future
    for (size_t i = 1; i < nnCount; ++i) {  // TODO(lf) use auto or other structure, as the "i" is not used
        // size_t index = nnIndices[i];
        size_t const index = pointsNNList[currentIdx][i];
        if (visited[index]) {
            accumulatedNormals += normals[index];
            ++numberOfNormals;
        } else {
            edges.emplace(fabs(dotProduct(normals[currentIdx], normals[index])), currentIdx, index);
        }
    }
}

void addNeighbors(const std::vector<uvgvpcc_enc::Vector3<double>>& normals, const size_t currentIdx,
                  const std::vector<std::vector<size_t>>& pointsNNList, const size_t nnCount, const std::vector<bool>& visited,
                  std::priority_queue<WeightedEdge>& edges) {
    // warning : we use the hypothesis that the knn search always return the query point as the first indexed point (index=0). This query
    // point is always visited when this function is called. TODO(lf): this may change in the future
    for (size_t i = 1; i < nnCount; ++i) {
        size_t const index = pointsNNList[currentIdx][i];
        if (!visited[index]) {
            edges.emplace(fabs(dotProduct(normals[currentIdx], normals[index])), currentIdx, index);
        }
    }
}
}  // anonymous namespace

void orientNormals(const std::shared_ptr<uvgvpcc_enc::Frame>& frame, std::vector<uvgvpcc_enc::Vector3<double>>& normals,
                   const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry,
                   const std::vector<std::vector<size_t>>& pointsNNList) {
    uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::TRACE>("PATCH GENERATION",
                             "Normal orientation of frame " + std::to_string(frame->frameId) + "\n");

    std::vector<bool> visited(pointsGeometry.size());
    std::fill(visited.begin(), visited.end(), false);
    std::priority_queue<WeightedEdge> edges;
    // TODO(lf): may or not be the best data structure for our case. Need some profiling.

    for (size_t ptIndex = 0; ptIndex < pointsGeometry.size(); ++ptIndex) {
        if (visited[ptIndex]) {
            continue;
        }
        visited[ptIndex] = true;
        size_t numberOfNormals = 0;
        uvgvpcc_enc::Vector3<double> accumulatedNormals = {0.0, 0.0, 0.0};
        addNeighborsSeed(normals, ptIndex, pointsNNList, accumulatedNormals, numberOfNormals, p_->normalOrientationKnnCount, visited, edges);

        if (numberOfNormals == 0U) {
            // No already visited surrounding points. Serve as seed. Always the first point. Can also be other points when the whole point
            // cloud is discontinued (the basketball for example)
            const uvgvpcc_enc::Vector3<double> viewPoint{0, 0, 0};
            accumulatedNormals = (viewPoint - pointsGeometry[ptIndex]);
        }

        if (dotProduct(normals[ptIndex], accumulatedNormals) < 0.0) {
            normals[ptIndex] = -normals[ptIndex];
        }

        while (!edges.empty()) {
            WeightedEdge const edge = edges.top();
            edges.pop();
            size_t const current = edge.end_;
            if (!visited[current]) {
                visited[current] = true;
                if (dotProduct(normals[edge.start_], normals[current]) < 0.0) {
                    normals[current] = -normals[current];
                }
                addNeighbors(normals, current, pointsNNList, p_->normalOrientationKnnCount, visited, edges);
            }
        }
    }

    if (p_->exportIntermediatePointClouds) {
        const std::string plyFilePath =
            p_->intermediateFilesDir + "/normalOrientation/NORMAL-ORIENTATION_f-" + uvgvpcc_enc::zeroPad(frame->frameNumber, 3) + ".ply";
        if (p_->geoBitDepthVoxelized == p_->geoBitDepthInput) {
            exportPointCloud(plyFilePath, pointsGeometry, frame->pointsAttribute, normals);
        } else {
            const std::vector<uvgvpcc_enc::Vector3<uint8_t>> attributes(pointsGeometry.size(), {128, 128, 128});
            exportPointCloud(plyFilePath, pointsGeometry, attributes, normals);
        }
    }
}

}  // namespace NormalOrientation