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

/// \file Set of tools necessary for the patch generation process.

#pragma once

#include "utils/utils.hpp"
#include "uvgvpcc/log.hpp"
#include <cstdint>
#include <fstream>
#include <vector>
#include "robin_hood.h"

using namespace uvgvpcc_enc;


// TODO(lf): how to handle this type of lut table variable ?
// NOLINTNEXTLINE(cert-err58-cpp)
static const std::array<std::vector<Vector3<int32_t>>, 9> adjacentPointsSearch = {{
    // Adjacent shift for squared distance 1
    {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}},
    // Adjacent shift for squared distance 2
    {{1, 1, 0},
     {1, -1, 0},
     {-1, 1, 0},
     {-1, -1, 0},
     {0, 1, 1},
     {0, 1, -1},
     {0, -1, 1},
     {0, -1, -1},
     {1, 0, 1},
     {-1, 0, 1},
     {1, 0, -1},
     {-1, 0, -1}},
    // Adjacent shift for squared distance 3
    {{1, 1, 1}, {1, 1, -1}, {1, -1, 1}, {1, -1, -1}, {-1, 1, 1}, {-1, 1, -1}, {-1, -1, 1}, {-1, -1, -1}},
    // Adjacent shift for squared distance 4
    {{2, 0, 0}, {-2, 0, 0}, {0, 2, 0}, {0, -2, 0}, {0, 0, 2}, {0, 0, -2}},
    // Adjacent shift for squared distance 5
    {{2, 1, 0}, {2, -1, 0}, {1, 2, 0}, {1, -2, 0}, {-1, 2, 0}, {-1, -2, 0}, {-2, 1, 0}, {-2, -1, 0},
     {0, 2, 1}, {0, 2, -1}, {0, 1, 2}, {0, 1, -2}, {0, -1, 2}, {0, -1, -2}, {0, -2, 1}, {0, -2, -1},
     {1, 0, 2}, {-1, 0, 2}, {2, 0, 1}, {-2, 0, 1}, {2, 0, -1}, {-2, 0, -1}, {1, 0, -2}, {-1, 0, -2}},
    // Adjacent shift for squared distance 6
    {{2, 1, 1},   {2, 1, -1},   {2, -1, 1},  {2, -1, -1},  {1, 2, 1},  {1, 2, -1},  {1, 1, 2},   {1, 1, -2},
     {1, -1, 2},  {1, -1, -2},  {1, -2, 1},  {1, -2, -1},  {-1, 2, 1}, {-1, 2, -1}, {-1, 1, 2},  {-1, 1, -2},
     {-1, -1, 2}, {-1, -1, -2}, {-1, -2, 1}, {-1, -2, -1}, {-2, 1, 1}, {-2, 1, -1}, {-2, -1, 1}, {-2, -1, -1}},
    // Adjacent shift for squared distance 7 (does not exist in an integer grid)
    {},
    // Adjacent shift for squared distance 8
    {{2, 2, 0},
     {2, -2, 0},
     {-2, 2, 0},
     {-2, -2, 0},
     {0, 2, 2},
     {0, 2, -2},
     {0, -2, 2},
     {0, -2, -2},
     {2, 0, 2},
     {-2, 0, 2},
     {2, 0, -2},
     {-2, 0, -2}},
    // Adjacent shift for squared distance 9
    {{3, 0, 0},   {-3, 0, 0},   {0, 3, 0},  {0, -3, 0},  {0, 0, 3},  {0, 0, -3},  {2, 2, 1},   {2, 2, -1},   {2, 1, 2},   {2, 1, -2},
     {2, -1, 2},  {2, -1, -2},  {2, -2, 1}, {2, -2, -1}, {1, 2, 2},  {1, 2, -2},  {1, -2, 2},  {1, -2, -2},  {-1, 2, 2},  {-1, 2, -2},
     {-1, -2, 2}, {-1, -2, -2}, {-2, 2, 1}, {-2, 2, -1}, {-2, 1, 2}, {-2, 1, -2}, {-2, -1, 2}, {-2, -1, -2}, {-2, -2, 1}, {-2, -2, -1}},
}};

template <typename T, typename TT>
inline double dotProduct(const std::array<T, 3>& arr1, const std::array<TT, 3>& arr2) {
    return arr1[0] * arr2[0] + arr1[1] * arr2[1] + arr1[2] * arr2[2];
}


// Hash function for vector3
template <typename T>
struct vector3Hash {
    size_t operator()(const Vector3<T>& vector) const {
        std::hash<T> hasher;
        size_t hash = 0;
        for (const auto& elem : vector) {
            // Use a simple hash combining algorithm
            hash ^= hasher(elem) + 0x9e3779b9 + (hash << 6U) + (hash >> 2U);
        }
        return hash;
    }
};






// The voxelization is done so to have those three vectors sharing the same order (that is sharing the same voxel index) :
// -> A list of voxel (vector of coordinates)
// -> A list of list of points (vector of vector of point index). This associate a voxel with the points inside it.
// -> A list of voxel PPI (vector of PPI (int))
//
// Example with a voxel index i:
// voxelizedPointsGeometry[i] stores the coordinate of the voxel i
// voxelIdToPointsId[i] stores the index of the points inside the voxel i
// voxelsPPIs[i] stores the PPI of the voxel i
//
// The use of a map is needed as several points can be inside one voxel. We want to find in a fast way if the current point resides in a voxel
// that has already been created. For an input point j:       voxelCoordToVoxelIndex[  voxelCoordsOf(j) ] = i
//
// This structure makes computation easier for the patch generation and for applying voxel data to points data.
// To achieve this, we need a temporary map that interfaces the voxel coordinates with the voxel index in those three list.

// voxelizedPointsGeometry -> list of the voxels. That is, a list of the coordinates of the voxels.
// voxelIdToPointsId -> For each voxel, stores the index of the points inside. A voxel is defined by a voxel index, that is given by the
// filling order of the vector. Thus, the third voxel to be added to this structure will have a voxel index of 2.
// voxelCoordToVoxelIndex -> Temporary structure. Associate a voxel to its index in 'voxelIdToPointsId' (its voxel index). This structure is a
// map whose key is a voxel coordinate and the value is a voxel index.
//
// For each point if the input point cloud :
//     Compute the coordinates of the voxel inside which the current point should be.
//     If those voxel coordinates are already in the map 'voxelCoordToVoxelIndex':
//         Add the current input point index to the list of points index of the found voxel.
//         That is, add the current input point index to the list of points of the voxel in 'voxelIdToPointsId'
//     If those voxel coordinates are not in the map 'voxelCoordToVoxelIndex':
//         Create a new item in the map 'voxelCoordToVoxelIndex'. This associates the voxel coordinates to its voxel index (the current size
//         of the list of voxel === the index of this voxel in the list of voxel). Add a new voxel to 'voxelIdToPointsId'. This associates the
//         voxel index (the current size of the list of voxel === the index of this voxel in the list of voxel) to an empty list of input
//         point index. Create a new voxel by adding the voxel coordinates in the voxel list 'voxelizedPointsGeometry'. Add the current input
//         point index to the list of points index of the new voxel. That is, add the current input point index to the list of points of the
//         voxel in 'voxelIdToPointsId'


// Notice that in the refined segmentation, the inputPointsGeometry can be the voxelizedPointsGeometry. Indeed, when the voxelization is
// activated, the refined segmentation is applying a second voxelization step. The created voxelized point cloud is then the result of two
// voxelizations.
inline void voxelization(
    const std::vector<Vector3<typeGeometryInput>>& inputPointsGeometry,
    std::vector<Vector3<typeGeometryInput>>& voxelizedPointsGeometry,
    std::vector<std::vector<size_t>>& voxelIdToPointsId,
    const size_t inputBitResolution,
    const size_t outputBitResolution) 
{
    Logger::log<LogLevel::TRACE>("PATCH GENERATION",
        "Voxelization from " + std::to_string(inputBitResolution) + " to " + std::to_string(outputBitResolution) + " bits of resolution.\n");

    const size_t voxelizationShift = inputBitResolution - outputBitResolution;
    const typeGeometryInput shift = static_cast<typeGeometryInput>(voxelizationShift);

    const size_t approximateVoxelCount = 1U << (outputBitResolution * 2U);
    voxelizedPointsGeometry.reserve(approximateVoxelCount);
    voxelIdToPointsId.reserve(approximateVoxelCount);

    robin_hood::unordered_map<Vector3<typeGeometryInput>, size_t, vector3Hash<typeGeometryInput>> voxelCoordToVoxelIndex;
    voxelCoordToVoxelIndex.reserve(approximateVoxelCount);

    const size_t inputSize = inputPointsGeometry.size();
    for (size_t inputPointIndex = 0; inputPointIndex < inputSize; ++inputPointIndex) {
        const Vector3<typeGeometryInput> & inputPoint = inputPointsGeometry[inputPointIndex];

        Vector3<typeGeometryInput> voxCoord{
            static_cast<typeGeometryInput>(static_cast<uint32_t>(inputPoint[0]) >> shift),
            static_cast<typeGeometryInput>(static_cast<uint32_t>(inputPoint[1]) >> shift),
            static_cast<typeGeometryInput>(static_cast<uint32_t>(inputPoint[2]) >> shift)
        };

        size_t voxelIndex = voxelizedPointsGeometry.size();
        auto [it, inserted] = voxelCoordToVoxelIndex.try_emplace(voxCoord, voxelIndex);
        if (inserted) {
            voxelizedPointsGeometry.emplace_back(voxCoord);
            voxelIdToPointsId.emplace_back();
            voxelIdToPointsId.back().reserve(16);
        }

        voxelIdToPointsId[it->second].push_back(inputPointIndex);
    }
}
