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

 #pragma once
#include <fstream>
#include <mutex>
#include <string>
#include <cstdint>
#include <vector>
#include <iostream>

enum class DataId {
    NumberOfPoints,
    NumberOfVoxels,
    NumberOfVoxelsRS,
    SkippedVoxels,
    NoEdge,
    IndirectEdge,
    SingleEdge,
    MultiEdge,
    ScoreComputations,
    NoEdge_R,
    IndirectEdge_R,
    SingleEdge_R,
    MultiEdge_R,
    PpiChange,
    NumberOfPatches,
    NumberOfLostPoints
};

struct uvgVPCCencStats {
        /*--------- General ---------*/
            // Number of points of the point cloud
    size_t NumberOfPoints;
            // Number of voxels (1st voxelization)
    size_t numberOfVoxels;
        /*--- Refine Segmentation ---*/
            // Number of voxel for RS (2nd voxelization)
    size_t numberOfVoxelsRS;
            // Number of skipped voxel per iteration
    std::vector<size_t> skippedVoxels;
                // 1 : 15000
                // 2 : 16000
                // ...
            // Classes histogram
    std::vector<size_t> NoEdge;
    std::vector<size_t> IndirectEdge;
    std::vector<size_t> SingleEdge;
    std::vector<size_t> MultiEdge;
            // Number of score computed
    std::vector<size_t> scoreComputations;
            // Number of computation inside each vox class = number of voxel in which a score computation has been made 
    std::vector<size_t> NoEdge_R;
    std::vector<size_t> IndirectEdge_R;
    std::vector<size_t> SingleEdge_R;
    std::vector<size_t> MultiEdge_R;
            // Number of PPI changes (point's level)
    std::vector<size_t> ppiChange;
                // 1 : 150 000
                // 2 : 120 000
                // ...
        /*--- Patch Segmentation ---*/
            // Number of patches
    size_t numberOfPatches;
            // Number of lost points
    size_t numberOfLostPoints; // due to oclusion
};

class StatsCollector {
public:
    std::vector<uvgVPCCencStats> stats_;
    static StatsCollector& instance() {
        static StatsCollector instance;
        return instance;
    }

    void init(std::size_t nbFrames);

    void collectData(size_t frameId, DataId id, size_t data);   

    // Export
    void writeToFile(const std::string& filename, const size_t gofId) const;

private:
    StatsCollector() = default;
};

extern StatsCollector& stats;
