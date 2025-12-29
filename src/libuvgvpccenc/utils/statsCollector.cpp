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


#include "statsCollector.hpp"
#include "parameters.hpp"

using namespace uvgvpcc_enc;

void StatsCollector::init(std::size_t nbFrames) {
    // Init the stats_ container number to the number of frames (take under consideration de loops ?)
    stats_.resize(nbFrames);

    // Resize the vectors to the number of iterations
    size_t refineIterations = p_->refineSegmentationIterationCount;
    for(auto& statContainer : stats_){
        statContainer.skippedVoxels.resize(refineIterations);
        statContainer.ppiChange.resize(refineIterations);
        statContainer.NoEdge.resize(refineIterations);
        statContainer.IndirectEdge.resize(refineIterations);
        statContainer.SingleEdge.resize(refineIterations);
        statContainer.MultiEdge.resize(refineIterations);
        statContainer.scoreComputations.resize(refineIterations);
        statContainer.NoEdge_R.resize(refineIterations);
        statContainer.IndirectEdge_R.resize(refineIterations);
        statContainer.SingleEdge_R.resize(refineIterations);
        statContainer.MultiEdge_R.resize(refineIterations);
    }
}

void StatsCollector::collectData(size_t frameId, DataId id, size_t data) {
    auto& s = stats_[frameId];
    
    switch (id) {
        // Data here is the value 
        case DataId::NumberOfPoints:            s.NumberOfPoints     = data; break;
        case DataId::NumberOfVoxels:            s.numberOfVoxels     = data; break;
        case DataId::NumberOfVoxelsRS:          s.numberOfVoxelsRS   = data; break;
        case DataId::NumberOfPatches:           s.numberOfPatches    = data; break;
        case DataId::NumberOfLostPoints:        s.numberOfLostPoints = data; break;

        // Data here is the iteration
        case DataId::SkippedVoxels:             s.skippedVoxels[data]++;     break;
        case DataId::ScoreComputations:         s.scoreComputations[data]++; break;
        case DataId::NoEdge:                    s.NoEdge[data]++;            break;
        case DataId::IndirectEdge:              s.IndirectEdge[data]++;      break;
        case DataId::SingleEdge:                s.SingleEdge[data]++;        break;
        case DataId::MultiEdge:                 s.MultiEdge[data]++;         break;
        case DataId::NoEdge_R:                  s.NoEdge_R[data]++;          break;
        case DataId::IndirectEdge_R:            s.IndirectEdge_R[data]++;    break;
        case DataId::SingleEdge_R:              s.SingleEdge_R[data]++;      break;
        case DataId::MultiEdge_R:               s.MultiEdge_R[data]++;       break;
        case DataId::PpiChange:                 s.ppiChange[data]++;         break;

    } 
}

static void removeJsonFooter(const std::string& filename) {
    std::ifstream in(filename);
    if (!in.is_open()) return;

    std::string content(
        (std::istreambuf_iterator<char>(in)),
        std::istreambuf_iterator<char>()
    );
    in.close();

    // Find the last occurence of "]"
    const auto pos = content.rfind("]");
    if (pos == std::string::npos) return;

    content.erase(pos);

    std::ofstream out(filename, std::ios::trunc);
    out << content;
    out.close();
}

void StatsCollector::writeToFile(const std::string& filename, const size_t gofId) const {

    /* ---------------------------------------------------------
     * 1) Open the file
     * --------------------------------------------------------- */
    if (gofId == 0) {
        std::ofstream out(filename, std::ios::trunc);
        out << "{\n  \"frames\": [\n";
        out.close();
    } else {
        removeJsonFooter(filename);
        std::ofstream out(filename, std::ios::app);
        out << ",\n";
        out.close();
    }

    std::ofstream out(filename, std::ios::app);

    /* ---------------------------------------------------------
     * 2) Write the frames of the GOF
     * --------------------------------------------------------- */
    const size_t start = gofId * p_->sizeGOF;
    const size_t end   = std::min(stats_.size(), (gofId + 1) * p_->sizeGOF);

    for (size_t i = start; i < end; ++i) {
        const auto& s = stats_[i];

        out << "    {\n";
        out << "      \"NumberOfPoints\": "    << s.NumberOfPoints    << ",\n";
        out << "      \"numberOfVoxels\": "     << s.numberOfVoxels     << ",\n";
        out << "      \"numberOfVoxelsRS\": "   << s.numberOfVoxelsRS   << ",\n";

        /* -------- Skipped voxels -------- */
        out << "      \"skippedVoxels\": [";
        for (size_t j = 0; j < s.skippedVoxels.size(); ++j) {
            out << s.skippedVoxels[j];
            if (j + 1 < s.skippedVoxels.size()) out << ", ";
        }
        out << "],\n";

        /* -------- Class histogram -------- */
        out << "      \"classHistogram\": {\n";

        auto writeArray = [&](const std::string& name,
                              const std::vector<size_t>& v,
                              bool last) {
            out << "        \"" << name << "\": [";
            for (size_t j = 0; j < v.size(); ++j) {
                out << v[j];
                if (j + 1 < v.size()) out << ", ";
            }
            out << "]";
            if (!last) out << ",";
            out << "\n";
        };

        writeArray("NoEdge",       s.NoEdge,       false);
        writeArray("IndirectEdge", s.IndirectEdge, false);
        writeArray("SingleEdge",   s.SingleEdge,   false);
        writeArray("MultiEdge",    s.MultiEdge,    true);

        out << "      },\n";

        /* -------- Score computations -------- */
        out << "      \"scoreComputations\": [";
        for (size_t j = 0; j < s.scoreComputations.size(); ++j) {
            out << s.scoreComputations[j];
            if (j + 1 < s.scoreComputations.size()) out << ", ";
        }
        out << "],\n";

        /* -------- Class refinement -------- */
        out << "      \"classRefined\": {\n";
        writeArray("NoEdgeRefined",       s.NoEdge_R,       false);
        writeArray("IndirectEdgeRefined", s.IndirectEdge_R, false);
        writeArray("SingleEdgeRefined",   s.SingleEdge_R,   false);
        writeArray("MultiEdgeRefined",    s.MultiEdge_R,    true);
        out << "      },\n";

        /* -------- PPI changes -------- */
        out << "      \"ppiChange\": [";
        for (size_t j = 0; j < s.ppiChange.size(); ++j) {
            out << s.ppiChange[j];
            if (j + 1 < s.ppiChange.size()) out << ", ";
        }
        out << "],\n";

        /* -------- Patches & lost points -------- */
        out << "      \"numberOfPatches\": "    << s.numberOfPatches    << ",\n";
        out << "      \"numberOfLostPoints\": " << s.numberOfLostPoints << "\n";
        out << "    }";

        if (i + 1 < end) out << ",";
        out << "\n";
    }

    /* ---------------------------------------------------------
     * 3) Close the JSON
     * --------------------------------------------------------- */
    out << "  ]\n";
    out << "}\n";
    out.close();
}

StatsCollector& stats = StatsCollector::instance();