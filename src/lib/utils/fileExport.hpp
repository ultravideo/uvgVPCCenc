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

/// \file Intermediary files exportation management.

#include <memory>
#include <vector>

#include "uvgvpcc/uvgvpcc.hpp"
#include "utils/utils.hpp"

using namespace uvgvpcc_enc;

namespace FileExport {

    void cleanIntermediateFiles();

    // Patch generation
    void exportPointCloudNormalComputation(const std::shared_ptr<Frame>& frame, const std::vector<Vector3<typeGeometryInput>>& pointsGeometry, std::vector<Vector3<double>>& normals);
    void exportPointCloudNormalOrientation(const std::shared_ptr<Frame>& frame, const std::vector<Vector3<typeGeometryInput>>& pointsGeometry, std::vector<Vector3<double>>& normals);
    void exportPointCloudInitialSegmentation(const std::shared_ptr<Frame>& frame, const std::vector<Vector3<typeGeometryInput>>& pointsGeometry, const std::vector<size_t>& pointsPPIs);
    void exportPointCloudRefineSegmentation(const std::shared_ptr<Frame>& frame, const std::vector<Vector3<typeGeometryInput>>& pointsGeometry, const std::vector<size_t>& pointsPPIs);
    void exportPointCloudPatchSegmentation(const std::shared_ptr<Frame>& frame);
    
    // Map generation
    void exportImageOccupancy(const std::shared_ptr<Frame>& frame);
    void exportImageOccupancyDS(const std::shared_ptr<Frame>& frame);
    void exportImageAttribute(const std::shared_ptr<Frame>& frame);
    void exportImageGeometry(const std::shared_ptr<Frame>& frame);
    void exportImageAttributeBgFill(const std::shared_ptr<Frame>& frame);
    void exportImageGeometryBgFill(const std::shared_ptr<Frame>& frame);  
    void exportImageAttributeYUV(const std::shared_ptr<Frame>& frame);

    // Map encoding
    void exportOccupancyBitstream(const std::shared_ptr<uvgvpcc_enc::GOF>& gof, const std::vector<uint8_t>& bitstream, const std::string& codecExtension);
    void exportAttributeBitstream(const std::shared_ptr<uvgvpcc_enc::GOF>& gof, const std::vector<uint8_t>& bitstream, const std::string& codecExtension);
    void exportGeometryBitstream(const std::shared_ptr<uvgvpcc_enc::GOF>& gof, const std::vector<uint8_t>& bitstream, const std::string& codecExtension);

}