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

#include <fstream>
#include "uvgvpcc/uvgvpcc.hpp"

enum ENCODER_TYPE {OCCUPANCY, GEOMETRY, ATTRIBUTE};

// All 2D encoder should derived from this class. Notice that there is one 2D encoder for each map (occupancy, geometry and attribute). Static functions can't be overrided. For example, the handling of the function pointer is not done by the derived class, as it should always be the same whatever the 2D encoder used.
class Abstract2DMapEncoder {
public:
    Abstract2DMapEncoder() = default;
    virtual ~Abstract2DMapEncoder() = default;
    
    virtual void configureGOFEncoder(const std::shared_ptr<uvgvpcc_enc::GOF>& gof, const ENCODER_TYPE& encoderType) = 0;
    virtual void encodeGOFMaps(std::shared_ptr<uvgvpcc_enc::GOF>& gof) = 0;

protected:
    

    // 2D encoders parameters (should be common with all 2D encoders)
    
    // Do no change between GOFs
    ENCODER_TYPE encoderType_;
    std::string encoderName_; // Use for debugging and log
    bool lossLess_;
    size_t nbThread_;
    std::string preset_;
    std::string format_;
    std::string mode_;
    size_t qp_;
    
    // Initialized for each GOF
    size_t width_;
    size_t height_;





};

inline void writeBitstreamToFile(const std::vector<uint8_t>& bitstream, const std::string& filename) {
    
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file for writing bitstream: " + filename);
    }
    file.write(reinterpret_cast<const char*>(bitstream.data()), bitstream.size());
    file.close();
}
