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

/// \file Entry point for the map encoding process.

#include "mapEncoding.hpp"

#include <cassert>
#include <cstring>
#include <memory>
#include <string>

#include "abstract2DMapEncoder.hpp"
#include "encoderKvazaar.hpp"

#include "uvgvpcc/log.hpp"
#include "uvgvpcc/uvgvpcc.hpp"


using namespace uvgvpcc_enc;

namespace {

std::unique_ptr<Abstract2DMapEncoder> occupancyMapDSEncoder;
std::unique_ptr<Abstract2DMapEncoder> geometryMapEncoder;
std::unique_ptr<Abstract2DMapEncoder> attributeMapEncoder;

} // anonymous namespace

void MapEncoding::initializeStaticParameters() {
    if (p_->occupancyEncoderName == "Kvazaar" || p_->geometryEncoderName == "Kvazaar" || p_->attributeEncoderName == "Kvazaar" ) {
        EncoderKvazaar::initializeLogCallback();
    } else {
        assert(false);
    }
}

void MapEncoding::initializeEncoderPointers() {
    
    if(p_->occupancyEncoderName == "Kvazaar") {
        occupancyMapDSEncoder = std::make_unique<EncoderKvazaar>();
    } else {
        assert(false);
    }
    
    if(p_->geometryEncoderName == "Kvazaar") {
        geometryMapEncoder = std::make_unique<EncoderKvazaar>();
    } else {
        assert(false);
    }

    if(p_->attributeEncoderName == "Kvazaar") {
        attributeMapEncoder = std::make_unique<EncoderKvazaar>();
    } else {
        assert(false);
    }        

}

void MapEncoding::encodeGOFMaps(std::shared_ptr<uvgvpcc_enc::GOF>& gof) {
    uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::TRACE, "MAP ENCODING", "Encode maps of GOF " + std::to_string(gof->gofId) + ".\n");

    occupancyMapDSEncoder->configureGOFEncoder(gof, OCCUPANCY);
    occupancyMapDSEncoder->encodeGOFMaps(gof);

    geometryMapEncoder->configureGOFEncoder(gof, GEOMETRY);
    geometryMapEncoder->encodeGOFMaps(gof);

    attributeMapEncoder->configureGOFEncoder(gof, ATTRIBUTE);
    attributeMapEncoder->encodeGOFMaps(gof);
    
}
