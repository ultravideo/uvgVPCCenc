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

/// \file Entry point for the map encoding process.

#include "mapEncoding.hpp"

#include <cassert>
#include <cstring>
#include <memory>
#include <string>

#include "abstract2DMapEncoder.hpp"
#include "encoderKvazaar.hpp"

#if LINK_FFMPEG
#include "encoderFFmpeg.hpp"  // Include FFmepg
#endif

#include "utils/parameters.hpp"
#include "uvgutils/log.hpp"
#include "uvgvpcc/uvgvpcc.hpp"

using namespace uvgvpcc_enc;

namespace {

std::unique_ptr<Abstract2DMapEncoder> occupancyMapDSEncoder;
std::unique_ptr<Abstract2DMapEncoder> geometryMapEncoder;
std::unique_ptr<Abstract2DMapEncoder> attributeMapEncoder;

}  // anonymous namespace

void MapEncoding::initializeStaticParameters() {
    if (p_->occupancyEncoderName == "Kvazaar" || p_->geometryEncoderName == "Kvazaar" || p_->attributeEncoderName == "Kvazaar") {
        EncoderKvazaar::initializeLogCallback();
#if LINK_FFMPEG
    } else if (p_->occupancyEncoderName == "FFmpeg" || p_->geometryEncoderName == "FFmpeg" || p_->attributeEncoderName == "FFmpeg") {
        EncoderFFmpeg::initializeLogCallback();
#endif
    } else {
        assert(false);
    }
}

void MapEncoding::initializeEncoderPointers() {
    if (p_->occupancyEncoderName == "Kvazaar") {
        occupancyMapDSEncoder = std::make_unique<EncoderKvazaar>(OCCUPANCY);
#if LINK_FFMPEG
    } else if (p_->occupancyEncoderName == "FFmpeg") {
        occupancyMapDSEncoder = std::make_unique<EncoderFFmpeg>(OCCUPANCY, p_->occupancyFFmpegCodecName, "", p_->occupancyFFmpegCodecOptions,
                                                                p_->occupancyFFmpegCodecParams);
#endif
    } else {
        assert(false);
    }

    if (p_->geometryEncoderName == "Kvazaar") {
        geometryMapEncoder = std::make_unique<EncoderKvazaar>(GEOMETRY);
#if LINK_FFMPEG
    } else if (p_->geometryEncoderName == "FFmpeg") {
        geometryMapEncoder = std::make_unique<EncoderFFmpeg>(GEOMETRY, p_->geometryFFmpegCodecName, "", p_->geometryFFmpegCodecOptions,
                                                             p_->geometryFFmpegCodecParams);
#endif
    } else {
        assert(false);
    }

    if (p_->attributeEncoderName == "Kvazaar") {
        attributeMapEncoder = std::make_unique<EncoderKvazaar>(ATTRIBUTE);
#if LINK_FFMPEG
    } else if (p_->attributeEncoderName == "FFmpeg") {
        attributeMapEncoder = std::make_unique<EncoderFFmpeg>(ATTRIBUTE, p_->attributeFFmpegCodecName, "", p_->attributeFFmpegCodecOptions,
                                                              p_->attributeFFmpegCodecParams);
#endif
    } else {
        assert(false);
    }
}

void MapEncoding::encodeGOFMaps(const std::shared_ptr<uvgvpcc_enc::GOF>& gof) {
    uvgutils::Logger::log<uvgutils::LogLevel::TRACE>("MAP ENCODING", "Encode maps of GOF " + std::to_string(gof->gofId) + ".\n");

    occupancyMapDSEncoder->encodeGOFMaps(gof);
    geometryMapEncoder->encodeGOFMaps(gof);
    attributeMapEncoder->encodeGOFMaps(gof);
}
