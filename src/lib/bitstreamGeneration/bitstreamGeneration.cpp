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

#include "bitstreamGeneration.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "atlas_context.hpp"
#include "bitstream_common.hpp"
#include "gof.hpp"
#include "uvgvpcc/log.hpp"
#include "uvgvpcc/uvgvpcc.hpp"
#include "video_sub_bitstream.hpp"
#include "vps.hpp"

/// \file Entry point for the whole bitstream generation process.

using namespace uvgvpcc_enc;

void BitstreamGeneration::createV3CGOFBitstream(const std::shared_ptr<uvgvpcc_enc::GOF>& gofUVG, const uvgvpcc_enc::Parameters& paramUVG,
                                                uvgvpcc_enc::API::v3c_unit_stream* output) {
    uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::INFO, "BITSTREAM GENERATION",
                             "GOF " + std::to_string(gofUVG->gofId) + " : Create V3C GOF bitstream using uvgVPCC.\n");

    v3c_gof gof(gofUVG->gofId);
    gof.set_n_frames(gofUVG->nbFrames);

    // --------------- Generate VPS ---------------
    auto v3c_parameter_set = std::make_unique<vps>(paramUVG, gofUVG);

    // --------------- Generate atlas context -------------------
    auto atlas = std::make_unique<atlas_context>();
    atlas->initialize_atlas_context(gofUVG, paramUVG);

    // --------------- Fetch video sub-bitstream data ---------------------------------------------
    // Occupancy map
    auto bitstream_ovd = std::make_unique<std::vector<uint8_t>>();
    std::vector<nal_info> ovd_nals;  // For low delay bitstreams
    // if (p_->useEncoderCommand) {
    //     read(gofUVG->baseNameOccupancy + ".hevc", *bitstream_ovd.get());
    // }
    *bitstream_ovd.get() = gofUVG->bitstreamOccupancy;
    byteStreamToSampleStream(*bitstream_ovd.get(), 4, ovd_nals, false);

    // Geometry map
    auto bitstream_gvd = std::make_unique<std::vector<uint8_t>>();
    std::vector<nal_info> gvd_nals;  // For low delay bitstreams
    // if (p_->useEncoderCommand) {
    //     read(gofUVG->baseNameGeometry + ".hevc", *bitstream_gvd.get());
    // }
    *bitstream_gvd.get() = gofUVG->bitstreamGeometry;
    byteStreamToSampleStream(*bitstream_gvd.get(), 4, gvd_nals, false);

    // Attribute map
    auto bitstream_avd = std::make_unique<std::vector<uint8_t>>();
    std::vector<nal_info> avd_nals;  // For low delay bitstreams
    // if (p_->useEncoderCommand) {
    //     read(gofUVG->baseNameAttribute + ".hevc", *bitstream_avd.get());
    // }
    *bitstream_avd.get() = gofUVG->bitstreamAttribute;
    byteStreamToSampleStream(*bitstream_avd.get(), 4, avd_nals, false);

    // --------------- Calculate V3C unit size precision -------------------------------------------
    size_t v3c_max_size = v3c_parameter_set.get()->get_vps_byte_len();
    if (atlas.get()->get_atlas_sub_size() + 4 > v3c_max_size) {
        v3c_max_size = atlas.get()->get_atlas_sub_size() + 4;
    }
    if (bitstream_ovd.get()->size() + 4 > v3c_max_size) {
        v3c_max_size = bitstream_ovd.get()->size() + 4;
    }
    if (bitstream_gvd.get()->size() + 4 > v3c_max_size) {
        v3c_max_size = bitstream_gvd.get()->size() + 4;
    }
    if (bitstream_avd.get()->size() + 4 > v3c_max_size) {
        v3c_max_size = bitstream_avd.get()->size() + 4;
    }
    const uint32_t v3c_precision =
        static_cast<uint32_t>(std::min(std::max(static_cast<int>(ceil(static_cast<double>(ceilLog2(v3c_max_size)) / 8.0)), 1), 8));
    gof.set_v3c_unit_precision(v3c_precision);

    /* Move the data pointers to V3C bitstream structure */
    gof.add_v3c_vps(std::move(v3c_parameter_set));
    gof.add_v3c_atlas_context(std::move(atlas));
    gof.add_v3c_ovd_sub(std::move(bitstream_ovd));
    gof.add_v3c_gvd_sub(std::move(bitstream_gvd));
    gof.add_v3c_avd_sub(std::move(bitstream_avd));

    // ---------- remove intermediate files ----------
    // if (!p_->exportIntermediateMaps /*&& p_->useEncoderCommand*/) {
    //     Utils::removeFile(gofUVG->baseNameOccupancy + ".hevc");
    //     Utils::removeFile(gofUVG->baseNameGeometry + ".hevc");
    //     Utils::removeFile(gofUVG->baseNameAttribute + ".hevc");
    // }
    if (paramUVG.lowDelayBitstream) {
        gof.write_v3c_ld_chunk(ovd_nals, gvd_nals, avd_nals, output, paramUVG.doubleLayer);
    } else {
        gof.write_v3c_chunk(output);
    }
    output->available_chunks.release();
}