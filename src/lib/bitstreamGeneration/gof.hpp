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
#include <memory>
#include <vector>

#include "atlas_context.hpp"
#include "uvgvpcc/uvgvpcc.hpp"
#include "video_sub_bitstream.hpp"
#include "vps.hpp"

/* Statistics related to a V3C Group Of Frames (GOF) */
class v3c_gof {
   public:
    v3c_gof(size_t id) {
        v3c_unit_precision_ = 0;
        gof_id_ = id;
        n_frames_ = 0;
    };

    // Used for LD mode
    void set_n_frames(size_t value) { n_frames_ = value; }

    // Set the V3C unit precision. If smaller than before, do nothing
    void set_v3c_unit_precision(uint32_t new_precision) {
        if (new_precision > v3c_unit_precision_) {
            v3c_unit_precision_ = new_precision;
        }
    };

    /* Add new data to the most recent GOF */
    void add_v3c_vps(std::unique_ptr<vps> data) { v3c_vps_sub_ = std::move(data); };
    void add_v3c_atlas_context(std::unique_ptr<atlas_context> data) { v3c_ad_unit_ = std::move(data); };
    void add_v3c_ovd_sub(std::unique_ptr<std::vector<uint8_t>> data) { v3c_ovd_sub_ = std::move(data); };
    void add_v3c_gvd_sub(std::unique_ptr<std::vector<uint8_t>> data) { v3c_gvd_sub_ = std::move(data); };
    void add_v3c_avd_sub(std::unique_ptr<std::vector<uint8_t>> data) { v3c_avd_sub_ = std::move(data); };

    /* Write the latest GOF to a single V3C unit stream buffer, with parsing information given separately */
    void write_v3c_chunk(uvgvpcc_enc::API::v3c_unit_stream *out);

    /* Write the latest GOF in Low Delay (LD) mode to a single V3C unit stream buffer, with parsing information given separately */
    void write_v3c_ld_chunk(const std::vector<nal_info> &ovd_nals, const std::vector<nal_info> &gvd_nals,
                            const std::vector<nal_info> &avd_nals, uvgvpcc_enc::API::v3c_unit_stream *out, bool double_layer);

   private:
    size_t gof_id_;
    size_t v3c_unit_precision_;
    size_t n_frames_;                               // N of frames in GOF, used for low-delay mode
    std::unique_ptr<vps> v3c_vps_sub_;                   // no V3C header
    std::unique_ptr<atlas_context> v3c_ad_unit_;         // incl. V3C header
    std::unique_ptr<std::vector<uint8_t>> v3c_ovd_sub_;  // no V3C header
    std::unique_ptr<std::vector<uint8_t>> v3c_gvd_sub_;  // no V3C header
    std::unique_ptr<std::vector<uint8_t>> v3c_avd_sub_;  // no V3C header
};