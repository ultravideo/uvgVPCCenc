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

#include "atlas_frame.hpp"
#include "bitstream_common.hpp"
#include "bitstream_util.hpp"
#include "uvgvpcc/uvgvpcc.hpp"

/* Atlas context is used to hold the atlas data (inside V3C_AD unit) of a single GOF */
class atlas_context {
   public:
    atlas_context() : asps_(), afps_(), atlas_data_(), gof_id_(), atlas_sub_size_(0), ad_nal_sizes_({}), ad_nal_precision_(0) {};

    /* Fill data structures with encoded data from gofUVG and sequence parameters from paramUVG */
    void initialize_atlas_context(const std::shared_ptr<uvgvpcc_enc::GOF>& gofUVG, const uvgvpcc_enc::Parameters& paramUVG);

    /* Write the atlas sub-bitstream */
    void write_atlas_sub_bitstream(bitstream_t* stream);

    /* Write ASPS and AFPS nal units */
    void write_atlas_parameter_set_nals(bitstream_t* stream);
    /* Write an atlas NAL unit at index */
    void write_atlas_nal(bitstream_t* stream, std::size_t index);

    /* Write an atlas NAL unit at index */
    void write_atlas_eob(bitstream_t* stream);

    // -------------- Getters - data structures --------------
    std::vector<atlas_tile_layer_rbsp>& get_atlases() { return atlas_data_; };
    atlas_sequence_parameter_set& get_asps() { return asps_; };
    atlas_frame_parameter_set& get_afps() { return afps_; };

    // -------------- Getters - helper variables --------------
    std::size_t get_gof_id() { return gof_id_; };
    std::size_t get_atlas_sub_size() { return atlas_sub_size_; };
    std::vector<std::size_t> get_ad_nal_sizes() { return ad_nal_sizes_; };
    std::size_t get_ad_nal_precision() { return ad_nal_precision_; };

   private:
    /* Calculate sizes of
        atlas sub-bitstream             = atlas_sub_size_
        atlas NAL units                 = ad_nal_sizes_
        atlas NAL unit precision        = ad_nal_precision_ */
    void calculate_atlas_size_values();

    // -------------- Functions to fill data structure values --------------
    static atlas_sequence_parameter_set create_atlas_sequence_parameter_set(const std::shared_ptr<uvgvpcc_enc::GOF>& gofUVG,
                                                                            const uvgvpcc_enc::Parameters& paramUVG);
    atlas_frame_parameter_set create_atlas_frame_parameter_set();
    atlas_frame_tile_information create_atlas_frame_tile_information() const;
    atlas_tile_header create_atlas_tile_header(std::size_t frameIndex, std::size_t tileIndex, const uvgvpcc_enc::Parameters& paramUVG) const;
    atlas_tile_data_unit create_atlas_tile_data_unit(const uvgvpcc_enc::Parameters& paramUVG, const uvgvpcc_enc::Frame& frameUVG,
                                                     atlas_tile_header& ath) const;
    atlas_tile_layer_rbsp create_atlas_tile_layer_rbsp(std::size_t frameIndex, std::size_t tileIndex, const uvgvpcc_enc::Parameters& paramUVG,
                                                       const uvgvpcc_enc::Frame& frameUVG);

    // -------------- Functions to write data structures to bitstream --------------
    static void write_nal_hdr(bitstream_t* stream, const uint8_t nal_type, const uint8_t nal_layer_id, const uint8_t nal_temporal_id_plus1);
    void write_atlas_seq_parameter_set(bitstream_t* stream);
    static void write_atlas_adaption_parameter_set(bitstream_t* stream);
    void write_atlas_frame_parameter_set(bitstream_t* stream) const;
    void write_atlas_tile_header(bitstream_t* stream, NAL_UNIT_TYPE nalu_t, const atlas_tile_header& ath) const;
    void write_atlas_tile_data_unit(bitstream_t* stream, const atlas_tile_data_unit& atdu, const atlas_tile_header& ath);
    void write_atlas_tile_layer_rbsp(bitstream_t* stream, NAL_UNIT_TYPE nalu_t, const atlas_tile_layer_rbsp& rbsp);
    void write_access_unit_delimiter(bitstream_t* stream);

    void write_patch_information_data(bitstream_t* stream, const patch_information_data& pid, const atlas_tile_header& ath);
    void write_patch_data_unit(bitstream_t* stream, const patch_data_unit& pdu, const atlas_tile_header& ath) const;

    /* -------------- Atlas data structures -------------- */
    atlas_sequence_parameter_set asps_;
    atlas_frame_parameter_set afps_;
    std::vector<atlas_tile_layer_rbsp> atlas_data_;

    /* -------------- Helper variables -------------- */
    std::size_t gof_id_;
    std::size_t atlas_sub_size_;
    std::vector<std::size_t> ad_nal_sizes_;
    std::size_t ad_nal_precision_;
};