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

#include <cmath>

#include "bitstream_util.hpp"
#include "uvgvpcc/uvgvpcc.hpp"

struct profile_toolset_constraints_information {
    bool ptc_one_v3c_frame_only_flag = false;
    bool ptc_eom_constraint_flag = false;
    uint8_t ptc_max_map_count_minus1 = 1;
    uint8_t ptc_max_atlas_count_minus1 = 0;
    bool ptc_multiple_map_streams_constraint_flag = false;
    bool ptc_plr_constraint_flag = false;
    uint8_t ptc_attribute_max_dimension_minus1 = 2;
    uint8_t ptc_attribute_max_dimension_partitions_minus1 = 0;
    bool ptc_no_eight_orientations_constraint_flag = true;
    bool ptc_no_45degree_projection_patch_constraint_flag = true;
    bool ptc_restricted_geometry_flag = false;
    uint8_t ptc_num_reserved_constraint_bytes = 0;
    std::vector<uint8_t> ptc_reserved_constraint_byte = {};
};

struct profile_tier_level {
    bool ptl_tier_flag = false;
    uint8_t ptl_profile_codec_group_idc = 0;
    uint8_t ptl_profile_toolset_idc = 0;
    uint8_t ptl_profile_reconstruction_idc = 0;
    uint8_t ptl_max_decodes_idc = 0;
    uint8_t ptl_level_idc = 0;
    uint8_t ptl_num_sub_profiles = 0;
    bool ptl_extended_sub_profile_flag = false;  // 0: use ptl_sub_profile_idc_32, 1: use ptl_sub_profile_idc_64
    std::vector<uint32_t> ptl_sub_profile_idc_32 = {};
    std::vector<uint64_t> ptl_sub_profile_idc_64 = {};
    bool ptl_toolset_constraints_present_flag = false;
    profile_toolset_constraints_information ptc;
};

struct occupancy_information {
    uint8_t oi_lossy_occupancy_compression_threshold = 0;
    uint8_t oi_occupancy_2d_bit_depth_minus1 = 10;
    bool oi_occupancy_MSB_align_flag = false;
    uint8_t oi_occupancy_codec_id = 0;
};

struct geometry_information {
    uint8_t gi_geometry_codec_id = 0;
    uint8_t gi_geometry_2d_bit_depth_minus1 = 10;
    bool gi_geometry_MSB_align_flag = false;
    uint8_t gi_geometry_3d_coordinates_bit_depth_minus1 = 9;
    uint8_t gi_auxiliary_geometry_codec_id = 0;
};

struct attribute_information {
    uint8_t ai_attribute_count = 0;
    std::vector<uint8_t> ai_attribute_type_id = {};
    std::vector<uint8_t> ai_attribute_codec_id = {};
    std::vector<uint8_t> ai_auxiliary_attribute_codec_id = {};
    std::vector<bool> ai_attribute_map_absolute_coding_persistence_flag = {};
    std::vector<uint8_t> ai_attribute_dimension_minus1 = {};
    std::vector<uint8_t> ai_attribute_dimension_partitions_minus1 = {};
    std::vector<std::vector<uint16_t>> ai_attribute_partition_channels_minus1{};
    std::vector<uint8_t> ai_attribute_2d_bit_depth_minus1 = {};
    std::vector<bool> ai_attribute_MSB_align_flag = {};
};

class vps {
   public:
    /* Constructor generates VPS values from paramUVG and gofUVG */
    vps(const uvgvpcc_enc::Parameters& paramUVG, const std::shared_ptr<uvgvpcc_enc::GOF>& gofUVG);

    /* Write the VPS into a bitstream form */
    bool write_vps(bitstream_t* stream);

    std::size_t get_vps_byte_len() { return vps_length_bytes_; };

   private:
    /* Fill the PTL values in VPS */
    profile_tier_level fill_ptl(std::size_t& len) const;

    // helper variables
    std::size_t vps_length_bytes_;
    uint8_t codec_group_;  // AVD, VVC, HEVC, or other

    profile_tier_level ptl_;
    uint8_t vps_v3c_parameter_set_id_;
    uint8_t vps_atlas_count_minus1_;

    std::vector<uint8_t> vps_atlas_id_;
    std::vector<std::size_t> vps_frame_width_;
    std::vector<std::size_t> vps_frame_height_;
    std::vector<uint8_t> vps_map_count_minus1_;
    std::vector<bool> vps_multiple_map_streams_present_flag_;
    std::vector<std::vector<bool>> vps_map_absolute_coding_enabled_flag_;
    std::vector<std::vector<uint16_t>> vps_map_predictor_index_diff_;
    std::vector<bool> vps_auxiliary_video_present_flag_;
    std::vector<bool> vps_occupancy_video_present_flag_;
    std::vector<bool> vps_geometry_video_present_flag_;
    std::vector<bool> vps_attribute_video_present_flag_;
    std::vector<occupancy_information> occupancy_info_;
    std::vector<geometry_information> geometry_info_;
    std::vector<attribute_information> attribute_info_;

    bool vps_extension_present_flag_;
    bool vps_packing_information_present_flag_;
    bool vps_miv_extension_present_flag_;
    uint8_t vps_extension_6bits_;
    std::size_t vps_extension_length_minus1_;
    uint8_t vps_extension_data_byte_;
};