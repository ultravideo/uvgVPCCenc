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

#include "vps.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>

#include "bitstream_util.hpp"
#include "uvgvpcc/uvgvpcc.hpp"

vps::vps(const uvgvpcc_enc::Parameters& paramUVG, const std::shared_ptr<uvgvpcc_enc::GOF>& gofUVG) {
    if (paramUVG.occupancyEncoderName=="Kvazaar" &&paramUVG.geometryEncoderName=="Kvazaar" &&paramUVG.attributeEncoderName=="Kvazaar" /*|| paramUVG.useEncoderCommand*/) {
        codec_group_ = 1;  // TMC2 : CODEC_GROUP_HEVC_MAIN10
    } else if (paramUVG.occupancyEncoderName=="uvg266" &&paramUVG.geometryEncoderName=="uvg266" &&paramUVG.attributeEncoderName=="uvg266") {
        codec_group_ = 3;  // TMC2 : CODEC_GROUP_VVC_MAIN10
    } else {
        throw std::runtime_error(
            "Error : unknown ptl_profile_codec_group_idc. This bitstream parameter indicates what codec is used to encode the 2D videos.");
    }
    std::size_t vps_length_bits = 0;
    ptl_ = fill_ptl(vps_length_bits);                // profile_tier_level
    vps_v3c_parameter_set_id_ = gofUVG->gofId % 16;  // The value of vps_v3c_parameter_set_id shall be in the range of 0 to 15
    vps_atlas_count_minus1_ = 0;                     // for atlas count 1
    vps_length_bits += 18;                           // fixed fields

    // Seems that there always is only one atlas
    for (uint8_t k = 0; k < (vps_atlas_count_minus1_ + 1); k++) {
        vps_atlas_id_.push_back(0);
        vps_frame_width_.push_back(paramUVG.mapWidth);
        vps_frame_height_.push_back(gofUVG->mapsHeight);

        vps_length_bits += 6 + uvg_calculate_ue_len(vps_frame_width_.back()) + uvg_calculate_ue_len(vps_frame_height_.back());

        bool vps_map_count_minus1 = paramUVG.doubleLayer;
        vps_map_count_minus1_.push_back(static_cast<uint8_t>(vps_map_count_minus1));
        if (vps_map_count_minus1_.back() > 0) {
            vps_multiple_map_streams_present_flag_.push_back(false);
        }
        vps_map_absolute_coding_enabled_flag_.push_back({true});
        vps_map_predictor_index_diff_.push_back({static_cast<uint16_t>(false)});

        vps_length_bits += 4 + (vps_map_count_minus1_.back() > 0 ? 1 : 0);  // vps_map_count_minus1 and vps_multiple_map_streams_present_flag_
        for (uint8_t i = 1; i <= vps_map_count_minus1_.at(k); ++i) {
            vps_map_absolute_coding_enabled_flag_.at(k).push_back(true);  // this does not get written
            if (!vps_map_absolute_coding_enabled_flag_.at(k).at(i)) {
                vps_map_predictor_index_diff_.at(k).push_back(static_cast<uint16_t>(false));
                vps_length_bits += uvg_calculate_ue_len(0);
            }
        }
        vps_auxiliary_video_present_flag_.push_back(false);
        vps_occupancy_video_present_flag_.push_back(true);
        vps_geometry_video_present_flag_.push_back(true);
        vps_attribute_video_present_flag_.push_back(true);
        vps_length_bits += 4;

        occupancy_information oi;
        // to do: codec_id same as codec_group_idc?
        oi.oi_occupancy_codec_id = codec_group_;
        oi.oi_lossy_occupancy_compression_threshold = 0;  // thresholdLossyOM from tmc2-interface
        oi.oi_occupancy_2d_bit_depth_minus1 = 7;
        oi.oi_occupancy_MSB_align_flag = false;
        occupancy_info_.push_back(oi);
        vps_length_bits += 22;  // occupancy info

        geometry_information gi;
        gi.gi_geometry_codec_id = codec_group_;
        const std::size_t geometryNominal2dBitdepth = 8; // TMC2 : Bit depth of geometry 2D
        gi.gi_geometry_2d_bit_depth_minus1 = static_cast<uint8_t>(geometryNominal2dBitdepth - 1);
        gi.gi_geometry_MSB_align_flag = false;
        gi.gi_geometry_3d_coordinates_bit_depth_minus1 =
            static_cast<uint8_t>(paramUVG.geoBitDepthInput);  // no -1 because it is already -1 from what is should be
        gi.gi_auxiliary_geometry_codec_id = codec_group_;
        geometry_info_.push_back(gi);
        vps_length_bits += 19 + (vps_auxiliary_video_present_flag_.at(k) ? 8 : 0);  // geometry info

        attribute_information ai;
        ai.ai_attribute_count = 1;
        vps_length_bits += 7;  // ai_attribute_count
        for (uint8_t i = 0; i < ai.ai_attribute_count; i++) {
            ai.ai_attribute_type_id.push_back(0);  // Texture
            ai.ai_attribute_codec_id.push_back(codec_group_); // to do : here can be implemented different encoder for each maps
            ai.ai_auxiliary_attribute_codec_id.push_back(codec_group_);
            vps_length_bits += 12 + (vps_auxiliary_video_present_flag_.at(k) ? 8 : 0);

            ai.ai_attribute_map_absolute_coding_persistence_flag.push_back(false);
            ai.ai_attribute_dimension_minus1.push_back(2);  // to do: 2 comes from bitstreamGenInterface, why?
            const uint8_t d = ai.ai_attribute_dimension_minus1.at(i);
            vps_length_bits += (vps_map_count_minus1_.at(k) > 0 ? 1 : 0) + 6;

            if (d != 0) {
                vps_length_bits += 6;  // ai_attribute_dimension_partitions_minus1
            }

            ai.ai_attribute_dimension_partitions_minus1.push_back(0);  // to do; default value, is it ok?
            ai.ai_attribute_partition_channels_minus1.push_back({0});
            // Because ai_attribute_dimension_partitions_minus1 == 0, there is no ai_attribute_partition_channels_minus1
            // So vps_length_bits will not increment either
            ai.ai_attribute_2d_bit_depth_minus1.push_back(7);  // to do: get this from paramUVG?
            ai.ai_attribute_MSB_align_flag.push_back(false);
            vps_length_bits += 6;
        }
        attribute_info_.push_back(ai);
    }
    vps_length_bits += 1;
    vps_extension_present_flag_ = false;
    vps_packing_information_present_flag_ = false;
    vps_miv_extension_present_flag_ = false;
    vps_extension_6bits_ = 0;
    vps_length_bytes_ = std::ceil(static_cast<float>(vps_length_bits) / 8.F);
    // vps_length_bits += 9; // length not increasing as these dont get written
    // std::cout << "VPS len in bits: " << vps_length_bits << std::endl;

    // Non initialized in constructor
    vps_extension_length_minus1_ = 0;
    vps_extension_data_byte_ = 0;
}

bool vps::write_vps(bitstream_t* stream) {
    // profile_tier_level
    WRITE_U(stream, ptl_.ptl_tier_flag, 1, "ptl_tier_flag");
    WRITE_U(stream, ptl_.ptl_profile_codec_group_idc, 7, "ptl_profile_codec_group_idc");
    WRITE_U(stream, ptl_.ptl_profile_toolset_idc, 8, "ptl_profile_toolset_idc");
    WRITE_U(stream, ptl_.ptl_profile_reconstruction_idc, 8, "ptl_profile_reconstruction_idc");
    WRITE_U(stream, 0, 16, "ptl_reserved_zero_16bits");
    WRITE_U(stream, ptl_.ptl_max_decodes_idc, 4, "ptl_max_decodes_idc");
    WRITE_U(stream, 0xfff, 12, "ptl_reserved_0xfff_12bits");
    WRITE_U(stream, ptl_.ptl_level_idc, 8, "ptl_level_idc");
    WRITE_U(stream, ptl_.ptl_num_sub_profiles, 6, "ptl_num_sub_profiles");
    WRITE_U(stream, ptl_.ptl_extended_sub_profile_flag, 1, "ptl_extended_sub_profile_flag");
    WRITE_U(stream, ptl_.ptl_toolset_constraints_present_flag, 1, "ptl_toolset_constraints_present_flag");

    // profile_toolset_constraints_information
    // to do: Do we need profile_toolset_constraints_information? Not present from TMC2 interface

    WRITE_U(stream, vps_v3c_parameter_set_id_, 4, "vps_v3c_parameter_set_id");
    WRITE_U(stream, 0, 8, "vps_reserved_zero_8bits");
    WRITE_U(stream, vps_atlas_count_minus1_, 6, "vps_atlas_count_minus1");

    for (uint8_t j = 0; j < (vps_atlas_count_minus1_ + 1); j++) {
        WRITE_U(stream, j, 6, "vps_atlas_id");
        WRITE_UE(stream, int(vps_frame_width_.at(j)), "vps_frame_width");
        WRITE_UE(stream, int(vps_frame_height_.at(j)), "vps_frame_height");
        WRITE_U(stream, vps_map_count_minus1_.at(j), 4, "vps_map_count_minus1");

        if (vps_map_count_minus1_.at(j) > 0) {
            WRITE_U(stream, int(vps_multiple_map_streams_present_flag_.at(j)), 1, "vps_multiple_map_streams_present_flag");
        }
        WRITE_U(stream, int(vps_auxiliary_video_present_flag_.at(j)), 1, "vps_auxiliary_video_present_flag");
        WRITE_U(stream, int(vps_occupancy_video_present_flag_.at(j)), 1, "vps_occupancy_video_present_flag");
        WRITE_U(stream, int(vps_geometry_video_present_flag_.at(j)), 1, "vps_geometry_video_present_flag");
        WRITE_U(stream, int(vps_attribute_video_present_flag_.at(j)), 1, "vps_attribute_video_present_flag");

        if (vps_occupancy_video_present_flag_.at(j)) {
            WRITE_U(stream, occupancy_info_.at(j).oi_occupancy_codec_id, 8, "oi_occupancy_codec_id");
            WRITE_U(stream, occupancy_info_.at(j).oi_lossy_occupancy_compression_threshold, 8, "oi_lossy_occupancy_compression_threshold");
            WRITE_U(stream, occupancy_info_.at(j).oi_occupancy_2d_bit_depth_minus1, 5, "oi_occupancy_2d_bit_depth_minus1");
            WRITE_U(stream, occupancy_info_.at(j).oi_occupancy_MSB_align_flag, 1, "oi_occupancy_MSB_align_flag");
        }

        if (vps_geometry_video_present_flag_.at(j)) {
            WRITE_U(stream, geometry_info_.at(j).gi_geometry_codec_id, 8, "gi_geometry_codec_id");
            WRITE_U(stream, geometry_info_.at(j).gi_geometry_2d_bit_depth_minus1, 5, "gi_geometry_2d_bit_depth_minus1");
            WRITE_U(stream, geometry_info_.at(j).gi_geometry_MSB_align_flag, 1, "gi_geometry_MSB_align_flag");
            WRITE_U(stream, geometry_info_.at(j).gi_geometry_3d_coordinates_bit_depth_minus1, 5,
                    "gi_geometry_3d_coordinates_bit_depth_minus1");

            if (vps_auxiliary_video_present_flag_.at(j)) {
                WRITE_U(stream, geometry_info_.at(j).gi_auxiliary_geometry_codec_id, 8, "gi_auxiliary_geometry_codec_id");
            }
        }

        if (vps_attribute_video_present_flag_.at(j)) {
            WRITE_U(stream, attribute_info_.at(j).ai_attribute_count, 7, "ai_attribute_count");

            for (uint8_t i = 0; i < attribute_info_.at(j).ai_attribute_count; ++i) {
                WRITE_U(stream, attribute_info_.at(j).ai_attribute_type_id.at(i), 4, "ai_attribute_type_id");
                WRITE_U(stream, attribute_info_.at(j).ai_attribute_codec_id.at(i), 8, "ai_attribute_codec_id");

                if (vps_auxiliary_video_present_flag_.at(j)) {
                    WRITE_U(stream, attribute_info_.at(j).ai_auxiliary_attribute_codec_id.at(i), 8, "ai_auxiliary_attribute_codec_id");
                }
                if (vps_map_count_minus1_.at(j) > 0) {
                    WRITE_U(stream, int(attribute_info_.at(j).ai_attribute_map_absolute_coding_persistence_flag.at(i)), 1,
                            "ai_attribute_map_absolute_coding_persistence_flag");
                }

                uint8_t d = attribute_info_.at(j).ai_attribute_dimension_minus1.at(i);
                WRITE_U(stream, d, 6, "ai_attribute_dimension_minus1");

                uint8_t m = 0;
                if (d == 0) {  // true
                    m = 0;
                    attribute_info_.at(j).ai_attribute_dimension_partitions_minus1.at(i) = 0;
                } else {
                    m = attribute_info_.at(j).ai_attribute_dimension_partitions_minus1.at(i);
                    WRITE_U(stream, attribute_info_.at(j).ai_attribute_dimension_partitions_minus1.at(i), 6,
                            "ai_attribute_dimension_partitions_minus1");
                }

                uint16_t n = 0;
                for (uint8_t k = 0; k < m; k++) {
                    if (k + d == m) {
                        n = 0;
                        attribute_info_.at(j).ai_attribute_partition_channels_minus1.at(i).at(k) = 0;
                    } else {
                        n = attribute_info_.at(j).ai_attribute_partition_channels_minus1.at(i).at(k);
                        WRITE_UE(stream, attribute_info_.at(j).ai_attribute_partition_channels_minus1.at(i).at(k),
                                 "ai_attribute_partition_channels_minus1");
                    }
                    d -= n + 1;
                }
                attribute_info_.at(j).ai_attribute_partition_channels_minus1.at(i).at(m) = d;

                WRITE_U(stream, attribute_info_.at(j).ai_attribute_2d_bit_depth_minus1.at(i), 5, "ai_attribute_2d_bit_depth_minus1");
                WRITE_U(stream, int(attribute_info_.at(j).ai_attribute_MSB_align_flag.at(i)), 1, "ai_attribute_MSB_align_flag");
            }
        }
        WRITE_U(stream, vps_extension_present_flag_, 1, "vps_extension_present_flag");

        if (vps_extension_present_flag_) {
            WRITE_U(stream, vps_packing_information_present_flag_, 1, "vps_packing_information_present_flag");
            WRITE_U(stream, vps_miv_extension_present_flag_, 1, "vps_miv_extension_present_flag");
            WRITE_U(stream, vps_extension_6bits_, 6, "vps_extension_6bits");
        }
        // No packing information
        // No MIV extension
        // No VPS extension
        uvg_bitstream_align(stream);
    }
    return true;
}

profile_tier_level vps::fill_ptl(std::size_t& len) const {
    profile_tier_level ptl;
    ptl.ptl_tier_flag = false;  // to do: Check if 1 (true) should be used instead
    ptl.ptl_profile_codec_group_idc = codec_group_;
    ptl.ptl_profile_toolset_idc = 1;         // V-PCC Extended, to do: Add V-PCC Basic
    ptl.ptl_profile_reconstruction_idc = 1;  // Rec0 reconstruction profile, to do: Should this be 255?
    ptl.ptl_max_decodes_idc = 15;            // to do: 15 comes from tmc2, why? is it correct?
    ptl.ptl_level_idc = 30;
    ptl.ptl_num_sub_profiles = 0;               // default (0)
    ptl.ptl_extended_sub_profile_flag = false;  // default (false)
    // ptl_sub_profile_idc                   // default (no sub-profiles)

    ptl.ptl_toolset_constraints_present_flag = false;  // default (false)
    ptl.ptc = profile_toolset_constraints_information();
    ptl.ptc.ptc_one_v3c_frame_only_flag = false;
    ptl.ptc.ptc_eom_constraint_flag = false;
    ptl.ptc.ptc_plr_constraint_flag = false;
    ptl.ptc.ptc_no_eight_orientations_constraint_flag = false;
    ptl.ptc.ptc_no_45degree_projection_patch_constraint_flag = false;

    len += 72 + ptl.ptl_num_sub_profiles * (ptl.ptl_extended_sub_profile_flag ? 64 : 32);
    const std::size_t ptc_len = 40 + (ptl.ptc.ptc_num_reserved_constraint_bytes * 8);
    len += (ptl.ptl_toolset_constraints_present_flag ? ptc_len : 0);

    return ptl;
}