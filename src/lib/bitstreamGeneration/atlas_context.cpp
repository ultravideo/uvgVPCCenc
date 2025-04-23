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


#include "atlas_context.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>

#include "atlas_frame.hpp"
#include "bitstream_common.hpp"
#include "bitstream_util.hpp"
#include "uvgvpcc/uvgvpcc.hpp"

atlas_tile_header atlas_context::create_atlas_tile_header(size_t frameIndex, size_t tileIndex,
                                                          const uvgvpcc_enc::Parameters& paramUVG) const {
    atlas_tile_header ath;

    ath.ath_no_output_of_prior_atlas_frames_flag = false;  // by default
    ath.ath_atlas_frame_parameter_set_id = 0;
    ath.ath_atlas_adaptation_parameter_set_id = 0;
    ath.ath_id = tileIndex;
    // const uint16_t tileID = ath.ath_id;

    ath.ath_type = ATH_TYPE::I_TILE;
    if (afps_.afps_output_flag_present_flag) {
        ath.ath_atlas_output_flag = false;  // by default
    }
    const size_t Log2MaxAtlasFrmOrderCntLsb = asps_.asps_log2_max_atlas_frame_order_cnt_lsb_minus4 + 4;
    ath.ath_atlas_frm_order_cnt_lsb = frameIndex % (static_cast<size_t>(1) << Log2MaxAtlasFrmOrderCntLsb);

    ath.ath_ref_atlas_frame_list_asps_flag = false;  // default value, usually changes to true below
    if (asps_.asps_num_ref_atlas_frame_lists_in_asps > 0) {
        ath.ath_ref_atlas_frame_list_asps_flag = true;
    }
    if (!ath.ath_ref_atlas_frame_list_asps_flag) {
        std::cout << "ERROR: NOT IMPLEMENTED" << std::endl;
        const ref_list_struct refs;  // TODO(lf): fill values
    } else if (asps_.asps_num_ref_atlas_frame_lists_in_asps > 1) {
        ath.ath_ref_atlas_frame_list_idx = 0;  // default value
    }
    const size_t NumLtrAtlasFrmEntries = 1;  // default value, usually a single ref list
    for (size_t j = 0; j < NumLtrAtlasFrmEntries; j++) {
        ath.ath_additional_afoc_lsb_present_flag.push_back(false);
        if (ath.ath_additional_afoc_lsb_present_flag.back()) {
            ath.ath_additional_afoc_lsb_val.push_back(0);  // default value
        }
    }
    if (ath.ath_type != ATH_TYPE::SKIP_TILE) {
        if (asps_.asps_normal_axis_limits_quantization_enabled_flag) {
            ath.ath_pos_min_d_quantizer = static_cast<uint8_t>(std::log2(paramUVG.minLevel));
            if (asps_.asps_normal_axis_max_delta_value_enabled_flag) {
                ath.ath_pos_delta_max_d_quantizer = static_cast<uint8_t>(std::log2(paramUVG.minLevel));
            }
        }
        if (asps_.asps_patch_size_quantizer_present_flag) {
            ath.ath_patch_size_x_info_quantizer = paramUVG.log2QuantizerSizeX;
            ath.ath_patch_size_y_info_quantizer = paramUVG.log2QuantizerSizeY;
        }

        const size_t geometryNominal2dBitdepth = 8; // TMC2 : Bit depth of geometry 2D
        if (afps_.afps_raw_3d_offset_bit_count_explicit_mode_flag) {
            ath.ath_raw_3d_offset_axis_bit_count_minus1 = paramUVG.geoBitDepthInput + 1 - geometryNominal2dBitdepth - 1;
        }
        if (ath.ath_type == ATH_TYPE::P_TILE && NumLtrAtlasFrmEntries > 1) {
            ath.ath_num_ref_idx_active_override_flag = false;  // default value
            if (ath.ath_num_ref_idx_active_override_flag) {
                ath.ath_num_ref_idx_active_minus1 = 0;  // default value
            }
        }
    }
    return ath;
}

atlas_tile_data_unit atlas_context::create_atlas_tile_data_unit(const uvgvpcc_enc::Parameters& paramUVG, const uvgvpcc_enc::Frame& frameUVG,
                                                                atlas_tile_header& ath) const {
    (void)paramUVG;

    atlas_tile_data_unit atdu;
    // all patches
    // const size_t quantizerSizeX = 1U << paramUVG.log2QuantizerSizeX;
    // const size_t quantizerSizeY = 1U << paramUVG.log2QuantizerSizeY;
    const size_t levelOfDetailX = 1;  // lf addition, in TMC2 those are patch parameters. However, they are also global parameters.
    const size_t levelOfDetailY = 1;  // TODO(lf): check if those are really constant, and do not depends on other parameters

    for (size_t patch_index = 0; patch_index < frameUVG.patchList.size(); ++patch_index) {
        // std::cout << "-- DEBUG: Creating atlas patch, index: " << patch_index << std::endl;
        const uvgvpcc_enc::Patch& patchUVG = frameUVG.patchList[patch_index];
        const uint8_t patchMode = static_cast<uint8_t>(ATDU_PATCH_MODE_I_TILE::I_INTRA);
        patch_information_data pid;
        pid.patchMode = patchMode;
        patch_data_unit& pdu = pid.patch_data_unit_;

        pdu.pdu_2d_pos_x = patchUVG.omDSPosX_;
        pdu.pdu_2d_pos_y = patchUVG.omDSPosY_;
        pdu.pdu_2d_size_x_minus1 = patchUVG.widthInOccBlk_ - 1;
        pdu.pdu_2d_size_y_minus1 = patchUVG.heightInOccBlk_ - 1;
        pdu.pdu_3d_offset_u = patchUVG.posU_;
        pdu.pdu_3d_offset_v = patchUVG.posV_;

        const size_t minLevel = static_cast<size_t>(pow(2., ath.ath_pos_min_d_quantizer));
        // Update from commit "Integration of m62985 patch A." of TMC2 version 22
        pdu.pdu_3d_offset_d = (patchUVG.posD_ / minLevel);

        // if( asps_normal_axis_max_delta_value_enabled_flag ) == true
        const size_t quantDD = patchUVG.sizeD_ == 0 ? 0 : ((patchUVG.sizeD_ + 1) / minLevel);
        pdu.pdu_3d_range_d = quantDD;

        pdu.pdu_projection_id = patchUVG.patchPpi_;
        pdu.pdu_orientation_index = static_cast<size_t>(patchUVG.axisSwap_);

        if (afps_.afps_lod_mode_enabled_flag) {
            pdu.pdu_lod_enabled_flag = (levelOfDetailX > 1 || levelOfDetailY > 1);
            if (pdu.pdu_lod_enabled_flag) {
                pdu.pdu_lod_scale_x_minus1 = 0;
                pdu.pdu_lod_scale_y_idc = 0;
            }
        }
        atdu.patch_information_data_.push_back(pid);
    }
    // Last patch is I_END patch
    patch_information_data end_patch;
    end_patch.patchMode = ATDU_PATCH_MODE_I_TILE::I_END;
    atdu.patch_information_data_.push_back(end_patch);

    return atdu;
}

atlas_tile_layer_rbsp atlas_context::create_atlas_tile_layer_rbsp(size_t frameIndex, size_t tileIndex,
                                                                  const uvgvpcc_enc::Parameters& paramUVG,
                                                                  const uvgvpcc_enc::Frame& frameUVG) {
    atlas_tile_layer_rbsp rbsp;

    // This should be enough for now
    rbsp.ath_ = create_atlas_tile_header(frameIndex, tileIndex, paramUVG);
    atlas_tile_header& ath = rbsp.ath_;

    // This should be enough for now
    rbsp.atdu_ = create_atlas_tile_data_unit(paramUVG, frameUVG, ath);
    const atlas_tile_data_unit& atdu = rbsp.atdu_;

    return rbsp;
}

atlas_frame_tile_information atlas_context::create_atlas_frame_tile_information() const {
    const size_t NumPartitionsInAtlasFrame = 1;  // TODO(lf)get this // lf : change from 0 to 1 to avoid overflow when -1 is applied later
    // TODO(lf): this is not complete, its not used yet anyways

    atlas_frame_tile_information afti;
    afti.afti_single_tile_in_atlas_frame_flag = true;
    if (!afti.afti_single_tile_in_atlas_frame_flag) {
        afti.afti_uniform_partition_spacing_flag = false;
        if (afti.afti_uniform_partition_spacing_flag) {
            afti.afti_partition_cols_width_minus1 = 0;
            afti.afti_partition_rows_height_minus1 = 0;
        } else {
            afti.afti_num_partition_columns_minus1 = 0;
            afti.afti_num_partition_rows_minus1 = 0;

            afti.afti_partition_column_width_minus1.resize(afti.afti_num_partition_columns_minus1 + 1, 0);
            afti.afti_partition_column_width_minus1.resize(afti.afti_num_partition_rows_minus1 + 1, 0);

            for (size_t i = 0; i < afti.afti_num_partition_columns_minus1; ++i) {
                afti.afti_partition_column_width_minus1.at(i) = 0;
            }
            for (size_t i = 0; i < afti.afti_num_partition_rows_minus1; ++i) {
                afti.afti_partition_row_height_minus1.at(i) = 0;
            }
        }
        afti.afti_single_partition_per_tile_flag = false;
        if (!afti.afti_single_partition_per_tile_flag) {
            afti.afti_top_left_partition_idx.resize(afti.afti_num_tiles_in_atlas_frame_minus1 + 1, 0);
            afti.afti_bottom_right_partition_column_offset.resize(afti.afti_num_tiles_in_atlas_frame_minus1 + 1, 0);
            afti.afti_bottom_right_partition_row_offset.resize(afti.afti_num_tiles_in_atlas_frame_minus1 + 1, 0);

            afti.afti_num_tiles_in_atlas_frame_minus1 = 0;
            for (size_t i = 0; i < afti.afti_num_tiles_in_atlas_frame_minus1; ++i) {
                afti.afti_top_left_partition_idx.at(i) = 0;
                afti.afti_bottom_right_partition_column_offset.at(i) = 0;
                afti.afti_bottom_right_partition_row_offset.at(i) = 0;
            }
        } else {
            afti.afti_num_tiles_in_atlas_frame_minus1 = NumPartitionsInAtlasFrame - 1;
        }
    } else {
        afti.afti_num_tiles_in_atlas_frame_minus1 = 0;

        afti.afti_single_partition_per_tile_flag = false;
        afti.afti_uniform_partition_spacing_flag = false;
    }
    if (asps_.asps_auxiliary_video_enabled_flag) {
        afti.afti_auxiliary_video_tile_row_width_minus1 = 0;
        afti.afti_auxiliary_video_tile_row_height.resize(afti.afti_num_tiles_in_atlas_frame_minus1 + 1, 0);
        for (size_t i = 0; i < afti.afti_num_tiles_in_atlas_frame_minus1 + 1; i++) {
            afti.afti_auxiliary_video_tile_row_height.at(i) = 0;
        }
    }
    afti.afti_signalled_tile_id_flag = false;
    if (afti.afti_signalled_tile_id_flag) {
        afti.afti_signalled_tile_id_length_minus1 = 0;
        afti.afti_tile_id.resize(afti.afti_num_tiles_in_atlas_frame_minus1 + 1, 0);
        for (size_t i = 0; i < afti.afti_num_tiles_in_atlas_frame_minus1 + 1; i++) {
            afti.afti_tile_id.at(i) = 0;
        }
    }
    // TODO(lf): missing some values
    return afti;
}

atlas_frame_parameter_set atlas_context::create_atlas_frame_parameter_set() {
    atlas_frame_parameter_set afps;
    afps.afps_atlas_frame_parameter_set_id = 0;
    afps.afps_atlas_sequence_parameter_set_id = 0;
    afps.afti = create_atlas_frame_tile_information();
    afps.afps_output_flag_present_flag = false;
    afps.afps_num_ref_idx_default_active_minus1 = 0;
    afps.afps_additional_lt_afoc_lsb_len = 0;
    afps.afps_lod_mode_enabled_flag = false;
    afps.afps_raw_3d_offset_bit_count_explicit_mode_flag = false;
    afps.afps_extension_present_flag = true;
    afps.afps_miv_extension_present_flag = false;
    afps.afps_extension_7bits = 0;

    return afps;
}

atlas_sequence_parameter_set atlas_context::create_atlas_sequence_parameter_set(const std::shared_ptr<uvgvpcc_enc::GOF>& gofUVG,
                                                                                const uvgvpcc_enc::Parameters& paramUVG) {
    atlas_sequence_parameter_set asps;

    asps.asps_atlas_sequence_parameter_set_id = 0;
    asps.asps_frame_width = paramUVG.mapWidth;
    asps.asps_frame_height = gofUVG->mapHeightGOF;
    asps.asps_geometry_3d_bit_depth_minus1 = paramUVG.geoBitDepthInput;
    const size_t geometryNominal2dBitdepth = 8; // TMC2 : Bit depth of geometry 2D
    asps.asps_geometry_2d_bit_depth_minus1 = geometryNominal2dBitdepth - 1;
    asps.asps_log2_max_atlas_frame_order_cnt_lsb_minus4 = 10 - 4;
    asps.asps_max_dec_atlas_frame_buffering_minus1 = 0;
    asps.asps_long_term_ref_atlas_frames_flag = false;
    asps.asps_num_ref_atlas_frame_lists_in_asps = 1;

    ref_list_struct refs;
    refs.num_ref_entries = 1;
    for (size_t i = 0; i < refs.num_ref_entries; ++i) {
        refs.st_ref_atlas_frame_flag.push_back(true);
        if (asps.asps_long_term_ref_atlas_frames_flag) {
            refs.st_ref_atlas_frame_flag.at(i) = false;
        }
        if (refs.st_ref_atlas_frame_flag.at(i)) {
            refs.abs_delta_afoc_st.push_back(1);
            if (refs.abs_delta_afoc_st.at(i) > 0) {
                refs.straf_entry_sign_flag.push_back(true);
            }
        }
    }
    asps.ref_lists.push_back(refs);

    asps.asps_use_eight_orientations_flag = false;
    asps.asps_extended_projection_enabled_flag = false;
    if (asps.asps_extended_projection_enabled_flag) {
        asps.asps_max_number_projections_minus1 = 6 - 1;
    }
    asps.asps_normal_axis_limits_quantization_enabled_flag = true;
    asps.asps_normal_axis_max_delta_value_enabled_flag = true;
    asps.asps_patch_precedence_order_flag = false;
    asps.asps_log2_patch_packing_block_size = static_cast<uint8_t>(std::log2(paramUVG.occupancyMapDSResolution));
    asps.asps_patch_size_quantizer_present_flag = false;
    asps.asps_map_count_minus1 = paramUVG.doubleLayer ? 1 : 0;
    asps.asps_pixel_deinterleaving_enabled_flag = false;

    if (asps.asps_pixel_deinterleaving_enabled_flag) {
        for (size_t j = 0; j < asps.asps_map_count_minus1; ++j) {
            asps.asps_map_pixel_deinterleaving_flag.push_back(false);
        }
    }

    asps.asps_raw_patch_enabled_flag = false;
    asps.asps_eom_patch_enabled_flag = false;
    if (asps.asps_eom_patch_enabled_flag && asps.asps_map_count_minus1 == 0) {
        const size_t EOMFixBitCount_ = 2;
        asps.asps_eom_fix_bit_count_minus1 = EOMFixBitCount_ - 1;
    }
    if (asps.asps_raw_patch_enabled_flag || asps.asps_eom_patch_enabled_flag) {
        const bool useRawPointsSeparateVideo = false;
        asps.asps_auxiliary_video_enabled_flag = useRawPointsSeparateVideo;
    }
    asps.asps_plr_enabled_flag = false;
    /* Not needed since above
    if( asps_plr_enabled_flag )
        asps_plr_information( asps_map_count_minus1 ) */

    asps.asps_vui_parameters_present_flag = false;
    /* Not needed since above
    if( asps_vui_parameters_present_flag )
        vui_parameters( ) */

    asps.asps_extension_present_flag = true;
    if (asps.asps_extension_present_flag) {
        asps.asps_vpcc_extension_present_flag = true;
        asps.asps_miv_extension_present_flag = false;
        asps.asps_extension_6bits = 0;
    }

    if (asps.asps_vpcc_extension_present_flag) {
        asps.asps_vpcc_remove_duplicate_point_enabled_flag = true;
        if (asps.asps_pixel_deinterleaving_enabled_flag || asps.asps_plr_enabled_flag) {
            asps.asps_vpcc_surface_thickness_minus1 = paramUVG.surfaceThickness - 1;
        }
    }
    return asps;
}

void atlas_context::initialize_atlas_context(const std::shared_ptr<uvgvpcc_enc::GOF>& gofUVG, const uvgvpcc_enc::Parameters& paramUVG) {
    gof_id_ = gofUVG->gofId;
    asps_ = create_atlas_sequence_parameter_set(gofUVG, paramUVG);
    afps_ = create_atlas_frame_parameter_set();
    // Create create_atlas_tile_layer_rbsp for each atlas frame/NAL unit
    for (size_t frame_index = 0; frame_index < gofUVG->nbFrames; ++frame_index) {
        // std::cout << "DEBUG: Creating atlas RBSP, index: " << frame_index << std::endl;
        auto& frameUVG = *(gofUVG->frames[frame_index]);

        const size_t tile_index = 0;  // Always 0, because we only have one tile per frame
        const atlas_tile_layer_rbsp rbsp = create_atlas_tile_layer_rbsp(frame_index, tile_index, paramUVG, frameUVG);
        atlas_data_.push_back(rbsp);
    }
    calculate_atlas_size_values();
}

void atlas_context::write_nal_hdr(bitstream_t* stream, const uint8_t nal_type, const uint8_t nal_layer_id,
                                  const uint8_t nal_temporal_id_plus1) {
    uvg_bitstream_put(stream, 0, 1);
    uvg_bitstream_put(stream, nal_type, 6);
    uvg_bitstream_put(stream, nal_layer_id, 6);
    uvg_bitstream_put(stream, nal_temporal_id_plus1, 3);
}

void atlas_context::write_atlas_seq_parameter_set(bitstream_t* stream) {
    WRITE_UE(stream, asps_.asps_atlas_sequence_parameter_set_id, "asps_atlas_sequence_parameter_set_id");
    WRITE_UE(stream, asps_.asps_frame_width, "asps_frame_width");
    WRITE_UE(stream, asps_.asps_frame_height, "asps_frame_height");
    WRITE_U(stream, uint8_t(asps_.asps_geometry_3d_bit_depth_minus1), 5, "asps_geometry_3d_bit_depth_minus1");
    WRITE_U(stream, uint8_t(asps_.asps_geometry_2d_bit_depth_minus1), 5, "asps_geometry_2d_bit_depth_minus1");
    WRITE_UE(stream, uint8_t(asps_.asps_log2_max_atlas_frame_order_cnt_lsb_minus4), "asps_log2_max_atlas_frame_order_cnt_lsb_minus4");
    WRITE_UE(stream, asps_.asps_max_dec_atlas_frame_buffering_minus1, "asps_max_dec_atlas_frame_buffering_minus1");

    WRITE_U(stream, asps_.asps_long_term_ref_atlas_frames_flag, 1, "asps_long_term_ref_atlas_frames_flag");
    WRITE_UE(stream, asps_.asps_num_ref_atlas_frame_lists_in_asps, "asps_num_ref_atlas_frame_lists_in_asps");

    for (size_t i = 0; i < asps_.asps_num_ref_atlas_frame_lists_in_asps; i++) {
        const ref_list_struct& ref = asps_.ref_lists.at(i);
        WRITE_UE(stream, ref.num_ref_entries, "num_ref_entries");
        for (size_t i = 0; i < ref.num_ref_entries; ++i) {
            if (asps_.asps_long_term_ref_atlas_frames_flag) {
                WRITE_U(stream, ref.st_ref_atlas_frame_flag.at(i), 1, "st_ref_atlas_frame_flag");
            }
            if (ref.st_ref_atlas_frame_flag.at(i)) {
                WRITE_UE(stream, ref.abs_delta_afoc_st.at(i), "abs_delta_afoc_st");
                if (ref.abs_delta_afoc_st.at(i) > 0) {
                    WRITE_U(stream, ref.straf_entry_sign_flag.at(i), 1, "straf_entry_sign_flag");
                }
            }
        }
    }
    WRITE_U(stream, asps_.asps_use_eight_orientations_flag, 1, "asps_use_eight_orientations_flag");
    WRITE_U(stream, asps_.asps_extended_projection_enabled_flag, 1, "asps_extended_projection_enabled_flag");

    if (asps_.asps_extended_projection_enabled_flag) {  // false
        WRITE_UE(stream, int(asps_.asps_max_number_projections_minus1), "asps_max_number_projections_minus1");
    }
    WRITE_U(stream, asps_.asps_normal_axis_limits_quantization_enabled_flag, 1, "asps_normal_axis_limits_quantization_enabled_flag");
    WRITE_U(stream, asps_.asps_normal_axis_max_delta_value_enabled_flag, 1, "asps_normal_axis_max_delta_value_enabled_flag");
    WRITE_U(stream, asps_.asps_patch_precedence_order_flag, 1, "asps_patch_precedence_order_flag");
    WRITE_U(stream, asps_.asps_log2_patch_packing_block_size, 3, "asps_log2_patch_packing_block_size");
    WRITE_U(stream, asps_.asps_patch_size_quantizer_present_flag, 1, "asps_patch_size_quantizer_present_flag");
    WRITE_U(stream, asps_.asps_map_count_minus1, 4, "asps_map_count_minus1");
    WRITE_U(stream, asps_.asps_pixel_deinterleaving_enabled_flag, 1, "asps_pixel_deinterleaving_enabled_flag");

    if (asps_.asps_pixel_deinterleaving_enabled_flag) {  // false
        for (size_t j = 0; j < asps_.asps_map_count_minus1; ++j) {
            WRITE_U(stream, int(asps_.asps_map_pixel_deinterleaving_flag.at(j)), 1, "asps_map_pixel_deinterleaving_flag");
        }
    }
    WRITE_U(stream, asps_.asps_raw_patch_enabled_flag, 1, "asps_raw_patch_enabled_flag");
    WRITE_U(stream, asps_.asps_eom_patch_enabled_flag, 1, "asps_eom_patch_enabled_flag");

    if (asps_.asps_eom_patch_enabled_flag && asps_.asps_map_count_minus1 == 0) {
        WRITE_U(stream, asps_.asps_eom_fix_bit_count_minus1, 4, "asps_eom_fix_bit_count_minus1");
    }
    if (asps_.asps_raw_patch_enabled_flag || asps_.asps_eom_patch_enabled_flag) {
        WRITE_U(stream, asps_.asps_auxiliary_video_enabled_flag, 4, "asps_auxiliary_video_enabled_flag");
    }
    WRITE_U(stream, asps_.asps_plr_enabled_flag, 1, "asps_plr_enabled_flag");
    /* Not needed since above
    if( asps_plr_enabled_flag )
        asps_plr_information( asps_map_count_minus1 ) */

    WRITE_U(stream, asps_.asps_vui_parameters_present_flag, 1, "asps_vui_parameters_present_flag");
    /* Not needed since above
    if( asps_vui_parameters_present_flag )
        vui_parameters( ) */

    WRITE_U(stream, asps_.asps_extension_present_flag, 1, "asps_extension_present_flag");

    if (asps_.asps_extension_present_flag) {
        WRITE_U(stream, asps_.asps_vpcc_extension_present_flag, 1, "asps_vpcc_extension_present_flag");
        WRITE_U(stream, asps_.asps_miv_extension_present_flag, 1, "asps_miv_extension_present_flag");
        WRITE_U(stream, asps_.asps_extension_6bits, 6, "asps_extension_6bits");
    }
    if (asps_.asps_vpcc_extension_present_flag) {
        WRITE_U(stream, asps_.asps_vpcc_remove_duplicate_point_enabled_flag, 1, "asps_vpcc_remove_duplicate_point_enabled_flag");
        if (asps_.asps_pixel_deinterleaving_enabled_flag || asps_.asps_plr_enabled_flag) {
            WRITE_UE(stream, asps_.asps_vpcc_surface_thickness_minus1, "asps_vpcc_surface_thickness_minus1");
        }
    }
    uvg_bitstream_align(stream);
}

void atlas_context::write_atlas_adaption_parameter_set(bitstream_t* stream) { uvg_bitstream_align(stream); }

void atlas_context::write_atlas_frame_parameter_set(bitstream_t* stream) const {
    WRITE_UE(stream, afps_.afps_atlas_frame_parameter_set_id, "afps_atlas_frame_parameter_set_id");
    WRITE_UE(stream, afps_.afps_atlas_sequence_parameter_set_id, "afps_atlas_sequence_parameter_set_id");
    WRITE_U(stream, afps_.afti.afti_single_tile_in_atlas_frame_flag, 1, "afti_single_tile_in_atlas_frame_flag");
    // parts of atlas frame tile information not written for now, as it it not used
    WRITE_U(stream, afps_.afti.afti_signalled_tile_id_flag, 1, "afti_signalled_tile_id_flag");
    WRITE_U(stream, afps_.afps_output_flag_present_flag, 1, "afps_output_flag_present_flag");
    WRITE_UE(stream, afps_.afps_num_ref_idx_default_active_minus1, "afps_num_ref_idx_default_active_minus1");
    WRITE_UE(stream, afps_.afps_additional_lt_afoc_lsb_len, "afps_additional_lt_afoc_lsb_len");
    WRITE_U(stream, afps_.afps_lod_mode_enabled_flag, 1, "afps_lod_mode_enabled_flag");
    WRITE_U(stream, afps_.afps_raw_3d_offset_bit_count_explicit_mode_flag, 1, "afps_raw_3d_offset_bit_count_explicit_mode_flag");
    WRITE_U(stream, afps_.afps_extension_present_flag, 1, "afps_extension_present_flag");
    WRITE_U(stream, afps_.afps_miv_extension_present_flag, 1, "afps_miv_extension_present_flag");
    WRITE_U(stream, afps_.afps_extension_7bits, 7, "afps_extension_7bits");
    uvg_bitstream_align(stream);
}

void atlas_context::write_atlas_tile_header(bitstream_t* stream, NAL_UNIT_TYPE nalu_t, const atlas_tile_header& ath) const {
    if (nalu_t >= NAL_GBLA_W_LP && nalu_t <= NAL_RSV_IRAP_ACL_29) {
        WRITE_U(stream, ath.ath_no_output_of_prior_atlas_frames_flag, 1, "ath_no_output_of_prior_atlas_frames_flag");
    }
    WRITE_UE(stream, ath.ath_atlas_frame_parameter_set_id, "ath_atlas_frame_parameter_set_id");
    WRITE_UE(stream, ath.ath_atlas_adaptation_parameter_set_id, "ath_atlas_adaptation_parameter_set_id");
    WRITE_U(stream, ath.ath_id, 0, "ath_id");  // TODO(lf): Dynamic bit length
    // const uint16_t tileID = ath.ath_id;              // ath_id: This doesnt get written, as the length gets inferred to u(0)
    WRITE_UE(stream, ath.ath_type, "ath_type");
    if (afps_.afps_output_flag_present_flag) {
        WRITE_U(stream, ath.ath_atlas_output_flag, 1, "ath_atlas_output_flag");
    }
    const size_t Log2MaxAtlasFrmOrderCntLsb = asps_.asps_log2_max_atlas_frame_order_cnt_lsb_minus4 + 4;
    WRITE_U(stream, int(ath.ath_atlas_frm_order_cnt_lsb), int(Log2MaxAtlasFrmOrderCntLsb), "ath_atlas_frm_order_cnt_lsb");  // u(v)

    if (asps_.asps_num_ref_atlas_frame_lists_in_asps > 0) {
        WRITE_U(stream, ath.ath_ref_atlas_frame_list_asps_flag, 1, "ath_ref_atlas_frame_list_asps_flag");
    }
    if (!ath.ath_ref_atlas_frame_list_asps_flag) {
        std::cout << "ERROR: NOT IMPLEMENTED" << std::endl;
        return;
    }
    if (asps_.asps_num_ref_atlas_frame_lists_in_asps > 1) {
        const size_t bit_len = std::ceil(std::log2(asps_.asps_num_ref_atlas_frame_lists_in_asps));
        WRITE_U(stream, int(ath.ath_ref_atlas_frame_list_idx), int(bit_len), "ath_ref_atlas_frame_list_idx");
    }
    const size_t NumLtrAtlasFrmEntries = 0;  // default value, ref list is from ASPS TODO(lf): dynamic
    for (size_t j = 0; j < NumLtrAtlasFrmEntries; j++) {
        WRITE_U(stream, ath.ath_additional_afoc_lsb_present_flag.at(j), 1, "ath_additional_afoc_lsb_present_flag");
        if (ath.ath_additional_afoc_lsb_present_flag.at(j)) {
            WRITE_U(stream, ath.ath_additional_afoc_lsb_val.at(j), afps_.afps_additional_lt_afoc_lsb_len, "ath_additional_afoc_lsb_val");
        }
    }
    if (ath.ath_type != SKIP_TILE) {
        if (asps_.asps_normal_axis_limits_quantization_enabled_flag) {
            WRITE_U(stream, ath.ath_pos_min_d_quantizer, 5, "ath_pos_min_d_quantizer");
            if (asps_.asps_normal_axis_max_delta_value_enabled_flag) {
                WRITE_U(stream, ath.ath_pos_delta_max_d_quantizer, 5, "ath_pos_delta_max_d_quantizer");
            }
        }
        if (asps_.asps_patch_size_quantizer_present_flag) {
            WRITE_U(stream, ath.ath_patch_size_x_info_quantizer, 3, "ath_patch_size_x_info_quantizer");
            WRITE_U(stream, ath.ath_patch_size_y_info_quantizer, 3, "ath_patch_size_y_info_quantizer");
        }
        if (afps_.afps_raw_3d_offset_bit_count_explicit_mode_flag) {
            const size_t bit_len = std::floor(std::log2(asps_.asps_geometry_3d_bit_depth_minus1 + 1));
            WRITE_U(stream, int(ath.ath_raw_3d_offset_axis_bit_count_minus1), int(bit_len), "ath_raw_3d_offset_axis_bit_count_minus1");
        }
        if (ath.ath_type == ATH_TYPE::P_TILE && NumLtrAtlasFrmEntries > 1) {
            WRITE_U(stream, ath.ath_num_ref_idx_active_override_flag, 1, "ath_num_ref_idx_active_override_flag");
            if (ath.ath_num_ref_idx_active_override_flag) {
                WRITE_UE(stream, ath.ath_num_ref_idx_active_minus1, "ath_num_ref_idx_active_minus1");
            }
        }
    }
    uvg_bitstream_align(stream);
}

void atlas_context::write_atlas_tile_data_unit(bitstream_t* stream, const atlas_tile_data_unit& atdu, const atlas_tile_header& ath) {
    // const uint16_t tileID = ath.ath_id;
    if (ath.ath_type == SKIP_TILE) {
        // skipPatchDataUnit(bitstream);
        //  This is just empty?
    } else {
        for (size_t puCount = 0; puCount < atdu.patch_information_data_.size(); puCount++) {
            uvg_bitstream_put_ue(stream, atdu.patch_information_data_.at(puCount).patchMode);

            if (atdu.patch_information_data_.at(puCount).patchMode == ATDU_PATCH_MODE_I_TILE::I_END) {
                break;
            }
            const patch_information_data pid = atdu.patch_information_data_.at(puCount);
            write_patch_information_data(stream, pid, ath);
        }
    }
}

void atlas_context::write_patch_information_data(bitstream_t* stream, const patch_information_data& pid, const atlas_tile_header& ath) {
    // if (ath.ath_type == SKIP_TILE) {
    //     // skip mode: currently not supported but added it for convenience. Could
    //     // easily be removed
    // } else if (ath.ath_type == P_TILE) {
    //     if (pid.patchMode == P_SKIP) {
    //         // skip mode: currently not supported but added it for convenience. Could
    //         // easily be removed
    //         // skipPatchDataUnit(bitstream);
    //     } else if (pid.patchMode == P_MERGE) {
    //         const auto& mpdu = pid.merge_patch_data_unit_;
    //         // mergePatchDataUnit(mpdu, ath, syntax, bitstream);
    //     } else if (pid.patchMode == P_INTRA) {
    //         const auto& pdu = pid.patch_data_unit_;
    //         // patchDataUnit(pdu, ath, syntax, bitstream);
    //     } else if (pid.patchMode == P_INTER) {
    //         const auto& ipdu = pid.inter_patch_data_unit_;
    //         // interPatchDataUnit(ipdu, ath, syntax, bitstream);
    //     } else if (pid.patchMode == P_RAW) {
    //         const auto& rpdu = pid.raw_patch_data_unit_;
    //         // rawPatchDataUnit(rpdu, ath, syntax, bitstream);
    //     } else if (pid.patchMode == P_EOM) {
    //         const auto& epdu = pid.eom_patch_data_unit_;
    //         // eomPatchDataUnit(epdu, ath, syntax, bitstream);
    //     }
    // } else if (ath.ath_type == I_TILE) {  // currently only use I_TILE types
    //     if (pid.patchMode == I_INTRA) {
    //         const auto& pdu = pid.patch_data_unit_;
    //         write_patch_data_unit(stream, pdu, ath);
    //         // patchDataUnit(pdu, ath, syntax, bitstream);
    //     } else if (pid.patchMode == I_RAW) {
    //         const auto& rpdu = pid.raw_patch_data_unit_;
    //         // rawPatchDataUnit(rpdu, ath, syntax, bitstream);
    //     } else if (pid.patchMode == I_EOM) {
    //         const auto& epdu = pid.eom_patch_data_unit_;
    //         // eomPatchDataUnit(epdu, ath, syntax, bitstream);
    //     }
    // }

    // currently only use I_TILE types
    assert(ath.ath_type == I_TILE);
    assert(pid.patchMode == I_INTRA);
    const auto& pdu = pid.patch_data_unit_;
    write_patch_data_unit(stream, pdu, ath);
}

void atlas_context::write_patch_data_unit(bitstream_t* stream, const patch_data_unit& pdu, const atlas_tile_header& ath) const {
    uvg_bitstream_put_ue(stream, pdu.pdu_2d_pos_x);
    uvg_bitstream_put_ue(stream, pdu.pdu_2d_pos_y);
    uvg_bitstream_put_ue(stream, pdu.pdu_2d_size_x_minus1);
    uvg_bitstream_put_ue(stream, pdu.pdu_2d_size_y_minus1);

    uvg_bitstream_put(stream, pdu.pdu_3d_offset_u, asps_.asps_geometry_3d_bit_depth_minus1 + 1);
    uvg_bitstream_put(stream, pdu.pdu_3d_offset_v, asps_.asps_geometry_3d_bit_depth_minus1 + 1);
    uvg_bitstream_put(stream, pdu.pdu_3d_offset_d, asps_.asps_geometry_3d_bit_depth_minus1 - ath.ath_pos_min_d_quantizer + 1);

    if (asps_.asps_normal_axis_max_delta_value_enabled_flag) {
        const uint32_t rangeDBitDepth = std::min(asps_.asps_geometry_2d_bit_depth_minus1, asps_.asps_geometry_3d_bit_depth_minus1) + 1;
        uvg_bitstream_put(stream, pdu.pdu_3d_range_d, rangeDBitDepth - ath.ath_pos_delta_max_d_quantizer);
    }
    uvg_bitstream_put(stream, pdu.pdu_projection_id, ceil(log2(6)));
    uvg_bitstream_put(stream, pdu.pdu_orientation_index, false ? 3 : 1);

    if (afps_.afps_lod_mode_enabled_flag) {
        uvg_bitstream_put(stream, static_cast<uint32_t>(pdu.pdu_lod_enabled_flag), 1);
        if (pdu.pdu_lod_enabled_flag) {
            uvg_bitstream_put_ue(stream, pdu.pdu_lod_scale_x_minus1);
            uvg_bitstream_put_ue(stream, pdu.pdu_lod_scale_y_idc);
        }
    }
    // if( asps_plr_enabled_flag )               == false
    // if( asps_miv_extension_present_flag )     == false
}

void atlas_context::write_access_unit_delimiter(bitstream_t* stream) {
    uvg_bitstream_put(stream, 0, 3);  // I_TILE
    uvg_bitstream_add_rbsp_trailing_bits(stream);
}

void atlas_context::write_atlas_tile_layer_rbsp(bitstream_t* stream, NAL_UNIT_TYPE nalu_t, const atlas_tile_layer_rbsp& rbsp) {
    write_atlas_tile_header(stream, nalu_t, rbsp.ath_);
    write_atlas_tile_data_unit(stream, rbsp.atdu_, rbsp.ath_);
    uvg_bitstream_add_rbsp_trailing_bits(stream);
}

void atlas_context::calculate_atlas_size_values() {
    bitstream_t temp_bitstream;
    uvg_bitstream_init(&temp_bitstream);
    uint32_t nal_max_size = 0;
    atlas_sub_size_ = 1;  // NAL sample stream header

    uint32_t previous_bitstream_size = 0;
    uint32_t current_bitstream_size = 0;
    uint32_t current_nal_size = 0;

    ad_nal_sizes_ = {};
    ad_nal_precision_ = 0;

    // ASPS NAL unit
    write_nal_hdr(&temp_bitstream, NAL_ASPS, 0, 1);
    write_atlas_seq_parameter_set(&temp_bitstream);
    current_bitstream_size = uvg_bitstream_tell(&temp_bitstream) / 8;
    // std::cout << "current_bitstream_size: " << current_bitstream_size << std::endl;

    current_nal_size = current_bitstream_size - previous_bitstream_size;
    ad_nal_sizes_.push_back(current_nal_size);
    // std::cout << "ASPS size: " << current_nal_size << std::endl;
    atlas_sub_size_ += current_nal_size;
    previous_bitstream_size = current_bitstream_size;

    // AFPS NAL unit
    write_nal_hdr(&temp_bitstream, NAL_AFPS, 0, 1);
    write_atlas_frame_parameter_set(&temp_bitstream);
    current_bitstream_size = uvg_bitstream_tell(&temp_bitstream) / 8;

    current_nal_size = current_bitstream_size - previous_bitstream_size;
    ad_nal_sizes_.push_back(current_nal_size);
    atlas_sub_size_ += current_nal_size;
    previous_bitstream_size = current_bitstream_size;

    for (size_t i = 0; i < atlas_data_.size(); ++i) {
        write_nal_hdr(&temp_bitstream, NAL_IDR_N_LP, 0, 1);  // TODO(lf): Dynamic NALU type
        write_atlas_tile_layer_rbsp(&temp_bitstream, NAL_IDR_N_LP, atlas_data_.at(i));
        current_bitstream_size = uvg_bitstream_tell(&temp_bitstream) / 8;
        current_nal_size = current_bitstream_size - previous_bitstream_size;
        ad_nal_sizes_.push_back(current_nal_size);
        atlas_sub_size_ += current_nal_size;
        if (current_nal_size > nal_max_size) {
            nal_max_size = current_nal_size;
        }

        previous_bitstream_size = current_bitstream_size;
    }

    const uint32_t nal_precision_minus1 =
        static_cast<uint32_t>(std::min(std::max(static_cast<int>(ceil(static_cast<double>(ceilLog2(nal_max_size + 1)) / 8.0)), 1), 8) - 1);

    ad_nal_precision_ = nal_precision_minus1 + 1;
    atlas_sub_size_ += 2 * ad_nal_precision_ + atlas_data_.size() * ad_nal_precision_;
    atlas_sub_size_ += ad_nal_precision_ + 2;  // NAL_EOB unit!

    uvg_bitstream_clear(&temp_bitstream);
}

void atlas_context::write_atlas_sub_bitstream(bitstream_t* stream) {
    const uint32_t nal_precision_in_bits = ad_nal_precision_ * 8;

    // Atlas NAL sample stream header
    uvg_bitstream_put(stream, ad_nal_precision_ - 1, 3);
    uvg_bitstream_put(stream, 0, 5);

    // ASPS NALU
    uvg_bitstream_put(stream, ad_nal_sizes_.at(0), nal_precision_in_bits);
    write_nal_hdr(stream, NAL_ASPS, 0, 1);
    write_atlas_seq_parameter_set(stream);

    // AFPS NAL unit
    uvg_bitstream_put(stream, ad_nal_sizes_.at(1), nal_precision_in_bits);
    write_nal_hdr(stream, NAL_AFPS, 0, 1);
    write_atlas_frame_parameter_set(stream);

    for (size_t i = 0; i < atlas_data_.size(); ++i) {
        uvg_bitstream_put(stream, ad_nal_sizes_.at(i + 2), nal_precision_in_bits);  // i + 2 = Skip ASPS and AFPS
        write_nal_hdr(stream, NAL_IDR_N_LP, 0, 1);                                  // TODO(lf): Dynamic NALU type
        write_atlas_tile_layer_rbsp(stream, NAL_IDR_N_LP, atlas_data_.at(i));
    }

    uvg_bitstream_put(stream, 2, nal_precision_in_bits);  // nal header + 1 byte NAL payload
    write_nal_hdr(stream, NAL_EOB, 0, 1);
    // No payload in end of bitstream NAL unit
}

void atlas_context::write_atlas_parameter_set_nals(bitstream_t* stream) {
    const uint32_t nal_precision_in_bits = ad_nal_precision_ * 8;

    // ASPS NALU
    uvg_bitstream_put(stream, ad_nal_sizes_.at(0), nal_precision_in_bits);
    write_nal_hdr(stream, NAL_ASPS, 0, 1);
    write_atlas_seq_parameter_set(stream);

    // AFPS NAL unit
    uvg_bitstream_put(stream, ad_nal_sizes_.at(1), nal_precision_in_bits);
    write_nal_hdr(stream, NAL_AFPS, 0, 1);
    write_atlas_frame_parameter_set(stream);
}

void atlas_context::write_atlas_nal(bitstream_t* stream, size_t index) {
    const uint32_t nal_precision_in_bits = ad_nal_precision_ * 8;

    uvg_bitstream_put(stream, ad_nal_sizes_.at(index + 2), nal_precision_in_bits);  // index + 2 = Skip ASPS and AFPS
    write_nal_hdr(stream, NAL_IDR_N_LP, 0, 1);                                      // TODO(lf): Dynamic NALU type
    write_atlas_tile_layer_rbsp(stream, NAL_IDR_N_LP, atlas_data_.at(index));
}

void atlas_context::write_atlas_eob(bitstream_t* stream) {
    const uint32_t nal_precision_in_bits = ad_nal_precision_ * 8;

    uvg_bitstream_put(stream, 2, nal_precision_in_bits);  // length 2 for NAL header
    write_nal_hdr(stream, NAL_EOB, 0, 1);
    // NO payload, only header
}