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

#include "gof.hpp"

#include <cstring>
#include <memory>
#include <new>
#include <stdexcept>
#include <utility>
#include <vector>

#include "bitstream_common.hpp"
#include "bitstream_util.hpp"
#include "uvgvpcc/log.hpp"
#include "uvgvpcc/uvgvpcc.hpp"
#include "video_sub_bitstream.hpp"
#include <cstdint>

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-owning-memory,cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays)

void v3c_gof::write_v3c_chunk(uvgvpcc_enc::API::v3c_unit_stream* out) {
    // --------------- Write V3C unit stream -----------------------------------------------------
    out->v3c_unit_size_precision_bytes = v3c_unit_precision_;
    bitstream_t* stream = new bitstream_t;
    uvg_bitstream_init(stream);
    uvgvpcc_enc::API::v3c_chunk new_chunk;

    const size_t vps_id = gof_id_ % 16;  // The value of vps_v3c_parameter_set_id shall be in the range of 0 to 15
    // V3C_VPS
    const size_t v3c_vps_unit_size = 4 + v3c_vps_sub_->get_vps_byte_len();  // 4 is v3c header
    new_chunk.v3c_unit_sizes.push_back(v3c_vps_unit_size);
    uvg_bitstream_put(stream, 0, 32);  // V3C_VPS unit header is just 4 bytes of 0
    v3c_vps_sub_->write_vps(stream);

    // V3C_AD
    auto* atlas = v3c_ad_unit_.get();
    new_chunk.v3c_unit_sizes.push_back(atlas->get_atlas_sub_size() + 4U);
    // V3C_AD header
    uvg_bitstream_put(stream, V3C_UNIT_TYPE::V3C_AD, 5);  // vuh_unit_type
    uvg_bitstream_put(stream, vps_id, 4);                 // vuh_v3c_parameter_set_id
    uvg_bitstream_put(stream, 0, 6);                      // vuh_atlas_id
    uvg_bitstream_put(stream, 0, 17);                     // vuh_reserved_zero_17bits
    atlas->write_atlas_sub_bitstream(stream);

    // V3C_OVD
    new_chunk.v3c_unit_sizes.push_back(v3c_ovd_sub_->size() + 4U);
    // V3C_OVD header
    uvg_bitstream_put(stream, V3C_UNIT_TYPE::V3C_OVD, 5);  // vuh_unit_type
    uvg_bitstream_put(stream, vps_id, 4);                  // vuh_v3c_parameter_set_id
    uvg_bitstream_put(stream, 0, 6);                       // vuh_atlas_id
    uvg_bitstream_put(stream, 0, 17);                      // vuh_reserved_zero_17bits
    // V3C_OVD NAL sub-bitstream
    uvg_bitstream_copy_bytes(stream, reinterpret_cast<uint8_t*>(v3c_ovd_sub_->data()), v3c_ovd_sub_->size());

    // V3C_GVD
    new_chunk.v3c_unit_sizes.push_back(v3c_gvd_sub_->size() + 4);
    // V3C_GVD header
    uvg_bitstream_put(stream, V3C_UNIT_TYPE::V3C_GVD, 5);  // vuh_unit_type
    uvg_bitstream_put(stream, vps_id, 4);                  // vuh_v3c_parameter_set_id
    uvg_bitstream_put(stream, 0, 6);                       // vuh_atlas_id
    uvg_bitstream_put(stream, 0, 4);                       // vuh_map_index
    uvg_bitstream_put(stream, 0, 1);                       // vuh_auxiliary_video_flag
    uvg_bitstream_put(stream, 0, 12);                      // vuh_reserved_zero_12bits
    // V3C_GVD NAL sub-bitstream
    uvg_bitstream_copy_bytes(stream, reinterpret_cast<uint8_t*>(v3c_gvd_sub_->data()), v3c_gvd_sub_->size());

    // V3C_AVD
    new_chunk.v3c_unit_sizes.push_back(v3c_avd_sub_->size() + 4);
    // V3C_AVD header
    uvg_bitstream_put(stream, V3C_UNIT_TYPE::V3C_AVD, 5);  // vuh_unit_type
    uvg_bitstream_put(stream, vps_id, 4);                  // vuh_v3c_parameter_set_id
    uvg_bitstream_put(stream, 0, 6);                       // vuh_atlas_id
    uvg_bitstream_put(stream, 0, 7);                       // vuh_attribute_index
    uvg_bitstream_put(stream, 0, 5);                       // vuh_attribute_partition_index
    uvg_bitstream_put(stream, 0, 4);                       // vuh_map_index
    uvg_bitstream_put(stream, 0, 1);                       // vuh_auxiliary_video_flag
    // V3C_AVD NAL sub-bitstream
    uvg_bitstream_copy_bytes(stream, reinterpret_cast<uint8_t*>(v3c_avd_sub_->data()), v3c_avd_sub_->size());

    // Last, write the chunks into a buffer
    uvg_data_chunk* data_out = nullptr;

    // Get stream length before taking chunks since that clears the stream.
    new_chunk.len = uvg_bitstream_tell(stream) / 8;
    new_chunk.data = std::make_unique<char[]>(new_chunk.len);
    data_out = stream->first;

    uint64_t written = 0;
    if (data_out != nullptr) {
        // Write data into the output file.
        for (uvg_data_chunk* chunk = data_out; chunk != nullptr; chunk = chunk->next) {
            memcpy(new_chunk.data.get() + written, &chunk->data, chunk->len);
            written += chunk->len;
        }
    }

    uvg_bitstream_finalize(stream);

    if (new_chunk.len != written) {
        throw std::runtime_error("Bitstream writing : Error: out.len != written ");
    }
    out->io_mutex.lock();
    out->v3c_chunks.push(std::move(new_chunk));
    out->io_mutex.unlock();
    uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::TRACE, "BITSTREAM GENERATION",
                             "New V3C chunk created, " + std::to_string(out->v3c_chunks.size()) + " chunk(s) in buffer. \n");
}

void v3c_gof::write_v3c_ld_chunk(const std::vector<nal_info>& ovd_nals, const std::vector<nal_info>& gvd_nals,
                                 const std::vector<nal_info>& avd_nals, uvgvpcc_enc::API::v3c_unit_stream* out, bool double_layer) {
    // --------------- Write Low Delay V3C unit stream -----------------------------------------------------
    out->v3c_unit_size_precision_bytes = v3c_unit_precision_;
    bitstream_t* stream = new bitstream_t;
    uvg_bitstream_init(stream);
    uvgvpcc_enc::API::v3c_chunk new_chunk;

    const size_t vps_id = gof_id_ % 16;  // The value of vps_v3c_parameter_set_id shall be in the range of 0 to 15
    // V3C_VPS - one per GOF
    const size_t v3c_vps_unit_size = 4 + v3c_vps_sub_->get_vps_byte_len();  // 4 is v3c header
    new_chunk.v3c_unit_sizes.push_back(v3c_vps_unit_size);
    uvg_bitstream_put(stream, 0, 32);  // V3C_VPS unit header is just 4 bytes of 0
    v3c_vps_sub_->write_vps(stream);

    size_t ovd_idx = 4;  // parameter sets x3 and SEI prefix
    size_t gvd_idx = 4;  // parameter sets x3 and SEI prefix
    size_t avd_idx = 4;  // parameter sets x3 and SEI prefix
    auto* atlas = v3c_ad_unit_.get();
    std::vector<size_t> ad_nal_sizes = atlas->get_ad_nal_sizes();
    for (size_t k = 0; k < n_frames_; ++k) {
        // V3C_AD size: V3C hdr, NAL sample header, NAL precision, NAL size
        size_t current_v3c_ad_unit_size = 4 + 1 + atlas->get_ad_nal_precision() + ad_nal_sizes.at(k + 2);  // 0 and 1 are ASPS and AFPS
        if (k == 0) {  // First AD unit, so copy also parameter sets
            current_v3c_ad_unit_size += ad_nal_sizes.at(0) + ad_nal_sizes.at(1) + (atlas->get_ad_nal_precision() * 2);
        }
        current_v3c_ad_unit_size += atlas->get_ad_nal_precision() + 2;  // NAL_EOB size
        new_chunk.v3c_unit_sizes.push_back(current_v3c_ad_unit_size);
        // V3C_AD header
        uvg_bitstream_put(stream, V3C_UNIT_TYPE::V3C_AD, 5);  // vuh_unit_type
        uvg_bitstream_put(stream, vps_id, 4);                 // vuh_v3c_parameter_set_id
        uvg_bitstream_put(stream, 0, 6);                      // vuh_atlas_id
        uvg_bitstream_put(stream, 0, 17);                     // vuh_reserved_zero_17bits

        // Atlas NAL sample stream header
        uvg_bitstream_put(stream, atlas->get_ad_nal_precision() - 1, 3);
        uvg_bitstream_put(stream, 0, 5);

        if (k == 0) {  // First AD unit, so copy also parameter sets
            atlas->write_atlas_parameter_set_nals(stream);
        }
        atlas->write_atlas_nal(stream, k);

        atlas->write_atlas_eob(stream);

        // V3C_OVD unit size
        size_t current_v3c_ovd_unit_size = 4;  // V3C header
        if (k == 0) {                               // First OVD unit, so copy also parameter sets AND SEI prefix
            for (size_t n = 0; n < 4; n++) {
                current_v3c_ovd_unit_size += 4 + ovd_nals.at(n).size;
            }
        }
        for (size_t n = 0; n < 1; n++) {  // 1 media NAL unit
            current_v3c_ovd_unit_size += 4 + ovd_nals.at(ovd_idx).size;
            ovd_idx++;
        }
        // std::cout << "current_v3c_ovd_unit_size " << current_v3c_ovd_unit_size << std::endl;
        new_chunk.v3c_unit_sizes.push_back(current_v3c_ovd_unit_size);
        // V3C_OVD header
        uvg_bitstream_put(stream, V3C_UNIT_TYPE::V3C_OVD, 5);  // vuh_unit_type
        uvg_bitstream_put(stream, vps_id, 4);                  // vuh_v3c_parameter_set_id
        uvg_bitstream_put(stream, 0, 6);                       // vuh_atlas_id
        uvg_bitstream_put(stream, 0, 17);                      // vuh_reserved_zero_17bits
        ovd_idx -= 1;                                          // reset index after size calculation
        // V3C_OVD NAL sub-bitstream
        if (k == 0) {  // First OVD unit, so copy also parameter sets AND SEI prefix
            for (size_t n = 0; n < 4; n++) {
                uvg_bitstream_put(stream, ovd_nals.at(n).size, 32);
                uvg_bitstream_copy_bytes(stream, reinterpret_cast<uint8_t*>(v3c_ovd_sub_->data() + ovd_nals.at(n).location),
                                         ovd_nals.at(n).size);
            }
        }
        // Picture NAL unit
        uvg_bitstream_put(stream, ovd_nals.at(ovd_idx).size, 32);
        uvg_bitstream_copy_bytes(stream, reinterpret_cast<uint8_t*>(v3c_ovd_sub_->data() + ovd_nals.at(ovd_idx).location),
                                 ovd_nals.at(ovd_idx).size);
        ovd_idx++;

        // V3C_GVD size
        size_t current_v3c_gvd_unit_size = 4;  // V3C header
        if (k == 0) {                               // First GVD unit, so copy also parameter sets AND SEI prefix
            for (size_t n = 0; n < 4; n++) {
                current_v3c_gvd_unit_size += 4 + gvd_nals.at(n).size;
            }
        }
        current_v3c_gvd_unit_size += 4 + gvd_nals.at(gvd_idx).size;
        if (double_layer) {  // If doubleLayer, there is 2 NAL units per frame
            current_v3c_gvd_unit_size += 4 + gvd_nals.at(gvd_idx + 1).size;
        }
        // std::cout << "current_v3c_gvd_unit_size " << current_v3c_gvd_unit_size << std::endl;
        new_chunk.v3c_unit_sizes.push_back(current_v3c_gvd_unit_size);
        // V3C_GVD header
        uvg_bitstream_put(stream, V3C_UNIT_TYPE::V3C_GVD, 5);  // vuh_unit_type
        uvg_bitstream_put(stream, vps_id, 4);                  // vuh_v3c_parameter_set_id
        uvg_bitstream_put(stream, 0, 6);                       // vuh_atlas_id
        uvg_bitstream_put(stream, 0, 4);                       // vuh_map_index
        uvg_bitstream_put(stream, 0, 1);                       // vuh_auxiliary_video_flag
        uvg_bitstream_put(stream, 0, 12);                      // vuh_reserved_zero_12bits
        // V3C_GVD NAL sub-bitstream
        if (k == 0) {  // First GVD unit, so copy also parameter sets AND SEI prefix
            for (size_t n = 0; n < 4; n++) {
                uvg_bitstream_put(stream, gvd_nals.at(n).size, 32);
                uvg_bitstream_copy_bytes(stream, reinterpret_cast<uint8_t*>(v3c_gvd_sub_->data() + gvd_nals.at(n).location),
                                         gvd_nals.at(n).size);
            }
        }
        uvg_bitstream_put(stream, gvd_nals.at(gvd_idx).size, 32);
        uvg_bitstream_copy_bytes(stream, reinterpret_cast<uint8_t*>(v3c_gvd_sub_->data() + gvd_nals.at(gvd_idx).location),
                                 gvd_nals.at(gvd_idx).size);
        gvd_idx++;
        if (double_layer) {  // If doubleLayer, there is 2 NAL units per frame
            uvg_bitstream_put(stream, gvd_nals.at(gvd_idx).size, 32);
            uvg_bitstream_copy_bytes(stream, reinterpret_cast<uint8_t*>(v3c_gvd_sub_->data() + gvd_nals.at(gvd_idx).location),
                                     gvd_nals.at(gvd_idx).size);
            gvd_idx++;
        }

        // V3C_AVD size
        size_t current_v3c_avd_unit_size = 4;  // V3C header
        if (k == 0) {                               // First AVD unit, so copy also parameter sets AND SEI prefix
            for (size_t n = 0; n < 4; n++) {
                current_v3c_avd_unit_size += 4 + avd_nals.at(n).size;
            }
        }
        current_v3c_avd_unit_size += 4 + avd_nals.at(avd_idx).size;
        if (double_layer) {  // If doubleLayer, there is 2 NAL units per frame
            current_v3c_avd_unit_size += 4 + avd_nals.at(avd_idx + 1).size;
        }
        // std::cout << "current_v3c_avd_unit_size " << current_v3c_avd_unit_size << std::endl;
        new_chunk.v3c_unit_sizes.push_back(current_v3c_avd_unit_size);
        // V3C_AVD header
        uvg_bitstream_put(stream, V3C_UNIT_TYPE::V3C_AVD, 5);  // vuh_unit_type
        uvg_bitstream_put(stream, vps_id, 4);                  // vuh_v3c_parameter_set_id
        uvg_bitstream_put(stream, 0, 6);                       // vuh_atlas_id
        uvg_bitstream_put(stream, 0, 7);                       // vuh_attribute_index
        uvg_bitstream_put(stream, 0, 5);                       // vuh_attribute_partition_index
        uvg_bitstream_put(stream, 0, 4);                       // vuh_map_index
        uvg_bitstream_put(stream, 0, 1);                       // vuh_auxiliary_video_flag
        // V3C_AVD NAL sub-bitstream
        if (k == 0) {  // First AVD unit, so copy also parameter sets AND SEI prefix
            for (size_t n = 0; n < 4; n++) {
                uvg_bitstream_put(stream, avd_nals.at(n).size, 32);
                uvg_bitstream_copy_bytes(stream, reinterpret_cast<uint8_t*>(v3c_avd_sub_->data() + avd_nals.at(n).location),
                                         avd_nals.at(n).size);
            }
        }
        uvg_bitstream_put(stream, avd_nals.at(avd_idx).size, 32);
        uvg_bitstream_copy_bytes(stream, reinterpret_cast<uint8_t*>(v3c_avd_sub_->data() + avd_nals.at(avd_idx).location),
                                 avd_nals.at(avd_idx).size);
        avd_idx++;
        if (double_layer) {  // If doubleLayer, there is 2 NAL units per frame
            uvg_bitstream_put(stream, avd_nals.at(avd_idx).size, 32);
            uvg_bitstream_copy_bytes(stream, reinterpret_cast<uint8_t*>(v3c_avd_sub_->data() + avd_nals.at(avd_idx).location),
                                     avd_nals.at(avd_idx).size);
            avd_idx++;
        }
    }
    // Last, write the chunks into a buffer
    uvg_data_chunk* data_out = nullptr;

    // Get stream length before taking chunks since that clears the stream.
    new_chunk.len = uvg_bitstream_tell(stream) / 8;
    new_chunk.data = std::make_unique<char[]>(new_chunk.len);

    data_out = uvg_bitstream_take_chunks(stream);
    uvg_bitstream_finalize(stream);

    uint64_t written = 0;
    if (data_out != nullptr) {
        // Write data into the output file.
        for (uvg_data_chunk* chunk = data_out; chunk != nullptr; chunk = chunk->next) {
            memcpy(new_chunk.data.get() + written, &chunk->data, chunk->len);
            written += chunk->len;
        }
    }

    if (new_chunk.len != written) {
        throw std::runtime_error("Bitstream writing : Error: out.len != written ");
    }
    out->io_mutex.lock();
    out->v3c_chunks.push(std::move(new_chunk));
    out->io_mutex.unlock();
    uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::TRACE, "BITSTREAM GENERATION",
                             "New V3C LD chunk created, " + std::to_string(out->v3c_chunks.size()) + " chunk(s) in buffer. \n");
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-owning-memory,cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays)
