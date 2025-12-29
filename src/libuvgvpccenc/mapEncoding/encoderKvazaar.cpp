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

/// \file Interface between uvgVPCCenc and the 2D encoder Kvazaar that implement the 'abstract2DMapEncoder'.

#include "encoderKvazaar.hpp"

#include <kvazaar.h>

#include <cassert>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "abstract2DMapEncoder.hpp"
#include "catchLibLog.hpp"
#include "utils/fileExport.hpp"
#include "utils/parameters.hpp"
#include "uvgvpcc/uvgvpcc.hpp"

using namespace uvgvpcc_enc;

void EncoderKvazaar::initializeLogCallback() { kvazaar_log_callback = static_cast<lib_log_callback>(kvazaar_lib_log_callback); }

namespace {

void setMapList(const std::shared_ptr<uvgvpcc_enc::GOF>& gof, std::vector<std::reference_wrapper<std::vector<uint8_t>>>& mapList,
                const ENCODER_TYPE& encoderType) {
    mapList.reserve(gof->nbFrames);
    if (encoderType == OCCUPANCY) {
        for (const std::shared_ptr<uvgvpcc_enc::Frame>& frame : gof->frames) {
            mapList.emplace_back(frame->occupancyMapDS);
        }
        // bitstream = &gof->bitstreamOccupancy;

    } else if (encoderType == GEOMETRY) {
        for (const std::shared_ptr<uvgvpcc_enc::Frame>& frame : gof->frames) {
            mapList.emplace_back(frame->geometryMapL1);
            if (p_->doubleLayer) {
                mapList.emplace_back(frame->geometryMapL2);
            }
        }
        // bitstream = &gof->bitstreamGeometry;

    } else if (encoderType == ATTRIBUTE) {
        for (const std::shared_ptr<uvgvpcc_enc::Frame>& frame : gof->frames) {
            mapList.emplace_back(frame->attributeMapL1);
            if (p_->doubleLayer) {
                mapList.emplace_back(frame->attributeMapL2);
            }
        }
        // bitstream = &gof->bitstreamAttribute;
    } else {
        assert(false);
    }
}

void setBitstream(const std::shared_ptr<uvgvpcc_enc::GOF>& gof, std::vector<uint8_t>& bitstream, const ENCODER_TYPE& encoderType) {
    switch (encoderType) {
        case OCCUPANCY:
            bitstream = gof->bitstreamOccupancy;
            break;
        case GEOMETRY:
            bitstream = gof->bitstreamGeometry;
            break;
        case ATTRIBUTE:
            bitstream = gof->bitstreamAttribute;
            break;
        default:
            assert(false);
    }
}

std::vector<uint8_t>& getBitstream(const std::shared_ptr<uvgvpcc_enc::GOF>& gof, const ENCODER_TYPE& encoderType) {
    switch (encoderType) {
        case OCCUPANCY:
            return gof->bitstreamOccupancy;
            break;
        case GEOMETRY:
            return gof->bitstreamGeometry;
            break;
        case ATTRIBUTE:
            return gof->bitstreamAttribute;
            break;
        default:
            assert(false);
            static std::vector<uint8_t> dummy;
            return dummy;
    }
}

//TODO(lf): verify the return value for each api->config_parse (make a wrapper)
void setKvazaarConfig(kvz_api* api, kvz_config* config, const size_t& width, const size_t& height, const ENCODER_TYPE& encoderType) {
    // Basic config
    api->config_parse(config, "enable-logging", "1");  // TODO(lf) what about performance ? It should depends on the log level
    api->config_parse(config, "psnr", "0");
    api->config_parse(config, "hash", "none");
    api->config_parse(config, "width", std::to_string(width).c_str());
    api->config_parse(config, "height", std::to_string(height).c_str());
    if(!p_->encoderInfoSEI) {
        api->config_parse(config, "info", "none");
    }

    // Map-specific settings
    switch (encoderType) {
        case OCCUPANCY:
            api->config_parse(config, "threads", std::to_string(p_->occupancyEncodingNbThread).c_str());
            api->config_parse(config, "preset", p_->occupancyEncodingPreset.c_str());
            if (p_->occupancyEncodingIsLossless) {
                api->config_parse(config, "lossless", "1");
            } else {
                throw std::runtime_error("Kvazaar encoder : uvgVPCCenc currently supports only lossless encoding for the occupancy map.\n");
            }

            if (p_->occupancyEncodingFormat == "YUV420") {
                api->config_parse(config, "input-format", "P420");
            } else {
                throw std::runtime_error(
                    "Kvazaar encoder : uvgVPCCenc currently supports only YUV420 encoding for the occupancy map. The given faulty format is: "
                    "'" +
                    p_->occupancyEncodingFormat + "'.\n");
            }

            if (p_->occupancyEncodingMode == "AI") {
                api->config_parse(config, "period", "1");
                api->config_parse(config, "gop", "0");
            } else if (p_->occupancyEncodingMode == "RA") {
                api->config_parse(config, "period", std::to_string(p_->intraFramePeriod).c_str());
                api->config_parse(config, "gop", std::to_string(p_->sizeGOP2DEncoding).c_str());
            } else {
                throw std::runtime_error("EncoderKvazaar: This occupancy map encoding mode is unknown : " + p_->occupancyEncodingMode +
                                         ". Only AI and RA are currently available.");
            }
            break;

        case GEOMETRY:
            api->config_parse(config, "threads", std::to_string(p_->geometryEncodingNbThread).c_str());
            api->config_parse(config, "preset", p_->geometryEncodingPreset.c_str());
            api->config_parse(config, "qp", std::to_string(p_->geometryEncodingQp).c_str());
            if (p_->geometryEncodingIsLossless) {
                api->config_parse(config, "lossless", "1");
            }
            if (p_->occupancyEncodingFormat == "YUV420") {
                api->config_parse(config, "input-format", "P420");
            } else {
                throw std::runtime_error(
                    "Kvazaar encoder : uvgVPCCenc currently supports only YUV420 encoding for the geometry map. The given faulty format is: "
                    "'" +
                    p_->occupancyEncodingFormat + "'.\n");
            }
            if (p_->geometryEncodingMode == "AI") {
                if (p_->doubleLayer) {
                    api->config_parse(config, "period", "2");
                } else {
                    api->config_parse(config, "period", "1");
                }
                api->config_parse(config, "gop", "0");
            } else if (p_->geometryEncodingMode == "RA") {
                api->config_parse(config, "period", std::to_string(p_->intraFramePeriod).c_str());
                api->config_parse(config, "gop", std::to_string(p_->sizeGOP2DEncoding).c_str());
            } else {
                throw std::runtime_error("EncoderKvazaar: This geometry map encoding mode is unknown : " + p_->geometryEncodingMode +
                                         ". Only AI and RA are currently available.");
            }
            break;

        case ATTRIBUTE:
            api->config_parse(config, "threads", std::to_string(p_->attributeEncodingNbThread).c_str());
            api->config_parse(config, "preset", p_->attributeEncodingPreset.c_str());
            api->config_parse(config, "qp", std::to_string(p_->attributeEncodingQp).c_str());
            if (p_->attributeEncodingIsLossless) {
                api->config_parse(config, "lossless", "1");
            }

            if (p_->occupancyEncodingFormat == "YUV420") {
                api->config_parse(config, "input-format", "P420");
            } else {
                throw std::runtime_error(
                    "Kvazaar encoder : uvgVPCCenc supports only YUV420 encoding for the attribute map. The given faulty format is: '" +
                    p_->occupancyEncodingFormat + "'.\n");
            }

            if (p_->attributeEncodingMode == "AI") {
                if (p_->doubleLayer) {
                    api->config_parse(config, "period", "2");
                } else {
                    api->config_parse(config, "period", "1");
                }
                api->config_parse(config, "gop", "0");
            } else if (p_->attributeEncodingMode == "RA") {
                api->config_parse(config, "period", std::to_string(p_->intraFramePeriod).c_str());
                api->config_parse(config, "gop", std::to_string(p_->sizeGOP2DEncoding).c_str());
            } else {
                throw std::runtime_error("EncoderKvazaar: This attribute map encoding mode is unknown : " + p_->attributeEncodingMode +
                                         ". Only AI and RA are currently available.");
            }

            // lf : advice from Joose
            if (p_->attributeEncodingPreset == "veryslow") {
                api->config_parse(config, "rd", "4");
                api->config_parse(config, "full-intra-search", "1");
                api->config_parse(config, "intra-chroma-search", "1");
            }
            break;

        default:
            assert(false);
    }
}

void encodeVideoKvazaar(const std::vector<std::reference_wrapper<std::vector<uint8_t>>>& mapList, kvz_api* api, kvz_config* config,
                        const size_t width, const size_t height, std::vector<uint8_t>& bitstream, const std::string& encoderName) {
    // TODO(lf): bitstream.reserve(...)
    kvz_encoder* cpu_enc = api->encoder_open(config);
    if (cpu_enc == nullptr) {
        throw std::runtime_error(encoderName +
                                 ": Failed to open Kvazaar encoder.");  // TODO(lf): suggest to use log level debug to see Kvazaar log
    };

    kvz_data_chunk* chunks_out = nullptr;
    kvz_picture* pic = nullptr;
    uint32_t len_out = 0;
    size_t frameCountIn = 0;
    size_t frameCountOut = 0;
    const size_t sizeMap = width * height;
    while (frameCountOut < mapList.size()) {
        pic = nullptr;
        if (frameCountIn < mapList.size()) {
            pic = api->picture_alloc(static_cast<int32_t>(width), static_cast<int32_t>(height));
            if (pic == nullptr) {
                throw std::runtime_error(encoderName + ": Failed to allocate Kvazaar picture.");
            };

            // All maps are already converted to YUV420 at this moment, even occupancy and geometry maps.
            std::vector<uint8_t>& map = mapList[frameCountIn].get();
            pic->y = map.data();
            pic->u = &map[sizeMap];
            pic->v = &map[sizeMap + (sizeMap >> 2U)];
            ++frameCountIn;
        }
        api->encoder_encode(cpu_enc, pic, &chunks_out, &len_out, nullptr, nullptr, nullptr);  // kvazaar gateway
        api->picture_free(pic);                                                               // Do nothing if pic == nullptr

        if (chunks_out == nullptr) {
            continue;
        }
        for (kvz_data_chunk* chunk = chunks_out; chunk != nullptr; chunk = chunk->next) {
            bitstream.insert(bitstream.end(), &chunk->data[0], &chunk->data[chunk->len]);
        }
        api->chunk_free(chunks_out);  // finaly makes chunks_out = nullptr;
        ++frameCountOut;
    }
    api->encoder_close(cpu_enc);
}

}  // anonymous namespace

void EncoderKvazaar::encodeGOFMaps(const std::shared_ptr<uvgvpcc_enc::GOF>& gof) {
    std::string encoderName;  // For log and debug
    switch (encoderType_) {
        case OCCUPANCY:
            encoderName = "Kvazaar occupancy map encoder";
            break;
        case GEOMETRY:
            encoderName = "Kvazaar geometry map encoder";
            break;
        case ATTRIBUTE:
            encoderName = "Kvazaar attribute map encoder";
            break;
        default:
            assert(false);
    }

    kvz_api* api = const_cast<kvz_api*>(kvz_api_get(8));  // NOLINT(cppcoreguidelines-pro-type-const-cast)
    kvz_config* config = api->config_alloc();

    if (config == nullptr) {
        throw std::runtime_error(encoderName + ": Failed to allocate Kvazaar config.");
    };

    if (api->config_init(config) == 0) {
        throw std::runtime_error(encoderName + ": Failed to initialize Kvazaar config.");
    }

    const size_t width = encoderType_ == OCCUPANCY ? p_->mapWidth / p_->occupancyMapDSResolution : p_->mapWidth;
    const size_t height = encoderType_ == OCCUPANCY ? gof->mapHeightDSGOF : gof->mapHeightGOF;
    setKvazaarConfig(api, config, width, height, encoderType_);

    std::vector<std::reference_wrapper<std::vector<uint8_t>>> mapList;
    setMapList(gof, mapList, encoderType_);

    std::vector<uint8_t>& bitstream = getBitstream(gof, encoderType_);

    encodeVideoKvazaar(mapList, api, config, width, height, bitstream, encoderName);

    api->config_destroy(config);

    if (p_->exportIntermediateFiles) {
        switch (encoderType_) {
            case OCCUPANCY:
                FileExport::exportOccupancyBitstream(gof, bitstream, ".hevc");
                break;
            case GEOMETRY:
                FileExport::exportGeometryBitstream(gof, bitstream, ".hevc");
                break;
            case ATTRIBUTE:
                FileExport::exportAttributeBitstream(gof, bitstream, ".hevc");
                break;
            default:
                assert(false);
        }
    }
}
