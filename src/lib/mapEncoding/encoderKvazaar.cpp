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

/// \file Interface between uvgVPCCenc and the 2D encoder Kvazaar that implement the 'abstract2DMapEncoder'.

#include "abstract2DMapEncoder.hpp"
#include "encoderKvazaar.hpp"

#include <kvazaar.h>

#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "uvgvpcc/log.hpp"
#include "uvgvpcc/uvgvpcc.hpp"
#include "catchLibLog.hpp"

using namespace uvgvpcc_enc;


void EncoderKvazaar::initializeLogCallback() {
    kvazaar_log_callback = static_cast<lib_log_callback>(kvazaar_lib_log_callback);
}

namespace {

void encodeVideoKvazaar(const std::vector<std::reference_wrapper<std::vector<uint8_t>>>& mapList, kvz_api* api, kvz_config* config,
                        size_t width, size_t height, std::vector<uint8_t>& bitstream, const std::string& encoderName) {
    
    // TODO(lf): bitstream.reserve(...)
    kvz_encoder* cpu_enc = api->encoder_open(config);
    if (cpu_enc == nullptr) {
        throw std::runtime_error( encoderName + ": Failed to open Kvazaar encoder."); // TODO(lf): suggest to use log level debug to see Kvazaar log
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
    api->config_destroy(config);
    api->encoder_close(cpu_enc);
}

} // anonymous namespace


void EncoderKvazaar::encodeGOFMaps(std::shared_ptr<uvgvpcc_enc::GOF>& gof) {
    
    std::vector<std::reference_wrapper<std::vector<uint8_t>>> mapList;
    mapList.reserve(gof->nbFrames);
    std::vector<uint8_t> *bitstream = nullptr;
    if(encoderType_ == OCCUPANCY) {
        for (const std::shared_ptr<uvgvpcc_enc::Frame>& frame : gof->frames) {
            mapList.emplace_back(frame->occupancyMap);
        }
        bitstream = &gof->bitstreamOccupancy;

    } else if(encoderType_ == GEOMETRY) {
        for (const std::shared_ptr<uvgvpcc_enc::Frame>& frame : gof->frames) {
            mapList.emplace_back(frame->geometryMapL1);
            if (p_->doubleLayer) {
                mapList.emplace_back(frame->geometryMapL2);
            }            
        }     
        bitstream = &gof->bitstreamGeometry;

    } else { // encoderType_ == ATTRIBUTE
        for (const std::shared_ptr<uvgvpcc_enc::Frame>& frame : gof->frames) {
            mapList.emplace_back(frame->attributeMapL1);
            if (p_->doubleLayer) {
                mapList.emplace_back(frame->attributeMapL2);
            }    
        }
        bitstream = &gof->bitstreamAttribute;
        
    }

    encodeVideoKvazaar(mapList, api_, config_, width_, height_, *bitstream, encoderName_);

    if (p_->exportIntermediateMaps) { // TODO(lf): export intermediate bitstream param should be added
        if(encoderType_ == OCCUPANCY) {
            writeBitstreamToFile(*bitstream, gof->baseNameOccupancy + ".hevc");
        } else if(encoderType_ == GEOMETRY) {
            writeBitstreamToFile(*bitstream, gof->baseNameGeometry + ".hevc");
        } else { // encoderType_ == ATTRIBUTE
            writeBitstreamToFile(*bitstream, gof->baseNameAttribute + ".hevc");
        }                
    }
}

void EncoderKvazaar::configureGOFEncoder(const std::shared_ptr<uvgvpcc_enc::GOF>& gof, const ENCODER_TYPE& encoderType) {

    encoderType_ = encoderType;
    if(encoderType_ == OCCUPANCY) {
        encoderName_ = "Kvazaar occupancy map encoder";
        lossLess_ = p_->occupancyEncodingIsLossless;
        nbThread_ = p_->occupancyEncodingNbThread;
        preset_ = p_->occupancyEncodingPreset;
        mode_ = p_->occupancyEncodingMode;
        width_ = p_->mapWidth / p_->occupancyMapResolution;
        format_ = p_->occupancyEncodingFormat;
        height_ = gof->occupancyMapHeight;
        qp_ = std::numeric_limits<size_t>::max(); // lf : This value should not be used (Occupancy use lossless mode)
    } else if(encoderType_ == GEOMETRY) {
        encoderName_ = "Kvazaar geometry map encoder";
        lossLess_ = p_->geometryEncodingIsLossless;
        nbThread_ = p_->geometryEncodingNbThread;
        preset_ = p_->geometryEncodingPreset;
        mode_ = p_->geometryEncodingMode;
        width_ = p_->mapWidth;
        format_ = p_->geometryEncodingFormat;
        height_ = gof->mapsHeight;       
        qp_ = p_->geometryEncodingQp; 
    } else if(encoderType_ == ATTRIBUTE) {
        encoderName_ = "Kvazaar attribute map encoder";
        lossLess_ = p_->attributeEncodingIsLossless;
        nbThread_ = p_->attributeEncodingNbThread;
        preset_ = p_->attributeEncodingPreset;
        mode_ = p_->attributeEncodingMode;
        width_ = p_->mapWidth;
        format_ = p_->attributeEncodingFormat;
        height_ = gof->mapsHeight;        
        qp_ = p_->attributeEncodingQp; 
    } else {
        uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::FATAL, "MAP ENCODING",
            "While configuring the GOF encoder, an unknown encoderType has been used : " + std::to_string(static_cast<int>(encoderType)) + "The type list is defined by the enum 'ENCODER_TYPE' in 'abstract2DMapEncoder.hpp'.\n");
        throw std::runtime_error("");
    }

    if(format_ == "YUV420") {
        format_ = "P420";
    } else if(format_ == "YUV400") {
        format_ = "P400";
        if(encoderType == ATTRIBUTE) {
            throw std::runtime_error("Kvazaar encoder : Attribute map need to be encoded with YUV420. The given faulty format is: '" + format_+"'.\n");
        }
    } else {
        throw std::runtime_error("Kvazaar encoder : Only accepted format are YUV420 and YUV400. The given faulty format is: '" + format_+"'.\n");
    }

    api_ = const_cast<kvz_api*>(kvz_api_get(8));  // NOLINT(cppcoreguidelines-pro-type-const-cast)
    config_ = api_->config_alloc();
    if (config_ == nullptr) {
        throw std::runtime_error(encoderName_ + ": Failed to allocate Kvazaar config.");
    };

    if(api_->config_init(config_)==0) {
        throw std::runtime_error(encoderName_ + ": Failed to initialize Kvazaar config.");
    }    

    api_->config_parse(config_, "enable-logging", "1"); // TODO(lf) what about performance ? It should depends on the log level
    api_->config_parse(config_, "psnr", "0");
    api_->config_parse(config_, "hash", "none");
    api_->config_parse(config_, "threads", std::to_string(nbThread_).c_str());
    api_->config_parse(config_, "width", std::to_string(width_).c_str());
    api_->config_parse(config_, "height", std::to_string(height_).c_str());
    api_->config_parse(config_, "input-format", format_.c_str());
    api_->config_parse(config_, "preset", preset_.c_str());

    if(lossLess_) {
        api_->config_parse(config_, "lossless", "1");
    }

    if(encoderType_ == OCCUPANCY) {
        // TODO(lf): useless ?
        api_->config_parse(config_, "period", "1");
        return;
    }

    // Only for geometry and attribute maps //
    api_->config_parse(config_, "qp", std::to_string(qp_).c_str());

    if(mode_ == "AI") {
        if (p_->doubleLayer) {
            api_->config_parse(config_, "period", "2");
        } else {
            api_->config_parse(config_, "period", "1");
        }
        api_->config_parse(config_, "gop", "0");
    } else if(mode_ == "RA") {
        api_->config_parse(config_, "period", std::to_string(p_->intraFramePeriod).c_str());
        api_->config_parse(config_, "gop", std::to_string(p_->sizeGOP2DEncoding).c_str());
    } else {
        throw std::runtime_error("EncoderKvazaar: This encoding mode is unknown : " + mode_ + ". Only AI and RA are currently available.");
    }
    
    // lf : advice from Joose
    if(encoderType_ == ATTRIBUTE) {
        if (preset_ == "veryslow") {
            api_->config_parse(config_, "rd", "4");
            api_->config_parse(config_, "full-intra-search", "1");
            api_->config_parse(config_, "intra-chroma-search", "1");
        }
    }

}













