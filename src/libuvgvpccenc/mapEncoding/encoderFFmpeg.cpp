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

#include "encoderFFmpeg.hpp"

#include <cassert>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "abstract2DMapEncoder.hpp"
#include "catchLibLog.hpp"
#include "utils/fileExport.hpp"
#include "utils/parameters.hpp"
#include "uvgutils/log.hpp"
#include "uvgvpcc/uvgvpcc.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libavutil/log.h>
#include <libavutil/mathematics.h>
#include <libavutil/opt.h>
#include <libavutil/timestamp.h>
}

using namespace uvgvpcc_enc;

static void ffmpeg_log_callback(void* ptr, int level, const char* fmt, va_list vl) {
    static const uvgutils::LogLevel levelMap[] = {
        uvgutils::LogLevel::FATAL,    // AV_LOG_PANIC (0)
        uvgutils::LogLevel::FATAL,    // AV_LOG_FATAL (8)
        uvgutils::LogLevel::ERROR,    // AV_LOG_ERROR (16)
        uvgutils::LogLevel::WARNING,  // AV_LOG_WARNING (24)
        uvgutils::LogLevel::INFO,     // AV_LOG_INFO (32)
        uvgutils::LogLevel::DEBUG,    // AV_LOG_VERBOSE (40)
        uvgutils::LogLevel::TRACE,    // AV_LOG_DEBUG (48)
        uvgutils::LogLevel::DEBUG     // AV_LOG_TRACE (56) -- if present
    };

    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), fmt, vl);

    int index = std::min(level >> 3, static_cast<int>(sizeof(levelMap) / sizeof(levelMap[0]) - 1));
    uvgutils::Logger::log<uvgutils::LogLevel::DEBUG>("FFmpeg", std::string(buffer));
}

void EncoderFFmpeg::initializeLogCallback() {
    av_log_set_level(AV_LOG_DEBUG);
    av_log_set_callback(ffmpeg_log_callback);
}

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

void set_ffmpeg_options(AVDictionary* opts, const std::map<std::string, std::string>& options) {
    for (const auto& option : options) {
        av_dict_set(&opts, option.first.c_str(), option.second.c_str(), 0);
    }
}

void setFFmpegConfig(const AVCodec* codec, AVCodecContext* codec_ctx, const size_t& width, const size_t& height,
                     const ENCODER_TYPE& encoderType, const std::string& encoderName, const std::string& codecParamsEnabler,
                     const std::string& codecParams, const std::map<std::string, std::string>& codecOptions) {
    // Basic config
    codec_ctx->width = width;
    codec_ctx->height = height;
    codec_ctx->pix_fmt = AV_PIX_FMT_YUV420P;
    codec_ctx->time_base = {1, 25};
    codec_ctx->framerate = {25, 1};
    codec_ctx->max_b_frames = 0;

    AVDictionary* opts = NULL;

    if (!codecOptions.empty()) {
        set_ffmpeg_options(opts, codecOptions);
    }
    if (!codecParamsEnabler.empty()) {
        av_dict_set(&opts, codecParamsEnabler.c_str(), codecParams.c_str(), 0);
    }

    // Map-specific settings
    switch (encoderType) {
        // codec_ctx->thread_count = 1;
        case OCCUPANCY:
            if (p_->occupancyEncodingIsLossless) {
            } else {
                throw std::runtime_error("FFmpeg encoder : uvgVPCCenc currently supports only lossless encoding for the occupancy map.\n");
            }

            if (p_->occupancyEncodingFormat == "YUV420") {
            } else {
                throw std::runtime_error(
                    "FFmpeg encoder : uvgVPCCenc currently supports only YUV420 encoding for the occupancy map. The given faulty format is: "
                    "'" +
                    p_->occupancyEncodingFormat + "'.\n");
            }

            if (p_->occupancyEncodingMode == "AI") {
                codec_ctx->gop_size = 0;
            } else if (p_->occupancyEncodingMode == "RA") {
                codec_ctx->gop_size = p_->sizeGOP2DEncoding;
            } else {
                throw std::runtime_error("EncoderFFmpeg: This occupancy map encoding mode is unknown : " + p_->occupancyEncodingMode +
                                         ". Only AI and RA are currently available.");
            }
            break;

        case GEOMETRY:
            // codec_ctx->thread_count = 1;
            if (p_->geometryEncodingIsLossless) {
            }
            if (p_->occupancyEncodingFormat == "YUV420") {
            } else {
                throw std::runtime_error(
                    "FFmepg encoder : uvgVPCCenc currently supports only YUV420 encoding for the geometry map. The given faulty format is: "
                    "'" +
                    p_->occupancyEncodingFormat + "'.\n");
            }
            if (p_->geometryEncodingMode == "AI") {
                if (p_->doubleLayer) {
                } else {
                }
                codec_ctx->gop_size = 0;
            } else if (p_->geometryEncodingMode == "RA") {
                codec_ctx->gop_size = p_->sizeGOP2DEncoding;
            } else {
                throw std::runtime_error("EncoderFFmpeg: This geometry map encoding mode is unknown : " + p_->geometryEncodingMode +
                                         ". Only AI and RA are currently available.");
            }
            break;

        case ATTRIBUTE:
            // codec_ctx->thread_count = 1;
            if (p_->attributeEncodingIsLossless) {
            }

            if (p_->occupancyEncodingFormat == "YUV420") {
            } else {
                throw std::runtime_error(
                    "FFmepg encoder : uvgVPCCenc supports only YUV420 encoding for the attribute map. The given faulty format is: '" +
                    p_->occupancyEncodingFormat + "'.\n");
            }

            if (p_->attributeEncodingMode == "AI") {
                if (p_->doubleLayer) {
                } else {
                }
                codec_ctx->gop_size = 0;
            } else if (p_->attributeEncodingMode == "RA") {
                codec_ctx->gop_size = p_->sizeGOP2DEncoding;
            } else {
                throw std::runtime_error("EncoderFFmpeg: This attribute map encoding mode is unknown : " + p_->attributeEncodingMode +
                                         ". Only AI and RA are currently available.");
            }

            break;

        default:
            assert(false);
    }

    if ((avcodec_open2(codec_ctx, codec, &opts)) < 0) {
        throw std::runtime_error(encoderName + ": Failed to open codec.");
    }
    av_dict_free(&opts);
}

static int encode(AVCodecContext* enc_ctx, AVFrame* frame, AVPacket* pkt, std::vector<uint8_t>& bitstream) {
    int ret = 0;

    ret = avcodec_send_frame(enc_ctx, frame);
    if (ret < 0) {
        fprintf(stderr, "Error sending a frame for encoding\n");
        exit(1);
    }

    while (ret >= 0) {
        ret = avcodec_receive_packet(enc_ctx, pkt);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
            return ret;
        else if (ret < 0) {
            fprintf(stderr, "Error during encoding\n");
            exit(1);
        }

        bitstream.insert(bitstream.end(), &pkt->data[0], &pkt->data[pkt->size]);
        av_packet_unref(pkt);
    }
    return ret;
}

void encodeVideoFFmpeg(const std::vector<std::reference_wrapper<std::vector<uint8_t>>>& mapList, AVCodecContext* codec_ctx,
                       const size_t width, const size_t height, std::vector<uint8_t>& bitstream, const std::string& encoderName) {
    codec_ctx->pix_fmt = AV_PIX_FMT_YUV420P;

    AVFrame* sw_frame = av_frame_alloc();
    sw_frame->format = AV_PIX_FMT_YUV420P;
    sw_frame->width = width;
    sw_frame->height = height;

    AVPacket* pkt = av_packet_alloc();
    if (!pkt) exit(1);

    av_log_set_level(AV_LOG_QUIET);

    if (av_frame_get_buffer(sw_frame, 0) < 0) {
        fprintf(stderr, "Could not allocate the video frame data\n");
        exit(1);
    }

    size_t frameCountIn = 0;
    const size_t sizeMap = width * height;
    int ret = 0;

    while (frameCountIn < mapList.size()) {
        std::vector<uint8_t>& map = mapList[frameCountIn].get();
        // Y
        sw_frame->data[0] = map.data();
        sw_frame->linesize[0] = width;
        // U
        sw_frame->data[1] = &map[sizeMap];
        sw_frame->linesize[1] = width >> 1U;
        // V
        sw_frame->data[2] = &map[sizeMap + (sizeMap >> 2U)];
        sw_frame->linesize[2] = width >> 1U;

        sw_frame->pts = frameCountIn;

        ++frameCountIn;

        ret = encode(codec_ctx, sw_frame, pkt, bitstream);
    }

    // Flush the encoder
    ret = encode(codec_ctx, NULL, pkt, bitstream);

    av_frame_free(&sw_frame);
    av_packet_free(&pkt);
    avcodec_free_context(&codec_ctx);
}

}  // anonymous namespace

void EncoderFFmpeg::encodeGOFMaps(const std::shared_ptr<uvgvpcc_enc::GOF>& gof) {
    std::string encoderName = "FFmpeg (" + codecName_ + "/" + deviceType_ + ")";  // For log and debug
    switch (encoderType_) {
        case OCCUPANCY:
            encoderName = encoderName + " occupancy map encoder";
            break;
        case GEOMETRY:
            encoderName = encoderName + " geometry map encoder";
            break;
        case ATTRIBUTE:
            encoderName = encoderName + " attribute map encoder";
            break;
        default:
            assert(false);
    }

    // Get the codec
    const AVCodec* codec = avcodec_find_encoder_by_name(codecName_.c_str());
    if (!codec) {
        throw std::runtime_error(encoderName + ": Failed to find codec.");
    }

    // Allocate codec context
    AVCodecContext* codec_ctx = avcodec_alloc_context3(codec);

    const size_t width = encoderType_ == OCCUPANCY ? p_->mapWidth / p_->occupancyMapDSResolution : p_->mapWidth;
    const size_t height = encoderType_ == OCCUPANCY ? gof->mapHeightDSGOF : gof->mapHeightGOF;
    // Set configuration for the codec
    setFFmpegConfig(codec, codec_ctx, width, height, encoderType_, encoderName, codecParamsEnabler_, codecParamsValues_, codecOptions_);

    std::vector<std::reference_wrapper<std::vector<uint8_t>>> mapList;
    setMapList(gof, mapList, encoderType_);

    std::vector<uint8_t>& bitstream = getBitstream(gof, encoderType_);

    // Encode the map video
    encodeVideoFFmpeg(mapList, codec_ctx, width, height, bitstream, encoderName);

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
