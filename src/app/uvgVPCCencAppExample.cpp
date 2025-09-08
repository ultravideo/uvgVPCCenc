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

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <iterator>
#include <memory>
#include <regex>
#include <semaphore>
#include <csignal>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <algorithm>

#include "cli.hpp"
#include "extras/miniply.h"
#include "uvgvpcc/log.hpp"
#include "uvgvpcc/uvgvpcc.hpp"
#include "../utils/utils.hpp"

#if defined(ENABLE_V3CRTP)
#include <uvgv3crtp/version.h>
#include <uvgv3crtp/v3c_api.h>
#endif

namespace {

// Semaphores for synchronization.
std::binary_semaphore available_input_slot{0};
std::binary_semaphore filled_input_slot{0};

enum class Retval : std::uint8_t {Running,Failure,Eof};

struct input_handler_args {
    // Parameters passed from main thread to input thread.
    
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
    const cli::opts_t& opts;

    // Picture and thread status passed from input thread to main thread.
    std::shared_ptr<uvgvpcc_enc::Frame> frame_in;
    Retval retval;
    input_handler_args(const cli::opts_t& opts, std::shared_ptr<uvgvpcc_enc::Frame> frame, Retval retval)
        : opts(opts), frame_in(std::move(frame)), retval(retval) {}
};


const size_t MAX_PATH_SIZE = 4096;
const size_t FORCED_V3C_SIZE_PRECISION = 5;  // Should be enough to hold any V3C unit sizes

/// @brief This function is used to write a number (V3C unit size) to a field of size len.
/// @param value 
/// @param dst 
/// @param len 
void create_bytes(uint64_t value, char* dst, size_t len) {
    const uint64_t mask = 0xFF;
    for (size_t i = 0; i < len; ++i) {
        *std::next(dst, static_cast<std::ptrdiff_t>(len - 1U - i)) = static_cast<char>((value >> (8 * i)) & mask);
        // dst[len - 1 - i] = static_cast<uint8_t>((value >> (8 * i)) & mask);
    }
}

/// @brief Simple wrapper for the miniply library for parsing a .ply file.
/// @param frame 
void loadFrameFromPlyFile(const std::shared_ptr<uvgvpcc_enc::Frame>& frame) {
    // uvgVPCCenc currently support only geometry of type unsigned int
    uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::TRACE>("APPLICATION",
                             "Loading frame " + std::to_string(frame->frameId) + " from " + frame->pointCloudPath + "\n");
    miniply::PLYReader reader(frame->pointCloudPath.c_str());
    if (!reader.valid()) {
        throw std::runtime_error("miniply : Failed to open " + frame->pointCloudPath);
    }

    // In a PLY file, an 'element' is a section of the file (it can be 'vertex' which list all the vertices, 'face' when dealing with
    // polygones etc.)
    bool vertexElementFound = false;
    while (reader.has_element()) {
        if (reader.element_is(miniply::kPLYVertexElement)) {
            vertexElementFound = true;
            break;  // Ensure that the current element is the vertex element for what follow
        }
        reader.next_element();
    }

    if (!vertexElementFound) {
        throw std::runtime_error("miniply : No vertex element (miniply::kPLYVertexElement) was found in this file : " +
                                 frame->pointCloudPath);
    }

    if (!reader.load_element()) {
        throw std::runtime_error("miniply : Vertex element did not load correctly (file: " + frame->pointCloudPath + ")");
    }

    std::array<uint32_t, 3> indicesPos{};  // Indices of position properties in the vertex line
    if (!reader.find_pos(indicesPos.data())) {
        throw std::runtime_error(
            "miniply : Position properties (x,y,z) were not located in the vertex element (file: " + frame->pointCloudPath + ")");
    }

    std::array<uint32_t, 3> indicesCol{};  // Indices of color properties in the vertex line
    if (!reader.find_color(indicesCol.data())) {
        throw std::runtime_error("miniply : Color properties (r,g,b or red,green,blue) were not located in the vertex element (file: " +
                                 frame->pointCloudPath + ")");
    }

    const size_t vertexCount = reader.element()->count;
    frame->pointsGeometry.resize(vertexCount);
    frame->pointsAttribute.resize(vertexCount);

    reader.extract_properties(indicesPos.data(), 3, miniply::PLYPropertyType::UShort, frame->pointsGeometry.data());
    reader.extract_properties(indicesCol.data(), 3, miniply::PLYPropertyType::UChar, frame->pointsAttribute.data());
    frame->printInfo();


    // Check if the point coordinates respect the voxel size
    const size_t geoBitDepthInput = uvgvpcc_enc::p_->geoBitDepthInput;
    const bool isCompliant = !std::any_of(frame->pointsGeometry.begin(), frame->pointsGeometry.end(),
        [geoBitDepthInput](const uvgvpcc_enc::Vector3<uvgvpcc_enc::typeGeometryInput>& point) {
            return (point[0] >> geoBitDepthInput) |
            (point[1] >> geoBitDepthInput) |
            (point[2] >> geoBitDepthInput);
        });

    if(!isCompliant) {
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::ERROR>("APPLICATION",
        "Frame " + std::to_string(frame->frameId) + " from " + frame->pointCloudPath + " contains at least one point which does not respect the input voxel size (uvgvpcc_enc::p_->geoBitDepthInput = " + std::to_string(uvgvpcc_enc::p_->geoBitDepthInput) + "). Maximum value is 2^"+ std::to_string(uvgvpcc_enc::p_->geoBitDepthInput) + "-1. All faulty points will not be processed.\n");            
        std::vector<uvgvpcc_enc::Vector3<uvgvpcc_enc::typeGeometryInput>> pointsGeometryTmp;
        std::vector<uvgvpcc_enc::Vector3<uint8_t>> pointsAttributeTmp;
        pointsGeometryTmp.reserve(frame->pointsGeometry.size());
        pointsAttributeTmp.reserve(frame->pointsGeometry.size());

        for(size_t pointIndex = 0; pointIndex < frame->pointsGeometry.size(); ++pointIndex) {
            const auto& point = frame->pointsGeometry[pointIndex];
            if((point[0] >> geoBitDepthInput) | (point[1] >> geoBitDepthInput) | (point[2] >> geoBitDepthInput)) continue;
            pointsGeometryTmp.emplace_back(point);
            pointsAttributeTmp.emplace_back(frame->pointsAttribute[pointIndex]);
        }
        frame->pointsGeometry.swap(pointsGeometryTmp);
        frame->pointsAttribute.swap(pointsAttributeTmp);
    }
}

/// @brief Application thread reading the input .ply files.
/// @param args 
void inputReadThread(const std::shared_ptr<input_handler_args>& args) {
    double inputReadTimerTotal = uvgvpcc_enc::p_->timerLog ? uvgvpcc_enc::global_timer.elapsed() : 0.0;
    const cli::opts_t& appParameters = args->opts;
    size_t frameId = 0;
    bool run = true;
    const size_t totalNbFrames = appParameters.nbFrames * appParameters.nbLoops;
    Retval returnValue = Retval::Running;
    // Producer thread that reads input frames.
    while (run) {

        // Signal that all input frames has been load
        if (appParameters.nbLoops != 0 && frameId == totalNbFrames) {
            available_input_slot.acquire();
            args->frame_in = nullptr;
            args->retval = Retval::Eof;
            filled_input_slot.release();
            break;
        }

        // Produce an item
        std::vector<char> pointCloudPath(MAX_PATH_SIZE);
        const int nbBytes =
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
            snprintf(pointCloudPath.data(), MAX_PATH_SIZE, appParameters.inputPath.c_str(),
                     appParameters.startFrame + (frameId % appParameters.nbFrames));
        if (nbBytes < 0) {
            uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::FATAL>("APPLICATION",
                                     "Error occurred while formatting string storing the point cloud path.\n");
            returnValue = Retval::Failure;
        }
        auto frame = std::make_shared<uvgvpcc_enc::Frame>(frameId, appParameters.startFrame + frameId,
                                                          std::string(pointCloudPath.begin(), pointCloudPath.end()));
        try {
            loadFrameFromPlyFile(frame);
        } catch (const std::runtime_error& e) {
            uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::FATAL>("APPLICATION",
                                     "Caught exception while loading frame " + std::to_string(frameId) + " from " + frame->pointCloudPath +
                                         ": " + std::string(e.what()) + "\n");
            returnValue = Retval::Failure;
        }
        
        // Signal that an item has been produced
        available_input_slot.acquire();
        args->frame_in = frame;
        if (returnValue == Retval::Failure) {
            args->retval = Retval::Failure;
            run = false;
        } else {
            assert(returnValue == Retval::Running && args->retval == Retval::Running);
        }
        filled_input_slot.release();
        
        frameId++;
    }
    if (uvgvpcc_enc::p_->timerLog) {
        inputReadTimerTotal = uvgvpcc_enc::global_timer.elapsed() - inputReadTimerTotal;
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::PROFILING>("TIMER INPUT READ TOTAL",std::to_string(inputReadTimerTotal) + " ms\n");
    }
}

/// @brief Application thread writing the final output bitstream.
/// @param chunks 
/// @param output_path 
void file_writer(uvgvpcc_enc::API::v3c_unit_stream* chunks, const std::string& output_path) {
    std::ofstream file(output_path, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Bitstream writing : Could not open output file " + output_path);
    }
    const char v3c_sample_stream_header = static_cast<char>((FORCED_V3C_SIZE_PRECISION - 1U) << 5U);
    file.write(&v3c_sample_stream_header, 1);

    while (true) {
        chunks->available_chunks.acquire();
        chunks->io_mutex.lock();
        const uvgvpcc_enc::API::v3c_chunk& chunk = chunks->v3c_chunks.front();
        if (chunk.data == nullptr && chunk.len == 0) {
            uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::TRACE>("APPLICATION", "All chunks written to file.\n");
            file.close();
            chunks->io_mutex.unlock();
            break;
        }
        if (chunk.data != nullptr) {
            std::ptrdiff_t ptr = 0;  // Keep track of ptr in the V3C unit stream
            for (const uint64_t current_size : chunk.v3c_unit_sizes) {
                // Create and write the V3C unit size to file
                std::array<char, FORCED_V3C_SIZE_PRECISION> size_field{};
                create_bytes(current_size, size_field.data(), FORCED_V3C_SIZE_PRECISION);
                file.write(size_field.data(), static_cast<std::streamsize>(FORCED_V3C_SIZE_PRECISION));

                // Write the V3C unit to file
                // NOLINTNEXTLINE(concurrency-mt-unsafe)
                file.write(std::next(chunk.data.get(), ptr), static_cast<std::streamsize>(current_size));
                ptr += static_cast<std::ptrdiff_t>(current_size);
            }
        }

        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::TRACE>("APPLICATION",
                                 "Wrote V3C chunk to file, size " + std::to_string(chunk.len) + " bytes.\n");

        chunks->v3c_chunks.pop();
        chunks->io_mutex.unlock();
    }
}

/// @brief Application thread for sending output over RTP.
/// @param chunks
/// @param output_path
void v3c_sender(uvgvpcc_enc::API::v3c_unit_stream* chunks, const std::string dst_address, const uint16_t dst_port) {
#if defined(ENABLE_V3CRTP)

    uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::TRACE>("APPLICATION", "Using uvgV3CRTP lib version " + uvgV3CRTP::get_version() + "\n");
    
    // ******** Initialize sample stream with input bitstream ***********
    //
    uvgV3CRTP::V3C_State<uvgV3CRTP::V3C_Sender> state(uvgV3CRTP::INIT_FLAGS::VPS | uvgV3CRTP::INIT_FLAGS::AD | uvgV3CRTP::INIT_FLAGS::OVD |
                                                      uvgV3CRTP::INIT_FLAGS::GVD | uvgV3CRTP::INIT_FLAGS::AVD,
                                                      dst_address.c_str(), dst_port  // Receiver address and port
    );                                                                    // Create a new state in a sender configuration
    state.init_sample_stream(FORCED_V3C_SIZE_PRECISION); // Use the forced precision for now
    //
    // ******************************************************************
    if (state.get_error_flag() != uvgV3CRTP::ERROR_TYPE::OK) {
        throw std::runtime_error(std::string("V3C Sender : Error initializing state (message: ") + state.get_error_msg() + ")"); 
    }

    // ******** Send sample stream **********
    //
    // TODO: Currently whole bitstream is stored, could also clear the sample stream at certain points
    while (state.get_error_flag() == uvgV3CRTP::ERROR_TYPE::OK) {
                
        // Get chunks and add them to state for sending
        chunks->available_chunks.acquire();
        chunks->io_mutex.lock();

        const uvgvpcc_enc::API::v3c_chunk& chunk = chunks->v3c_chunks.front();
        if (chunk.data == nullptr && chunk.len == 0) {
            uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::TRACE>("APPLICATION", "All chunks sent.\n");
            chunks->io_mutex.unlock();

            break;
        }

        // Add new data to state
        size_t len = chunk.len;
        std::ptrdiff_t ptr = 0;  // Keep track of ptr in the V3C unit stream
        for (const uint64_t current_size : chunk.v3c_unit_sizes) {
            // Add the V3C unit to existing sample stream
            // NOLINTNEXTLINE(concurrency-mt-unsafe)
            state.append_to_sample_stream(std::next(chunk.data.get(), ptr), static_cast<size_t>(current_size));
            ptr += static_cast<std::ptrdiff_t>(current_size);
        }

        chunks->v3c_chunks.pop();
        chunks->io_mutex.unlock();

        // Send newly added data
        while (state.get_error_flag() == uvgV3CRTP::ERROR_TYPE::OK) {
            // Send gof if full
            if(state.cur_gof_is_full()) {
                uvgV3CRTP::send_gof(&state);
                state.next_gof();

                uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::TRACE>("APPLICATION",
                                                                       "Sent one gof " + std::to_string(len) + " bytes.\n");
            }
            else
            {
            //TODO: track which unit has been sent
            //    for (uint8_t id = 0; id < uvgV3CRTP::NUM_V3C_UNIT_TYPES; id++) {
            //    if (!state.cur_gof_has_unit(uvgV3CRTP::V3C_UNIT_TYPE(id))) continue;

            //    // TODO: send side-channel info

            //    uvgV3CRTP::send_unit(&state, uvgV3CRTP::V3C_UNIT_TYPE(id));
            //}
                //state.next_gof();
                break;
            }
        }

        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::TRACE>("APPLICATION",
                                                               "Processed V3C chunk of size " + std::to_string(len) + " bytes.\n");

        if (state.get_error_flag() == uvgV3CRTP::ERROR_TYPE::EOS) {
            state.reset_error_flag(); //More chunks are added later so reset EOS
        }
    }

    // ******** Print info about sample stream **********
    //
    // Print state and bitstream info
    state.print_state(false);

    std::cout << "Bitstream info: " << std::endl;
    state.print_bitstream_info();

    size_t len = 0;
    auto info = std::unique_ptr<char, decltype(&free)>(state.get_bitstream_info_string(uvgV3CRTP::INFO_FMT::PARAM, &len), &free);
    uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::INFO>("APPLICATION", "Bitstream info string: \n" + std::string(info.get(), len) + "\n");
    //
    //  **************************************

    if (state.get_error_flag() != uvgV3CRTP::ERROR_TYPE::OK && state.get_error_flag() != uvgV3CRTP::ERROR_TYPE::EOS) {
        throw std::runtime_error(std::string("V3C Sender : Sending error (message: ") + state.get_error_msg() + ")");
    }
#else
    throw std::runtime_error("V3C RTP not enabled, re-run cmake with '-DENABLE_V3CRTP=ON'.");
#endif
}

/// @brief Simple application wrapper taking a command string as input to set multiple encoder parameters.
/// @param parametersCommand 
void setParameters(const std::string& parametersCommand) {
    // Iterate over each substring separated by commas
    std::string segment;
    std::stringstream ss(parametersCommand);
    while (std::getline(ss, segment, ',')) {
        if (segment.empty()) {continue;}  // Skip empty segments (e.g., trailing comma)

        // Check if the segment matches the "parameterName=parameterValue" pair pattern
        std::smatch match;
        if (std::regex_match(segment, match, std::regex(R"((\w+)=([^,]*))"))) {
            uvgvpcc_enc::API::setParameter(match[1],match[2]);
        } else {
            // If the regex does not match, the format is incorrect
            std::string errorMessage = "Invalid format detected here: '";
            errorMessage += segment;
            errorMessage += "'. Here is the expected format: 'parameterName=parameterValue'.\nThe full parameters command: ";
            errorMessage += parametersCommand;
            errorMessage += "\n";
            throw std::runtime_error(errorMessage);
        }
    }
}

}  // anonymous namespace

/// @brief This example application offers a simple approach to working with the uvgVPCCenc library and highlights the essential setup steps required for its use.
/// @param argc 
/// @param argv Application command line
/// @return 
int main(const int argc, const char* const argv[]) {
    double encodingTimerTotal = uvgvpcc_enc::p_->timerLog ? uvgvpcc_enc::global_timer.elapsed() : 0.0;

    uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::INFO>("APPLICATION", "uvgVPCCenc application starts.\n");

    // Parse application parameters //
    cli::opts_t appParameters;
    bool exitOnParse = false;
    try {
        exitOnParse = cli::opts_parse(appParameters, argc, std::span<const char* const>(argv, argc));
    } catch (const std::exception& e) {
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::FATAL>("APPLICATION",
                                 "An exception was caught during the parsing of the application parameters.\n");
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::FATAL>("APPLICATION", e.what() + std::string("\n"));
        cli::print_usage();
        return EXIT_FAILURE;
    }
    if(exitOnParse) {
        // --version or --help //
        return EXIT_SUCCESS;
    }

    // The only way for the application to change the encoder parameters is through the uvgvpcc_enc::API::setParameter(...) function. //
    try {
        setParameters(appParameters.uvgvpccParametersString);
        uvgvpcc_enc::API::setParameter("geoBitDepthInput",std::to_string(appParameters.inputGeoPrecision));
        uvgvpcc_enc::API::setParameter("nbThreadPCPart",std::to_string(appParameters.threads));
        uvgvpcc_enc::API::setParameter("occupancyEncodingNbThread",std::to_string(appParameters.threads));
        uvgvpcc_enc::API::setParameter("geometryEncodingNbThread",std::to_string(appParameters.threads));
        uvgvpcc_enc::API::setParameter("attributeEncodingNbThread",std::to_string(appParameters.threads));
    } catch (const std::exception& e) {
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::FATAL>("LIBRARY","An exception was caught when setting parameters in the application.\n");
        return EXIT_FAILURE;
    }
    
    try {
        uvgvpcc_enc::API::initializeEncoder();
    } catch (const std::exception& e) {
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::FATAL>("LIBRARY","An exception was caught during the initialization of the encoder.\n");
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::FATAL>("LIBRARY", e.what() + std::string("\n"));
        cli::print_usage();
        return EXIT_FAILURE;
    }

    // Initialize the application input and output threads
    const std::shared_ptr<input_handler_args> in_args = std::make_shared<input_handler_args>(appParameters, nullptr, Retval::Running);
    std::thread inputTh(&inputReadThread, in_args); // TODO(gg): in_args is read and modified by two threads at the same time
    size_t frameRead = 0;
    std::shared_ptr<uvgvpcc_enc::Frame> currFrame = nullptr;
    available_input_slot.release();

    uvgvpcc_enc::API::v3c_unit_stream output;  // Each v3c chunk gets appended to the V3C unit stream as they are encoded
    std::thread file_writer_thread;
    std::thread v3c_sender_thread;

    if (!appParameters.outputPath.empty()) file_writer_thread = std::thread(file_writer, &output, appParameters.outputPath);
    if (!appParameters.dstAddress.empty()) v3c_sender_thread = std::thread(v3c_sender, &output, appParameters.dstAddress, appParameters.dstPort);
    
    // Main loop of the application, feeding one frame to the encoder at each iteration
    for (;;) {
        filled_input_slot.acquire();
        currFrame = in_args->frame_in;
        in_args->frame_in = nullptr;
        if (in_args->retval == Retval::Eof) {
            break;
        }
        if (in_args->retval == Retval::Failure) {
            return EXIT_FAILURE;
        }
        available_input_slot.release();

        try {
            // Entry point of the uvgVPCCenc library
            uvgvpcc_enc::API::encodeFrame(currFrame, &output);
        } catch (const std::runtime_error& e) {
            // Only one try and catch block. All exceptions thrown by the library are catched here.
            uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::FATAL>("APPLICATION",
                                     "Caught exception using uvgvpcc_enc library: " + std::string(e.what()) + " failed after processing\n");
            return EXIT_FAILURE;
        }
        frameRead++;
    }

    // After all frames are encoded, an empty v3c_chunk is pushed to output. It signals the end of data to file_writer thread.
    uvgvpcc_enc::API::emptyFrameQueue();

    output.io_mutex.lock();
    output.v3c_chunks.emplace();  // Push empty chunk to signal end of data
    output.io_mutex.unlock();
    output.available_chunks.release();

    if(file_writer_thread.joinable()) file_writer_thread.join();
    if(v3c_sender_thread.joinable()) v3c_sender_thread.join();

    uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::INFO>("APPLICATION", "Encoded " + std::to_string(frameRead) + " frames.\n");

    inputTh.join();
    // uvgvpcc_enc::API::stopEncoder(); // lf: not usefull (call two times ThreadQueue::stop()) because of custom destructor for ThreadQueue

    if (uvgvpcc_enc::p_->timerLog) {
        encodingTimerTotal = uvgvpcc_enc::global_timer.elapsed() - encodingTimerTotal;
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::PROFILING>("TIMER ENCODING TOTAL",std::to_string(encodingTimerTotal) + " ms\n");
    }    
    return EXIT_SUCCESS;
}