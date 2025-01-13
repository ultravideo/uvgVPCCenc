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

#include <array>
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

#include "cli.hpp"
#include "miniply.h"
#include "uvgvpcc/log.hpp"
#include "uvgvpcc/uvgvpcc.hpp"


namespace {

// Semaphores for synchronization.
std::binary_semaphore available_input_slots{0};
std::binary_semaphore filled_input_slots{0};

struct input_handler_args {
    // Parameters passed from main thread to input thread.
    const cli::opts_t* opts;

    // Picture and thread status passed from input thread to main thread.
    std::shared_ptr<uvgvpcc_enc::Frame> frame_in;
    int retval;
    input_handler_args(const cli::opts_t* opts, std::shared_ptr<uvgvpcc_enc::Frame> frame, int retval)
        : opts(opts), frame_in(std::move(frame)), retval(retval) {}
};

enum : std::uint8_t { RETVAL_RUNNING, RETVAL_FAILURE, RETVAL_EOF };

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
void loadFrameFromPlyFile(std::shared_ptr<uvgvpcc_enc::Frame>& frame) {
    // uvgVPCCenc currently support only geometry of type unsigned int
    uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::TRACE, "APPLICATION",
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
}

/// @brief Application thread reading the input .ply files.
/// @param args 
void inputReadThread(const std::shared_ptr<input_handler_args>& args) {
    const cli::opts_t& appParameters = *args->opts;
    size_t frameId = 0;
    const size_t frameLimit = appParameters.frames * appParameters.loop_input;
    bool run = true;
    // Producer thread that reads input frames.
    while (run) {
        // Produce an item
        std::vector<char> pointCloudPath(MAX_PATH_SIZE);
        const int nbBytes =
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
            snprintf(pointCloudPath.data(), MAX_PATH_SIZE, appParameters.inputPath.c_str(),
                     appParameters.startFrame + (frameId % appParameters.frames));
        if (nbBytes < 0) {
            uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::FATAL, "APPLICATION",
                                     "Error occurred while formatting string storing the point cloud path.\n");
            args->retval = RETVAL_FAILURE;
        }
        auto frame = std::make_shared<uvgvpcc_enc::Frame>(frameId, appParameters.startFrame + frameId,
                                                          std::string(pointCloudPath.begin(), pointCloudPath.end()));
        try {
            loadFrameFromPlyFile(frame);
        } catch (const std::runtime_error& e) {
            uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::FATAL, "APPLICATION",
                                     "Caught exception while loading frame " + std::to_string(frameId) + " from " + frame->pointCloudPath +
                                         ": " + std::string(e.what()) + "\n");
            args->retval = RETVAL_FAILURE;
        }
        frameId++;

        // Signal that an item has been produced
        available_input_slots.acquire();
        args->frame_in = frame;
        if (frameLimit != 0 && frameId > frameLimit) {
            args->retval = RETVAL_EOF;
            run = false;
        } else if (args->retval == RETVAL_FAILURE) {
            run = false;
        } else {
            args->retval = RETVAL_RUNNING;
        }
        filled_input_slots.release();
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
            uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::TRACE, "APPLICATION", "All chunks written to file.\n");
            file.close();
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

        uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::TRACE, "APPLICATION",
                                 "Wrote V3C chunk to file, size " + std::to_string(chunk.len) + " bytes.\n");

        chunks->v3c_chunks.pop();
        chunks->io_mutex.unlock();
    }
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

    uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::INFO, "APPLICATION", "uvgVPCCenc application starts.\n");


    // Parse application parameters //
    cli::opts_t appParameters;
    bool exitOnParse = false;
    try {
        exitOnParse = cli::opts_parse(appParameters, argc, std::span<const char* const>(argv, argc));
    } catch (const std::exception& e) {
        uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::FATAL, "APPLICATION",
                                 "An exception was caught during the parsing of the application parameters.\n");
        uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::FATAL, "APPLICATION", e.what() + std::string("\n"));
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
        uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::FATAL, "LIBRARY","An exception was caught when setting parameters in the application.\n");
        return EXIT_FAILURE;
    }
    
    try {
        uvgvpcc_enc::API::initializeEncoder();
    } catch (const std::exception& e) {
        uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::FATAL, "LIBRARY","An exception was caught during the initialization of the encoder.\n");
        uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::FATAL, "LIBRARY", e.what() + std::string("\n"));
        cli::print_usage();
        return EXIT_FAILURE;
    }

    // Initialize the application input and output threads
    const std::shared_ptr<input_handler_args> in_args = std::make_shared<input_handler_args>(&appParameters, nullptr, RETVAL_RUNNING);
    std::thread inputTh(&inputReadThread, in_args);
    size_t frameRead = 0;
    std::shared_ptr<uvgvpcc_enc::Frame> currFrame = nullptr;
    available_input_slots.release();

    uvgvpcc_enc::API::v3c_unit_stream output;  // Each v3c chunk gets appended to the V3C unit stream as they are encoded
    std::thread file_writer_thread;
    file_writer_thread = std::thread(file_writer, &output, appParameters.outputPath);
    
    // Main loop of the application, feeding one frame to the encoder at each iteration
    for (;;) {
        filled_input_slots.acquire();
        currFrame = in_args->frame_in;
        in_args->frame_in = nullptr;
        available_input_slots.release();
        if (in_args->retval == RETVAL_EOF) {
            break;
        }
        if (in_args->retval == RETVAL_FAILURE) {
            return EXIT_FAILURE;
        }
        try {
            // Entry point of the uvgVPCCenc library
            uvgvpcc_enc::API::encodeFrame(currFrame, &output);
        } catch (const std::runtime_error& e) {
            // Only one try and catch block. All exceptions thrown by the library are catched here.
            uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::FATAL, "APPLICATION",
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

    file_writer_thread.join();

    uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::INFO, "APPLICATION", "Encoded " + std::to_string(frameRead) + " frames.\n");

    inputTh.join();
    uvgvpcc_enc::API::stopEncoder();

    return EXIT_SUCCESS;
}