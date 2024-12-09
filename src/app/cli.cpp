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

#include "cli.hpp"

// NOLINTBEGIN(misc-include-cleaner)
#include <getopt.h>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <regex>
#include <span>
#include <stdexcept>
#include <string>
#include "uvgvpcc/log.hpp"
#include "uvgvpcc/version.hpp"

namespace cli {

namespace {

// NOLINTNEXTLINE(cert-err58-cpp)
const std::string short_options = "i:g:l:n:o:s:t:b:";
const std::array<struct option, 10> long_options{{{"input", required_argument, nullptr, 'i'},
                                                  {"output", required_argument, nullptr, 'o'},
                                                  {"frames", required_argument, nullptr, 'n'},
                                                  {"start-frame", required_argument, nullptr, 's'},
                                                  {"geo-precision", required_argument, nullptr, 'g'},
                                                  {"threads", required_argument, nullptr, 't'},
                                                  {"uvgvpcc", required_argument, nullptr, 0},
                                                  {"loop-input", required_argument, nullptr, 'l'},
                                                  {"help", no_argument, nullptr, 0},
                                                  {"version", no_argument, nullptr, 0}}};

/**
 * \brief Try to detect voxel size from file name automatically
 *
 * \param file_name    file name to get voxel size from
 * \return voxel_size on success, 0 on fail
 */
size_t select_voxel_size_auto(std::string& file_name) {
    const std::regex pattern("vox([0-9]+)");
    std::smatch match;
    size_t number = 0;
    if (std::regex_search(file_name, match, pattern)) {
        // The first match captures the entire pattern, so we need to access the second capture group (index 1)
        const std::string number_str = match[1].str();
        number = static_cast<size_t>(std::stoi(number_str));
    }
    return number;
}

/**
 * \brief Try to detect frame count from file name automatically
 *
 * \param file_name    file name to get frame count from
 * \return frame count on success, 0 on fail
 */
size_t select_frame_count_auto(std::string& file_name) {
    const std::regex pattern("([0-9]+)_%");
    std::smatch match;
    int number = 0;
    if (std::regex_search(file_name, match, pattern)) {
        // The first match captures the entire pattern, so we need to access the second capture group (index 1)
        const std::string number_str = match[1].str();
        number = std::stoi(number_str);
    }
    return number;
}

/**
 * \brief Try to detect start frame from file name automatically
 *
 * \param file_name    file name to get frame count from
 * \return start frame on success, 0 on fail
 */
size_t select_start_frame_auto(std::string& file_name) {
    const std::regex pattern("([0-9]+)_[0-9]+_%");
    std::smatch match;
    int number = 0;
    if (std::regex_search(file_name, match, pattern)) {
        // The first match captures the entire pattern, so we need to access the second capture group (index 1)
        const std::string number_str = match[1].str();
        number = std::stoi(number_str);
    }
    return number;
}
}  // anonymous namespace

bool opts_parse(cli::opts_t& opts, const int& argc, const std::span<const char* const>& args) {

    // Parse command line options
    for (optind = 0;;) {
        int long_options_index = -1;

        const int c =
            // NOLINTNEXTLINE(concurrency-mt-unsafe,cppcoreguidelines-pro-type-const-cast)
            getopt_long(argc, const_cast<char* const*>(args.data()), short_options.c_str(), long_options.data(), &long_options_index);
        if (c == -1) {
            break;
        }

        if (long_options_index < 0) {
            for (int i = 0; static_cast<bool>(long_options[i].name); i++) {
                if (long_options[i].val == c) {
                    long_options_index = i;
                    break;
                }
            }
        }

        const std::string name = long_options[long_options_index].name;
        if (name == "input") {
            if (!opts.inputPath.empty()) {
                throw std::runtime_error("Input error: More than one input file given.");
            }
            opts.inputPath = optarg;
        } else if (name == "output") {
            if (!opts.outputPath.empty()) {
                throw std::runtime_error("Input error: More than one output file given.");
            }
            opts.outputPath = optarg;
        } else if (name == "geo-precision") {
            opts.inputGeoPrecision = std::stoi(optarg);
            if (opts.inputGeoPrecision == 0) {
                throw std::runtime_error("Input error: Geometry precision is set to zero");
            }
        } else if (name == "frames") {
            opts.frames = std::stoi(optarg);
            if (opts.frames == 0) {
                throw std::runtime_error("Input error: Frame count is zero");
            }
        } else if (name == "start-frame") {
            opts.startFrame = std::stoi(optarg);
        } else if (name == "threads") {
            if (std::stoi(optarg) < 0) {
                throw std::runtime_error("Input error: Given thread count should positive (threads=0 to leave it in auto).");
            }
            opts.threads = std::stoi(optarg);
        } else if (name == "uvgvpcc") {
            opts.uvgvpccParametersString = optarg;
        } else if (name == "loop-input") {
            opts.loop_input = std::stoi(optarg);
        } else if (name == "version") {
            cli::print_version();
            return true;
        } else if (name == "help") {
            cli::print_help();
            return true;
        }
    }
    // Check for extra arguments.
    if (args.size() - optind > 0) {
        throw std::runtime_error("Input error: Extra argument found: " + std::string(args[optind]) + ".");
    }

    // Check that the required files were defined
    if (opts.inputPath.empty() || opts.outputPath.empty()) {
        throw std::runtime_error("Input error: Input or output path is empty\n");
    }

    if (opts.inputGeoPrecision == 0) {
        
        opts.inputGeoPrecision = select_voxel_size_auto(opts.inputPath);
        if (opts.inputGeoPrecision == 0) {
            throw std::runtime_error("Input geometry precision is not manually set by the application and it is not detected from the file name. The geometry precision (the library parameter 'geoBitDepthInput', a.k.a voxel size) is a parameter needed by the encoder. It should be set in the application using function 'uvgvpcc_enc::API::setParameter()'.\n");
        }
        // else 
        uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::INFO, "APPLICATION",
            "The input geometry precision is not manually set by the application but it is detected from file name: " + std::to_string(opts.inputGeoPrecision) + ".\n");
    
    }

    if (opts.frames == 0) {
        opts.frames = select_frame_count_auto(opts.inputPath);
        uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::INFO, "APPLICATION",
                                 "Detected frame count from file name: " + std::to_string(opts.frames) + ".\n");
        if (opts.frames == 0) {
            throw std::runtime_error("Input error: Frame count is zero");
        }
    }

    if (opts.startFrame == std::numeric_limits<size_t>::max()) {
        opts.startFrame = select_start_frame_auto(opts.inputPath);
        uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::INFO, "APPLICATION",
                                 "Detected start frame from file name: " + std::to_string(opts.startFrame) + ".\n");
        if (opts.startFrame == std::numeric_limits<size_t>::max()) {
            throw std::runtime_error("Input error: Frame count is zero");
        }
    }

    return false;
}

void print_usage(void) {
    std::cout << "usage: uvpVPCCenc -i <input> -n <frame number> -o <output>\n"
              << "       --help for more information" << std::endl;
}

void print_version(void) { std::cout << "uvgVPCC " << uvgvpcc_enc::get_version() << std::endl; }

void print_help(void) {
    std::cout << "Usage: uvpVPCCenc -i <input> -n <frame number> -o <output>\n\n";
                 /* Word wrap to this width to stay under 80 characters (including ") *************/
    std::cout << "Options:\n";
    std::cout << "  -i, --input <file>           Input filename\n";
    std::cout << "  -o, --output <file>          Output filename\n";
    std::cout << "  -n, --frames <number>        Number of frames to encode\n";
    std::cout << "  -s, --start-frame <number>   Frame number to start the encoding\n";
    std::cout << "  -g, --geo-precision <number> Geometry precision for encoding\n";
    std::cout << "  -t, --threads <number>       Maximum number of threads to be used\n";
    std::cout << "      --uvgvpcc <params>       Encoder configuration parameters\n";
    std::cout << "  -l, --loop-input <number>    Number of input loop\n";
    std::cout << "      --help                   Show this help message\n";
    std::cout << "      --version                Show version information\n";
    
    std::cout << "\nDescription:\n";
    std::cout << "  This tool encodes point cloud video frames using the uvgVPCCenc codec\n";
    std::cout << "  with specified parameters.\n";
    std::cout << "  The input and output file paths must be specified, using \%0Xd\n";
}
}  // namespace cli
// NOLINTEND(misc-include-cleaner)
