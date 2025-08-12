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

#pragma once

/**
 * \file
 * Command line interface
 */
#include <limits>
#include <string>
#include <span>

namespace cli {

struct opts_t {
    /** \brief Input filename */
    std::string inputPath{};
    /** \brief Output filename */
    std::string outputPath{};
    /** \brief Number of frames to encode */
    size_t nbFrames{};
    /** \brief Input geometry precision */
    size_t inputGeoPrecision = 0; 
    /** \brief Frame number to start the encoding */
    size_t startFrame = std::numeric_limits<size_t>::max();
    /** \brief Maximum number of threads to be used */
    size_t threads{};
    /** \brief Encoder configuration */
    std::string uvgvpccParametersString{};
    /** \brief Print help */
    bool help = false;
    /** \brief Print version */
    bool version = false;
    /** \brief Whether to loop input */
    size_t nbLoops = 1;
    /** \brief If dummyRun is true, config is verified but no encoding is done */
    bool dummyRun = false;    
};

bool opts_parse(cli::opts_t& opts, const int& argc, const std::span<const char* const>& args);

void print_usage(void);
void print_version(void);
void print_help(void);

}  // namespace cli