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


#include <cstdarg>
#include <cstdio>

#include "catchLibLog.hpp"
#include "uvgvpcc/log.hpp"

#if defined(__GNUC__) && !defined(__clang__)
// For GCC
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#elif defined(__clang__)
// For Clang
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
#endif

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-array-to-pointer-decay,hicpp-no-array-decay,hicpp-named-parameter,readability-named-parameter,cppcoreguidelines-pro-type-vararg,hicpp-vararg,cert-dcl50-cpp)

int kvazaar_lib_log_callback(FILE*, const char* log_content, ...) {
    va_list args;
    va_start(args, log_content);
    uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::DEBUG, "KVAZAAR", uvgvpcc_enc::Logger::vprintfStrToStdStr(log_content, args));
    va_end(args);
    return 0;
}

/*int uvg266_lib_log_callback(FILE*, const char* log_content, ...) {
    va_list args;
    va_start(args, log_content);
    uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::DEBUG, "UVG266", uvgvpcc_enc::Logger::vprintfStrToStdStr(log_content, args));
    va_end(args);
    return 0;
}*/

// NOLINTEND(cppcoreguidelines-pro-bounds-array-to-pointer-decay,hicpp-no-array-decay,hicpp-named-parameter,readability-named-parameter,cppcoreguidelines-pro-type-vararg,hicpp-vararg,cert-dcl50-cpp)


#if defined(__GNUC__) && !defined(__clang__)
// For GCC
#pragma GCC diagnostic pop
#elif defined(__clang__)
// For Clang
#pragma clang diagnostic pop
#endif
