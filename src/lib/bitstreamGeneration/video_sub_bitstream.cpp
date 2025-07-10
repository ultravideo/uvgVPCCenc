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

#include "video_sub_bitstream.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <ios>
#include <new>
#include <string>
#include <vector>

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-owning-memory)

namespace {

size_t getEndOfNaluPosition(const std::vector<uint8_t> &data, size_t startIndex) {
    const size_t size = data.size();
    if (size < startIndex + 4) {
        return size;
    }
    for (size_t i = startIndex; i < size - 4; i++) {
        if ((data[i + 0] == 0x00) && (data[i + 1] == 0x00) && ((data[i + 2] == 0x01) || ((data[i + 2] == 0x00) && (data[i + 3] == 0x01)))) {
            return i;
        }
    }
    return size;
}
}  // anonymous namespace

bool read(const std::string &filename, std::vector<uint8_t> &data) {
    std::ifstream file(filename, std::ios::binary | std::ios::ate);
    if (!file.good()) {
        return false;
    }
    const uint64_t fileSize = file.tellg();
    data.resize(static_cast<size_t>(fileSize));
    file.clear();
    file.seekg(0);
    file.read(reinterpret_cast<char *>(data.data()), static_cast<std::streamsize>(data.size()));
    file.close();
    return true;
}

void byteStreamToSampleStream(std::vector<uint8_t> &input_data, size_t precision, std::vector<nal_info> &nals,
                              bool emulationPreventionBytes) {
    size_t startIndex = 0;
    size_t endIndex = 0;
    std::vector<uint8_t> data;
    do {
        const size_t sizeStartCode = input_data[startIndex + 2] == 0x00 ? 4 : 3;
        endIndex = getEndOfNaluPosition(input_data, startIndex + sizeStartCode);
        const size_t headerIndex = data.size();
        for (size_t i = 0; i < precision; i++) {
            data.push_back(0);
        }  // reserve nalu size
        if (emulationPreventionBytes) {
            for (size_t i = startIndex + sizeStartCode, zeroCount = 0; i < endIndex; i++) {
                if ((zeroCount == 3) && (input_data[i] <= 3)) {
                    zeroCount = 0;
                } else {
                    zeroCount = (input_data[i] == 0) ? zeroCount + 1 : 0;
                    data.push_back(input_data[i]);
                }
            }
        } else {
            for (size_t i = startIndex + sizeStartCode; i < endIndex; i++) {
                data.push_back(input_data[i]);
            }
        }
        const size_t naluSize = data.size() - (headerIndex + precision);
        for (size_t i = 0; i < precision; i++) {
            data[headerIndex + i] = (naluSize >> (8 * (precision - (i + 1)))) & 0xffU;
        }
        const nal_info current = {headerIndex + precision, naluSize};
        nals.push_back(current);
        startIndex = endIndex;
    } while (endIndex < input_data.size());
    input_data.swap(data);
}

namespace {

size_t combine_bytes(const uint8_t *bytes, size_t len) {
    uint8_t *buf = new uint8_t[len];
    memcpy(buf, bytes, len);
    size_t result = 0;
    for (size_t i = 0; i < len; ++i) {
        result |= static_cast<size_t>(buf[i]) << (8 * (len - 1 - i));
    }
    delete[] buf;
    return result;
}
}  // anonymous namespace

void find_nals(std::vector<uint8_t> &input_data, std::vector<nal_info> &nals) {
    const uint8_t *buf = input_data.data();
    size_t ptr = 0;
    while (ptr < input_data.size()) {
        const size_t nal_size = combine_bytes(&buf[ptr], 4);

        // const uint8_t nal_type = input_data[ptr + 4] >> 1;
        // std::cout << "-- NAL size: " << nal_size << ", type: " << (uint32_t)nal_type << std::endl;

        nal_info current;
        current.location = ptr + 4;
        current.size = nal_size;
        // std::cout << "uvg: location " << current.location << ", size " << current.size << std::endl;
        nals.push_back(current);
        ptr += 4 + nal_size;
    }
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-owning-memory)
