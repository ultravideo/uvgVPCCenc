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

/// \file Library common memory related operations.

#pragma once

#include <cassert>
#include "../patchGeneration/robin_hood.h"
#include "uvgvpcc/uvgvpcc.hpp"
#include "constants.hpp"

namespace uvgvpcc_enc {

struct Patch;

class CommonMemory {
    
    public:
    static CommonMemory& get();  // lf: leaky singleton
    
    robin_hood::unordered_map<size_t,std::unique_ptr<std::array<std::vector<Patch>, MAX_GOF_SIZE>>> mapFramePatches;
    
    robin_hood::unordered_map<size_t,std::unique_ptr<std::array<std::vector<uint8_t>, MAX_GOF_SIZE>>> mapFrameOccupancyMaps;
    robin_hood::unordered_map<size_t,std::unique_ptr<std::array<std::vector<uint8_t>, MAX_GOF_SIZE>>> mapFrameOccupancyMapsDS;
    robin_hood::unordered_map<size_t,std::unique_ptr<std::array<std::vector<uint8_t>, MAX_GOF_SIZE>>> mapFrameGeometryMapsL1;
    robin_hood::unordered_map<size_t,std::unique_ptr<std::array<std::vector<uint8_t>, MAX_GOF_SIZE>>> mapFrameGeometryMapsL2;
    robin_hood::unordered_map<size_t,std::unique_ptr<std::array<std::vector<uint8_t>, MAX_GOF_SIZE>>> mapFrameAttributeMapsL1;
    robin_hood::unordered_map<size_t,std::unique_ptr<std::array<std::vector<uint8_t>, MAX_GOF_SIZE>>> mapFrameAttributeMapsL2;


    std::array<std::vector<Patch>,    MAX_GOF_SIZE>* getOrCreateFramePatches        (size_t gofId) { return getOrCreate(mapFramePatches,          gofId); }
    std::array<std::vector<uint8_t>,  MAX_GOF_SIZE>* getOrCreateFrameOccupancyMaps  (size_t gofId) { return getOrCreate(mapFrameOccupancyMaps,     gofId); }
    std::array<std::vector<uint8_t>,  MAX_GOF_SIZE>* getOrCreateFrameOccupancyMapsDS(size_t gofId) { return getOrCreate(mapFrameOccupancyMapsDS,   gofId); }
    std::array<std::vector<uint8_t>,  MAX_GOF_SIZE>* getOrCreateFrameGeometryMapsL1 (size_t gofId) { return getOrCreate(mapFrameGeometryMapsL1,    gofId); }
    std::array<std::vector<uint8_t>,  MAX_GOF_SIZE>* getOrCreateFrameGeometryMapsL2 (size_t gofId) { return getOrCreate(mapFrameGeometryMapsL2,    gofId); }
    std::array<std::vector<uint8_t>,  MAX_GOF_SIZE>* getOrCreateFrameAttributeMapsL1(size_t gofId) { return getOrCreate(mapFrameAttributeMapsL1,   gofId); }
    std::array<std::vector<uint8_t>,  MAX_GOF_SIZE>* getOrCreateFrameAttributeMapsL2(size_t gofId) { return getOrCreate(mapFrameAttributeMapsL2,   gofId); }

    void clearGofMaps(const size_t& gofId);

    private:
    std::mutex mapMutex;

    template<typename Map>
    typename Map::mapped_type::element_type* getOrCreate(Map& map, size_t gofId) {
        std::lock_guard<std::mutex> lock(mapMutex);
        auto& ptr = map[gofId];
        if (!ptr) {
            ptr = std::make_unique<typename Map::mapped_type::element_type>();
        }
        return ptr.get();
    }

};




}  // namespace uvgvpcc_enc
