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

/// \file Entry point for the patch packing process. Assign a 2D location to each patch.


#pragma once

#include <span>

#include "uvgvpcc/uvgvpcc.hpp"

enum PCCaxisSwap {
    PATCH_ORIENTATION_DEFAULT = 0,  // 0: default
    PATCH_ORIENTATION_SWAP = 1,     // 1: swap
    PATCH_ORIENTATION_ROT90 = 2,    // 2: rotation 90
    PATCH_ORIENTATION_ROT180 = 3,   // 3: rotation 180
    PATCH_ORIENTATION_ROT270 = 4,   // 4: rotation 270
    PATCH_ORIENTATION_MIRROR = 5,   // 5: mirror
    PATCH_ORIENTATION_MROT90 = 6,   // 6: mirror + rotation 90
    PATCH_ORIENTATION_MROT180 = 7,  // 7: mirror + rotation 180
    PATCH_ORIENTATION_MROT270 = 8   // 8: similar to SWAP, not used switched SWAP with ROT90 positions
};

const std::vector<int> g_orientationHorizontal = {
    PATCH_ORIENTATION_SWAP,     // Horizontal orientation swap
    PATCH_ORIENTATION_DEFAULT,  // Horizontal orientation default
};
const std::vector<int> g_orientationVertical = {
    PATCH_ORIENTATION_DEFAULT,  // Vertical orientation default
    PATCH_ORIENTATION_SWAP,     // Vertical orientation swap
};

class PatchPacking {
   public:
    PatchPacking();
    static void frameIntraPatchPacking(const std::shared_ptr<uvgvpcc_enc::Frame>& frame, std::span<uvgvpcc_enc::Patch>* patchListSpan);
    static void frameInterPatchPacking(const std::vector<uvgvpcc_enc::Patch>& unionPatches, const std::shared_ptr<uvgvpcc_enc::Frame>& frame,
                                       std::span<uvgvpcc_enc::Patch>* matchedPatchList);

    static void allocateDefaultOccupancyMap(const std::shared_ptr<uvgvpcc_enc::Frame>& frame, const size_t mapHeight);
    static void gofPatchPacking(const std::shared_ptr<uvgvpcc_enc::GOF>& gof);

   private:

    static bool findPatchLocation(const size_t& mapHeight, size_t& maxPatchHeight,
                                  uvgvpcc_enc::Patch& patch, const std::vector<uint8_t>& frameOccupancyMap);
    static bool checkLocation(const size_t& mapHeight, const size_t& posOMu, const size_t& posOMv,
                              const size_t& patchWidth, const size_t& patchHeight, size_t& maxPatchHeight,
                              uvgvpcc_enc::Patch& patch, const std::vector<uint8_t>& frameOccupancyMap);

    static bool checkFitPatch(const size_t& patchPosX, const size_t& patchPosY, const size_t& patchWidth,
                              const size_t& patchHeight, const size_t& mapHeight, const std::vector<uint8_t>& frameOccupancyMap);

    static void patchMatchingBetweenTwoFrames(const std::shared_ptr<uvgvpcc_enc::Frame>& currentFrame,
                                              const std::shared_ptr<uvgvpcc_enc::Frame>& previousFrame);
    static float computeIoU(const uvgvpcc_enc::Patch& currentPatch, const uvgvpcc_enc::Patch& previousPatch);
};
