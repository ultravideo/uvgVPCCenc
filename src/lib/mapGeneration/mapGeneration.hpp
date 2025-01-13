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

/// \file Entry point for the map generation process. Use the 2D location of the patch obtained during patch packing to create the occupancy, geometry and attribute 2D maps.

#include "uvgvpcc/uvgvpcc.hpp"

using namespace uvgvpcc_enc;


struct Filter {
    std::vector<float> data_;
    double shift_;
};
struct Filter444to420 {
    Filter horizontal_;
    Filter vertical_;
};

class MapGenerationBaseLine {
   public:
    static void initializeStaticParameters();
    static void initGOFMapGeneration(std::shared_ptr<uvgvpcc_enc::GOF>& gof);
    static void generateFrameMaps(std::shared_ptr<uvgvpcc_enc::Frame>& frame);
    static void writeFrameMapsYUV(std::shared_ptr<uvgvpcc_enc::Frame>& frame);

   private:
    
    static Filter444to420 g_filter444to420_unique_;

    static void mapsGeneration(uvgvpcc_enc::Frame& frame, const size_t& gofMapsHeight);
    static void writePatch(const uvgvpcc_enc::Patch& patch, const size_t& imageSize, uvgvpcc_enc::Frame& frame);
    static void writePatchAxisSwap(const uvgvpcc_enc::Patch& patch, const size_t& imageSize, uvgvpcc_enc::Frame& frame);

    static void fillBackgroundImages(uvgvpcc_enc::Frame& frame, const size_t& gofMapsHeight);

    static void fillBackgroundEmptyBlock(uvgvpcc_enc::Frame& frame, const size_t blockSize, const size_t imageSize,
                                         const size_t uBlk, const size_t vBlk, const size_t uom, const size_t vom);
    static void fillBackgroundNonEmptyBlock(uvgvpcc_enc::Frame& frame, const size_t blockSize, const size_t imageSize,
                                            const size_t uom, const size_t vom, const size_t pixelBlockCount,
                                            size_t missingPixelCount, std::vector<size_t>& iterations);
    static void updateSums(uvgvpcc_enc::Frame& frame, const size_t blockLeft, const size_t blockTop, const size_t iBlk,
                           const size_t jBlk, const size_t imageSize, std::vector<size_t>& iterations,
                           const size_t blockSize, std::vector<size_t>& sumGeo, std::vector<size_t>& sumR,
                           std::vector<size_t>& sumG, std::vector<size_t>& sumB, std::vector<size_t>& count);
    static void allocateMaps(uvgvpcc_enc::Frame& frame, const size_t& gofMapsHeight);
    static void RGB444toYUV420(std::vector<uint8_t>& img, const size_t& width, const size_t& height);

    static void downsampling(const std::vector<float>& chroma_in, std::vector<float>& chroma_out, const size_t widthIn,
                             const size_t heightIn);
};
