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

#include "preset.hpp"

#include <string>
#include <utility>

#include "utils/parameters.hpp"
#include "uvgvpcc/log.hpp"

namespace uvgvpcc_enc {

namespace {

using Preset = std::vector<std::pair<const char*, const char*>>;

Preset preset_vox9_fast = {
    // ___ General parameters __ //
    {"sizeGOF","16"},

    // ___ Voxelization ___ //       (grid-based segmentation)
    {"geoBitDepthVoxelized","8"}, 
    
    // ___ Normal computation ___ //
    {"normalComputationKnnCount","6"}, 
    {"normalComputationMaxDiagonalStep","4"},

    // ___ PPI smoothing  ___  //    (fast grid-based refine segmentation)
    {"geoBitDepthRefineSegmentation","7"}, 
    {"refineSegmentationMaxNNVoxelDistanceLUT","2"}, 
    {"refineSegmentationMaxNNTotalPointCount","32"}, 
    {"refineSegmentationLambda","3.5"},  
    {"refineSegmentationIterationCount","3"},

    // ___ Patch generation ___ //   (patch segmentation)
    {"minPointCountPerCC","16"},   // TODO-PRESET: subjective quality and performance only 
    {"maxPatchSize","608"},    // TODO-PRESET: subjective quality and performance only

    // ___ Patch packing ___ //
    {"mapWidth","608"},
    {"minimumMapHeight","608"},

    // ___ 2D encoding parameters ___ //
    {"sizeGOP2DEncoding","16"},
    {"occupancyEncodingPreset","ultrafast"}, // (Always lossless) Negligeable impact on both performance and bitrate (Kvazaar)
    {"geometryEncodingPreset","fast"}, // TODO-PRESET: fast or medium ?
    {"attributeEncodingPreset","ultrafast"}  // TODO-PRESET: fixed ?
};

Preset preset_vox9_slow = {
    // ___ General parameters __ //
    {"sizeGOF","8"},

    // ___ Voxelization ___ //       (grid-based segmentation)
    {"geoBitDepthVoxelized","9"},   // TODO-PRESET: fixed
    
    // ___ Normal computation ___ //
    {"normalComputationKnnCount","12"}, // TODO-PRESET: fixed    
    {"normalComputationMaxDiagonalStep","8"}, // TODO-PRESET: relaunch on all sequences all frames    

    // ___ PPI smoothing  ___  //    (fast grid-based refine segmentation)
    {"geoBitDepthRefineSegmentation","8"},  // TODO-PRESET: fixed
    {"refineSegmentationMaxNNVoxelDistanceLUT","9"}, // TODO-PRESET: fixed
    {"refineSegmentationMaxNNTotalPointCount","256"}, // TODO-PRESET: fixed
    {"refineSegmentationLambda","3.0"}, // TODO-PRESET : should be tested with iterations
    {"refineSegmentationIterationCount","15"}, // TODO-PRESET : should be tested with lambda

    // ___ Patch generation ___ //   (patch segmentation)
    {"minPointCountPerCC","5"}, // TODO-PRESET: fixed
    {"maxPatchSize","608"},

    // ___ Patch packing ___ //
    {"mapWidth","608"},
    {"minimumMapHeight","608"},

    // ___ 2D encoding parameters ___ //
    {"sizeGOP2DEncoding","8"},
    {"occupancyEncodingPreset","veryslow"},  // (Always lossless) Negligeable impact on both performance and bitrate (Kvazaar)
    {"geometryEncodingPreset","veryslow"},  // TODO-PRESET: Really useful ?
    {"attributeEncodingPreset","veryslow"}  // TODO-PRESET: Really useful ?
};


Preset preset_vox10_fast = {
    // ___ General parameters __ //
    {"sizeGOF","16"},

    // ___ Voxelization ___ //       (grid-based segmentation)
    {"geoBitDepthVoxelized","9"}, // fixed
    
    // ___ Normal computation ___ //
    {"normalComputationKnnCount","6"},
    {"normalComputationMaxDiagonalStep","4"},

    // ___ PPI smoothing  ___  //    (fast grid-based refine segmentation)
    {"geoBitDepthRefineSegmentation","8"},// fixed
    {"refineSegmentationMaxNNVoxelDistanceLUT","2"},  
    {"refineSegmentationMaxNNTotalPointCount","32"},
    {"refineSegmentationLambda","3.5"},
    {"refineSegmentationIterationCount","3"},

    // ___ Patch generation ___ //   (patch segmentation)
    {"minPointCountPerCC","16"},
    {"maxPatchSize","1024"},

    // ___ Patch packing ___ //
    {"mapWidth","1024"},
    {"minimumMapHeight","1024"},

    // ___ 2D encoding parameters ___ //
    {"sizeGOP2DEncoding","16"},
    {"occupancyEncodingPreset","ultrafast"},
    {"geometryEncodingPreset","fast"},
    {"attributeEncodingPreset","ultrafast"}
};

Preset preset_vox10_slow = {
    // ___ General parameters __ //
    {"sizeGOF","16"},

    // ___ Voxelization ___ //       (grid-based segmentation)
    {"geoBitDepthVoxelized","10"}, // fixed
    
    // ___ Normal computation ___ //
    {"normalComputationKnnCount","12"},
    {"normalComputationMaxDiagonalStep","8"},

    // ___ PPI smoothing  ___  //    (fast grid-based refine segmentation)
    {"geoBitDepthRefineSegmentation","9"}, // fixed
    {"refineSegmentationMaxNNVoxelDistanceLUT","9"},  
    {"refineSegmentationMaxNNTotalPointCount","256"},
    {"refineSegmentationLambda","3.0"},
    {"refineSegmentationIterationCount","15"},

    // ___ Patch generation ___ //   (patch segmentation)
    {"minPointCountPerCC","5"},
    {"maxPatchSize","1024"},

    // ___ Patch packing ___ //
    {"mapWidth","1024"},
    {"minimumMapHeight","1024"},

    // ___ 2D encoding parameters ___ //
    {"sizeGOP2DEncoding","8"},
    {"occupancyEncodingPreset","veryslow"},
    {"geometryEncodingPreset","veryslow"},
    {"attributeEncodingPreset","veryslow"}
};

Preset preset_vox11_fast = {
    // ___ General parameters __ //
    {"sizeGOF","16"},

    // ___ Voxelization ___ //       (grid-based segmentation)
    {"geoBitDepthVoxelized","10"},
    
    // ___ Normal computation ___ //
    {"normalComputationKnnCount","6"},
    {"normalComputationMaxDiagonalStep","8"},

    // ___ PPI smoothing  ___  //    (fast grid-based refine segmentation)
    {"geoBitDepthRefineSegmentation","9"},
    {"refineSegmentationMaxNNVoxelDistanceLUT","4"},  
    {"refineSegmentationMaxNNTotalPointCount","128"},
    {"refineSegmentationLambda","3.0"},
    {"refineSegmentationIterationCount","4"},

    // ___ Patch generation ___ //   (patch segmentation)
    {"minPointCountPerCC","16"},
    {"maxPatchSize","2048"},

    // ___ Patch packing ___ //
    {"mapWidth","2048"},
    {"minimumMapHeight","2048"},

    // ___ 2D encoding parameters ___ //
    {"sizeGOP2DEncoding","16"},
    {"occupancyEncodingPreset","ultrafast"},
    {"geometryEncodingPreset","fast"},
    {"attributeEncodingPreset","ultrafast"}
};

Preset preset_vox11_slow = {
    // ___ General parameters __ //
    {"sizeGOF","16"},

    // ___ Voxelization ___ //       (grid-based segmentation)
    {"geoBitDepthVoxelized","10"}  ,
    
    // ___ Normal computation ___ //
    {"normalComputationKnnCount","6"},
    {"normalComputationMaxDiagonalStep","8"},

    // ___ PPI smoothing  ___  //    (fast grid-based refine segmentation)
    {"geoBitDepthRefineSegmentation","9"},
    {"refineSegmentationMaxNNVoxelDistanceLUT","4"},  
    {"refineSegmentationMaxNNTotalPointCount","128"},
    {"refineSegmentationLambda","3.0"},
    {"refineSegmentationIterationCount","4"},

    // ___ Patch generation ___ //   (patch segmentation)
    {"minPointCountPerCC","16"},
    {"maxPatchSize","2048"},

    // ___ Patch packing ___ //
    {"mapWidth","2048"},
    {"minimumMapHeight","2048"},

    // ___ 2D encoding parameters ___ //
    {"sizeGOP2DEncoding","8"},
    {"occupancyEncodingPreset","veryslow"},
    {"geometryEncodingPreset","veryslow"},
    {"attributeEncodingPreset","veryslow"}
};

void setPresetValues(const Preset& preset) {
    for(const auto& pair : preset) {
        uvgvpcc_enc::setParameterValue(pair.first,pair.second,true);
    }
}

// For voxel size 9, 10 and 11
void applyPresetCommon(Parameters& param) {

    Preset selectedPreset;

    switch (param.geoBitDepthInput) {
        case 9:
            if(param.presetName == "fast") selectedPreset = preset_vox9_fast;
            if(param.presetName == "slow") selectedPreset = preset_vox9_slow;
            break;
        case 10:
            if(param.presetName == "fast") selectedPreset = preset_vox10_fast;
            if(param.presetName == "slow") selectedPreset = preset_vox10_slow;
            break;
        case 11:
            if(param.presetName == "fast") selectedPreset = preset_vox11_fast;
            if(param.presetName == "slow") selectedPreset = preset_vox11_slow;
            break;                        
    }

    // if (param.geoBitDepthInput == 9 && param.presetName == "fast") {
    //     selectedPreset = preset_vox9_fast;
    // } else if (param.geoBitDepthInput == 9 && param.presetName == "slow") {
    //     selectedPreset = preset_vox9_slow;
    // } else if (param.geoBitDepthInput == 10 && param.presetName == "fast") {
    //     selectedPreset = preset_vox10_fast;
    // } else if (param.geoBitDepthInput == 10 && param.presetName == "slow") {
    //     selectedPreset = preset_vox10_slow;
    // } else if (param.geoBitDepthInput == 11 && param.presetName == "fast") {
    //     selectedPreset = preset_vox11_fast;
    // } else if (param.geoBitDepthInput == 11 && param.presetName == "slow") {
    //     selectedPreset = preset_vox11_slow;
    // } else {
    //     Logger::log(
    //         LogLevel::ERROR, "LIBRARY INTERFACE",
    //         "Error occurred while parsing library arguments. This preset name '" + param.presetName +
    //             "' is not recognized or does not exist for an input geometry bit depth of " + std::to_string(param.geoBitDepthInput) +
    //             ".\nNotice that only two presets are currently available: 'fast' and 'slow'.\nNotice that uvgVPCC has been tested for voxel "
    //             "9,10 and 11. Other input resolution sizes could be handled, but they are not supported yet.\n");
    //     throw std::runtime_error("uvgVPCC log of type ERROR");
    // }
    setPresetValues(selectedPreset);

}

}  // anonymous namespace



void applyPreset(Parameters& param) {

    if(param.geoBitDepthInput == 9 ||param.geoBitDepthInput == 10 ||param.geoBitDepthInput == 11) {
        applyPresetCommon(param);
        return;
    }
    
    if(param.geoBitDepthInput < 9) {
        if(param.presetName == "fast") {
            setPresetValues(preset_vox9_fast);
            // Apply voxelization
            setParameterValue("geoBitDepthVoxelized", std::to_string(param.geoBitDepthInput-1), true);
            setParameterValue("geoBitDepthRefineSegmentation", std::to_string(param.geoBitDepthInput-2), true);
        }
        if(param.presetName == "slow") {
            setPresetValues(preset_vox9_slow);
            // Do not apply voxelization
            setParameterValue("geoBitDepthVoxelized", std::to_string(param.geoBitDepthInput), true);
            setParameterValue("geoBitDepthRefineSegmentation", std::to_string(param.geoBitDepthInput-1), true);
        }
        Logger::log(
            LogLevel::WARNING, "LIBRARY INTERFACE",
            "uvgVPCCenc can support most of the point cloud voxel sizes (or input bit depths). However, it is tested only for voxel 9, 10 and 11. Strange things may happened. The presets are tuned for those voxel sizes only.\n The current voxel size is: " + std::to_string(param.geoBitDepthInput) +  ". The preset used is based on the preset 'vox9" + param.presetName + "'. Parameters link to the input bitdepth are changed accordingly.\n");    
    }

    if(param.geoBitDepthInput > 11) {
        if(param.presetName == "fast") {
            setPresetValues(preset_vox11_fast);
            // Adjust the voxelization
            setParameterValue("geoBitDepthVoxelized", std::to_string(param.geoBitDepthInput-1), true);
            setParameterValue("geoBitDepthRefineSegmentation", std::to_string(param.geoBitDepthInput-2), true);
        }
        if(param.presetName == "slow") {
            setPresetValues(preset_vox11_slow);
            // Adjust the non-voxelization
            setParameterValue("geoBitDepthVoxelized", std::to_string(param.geoBitDepthInput), true);
            setParameterValue("geoBitDepthRefineSegmentation", std::to_string(param.geoBitDepthInput-1), true);
        }

        // Adjust the size of the max size of the patch and maps dimension
        const std::string scaledSize = std::to_string((1 + param.geoBitDepthInput - 11) * 2048);

        setParameterValue("maxPatchSize", scaledSize, true);
        setParameterValue("mapWidth", scaledSize, true);
        setParameterValue("minimumMapHeight", scaledSize, true);

        Logger::log(
            LogLevel::WARNING, "LIBRARY INTERFACE",
            "uvgVPCCenc can support most of the point cloud voxel sizes (or input bit depths). However, it is tested only for voxel 9, 10 and 11. Strange things may happened. The presets are tuned for those voxel sizes only.\n The current voxel size is: " + std::to_string(param.geoBitDepthInput) +  ". The preset used is based on the preset 'vox11" + param.presetName + "'. Parameters link to the input bitdepth are changed accordingly.\n");    
    }
}

} // uvgVPCCenc namespace