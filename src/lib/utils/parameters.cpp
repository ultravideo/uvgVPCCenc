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

#include "utils/parameters.hpp"

#include <numeric>
#include <cassert>
#include <cstddef>
#include <cstdlib>
#include <iterator>
#include <algorithm>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include <regex>
#include <string>
#include <vector>
#include <algorithm>
#include <limits>

#include "uvgvpcc/log.hpp"



namespace uvgvpcc_enc {

std::unordered_map<std::string, ParameterInfo> parameterMap; 



inline int toInt(const std::string& paramValue, const std::string& paramName) {
    try {

        size_t pos = 0;
        int value = std::stoi(paramValue,&pos);
        // If pos is not at the end of the string, it means there were non-numeric characters
        if(pos != paramValue.length()) throw std::invalid_argument("");
        return value;
    } catch (const std::exception& e) {
        throw std::runtime_error("During the parsing of the uvgVPCC library command, an error occured: " + std::string(e.what()) +
                                 "\nThe value assign to '" + paramName + "' is: '" + paramValue +
                                 "'\nThis value was not converted into an int.");
    }
}

inline size_t toUInt(const std::string& paramValue, const std::string& paramName) {
    try {
        if (paramValue[0] == '-') {
            throw std::runtime_error("");
        }
        size_t pos = 0;
        size_t value = static_cast<size_t>(std::stoi(paramValue,&pos));
        // If pos is not at the end of the string, it means there were non-numeric characters
        if(pos != paramValue.length()) throw std::invalid_argument("");
        // to do : check the overflow during int and size_t conversion
        return value;
    } catch (const std::exception& e) {
        throw std::runtime_error("During the parsing of the uvgVPCC library command, an error occured: " + std::string(e.what()) +
                                 "\nThe value assign to '" + paramName + "' is: '" + paramValue +
                                 "'\nThis value was not converted into an unsigned int (std::size_t).");
    }
}

inline float toFloat(const std::string& paramValue, const std::string& paramName) {
    try {
        size_t pos = 0;
        float value = std::stof(paramValue,&pos);
        // If pos is not at the end of the string, it means there were non-numeric characters
        if(pos != paramValue.length()) throw std::invalid_argument("");
        return value;
    } catch (const std::exception& e) {
        throw std::runtime_error("During the parsing of the uvgVPCC library command, an error occured: " + std::string(e.what()) +
                                 "\nThe value assign to '" + paramName + "' is: '" + paramValue +
                                 "'\nThis value was not converted into a float.");
    }
}

inline double toDouble(const std::string& paramValue, const std::string& paramName) {
    try {
        size_t pos = 0;
        double value = std::stod(paramValue,&pos);
        // If pos is not at the end of the string, it means there were non-numeric characters
        if(pos != paramValue.length()) throw std::invalid_argument("");
        // to do : check the overflow during int and size_t conversion
        return value;
    } catch (const std::exception& e) {
        throw std::runtime_error("During the parsing of the uvgVPCC library command, an error occured: " + std::string(e.what()) +
                                 "\nThe value assign to '" + paramName + "' is: '" + paramValue +
                                 "'\nThis value was not converted into a double.");
    }
}

inline bool toBool(const std::string& paramValue, const std::string& paramName) {
    if (paramValue == "true" || paramValue == "True" || paramValue == "1") {
        return true;
    }
    if (paramValue == "false" || paramValue == "False" || paramValue == "0") {
        return false;
    }
    throw std::runtime_error("During the parsing of the uvgVPCC library command, an error occured.\nThe value assign to '" + paramName +
                             "' is: '" + paramValue +
                             "'\nThis value was not converted into a boolean. Only those values are accepted: [true,false,1,0]");
}

void initializeParameterMap(Parameters& param) {

    parameterMap = {
        // ___ General parameters __ //
        {"geoBitDepthInput", {UINT, "", &param.geoBitDepthInput}},
        {"presetName", {STRING, "fast,slow", &param.presetName}},
        {"intermediateFilesDir", {STRING, "", &param.intermediateFilesDir}},
        {"sizeGOF", {UINT, "8,16", &param.sizeGOF}}, // to do merge both gof size param ?
        {"nbThreadPCPart", {UINT, "", &param.nbThreadPCPart}},
        {"doubleLayer", {BOOL, "", &param.doubleLayer}},
        {"logLevel", {STRING, std::accumulate(std::begin(LogLevelStr) + 1, std::end(LogLevelStr), LogLevelStr[0], 
                                         [](const std::string& a, const std::string& b) { return a + "," + b; }), &param.logLevel}},
        {"errorsAreFatal", {BOOL, "", &param.errorsAreFatal}},

        // ___ Debug parameters ___ //
        {"exportIntermediateMaps", {BOOL, "", &param.exportIntermediateMaps}},
        {"exportIntermediatePointClouds", {BOOL, "", &param.exportIntermediatePointClouds}},

        // ___ Activate or not some features ___ //
        {"lowDelayBitstream", {BOOL, "", &param.lowDelayBitstream}},

        // ___ Voxelization ___ //       (grid-based segmentation)
        {"geoBitDepthVoxelized", {UINT, "", &param.geoBitDepthVoxelized}},
        
        // ___ KdTree ___ //
        {"kdTreeMaxLeafSize", {UINT, "", &param.kdTreeMaxLeafSize}},

        // Normal computation //
        {"normalComputationKnnCount", {UINT, "", &param.normalComputationKnnCount}},
        {"normalComputationMaxDiagonalStep", {UINT, "", &param.normalComputationMaxDiagonalStep}},

        // Normal orientation //
        {"normalOrientationKnnCount", {UINT, "", &param.normalOrientationKnnCount}},

        // PPI segmentation //

        // ___ PPI smoothing  ___  //    (fast grid-based refine segmentation)
        {"geoBitDepthRefineSegmentation", {UINT, "", &param.geoBitDepthRefineSegmentation}},
        {"refineSegmentationMaxNNVoxelDistanceLUT", {UINT, "", &param.refineSegmentationMaxNNVoxelDistanceLUT}},
        {"refineSegmentationMaxNNTotalPointCount", {UINT, "", &param.refineSegmentationMaxNNTotalPointCount}},
        {"refineSegmentationLambda", {DOUBLE, "", &param.refineSegmentationLambda}},
        {"refineSegmentationIterationCount", {UINT, "", &param.refineSegmentationIterationCount}},

        // ___ Patch generation ___ //   (patch segmentation)
        {"maxAllowedDist2RawPointsDetection", {UINT, "", &param.maxAllowedDist2RawPointsDetection}},
        {"minPointCountPerCC", {UINT, "", &param.minPointCountPerCC}},
        {"maxPatchSize", {UINT, "", &param.maxPatchSize}},
        {"maxNNCountPatchSegmentation", {UINT, "", &param.maxNNCountPatchSegmentation}},
        {"patchSegmentationMaxPropagationDistance", {UINT, "", &param.patchSegmentationMaxPropagationDistance}},
        {"enablePatchSplitting", {BOOL, "", &param.enablePatchSplitting}},
        {"minLevel", {UINT, "", &param.minLevel}},
        {"log2QuantizerSizeX", {UINT, "", &param.log2QuantizerSizeX}},
        {"log2QuantizerSizeY", {UINT, "", &param.log2QuantizerSizeY}},
        {"quantizerSizeX", {UINT, "", &param.quantizerSizeX}},
        {"quantizerSizeY", {UINT, "", &param.quantizerSizeY}},
        {"surfaceThickness", {UINT, "", &param.surfaceThickness}},
        
        // ___ Patch packing ___ //
        {"mapWidth", {UINT, "", &param.mapWidth}},
        {"minimumMapHeight", {UINT, "", &param.minimumMapHeight}},
        {"spacePatchPacking", {UINT, "", &param.spacePatchPacking}},
        {"interPatchPacking", {BOOL, "", &param.interPatchPacking}},
        {"gpaTresholdIoU", {FLOAT, "", &param.gpaTresholdIoU}},

        // ___ Map generation ___ //
        {"mapGenerationFillEmptyBlock", {BOOL, "", &param.mapGenerationFillEmptyBlock}},
        {"mapGenerationBackgroundValueAttribute", {UINT, "", &param.mapGenerationBackgroundValueAttribute}},
        {"mapGenerationBackgroundValueGeometry", {UINT, "", &param.mapGenerationBackgroundValueGeometry}},
        {"cuSizeInPixels", {UINT, "", &param.cuSizeInPixels}},

        // ___ 2D encoding parameters ___ //
        {"basenameOccupancyFiles", {STRING, "", &param.basenameOccupancyFiles}},
        {"basenameGeometryFiles", {STRING, "", &param.basenameGeometryFiles}},
        {"basenameAttributeFiles", {STRING, "", &param.basenameAttributeFiles}},
        {"sizeGOP2DEncoding", {UINT, "8,16", &param.sizeGOP2DEncoding}},
        {"intraFramePeriod", {UINT, "", &param.intraFramePeriod}},

        // Occupancy map
        {"occupancyEncoderName", {STRING, "Kvazaar", &param.occupancyEncoderName}},
        {"occupancyEncodingIsLossless", {BOOL, "", &param.occupancyEncodingIsLossless}},
        {"occupancyEncodingMode", {STRING, "AI,RA", &param.occupancyEncodingMode}},
        {"occupancyEncodingFormat", {STRING, "YUV420", &param.occupancyEncodingFormat}},
        {"occupancyEncodingNbThread", {UINT, "", &param.occupancyEncodingNbThread}},
        {"occupancyMapResolution", {UINT, "1,2,4,8", &param.occupancyMapResolution}},
        {"occupancyEncodingPreset", {STRING, "ultrafast,superfast,veryfast,faster,fast,medium,slow,slower,veryslow", &param.occupancyEncodingPreset}},

        // Geometry map
        {"geometryEncoderName", {STRING, "Kvazaar", &param.geometryEncoderName}},
        {"geometryEncodingIsLossless", {BOOL, "", &param.geometryEncodingIsLossless}},
        {"geometryEncodingMode", {STRING, "AI,RA", &param.geometryEncodingMode}},
        {"geometryEncodingFormat", {STRING, "YUV420", &param.geometryEncodingFormat}},
        {"geometryEncodingNbThread", {UINT, "", &param.geometryEncodingNbThread}},
        {"geometryEncodingQp", {UINT, "", &param.geometryEncodingQp}},
        {"geometryEncodingPreset", {STRING, "ultrafast,superfast,veryfast,faster,fast,medium,slow,slower,veryslow", &param.geometryEncodingPreset}},        

        // Attribute map
        {"attributeEncoderName", {STRING, "Kvazaar", &param.attributeEncoderName}},
        {"attributeEncodingIsLossless", {BOOL, "", &param.attributeEncodingIsLossless}},
        {"attributeEncodingMode", {STRING, "AI,RA", &param.attributeEncodingMode}},
        {"attributeEncodingFormat", {STRING, "YUV420", &param.attributeEncodingFormat}},
        {"attributeEncodingNbThread", {UINT, "", &param.attributeEncodingNbThread}},
        {"attributeEncodingQp", {UINT, "", &param.attributeEncodingQp}},
        {"attributeEncodingPreset", {STRING, "ultrafast,superfast,veryfast,faster,fast,medium,slow,slower,veryslow", &param.attributeEncodingPreset}},         
    }; 
}       

int LevenshteinDistance(const std::string& a, const std::string& b) {
    const size_t m = a.size();
    const size_t n = b.size();
    std::vector<std::vector<int>> dp(m + 1, std::vector<int>(n + 1));
    for (size_t i = 0; i <= m; ++i) {
        dp[i][0] = i;
    }
    for (size_t j = 0; j <= n; ++j) {
        dp[0][j] = j;
    }
    for (size_t i = 1; i <= m; ++i) {
        for (size_t j = 1; j <= n; ++j) {
            int cost = (a[i - 1] == b[j - 1]) ? 0 : 1;
            dp[i][j] = std::min({
                dp[i - 1][j] + 1, // Deletion
                dp[i][j - 1] + 1, // Insertion
                dp[i - 1][j - 1] + cost // Substitution
            });
        }
    }
    return dp[m][n];
}


std::string SuggestClosestString(const std::string& inputStr) {
    int minDistance = std::numeric_limits<int>::max();
    std::string closestString;
    for (const auto& option : parameterMap) {
        int distance = LevenshteinDistance(inputStr, option.first);
        if (distance < minDistance) {
            minDistance = distance;
            closestString = option.first;
        }
    }
    return closestString;
}


void setParameterValue(const std::string& parameterName,const std::string& parameterValue, const bool& fromPreset) {
    uvgvpcc_enc::Logger::log(
            uvgvpcc_enc::LogLevel::DEBUG, "API","Set parameter value: " + parameterName + " -> " + parameterValue + "\n");
    if(parameterMap.find(parameterName) == parameterMap.end()) {
        throw std::invalid_argument(std::string(fromPreset ? "[PRESET] " : "") + "The parameter '" + parameterName + "' is not a valid parameter name. Did you mean '" + SuggestClosestString(parameterName) + "'? (c.f. parameterMap)");
    }
    if (parameterValue.empty()) {
        throw std::invalid_argument("It seems an empty value is assigned to the parameter " + parameterName + ".");
    }
    ParameterInfo& paramInfo = parameterMap.find(parameterName)->second;
    if(!paramInfo.possibleValues.empty()) {
        // Make a matching regex from the list of possible values
        std::regex possibleValueRegex("^(" + std::regex_replace(paramInfo.possibleValues, std::regex(","), "|") + ")$");

        // Check if the matched value is valid
        if (!std::regex_match(parameterValue, possibleValueRegex)) {
            throw std::invalid_argument("Invalid value for parameter '"+ parameterName +    "': '" + parameterValue + "'. Accepted values are: [" + paramInfo.possibleValues + "]");
        } 
    }



    // Assign the parameter value to the correct parameter variable. The 'paramInfo.parameterPtr' is a pointer to one member of p_, the only uvgvpcc_enc::Parameters instance of uvgVPCCenc.
    switch (paramInfo.type) {
        case INT: *static_cast<int*>(paramInfo.parameterPtr) = toInt(parameterValue, parameterName); break;
        case BOOL: *static_cast<bool*>(paramInfo.parameterPtr) = toBool(parameterValue, parameterName); break;
        case UINT: *static_cast<std::size_t*>(paramInfo.parameterPtr) = toUInt(parameterValue, parameterName); break;
        case FLOAT: *static_cast<float*>(paramInfo.parameterPtr) = toFloat(parameterValue, parameterName); break;
        case DOUBLE: *static_cast<double*>(paramInfo.parameterPtr) = toDouble(parameterValue, parameterName); break;
        case STRING: *static_cast<std::string*>(paramInfo.parameterPtr) = parameterValue; break;
        default:assert(false);
    }

    if(fromPreset) {
        paramInfo.inPreset = true;
    } else if (paramInfo.inPreset) {
        uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::INFO, "API","The value assigned to parameter '" + parameterName +  "' overwrite the preset value.\n");
    }

}

}