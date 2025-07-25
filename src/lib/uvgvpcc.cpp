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

/// \file Main file of the uvgVPCCenc library that defines the main structures (GOF, frame, patch) and the API.

#include "uvgvpcc/uvgvpcc.hpp"

#include <cassert>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <ostream>
#include <algorithm>
#include <thread>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include <regex>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>


#include "uvgvpcc/threadqueue.hpp"
#include "patchPacking/patchPacking.hpp"
#include "mapEncoding/mapEncoding.hpp"
#include "mapGeneration/mapGeneration.hpp"
#include "patchGeneration/patchGeneration.hpp"
#include "bitstreamGeneration/bitstreamGeneration.hpp"
#include "utils/preset.hpp"
#include "uvgvpcc/log.hpp"
#include "utils/parameters.hpp"
#include "utils/fileExport.hpp"

namespace uvgvpcc_enc {

namespace {

Parameters param;
// Map storing all parameters intending to change the state of the encoder from outside of the library.
std::unordered_map<std::string,std::string> apiInputParameters;
bool errorInAPI = false;
bool initializationDone = false;

void createDirectory(const std::string &path) {
    std::filesystem::path dirPath(path);
    if (!std::filesystem::exists(dirPath)) {
        if (std::filesystem::create_directory(dirPath)) {
            uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::DEBUG>("UTILS","Directory created: " + path + ".\n");
        } else {
            throw std::runtime_error("Failed to create directory: " + path);
        }
    } else {
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::DEBUG>("UTILS","Directory already exists: " + path + ".\n");
    }
}  

struct ThreadHandler {
    size_t gofId;
    std::shared_ptr<GOF> currentGOF;
    std::shared_ptr<Job> currentGOFInterPackJob;
    std::shared_ptr<Job> currentGOFInitMapGenJob;
    std::shared_ptr<Job> currentGOF2DEncodingJob;
    std::shared_ptr<Job> currentGOFBitstreamGenJob;
    std::shared_ptr<Job> previousGOFBitstreamGenJob;
    std::shared_ptr<ThreadQueue> queue;
};

ThreadHandler g_threadHandler;

void initializeStaticParameters() {
    uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::TRACE>("API", "Initialize static parameters.\n");
    Job::setExecutionMethod(p_->timerLog);
    MapGenerationBaseLine::initializeStaticParameters();
    MapEncoding::initializeStaticParameters();
}

void initializeStaticFunctionPointers() {
    uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::TRACE>("API", "Initialize static function pointers.\n");
    MapEncoding::initializeEncoderPointers();
}


void verifyConfig() {
    uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::TRACE>("VERIFY CONFIG", "Verify the parameter configuration.\n");
    if (p_->timerLog && Logger::getLogLevel() < LogLevel::PROFILING) {
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::WARNING>("VERIFY CONFIG",
            "The parameter 'timerLog' has been set to 'True' but the current 'logLevel' (" + p_->logLevel + ") does not display profiling information. Consider switching to at least logLevel=PROFILING.\n");        
    }

    if ( !( (p_->occupancyEncoderName=="Kvazaar" && p_->geometryEncoderName=="Kvazaar" && p_->attributeEncoderName=="Kvazaar"))) {
        std::cerr << p_->occupancyEncoderName << " " << p_->geometryEncoderName << " " << p_->attributeEncoderName << std::endl;
        throw std::runtime_error("A single 2D encoder is currently supported : 'Kvazaar'. Here are the values used : occupancy encoder: '" + p_->occupancyEncoderName + "',  geometry encoder: '" + p_->geometryEncoderName + "', attribute encoder: '" + p_->attributeEncoderName + "'. Moreover, you have to use the same 2D encoder for all maps (occupancy, geometry and attribute). This is due to the V3C parameter 'CodecGroupIdc' that operate at GOF level. (Notice that a modification in vps.cpp could solve this issue).");
    }
    
    if (p_->sizeGOF > p_->maxConcurrentFrames) {
        throw std::runtime_error("The parameter 'maxConcurrentFrames' (" + std::to_string(p_->maxConcurrentFrames) +
                                 ") is lower than the parameter 'sizeGOF' (" +
                                 std::to_string(p_->sizeGOF) + "). It will lead to a dealock. This is not a valid configuration. Set 'maxConcurrentFrames' to a value greater or equal to 'sizeGOF'.");
    }

    if (p_->gpaTresholdIoU < 0 || p_->gpaTresholdIoU > 1) {
        throw std::runtime_error("The parameter 'gpaTresholdIoU' has been set to " + std::to_string(p_->gpaTresholdIoU) +
                                 ". This is not a valid value. The treshold should be a float between 0 and 1.");
    }

    if (p_->gpaTresholdIoU == 0.F) {
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::WARNING>("VERIFY CONFIG",
                                 "The parameter 'gpaTresholdIoU' has been set to " + std::to_string(p_->gpaTresholdIoU) +
                                     ". This means that all patches will be matched.");
    }

    if (p_->gpaTresholdIoU == 1.F) {
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::WARNING>("VERIFY CONFIG",
                                 "The parameter 'gpaTresholdIoU' has been set to " + std::to_string(p_->gpaTresholdIoU) +
                                     ". This means that no patches will be matched.");
    }

    if (p_->sizeGOP2DEncoding != 8 && p_->sizeGOP2DEncoding != 16) {
        throw std::runtime_error("The parameter 'sizeGOP2DEncoding' has been set to " +
                                 std::to_string(static_cast<int>(p_->sizeGOP2DEncoding)) +
                                 " which is not a valid. Currently, this parameter is only link to Kvazaar. This encoder accept "
                                 "only a GOP size of 8 or 16. The GOP size is here link to the size of the inter coding pyramid. Lots of "
                                 "other configurations are possible but they are not yet configurable through the uvgVPCC interface, but within the encoderKvazaar.cpp file directly.");
    }

    if ( (p_->geometryEncodingMode == "RA" || p_->attributeEncodingMode == "RA") && !p_->interPatchPacking) {
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::WARNING>("VERIFY CONFIG",
                                 "You choose to encode the geometry or attribute maps using Random Acess mode. However, you didn't activate "
                                 "the inter patch packing. ('interPatchPacking=false')\n");
    }

    if (p_->occupancyEncodingFormat == "YUV400" || p_->geometryEncodingFormat == "YUV400" || p_->attributeEncodingFormat == "YUV400") {
        throw std::runtime_error("You choose the format 'YUV400' for at least one of the 2D encoder. Currently, this format is not supported. The V3C bitstream and the TMC2 decoder can't handle YUV400 video.");        
    }

    if (p_->occupancyEncodingIsLossless == 0) {
        throw std::runtime_error("The occupancy maps should not be encoded in lossy mode. (At least, this is a very dangerous things to try.)");
    }

    if (roundUp(p_->minimumMapHeight, p_->occupancyMapDSResolution) != p_->minimumMapHeight ||
        roundUp(p_->minimumMapHeight / p_->occupancyMapDSResolution, 8) != p_->minimumMapHeight / p_->occupancyMapDSResolution) {
        throw std::runtime_error(
            "To avoid a padding operation in Kvazaar, all the 2D maps (including the occupancy map) need to have width and height being "
            "multiple of 8.\nThe parameter "
            "minimumMapHeight is set to: " +
            std::to_string(p_->minimumMapHeight) +
            "\nThe parameter occupancyMapDSResolution (OM block size) is set to: " + std::to_string(p_->occupancyMapDSResolution) +
            "\nMap height is multiple of OM block size ? " +
            ((roundUp(p_->minimumMapHeight, p_->occupancyMapDSResolution) == p_->minimumMapHeight) ? "YES" : "NO") +
            "\nOccupancy map height is multiple of 8 ? " +
            ((roundUp(p_->minimumMapHeight / p_->occupancyMapDSResolution, 8) ==
              p_->minimumMapHeight / p_->occupancyMapDSResolution)
                 ? "YES"
                 : "NO") +
            "\n" +
            "\nNearest possible map height value : " +
            std::to_string(
                std::max(roundUp(p_->minimumMapHeight, p_->occupancyMapDSResolution),
                         p_->occupancyMapDSResolution * roundUp(p_->minimumMapHeight / p_->occupancyMapDSResolution, 8))));
    }

    if (roundUp(p_->mapWidth, p_->occupancyMapDSResolution) != p_->mapWidth ||
        roundUp(p_->mapWidth / p_->occupancyMapDSResolution, 8) != p_->mapWidth / p_->occupancyMapDSResolution) {
        throw std::runtime_error(
            "To avoid a padding operation in Kvazaar, all the 2D maps (including the occupancy map) need to have width and height being "
            "multiple of 8.\nThe parameter "
            "mapWidth is set to: " +
            std::to_string(p_->mapWidth) +
            "\nThe parameter occupancyMapDSResolution (OM block size) is set to: " + std::to_string(p_->occupancyMapDSResolution) +
            "\nMap width is multiple of OM block size ? " +
            ((roundUp(p_->mapWidth, p_->occupancyMapDSResolution) == p_->mapWidth) ? "YES" : "NO") +
            "\nOccupancy map width is multiple of 8 ? " +
            ((roundUp(p_->mapWidth / p_->occupancyMapDSResolution, 8) == p_->mapWidth / p_->occupancyMapDSResolution) ? "YES"
                                                                                                                                 : "NO") +
            "\nMap width recommanded value : " +
            std::to_string(std::max(roundUp(p_->mapWidth, p_->occupancyMapDSResolution),
                                    roundUp(p_->mapWidth / p_->occupancyMapDSResolution, 8))));
    }

    if(p_->intraFramePeriod != 64) {
                uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::WARNING>("VERIFY CONFIG",
                                 "It seems that you are modifying the parameter 'intraFramePeriod'. Currently, one Kvazaar instance is spawn for each uvgVPCCenc GOF. Thus, the intraFramePeriod parameter is indirectly constrained and will have no impact if set to a value higher than the GOF size.\n");
    }

    if(p_->intraFramePeriod % p_->sizeGOP2DEncoding != 0) {
        throw std::runtime_error("The intraFramePeriod (" + std::to_string(p_->intraFramePeriod) + ") needs to be a multiple of the GOP length ('sizeGOP2DEncoding'=" + std::to_string(p_->sizeGOP2DEncoding) + "). C.f. Kvazaar configuration.");        
    }

    if (p_->exportIntermediateFiles && p_->intermediateFilesDir.empty()) {
        throw std::runtime_error("Intermediate files need to be exported (exportIntermediateFiles=true) but no intermediate files directory has been set (intermediateFilesDir parameter is empty).");        
    }
}

void setInputGeoPrecision() {
    auto geoBitDepthInputIt = apiInputParameters.find("geoBitDepthInput");
    if(geoBitDepthInputIt == apiInputParameters.end()) {
        throw std::runtime_error("The parameter 'geoBitDepthInput' has to be defined.");
    }
    
    // geoBitDepthInput has been defined by the application
    setParameterValue("geoBitDepthInput",geoBitDepthInputIt->second,false);
    uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::INFO>("API","The geoBitDepthInput is set to '" + std::to_string(p_->geoBitDepthInput) + "'.\n");         
}

void setPreset() {
    auto presetNameIt = apiInputParameters.find("presetName");
    if(presetNameIt != apiInputParameters.end()) { // presetName has been defined by the application
        setParameterValue("presetName",presetNameIt->second,false);
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::INFO>("API","The presetName is set to '" + p_->presetName + "'.\n");         
    } else { // presetName is not defined by the application. Use the default preset name.
        setParameterValue("presetName", "fast",false);
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::INFO>("API","The presetName is set by default to '" + p_->presetName + "'.\n");         
    }
    applyPreset(param);
}

void setRate() {
    auto rateIt = apiInputParameters.find("rate");
    if(rateIt != apiInputParameters.end()) { // rate has been defined by the application
        // Here is the expected format: rate=[geometryQP]-[attributeQP]-[occupancyResolution] c.f. ctc rate config files in TMC2
        std::smatch matches;
        if (std::regex_match(rateIt->second, matches, std::regex(R"((\d+)-(\d+)-(\d+))"))) {
            try {
                setParameterValue("geometryEncodingQp", matches[1],false);
                setParameterValue("attributeEncodingQp", matches[2],false);
                setParameterValue("occupancyMapDSResolution", matches[3],false);
            } catch (const std::invalid_argument& e) {
                uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::FATAL>("LIBRARY", "A problem occured when assigning the values from the 'rate' parameter. Here is the full match: '" + matches[0].str() +"'\n");
                throw;
            }
        } else {
            throw std::invalid_argument("The value assigned to the parameter 'rate' does not have a correct format. Here is the given value: '"+ rateIt->second +"'. The expected format is the following: '[geometryQP]-[attributeQP]-[occupancyResolution]' Here is a correct usage: 'rate=16-22-2'.");
        }
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::INFO>("API","The rate used is '" + rateIt->second + "'.\n");      
    } else { // The rate is not set by the application. Use the default rate which is rate=16-22-2 (R5)
        try {
            setParameterValue("geometryEncodingQp", "16",false);
            setParameterValue("attributeEncodingQp", "22",false);
            setParameterValue("occupancyMapDSResolution", "2" ,false  );     
        } catch (const std::invalid_argument& e) {
            uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::FATAL>("API", "A problem occured when assigning the values from the 'rate' parameter.\n");
            throw;
        }
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::INFO>("API",
        "The rate is not defined in the library command line. The default rate used is '16-22-2'.\n");       
    }
}

void setLogParameters() {
    bool defaultErrorsAreFatalValue;
    auto errorsAreFatalIt = apiInputParameters.find("errorsAreFatal");
    if(errorsAreFatalIt != apiInputParameters.end()) { // errorsAreFatal has been defined by the application
        setParameterValue("errorsAreFatal", errorsAreFatalIt->second,false);
        defaultErrorsAreFatalValue = false;
    } else { // errorsAreFatal default value
        setParameterValue("errorsAreFatal", std::to_string(errorsAreFatalDefaultValue),false);
        defaultErrorsAreFatalValue = true;
    }
    uvgvpcc_enc::Logger::setErrorsAreFatal(p_->errorsAreFatal);

    auto logLevelIt = apiInputParameters.find("logLevel");
    if(logLevelIt != apiInputParameters.end()) { // logLevel has been defined by the application
        setParameterValue("logLevel",logLevelIt->second,false);
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::INFO>("API","The logLevel is set to '" + p_->logLevel + "'.\n");         
    } else { // logLevel default value
        setParameterValue("logLevel",LogLevelStr[static_cast<size_t>(logLevelDefaultValue)],false);
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::INFO>("API","The logLevel is set by default to '" + p_->logLevel + "'.\n");         
    }
    uvgvpcc_enc::Logger::setLogLevel(static_cast<LogLevel>(std::distance(std::begin(LogLevelStr), std::find(std::begin(LogLevelStr), std::end(LogLevelStr), p_->logLevel))));

    if(defaultErrorsAreFatalValue) {
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::INFO>("API","The parameter 'errorsAreFatal' is set by default to '" + std::string(p_->errorsAreFatal ? "True" : "False") + "'.\n");         
    } else {
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::INFO>("API","The parameter 'errorsAreFatal' is set to '" + std::string(p_->errorsAreFatal ? "True" : "False") + "'.\n");         
    }

}


void setMode() {
    std::string modeValue;
    auto modeIt = apiInputParameters.find("mode");
    if(modeIt != apiInputParameters.end()) { // mode has been defined by the application
        modeValue = modeIt->second;
        if(modeValue!="RA" && modeValue!="AI") {
            throw std::invalid_argument("The value assigned to the parameter 'mode' does not have a correct format. Here is the given value: '"+ modeValue +"'. The expected values are: [RA,AI].\n");
        }

        try {
            setParameterValue("occupancyEncodingMode", modeValue,false);
            setParameterValue("geometryEncodingMode", modeValue,false);
            setParameterValue("attributeEncodingMode", modeValue,false);
            setParameterValue("interPatchPacking", std::to_string(modeValue=="RA"),false);
        } catch (const std::invalid_argument& e) {
            uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::FATAL>("LIBRARY", "A problem occured when assigning the values from the 'mode' parameter. Here is the mode value: '" + modeValue +"'\n");
            throw;
        }
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::INFO>("API","The encoding mode used is '" + modeValue + "'.\n");         
    } else {
        // The mode is not defined in the lib command. Use the default mode which is mode=RA
        modeValue = "RA";
        try {
            setParameterValue("occupancyEncodingMode", modeValue,false);
            setParameterValue("geometryEncodingMode", modeValue,false);
            setParameterValue("attributeEncodingMode", modeValue,false);
            setParameterValue("interPatchPacking", std::to_string(modeValue=="RA"),false );       
        } catch (const std::invalid_argument& e) {
            uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::FATAL>("API", "A problem occured when assigning the values from the 'mode' parameter.\n");
            throw;
        }
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::INFO>("API",
        "The mode is not defined in the library command line. The default mode used is 'RA'.\n");        
    }
}



// TODO(lf)check in debug mode for example if rate and occupancyMapDSResolution are both in the lib command line TODO(lf)a warning
void parseUvgvpccParameters() {
    
    // Special parameters need to be handle first
    setLogParameters(); 
    setInputGeoPrecision();
    setPreset();
    setRate();
    setMode();

    // Now that the preset is applied, all other parameters set by the application can overwrite the preset values.
    for (const auto& paramPair : apiInputParameters) {
        if(paramPair.first=="presetName" || paramPair.first=="geoBitDepthInput" || paramPair.first=="rate" || paramPair.first=="logLevel" || paramPair.first=="errorsAreFatal" || paramPair.first=="mode") {
            // Those parameters have been handled at the top of this function
            continue;
        }

        setParameterValue(paramPair.first,paramPair.second,false);
    }

    const std::string detectedThreadNumber = std::to_string(std::thread::hardware_concurrency());
    if(p_->nbThreadPCPart == 0) {
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::INFO>("API","'nbThreadPCPart' is set to 0. The number of thread used for the Point Cloud part of uvgVPCC is then the detected number of threads: "+detectedThreadNumber + "\n");
        setParameterValue("nbThreadPCPart",detectedThreadNumber,false);
    }
    if(p_->maxConcurrentFrames == 0) {
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::INFO>("API","'maxConcurrentFrames' is set to 0. The maximum number of frame processed in parallel by uvgVPCC is then the four times GOF size: "+ std::to_string(4*p_->sizeGOF) + "\n");
        setParameterValue("maxConcurrentFrames",std::to_string(4*p_->sizeGOF),false);
    }
    if(p_->occupancyEncodingNbThread == 0) {
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::DEBUG>("API","'occupancyEncodingNbThread' is set to 0. The number of thread used for the occcupancy video 2D encoding is then the detected number of threads: "+detectedThreadNumber + "\n");
        setParameterValue("occupancyEncodingNbThread",detectedThreadNumber,false);
    }
    if(p_->geometryEncodingNbThread == 0) {
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::DEBUG>("API","'geometryEncodingNbThread' is set to 0. The number of thread used for the geometry video 2D encoding is then the detected number of threads: "+detectedThreadNumber + "\n");
        setParameterValue("geometryEncodingNbThread",detectedThreadNumber,false);
    }
    if(p_->attributeEncodingNbThread == 0) {
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::DEBUG>("API","'attributeEncodingNbThread' is set to 0. The number of thread used for the attribute video 2D encoding is then the detected number of threads: "+detectedThreadNumber + "\n");
        setParameterValue("attributeEncodingNbThread",detectedThreadNumber,false);
    }

    if(p_->exportIntermediateFiles && p_->intermediateFilesDirTimeStamp) {
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::DEBUG>("API","'intermediateFilesDirTimeStamp' is true, so a time stamp is added to the 'intermediateFilesDir' path.\n");
        
        std::time_t now = std::time(nullptr);
        std::tm* localTime = std::localtime(&now);

        std::ostringstream oss;
        oss << std::setfill('0') << std::setw(2) << (localTime->tm_year % 100)
            << std::setw(2) << (localTime->tm_mon + 1)
            << std::setw(2) << localTime->tm_mday
            << std::setw(2) << localTime->tm_hour
            << std::setw(2) << localTime->tm_min
            << std::setw(2) << localTime->tm_sec;

        std::string dir = p_->intermediateFilesDir;
        if (!dir.empty() && dir.back() == '/') {
            dir.pop_back(); // Remove trailing slash
        }

        setParameterValue("intermediateFilesDir",dir + oss.str(),false);        
    }


}

static void initializeContext() {
    uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::TRACE>("API", "Initialize context.\n");
    g_threadHandler.queue = std::make_shared<ThreadQueue>(p_->nbThreadPCPart);
    g_threadHandler.gofId = 0;
}

} // anonymous namespace

const Parameters* p_ = &param;


void Frame::printInfo() const {
    Logger::log<LogLevel::DEBUG>("FRAME-INFO",
                "Frame " + std::to_string(frameId) + " :\n" + "\tPath: " + pointCloudPath + "\n" + "\tFrame Number: " +
                    std::to_string(frameNumber) + "\n" + "\tpointsGeometry size: " + std::to_string(pointsGeometry.size()) + "\n" +
                    "\tpointsAttribute size: " + std::to_string(pointsAttribute.size()) + "\n" +
                    "\tpatchList size: " + std::to_string(patchList.size()) + "\n" + "\toccupancyMapDS size: " + std::to_string(occupancyMapDS.size()) + "\n" +
                    "\tgeometryMapL1 size: " + std::to_string(geometryMapL1.size()) + "\n" + "\tgeometryMapL2 size: " +
                    std::to_string(geometryMapL2.size()) + "\n" + "\tattributeMapL1 size: " + std::to_string(attributeMapL1.size()) + "\n" +
                    "\tattributeMapL2 size: " + std::to_string(attributeMapL2.size()) + "\n");
}




/// @brief Create the context of the uvgVPCCenc encoder. Parse the input parameters and verify if the given configuration is valid. Initialize static parameters and function pointers.
void API::initializeEncoder() {
    uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::TRACE>("API", "Initialize the encoder.\n");
    uvgvpcc_enc::initializeParameterMap(param);
    parseUvgvpccParameters();
    if(errorInAPI && p_->errorsAreFatal) throw std::runtime_error("An error occured while handling application input parameters. If you want to not stop the execution of the program when an error is detected, set the parameter 'errorsAreFatal' to 'False'.");
    verifyConfig();
    initializeStaticParameters();
    initializeStaticFunctionPointers();
    initializeContext();
    if(p_->exportIntermediateFiles && !p_->intermediateFilesDirTimeStamp) FileExport::cleanIntermediateFiles();
    initializationDone = true;
}

/// @brief The only way to modify the exposed uvgVPCCenc parameters is by calling this function.
/// @param parameterName Name of the parameter. All exposed parameters are listed in the object parameterMap defined in lib/utils/parameters.cpp
/// @param parameterValue The value of the parameter written as a string.
void API::setParameter(const std::string& parameterName,const std::string& parameterValue) {
    if(initializationDone) {
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::FATAL>("API","The API function 'setParameter' can't be called after the API function 'initializeEncoder'.\n");
        throw std::runtime_error("");
    }
    if(apiInputParameters.find(parameterName) != apiInputParameters.end()) {
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::ERROR>("API","The parameter '" + parameterName +  "' has already been set. The value used is: '" + apiInputParameters.at(parameterName) +"'.\n");
        errorInAPI = true;
    }
    apiInputParameters.emplace(parameterName,parameterValue);
}

/// @brief Entry point of the uvgVPCCenc library. Take as input a frame. Create all the jobs for processing this frame. This function also handles the GOF processing.
/// @param frame uvgvpcc_enc::Frame
/// @param output GOF bitstream
void API::encodeFrame(std::shared_ptr<Frame>& frame, v3c_unit_stream* output) {
    static std::shared_ptr<std::counting_semaphore<UINT16_MAX>> conccurentFrameSem = std::make_shared<std::counting_semaphore<UINT16_MAX>>(std::min(p_->maxConcurrentFrames, size_t(UINT16_MAX)));

    conccurentFrameSem->acquire();
    frame->conccurentFrameSem = conccurentFrameSem;

    Logger::log<LogLevel::TRACE>("API", "Encoding frame " + std::to_string(frame->frameId) + "\n");
    if (frame == nullptr) {
        Logger::log<LogLevel::ERROR>("API", "The frame is null.\n");
        if(p_->errorsAreFatal) throw std::runtime_error("");    
    }

    if (frame->frameId % p_->sizeGOF == 0) {
        // The current frame is the first of its GOF. Create all GOF related jobs.
        g_threadHandler.currentGOF = std::make_shared<GOF>();
        g_threadHandler.currentGOF->gofId = g_threadHandler.gofId++;
        g_threadHandler.currentGOF->nbFrames = 0;
        g_threadHandler.currentGOF->mapHeightGOF = p_->minimumMapHeight;
        g_threadHandler.currentGOF->mapHeightDSGOF = p_->minimumMapHeight / p_->occupancyMapDSResolution;
        if (p_->interPatchPacking) {
            g_threadHandler.currentGOFInterPackJob =
                std::make_shared<Job>("GOF " + std::to_string(g_threadHandler.currentGOF->gofId) + " PatchPacking::gofPatchPacking", 3,
                                      PatchPacking::gofPatchPacking, g_threadHandler.currentGOF);
            // TODO(lf): add a new priority level ?
        }
        g_threadHandler.currentGOFInitMapGenJob =
            std::make_shared<Job>("GOF " + std::to_string(g_threadHandler.currentGOF->gofId) + " MapGeneration::initGOFMapGeneration", 3,
                                  MapGenerationBaseLine::initGOFMapGeneration, g_threadHandler.currentGOF);
        g_threadHandler.currentGOF2DEncodingJob =
            std::make_shared<Job>("GOF " + std::to_string(g_threadHandler.currentGOF->gofId) + " MapEncoding::encodeGOFMaps", 5,
                                  MapEncoding::encodeGOFMaps, g_threadHandler.currentGOF);
        g_threadHandler.previousGOFBitstreamGenJob = g_threadHandler.currentGOFBitstreamGenJob;
        g_threadHandler.currentGOFBitstreamGenJob = std::make_shared<Job>(
            "GOF " + std::to_string(g_threadHandler.currentGOF->gofId) + " BitstreamGeneration::createV3CGOFBitstream", 5,
            BitstreamGeneration::createV3CGOFBitstream, g_threadHandler.currentGOF, *(p_), output);

        if (p_->interPatchPacking) {
            g_threadHandler.currentGOFInitMapGenJob->addDependency(g_threadHandler.currentGOFInterPackJob);
        }
        g_threadHandler.currentGOF2DEncodingJob->addDependency(g_threadHandler.currentGOFInitMapGenJob);
        g_threadHandler.currentGOFBitstreamGenJob->addDependency(g_threadHandler.currentGOF2DEncodingJob);

        if (g_threadHandler.previousGOFBitstreamGenJob != nullptr) {
            g_threadHandler.currentGOFBitstreamGenJob->addDependency(g_threadHandler.previousGOFBitstreamGenJob);
        }
    }

    g_threadHandler.currentGOF->frames.push_back(frame);
    g_threadHandler.currentGOF->nbFrames++;
    frame->gof = g_threadHandler.currentGOF;
    auto patchGen = std::make_shared<Job>("Frame " + std::to_string(frame->frameId) + " PatchGeneration::generateFramePatches", 0,
                                          PatchGeneration::generateFramePatches, frame);

    if (p_->interPatchPacking) {
        g_threadHandler.currentGOFInterPackJob->addDependency(patchGen);
    } else {
        auto occAlloc = std::make_shared<Job>("Frame " + std::to_string(frame->frameId) + " PatchPacking::allocateDefaultOccupancyMap", 1,
                                              PatchPacking::allocateDefaultOccupancyMap, frame, p_->minimumMapHeight);
        auto patchPack = std::make_shared<Job>("Frame " + std::to_string(frame->frameId) + " PatchPacking::framePatchPacking", 1,
                                               PatchPacking::frameIntraPatchPacking, frame, nullptr);

        occAlloc->addDependency(patchGen);
        patchPack->addDependency(occAlloc);
        g_threadHandler.currentGOFInitMapGenJob->addDependency(patchPack);
        g_threadHandler.queue->submitJob(occAlloc);
        g_threadHandler.queue->submitJob(patchPack);
    }

    auto mapGen = std::make_shared<Job>("Frame " + std::to_string(frame->frameId) + " MapGeneration::generateFrameMaps", 4,
                                    MapGenerationBaseLine::generateFrameMaps, frame);

    mapGen->addDependency(g_threadHandler.currentGOFInitMapGenJob);
    g_threadHandler.currentGOF2DEncodingJob->addDependency(mapGen);
    g_threadHandler.queue->submitJob(patchGen);
    g_threadHandler.queue->submitJob(mapGen);

    if (g_threadHandler.currentGOF->nbFrames == p_->sizeGOF) {
        if (p_->interPatchPacking) {
            g_threadHandler.queue->submitJob(g_threadHandler.currentGOFInterPackJob);
        }
        g_threadHandler.queue->submitJob(g_threadHandler.currentGOFInitMapGenJob);
        g_threadHandler.queue->submitJob(g_threadHandler.currentGOF2DEncodingJob);
        g_threadHandler.queue->submitJob(g_threadHandler.currentGOFBitstreamGenJob);
    }
}

/// @brief This function is called when all frames to be processed have been sent to the encoder. Wait for all remaining jobs to be executed.
void API::emptyFrameQueue() {
    if (g_threadHandler.currentGOF != nullptr) {
        if (g_threadHandler.currentGOF->nbFrames < p_->sizeGOF) {
            if (p_->interPatchPacking) {
                g_threadHandler.queue->submitJob(g_threadHandler.currentGOFInterPackJob);
            }
            g_threadHandler.queue->submitJob(g_threadHandler.currentGOFInitMapGenJob);
            g_threadHandler.queue->submitJob(g_threadHandler.currentGOF2DEncodingJob);
            g_threadHandler.queue->submitJob(g_threadHandler.currentGOFBitstreamGenJob);
        }
        g_threadHandler.queue->waitForJob(g_threadHandler.currentGOFBitstreamGenJob);
    }
}

/// @brief Insure a proper end of the encoder execution.
void API::stopEncoder() { g_threadHandler.queue->stop(); }

}  // namespace uvgvpcc_enc