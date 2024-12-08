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

#pragma once

#include <stdexcept>
#include <string>
#include <vector>

#include "utils.hpp"

namespace uvgvpcc_enc {


// Configuration of the encoder, only const variable //
// Value here are default values non defined in presets. It is strongly NON-recommended to modify the values here. Use 'setParameter' or 'setParameters' function in the application instead.
struct Parameters {
    
    // ___ General parameters __ //
    std::size_t geoBitDepthInput;      // (inputGeometryBitDepth3D?) geometry3dCoordinatesBitdepth  // to do rename in resolution or something more catchy ?
    std::string presetName;
    std::size_t sizeGOF;
    std::size_t nbThreadPCPart = 0; // 0 means the actual number of detected threads
    bool doubleLayer = true;
    std::string logLevel = "INFO";
    bool errorsAreFatal = true;
    std::string intermediateFilesDir;

    // ___ Debug parameters ___ //
    bool exportIntermediateMaps = false;
    bool exportIntermediatePointClouds = false;

    // ___ Activate or not some features ___ //
    bool lowDelayBitstream = false;

    // ___ Voxelization ___ //       (grid-based segmentation)
    std::size_t geoBitDepthVoxelized;  // voxelizedGeometryBitDepth3D         // grid-based segmentation

    // ___ KdTree ___ //
    std::size_t kdTreeMaxLeafSize = 10; // to do deprecated no ? (as there are other parameterrs for it, for each kdtree case)

    // Normal computation //
    std::size_t normalComputationKnnCount;
    std::size_t normalComputationMaxDiagonalStep;


    // Normal orientation //
    std::size_t normalOrientationKnnCount = 4;


    // PPI segmentation //
    // const std::vector<std::array<double,3>> projectionPlaneOrientations = {
    const std::vector<Vector3<double>> projectionPlaneOrientations = {
        {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}, {-1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, -1.0}}};
    const std::size_t projectionPlaneCount = 6; // to do : move out from parameters ?


    // ___ PPI smoothing  ___  //    (fast grid-based refine segmentation)
    std::size_t geoBitDepthRefineSegmentation;  // refineSegmentationGeometryBitDepth3D   
    // to do verify that all scale set by user are compatible
    std::size_t refineSegmentationMaxNNVoxelDistanceLUT;  // lf note : 9**2 = 81 ~ 192/2
    std::size_t refineSegmentationMaxNNTotalPointCount;
    double refineSegmentationLambda;
    std::size_t refineSegmentationIterationCount;
    // to do check the config if all concerned parameters are poqwer of two


    // ___ Patch generation ___ //   (patch segmentation)
    std::size_t maxAllowedDist2RawPointsDetection = 5; // to do : add verification to avoid segfault because index out of bound
    std::size_t minPointCountPerCC;
    std::size_t maxPatchSize;               // to do : debug when maxPatchSize = 200
    std::size_t maxNNCountPatchSegmentation = 5;  // to do check max all NN
    std::size_t patchSegmentationMaxPropagationDistance = 3;  
        // lf : for reworked function only. If the value is 4, the euclidian distance is 16.  // to do: the default value should be 2 I
            // guess // Nop, it should be 1 // to do : make sure to use <= instead of < in the for loop so to avoid this confusion.
    bool enablePatchSplitting = true;
    // to do there is a mix with occupancyPrecision // to do rename it as blocSizeOccupancyMap or something like this
    std::size_t minLevel = 64;  // to do : must be a power of 2 ? So give the power of two and do only shifting, no division // might be
                                // related to the different avaliable position of the projection plan of each path
    std::size_t log2QuantizerSizeX = 4;
    std::size_t log2QuantizerSizeY = 4;
    std::size_t quantizerSizeX = static_cast<std::size_t>(1) << log2QuantizerSizeX; // to do : investigate
    std::size_t quantizerSizeY = static_cast<std::size_t>(1) << log2QuantizerSizeY;
    std::size_t surfaceThickness = 4;


    // ___ Patch packing ___ //
    std::size_t mapWidth;  // to do check if it is a multipl of occupancy resolution
    std::size_t minimumMapHeight;
    // to do : lf : As Joose explain to me, in theory, it would be the height which is constant and equals to 64*nbThreads
    std::size_t spacePatchPacking = 1;
    bool interPatchPacking;
    float gpaTresholdIoU = 0.3;  // global patch allocation treshold for the intersection over union process


    // ___ Map generation ___ //
    bool mapGenerationFillEmptyBlock = true;
    size_t mapGenerationBackgroundValueAttribute = 128;
    size_t mapGenerationBackgroundValueGeometry = 128;
    std::size_t cuSizeInPixels = 8;       // only for refacctored map gen to do : 8 is better than 16 for the quality, but what about computational time ?


    // ___ 2D encoding parameters ___ //
    std::string basenameOccupancyFiles;
    std::string basenameGeometryFiles;
    std::string basenameAttributeFiles;
    std::size_t sizeGOP2DEncoding;
    std::size_t intraFramePeriod = 64; // to do : Not useful yet as a new 2D encoder is created for each GOF. (64 is default Kvazaar value. In uvgVPCCenc, the value is indirectly set by 8 or 16, depending on the size of the 2D encoding GOP)

    // Occupancy map
    std::string occupancyEncoderName = "Kvazaar";
    bool occupancyEncodingIsLossless = true;
    std::string occupancyEncodingMode;
    std::string occupancyEncodingFormat = "YUV420";
    std::size_t occupancyEncodingNbThread = 0; // 0 by default means that this variable will have for value during execution the actual number of detected threads
    std::size_t occupancyMapResolution; // 'Rate' or 'qp' for the occupancy map
    std::string occupancyEncodingPreset;

    // Geometry map
    std::string geometryEncoderName = "Kvazaar";
    bool geometryEncodingIsLossless = false;
    std::string geometryEncodingMode;
    std::string geometryEncodingFormat = "YUV420";
    std::size_t geometryEncodingNbThread = 0; // 0 by default means that this variable will have for value during execution the actual number of detected threads
    std::size_t geometryEncodingQp;
    std::string geometryEncodingPreset;

    // Attribute map
    std::string attributeEncoderName = "Kvazaar";
    bool attributeEncodingIsLossless = false;
    std::string attributeEncodingMode;
    std::string attributeEncodingFormat = "YUV420";
    std::size_t attributeEncodingNbThread = 0;  // 0 by default means that this variable will have for value during execution the actual number of detected threads        
    std::size_t attributeEncodingQp;
    std::string attributeEncodingPreset;


    // ___ Miscellaneous ___ //
    // bool useEncoderCommand = false; // lf : All mention of this parameter has been commented. This might be usefull to support command line 2D encoder in the futur.
};


enum ParameterType {BOOL,INT,UINT,STRING,FLOAT,DOUBLE};

struct ParameterInfo{
    ParameterType type;
    std::string possibleValues;
    void* parameterPtr;
    bool inPreset = false;

    ParameterInfo(const ParameterType& type, const std::string& possibleValues,bool* parameterPtr):type(type),possibleValues(possibleValues),parameterPtr((void*)parameterPtr) {
        if(type != BOOL) {
            throw std::runtime_error("During the initialization of the library parameter maps, a type mismatch has been found. Apparently, the given parameterType is: '" + std::to_string(type) + "' while the type of the parameter variable is BOOL (0). The corresponding variable name is not known, but here are its possible values :'" + possibleValues +"'. If you recently added a new parameter in the parameter map, the given type is probably wrong.");
        }
    }
    ParameterInfo(const ParameterType& type, const std::string& possibleValues,int* parameterPtr):type(type),possibleValues(possibleValues),parameterPtr((void*)parameterPtr) {
        if(type != INT) {
            throw std::runtime_error("During the initialization of the library parameter maps, a type mismatch has been found. Apparently, the given parameterType is: '" + std::to_string(type) + "' while the type of the parameter variable is INT (1). The corresponding variable name is not known, but here are its possible values :'" + possibleValues +"'. If you recently added a new parameter in the parameter map, the given type is probably wrong.");
        }
    }
    ParameterInfo(const ParameterType& type, const std::string& possibleValues,size_t* parameterPtr):type(type),possibleValues(possibleValues),parameterPtr((void*)parameterPtr) {
        if(type != UINT) {
            throw std::runtime_error("During the initialization of the library parameter maps, a type mismatch has been found. Apparently, the given parameterType is: '" + std::to_string(type) + "' while the type of the parameter variable is UINT (2). The corresponding variable name is not known, but here are its possible values :'" + possibleValues +"'. If you recently added a new parameter in the parameter map, the given type is probably wrong.");
        }
    }
    ParameterInfo(const ParameterType& type, const std::string& possibleValues,std::string* parameterPtr):type(type),possibleValues(possibleValues),parameterPtr((void*)parameterPtr) {
        if(type != STRING) {
            throw std::runtime_error("During the initialization of the library parameter maps, a type mismatch has been found. Apparently, the given parameterType is: '" + std::to_string(type) + "' while the type of the parameter variable is STRING (3). The corresponding variable name is not known, but here are its possible values :'" + possibleValues +"'. If you recently added a new parameter in the parameter map, the given type is probably wrong.");
        }
    }
    ParameterInfo(const ParameterType& type, const std::string& possibleValues,float* parameterPtr):type(type),possibleValues(possibleValues),parameterPtr((void*)parameterPtr) {
        if(type != FLOAT) {
            throw std::runtime_error("During the initialization of the library parameter maps, a type mismatch has been found. Apparently, the given parameterType is: '" + std::to_string(type) + "' while the type of the parameter variable is FLOAT (4). The corresponding variable name is not known, but here are its possible values :'" + possibleValues +"'. If you recently added a new parameter in the parameter map, the given type is probably wrong.");
        }
    }
    ParameterInfo(const ParameterType& type, const std::string& possibleValues,double* parameterPtr):type(type),possibleValues(possibleValues),parameterPtr((void*)parameterPtr) {
        if(type != DOUBLE) {
            throw std::runtime_error("During the initialization of the library parameter maps, a type mismatch has been found. Apparently, the given parameterType is: '" + std::to_string(type) + "' while the type of the parameter variable is DOUBLE (5). The corresponding variable name is not known, but here are its possible values :'" + possibleValues +"'. If you recently added a new parameter in the parameter map, the given type is probably wrong.");
        }
    }
            
};


void initializeParameterMap(Parameters& param);
void setParameterValue(const std::string& parameterName,const std::string& parameterValue, const bool& fromPreset);
  


} // uvgvpcc_enc namespace